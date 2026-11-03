#!/usr/bin/env python3
"""Measure the drive ESP32's CPU load, loop time, heap and stack. Needs the PROFILING firmware. WHEELS OFF THE GROUND.

    cd firmware && pio run -e profile -t upload                   # 1. flash the profiling build
    python3 tools/profile.py                                        # 2. 120 s of worst-case load, then a summary
    python3 tools/profile.py --load idle                            #    neutral commands only, for comparison
    cd firmware && pio run -e esp32doit-devkit-v1 -t upload        # 3. flash the normal firmware back (fw 2.0.2)

Stop drive_node first: only one program may use the PC's UDP port. Needs only the Python standard library.

The "worst" load sends what drive_node sends (DRIVE_COMMAND at 50 Hz) but with wheel targets that change on every
command, so the ramp works on all four wheels on every control tick; plus a burst of 3 back-to-back commands every 2 s
(queued packets arriving together) and a few garbage bytes every 5 s (the decoder's error path). The ESP32 keeps sending
its status at 10 Hz. CPU load comes from FreeRTOS's idle-task run time per 100 ms window (firmware/src/profiler.cpp),
so it counts everything the chip does, not just the drive code. The profiler's own time is reported separately: the
normal firmware does not have it, so it uses a little less than measured.
"""
import argparse
import math
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'rover_pilot_remastered'))
from rover_pilot_remastered import protocol  # noqa: E402
from rover_pilot_remastered.udp_link import UdpLink  # noqa: E402

TYPE_PROFILE = 0x05  # firmware/src/profiler.h kTypeProfile
_PROFILE = struct.Struct('<B8HIIHHIIIHHI')  # layout in firmware/src/profiler.cpp build_report()
ZERO = (0.0, 0.0, 0.0, 0.0)


def parse_profile(payload: bytes) -> dict | None:
    if len(payload) != _PROFILE.size or payload[0] != 1:
        return None
    (_, window_ms, windows, c0_avg, c0_peak, c1_avg, c1_peak, c0_ever, c1_ever, passes, busy_total_us, busy_max_us,
     busy_max_ever_us, own_us, heap, heap_min, stack_free, stack_size, uptime_ms) = _PROFILE.unpack(payload)
    return {'window_ms': window_ms, 'windows': windows, 'core0_avg': c0_avg / 100, 'core0_peak': c0_peak / 100,
            'core1_avg': c1_avg / 100, 'core1_peak': c1_peak / 100, 'core0_peak_ever': c0_ever / 100,
            'core1_peak_ever': c1_ever / 100, 'passes': passes, 'busy_total_us': busy_total_us,
            'busy_max_us': busy_max_us, 'busy_max_ever_us': busy_max_ever_us, 'own_us': own_us, 'heap': heap,
            'heap_min': heap_min, 'stack_free': stack_free, 'stack_size': stack_size, 'uptime_ms': uptime_ms}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--master', default='192.168.77.2', help="the ESP32's address (default: LINK.md's)")
    parser.add_argument('--port', type=int, default=47700, help="the ESP32's UDP port")
    parser.add_argument('--local-port', type=int, default=47701, help="this PC's UDP port (the ESP32 sends status to it)")
    parser.add_argument('--interface', default='', help='the PC port the ESP32 is plugged into: send only through it (Linux)')
    parser.add_argument('--seconds', type=float, default=120.0, help='length of the measurement')
    parser.add_argument('--load', choices=('worst', 'idle'), default='worst', help='worst: changing targets, bursts, '
                        'garbage bytes; idle: neutral commands only')
    parser.add_argument('--percent', type=float, default=30.0, help='wheel speed swing for --load worst, %% of full speed')
    args = parser.parse_args()

    link = UdpLink(args.master, args.port, args.local_port, args.interface)
    try:
        link.open()
    except OSError as error:
        print(f'cannot open UDP port {args.local_port}: {error} (stop drive_node or the CAN bridge first: only one may use it)')
        return 1
    seq = 0
    status, reports = None, []

    def send(estop: bool, targets: tuple) -> None:
        nonlocal seq
        link.send(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, seq, protocol.drive_command_payload(estop, targets)))
        seq = (seq + 1) & 0xFF

    def receive(show: bool) -> None:
        nonlocal status
        for packet in link.receive():
            if packet.type == protocol.TYPE_DRIVE_STATUS:
                status = protocol.parse_drive_status(packet.payload) or status
            elif packet.type == TYPE_PROFILE and (report := parse_profile(packet.payload)) is not None:
                reports.append(report)
                if show:
                    print_report(report)

    def print_report(r: dict) -> None:
        passes = max(1, r['passes'])
        print(f"{r['uptime_ms'] / 1000:8.1f} s  core1 {r['core1_avg']:5.2f} % (peak {r['core1_peak']:5.2f})  "
              f"core0 {r['core0_avg']:5.2f} % (peak {r['core0_peak']:5.2f})  {r['passes']:5d} passes, "
              f"avg {r['busy_total_us'] / passes:6.1f} us, longest {r['busy_max_us']:5d} us  "
              f"heap {r['heap']:6d} B  profiler {r['own_us'] / 1e4:.2f} %"
              f"{'' if status is None else f'  {status.state_name}, bad packets {status.rx_bad}'}", flush=True)

    try:
        # Send neutral until it drives (as wheel_test does): its Ethernet link may still be coming up after a reset.
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not (status and status.state_name == 'DRIVING' and reports):
            send(False, ZERO)
            receive(show=False)
            time.sleep(0.02)
        if not reports:
            print('No PROFILE reports: this is not the profiling firmware (cd firmware && pio run -e profile -t upload), '
                  f'or nothing answers at {args.master}:{args.port} (cable, the PC port\'s address: LINK.md).')
            return 1
        if not (status and status.state_name == 'DRIVING'):
            print(f'The ESP32 did not reach DRIVING (state {status.state_name if status else "unknown"}).')
            return 1
        print(f"Profiling firmware {'.'.join(map(str, status.firmware_version))}, load '{args.load}' for {args.seconds:g} s. "
              'WHEELS OFF THE GROUND.\n', flush=True)
        reports.clear()  # measure only the load phase

        rpm = status.max_wheel_rpm * max(0.0, min(100.0, args.percent)) / 100.0
        start = next_command = time.monotonic()
        next_burst, next_garbage = start + 2.0, start + 5.0
        while (now := time.monotonic()) < start + args.seconds:
            if now >= next_command:
                next_command += 0.02
                if args.load == 'worst':
                    t = now - start
                    send(False, tuple(rpm * math.sin(2 * math.pi * 0.5 * t + phase) for phase in (0, 1.6, 3.1, 4.7)))
                    if now >= next_burst:
                        next_burst += 2.0
                        for _ in range(2):  # with the command above: 3 back to back
                            send(False, tuple(rpm * math.sin(2 * math.pi * 0.5 * t + phase) for phase in (0, 1.6, 3.1, 4.7)))
                    if now >= next_garbage:
                        next_garbage += 5.0
                        link.send(b'\x55' * 20 + b'\x00')
                else:
                    send(False, ZERO)
            receive(show=True)
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            send(True, ZERO)
        link.close()

    reports = reports[1:]  # the first one's second can begin before the load did
    if not reports:
        print('\nNot enough reports during the measurement (each covers a second; run for at least 3 s).')
        return 1
    last = reports[-1]
    core1_peak = max(r['core1_peak'] for r in reports)
    core1_avg = sum(r['core1_avg'] for r in reports) / len(reports)
    core0_peak = max(r['core0_peak'] for r in reports)
    core0_avg = sum(r['core0_avg'] for r in reports) / len(reports)
    passes = sum(r['passes'] for r in reports)
    busy = sum(r['busy_total_us'] for r in reports)
    longest = max(r['busy_max_us'] for r in reports)
    own = sum(r['own_us'] for r in reports) / (len(reports) * 1e6) * 100
    stack_used = last['stack_size'] - last['stack_free']
    print(f"""
Summary over {len(reports)} s of '{args.load}' load ({last['window_ms']} ms windows):
  CPU core 1 (runs this firmware)   average {core1_avg:5.2f} %   peak window {core1_peak:5.2f} %
    of which the profiler itself    about {own:.2f} % (the normal firmware does not have it)
  CPU core 0                        average {core0_avg:5.2f} %   peak window {core0_peak:5.2f} %
  loop passes                       {passes / len(reports):.0f} per second, average {busy / max(1, passes):.1f} us busy,
                                    longest {longest} us (since boot {last['busy_max_ever_us']} us; the loop period is 1000 us)
  heap                              {last['heap']} B free now, lowest ever {last['heap_min']} B
  loop task stack                   {stack_used} of {last['stack_size']} B ever used ({stack_used / last['stack_size'] * 100:.0f} %)
Flash the normal firmware back: cd firmware && pio run -e esp32doit-devkit-v1 -t upload""")
    return 0


if __name__ == '__main__':
    sys.exit(main())
