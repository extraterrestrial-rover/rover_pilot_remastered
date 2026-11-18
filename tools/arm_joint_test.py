#!/usr/bin/env python3
"""Motor identification test for the arm master and its four slaves, without ROS. KEEP CLEAR OF THE ARM.

Turns each motor one way and back on its own, then the wrist's two bevel motors together (wrist pitch) and against
each other (wrist roll), printing what the master reports: state, PWM, speed and encoder count per motor. Each step
returns to where it started, so the arm ends roughly where it began. Use it after flashing to check that every motor
is the one you think, that positive is the direction config.h says, and that its encoder counts the same way.
Needs only pyserial (pip install pyserial). Stop the arm driver first: only one program may use the port.

    python3 tools/arm_joint_test.py /dev/arm-master             # default 20 % of full speed, 1 s per step
    python3 tools/arm_joint_test.py /dev/ttyUSB0 --percent 30 --seconds 0.5
"""
import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'arm_pilot_redemption'))
from arm_pilot_redemption import protocol  # noqa: E402

ZERO = (0.0, 0.0, 0.0, 0.0)


def problems(status: protocol.ArmStatus) -> str:
    parts = []
    for name, motor in zip(protocol.MOTORS, status.motors):
        reasons = [protocol.describe_faults(motor.link_faults, protocol.LINK_FAULT_NAMES)] if motor.link_faults else []
        if motor.slave_faults:
            reasons.append(protocol.describe_faults(motor.slave_faults, protocol.SLAVE_FAULT_NAMES))
        if reasons:
            parts.append(f'{name}: {", ".join(reasons)}')
    return '; '.join(parts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--percent', type=float, default=20.0, help='speed as a percentage of each motor\'s full speed')
    parser.add_argument('--seconds', type=float, default=1.0, help='duration of each move')
    args = parser.parse_args()

    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, args.baud, 0
    port.dtr = port.rts = False
    port.open()
    decoder, seq = protocol.PacketDecoder(), 0
    status: protocol.ArmStatus | None = None
    info: tuple[protocol.MotorInfo, ...] | None = None

    def run(label: str, estop: bool, targets: tuple, seconds: float, until=None) -> bool:
        """Send `targets` at 50 Hz for `seconds`, or until `until(status)` holds (then returns True)."""
        nonlocal seq, status, info
        print(f'\n>>> {label}')
        end, next_print = time.monotonic() + seconds, time.monotonic() + 0.3
        while time.monotonic() < end:
            port.write(protocol.encode_packet(protocol.TYPE_ARM_COMMAND, seq, protocol.arm_command_payload(estop, targets)))
            seq = (seq + 1) & 0xFF
            for packet in decoder.feed(port.read(port.in_waiting or 0)):
                if packet.type == protocol.TYPE_ARM_STATUS:
                    status = protocol.parse_arm_status(packet.payload) or status
                elif packet.type == protocol.TYPE_ARM_INFO:
                    info = protocol.parse_arm_info(packet.payload) or info
            if status and time.monotonic() >= next_print:
                next_print = time.monotonic() + 0.5
                motors = '  '.join(f'{name} {m.pwm:4} {m.velocity_rpm:6.1f}rpm {m.position:7}'
                                   for name, m in zip(protocol.MOTORS, status.motors))
                print(f'    {status.state_name:<12} PWM / speed / count:  {motors}   '
                      f'(fw {".".join(map(str, status.firmware_version))}, bad packets {status.rx_bad})')
                if problems(status):
                    print(f'    !! {problems(status)}')
            if until is not None and status is not None and until(status):
                return True
            time.sleep(0.02)
        return until is None

    try:
        # Opening the port can reset the master; it ignores commands while it boots and waits for its slaves. Send
        # neutral until it is enabled.
        ready = run('neutral until the master and all four slaves are ready', False, ZERO, 5.0,
                    until=lambda s: s.state_name == 'ENABLED' and s.slaves_ready)
        ready = ready and run('waiting for the slaves\' calibration', False, ZERO, 2.0,
                              until=lambda s: info is not None and all(motor.known for motor in info))
        if not ready:
            detail = f'{status.state_name}: {problems(status) or "no slave problem reported"}' if status else 'no status at all'
            print(f'\nThe arm was not ready within 7 s ({detail}). Check the port, that the master and all four slaves are '
                  'flashed (fw versions), powered and on the I2C bus, and that no other program has the port.')
            return 1
        fraction = max(0.0, min(100.0, args.percent)) / 100.0
        rpm = [motor.max_rpm * fraction for motor in info]  # each slave's own calibration
        steps = []
        for index, name in enumerate(protocol.MOTORS):
            forward = [0.0] * 4
            forward[index] = rpm[index]
            backward = [-value for value in forward]
            steps += [(f'{name} positive only', False, tuple(forward), args.seconds), ('stop', False, ZERO, 0.5),
                      (f'{name} back', False, tuple(backward), args.seconds), ('stop', False, ZERO, 0.5)]
        wrist = min(rpm[2], rpm[3])
        steps += [('wrist pitch up (wrist_a and wrist_b positive)', False, (0.0, 0.0, wrist, wrist), args.seconds),
                  ('stop', False, ZERO, 0.5), ('wrist pitch back down', False, (0.0, 0.0, -wrist, -wrist), args.seconds),
                  ('stop', False, ZERO, 0.5),
                  ('wrist roll positive / auger drilling way (wrist_a positive, wrist_b negative)', False,
                   (0.0, 0.0, wrist, -wrist), args.seconds),
                  ('stop', False, ZERO, 0.5), ('wrist roll back', False, (0.0, 0.0, -wrist, wrist), args.seconds),
                  ('e-stop', True, ZERO, 1.0)]
        for step in steps:
            run(*step)
            if status is not None and status.state_name == 'FAULT':
                print(f'\nThe master went to FAULT: {problems(status) or "see the slave serial output"}. Stopping.')
                return 1
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            port.write(protocol.encode_packet(protocol.TYPE_ARM_COMMAND, seq, protocol.arm_command_payload(True, ZERO)))
            seq = (seq + 1) & 0xFF
        port.close()
    print('\nDone. If a motor turned the wrong way, fix forward_level in firmware_arm/include/config.h; if its count went '
          'the other way, fix encoder_reversed; if the wrong motor moved, check which slave environment each board got.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
