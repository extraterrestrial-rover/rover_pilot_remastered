#!/usr/bin/env python3
"""Dress rehearsal of the arm bench test (README_ARM.md, "First test on the bench" and "Daily workflow") without hardware.

Runs the real launch files and parameter files (`ros2 launch arm_pilot_redemption arm.launch.py` /
`base_station.launch.py`) against the simulated master and slaves, presses the PS5 buttons in the README's order, and
times every stop. Stand-ins: the master and slaves are fake_arm on a pseudo-terminal inside this process; the PS5 is
this script publishing /joy the way joy_node does (20 Hz autorepeat); "unplugging" either of them stops it. Nothing
touches real hardware. It uses the default joystick indices of config/arm.yaml.

Needs ROS 2 Humble and this workspace built and sourced, with the RMW and ROS_DOMAIN_ID you use on the arm:
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    python3 tools/arm_rehearsal.py                 # every step; PASS/FAIL table, exit code 1 on any failure
"""
from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'arm_pilot_redemption'))

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

from arm_pilot_redemption import fake_arm, protocol  # noqa: E402

LINK = '/tmp/arm-master-rehearsal'
CIRCLE, L1, OPTIONS = 1, 4, 9  # config/arm.yaml defaults
STOPPED = (0, 0, 0, 0)
ENABLED, ESTOP, FAULT = protocol.ENABLED, protocol.ESTOP, protocol.FAULT
VERSION = '.'.join(map(str, fake_arm.FIRMWARE_VERSION))


class SimulatedArm:
    """fake_arm on a fresh pseudo-terminal, reachable at LINK (the stand-in for /dev/arm-master)."""

    def __init__(self) -> None:
        self.fd, path = fake_arm.open_pty()
        self.fake = fake_arm.FakeArm(self.fd)
        self.thread = threading.Thread(target=self.fake.run, name='fake-arm', daemon=True)
        self.thread.start()
        link = Path(LINK)
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(path)

    def read(self) -> tuple[int, tuple[int, ...]]:
        return self.fake.read()

    def unplug(self) -> None:
        self.fake.stop_event.set()
        self.thread.join(2)
        os.close(self.fd)  # the driver's port now fails, as when the USB cable is pulled
        Path(LINK).unlink(missing_ok=True)


class Ps5:
    """Publishes /joy at 20 Hz like joy_node with autorepeat, while plugged in (Linux joy_node layout)."""

    def __init__(self, node) -> None:
        self.publisher = node.create_publisher(Joy, '/joy', 10)
        self.axes = {}
        self.pressed: set[int] = set()
        self.plugged = True
        node.create_timer(0.05, self.publish)

    def publish(self) -> None:
        if not self.plugged:
            return
        message = Joy()
        axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        for index, value in self.axes.items():
            axes[index] = value
        message.axes = axes
        message.buttons = [1 if index in self.pressed else 0 for index in range(13)]
        self.publisher.publish(message)

    def set(self, left_y: float = 0.0, right_y: float = 0.0, dpad_x: float = 0.0, dpad_y: float = 0.0,
            pressed: tuple[int, ...] = ()) -> None:
        self.axes, self.pressed = {1: left_y, 4: right_y, 6: dpad_x, 7: dpad_y}, set(pressed)

    def press(self, button: int, hold_s: float = 0.25, **inputs: float) -> None:
        self.set(pressed=(button,), **inputs)
        time.sleep(hold_s)
        self.set(**inputs)


class Launch:
    """One `ros2 launch` in its own process group, so Ctrl-C reaches every node like in a terminal."""

    def __init__(self, name: str, logs: Path, *arguments: str) -> None:
        self.name, self.log = name, logs / f'{name}.log'
        environment = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')
        self.process = subprocess.Popen(['ros2', 'launch', 'arm_pilot_redemption', *arguments], stdout=self.log.open('a'),
                                        stderr=subprocess.STDOUT, start_new_session=True, env=environment)

    def interrupt(self) -> None:
        os.killpg(self.process.pid, signal.SIGINT)

    def wait_exit(self) -> int | None:
        try:
            return self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.kill()
            return None

    def ctrl_c(self) -> int | None:
        self.interrupt()
        return self.wait_exit()

    def kill(self) -> None:
        """The laptop dies: no shutdown code runs."""
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait(timeout=10)

    def text(self) -> str:
        return self.log.read_text(errors='replace')


class Rehearsal:
    def __init__(self, logs: Path) -> None:
        self.logs = logs
        self.results: list[tuple[str, bool, str]] = []
        rclpy.init()
        self.node = rclpy.create_node('arm_rehearsal_ps5')
        self.ps5 = Ps5(self.node)
        self.diagnostics: dict[str, dict[str, str]] = {}
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spinner = threading.Thread(target=self.executor.spin, daemon=True)
        self.spinner.start()
        self.arm: SimulatedArm | None = None
        self.driver: Launch | None = None
        self.base: Launch | None = None

    def on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name.startswith('arm '):
                self.diagnostics[status.name] = {item.key: item.value for item in status.values} | {'_message': status.message}

    # ------------------------------------------------------------------ helpers
    def state(self) -> tuple[int, tuple[int, ...]] | None:
        return self.arm.read() if self.arm else None

    def wait(self, condition, timeout: float) -> float | None:
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            current = self.state()
            if current is not None and condition(current):
                return time.monotonic() - started
            time.sleep(0.005)
        return None

    def holds(self, condition, seconds: float) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            current = self.state()
            if current is None or not condition(current):
                return False
            time.sleep(0.01)
        return True

    def describe(self) -> str:
        current = self.state()
        return 'no arm' if current is None else f'{protocol.state_name(current[0])} PWM {current[1]}'

    def check(self, name: str, ok: bool, detail: str = '') -> bool:
        self.results.append((name, bool(ok), detail or self.describe()))
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail or self.describe()}", flush=True)
        return bool(ok)

    def timed(self, latency: float | None) -> str:
        return f'{latency:.3f} s' if latency is not None else self.describe()

    def start_driver(self) -> None:
        self.driver = Launch('arm', self.logs, 'arm.launch.py', f'port:={LINK}')

    def start_base(self) -> None:
        self.base = Launch('base_station', self.logs, 'base_station.launch.py')

    def release(self) -> bool:
        """Centre every input and press Options, as the operator does; True once the arm is enabled."""
        self.ps5.set()
        time.sleep(0.15)
        self.ps5.press(OPTIONS)
        return self.wait(lambda s: s == (ENABLED, STOPPED), 2.0) is not None

    def estop_and_release(self) -> bool:
        self.ps5.press(CIRCLE)
        time.sleep(0.15)
        return self.release()

    def move_shoulder(self, amount: float = 0.5) -> bool:
        self.ps5.set(left_y=amount)
        return self.wait(lambda s: s[0] == ENABLED and s[1][0] > 0, 2.0) is not None

    def installed_matches_source(self) -> list[str]:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory  # noqa: PLC0415

        source = ROOT / 'ros2_ws' / 'src' / 'arm_pilot_redemption'
        share = Path(get_package_share_directory('arm_pilot_redemption'))
        site = next(Path(get_package_prefix('arm_pilot_redemption')).glob('lib/python3*/site-packages'))
        modules = site / 'arm_pilot_redemption'  # a copy of the modules ...
        link = next(site.glob('*.egg-link'), None)
        if not modules.is_dir() and link is not None:  # ... or, with --symlink-install, an egg-link to the build tree
            modules = Path(link.read_text().splitlines()[0]) / 'arm_pilot_redemption'
        pairs = [(path, modules / path.name) for path in (source / 'arm_pilot_redemption').glob('*.py')]
        pairs += [(path, share / path.relative_to(source)) for pattern in ('config/*.yaml', 'launch/*.py') for path in source.glob(pattern)]
        return [str(src.relative_to(source)) for src, installed in pairs if not installed.exists() or installed.read_bytes() != src.read_bytes()]

    # ------------------------------------------------------------------ the run
    def run(self) -> bool:
        print(f'logs in {self.logs}', flush=True)
        stale = self.installed_matches_source()
        if not self.check('installed package matches the source', not stale, 'up to date' if not stale else
                          f"rebuild first (cd ros2_ws && colcon build --symlink-install): {', '.join(stale)}"):
            return False
        self.arm = SimulatedArm()

        print('\n1. Joint test (tools/arm_joint_test.py, arm driver not running)', flush=True)
        result = subprocess.run([sys.executable, str(ROOT / 'tools' / 'arm_joint_test.py'), LINK, '--seconds', '0.6'],
                                capture_output=True, text=True, timeout=90)
        (self.logs / 'arm_joint_test.log').write_text(result.stdout + result.stderr)
        lines = result.stdout.splitlines()

        def step_pwm(label: str) -> tuple[int, ...] | None:
            index = next((i for i, line in enumerate(lines) if line.startswith(f'>>> {label}')), None)
            line = next((line for line in lines[index + 1:] if 'PWM' in line), '') if index is not None else ''
            match = re.search(r'shoulder\s+(-?\d+).*elbow\s+(-?\d+).*wrist_a\s+(-?\d+).*wrist_b\s+(-?\d+)', line)
            return tuple(int(value) for value in match.groups()) if match else None

        speed = round(round(117.0 * 0.2 * 10) / 10 / 117.0 * 255)  # 20 % of full speed, rounded like the firmware
        self.check('joint test exits cleanly', result.returncode == 0, f'exit {result.returncode}')
        self.check(f'joint test reports fw {VERSION}', f'fw {VERSION}' in result.stdout)
        for index, name in enumerate(protocol.MOTORS):
            expected = tuple(speed if i == index else 0 for i in range(4))
            self.check(f'joint test: {name} positive only', step_pwm(f'{name} positive only') == expected, str(step_pwm(f'{name} positive only')))
            self.check(f'joint test: {name} back', step_pwm(f'{name} back') == tuple(-p for p in expected), str(step_pwm(f'{name} back')))
        self.check('joint test: wrist pitch (bevel motors together)', step_pwm('wrist pitch up') == (0, 0, speed, speed), str(step_pwm('wrist pitch up')))
        self.check('joint test: wrist roll (bevel motors opposite)', step_pwm('wrist roll positive') == (0, 0, speed, -speed),
                   str(step_pwm('wrist roll positive')))
        self.check('joint test leaves the arm in e-stop', self.wait(lambda s: s == (ESTOP, STOPPED), 1.0) is not None)
        self.arm.unplug()
        self.arm = SimulatedArm()  # freshly booted

        print('\n2. Daily workflow: launch the arm and the base station', flush=True)
        self.start_driver()
        self.start_base()
        self.check('master boots with every motor off', self.state() == (protocol.BOOT, STOPPED))
        self.check('launches come up; the e-stop starts latched', self.wait(lambda s: s[0] == ESTOP, 30.0) is not None
                   and self.holds(lambda s: s[0] == ESTOP, 1.0))
        self.check('Options with every input centred: arm enabled', self.release())

        print('\n3. Joystick', flush=True)
        for label, inputs, expected in (('left stick up: shoulder up at the 35 % PWM limit', {'left_y': 1.0}, (89, 0, 0, 0)),
                                        ('right stick down: elbow down at the limit', {'right_y': -1.0}, (0, -89, 0, 0)),
                                        ('D-pad up: wrist pitch (both bevel motors forward)', {'dpad_y': 1.0}, (0, 0, 89, 89)),
                                        ('D-pad right: auger roll (bevel motors opposite)', {'dpad_x': -1.0}, (0, 0, 89, -89))):
            self.ps5.set(**inputs)
            self.check(label, self.wait(lambda s, e=expected: s == (ENABLED, e), 2.0) is not None)
        self.ps5.set(dpad_y=1.0, dpad_x=-1.0)
        self.check('D-pad up + right: pitch and roll together, scaled to fit the motors',
                   self.wait(lambda s: s[0] == ENABLED and s[1][2] == 89 and s[1][3] == 0, 2.0) is not None)
        self.ps5.set(left_y=1.0, pressed=(L1,))
        self.check('L1 held: slow mode (35 % of the limit)', self.wait(lambda s: s[0] == ENABLED and 29 <= s[1][0] <= 33, 2.0) is not None)
        self.ps5.set()
        self.check('inputs released: motors stop, still enabled', self.wait(lambda s: s == (ENABLED, STOPPED), 1.0) is not None)

        print('\n4. Stops', flush=True)
        self.move_shoulder(1.0)
        self.ps5.set(left_y=1.0, pressed=(CIRCLE,))
        latency = self.wait(lambda s: s == (ESTOP, STOPPED), 2.0)
        self.check('Circle: e-stop', latency is not None and latency < 0.15, self.timed(latency))
        self.ps5.set(left_y=1.0)
        time.sleep(0.2)
        self.ps5.press(OPTIONS, left_y=1.0)
        self.check('Options with a stick pushed: refused', self.holds(lambda s: s == (ESTOP, STOPPED), 1.0))
        self.check('centred + Options: enabled again', self.release())

        self.move_shoulder()
        self.ps5.plugged = False
        latency = self.wait(lambda s: s == (ESTOP, STOPPED), 3.0)
        self.check('PS5 unplugged: e-stop', latency is not None and latency < 0.7, self.timed(latency))
        self.ps5.plugged = True
        self.ps5.set(left_y=0.5)
        self.check('PS5 back, stick pushed: still stopped', self.holds(lambda s: s == (ESTOP, STOPPED), 1.0))
        self.check('centred + Options after replugging: enabled again', self.release())

        self.move_shoulder()
        self.base.interrupt()
        latency = self.wait(lambda s: s == (ESTOP, STOPPED), 2.0)
        self.check('Ctrl-C on the base station launch: e-stop before the heartbeat timeout', latency is not None and latency < 0.4,
                   self.timed(latency))
        code = self.base.wait_exit()
        self.check('base station shutdown without a traceback', 'Traceback' not in self.base.text() and code == 0, f'launch exit {code}')
        self.start_base()
        time.sleep(3.0)
        self.check('base station restarted: still stopped until released', self.state() == (ESTOP, STOPPED))
        self.check('centred + Options: enabled again', self.release())

        self.move_shoulder()
        self.base.kill()
        latency = self.wait(lambda s: s == (ESTOP, STOPPED), 3.0)
        self.check('base station lost (no shutdown): e-stop from the heartbeat timeout', latency is not None and latency < 0.8,
                   self.timed(latency))
        self.start_base()
        time.sleep(3.0)
        self.check('base station back: enabled again after Options', self.release())

        self.move_shoulder()
        self.driver.interrupt()
        latency = self.wait(lambda s: s[1] == STOPPED, 2.0)
        self.check('Ctrl-C on the arm launch: motors stop before the master timeout', latency is not None and latency < 0.3,
                   f'{self.timed(latency)}, {self.describe()}')
        code = self.driver.wait_exit()
        self.check('the driver e-stopped the master on its way out', self.state() == (ESTOP, STOPPED), f'{self.describe()}, exit {code}')
        self.check('arm shutdown without a traceback', 'Traceback' not in self.driver.text() and code == 0, f'launch exit {code}')
        self.ps5.set(left_y=0.5)
        self.start_driver()
        time.sleep(4.0)
        self.check('arm relaunched with a stick pushed: the master waits for neutral',
                   self.state() is not None and self.state()[1] == STOPPED and self.state()[0] in (ESTOP, protocol.WAIT_NEUTRAL))
        self.ps5.set()
        self.check('inputs centred: enabled again', self.wait(lambda s: s == (ENABLED, STOPPED), 2.0) is not None)

        self.move_shoulder()
        self.arm.unplug()
        time.sleep(1.5)
        self.check('master USB unplugged: the driver notices and keeps running',
                   'lost' in self.driver.text() and self.driver.process.poll() is None)
        self.arm = SimulatedArm()
        self.check('USB back with the stick still pushed: the master waits for neutral',
                   self.wait(lambda s: s[0] == protocol.WAIT_NEUTRAL, 4.0) is not None and self.holds(lambda s: s[1] == STOPPED, 0.5))
        self.ps5.set()
        self.check('inputs centred: enabled again', self.wait(lambda s: s == (ENABLED, STOPPED), 2.0) is not None)

        self.move_shoulder()
        self.arm.fake.reset_master()  # EN button on the master with the stick pushed
        latency = self.wait(lambda s: s[1] == STOPPED, 1.0)
        self.check('master reset (EN): the slaves stop on their own timeout', latency is not None and latency < 0.2, self.timed(latency))
        self.check('master back with the stick pushed: nothing moves until centred',
                   self.wait(lambda s: s[0] == protocol.WAIT_NEUTRAL, 1.0) is not None and self.holds(lambda s: s[1] == STOPPED, 1.0))
        self.ps5.set()
        self.check('inputs centred: enabled again', self.wait(lambda s: s == (ENABLED, STOPPED), 2.0) is not None)

        print('\n5. Slaves', flush=True)
        self.ps5.set(dpad_y=1.0)
        self.wait(lambda s: s == (ENABLED, (0, 0, 89, 89)), 2.0)
        self.arm.fake.slaves[3].online = False  # wrist_b drops off the bus while the wrist moves
        latency = self.wait(lambda s: s == (FAULT, STOPPED), 2.0)
        self.check('wrist_b slave lost mid-move: every motor stops (FAULT)', latency is not None and latency < 0.3, self.timed(latency))
        time.sleep(0.5)
        self.check('the arm log names the lost slave', 'wrist_b: not answering on I2C' in self.driver.text())
        self.arm.fake.slaves[3].online = True
        self.ps5.set()
        time.sleep(0.3)
        self.ps5.press(OPTIONS)
        self.check('slave back, Options alone: still FAULT', self.holds(lambda s: s == (FAULT, STOPPED), 0.5))
        self.check('Circle then Options: enabled again', self.estop_and_release())

        self.ps5.set(right_y=1.0)
        self.wait(lambda s: s[0] == ENABLED and s[1][1] > 0, 2.0)
        self.arm.fake.slaves[1].inject_fault(protocol.SLAVE_FAULT_STALL)
        latency = self.wait(lambda s: s == (FAULT, STOPPED), 2.0)
        self.check('elbow stall: every motor stops (FAULT)', latency is not None and latency < 0.3, self.timed(latency))
        time.sleep(0.5)
        self.check('the arm log says the elbow stalled', 'elbow: stalled' in self.driver.text())
        self.ps5.set()
        self.check('Circle then Options: enabled again', self.estop_and_release())

        self.arm.fake.slaves[0].reported_joint = 1  # a board flashed with slave_elbow at the shoulder's address
        self.ps5.press(CIRCLE)
        time.sleep(0.2)
        self.ps5.press(OPTIONS)
        self.check('a slave built for the wrong joint: the master will not enable',
                   self.holds(lambda s: s[0] == protocol.WAIT_SLAVES and s[1] == STOPPED, 1.0))
        self.check('the arm log says which slave', 'shoulder: built for another joint' in self.driver.text())
        self.arm.fake.slaves[0].reported_joint = 0  # reflashed
        self.check('right slave fitted: enabled without another release', self.wait(lambda s: s == (ENABLED, STOPPED), 2.0) is not None)

        print('\n6. Diagnostics', flush=True)
        self.move_shoulder()
        time.sleep(1.5)
        master = dict(self.diagnostics.get('arm master', {}))
        self.check(f'/diagnostics: firmware {VERSION}', master.get('firmware') == VERSION, master.get('firmware', 'none'))
        self.check('/diagnostics: rx_bad stays 0', master.get('rx_bad') == '0', master.get('rx_bad', 'none'))
        self.check('/diagnostics: enabled', master.get('_message') == 'enabled', master.get('_message', 'none'))
        motors = [self.diagnostics.get(f'arm motor {name}', {}).get('_message') for name in protocol.MOTORS]
        self.check('/diagnostics: all four motors enabled', motors == ['enabled'] * 4, str(motors))

        print('\n7. End of the session: Circle, then Ctrl-C on both machines', flush=True)
        self.ps5.press(CIRCLE)
        self.check('Circle: e-stop', self.wait(lambda s: s == (ESTOP, STOPPED), 1.0) is not None)
        self.base.ctrl_c()
        self.driver.ctrl_c()
        self.check('no tracebacks in any launch log', 'Traceback' not in self.base.text() + self.driver.text())
        return all(ok for _, ok, _ in self.results)

    def close(self) -> None:
        for launch in (self.base, self.driver):
            if launch is not None and launch.process.poll() is None:
                launch.ctrl_c()
        if self.arm is not None:
            self.arm.unplug()
        self.executor.shutdown()
        self.spinner.join(5)  # destroying the node under a spinning executor aborts the process
        self.node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()
    logs = Path(tempfile.mkdtemp(prefix='arm-rehearsal-'))
    rehearsal = Rehearsal(logs)
    try:
        passed = rehearsal.run()
    except KeyboardInterrupt:
        passed = False
    finally:
        rehearsal.close()
    failed = [name for name, ok, _ in rehearsal.results if not ok]
    print(f"\n{len(rehearsal.results) - len(failed)}/{len(rehearsal.results)} checks passed" + (f"; failed: {', '.join(failed)}" if failed else ''))
    print(f'launch logs: {logs}')
    return 0 if passed and not failed else 1


if __name__ == '__main__':
    sys.exit(main())
