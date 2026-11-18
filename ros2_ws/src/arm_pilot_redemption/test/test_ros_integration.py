"""Joystick -> arm_teleop -> /arm/joint_cmd + /arm/estop -> arm_driver -> serial -> simulated master and slaves.

arm_driver and arm_teleop run as separate processes, as on the arm computer and the base station; this test publishes
/joy and watches the simulated arm. Needs a sourced ROS 2 environment (skipped otherwise):
    source /opt/ros/humble/setup.bash && python3 -m pytest test/test_ros_integration.py
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import JointState, Joy  # noqa: E402

from arm_pilot_redemption import fake_arm, protocol  # noqa: E402

pytestmark = pytest.mark.skipif(not hasattr(os, 'openpty'), reason='needs POSIX pseudo-terminals')
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CIRCLE, OPTIONS = 1, 9
STOPPED = (0, 0, 0, 0)


def start(module: str, *ros_args: str) -> subprocess.Popen:
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(PACKAGE_ROOT), os.environ.get('PYTHONPATH', '')]))
    return subprocess.Popen([sys.executable, '-m', f'arm_pilot_redemption.{module}', '--ros-args', *ros_args],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def stop(*processes: subprocess.Popen) -> None:
    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


class Joystick:
    def __init__(self, node):
        self.publisher = node.create_publisher(Joy, '/joy', 1)

    def publish(self, left_y=0.0, right_y=0.0, dpad_x=0.0, dpad_y=0.0, pressed=()):
        message = Joy()
        message.axes = [0.0, left_y, 1.0, 0.0, right_y, 1.0, dpad_x, dpad_y]
        message.buttons = [1 if index in pressed else 0 for index in range(13)]
        self.publisher.publish(message)

    def hold(self, fake, until=None, timeout=1.0, **joy):
        """Publish one joystick state at 50 Hz until `until(fake.read())` holds (returns the elapsed time) or the timeout."""
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.publish(**joy)
            if until is not None and until(fake.read()):
                return time.monotonic() - started
            time.sleep(0.02)
        return None


def test_joystick_to_motors_through_ros():
    fake, path = fake_arm.start_in_background()
    processes = [start('arm_driver', '-p', f'port:={path}'), start('arm_teleop')]
    rclpy.init()
    node = rclpy.create_node('fake_arm_joystick')
    joystick = Joystick(node)
    states = []
    node.create_subscription(JointState, '/arm/joint_states', states.append, 10)
    try:
        # Discovery; the teleop starts latched, so the master is told to e-stop.
        assert joystick.hold(fake, lambda s: s == (protocol.ESTOP, STOPPED), timeout=15.0) is not None, fake.read()
        joystick.hold(fake, timeout=0.2, pressed=(OPTIONS,))  # release with every input centred
        assert joystick.hold(fake, lambda s: s == (protocol.ENABLED, STOPPED), timeout=2.0) is not None, fake.read()

        assert joystick.hold(fake, lambda s: s == (protocol.ENABLED, (89, 0, 0, 0)), timeout=2.0, left_y=1.0) is not None, fake.read()
        assert joystick.hold(fake, lambda s: s == (protocol.ENABLED, (0, -89, 0, 0)), timeout=2.0, right_y=-1.0) is not None, fake.read()
        # D-pad up: wrist pitch, both bevel motors forward. D-pad right: auger roll, opposite ways.
        assert joystick.hold(fake, lambda s: s == (protocol.ENABLED, (0, 0, 89, 89)), timeout=2.0, dpad_y=1.0) is not None, fake.read()
        assert joystick.hold(fake, lambda s: s == (protocol.ENABLED, (0, 0, 89, -89)), timeout=2.0, dpad_x=-1.0) is not None, fake.read()

        latency = joystick.hold(fake, lambda s: s == (protocol.ESTOP, STOPPED), timeout=2.0, left_y=1.0, pressed=(CIRCLE,))
        assert latency is not None and latency < 0.6, (latency, fake.read())
        joystick.hold(fake, timeout=0.6, left_y=1.0, pressed=(OPTIONS,))  # release refused while the stick is pushed
        assert fake.read() == (protocol.ESTOP, STOPPED)

        # Joint states flow back from the encoders, with positions once the slaves' calibration has arrived.
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert states and list(states[-1].name) == ['shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll']
        assert len(states[-1].position) == 4 and states[-1].position[0] > 0.0

        # The base station goes silent: joystick lost / heartbeat stale -> e-stop.
        joystick.hold(fake, timeout=0.3)
        joystick.hold(fake, timeout=0.2, pressed=(OPTIONS,))
        assert joystick.hold(fake, lambda s: s[0] == protocol.ENABLED and s[1][0] > 30, timeout=2.0, left_y=0.5) is not None
        started = time.monotonic()
        while fake.read() != (protocol.ESTOP, STOPPED) and time.monotonic() - started < 3.0:
            time.sleep(0.02)
        assert fake.read() == (protocol.ESTOP, STOPPED)
        assert time.monotonic() - started < 1.0
    finally:
        stop(*processes)
        node.destroy_node()
        rclpy.shutdown()
        fake.stop_event.set()


def test_a_lost_slave_stops_the_arm_and_the_operator_recovers_with_estop_and_release():
    fake, path = fake_arm.start_in_background()
    driver, teleop = start('arm_driver', '-p', f'port:={path}'), start('arm_teleop')
    rclpy.init()
    node = rclpy.create_node('fake_arm_joystick_fault')
    joystick = Joystick(node)
    try:
        assert joystick.hold(fake, lambda s: s[0] == protocol.ESTOP, timeout=15.0) is not None
        joystick.hold(fake, timeout=0.2, pressed=(OPTIONS,))
        assert joystick.hold(fake, lambda s: s == (protocol.ENABLED, (0, 0, 89, 89)), timeout=2.0, dpad_y=1.0) is not None
        fake.slaves[2].online = False  # wrist_a's slave drops off the bus mid-move
        latency = joystick.hold(fake, lambda s: s == (protocol.FAULT, STOPPED), timeout=2.0, dpad_y=1.0)
        assert latency is not None and latency < 0.3, (latency, fake.read())
        fake.slaves[2].online = True
        joystick.hold(fake, timeout=0.3)
        joystick.hold(fake, timeout=0.2, pressed=(OPTIONS,))  # release alone does not clear a fault
        assert fake.read()[0] == protocol.FAULT
        joystick.hold(fake, timeout=0.2, pressed=(CIRCLE,))
        joystick.hold(fake, timeout=0.2)
        joystick.hold(fake, timeout=0.2, pressed=(OPTIONS,))
        assert joystick.hold(fake, lambda s: s == (protocol.ENABLED, STOPPED), timeout=2.0) is not None, fake.read()
    finally:
        stop(driver, teleop)
        output = driver.stdout.read()
        node.destroy_node()
        rclpy.shutdown()
        fake.stop_event.set()
    assert 'wrist_a: not answering on I2C' in output, output


def test_stopping_the_base_station_latches_the_estop_at_once():
    """Ctrl-C on arm_teleop sends a final e-stop, so the arm stops well before the 0.5 s heartbeat timeout."""
    fake, path = fake_arm.start_in_background()
    driver, teleop = start('arm_driver', '-p', f'port:={path}'), start('arm_teleop')
    rclpy.init()
    node = rclpy.create_node('fake_arm_joystick_shutdown')
    joystick = Joystick(node)
    try:
        assert joystick.hold(fake, lambda s: s[0] == protocol.ESTOP, timeout=15.0) is not None
        joystick.hold(fake, timeout=0.2, pressed=(OPTIONS,))
        assert joystick.hold(fake, lambda s: s[0] == protocol.ENABLED and s[1][0] > 30, timeout=3.0, left_y=0.5) is not None
        teleop.send_signal(signal.SIGINT)
        started = time.monotonic()
        while fake.read() != (protocol.ESTOP, STOPPED) and time.monotonic() - started < 2.0:  # the motors, not just the master
            time.sleep(0.005)
        elapsed = time.monotonic() - started
        assert fake.read() == (protocol.ESTOP, STOPPED)
        assert elapsed < 0.35, elapsed
        teleop.wait(timeout=5)
        assert 'Traceback' not in teleop.stdout.read()
    finally:
        stop(driver, teleop)
        node.destroy_node()
        rclpy.shutdown()
        fake.stop_event.set()


def test_the_double_sigint_from_ros2_launch_still_shuts_down_cleanly():
    fake, path = fake_arm.start_in_background()
    driver, teleop = start('arm_driver', '-p', f'port:={path}'), start('arm_teleop')
    try:
        started = time.monotonic()
        while fake.decoder.good < 20 and time.monotonic() - started < 15.0:  # the driver is sending commands
            time.sleep(0.05)
        time.sleep(1.0)  # and the teleop is up
        for process in (teleop, driver):
            process.send_signal(signal.SIGINT)
            time.sleep(0.005)
            process.send_signal(signal.SIGINT)
        for name, process in (('arm_teleop', teleop), ('arm_driver', driver)):
            assert process.wait(timeout=5) == 0, name
            output = process.stdout.read()
            assert 'Traceback' not in output, f'{name}:\n{output}'
        assert fake.read()[0] == protocol.ESTOP  # the final e-stop still went out
    finally:
        for process in (driver, teleop):
            if process.poll() is None:
                process.kill()
        fake.stop_event.set()
