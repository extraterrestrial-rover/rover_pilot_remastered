"""Joystick -> base_teleop -> /cmd_vel + /rover/estop -> drive_node -> UDP -> simulated ESP32, through real ROS 2.

drive_node and base_teleop run as separate processes, as on the rover and the base station; this test publishes
/joy and watches the simulated ESP32. Needs a sourced ROS 2 environment (skipped otherwise):
    source /opt/ros/humble/setup.bash && python3 -m pytest test/test_ros_integration.py
"""
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import Joy  # noqa: E402

from rover_pilot_remastered import fake_esp  # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CIRCLE, OPTIONS = 1, 6
STOPPED = (0, 0, 0, 0)


def start(module: str, *ros_args: str) -> subprocess.Popen:
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(PACKAGE_ROOT), os.environ.get('PYTHONPATH', '')]))
    return subprocess.Popen([sys.executable, '-m', f'rover_pilot_remastered.{module}', '--ros-args', *ros_args],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def simulated_esp():
    """A simulated ESP32 on a free loopback port, and the drive_node parameters that reach it."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        pc_port = probe.getsockname()[1]  # for drive_node; free a moment ago
    fake, (host, port) = fake_esp.start_in_background(pc=('127.0.0.1', pc_port))
    return fake, ['-p', f'master_host:={host}', '-p', f'master_port:={port}', '-p', f'local_port:={pc_port}']


def test_joystick_to_wheels_through_ros():
    fake, link = simulated_esp()
    processes = [start('drive_node', *link), start('base_teleop')]
    rclpy.init()
    node = rclpy.create_node('fake_joystick')
    publisher = node.create_publisher(Joy, '/joy', 1)

    def esp():
        with fake.lock:
            return fake.controller.state, fake.controller.pwm()

    def hold(until=None, timeout=1.0, left_y=0.0, right_x=0.0, pressed=()):
        """Publish one joystick state at 50 Hz until `until(esp())` holds (returns the elapsed time) or the timeout."""
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            message = Joy()
            message.axes = [0.0, left_y, right_x, 0.0, 1.0, 1.0]
            message.buttons = [1 if index in pressed else 0 for index in range(15)]
            publisher.publish(message)
            if until is not None and until(esp()):
                return time.monotonic() - started
            time.sleep(0.02)
        return None

    try:
        # Discovery; the teleop starts latched, so the ESP32 is told to e-stop.
        assert hold(lambda s: s == (fake_esp.ESTOP, STOPPED), timeout=15.0) is not None, esp()

        hold(timeout=0.2, pressed=(OPTIONS,))  # release with the sticks centred
        assert hold(lambda s: s == (fake_esp.DRIVING, STOPPED), timeout=2.0) is not None, esp()

        assert hold(lambda s: s == (fake_esp.DRIVING, (255, 255, 255, 255)), timeout=2.0, left_y=1.0) is not None, esp()
        # Stick right = turn right: left wheels forward, right wheels backward.
        assert hold(lambda s: s == (fake_esp.DRIVING, (255, -255, 255, -255)), timeout=2.0, right_x=-1.0) is not None, esp()

        latency = hold(lambda s: s == (fake_esp.ESTOP, STOPPED), timeout=2.0, left_y=1.0, pressed=(CIRCLE,))
        assert latency is not None and latency < 0.6, (latency, esp())

        hold(timeout=0.6, left_y=1.0, pressed=(OPTIONS,))  # release refused while the stick is pushed
        assert esp() == (fake_esp.ESTOP, STOPPED)

        hold(timeout=0.3)
        hold(timeout=0.2, pressed=(OPTIONS,))
        assert hold(lambda s: s[0] == fake_esp.DRIVING and all(100 < p < 150 for p in s[1]), timeout=2.0, left_y=0.5) is not None, esp()

        # The base station goes silent: joystick lost / heartbeat stale -> e-stop.
        started = time.monotonic()
        while esp() != (fake_esp.ESTOP, STOPPED) and time.monotonic() - started < 3.0:
            time.sleep(0.02)
        assert esp() == (fake_esp.ESTOP, STOPPED)
        assert time.monotonic() - started < 1.0
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        node.destroy_node()
        rclpy.shutdown()
        fake.stop_event.set()


def test_stopping_the_base_station_latches_the_estop_at_once():
    """Ctrl-C on base_teleop sends a final e-stop, so the rover stops well before the 0.5 s heartbeat timeout."""
    fake, link = simulated_esp()
    drive, teleop = start('drive_node', *link), start('base_teleop')
    rclpy.init()
    node = rclpy.create_node('fake_joystick_shutdown')
    publisher = node.create_publisher(Joy, '/joy', 1)

    def esp():
        with fake.lock:
            return fake.controller.state, fake.controller.pwm()

    def publish(left_y=0.0, pressed=()):
        message = Joy()
        message.axes = [0.0, left_y, 0.0, 0.0, 1.0, 1.0]
        message.buttons = [1 if index in pressed else 0 for index in range(15)]
        publisher.publish(message)

    def wait(condition, timeout, **joy):
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            publish(**joy)
            if condition(esp()):
                return True
            time.sleep(0.02)
        return False

    try:
        assert wait(lambda s: s[0] == fake_esp.ESTOP, 15.0), esp()  # discovery; the teleop starts latched
        wait(lambda s: False, 0.2, pressed=(OPTIONS,))
        assert wait(lambda s: s[0] == fake_esp.DRIVING and s[1][0] > 100, 3.0, left_y=0.5), esp()
        teleop.send_signal(signal.SIGINT)
        started = time.monotonic()
        while esp()[0] != fake_esp.ESTOP and time.monotonic() - started < 2.0:
            time.sleep(0.005)
        elapsed = time.monotonic() - started
        assert esp() == (fake_esp.ESTOP, STOPPED)
        assert elapsed < 0.35, elapsed  # the heartbeat timeout alone would take 0.5-0.6 s
        teleop.wait(timeout=5)
        assert 'Traceback' not in teleop.stdout.read()
    finally:
        for process in (drive, teleop):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        node.destroy_node()
        rclpy.shutdown()
        fake.stop_event.set()


def test_the_double_sigint_from_ros2_launch_still_shuts_down_cleanly():
    """Ctrl-C on `ros2 launch` reaches each node twice (the terminal's SIGINT to the process group, then the one launch
    forwards). The second used to interrupt the final e-stop and end the node with a KeyboardInterrupt traceback."""
    fake, link = simulated_esp()
    drive, teleop = start('drive_node', *link), start('base_teleop')
    try:
        started = time.monotonic()
        while fake.decoder.good < 20 and time.monotonic() - started < 15.0:  # the drive node is sending commands
            time.sleep(0.05)
        time.sleep(1.0)  # and the teleop is up
        for process in (teleop, drive):
            process.send_signal(signal.SIGINT)
            time.sleep(0.005)
            process.send_signal(signal.SIGINT)
        for name, process in (('base_teleop', teleop), ('drive_node', drive)):
            assert process.wait(timeout=5) == 0, name
            output = process.stdout.read()
            assert 'Traceback' not in output, f'{name}:\n{output}'
        with fake.lock:
            assert fake.controller.state == fake_esp.ESTOP  # the final e-stop still went out
    finally:
        for process in (drive, teleop):
            if process.poll() is None:
                process.kill()
        fake.stop_event.set()
