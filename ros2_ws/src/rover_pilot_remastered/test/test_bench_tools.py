"""tools/wheel_test.py against the simulated ESP32 (no ROS)."""
import socket
import subprocess
import sys
from pathlib import Path

from rover_pilot_remastered import fake_esp

TOOLS = Path(__file__).resolve().parents[4] / 'tools'


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def test_wheel_test_says_so_when_the_esp32_does_not_answer():
    simulator = fake_esp.Simulator(port=free_port(), pc=('127.0.0.1', free_port()))
    try:
        simulator.unplug()  # the Ethernet cable is out
        result = subprocess.run([sys.executable, str(TOOLS / 'wheel_test.py'), *simulator.tool_arguments, '--seconds', '0.5'],
                                capture_output=True, text=True, timeout=60)
    finally:
        simulator.close()
    output = result.stdout + result.stderr
    assert result.returncode == 1, output
    assert 'did not reach DRIVING' in output and 'Traceback' not in output, output
