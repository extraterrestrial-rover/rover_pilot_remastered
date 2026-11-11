"""tools/wheel_test.py against the simulated ESP32 (no ROS)."""
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from rover_pilot_remastered import fake_esp

pytestmark = pytest.mark.skipif(not hasattr(os, 'openpty'), reason='needs POSIX pseudo-terminals')
TOOLS = Path(__file__).resolve().parents[4] / 'tools'


def test_wheel_test_reports_a_lost_port_instead_of_a_traceback():
    simulator = fake_esp.Simulator()
    try:
        tool = subprocess.Popen([sys.executable, str(TOOLS / 'wheel_test.py'), *simulator.tool_arguments, '--seconds', '3'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and (simulator.state() or (0,))[0] != fake_esp.DRIVING:
            time.sleep(0.05)
        assert simulator.state()[0] == fake_esp.DRIVING
        time.sleep(0.5)
        simulator.unplug()  # the USB cable is pulled while a wheel turns
        output, _ = tool.communicate(timeout=15)
    finally:
        simulator.close()
    assert tool.returncode == 1, output
    assert 'Lost the port' in output and 'Traceback' not in output, output
