"""gps_node through real ROS 2, with the receiver on a pseudo-terminal: what it passes on, and what it sends.
Needs a sourced ROS 2 environment (skipped otherwise)."""
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import NavSatFix, NavSatStatus  # noqa: E402

from rover_autonomy import nmea  # noqa: E402
from rover_pilot_remastered import fake_esp  # noqa: E402

pytestmark = pytest.mark.skipif(not hasattr(os, 'openpty'), reason='needs POSIX pseudo-terminals')
PACKAGE = Path(__file__).resolve().parents[1]
CAPTURE = PACKAGE / 'test' / 'data' / 'l89ha_no_fix.nmea'


def test_gps_node_passes_on_real_fixes_only(ros_domain):
    master, _slave, path = fake_esp.open_pty()
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(PACKAGE), str(PACKAGE.parent / 'rover_pilot_remastered'),
                                                       os.environ.get('PYTHONPATH', '')]))
    process = subprocess.Popen([sys.executable, '-m', 'rover_autonomy.gps_node', '--ros-args', '-p', f'gps_port:={path}',
                                '-p', 'gps_setup:=PAIR410,1; PQTMSAVEPAR'],
                               env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('gps_test')
    fixes, statuses = [], []
    node.create_subscription(NavSatFix, '/gps/fix', fixes.append, qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/diagnostics',
                             lambda m: statuses.extend(s for s in m.status if s.name == 'rover gps'), 10)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        sent = b''
        end = time.monotonic() + 15.0
        while time.monotonic() < end and sent.count(b'\n') < 2:  # the setup commands, once the port is open
            spin(0.1)
            try:
                sent += os.read(master, 1024)
            except BlockingIOError:
                pass
        assert sent.decode() == nmea.sentence('PAIR410,1') + nmea.sentence('PQTMSAVEPAR')
        os.write(master, nmea.sentence('PAIR001,410,0').encode())
        for line in CAPTURE.read_text().splitlines()[:60]:  # the real receiver without a fix
            os.write(master, (line + '\r\n').encode())
            spin(0.01)
        spin(1.5)
        assert fixes == [] and statuses and statuses[-1].message == 'no fix yet'
        for second in range(3):
            os.write(master, nmea.gga_sentence(43200.0 + second, 28.7501234, 77.1174567, quality=2, hdop=0.8).encode())
            spin(0.3)
        spin(1.5)
        assert len(fixes) == 3
        fix = fixes[-1]
        assert fix.latitude == pytest.approx(28.7501234, abs=1e-8) and fix.longitude == pytest.approx(77.1174567, abs=1e-8)
        assert fix.status.status == NavSatStatus.STATUS_SBAS_FIX and fix.header.frame_id == 'gps_link'
        assert fix.position_covariance[0] == pytest.approx(1.2 ** 2) and fix.position_covariance_type == 1
        assert statuses[-1].message == 'fix'
        os.write(master, nmea.gga_sentence(43210.0, 28.75, 77.11, satellites=3).encode())
        spin(1.5)
        assert len(fixes) == 3 and 'only 3 satellites' in statuses[-1].message
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        log = process.communicate(timeout=10)[0]
    assert 'the GPS answered PAIR410: done' in log, log
