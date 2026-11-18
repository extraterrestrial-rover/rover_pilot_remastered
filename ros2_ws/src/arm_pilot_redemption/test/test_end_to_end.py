"""Arm gate + kinematics + protocol over a real pseudo-terminal to the simulated master and slaves (no ROS needed)."""
import os
import time

import pytest
import serial

from arm_pilot_redemption import fake_arm, protocol
from arm_pilot_redemption.arm_logic import ArmGate
from arm_pilot_redemption.kinematics import ArmGeometry, joint_velocity_to_motor_rpm, with_pwm_limit

pytestmark = pytest.mark.skipif(not hasattr(os, 'openpty'), reason='needs POSIX pseudo-terminals')
ARM = with_pwm_limit(ArmGeometry((117.0,) * 4, 1.0, 1.0, 1.0, 1.0), 0.35)  # full stick = PWM 89
TOP = ARM.max_joint_velocity


class Link:
    def __init__(self, path):
        self.port = serial.Serial(path, 115200, timeout=0)
        self.decoder = protocol.PacketDecoder()
        self.seq = 0
        self.status = None
        self.info = None

    def run(self, gate, seconds, refresh=None):
        """Send at 50 Hz for `seconds`; `refresh(now)` keeps the gate's inputs fresh. Returns the last ARM_STATUS."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if refresh is not None:
                refresh(time.monotonic())
            decision = gate.decide(time.monotonic())
            self.send(decision.estop, joint_velocity_to_motor_rpm(decision.velocities, ARM))
            time.sleep(0.02)
        return self.status

    def send(self, estop, targets):
        self.port.write(protocol.encode_packet(protocol.TYPE_ARM_COMMAND, self.seq, protocol.arm_command_payload(estop, targets)))
        self.seq = (self.seq + 1) & 0xFF
        self.read()

    def read(self):
        for packet in self.decoder.feed(self.port.read(self.port.in_waiting or 0)):
            if packet.type == protocol.TYPE_ARM_STATUS:
                self.status = protocol.parse_arm_status(packet.payload) or self.status
            elif packet.type == protocol.TYPE_ARM_INFO:
                self.info = protocol.parse_arm_info(packet.payload) or self.info


def operator(gate, velocities, stopped=False):
    def refresh(now):
        gate.on_estop(stopped, now)
        gate.on_joint_cmd(velocities, now)
    return refresh


def pwm(status):
    return tuple(motor.pwm for motor in status.motors)


def test_full_link_with_slaves_faults_and_timeouts():
    fake, path = fake_arm.start_in_background()
    link = Link(path)
    gate = ArmGate()
    try:
        # Operator holds the shoulder stick when the link comes up: the master must wait for neutral.
        status = link.run(gate, 0.3, operator(gate, (TOP[0], 0.0, 0.0, 0.0)))
        assert status.state_name == 'WAIT_NEUTRAL' and pwm(status) == (0, 0, 0, 0)
        link.run(gate, 0.2, operator(gate, (0.0,) * 4))
        status = link.run(gate, 0.3, operator(gate, (TOP[0] / 2, 0.0, 0.0, 0.0)))
        assert status.state_name == 'ENABLED' and status.slaves_ready
        assert 44 <= status.motors[0].pwm <= 46 and pwm(status)[1:] == (0, 0, 0)
        assert status.motors[0].velocity_rpm == pytest.approx(20.5, abs=0.1) and status.motors[0].position > 0
        assert all(motor.known and motor.counts_per_rev == 1425.1 for motor in link.info)

        # Wrist roll alone: the bevel motors turn opposite ways.
        status = link.run(gate, 0.3, operator(gate, (0.0, 0.0, 0.0, TOP[3])))
        assert pwm(status) == (0, 0, 89, -89)

        # wrist_b's slave drops off the bus: every motor stops, FAULT names it.
        fake.slaves[3].online = False
        status = link.run(gate, 0.4, operator(gate, (TOP[0] / 2, 0.0, 0.0, 0.0)))
        assert status.state_name == 'FAULT' and fake.read() == (protocol.FAULT, (0, 0, 0, 0))
        assert status.motors[3].link_faults & protocol.LINK_LOST and status.motors[3].i2c_errors > 0
        # Back on the bus: still FAULT until the operator e-stops and releases with the sticks centred.
        fake.slaves[3].online = True
        status = link.run(gate, 0.3, operator(gate, (0.0,) * 4))
        assert status.state_name == 'FAULT'
        link.run(gate, 0.2, operator(gate, (0.0,) * 4, stopped=True))
        status = link.run(gate, 0.3, operator(gate, (0.0,) * 4))
        assert status.state_name == 'ENABLED' and status.slaves_ready and status.motors[3].link_faults == 0

        # A slave detects a stall: everything stops and the reason survives the master's e-stop to the slaves.
        link.run(gate, 0.2, operator(gate, (0.0, TOP[1], 0.0, 0.0)))
        fake.slaves[1].inject_fault(protocol.SLAVE_FAULT_STALL)
        status = link.run(gate, 0.3, operator(gate, (0.0, TOP[1], 0.0, 0.0)))
        assert status.state_name == 'FAULT' and status.motors[1].slave_faults == protocol.SLAVE_FAULT_STALL
        assert fake.read()[1] == (0, 0, 0, 0)
        link.run(gate, 0.2, operator(gate, (0.0,) * 4, stopped=True))
        link.run(gate, 0.3, operator(gate, (0.0,) * 4))

        # Base station disappears: heartbeat stale -> e-stop flag -> motors off.
        link.run(gate, 0.2, operator(gate, (TOP[0], 0.0, 0.0, 0.0)))
        status = link.run(gate, 0.8)
        assert status.state_name == 'ESTOP' and pwm(status) == (0, 0, 0, 0)

        # The driver itself dies: no commands at all -> master timeout.
        link.run(gate, 0.2, operator(gate, (0.0,) * 4))
        time.sleep(0.6)
        link.read()
        assert link.status.state_name == 'NO_COMMAND'
        assert link.status.rx_bad == 0
    finally:
        fake.stop_event.set()
        link.port.close()


def test_slaves_stop_on_their_own_when_the_master_hangs():
    fake, path = fake_arm.start_in_background()
    link = Link(path)
    gate = ArmGate()
    try:
        link.run(gate, 0.3, operator(gate, (0.0,) * 4))
        link.run(gate, 0.3, operator(gate, (TOP[0], 0.0, 0.0, 0.0)))
        assert fake.read()[1][0] == 89
        with fake.lock:  # the master's loop stops exchanging frames
            fake.bus_cycle = lambda now_ms: None
        time.sleep(0.2)
        with fake.lock:
            assert [slave.state for slave in fake.slaves] == [protocol.NO_COMMAND] * 4
            assert all(slave.pwm() == 0 for slave in fake.slaves)
    finally:
        fake.stop_event.set()
        link.port.close()
