"""Serial protocol between the arm driver (PC) and the arm master ESP32, and the I2C frames master <-> slaves.

Framing is the rover's: each serial packet is ``version | type | seq | payload | crc16-le``, COBS-encoded and terminated
by a single 0x00 byte. The packet types differ from the rover's, so the arm master never accepts a rover drive command
and the rover's drive ESP32 never accepts an arm command. The firmware implements the same protocol in
firmware_arm/lib/arm_core (C++). Motor order everywhere: shoulder, elbow, wrist_a, wrist_b.

Serial packets:
  0x83 ARM_COMMAND  PC -> master, 50 Hz: flags (bit 0 = e-stop) + four gearmotor targets in 0.1 rpm
  0x05 ARM_STATUS   master -> PC, 20 Hz: master state, link counters, uptime, version, and per motor the slave state,
                    faults, PWM, speed, encoder position and I2C error count
  0x06 ARM_INFO     master -> PC, 1 Hz: per motor the slave's calibration (max rpm, encoder counts per rev) and version
I2C frames (fixed size, CRC-16 like the serial packets):
  0x10 slave command  master -> slave: seq, flags (bit 0 = e-stop), target in 0.1 rpm
  0x11 slave status   slave -> master: counter, state, faults, joint, PWM, speed, position, counters, calibration
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

PROTOCOL_VERSION = 1
TYPE_ARM_STATUS = 0x05
TYPE_ARM_INFO = 0x06
TYPE_ARM_COMMAND = 0x83
I2C_TYPE_COMMAND = 0x10
I2C_TYPE_STATUS = 0x11
FLAG_ESTOP = 0x01
STATUS_FLAG_SLAVES_READY = 0x01
MAX_PACKET = 96  # decoded bytes: header + payload + CRC
MOTORS = ('shoulder', 'elbow', 'wrist_a', 'wrist_b')

# States of the master and of each slave (firmware_arm/lib/arm_core/src/arm_core.h).
BOOT, NO_COMMAND, ESTOP, WAIT_NEUTRAL, ENABLED, FAULT, WAIT_SLAVES = range(7)
STATE_NAMES = {BOOT: 'BOOT', NO_COMMAND: 'NO_COMMAND', ESTOP: 'ESTOP', WAIT_NEUTRAL: 'WAIT_NEUTRAL', ENABLED: 'ENABLED',
               FAULT: 'FAULT', WAIT_SLAVES: 'WAIT_SLAVES'}
# Faults a slave detects itself.
SLAVE_FAULT_STALL, SLAVE_FAULT_DIRECTION = 0x01, 0x02
SLAVE_FAULT_NAMES = {SLAVE_FAULT_STALL: 'stalled', SLAVE_FAULT_DIRECTION: 'turning against its PWM (encoder or motor reversed)'}
# Faults the master sees on a slave's link.
LINK_LOST, LINK_WRONG_JOINT, LINK_NOT_ENABLED, LINK_SLAVE_FAULT = 0x01, 0x02, 0x04, 0x08
LINK_FAULT_NAMES = {LINK_LOST: 'not answering on I2C', LINK_WRONG_JOINT: 'built for another joint',
                    LINK_NOT_ENABLED: 'did not follow the master', LINK_SLAVE_FAULT: 'reports a fault'}

_COMMAND = struct.Struct('<B4h')
_STATUS_HEAD = struct.Struct('<BBHHHIB3B')
_MOTOR_STATUS = struct.Struct('<BBBhhiH')
_MOTOR_INFO = struct.Struct('<HI3BB')
_I2C_COMMAND = struct.Struct('<BBBBh')
_I2C_STATUS = struct.Struct('<BBBBBBhhiHHHI3BB')
STATUS_PAYLOAD = _STATUS_HEAD.size + 4 * _MOTOR_STATUS.size
INFO_PAYLOAD = 4 * _MOTOR_INFO.size
I2C_COMMAND_SIZE = _I2C_COMMAND.size + 2
I2C_STATUS_SIZE = _I2C_STATUS.size + 2


def describe_faults(bits: int, names: dict[int, str]) -> str:
    return ', '.join(text for bit, text in names.items() if bits & bit) or 'none'


def state_name(state: int) -> str:
    return STATE_NAMES.get(state, f'UNKNOWN({state})')


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF); crc16(b'123456789') == 0x29B1."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_index, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index, code = len(out), 1
                out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        if code == 0 or index + code - 1 > len(data) or 0 in data[index:index + code - 1]:
            return None
        out += data[index:index + code - 1]
        index += code - 1
        if code != 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def encode_packet(packet_type: int, seq: int, payload: bytes) -> bytes:
    """Complete wire bytes for one packet, including the trailing 0x00 delimiter."""
    body = bytes([PROTOCOL_VERSION, packet_type & 0xFF, seq & 0xFF]) + payload
    if len(body) + 2 > MAX_PACKET:
        raise ValueError('packet too long')
    return cobs_encode(body + struct.pack('<H', crc16(body))) + b'\x00'


@dataclass(frozen=True)
class Packet:
    type: int
    seq: int
    payload: bytes


class PacketDecoder:
    """Feed arbitrary byte chunks; get back complete packets that pass COBS, CRC and version checks."""

    def __init__(self) -> None:
        self.good = 0
        self.bad = 0
        self._buffer = bytearray()
        self._overflow = False
        self._first = True

    def feed(self, chunk: bytes) -> list[Packet]:
        packets = []
        for byte in chunk:
            if byte:
                if len(self._buffer) <= MAX_PACKET:
                    self._buffer.append(byte)
                else:
                    self._overflow = True
                continue
            block, overflow, first = bytes(self._buffer), self._overflow, self._first
            self._buffer, self._overflow, self._first = bytearray(), False, False
            if not block:
                continue
            packet = None if overflow else self._decode(block)
            if packet is not None:
                self.good += 1
                packets.append(packet)
            elif not first:  # the first block may be the tail of a packet we joined mid-stream
                self.bad += 1
        return packets

    @staticmethod
    def _decode(block: bytes) -> Packet | None:
        decoded = cobs_decode(block)
        if decoded is None or len(decoded) < 5:
            return None
        body, crc = decoded[:-2], struct.unpack('<H', decoded[-2:])[0]
        if crc16(body) != crc or body[0] != PROTOCOL_VERSION:
            return None
        return Packet(type=body[1], seq=body[2], payload=bytes(body[3:]))


def _decirpm(rpm: float) -> int:
    return max(-32767, min(32767, round(rpm * 10))) if math.isfinite(rpm) else 0


# ------------------------------------------------------------------------------------------------ ARM_COMMAND
def arm_command_payload(estop: bool, targets_rpm: list[float] | tuple[float, ...]) -> bytes:
    """Payload of ARM_COMMAND: gearmotor targets (shoulder, elbow, wrist_a, wrist_b) in rpm, 0.1 rpm resolution.
    A non-finite target is sent as 0: the driver's 50 Hz loop must never fail on a bad value."""
    if len(targets_rpm) != 4:
        raise ValueError('need four motor targets (shoulder, elbow, wrist_a, wrist_b)')
    return _COMMAND.pack(FLAG_ESTOP if estop else 0, *(_decirpm(rpm) for rpm in targets_rpm))


def parse_arm_command(payload: bytes) -> tuple[bool, tuple[float, ...]] | None:
    """(estop, targets rpm), or None if the length is wrong or a reserved flag bit is set (as the firmware does)."""
    if len(payload) != _COMMAND.size:
        return None
    flags, *raw = _COMMAND.unpack(payload)
    if flags & ~FLAG_ESTOP:
        return None
    return bool(flags & FLAG_ESTOP), tuple(value / 10 for value in raw)


# ------------------------------------------------------------------------------------------------ ARM_STATUS
@dataclass(frozen=True)
class MotorStatus:
    slave_state: int
    slave_faults: int  # SLAVE_FAULT_* bits (latched by the master while it is in FAULT)
    link_faults: int  # LINK_* bits
    pwm: int  # applied duty -255..255
    velocity_rpm: float  # measured gearmotor output speed
    position: int  # encoder counts since the slave powered up
    i2c_errors: int  # exchanges with this slave that failed (no answer or a bad frame)

    @property
    def state_name(self) -> str:
        return state_name(self.slave_state)


@dataclass(frozen=True)
class ArmStatus:
    state: int
    flags: int
    command_age_ms: int
    rx_good: int
    rx_bad: int
    uptime_ms: int
    reset_reason: int  # esp_reset_reason_t of the master's last boot
    firmware_version: tuple[int, int, int]
    motors: tuple[MotorStatus, MotorStatus, MotorStatus, MotorStatus]

    @property
    def state_name(self) -> str:
        return state_name(self.state)

    @property
    def slaves_ready(self) -> bool:
        return bool(self.flags & STATUS_FLAG_SLAVES_READY)


def arm_status_payload(status: ArmStatus) -> bytes:
    head = _STATUS_HEAD.pack(status.state, status.flags, min(status.command_age_ms, 0xFFFF), min(status.rx_good, 0xFFFF),
                             min(status.rx_bad, 0xFFFF), status.uptime_ms & 0xFFFFFFFF, status.reset_reason,
                             *status.firmware_version)
    return head + b''.join(_MOTOR_STATUS.pack(m.slave_state, m.slave_faults, m.link_faults, m.pwm, _decirpm(m.velocity_rpm),
                                              m.position, min(m.i2c_errors, 0xFFFF)) for m in status.motors)


def parse_arm_status(payload: bytes) -> ArmStatus | None:
    if len(payload) != STATUS_PAYLOAD:
        return None
    state, flags, age, good, bad, uptime, reset, major, minor, patch = _STATUS_HEAD.unpack_from(payload)
    motors = []
    for index in range(4):
        slave_state, slave_faults, link_faults, pwm, velocity, position, errors = _MOTOR_STATUS.unpack_from(
            payload, _STATUS_HEAD.size + index * _MOTOR_STATUS.size)
        motors.append(MotorStatus(slave_state, slave_faults, link_faults, pwm, velocity / 10, position, errors))
    return ArmStatus(state, flags, age, good, bad, uptime, reset, (major, minor, patch), tuple(motors))  # type: ignore[arg-type]


# ------------------------------------------------------------------------------------------------ ARM_INFO
@dataclass(frozen=True)
class MotorInfo:
    max_rpm: float  # the slave's open-loop calibration: PWM 255 == this gearmotor rpm (0 = not heard from yet)
    counts_per_rev: float  # encoder counts per gearmotor output revolution
    firmware_version: tuple[int, int, int]
    reset_reason: int

    @property
    def known(self) -> bool:
        return self.counts_per_rev > 0


def arm_info_payload(motors: tuple[MotorInfo, ...] | list[MotorInfo]) -> bytes:
    return b''.join(_MOTOR_INFO.pack(round(m.max_rpm * 10), round(m.counts_per_rev * 100), *m.firmware_version, m.reset_reason)
                    for m in motors)


def parse_arm_info(payload: bytes) -> tuple[MotorInfo, ...] | None:
    if len(payload) != INFO_PAYLOAD:
        return None
    motors = []
    for index in range(4):
        max_rpm_x10, cpr_x100, major, minor, patch, reset = _MOTOR_INFO.unpack_from(payload, index * _MOTOR_INFO.size)
        motors.append(MotorInfo(max_rpm_x10 / 10, cpr_x100 / 100, (major, minor, patch), reset))
    return tuple(motors)


# ------------------------------------------------------------------------------------------------ I2C frames
@dataclass(frozen=True)
class SlaveCommand:
    seq: int
    estop: bool
    target_rpm: float


@dataclass(frozen=True)
class SlaveStatus:
    counter: int
    state: int
    faults: int
    joint: int
    pwm: int
    velocity_rpm: float
    position: int
    rx_bad: int
    command_age_ms: int
    max_rpm: float
    counts_per_rev: float
    firmware_version: tuple[int, int, int]
    reset_reason: int


def _with_crc(body: bytes) -> bytes:
    return body + struct.pack('<H', crc16(body))


def _crc_ok(frame: bytes) -> bool:
    return crc16(frame[:-2]) == struct.unpack('<H', frame[-2:])[0]


def slave_command_frame(command: SlaveCommand) -> bytes:
    return _with_crc(_I2C_COMMAND.pack(PROTOCOL_VERSION, I2C_TYPE_COMMAND, command.seq & 0xFF,
                                       FLAG_ESTOP if command.estop else 0, _decirpm(command.target_rpm)))


def parse_slave_command_frame(frame: bytes) -> SlaveCommand | None:
    if len(frame) != I2C_COMMAND_SIZE or not _crc_ok(frame):
        return None
    version, kind, seq, flags, target = _I2C_COMMAND.unpack_from(frame)
    if version != PROTOCOL_VERSION or kind != I2C_TYPE_COMMAND or flags & ~FLAG_ESTOP:
        return None
    return SlaveCommand(seq, bool(flags & FLAG_ESTOP), target / 10)


def slave_status_frame(status: SlaveStatus) -> bytes:
    return _with_crc(_I2C_STATUS.pack(PROTOCOL_VERSION, I2C_TYPE_STATUS, status.counter & 0xFF, status.state, status.faults,
                                      status.joint, status.pwm, _decirpm(status.velocity_rpm), status.position,
                                      min(status.rx_bad, 0xFFFF), min(status.command_age_ms, 0xFFFF),
                                      round(status.max_rpm * 10), round(status.counts_per_rev * 100),
                                      *status.firmware_version, status.reset_reason))


def parse_slave_status_frame(frame: bytes) -> SlaveStatus | None:
    if len(frame) != I2C_STATUS_SIZE or not _crc_ok(frame):
        return None
    (version, kind, counter, state, faults, joint, pwm, velocity, position, rx_bad, age, max_rpm_x10, cpr_x100,
     major, minor, patch, reset) = _I2C_STATUS.unpack_from(frame)
    if version != PROTOCOL_VERSION or kind != I2C_TYPE_STATUS or state not in STATE_NAMES or joint >= 4:
        return None
    return SlaveStatus(counter, state, faults, joint, pwm, velocity / 10, position, rx_bad, age, max_rpm_x10 / 10,
                       cpr_x100 / 100, (major, minor, patch), reset)
