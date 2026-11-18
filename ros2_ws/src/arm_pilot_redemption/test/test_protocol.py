import random
import subprocess
import sys
from pathlib import Path

from arm_pilot_redemption import protocol


def test_crc_matches_the_rover_framing(vectors):
    assert protocol.crc16(b'123456789') == vectors['crc_check']['crc'] == 0x29B1
    # PC_HEARTBEAT from the telemetry repo's C reference implementation: the rover's framing, byte for byte.
    assert protocol.encode_packet(0x81, 0, b'\x01\x00').hex() == '030181020103e02300'


def test_commands_match_the_shared_vectors(vectors):
    decoder = protocol.PacketDecoder()
    decoder.feed(b'\x00')
    for vector in vectors['commands']:
        payload = protocol.arm_command_payload(vector['estop'], vector['targets_rpm'])
        assert protocol.encode_packet(protocol.TYPE_ARM_COMMAND, vector['seq'], payload).hex() == vector['wire']
        (packet,) = decoder.feed(bytes.fromhex(vector['wire']))
        estop, targets = protocol.parse_arm_command(packet.payload)
        assert estop == vector['estop'] and list(targets) == vector['targets_rpm']
    assert decoder.bad == 0


def test_status_and_info_parse_from_the_shared_vectors(vectors):
    (packet,) = protocol.PacketDecoder().feed(b'\x00' + bytes.fromhex(vectors['status']['wire']))
    status = protocol.parse_arm_status(packet.payload)
    assert status.state_name == 'FAULT' and not status.slaves_ready and status.firmware_version == (1, 0, 0)
    shoulder, elbow, wrist_a, wrist_b = status.motors
    assert (shoulder.pwm, shoulder.velocity_rpm, shoulder.position) == (128, 58.5, 100000)
    assert (elbow.pwm, elbow.velocity_rpm, elbow.i2c_errors) == (-255, -117.0, 3)
    assert wrist_a.state_name == 'FAULT' and wrist_a.slave_faults == protocol.SLAVE_FAULT_STALL and wrist_a.position == -2000000
    assert wrist_b.link_faults == protocol.LINK_LOST and wrist_b.i2c_errors == 65535
    assert protocol.arm_status_payload(status).hex() == vectors['status']['payload']

    (packet,) = protocol.PacketDecoder().feed(b'\x00' + bytes.fromhex(vectors['info']['wire']))
    info = protocol.parse_arm_info(packet.payload)
    assert info[0].max_rpm == 117.0 and info[0].counts_per_rev == 1425.1 and info[0].known
    assert info[2].max_rpm == 312.0 and info[2].counts_per_rev == 537.7
    assert not info[3].known


def test_i2c_frames_match_the_shared_vectors(vectors):
    for vector in vectors['slave_commands']:
        command = protocol.parse_slave_command_frame(bytes.fromhex(vector['frame']))
        assert command == protocol.SlaveCommand(vector['seq'], vector['estop'], vector['target_rpm'])
        assert protocol.slave_command_frame(command).hex() == vector['frame']
    frame = bytes.fromhex(vectors['slave_status']['frame'])
    assert len(frame) == protocol.I2C_STATUS_SIZE <= 32  # the ESP32's I2C slave TX FIFO
    status = protocol.parse_slave_status_frame(frame)
    assert (status.joint, status.position, status.counts_per_rev, status.velocity_rpm) == (2, -123456, 1425.1, -58.5)
    assert protocol.slave_status_frame(status) == frame


def test_corrupted_or_empty_i2c_frames_are_rejected(vectors):
    frame = bytearray.fromhex(vectors['slave_status']['frame'])
    frame[11] ^= 0x01
    assert protocol.parse_slave_status_frame(bytes(frame)) is None
    assert protocol.parse_slave_status_frame(b'\xff' * protocol.I2C_STATUS_SIZE) is None  # nobody drives the bus
    assert protocol.parse_slave_status_frame(bytes(protocol.I2C_STATUS_SIZE)) is None
    assert protocol.parse_slave_command_frame(bytes.fromhex(vectors['slave_commands'][1]['frame'])[:-1]) is None


def test_noise_and_split_chunks_resynchronise(vectors):
    wires = [bytes.fromhex(v['wire']) for v in vectors['commands']]
    stream = b'arm\r\n\x13\x00' + b''.join(wires) + b'\x05\x05\x05\x00' + wires[1]
    decoder, packets, rng, index = protocol.PacketDecoder(), [], random.Random(1), 0
    while index < len(stream):
        size = rng.randrange(1, 7)
        packets += decoder.feed(stream[index:index + size])
        index += size
    assert len(packets) == len(wires) + 1
    assert decoder.bad == 1


def test_targets_are_clamped_and_non_finite_targets_sent_as_zero():
    assert protocol.parse_arm_command(protocol.arm_command_payload(False, (1e9, -1e9, 0.04, -0.06)))[1] == (3276.7, -3276.7, 0.0, -0.1)
    assert protocol.parse_arm_command(protocol.arm_command_payload(False, (float('nan'), float('inf'), 1.0, 0.0)))[1] == (0.0, 0.0, 1.0, 0.0)


def test_reserved_flags_and_rover_commands_are_rejected():
    assert protocol.parse_arm_command(protocol.arm_command_payload(True, (0.0,) * 4)) == (True, (0.0,) * 4)
    assert protocol.parse_arm_command(b'\x02' + bytes(8)) is None
    assert protocol.parse_arm_command(bytes(8)) is None
    assert protocol.TYPE_ARM_COMMAND != 0x82  # the rover's DRIVE_COMMAND


def test_vector_files_are_up_to_date():
    root = Path(__file__).resolve().parents[4]
    script = root / 'tools' / 'arm_gen_test_vectors.py'
    if script.exists():  # only inside the full repository checkout
        result = subprocess.run([sys.executable, str(script), '--check'], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
