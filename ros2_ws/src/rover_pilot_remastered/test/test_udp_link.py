"""The UDP link's datagram rules, source check, port sharing and interface pin (udp_link.py; LINK.md)."""
import errno
import socket
import sys

import pytest

from rover_pilot_remastered import protocol
from rover_pilot_remastered.udp_link import UdpLink

STATUS = protocol.encode_packet(protocol.TYPE_DRIVE_STATUS, 7, protocol.drive_status_payload(
    protocol.DriveStatus(4, 12, (10, 20, 30, 40), 100, 0, 5000, 1, 94.7, (3, 0, 0))))
COMMAND = protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, 1, protocol.drive_command_payload(False, (10.0, 10.0, 10.0, 10.0)))


@pytest.fixture
def esp():
    """The ESP32's side: a plain UDP socket on the loopback."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.settimeout(1.0)
    yield sock
    sock.close()


def link_to(esp_sock, master_host='127.0.0.1'):
    link = UdpLink(master_host, esp_sock.getsockname()[1], local_port=0)
    link.open()
    return link


def wait_packets(link, count=1):
    import time  # noqa: PLC0415
    packets, end = [], time.monotonic() + 1.0
    while len(packets) < count and time.monotonic() < end:
        packets += link.receive()
        time.sleep(0.005)
    return packets


def test_status_from_the_esp32_is_decoded_and_commands_reach_it(esp):
    link = link_to(esp)
    try:
        assert link.send(COMMAND)
        data, _ = esp.recvfrom(2048)
        assert data == COMMAND  # one packet per datagram, unchanged
        esp.sendto(STATUS, link.address if link.address[0] != '0.0.0.0' else ('127.0.0.1', link.address[1]))
        packets = wait_packets(link)
        assert [p.seq for p in packets] == [7]
        assert protocol.parse_drive_status(packets[0].payload).firmware_version == (3, 0, 0)
        assert (link.datagrams, link.bad) == (1, 0)
    finally:
        link.close()


def test_datagrams_from_another_address_are_dropped_and_counted(esp):
    link = link_to(esp, master_host='127.0.0.2')  # expects the ESP32 elsewhere: 127.0.0.1 is a stranger now
    try:
        esp.sendto(STATUS, ('127.0.0.1', link.address[1]))
        import time  # noqa: PLC0415
        time.sleep(0.05)
        assert link.receive() == []
        assert (link.foreign, link.bad) == (1, 1)
    finally:
        link.close()


def test_several_packets_per_datagram_and_a_cut_off_tail(esp):
    link = link_to(esp)
    target = ('127.0.0.1', link.address[1])
    try:
        esp.sendto(STATUS + STATUS, target)
        assert len(wait_packets(link, 2)) == 2
        esp.sendto(STATUS[:-1], target)  # its 0x00 is missing: rejected, not completed by the next datagram
        esp.sendto(STATUS, target)
        assert len(wait_packets(link)) == 1
        assert (link.tails, link.bad) == (1, 1)
    finally:
        link.close()


def test_only_one_program_may_hold_the_pc_port(esp):
    first = link_to(esp)
    try:
        second = UdpLink('127.0.0.1', esp.getsockname()[1], local_port=first.address[1])
        with pytest.raises(OSError) as raised:
            second.open()
        assert raised.value.errno == errno.EADDRINUSE  # drive_node reports this as "the CAN bridge has it"
        assert not second.is_open and not second.send(COMMAND)
    finally:
        first.close()


def test_carrier_comes_from_sys_class_net(tmp_path):
    (tmp_path / 'enx1').mkdir()
    (tmp_path / 'enx1' / 'carrier').write_text('0\n')
    assert UdpLink(interface='enx1', sys_class_net=tmp_path).carrier() is False
    (tmp_path / 'enx1' / 'carrier').write_text('1\n')
    assert UdpLink(interface='enx1', sys_class_net=tmp_path).carrier() is True
    assert UdpLink(interface='gone', sys_class_net=tmp_path).carrier() is False  # adapter unplugged
    assert UdpLink(sys_class_net=tmp_path).carrier() is None  # no interface named


@pytest.mark.skipif(not sys.platform.startswith('linux'), reason='IP_UNICAST_IF is Linux-only')
def test_sends_are_pinned_to_the_named_interface_on_linux(esp):
    link = UdpLink('127.0.0.1', esp.getsockname()[1], local_port=0, interface='lo')
    link.open()
    try:
        assert link.pinned and link.pin_error is None
        assert link.carrier() is True
        assert link.send(COMMAND)
        assert esp.recvfrom(2048)[0] == COMMAND  # the loopback is the pinned interface here
    finally:
        link.close()


class Socket:
    """Stands in for the UDP socket: every recvfrom() gives `answer`, or raises it."""

    def __init__(self, answer):
        self.answer, self.calls = answer, 0

    def recvfrom(self, _size):
        self.calls += 1
        if isinstance(self.answer, Exception):
            raise self.answer
        return self.answer

    def close(self):
        pass


def test_a_socket_that_keeps_failing_cannot_hang_the_caller():
    link = UdpLink('127.0.0.1', 47700, local_port=0)
    link._sock, link._master_ip = Socket(OSError(errno.ENETDOWN, 'Network is down')), '127.0.0.1'
    assert link.receive() == []  # used to loop for ever: drive_node's 50 Hz tick never returned
    assert link.receive_errors == link._sock.calls > 0
    link._sock = Socket(BlockingIOError())
    assert link.receive() == [] and link._sock.calls == 1  # the normal case: nothing waiting, one call


def test_a_flood_of_datagrams_is_read_in_bounded_portions():
    link = UdpLink('127.0.0.1', 47700, local_port=0)
    link._sock, link._master_ip = Socket((STATUS, ('127.0.0.1', 47700))), '127.0.0.1'
    first = link.receive()
    assert 0 < len(first) == link._sock.calls  # one packet per datagram, and it came back
    assert len(link.receive()) == len(first)  # the rest is read by the next calls
