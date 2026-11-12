"""UDP link to the drive ESP32's W5500 (LINK.md): the same packets as over USB, carried in datagrams. No ROS needed.

The ESP32 listens on 192.168.77.2:47700 and sends DRIVE_STATUS to the PC's port 47701 (the addresses of the rover's
CAN master, telemetry_dashboard protocol/ETHERNET_LINK.md). Each datagram carries complete packets, each ending in
0x00; a cut-off tail is rejected, and only datagrams from the ESP32's address are accepted. Nothing here blocks.

With `interface` set (the PC port the ESP32 is plugged into), Linux sends only through that port (IP_UNICAST_IF, no
privileges needed): when the cable is out, NetworkManager removes the port's address, and without the pin the
commands would follow the default route out of the radio port. The port's carrier (cable in, other end powered) is
reported too, which tells "cable out" apart from "ESP32 silent".
"""
from __future__ import annotations

import socket
import struct
import sys
import time
from pathlib import Path

from rover_pilot_remastered import protocol

DEFAULT_MASTER = ('192.168.77.2', 47700)
DEFAULT_LOCAL_PORT = 47701
IP_UNICAST_IF = getattr(socket, 'IP_UNICAST_IF', 50)  # Linux <linux/in.h>; not every Python exports it
CARRIER_CHECK_S = 0.5
MAX_DATAGRAMS_PER_CALL = 64  # the ESP32 sends 10 a second; more than this in one call is a flood or a failing socket


class UdpLink:
    def __init__(self, master_host: str = DEFAULT_MASTER[0], master_port: int = DEFAULT_MASTER[1],
                 local_port: int = DEFAULT_LOCAL_PORT, interface: str = '', sys_class_net: Path = Path('/sys/class/net')) -> None:
        self.master_host, self.master_port, self.local_port = master_host, master_port, local_port
        self.interface = interface or None
        self.decoder = protocol.PacketDecoder()
        self.datagrams = 0     # accepted from the ESP32
        self.foreign = 0       # datagrams from any other address: dropped
        self.tails = 0         # cut-off packets at the end of a datagram: dropped
        self.send_errors = 0   # sends the OS refused (no route, port down, buffer full)
        self.receive_errors = 0  # receives the OS failed (an ICMP error for an earlier send, the port going down)
        self.pin_error: str | None = None
        self._sock: socket.socket | None = None
        self._master_ip: str | None = None
        self._sys_class_net = sys_class_net
        self._pinned_index: int | None = None
        self._carrier: bool | None = None
        self._carrier_checked: float | None = None

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    @property
    def address(self) -> tuple[str, int]:
        """The local (address, port) the link is bound to; port 0 before open()."""
        return self._sock.getsockname() if self._sock else ('', self.local_port)

    @property
    def bad(self) -> int:
        """Everything rejected: undecodable packets, cut-off tails, datagrams from elsewhere."""
        return self.decoder.bad + self.foreign + self.tails

    def open(self) -> None:
        """Binds the PC's port on every address, so it works while the cable is out and the port has no address.
        Raises OSError, for example EADDRINUSE when another program (the CAN bridge, a second drive_node) has it."""
        self._master_ip = socket.gethostbyname(self.master_host)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        try:
            sock.bind(('', self.local_port))
        except OSError:
            sock.close()
            raise
        self._sock, self._pinned_index, self._carrier_checked = sock, None, None
        self.carrier()  # pins the interface at once when it is there

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def send(self, wire: bytes) -> bool:
        """One packet per datagram to the ESP32; False (and counted) if the OS refused it."""
        if self._sock is None:
            return False
        try:
            self._sock.sendto(wire, (self._master_ip, self.master_port))
            return True
        except OSError:  # ENETUNREACH / EHOSTUNREACH with the port down, ENOBUFS, EAGAIN, ...
            self.send_errors += 1
            return False

    def receive(self) -> list[protocol.Packet]:
        """The packets that have arrived since the last call. One call reads at most MAX_DATAGRAMS_PER_CALL
        datagrams, so neither a flood nor a socket that keeps failing can hold up the caller's 50 Hz loop."""
        packets: list[protocol.Packet] = []
        for _ in range(MAX_DATAGRAMS_PER_CALL):
            if self._sock is None:
                break
            try:
                data, (host, _port) = self._sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:  # usually an ICMP error reported for an earlier send: nothing to read
                self.receive_errors += 1
                continue
            if host != self._master_ip:
                self.foreign += 1
                continue
            self.datagrams += 1
            end = data.rfind(b'\x00') + 1  # complete packets end at the datagram's last 0x00
            if end < len(data):
                self.tails += 1
            packets += self.decoder.feed(data[:end])
        return packets

    def carrier(self) -> bool | None:
        """The interface's carrier (None if no interface is named, or not on Linux); re-pins sends when it comes back
        with a new index, as a re-plugged USB adapter does."""
        if self.interface is None or not self._sys_class_net.is_dir():
            return None
        now = time.monotonic()
        if self._carrier_checked is None or now - self._carrier_checked >= CARRIER_CHECK_S:
            self._carrier_checked = now
            try:
                self._carrier = (self._sys_class_net / self.interface / 'carrier').read_text().strip() == '1'
            except OSError:  # interface missing (adapter unplugged) or down (reading carrier fails with EINVAL)
                self._carrier = False
            self._pin_interface()
        return self._carrier

    def _pin_interface(self) -> None:
        if self.interface is None or self._sock is None or not sys.platform.startswith('linux'):
            return
        try:
            index = socket.if_nametoindex(self.interface)
        except OSError:
            return  # not there now (adapter unplugged); checked again with the carrier
        if index == self._pinned_index:
            return
        try:
            self._sock.setsockopt(socket.IPPROTO_IP, IP_UNICAST_IF, struct.pack('!I', index))
        except OSError as error:
            self.pin_error = str(error)
            return
        self._pinned_index, self.pin_error = index, None

    @property
    def pinned(self) -> bool:
        return self._pinned_index is not None
