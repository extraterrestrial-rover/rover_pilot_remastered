"""The simulated ESP32 on UDP (fake_esp.Simulator): what the rehearsals and tests plug and unplug."""
import os
import socket

from rover_pilot_remastered import fake_esp


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def open_descriptors() -> int:
    return len(os.listdir('/dev/fd'))


def test_plugging_and_unplugging_leaks_no_file_descriptors():
    simulator = fake_esp.Simulator(port=free_port(), pc=('127.0.0.1', free_port()))
    try:
        before = open_descriptors()
        for _ in range(10):
            simulator.unplug()
            simulator.plug()
        assert open_descriptors() == before
        assert simulator.state() is not None  # the ESP32 keeps running on its USB power while the cable is out
    finally:
        simulator.close()


def test_close_frees_the_port_for_the_next_simulator():
    port, pc = free_port(), ('127.0.0.1', free_port())
    before = open_descriptors()
    fake_esp.Simulator(port=port, pc=pc).close()
    second = fake_esp.Simulator(port=port, pc=pc)  # binds the same port again
    try:
        assert second.launch_arguments == ['master:=127.0.0.1']
        assert second.tool_arguments == ['--master', '127.0.0.1', '--port', str(port), '--local-port', str(pc[1])]
    finally:
        second.close()
    assert open_descriptors() == before
