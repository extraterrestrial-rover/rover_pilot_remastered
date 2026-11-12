"""The simulated ESP32's pseudo-terminal (fake_esp.Simulator): what the rehearsals and tests plug and unplug."""
import os

import pytest

from rover_pilot_remastered import fake_esp

pytestmark = pytest.mark.skipif(not hasattr(os, 'openpty'), reason='needs POSIX pseudo-terminals')


def open_descriptors() -> int:
    return len(os.listdir('/dev/fd'))


def test_plugging_and_unplugging_leaks_no_file_descriptors():
    simulator = fake_esp.Simulator()
    try:
        before = open_descriptors()
        for _ in range(10):
            simulator.unplug()
            simulator.plug()
        assert open_descriptors() == before
    finally:
        simulator.close()


def test_each_simulator_has_a_path_of_its_own_and_cleans_it_up():
    first, second = fake_esp.Simulator(), fake_esp.Simulator()
    try:
        assert first.path != second.path
        assert os.path.exists(first.path) and os.path.exists(second.path)
        assert first.launch_arguments == [f'port:={first.path}'] and first.tool_arguments == [first.path]
    finally:
        first.close()
        second.close()
    assert not os.path.exists(first.path) and not os.path.exists(os.path.dirname(first.path))


def test_a_given_path_is_used_as_is(tmp_path):
    simulator = fake_esp.Simulator(str(tmp_path / 'rover-drive'))
    try:
        assert simulator.path == str(tmp_path / 'rover-drive') and os.path.exists(simulator.path)
    finally:
        simulator.close()
    assert tmp_path.exists()  # only its own link is removed
