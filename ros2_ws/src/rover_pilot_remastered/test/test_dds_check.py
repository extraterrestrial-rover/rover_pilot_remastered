"""dds_check: CycloneDDS must not send ROS data as multicast over the radio link."""
from rover_pilot_remastered.dds_check import multicast_problem

SPDP = '<CycloneDDS><Domain Id="any"><General><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'


def test_fast_dds_and_a_cyclone_config_with_spdp_are_fine(tmp_path):
    assert multicast_problem({}) == ''  # Fast DDS, the default: data unicast
    assert multicast_problem({'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}) == ''
    config = tmp_path / 'cyclonedds.xml'
    config.write_text(SPDP)
    for uri in (f'file://{config}', str(config), SPDP, f' file://{config} '):
        assert multicast_problem({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'CYCLONEDDS_URI': uri}) == '', uri
    config.write_text(SPDP.replace('spdp', ' FALSE '))
    assert multicast_problem({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'CYCLONEDDS_URI': f'file://{config}'}) == ''


def test_cyclone_without_the_setting_is_reported(tmp_path):
    # The rover laptop is wired to its radio, so CycloneDDS's default ("true" on a wired port) multicast the ZED's
    # point cloud (two readers: both costmaps), and the radios lost each other.
    assert 'no CycloneDDS config sets AllowMulticast' in multicast_problem({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'})
    pinned = tmp_path / 'pinned.xml'
    pinned.write_text('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General>'
                      '</Domain></CycloneDDS>')  # the Ethernet branch's interface pin alone
    assert 'no CycloneDDS config' in multicast_problem({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                                                        'CYCLONEDDS_URI': f'file://{pinned}'})
    for value in ('true', 'default', 'spdp,asm'):
        problem = multicast_problem({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'CYCLONEDDS_URI': SPDP.replace('spdp', value)})
        assert f"AllowMulticast is '{value}'" in problem and 'README' in problem
    assert 'no CycloneDDS config' in multicast_problem({'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                                                        'CYCLONEDDS_URI': 'file:///nowhere/cyclonedds.xml'})
