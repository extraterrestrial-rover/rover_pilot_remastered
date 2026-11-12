"""ros_sandbox: the tests and rehearsals never share a ROS domain with a real rover."""
import os

import pytest

from rover_pilot_remastered import ros_sandbox


def test_enter_sets_a_private_domain():
    environ = {'ROS_DOMAIN_ID': '5', 'OTHER': 'kept'}  # 5: the rover's, from the shell
    assert ros_sandbox.enter(42, environ) == 42
    # Not ROS_LOCALHOST_ONLY: with CycloneDDS a loopback-only domain holds about ten nodes, too few for the autonomy
    # stack (its nodes failed to start in the AUTO rehearsal).
    assert environ == {'ROS_DOMAIN_ID': '42', 'OTHER': 'kept'}


@pytest.mark.parametrize('bad', [-1, 102, 2.5, True, '7', None])
def test_enter_refuses_what_is_not_a_ros_domain(bad):
    environ = {'ROS_DOMAIN_ID': '5'}
    with pytest.raises(ValueError):
        ros_sandbox.enter(bad, environ)
    assert environ == {'ROS_DOMAIN_ID': '5'}  # untouched


def test_this_test_session_runs_in_the_sandbox(ros_domain):
    assert os.environ['ROS_DOMAIN_ID'] == str(ros_domain)


def test_other_nodes_are_reported_and_hidden_ones_are_not():
    class Node:
        def get_name(self):
            return 'rehearsal_ps5'

        def get_namespace(self):
            return '/'

        def get_node_names_and_namespaces(self):
            return [('rehearsal_ps5', '/'), ('drive_node', '/'), ('_ros2cli_daemon_63_abc', '/'), ('controller', '/arm')]

    assert ros_sandbox.foreign_nodes(Node(), settle_s=0.0) == ['/arm/controller', '/drive_node']
    assert 'drive_node' in ros_sandbox.refusal(63, ['/drive_node']) and '63' in ros_sandbox.refusal(63, ['/drive_node'])
