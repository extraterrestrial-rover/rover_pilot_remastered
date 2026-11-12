"""Keeps the tests and rehearsals away from the real rover.

They start real nodes, release the e-stop and command full speed, against a simulated ESP32. In the ROS domain the
rover uses, a drive_node that is really running would follow them and the rover would drive. So they run in a ROS
domain of their own, and refuse to start when anything else is already in that domain: discovery reaches the whole
network, so a rover that happens to use the same domain number is seen, and refused.

Not ROS_LOCALHOST_ONLY: with CycloneDDS it means the loopback without multicast, where a domain holds only about ten
nodes. The autonomy stack has more, and its nodes then fail to start ("Failed to find a free participant index").
"""
from __future__ import annotations

import os
import time

TEST_DOMAIN = 87  # pytest (the environment variable ROVER_TEST_DOMAIN overrides it)
REHEARSAL_DOMAIN = 63  # tools/rehearsal.py (--domain overrides it)


def enter(domain: int, environ=None) -> int:
    """Call before rclpy.init() and before starting any ROS process. This process and its children then use ROS
    domain `domain`."""
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 101:
        raise ValueError(f'ROS domain {domain!r}: use a whole number from 0 to 101')
    environ = os.environ if environ is None else environ
    environ['ROS_DOMAIN_ID'] = str(domain)
    return domain


def foreign_nodes(node, settle_s: float = 2.0) -> list[str]:
    """The other nodes in `node`'s ROS domain, after `settle_s` for discovery. Hidden nodes (names starting with an
    underscore, such as the ros2 command's daemon) do not count: they never drive."""
    time.sleep(settle_s)
    own = (node.get_name(), node.get_namespace())
    return sorted(f"{namespace.rstrip('/')}/{name}" for name, namespace in node.get_node_names_and_namespaces()
                  if (name, namespace) != own and not name.startswith('_'))


def refusal(domain: int, others: list[str]) -> str:
    return (f'ROS domain {domain} is already in use by {", ".join(others)}. This run releases the e-stop and commands '
            'full speed, so it must have a domain to itself: stop those nodes or pick another domain.')
