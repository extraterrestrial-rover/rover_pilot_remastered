import sys
from pathlib import Path

# The package itself, and rover_pilot_remastered next to it (kinematics), without installing either.
SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'rover_autonomy'))
sys.path.insert(0, str(SRC / 'rover_pilot_remastered'))


import os  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture(scope='session')
def ros_domain() -> int:
    """ROS tests run in a ROS domain of their own (rover_pilot_remastered's ros_sandbox), away from any real rover."""
    from rover_pilot_remastered import ros_sandbox  # noqa: PLC0415
    return ros_sandbox.enter(int(os.environ.get('ROVER_TEST_DOMAIN', ros_sandbox.TEST_DOMAIN)))
