import json
import os
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rover_pilot_remastered import ros_sandbox  # noqa: E402


@pytest.fixture(scope='session', autouse=True)
def ros_domain() -> int:
    """Every test, and every node a test starts, runs in a ROS domain of its own on this machine (ros_sandbox):
    the ROS tests release the e-stop and command full speed, which must never reach a real rover."""
    return ros_sandbox.enter(int(os.environ.get('ROVER_TEST_DOMAIN', ros_sandbox.TEST_DOMAIN)))


@pytest.fixture(scope='session')
def vectors() -> dict:
    return json.loads((Path(__file__).parent / 'vectors.json').read_text())
