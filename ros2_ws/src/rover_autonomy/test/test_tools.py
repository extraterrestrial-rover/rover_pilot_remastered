"""The repository's tools for the autonomy stack that are not Python (tools/record_run.sh)."""
import os
import re
import subprocess
from pathlib import Path

import yaml

from rover_autonomy import stack_config

REPO = Path(__file__).resolve().parents[4]
PACKAGE = Path(__file__).resolve().parents[1]
RECORDER = REPO / 'tools' / 'record_run.sh'


def recorded_topics() -> set[str]:
    return set(re.findall(r'(?<![\w.~])/[a-z_]+(?:/[a-z_]+)*', RECORDER.read_text().split('TOPICS=(')[1].split('PACKAGES=')[0]))


def test_the_recorder_is_a_runnable_script():
    assert os.access(RECORDER, os.X_OK)
    assert subprocess.run(['bash', '-n', str(RECORDER)], capture_output=True, text=True).returncode == 0
    usage = subprocess.run(['bash', str(RECORDER), '--help'], capture_output=True, text=True)
    assert usage.returncode == 0 and 'record_run.sh [--cloud] [name]' in usage.stdout
    assert subprocess.run(['bash', str(RECORDER), '--nonsense'], capture_output=True, text=True).returncode == 2


def test_the_recorder_takes_what_a_failed_run_is_judged_by():
    """A run that went wrong on the rover left only its console log: not what Nav2 commanded, what the mux sent on,
    where the rover was, or what the costmaps held."""
    topics = recorded_topics()
    mux = yaml.safe_load((PACKAGE / 'config' / 'autonomy.yaml').read_text())
    settings = mux['/**']['ros__parameters']
    assert {'/tf', '/tf_static', '/rover/autonomy_events', '/rover/auto_fault', '/rover/goal_active'} <= topics
    assert {'/cmd_vel', '/cmd_vel_nav', '/cmd_vel_auto', '/cmd_vel_drive'} <= topics  # every stage of a command
    assert {'/odometry/filtered', '/zed/odom_base', settings['zed_odom_topic'], settings['zed_status_topic']} <= topics
    assert {stack_config.OBSTACLES_TOPIC, stack_config.GROUND_TOPIC, '/map', '/plan', '/local_plan'} <= topics
    for costmap in ('local', 'global'):
        assert {f'/{costmap}_costmap/costmap', f'/{costmap}_costmap/published_footprint'} <= topics
    assert settings['zed_cloud_topic'] in topics  # only with --cloud: it is about 1 GB a minute
    assert f'CLOUD={settings["zed_cloud_topic"]}' in RECORDER.read_text()


def test_rviz_shows_what_nav2_takes_for_obstacles():
    """The raw point cloud is 18 MB/s, too much for the radio, so it is off by default. The detector's obstacle cloud
    is a few hundred points: it shows at a glance why Nav2 goes round something, or does not."""
    displays = yaml.safe_load((PACKAGE / 'rviz' / 'base_station.rviz').read_text())['Visualization Manager']['Displays']
    clouds = {display['Topic']['Value']: display for display in displays if display['Class'].endswith('PointCloud2')}
    assert clouds[stack_config.OBSTACLES_TOPIC]['Enabled'] is True
    assert clouds[stack_config.OBSTACLES_TOPIC]['Topic']['Reliability Policy'] == 'Best Effort'  # as the detector sends
    settings = yaml.safe_load((PACKAGE / 'config' / 'autonomy.yaml').read_text())['/**']['ros__parameters']
    assert clouds[settings['zed_cloud_topic']]['Enabled'] is False
