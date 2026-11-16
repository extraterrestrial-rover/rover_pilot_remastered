"""The launch's checks of the installed ROS packages."""
import pytest

from rover_autonomy.system_checks import package_version, tf2_problem


def test_the_tf2_release_that_deadlocks_nav2_is_reported():
    """On the rover Nav2's controller steered from a 70 s old position: its costmap's transform listener and its
    point cloud callback had locked each other (tf2 0.25.23)."""
    problem = tf2_problem('0.25.23', '0.25.23')
    assert '0.25.23' in problem and 'deadlock' in problem
    assert "apt install --only-upgrade 'ros-humble-tf2*'" in problem  # both packages, see the next test


@pytest.mark.parametrize('tf2, tf2_ros', [('0.25.22', '0.25.22'), ('0.25.20', '0.25.20'), ('0.25.24', '0.25.24'),
                                           ('0.25.25', '0.25.25')])
def test_the_releases_before_and_after_it_are_fine(tf2, tf2_ros):
    assert tf2_problem(tf2, tf2_ros) == ''


def test_upgrading_tf2_ros_alone_does_not_fix_it():
    """The fix is in tf2 (it calls back without its lock); tf2_ros 0.25.24 still holds its own lock while it asks."""
    assert tf2_problem('0.25.23', '0.25.24') != ''
    assert tf2_problem('0.25.24', '0.25.23') == ''  # tf2 fixed: tf2_ros' lock no longer meets tf2's


@pytest.mark.parametrize('tf2, tf2_ros', [('', ''), ('0.25.23', ''), ('0.36.4', '0.36.4'), ('0.25', '0.25.23'),
                                           ('0.25.x', '0.25.23')])
def test_what_cannot_be_judged_is_not_reported(tf2, tf2_ros):
    assert tf2_problem(tf2, tf2_ros) == ''


def test_a_package_xml_gives_its_version():
    assert package_version('<package format="3">\n  <name>tf2_ros</name>\n  <version>0.25.23</version>\n</package>') == '0.25.23'
    assert package_version('<package/>') == ''
