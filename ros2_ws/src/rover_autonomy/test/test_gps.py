"""The GPS: NMEA as the L89HA sends it, coordinates as people type them, and which fixes are passed on."""
import math
from pathlib import Path

import pytest

from rover_autonomy import geo, nmea
from rover_autonomy.gps_logic import GpsConfig, GpsState

CAPTURE = Path(__file__).parent / 'data' / 'l89ha_no_fix.nmea'  # 30 s from the rover's L89HA, indoors, no fix yet


def test_every_sentence_of_the_real_receiver_is_read():
    lines = CAPTURE.read_text().splitlines()
    parsed = [nmea.parse(line) for line in lines if line.strip()]
    assert all(parsed), [line for line, s in zip(lines, parsed) if s is None]
    kinds = {(s.talker, s.kind) for s in parsed}
    assert {('GN', 'RMC'), ('GN', 'GGA'), ('GN', 'GSA'), ('GP', 'GSV'), ('GA', 'GSV'), ('GI', 'GSV'),
            ('P', 'PQTMANTENNASTATUS'), ('P', 'PAIR010')} <= kinds


def test_without_a_fix_nothing_is_passed_on_and_the_status_says_why():
    state = GpsState()
    fixes = [state.feed(line, 1.0) for line in CAPTURE.read_text().splitlines()]
    assert not any(fixes) and state.problem(1.0) == 'no fix yet'
    assert state.bad_lines == 0 and state.sentences > 400
    # The default systems: GPS, Galileo and NavIC (the 4.11 system IDs 1, 3, 6 in GSA), and what was heard.
    assert set(state.used) == {'GPS', 'Galileo', 'NavIC'} and state.in_view['GPS'] == 7
    assert state.summary().startswith('GPS 0/7 (') and 'Galileo 0/0' in state.summary()
    assert state.antenna == '1,0,1'
    assert state.problem(5.0) == 'no position sentence for 4 s'


def test_cut_off_and_corrupted_lines_are_refused():
    good = nmea.gga_sentence(43200.0, 28.75, 77.11)
    assert nmea.parse(good)
    assert nmea.parse(good[:30]) is None  # cut off
    assert nmea.parse(good.replace('2845', '2846')) is None  # a flipped digit: wrong checksum
    assert nmea.parse(good.strip()[:-2] + '00') is None
    assert nmea.parse('$GNGGA,1*' + nmea.checksum('GNGGA,1') + '$GN') is None
    assert nmea.parse(b'\xff\xfe garbage') is None
    state = GpsState()
    state.feed('ABC', 0.0)
    assert state.bad_lines == 1 and state.problem(0.0).startswith('no position sentences')


def test_a_good_fix_is_passed_on_with_its_uncertainty():
    state = GpsState(GpsConfig(uere_m=1.5))
    fix = state.feed(nmea.gga_sentence(43200.0, 28.7501234, 77.1174567, quality=2, satellites=14, hdop=0.8,
                                       altitude=216.0, geoid_separation=-45.0), 10.0)
    assert fix.latitude == pytest.approx(28.7501234, abs=1e-8) and fix.longitude == pytest.approx(77.1174567, abs=1e-8)
    assert fix.altitude == pytest.approx(171.0)  # ellipsoid height, as NavSatFix wants it
    assert fix.horizontal_sigma == pytest.approx(1.2) and fix.quality == 2 and state.problem(10.5) == ''


@pytest.mark.parametrize('satellites, hdop, why', [(4, 0.9, 'only 4 satellites'), (9, 4.2, 'HDOP 4.2')])
def test_a_weak_fix_is_withheld(satellites, hdop, why):
    state = GpsState()
    assert state.feed(nmea.gga_sentence(0.0, 28.75, 77.11, satellites=satellites, hdop=hdop), 0.0) is None
    assert why in state.problem(0.0)


def test_southern_and_western_positions_and_rmc():
    fix = GpsState().feed(nmea.gga_sentence(0.0, -33.8568, -70.6483), 0.0)
    assert (fix.latitude, fix.longitude) == (pytest.approx(-33.8568, abs=1e-8), pytest.approx(-70.6483, abs=1e-8))
    rmc = nmea.rmc(nmea.parse(nmea.rmc_sentence(0.0, 28.75, 77.11, speed=0.5, course=270.0)))
    assert rmc.valid and rmc.speed == pytest.approx(0.5, abs=1e-3) and rmc.course == 270.0
    assert nmea.coordinate('2845.0000', 'X', 2) is None and nmea.coordinate('9130.0', 'N', 2) is None


def test_commands_get_their_checksum_and_answers_are_read():
    assert nmea.sentence('PAIR050,1000') == '$PAIR050,1000*12\r\n'
    assert nmea.sentence('$PQTMSAVEPAR') == nmea.sentence('PQTMSAVEPAR')
    state = GpsState()
    state.feed(nmea.sentence('PAIR001,050,0'), 0.0)
    assert state.acks == [(50, 0)] and nmea.ACK_RESULTS[0] == 'done'


@pytest.mark.parametrize('text', [
    '28.750123, 77.117456', '28.750123N 77.117456E', 'N28.750123 E77.117456', '28.750123 77.117456',
    '28°45\'00.4428"N 77°07\'02.8416"E', '28 45 00.4428 N, 77 7 2.8416 E', "28°45.00738'N 77°07.04736'E",
    '28d45m00.4428s N 77d07m02.8416s E', '28:45:00.4428N, 77:07:02.8416E'])
def test_a_position_can_be_typed_in_any_usual_form(text):
    latitude, longitude = geo.parse_position(text)
    assert latitude == pytest.approx(28.750123, abs=2e-6) and longitude == pytest.approx(77.117456, abs=2e-6)


def test_south_and_west():
    assert geo.parse_position('33°51\'24.5"S 151°12\'55.1"E') == (pytest.approx(-33.856806, abs=1e-6), pytest.approx(151.215306, abs=1e-6))
    assert geo.parse_position('-33.8568, -70.6483') == (-33.8568, -70.6483)
    assert geo.parse_position('33.8568 S, 70.6483 W') == (-33.8568, -70.6483)


@pytest.mark.parametrize('text, why', [
    ('28.75E, 77.1N', 'not a latitude direction'), ('95, 10', 'at most 90'), ('28, 190', 'at most 180'),
    ('28 61 0, 77 0 0', 'below 60'), ('-28.7S, 77', 'ambiguous'), ('abc, def', 'not a coordinate'),
    ('28.75', 'put a comma'), ('28.5 30.2 12, 77', 'only the last number may have decimals'), ('1, 2, 3', 'expected a latitude')])
def test_what_cannot_be_read_unambiguously_is_refused(text, why):
    with pytest.raises(ValueError, match=why):
        geo.parse_position(text)


def test_east_north_up_and_back():
    origin = (28.75, 77.11)
    east, north, _ = geo.enu(28.751, 77.112, origin)
    assert north == pytest.approx(110.83, abs=0.05) and east == pytest.approx(195.2, abs=0.2)  # 0.001 degrees each
    assert geo.from_enu(east, north, origin) == (pytest.approx(28.751, abs=1e-9), pytest.approx(77.112, abs=1e-9))
    assert geo.distance(origin, (28.751, 77.112)) == pytest.approx(math.hypot(east, north), abs=1e-6)
    assert '28°45\'00.00"N 77°06\'36.00"E' in geo.format_position(28.75, 77.11)


def test_waypoints_are_read_from_the_command_line_and_a_file(tmp_path):
    pytest.importorskip('rclpy')
    from rover_autonomy.gps_goals import read_positions  # noqa: PLC0415  (imports rclpy: only where ROS is)
    file = tmp_path / 'waypoints.txt'
    file.write_text('# the competition sheet\n28.7501, 77.1171\n\n28°45\'02.1"N 77°07\'05.3"E  # post B\n')
    positions = read_positions(['28.75 77.11'], str(file))
    assert positions[0] == (28.75, 77.11) and positions[1] == (28.7501, 77.1171) and len(positions) == 3
    with pytest.raises(ValueError, match='waypoint 2: "95"'):
        read_positions(['28.75, 77.11', '95, 77'], None)
    with pytest.raises(ValueError, match='no positions'):
        read_positions([], None)
