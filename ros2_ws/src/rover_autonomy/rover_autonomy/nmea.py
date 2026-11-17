"""NMEA 0183 (4.11) sentences of the Quectel L89HA, read and written. No ROS, so it is unit-tested.

The L89HA (Airoha chipset) sends RMC, GGA, VTG, GSA (one per satellite system), GSV (per system), GLL and Quectel's
own $PQTM... sentences, with the talker GN for combined solutions (GP: GPS, GA: Galileo, GI: NavIC, GL: GLONASS, GB:
BeiDou). It takes $PAIR... commands and answers each with $PAIR001,<command>,<result>. Only what the rover uses is
decoded here; anything else is passed over.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# GSA/GSV system IDs (NMEA 4.11) and talkers, for the status report
SYSTEMS = {1: 'GPS', 2: 'GLONASS', 3: 'Galileo', 4: 'BeiDou', 5: 'QZSS', 6: 'NavIC'}
TALKERS = {'GP': 'GPS', 'GL': 'GLONASS', 'GA': 'Galileo', 'GB': 'BeiDou', 'BD': 'BeiDou', 'GQ': 'QZSS', 'QZ': 'QZSS',
           'GI': 'NavIC', 'GN': 'GNSS'}
FIX_QUALITY = {0: 'no fix', 1: 'GPS fix', 2: 'differential (SBAS)', 4: 'RTK fixed', 5: 'RTK float', 6: 'dead reckoning'}
KNOT = 1852.0 / 3600.0  # m/s


@dataclass(frozen=True)
class Sentence:
    talker: str  # 'GN', 'GP', ... ('P' for proprietary sentences such as $PAIR001)
    kind: str  # 'GGA', 'RMC', ... ('PAIR001', 'PQTMANTENNASTATUS' for proprietary ones)
    fields: tuple[str, ...]


@dataclass(frozen=True)
class Gga:
    utc_s: float | None  # seconds since midnight UTC
    latitude: float | None  # degrees, north positive
    longitude: float | None  # degrees, east positive
    quality: int  # FIX_QUALITY
    satellites: int  # used in the fix
    hdop: float | None
    altitude: float | None  # m above mean sea level
    geoid_separation: float | None  # m: ellipsoid height = altitude + geoid_separation


@dataclass(frozen=True)
class Rmc:
    utc_s: float | None
    valid: bool
    latitude: float | None
    longitude: float | None
    speed: float | None  # m/s over ground
    course: float | None  # degrees from true north, clockwise
    date: str  # ddmmyy as sent


def checksum(body: str) -> str:
    """The two hex digits of an NMEA checksum: XOR of everything between $ and *."""
    value = 0
    for character in body.encode('ascii'):
        value ^= character
    return f'{value:02X}'


def sentence(body: str) -> str:
    """A complete sentence for `body` (without $ and checksum), e.g. sentence('PAIR050,1000')."""
    body = body.strip().lstrip('$').split('*')[0]
    return f'${body}*{checksum(body)}\r\n'


def parse(line: str | bytes) -> Sentence | None:
    """A sentence with a correct checksum, or None (noise, a cut-off line, a wrong checksum)."""
    if isinstance(line, bytes):
        line = line.decode('ascii', errors='replace')
    line = line.strip()
    if not line.startswith('$') or len(line) < 7 or line[-3] != '*':
        return None
    body, given = line[1:-3], line[-2:].upper()
    if checksum(body) != given or '$' in body:
        return None
    address, *fields = body.split(',')
    if address.startswith('P'):
        return Sentence('P', address, tuple(fields))
    if len(address) != 5:
        return None
    return Sentence(address[:2], address[2:], tuple(fields))


def _float(text: str) -> float | None:
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _int(text: str, default: int = 0) -> int:
    try:
        return int(text)
    except ValueError:
        return default


def utc_seconds(text: str) -> float | None:
    """hhmmss.sss -> seconds since midnight."""
    if len(text) < 6:
        return None
    hours, minutes, seconds = _int(text[0:2], -1), _int(text[2:4], -1), _float(text[4:])
    if not (0 <= hours < 24 and 0 <= minutes < 60) or seconds is None or not 0 <= seconds < 61:
        return None
    return hours * 3600 + minutes * 60 + seconds


def coordinate(value: str, hemisphere: str, degree_digits: int) -> float | None:
    """ddmm.mmmm (latitude, 2 degree digits) or dddmm.mmmm (longitude, 3) with N/S/E/W -> signed degrees."""
    if not value or hemisphere not in ('N', 'S', 'E', 'W') or '.' not in value:
        return None
    degrees, minutes = _int(value[:degree_digits], -1), _float(value[degree_digits:])
    if degrees < 0 or minutes is None or not 0 <= minutes < 60:
        return None
    result = degrees + minutes / 60.0
    limit = 90.0 if degree_digits == 2 else 180.0
    if result > limit:
        return None
    return -result if hemisphere in ('S', 'W') else result


def gga(s: Sentence) -> Gga | None:
    if s.kind != 'GGA' or len(s.fields) < 11:
        return None
    f = s.fields
    return Gga(utc_seconds(f[0]), coordinate(f[1], f[2], 2), coordinate(f[3], f[4], 3), _int(f[5]), _int(f[6]),
               _float(f[7]), _float(f[8]), _float(f[10]))


def rmc(s: Sentence) -> Rmc | None:
    if s.kind != 'RMC' or len(s.fields) < 9:
        return None
    f = s.fields
    speed = _float(f[6])
    return Rmc(utc_seconds(f[0]), f[1] == 'A', coordinate(f[2], f[3], 2), coordinate(f[4], f[5], 3),
               None if speed is None else speed * KNOT, _float(f[7]), f[8])


def gsa(s: Sentence) -> tuple[str, int, float | None, float | None, float | None] | None:
    """(system, satellites used, PDOP, HDOP, VDOP). The system comes from the 4.11 system ID (the last field)."""
    if s.kind != 'GSA' or len(s.fields) < 17:
        return None
    f = s.fields
    used = sum(1 for prn in f[2:14] if prn)
    system = SYSTEMS.get(_int(f[17]), TALKERS.get(s.talker, s.talker)) if len(f) > 17 else TALKERS.get(s.talker, s.talker)
    return system, used, _float(f[14]), _float(f[15]), _float(f[16])


def gsv(s: Sentence) -> tuple[str, int, list[tuple[str, int | None]]] | None:
    """(system, satellites in view, [(satellite, C/N0 in dB-Hz or None), ...] in this sentence)."""
    if s.kind != 'GSV' or len(s.fields) < 3:
        return None
    f = s.fields
    satellites = []
    blocks = f[3:]
    if len(blocks) % 4 == 1:  # 4.11: a signal ID after the satellite blocks
        blocks = blocks[:-1]
    for i in range(0, len(blocks) - 3, 4):
        cn0 = blocks[i + 3]
        satellites.append((blocks[i], _int(cn0, -1) if cn0 else None))
    return TALKERS.get(s.talker, s.talker), _int(f[2]), satellites


def ack(s: Sentence) -> tuple[int, int] | None:
    """$PAIR001,<command>,<result>: (command number, result; 0 = done, 1 = processing, 2 = failed, 3 = unsupported,
    4 = bad parameters, 5 = busy)."""
    if s.talker != 'P' or s.kind != 'PAIR001' or len(s.fields) < 2:
        return None
    return _int(s.fields[0], -1), _int(s.fields[1], -1)


ACK_RESULTS = {0: 'done', 1: 'processing', 2: 'failed', 3: 'not supported', 4: 'wrong parameters', 5: 'busy'}


def to_ddmm(value: float, degree_digits: int) -> tuple[str, str]:
    """Signed degrees -> (ddmm.mmmmmm, hemisphere), as GGA and RMC carry them (for simulators and tests)."""
    hemisphere = ('N' if value >= 0 else 'S') if degree_digits == 2 else ('E' if value >= 0 else 'W')
    value = abs(value)
    degrees = int(value)
    minutes = (value - degrees) * 60.0
    if round(minutes, 6) >= 60.0:
        degrees, minutes = degrees + 1, 0.0
    return f'{degrees:0{degree_digits}d}{minutes:09.6f}', hemisphere


def gga_sentence(utc_s: float, latitude: float, longitude: float, quality: int = 1, satellites: int = 12,
                 hdop: float = 0.8, altitude: float = 216.0, geoid_separation: float = -45.0) -> str:
    hours, rest = divmod(utc_s % 86400.0, 3600.0)
    minutes, seconds = divmod(rest, 60.0)
    lat, ns = to_ddmm(latitude, 2)
    lon, ew = to_ddmm(longitude, 3)
    body = (f'GNGGA,{int(hours):02d}{int(minutes):02d}{seconds:06.3f},{lat},{ns},{lon},{ew},{quality},{satellites:02d},'
            f'{hdop:.2f},{altitude:.1f},M,{geoid_separation:.1f},M,,')
    return sentence(body)


def rmc_sentence(utc_s: float, latitude: float, longitude: float, speed: float = 0.0, course: float = 0.0,
                 date: str = '171126') -> str:
    hours, rest = divmod(utc_s % 86400.0, 3600.0)
    minutes, seconds = divmod(rest, 60.0)
    lat, ns = to_ddmm(latitude, 2)
    lon, ew = to_ddmm(longitude, 3)
    body = (f'GNRMC,{int(hours):02d}{int(minutes):02d}{seconds:06.3f},A,{lat},{ns},{lon},{ew},{speed / KNOT:.3f},'
            f'{course:.2f},{date},,,A,V')
    return sentence(body)
