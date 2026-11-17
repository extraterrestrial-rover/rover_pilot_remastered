"""Coordinates: reading them as people write them, and local east-north-up positions. No ROS, so it is unit-tested.

A position can be typed in any of the usual forms, each coordinate on its own:
    28.750123, 77.117456                        decimal degrees (south and west negative)
    28.750123N 77.117456E                       ... or with N/S/E/W, before or after
    28°45'00.4"N 77°07'02.8"E                   degrees minutes seconds
    28 45 00.4 N, 77 7 2.8 E                    ... with spaces
    28°45.0074'N 77°07.0474'E                   degrees decimal minutes
Latitude first, longitude second. Anything that cannot be read unambiguously, or is out of range, is refused with why.
"""
from __future__ import annotations

import math
import re

# WGS84
A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)

_NUMBER = r'[+-]?\d+(?:\.\d+)?'
_HEMISPHERE = r'[NSEWnsew]'


def _coordinate(text: str, axis: str) -> float:
    """One coordinate ('lat' or 'lon') in any of the forms above."""
    original = text
    text = text.strip().replace('º', '°').replace('’', "'").replace('′', "'").replace('″', '"').replace("''", '"')
    hemisphere = None
    match = re.fullmatch(rf'({_HEMISPHERE})\s*(.+)|(.+?)\s*({_HEMISPHERE})', text)
    if match:
        hemisphere = (match.group(1) or match.group(4)).upper()
        text = (match.group(2) or match.group(3)).strip()
    parts = [part for part in re.split(r'[°d\'m"s\s:]+', text) if part]
    if not 1 <= len(parts) <= 3 or not all(re.fullmatch(_NUMBER, part) for part in parts):
        raise ValueError(f'"{original}" is not a coordinate (use e.g. 28.750123, 28°45.0074\' or 28°45\'00.4")')
    numbers = [float(part) for part in parts]
    if any(part.startswith(('+', '-')) for part in parts[1:]) or (len(parts) > 1 and '.' in ''.join(parts[:-1])):
        raise ValueError(f'"{original}": only the last number may have decimals, and only the first a sign')
    if any(not 0 <= value < 60 for value in numbers[1:]):
        raise ValueError(f'"{original}": minutes and seconds must be below 60')
    sign = -1.0 if numbers[0] < 0 or parts[0].startswith('-') else 1.0
    value = abs(numbers[0]) + sum(value / 60.0 ** i for i, value in enumerate(numbers[1:], start=1))
    if hemisphere is not None:
        expected = ('N', 'S') if axis == 'lat' else ('E', 'W')
        if hemisphere not in expected:
            raise ValueError(f'"{original}": {hemisphere} is not a {"latitude" if axis == "lat" else "longitude"} '
                             f'direction ({" or ".join(expected)} expected; latitude comes first)')
        if sign < 0:
            raise ValueError(f'"{original}": a minus sign and {hemisphere} together are ambiguous')
        if hemisphere in ('S', 'W'):
            sign = -1.0
    limit = 90.0 if axis == 'lat' else 180.0
    if value > limit:
        raise ValueError(f'"{original}": a {"latitude" if axis == "lat" else "longitude"} is at most {limit:g} degrees')
    return sign * value


def parse_position(text: str) -> tuple[float, float]:
    """'latitude longitude' in any of the forms above -> (latitude, longitude) in signed decimal degrees."""
    text = text.strip()
    if ',' in text:
        pieces = [piece for piece in text.split(',') if piece.strip()]
    else:
        # Without a comma: split after the latitude's hemisphere letter, or into halves of the numbers.
        split = re.search(rf'(?<=[0-9"\'°\s])({_HEMISPHERE})(?=\s*[0-9EWew+-])', text)
        if split and split.group(1).upper() in ('N', 'S'):
            pieces = [text[:split.end()], text[split.end():]]
        elif re.fullmatch(rf'\s*([NSns])\s*[^EWew]+[EWew].+', text):
            index = re.search(r'[EWew]', text).start()
            pieces = [text[:index], text[index:]]
        else:
            numbers = text.split()
            if len(numbers) % 2:
                raise ValueError(f'"{text}": cannot tell where the latitude ends; put a comma between the two')
            half = len(numbers) // 2
            pieces = [' '.join(numbers[:half]), ' '.join(numbers[half:])]
    if len(pieces) != 2:
        raise ValueError(f'"{text}": expected a latitude and a longitude')
    return _coordinate(pieces[0], 'lat'), _coordinate(pieces[1], 'lon')


def format_position(latitude: float, longitude: float) -> str:
    """Decimal degrees and degrees-minutes-seconds, for messages."""
    def dms(value: float, positive: str, negative: str) -> str:
        hemisphere = positive if value >= 0 else negative
        value = abs(value)
        degrees = int(value)
        minutes = int((value - degrees) * 60)
        seconds = (value - degrees - minutes / 60) * 3600
        return f'{degrees}°{minutes:02d}\'{seconds:05.2f}"{hemisphere}'  # 0.01" is 0.3 m
    return f'{latitude:.6f}, {longitude:.6f} ({dms(latitude, "N", "S")} {dms(longitude, "E", "W")})'


def ecef(latitude: float, longitude: float, height: float = 0.0) -> tuple[float, float, float]:
    lat, lon = math.radians(latitude), math.radians(longitude)
    n = A / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
    return ((n + height) * math.cos(lat) * math.cos(lon), (n + height) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - E2) + height) * math.sin(lat))


def enu(latitude: float, longitude: float, origin: tuple[float, float], height: float = 0.0,
        origin_height: float = 0.0) -> tuple[float, float, float]:
    """East, north, up in metres of a position from `origin` (latitude, longitude)."""
    x, y, z = ecef(latitude, longitude, height)
    ox, oy, oz = ecef(*origin, origin_height)
    dx, dy, dz = x - ox, y - oy, z - oz
    lat, lon = math.radians(origin[0]), math.radians(origin[1])
    east = -math.sin(lon) * dx + math.cos(lon) * dy
    north = -math.sin(lat) * math.cos(lon) * dx - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz
    up = math.cos(lat) * math.cos(lon) * dx + math.cos(lat) * math.sin(lon) * dy + math.sin(lat) * dz
    return east, north, up


def from_enu(east: float, north: float, origin: tuple[float, float], up: float = 0.0,
             origin_height: float = 0.0) -> tuple[float, float]:
    """(latitude, longitude) of a point east/north/up of `origin` (the inverse of enu, to well below a millimetre)."""
    lat0, lon0 = math.radians(origin[0]), math.radians(origin[1])
    ox, oy, oz = ecef(*origin, origin_height)
    x = ox - math.sin(lon0) * east - math.sin(lat0) * math.cos(lon0) * north + math.cos(lat0) * math.cos(lon0) * up
    y = oy + math.cos(lon0) * east - math.sin(lat0) * math.sin(lon0) * north + math.cos(lat0) * math.sin(lon0) * up
    z = oz + math.cos(lat0) * north + math.sin(lat0) * up
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - E2))
    for _ in range(5):
        n = A / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
        height = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - E2 * n / (n + height)))
    return math.degrees(lat), math.degrees(lon)


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Metres along the ground between two (latitude, longitude), for the distances a rover drives."""
    east, north, _ = enu(*b, origin=a)
    return math.hypot(east, north)
