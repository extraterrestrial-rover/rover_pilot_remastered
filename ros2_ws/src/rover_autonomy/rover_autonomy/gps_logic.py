"""What gps_node makes of the L89HA's sentences: fixes worth passing on, and a status. No ROS, so it is unit-tested.

A fix is passed on only when the receiver says it has one (GGA quality above 0), with enough satellites and a small
enough HDOP: a fix from three satellites, or with HDOP 5, can be tens of metres off and would pull the rover's position
away. Its uncertainty, for the EKF, is HDOP x uere_m (the L89HA's datasheet gives under 1.8 m CEP in the open, about
1.5 m standard deviation at HDOP 1).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from rover_autonomy import nmea


@dataclass(frozen=True)
class GpsConfig:
    uere_m: float = 1.5  # horizontal standard deviation at HDOP 1
    min_satellites: int = 5  # fewer used in the fix: not passed on
    max_hdop: float = 3.0  # a larger HDOP: not passed on
    fix_timeout_s: float = 3.0  # no GGA for this long: the receiver is lost (unplugged, crashed)


@dataclass(frozen=True)
class Fix:
    latitude: float
    longitude: float
    altitude: float  # m above the WGS84 ellipsoid (what NavSatFix carries)
    horizontal_sigma: float  # m
    vertical_sigma: float  # m
    quality: int  # nmea.FIX_QUALITY
    satellites: int
    hdop: float
    utc_s: float | None


@dataclass
class GpsState:
    config: GpsConfig = field(default_factory=GpsConfig)
    last_gga_time: float | None = None
    last_fix: Fix | None = None
    withheld: str = 'no data from the receiver yet'  # why the last GGA was not passed on ('' when it was)
    used: dict[str, int] = field(default_factory=dict)  # satellites used per system (GSA)
    in_view: dict[str, int] = field(default_factory=dict)  # satellites in view per system (GSV)
    strongest: dict[str, int] = field(default_factory=dict)  # best C/N0 per system (dB-Hz)
    speed: float | None = None  # m/s (RMC)
    course: float | None = None  # degrees from north (RMC)
    antenna: str = ''  # $PQTMANTENNASTATUS, as sent
    acks: list[tuple[int, int]] = field(default_factory=list)  # $PAIR001 answers, oldest first
    sentences: int = 0
    bad_lines: int = 0

    def feed(self, line: str | bytes, now: float) -> Fix | None:
        """One line from the receiver; returns a fix to pass on, if this line completed one."""
        sentence = nmea.parse(line)
        if sentence is None:
            if isinstance(line, bytes):
                line = line.decode('ascii', errors='replace')
            if line.strip():
                self.bad_lines += 1
            return None
        self.sentences += 1
        if sentence.talker == 'P':
            answer = nmea.ack(sentence)
            if answer is not None:
                self.acks.append(answer)
            elif sentence.kind == 'PQTMANTENNASTATUS':
                self.antenna = ','.join(sentence.fields)
            return None
        if sentence.kind == 'GSA':
            decoded = nmea.gsa(sentence)
            if decoded:
                self.used[decoded[0]] = decoded[1]
        elif sentence.kind == 'GSV':
            decoded = nmea.gsv(sentence)
            if decoded:
                system, count, satellites = decoded
                self.in_view[system] = count
                if sentence.fields[1] == '1':  # the first of a group: start this system's strongest anew
                    self.strongest.pop(system, None)
                for _, cn0 in satellites:
                    if cn0 is not None and cn0 >= 0:
                        self.strongest[system] = max(cn0, self.strongest.get(system, 0))
        elif sentence.kind == 'RMC':
            decoded = nmea.rmc(sentence)
            if decoded:
                self.speed, self.course = (decoded.speed, decoded.course) if decoded.valid else (None, None)
        elif sentence.kind == 'GGA':
            decoded = nmea.gga(sentence)
            if decoded:
                self.last_gga_time = now
                return self._judge(decoded)
        return None

    def _judge(self, gga: nmea.Gga) -> Fix | None:
        config = self.config
        if gga.quality <= 0 or gga.latitude is None or gga.longitude is None:
            self.withheld = 'no fix yet'
        elif gga.satellites < config.min_satellites:
            self.withheld = f'only {gga.satellites} satellites (at least {config.min_satellites} wanted)'
        elif gga.hdop is None or gga.hdop > config.max_hdop:
            self.withheld = f'HDOP {gga.hdop} (at most {config.max_hdop:g} wanted)'
        else:
            self.withheld = ''
            altitude = (gga.altitude or 0.0) + (gga.geoid_separation or 0.0)
            sigma = gga.hdop * config.uere_m
            self.last_fix = Fix(gga.latitude, gga.longitude, altitude, sigma, 2.0 * sigma, gga.quality,
                                gga.satellites, gga.hdop, gga.utc_s)
            return self.last_fix
        return None

    def problem(self, now: float) -> str:
        """'' while fixes are passed on; else why not."""
        if self.last_gga_time is None:
            return 'no position sentences from the receiver (wrong port, or wrong baud rate?)' if self.sentences or self.bad_lines \
                else 'nothing from the receiver'
        if now - self.last_gga_time > self.config.fix_timeout_s:
            return f'no position sentence for {now - self.last_gga_time:.0f} s'
        return self.withheld

    def summary(self) -> str:
        """Satellites per system: used / in view (strongest C/N0), for the status line."""
        systems = sorted(set(self.in_view) | set(self.used))
        parts = []
        for system in systems:
            if system == 'GNSS':
                continue
            strongest = self.strongest.get(system)
            parts.append(f'{system} {self.used.get(system, 0)}/{self.in_view.get(system, 0)}'
                         + (f' ({strongest} dB-Hz)' if strongest else ''))
        return ', '.join(parts) or 'no satellites in view'
