"""Which way is north? The rover's own track against the GPS's, for the GPS mode. No ROS, so it is unit-tested.

A single GPS antenna cannot tell which way the rover faces while it stands. But as it drives, the same track appears
twice: in the camera's odometry (precise, but turned by however the rover happened to face at the start) and in GPS
east/north coordinates (true directions, a metre or two of noise). The rotation and shift that lays the first over the
second (a least-squares fit of the two point sets) gives the odometry's heading against north, and with it where every
GPS fix, and every GPS goal, lies in the rover's map.

The fit needs some track: min_track_m between the farthest two antenna positions, straight or curved. It is redone
every refit_every_m of new track while the track is short, and kept once it is long (freeze_track_m): later refits
would turn the map frame under the rover and its costmaps. Fixes that do not fit (multipath next to a wall) are left
out of the fit.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class AlignmentConfig:
    min_track_m: float = 15.0  # the first fit needs the antenna to have covered this much (farthest two points)
    refit_every_m: float = 10.0  # then a new fit after this much more track ...
    freeze_track_m: float = 100.0  # ... until the track is this long: from then on the fit stays
    min_spacing_m: float = 0.5  # a new pair is taken only this far from the last (standing still adds nothing)
    outlier_m: float = 3.0  # a fix further than this (or 3 x the fit's RMS) from the fitted track is left out


@dataclass(frozen=True)
class Fit:
    theta: float  # rad: the rotation from the odometry frame to east-north (counter-clockwise, east = 0)
    tx: float  # m: east of the odometry origin, ...
    ty: float  # ... and north of it, relative to the GPS datum
    rms: float  # m: how far the fixes are from the fitted track, root mean square
    heading_sigma: float  # rad: roughly how well theta is known
    points: int  # pairs used
    track_m: float

    def odom_to_enu(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return c * x - s * y + self.tx, s * x + c * y + self.ty

    def enu_to_odom(self, east: float, north: float) -> tuple[float, float]:
        c, s = math.cos(self.theta), math.sin(self.theta)
        dx, dy = east - self.tx, north - self.ty
        return c * dx + s * dy, -s * dx + c * dy


def rigid_fit(odom: list[tuple[float, float]], enu: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    """(theta, tx, ty, rms) of the rotation and shift taking `odom` points onto `enu` points (least squares)."""
    n = len(odom)
    ox, oy = sum(p[0] for p in odom) / n, sum(p[1] for p in odom) / n
    ex, ey = sum(p[0] for p in enu) / n, sum(p[1] for p in enu) / n
    cross = dot = 0.0
    for (x, y), (e, nn) in zip(odom, enu):
        x, y, e, nn = x - ox, y - oy, e - ex, nn - ey
        dot += x * e + y * nn
        cross += x * nn - y * e
    theta = math.atan2(cross, dot)
    c, s = math.cos(theta), math.sin(theta)
    tx, ty = ex - (c * ox - s * oy), ey - (s * ox + c * oy)
    squares = sum((c * x - s * y + tx - e) ** 2 + (s * x + c * y + ty - nn) ** 2 for (x, y), (e, nn) in zip(odom, enu))
    return theta, tx, ty, math.sqrt(squares / n)


def extent(points: list[tuple[float, float]]) -> float:
    """The largest distance from the first point (the track's length as far as the fit cares)."""
    if not points:
        return 0.0
    x0, y0 = points[0]
    return max(math.hypot(x - x0, y - y0) for x, y in points)


@dataclass
class Alignment:
    config: AlignmentConfig = field(default_factory=AlignmentConfig)
    odom: list[tuple[float, float]] = field(default_factory=list)
    enu: list[tuple[float, float]] = field(default_factory=list)
    fit: Fit | None = None
    fitted_at_m: float = 0.0  # the track length of the last fit
    frozen: bool = False

    @property
    def ready(self) -> bool:
        return self.fit is not None

    def add(self, odom_xy: tuple[float, float], enu_xy: tuple[float, float]) -> bool:
        """A GPS fix (east, north of the datum) and where the odometry had the antenna at that moment. Returns True
        when the fit changed."""
        if self.frozen:
            return False
        if self.odom and math.hypot(odom_xy[0] - self.odom[-1][0], odom_xy[1] - self.odom[-1][1]) < self.config.min_spacing_m:
            return False
        self.odom.append(odom_xy)
        self.enu.append(enu_xy)
        track = extent(self.odom)
        due = track >= self.config.min_track_m if self.fit is None else track >= self.fitted_at_m + self.config.refit_every_m
        if not due:
            return False
        self.fit = self._fit(track)
        self.fitted_at_m = track
        if track >= self.config.freeze_track_m:
            self.frozen = True
        return True

    def _fit(self, track: float) -> Fit:
        odom, enu = self.odom, self.enu
        theta, tx, ty, rms = rigid_fit(odom, enu)
        # Leave out what does not fit (multipath), then fit again on the rest.
        limit = max(self.config.outlier_m, 3.0 * rms)
        c, s = math.cos(theta), math.sin(theta)
        keep = [i for i, ((x, y), (e, n)) in enumerate(zip(odom, enu))
                if math.hypot(c * x - s * y + tx - e, s * x + c * y + ty - n) <= limit]
        if 3 <= len(keep) < len(odom):
            odom, enu = [odom[i] for i in keep], [enu[i] for i in keep]
            theta, tx, ty, rms = rigid_fit(odom, enu)
        cx, cy = sum(p[0] for p in odom) / len(odom), sum(p[1] for p in odom) / len(odom)
        spread = math.sqrt(sum((x - cx) ** 2 + (y - cy) ** 2 for x, y in odom) / len(odom))
        # GPS errors wander slowly, so neighbouring fixes are far from independent: count about one per 5 m of track.
        independent = max(2.0, track / 5.0)
        sigma = max(rms, 0.5) / max(spread, 1e-6) / math.sqrt(independent)
        return Fit(theta, tx, ty, rms, sigma, len(odom), track)

    def status(self) -> str:
        track = extent(self.odom)
        if self.fit is None:
            if not self.odom:
                return 'GPS heading unknown: no GPS fix yet'
            return (f'GPS heading unknown: drive {self.config.min_track_m - track:.0f} m more (straight or in curves), '
                    'then GPS goals are taken')
        fit = self.fit
        return (f'GPS ready: heading known to about {math.degrees(fit.heading_sigma):.1f} degrees, fixes within '
                f'{fit.rms:.1f} m of the track ({fit.points} fixes over {fit.track_m:.0f} m'
                + (', final)' if self.frozen else ')'))
