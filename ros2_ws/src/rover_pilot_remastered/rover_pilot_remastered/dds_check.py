"""Is CycloneDDS set up for the radio link? No ROS, so it is unit-tested.

The rover and the base station talk over the MikroTik link, a radio bridge behind a cable. CycloneDDS sees the
cable and, by default on a wired port, sends a topic as multicast once two or more other processes read it. The
MikroTik relays multicast over the air at its slowest rate, without acknowledgements: the rover's point clouds filled
the link until the two radios lost each other, and multicast heartbeats got lost. With AllowMulticast set to "spdp"
(or "false"), multicast is used for discovery only and all data goes unicast.
"""
from __future__ import annotations

import re
from pathlib import Path

CYCLONE = 'rmw_cyclonedds_cpp'
SAFE = {'spdp', 'false'}
SETUP = 'README, "Every terminal on both machines"'


def _sources(uri: str) -> list[str]:
    """The texts CYCLONEDDS_URI points at: inline XML, and the files it names (file://, or a plain path)."""
    texts = [uri]  # inline XML is checked as it is
    for part in uri.split(','):
        part = part.strip()
        if part.startswith('<'):
            continue
        path = part[len('file://'):] if part.startswith('file://') else part
        if path:
            try:
                texts.append(Path(path).expanduser().read_text())
            except OSError:
                pass
    return texts


def multicast_problem(environ) -> str:
    """'' if ROS data cannot go out as multicast here; else what is wrong and where the fix is described."""
    if environ.get('RMW_IMPLEMENTATION', '') != CYCLONE:
        return ''  # Fast DDS, the default, sends data unicast
    uri = environ.get('CYCLONEDDS_URI', '').strip()
    values = [value for text in _sources(uri) if uri
              for value in re.findall(r'<\s*AllowMulticast\s*>([^<]*)<\s*/\s*AllowMulticast\s*>', text, re.IGNORECASE)]
    if values and all({word.strip().lower() for word in value.split(',')} <= SAFE for value in values):
        return ''
    found = f'AllowMulticast is {values[-1].strip()!r}' if values else 'no CycloneDDS config sets AllowMulticast'
    return (f'CycloneDDS may send ROS data as multicast ({found}): over the MikroTik radio that can fill the link until '
            f'it drops. Set AllowMulticast to spdp on both machines ({SETUP}).')
