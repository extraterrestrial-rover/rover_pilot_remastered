"""Settings outside this package that the rover's behaviour depends on."""
import configparser
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]


def test_the_firmware_platform_is_pinned():
    """An unpinned `platform = espressif32` builds with whatever each computer has installed: Arduino core 2.x from
    the PlatformIO registry (loop watchdog 5 s) or 3.x (1 s, the build the rover runs)."""
    config = configparser.ConfigParser()
    config.read(REPO / 'firmware' / 'platformio.ini')
    platform = config['env:esp32doit-devkit-v1']['platform']
    assert platform.startswith('https://github.com/pioarduino/platform-espressif32/releases/download/'), platform
    release = platform.split('/download/')[1].split('/')[0]
    assert release[0].isdigit(), f'"{release}" is a moving tag, not a release'
    assert config['env:profile']['extends'] == 'env:esp32doit-devkit-v1'  # the profiling build uses the same platform
