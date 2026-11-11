import sys
from pathlib import Path

# The package itself, and rover_pilot_remastered next to it (kinematics), without installing either.
SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'rover_autonomy'))
sys.path.insert(0, str(SRC / 'rover_pilot_remastered'))
