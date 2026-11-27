"""Optional power-on launcher. Install explicitly as /main.py for standalone BLE."""

import sys

sys.path.insert(0, '/robot_arm')
from arm_service import run

run(announce_usb=False)
