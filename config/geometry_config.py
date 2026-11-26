"""Arm geometry, in millimeters, and the horizontal joint reference."""

BASE_HEIGHT_MM = 112
SHOULDER_OFFSET_MM = 0
UPPER_ARM_MM = 104
FOREARM_MM = 99
TOOL_MM = 172

JOINTS = (1, 2, 3, 4, 5)
REFERENCE_PWM = {1: 1500, 2: 497, 3: 1500, 4: 1500, 5: 2031}
JOINT_DIRECTION = {1: 1, 2: 1, 3: -1, 4: -1, 5: 1}

# Margin above z = 0 for the modeled link centerlines and TCP.
TABLE_CLEARANCE_MM = 10
