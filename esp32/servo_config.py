"""Hardware settings and servo parameters. Edit this file to change defaults."""

I2C_BUS = 0
SDA_PIN = 8
SCL_PIN = 9
I2C_FREQ = 100_000
PCA_ADDRESS = 0x40
PCA_PRESCALE = 121
MEASURED_HZ = 50.0
UPDATE_MS = 20

# Absolute software bounds for supervised calibration ONLY.
# These are not guaranteed mechanical limits for any servo.
PROBE_MIN_US = 200
PROBE_MAX_US = 3000
DEFAULT_STEP_US = 25
DEFAULT_SPEED = 500.0       # us/s (commanded pulse trajectory)
DEFAULT_ACCEL = 1500.0     # us/s^2

# No servo is automatically moved to center_us at startup.
SERVOS = {
    1: {
        'name': 'Base', 'model': 'MS24', 'channel': 0,
        'min_us': 420, 'max_us': 2430, 'center_us': 1500,
        'speed': DEFAULT_SPEED, 'accel': DEFAULT_ACCEL,
    },
    2: {
        'name': 'Shoulder', 'model': 'MS24', 'channel': 2,
        'min_us': PROBE_MIN_US, 'max_us': 3000, 'center_us': 1500,
        'speed': DEFAULT_SPEED, 'accel': DEFAULT_ACCEL,
    },
    3: {
        'name': 'Elbow', 'model': 'MG996R', 'channel': 4,
        'min_us': 350, 'max_us': 2600, 'center_us': 1500,
        'speed': DEFAULT_SPEED, 'accel': DEFAULT_ACCEL,
    },
    4: {
        'name': 'Wrist pitch', 'model': 'MG996R', 'channel': 6,
        'min_us': 750, 'max_us': 2600, 'center_us': 1500,
        'speed': DEFAULT_SPEED, 'accel': DEFAULT_ACCEL,
    },
    5: {
        'name': 'Wrist roll', 'model': 'MG996R', 'channel': 8,
        'min_us': 500, 'max_us': 2600, 'center_us': 1500,
        'speed': DEFAULT_SPEED, 'accel': DEFAULT_ACCEL,
    },
    6: {
        'name': 'Gripper', 'model': 'MG996R', 'channel': 10,
        'min_us': 1500, 'max_us': 2150, 'center_us': 1900,
        'speed': DEFAULT_SPEED, 'accel': DEFAULT_ACCEL,
    },
}

# Converted from the desktop joint calibration. Motor 6 is specified in PWM.
PARK_POSITION = {
    1: 2392,  # 176.60 degrees
    2: 897,   # 136.22 degrees
    3: 1927,  # 133.22 degrees
    4: 2035,  # 143.14 degrees
    5: 1998,  # 85.54 degrees
    6: 1563,
}

# Fill in after checking which pulse actually opens/closes the gripper.
GRIPPER_OPEN_US = None
GRIPPER_CLOSED_US = None
