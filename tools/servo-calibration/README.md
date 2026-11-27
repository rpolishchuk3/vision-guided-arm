# Servo calibration

Use this standalone GUI to record the measured angle of a servo at a known PWM pulse. The [main robot GUI](../../README.md#desktop-applications) uses those measurements for joint and Cartesian control.

## Setup

Use Python 3.10 or newer and install the shared dependencies with `python -m pip install -r requirements.txt`. For the ESP32, follow the [MicroPython deployment instructions](../../README.md#micropython-deployment).

Run from the repository root:

```powershell
python tools/servo-calibration/calibration_gui.py
```

Disconnect the main GUI before taking control in this app. Support the arm and keep its travel clear. In **USB Serial** mode, select the port and click **Connect**. In **Bluetooth LE** mode, click **Scan**, choose **RobotArm**, then connect with the shared key from the [BLE setup](../../README.md#bluetooth-le-setup). Losing the connection cancels motion and holds the most recent PWM. The first enable can move a joint abruptly. Disabling PWM leaves servo power connected.

## Record joint angles

1. Set the joint slider to a known pulse close to its physical position and enable the joint. Use PWM to move within its verified travel. Wait for it to settle.
2. Click **Calibrate**. Measure the angle from a consistent reference, enter it, and click **Record point**. Record at least two distinct pulses with angles that increase or decrease consistently. Click **Save** to update the shared [calibration data](../../config/joint_calibration.json).
3. Use **?s** to preview degree control in the calibrated range. An asterisk marks extrapolated values. The gripper uses PWM.

Both GUIs load the same data on connection. Reconnect the main GUI to pick up calibration changes. The displayed output angle is computed from the commanded pulse. The servos do not report physical position. Verify travel before updating joint limits or using the included data on another assembly.
