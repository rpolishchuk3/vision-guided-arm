# Servo calibration

Desktop PWM and angle calibration for the ROT3U arm with an ESP32-S3 running MicroPython and a PCA9685 driver.

- `pc/servo_calibration_gui.py` provides per-joint PWM control, measurement capture, and degree control.
- `pc/joint_calibration.py` converts between PWM and degrees. `pc/joint_calibration.json` contains the measured points and approved extrapolation intervals for this assembled arm.
- `esp32/servo_config.py`, `servo_driver.py`, and `serial_bridge.py` are the board-side settings, PWM driver, and USB serial bridge.
- `esp32/pwm_calibration_cli.py` is an optional serial tool for probing PWM limits.

## Setup

Run commands from this directory. Use the ESP32 serial port in place of `COM10` if it differs. Support the arm and clear its travel before connecting, uploading, or resetting. The first enable can move a joint abruptly. Disabling PWM leaves servo power connected.

```powershell
python -m pip install pyserial mpremote
```

With MicroPython already installed, create `/robot_arm` on the ESP32 if it does not exist:

```powershell
python -m mpremote connect COM10 fs mkdir :/robot_arm
```

Close other serial clients, then copy the bridge and its dependencies to the board:

```powershell
python -m mpremote connect COM10 fs cp esp32/servo_config.py esp32/servo_driver.py esp32/serial_bridge.py :/robot_arm/
python -m mpremote connect COM10 reset
python pc/servo_calibration_gui.py
```

Choose the serial port and click **Connect**. The GUI can start the bridge from the MicroPython REPL and also recognizes existing boards with the bridge installed as `slider_control_v2.py`. The copy uses `/robot_arm` and leaves the board's startup files in place.

## Record and use joint angles

1. In **Calibrate**, move one joint within its verified mechanical travel using PWM. Wait for it to settle and measure the angle from a consistent reference.
2. Enter the angle and click **Record point**. Record at least two distinct pulses with angles that increase or decrease consistently. Click **Save** to write `pc/joint_calibration.json`.
3. Select a target in the degree-control range and click **µs** to switch that row to degrees. An asterisk marks an extrapolated angle. The gripper uses PWM.

The displayed angle is calculated from the commanded pulse. The controller has no measured position feedback. The included calibration depends on this arm's assembly and servo horn positions. The shoulder's configured PWM limits are provisional. Verify the mechanical travel before changing joint limits or reusing the calibration on another arm.
