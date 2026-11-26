# Servo calibration

Desktop PWM and angle calibration for the ROT3U arm with an ESP32-S3 running MicroPython and a PCA9685 driver.

- `pc/servo_calibration_gui.py` provides per-joint PWM control, measurement capture, and degree control.
- `pc/joint_calibration.py` converts between PWM and degrees. `pc/joint_calibration.json` contains the measured points and approved extrapolation intervals for this assembled arm.
- `esp32/servo_config.py`, `servo_driver.py`, and `serial_bridge.py` are the board-side settings, PWM driver, and USB serial bridge.
- `esp32/pwm_calibration_cli.py` is an optional serial tool for probing PWM limits.

## Setup

Run commands from the repository root. Support the arm and clear its travel before uploading or connecting. The first enable can move a joint abruptly. Disabling PWM leaves servo power connected.

```powershell
python -m pip install pyserial mpremote
```

With MicroPython already installed, disconnect the GUI and close any serial monitor before deploying. In VS Code, run **Upload to ESP32** with **Ctrl+Shift+B**, or run the script in a terminal:

```powershell
python scripts/upload_esp32.py
python scripts/upload_esp32.py --port COM7  # if your ESP32 uses COM7
```

The default port is `COM10`. The script creates `/robot_arm/` and copies the ESP32 Python files into it, skipping identical files when the board supports SHA-256. It leaves other files and the board's root `main.py` and `boot.py` in place. An upload may interrupt the running controller as `mpremote` enters the raw REPL. The script stops at the first error and finishes without launching the controller.

To open the GUI after deployment:

```powershell
python tools/servo-calibration/pc/servo_calibration_gui.py
```

Choose the serial port and click **Connect**. The GUI can start the bridge from the MicroPython REPL and also recognizes existing boards with the bridge installed as `slider_control_v2.py`.

## Record and use joint angles

1. In **Calibrate**, move one joint within its verified mechanical travel using PWM. Wait for it to settle and measure the angle from a consistent reference.
2. Enter the angle and click **Record point**. Record at least two distinct pulses with angles that increase or decrease consistently. Click **Save** to write `pc/joint_calibration.json`.
3. Select a target in the degree-control range and click **µs** to switch that row to degrees. An asterisk marks an extrapolated angle. The gripper uses PWM.

The displayed angle is calculated from the commanded pulse. The controller has no measured position feedback. The included calibration depends on this arm's assembly and servo horn positions. The shoulder's configured PWM limits are provisional. Verify the mechanical travel before changing joint limits or reusing the calibration on another arm.

## Park the arm

**Park & Disable PWM** sends the `park` command to the ESP32. Enable all six motors using pulses close to their physical positions before parking. The ESP32 checks each parking target against its configured limits and reports any motors that need initialization. After the last trajectory finishes, it waits one second and disables PWM. **Hold / cancel motion** and **Disable all PWM** can interrupt either the movement or the wait.

The button is available with a serial bridge that advertises parking support. The MS24 servos may still hold torque after PWM is disabled. Servo power remains connected.

## Cartesian control

The origin is on the table below S1, with +Z up and +X along the straight horizontal arm. The right-hand rule defines +Y. TCP coordinates are in millimeters. Angles use the kinematic reference and are in degrees. Pitch is S2 + S3 + S4 in the plane defined by the base. Roll is S5, positive counterclockwise as viewed from the front of the gripper. The reference PWM values in `pc/geometry_config.py` give TCP (375, 0, 112) and zero pitch and roll. The FK result includes the S1–S4 axis points and TCP. S5 roll does not move the TCP in this model.

1. Connect to load the board's PWM limits. With known, settled commands on S1–S5, the GUI displays the estimated current pose.
2. Enter X, Y, Z and pitch. Leave roll blank to preserve the current commanded roll when available. **Calculate IK** previews a solution and its pulses without sending motion commands. The solver evaluates valid elbow and base branches and prefers the least joint travel from the current pose.
3. With a bridge that supports synchronized motion, click **Move to Target** to send the validated move. Recalculate after changing the target fields or starting pose. S6 is unaffected. **Hold / cancel motion** and **Disable all PWM** remain available during the move.

The path check uses calibrated angles and the synchronized PWM profile to check a 10 mm table clearance for the modeled link centerlines and TCP. It does not model arm thickness, self-collisions, workpieces, or other obstacles. The path is an interpolation in joint PWM space. The TCP can follow a curved path. Coordinates and angles are estimates derived from commanded pulses.

Run the software tests from the repository root:

```powershell
python -B -m unittest discover -s tools/servo-calibration/tests -v
```
