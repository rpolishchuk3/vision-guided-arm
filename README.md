# Vision-Guided Robotic Manipulation with Adaptive Calibration and Visual Servoing

A low-cost 6-DOF robot arm that uses two cameras to find objects, pick them up, and place them accurately, correcting its own positioning errors with visual feedback. The pipeline is task-independent: only the task logic changes between applications. Our showcase application is **tic-tac-toe against a human**.

> CMPUT 312 (Introduction to Robotics) final project, University of Alberta, Fall 2026
> Team: Vlad, Ruslan, Robert
>
> **Status:** in development. Sections marked *(planned)* describe the intended design.

---

## Table of contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Software architecture](#software-architecture)
- [Repository structure](#repository-structure)
- [Getting started](#getting-started)
- [Usage](#usage)
- [Serial protocol](#serial-protocol)
- [How it works](#how-it-works)
- [Calibration](#calibration)
- [Evaluation](#evaluation)
- [Known challenges](#known-challenges)
- [Roadmap](#roadmap)
- [Safety](#safety)
- [Team](#team)

---

## Overview

Hobby servos such as the MG996 have gear backlash and **no position feedback**, so the arm never knows where it actually is. Instead of buying expensive hardware, this project closes the loop with vision:

1. A **side camera** looks down at the workspace. ArUco markers on the board give a homography that maps image pixels to table coordinates in centimetres. If the board is moved or rotated, the mapping is recomputed (**adaptive calibration**).
2. A **gripper camera** on the end effector measures the remaining offset between the gripper and the target, and the arm corrects itself in small steps before grasping and placing (**visual servoing**).
3. **Task logic** decides what to pick and where to place it. For tic-tac-toe this is minimax with alpha-beta pruning.

| | |
|---|---|
| **Open-loop goal** | within 2 cm using inverse kinematics alone |
| **Closed-loop goal** | under 5 mm with visual servoing |
| **Showcase** | full autonomous tic-tac-toe game against a human |

---

## Hardware

| Component | Details |
|---|---|
| Robot arm | diymore ROT3U aluminium 6-DOF arm kit |
| Servos | 6 × MG996 (5 joints + gripper): 9.4–11 kg·cm stall torque, 0.14–0.17 s per 60°, no position feedback |
| Microcontroller | ESP32-S3 mini |
| Servo driver | PCA9685 16-channel 12-bit PWM board (I2C) |
| Servo power | Separate 6 V supply (10 A or more recommended; each MG996 can draw up to 2.5 A when stalled) |
| Side camera | USB webcam on a fixed mount, looking down at the arm and board |
| Gripper camera | Small USB camera mounted on the end effector |
| Computer | Laptop running Python |
| Task objects | Tic-tac-toe board with 4 ArUco markers, X and O pieces |

### Joints

| Channel | Joint | Role |
|---|---|---|
| 0 | Base | Rotates the whole arm left and right |
| 1 | Shoulder | Raises and lowers the arm |
| 2 | Elbow | Extends and retracts the reach |
| 3 | Wrist pitch | Keeps the gripper pointing straight down |
| 4 | Wrist roll | Rotates the gripper to line up with a piece |
| 5 | Gripper | Opens and closes the claw |

> Check the channel numbers against your own wiring and update `config.py`.

---

## Wiring

| From | To |
|---|---|
| ESP32 **3V3** | PCA9685 **VCC** (logic power only) |
| ESP32 **GND** | PCA9685 **GND** |
| ESP32 **GPIO 8** | PCA9685 **SDA** |
| ESP32 **GPIO 9** | PCA9685 **SCL** |
| 6 V supply **+** | PCA9685 screw terminal **V+** |
| 6 V supply **−** | PCA9685 screw terminal **GND** |
| Servos | PCA9685 channels 0–5 (brown → GND, red → V+, orange → PWM) |
| ESP32 USB-C | Laptop (upload, serial commands) |

**Never power the servos from the ESP32.** A laptop USB port supplies well under 1 A; servo current spikes will reset the controller or damage the port. All grounds must be shared.

---

## Software architecture

```
 Side camera ──┐                                  ┌── Gripper camera
               ▼                                  ▼
          ┌─────────────────── vision.py ───────────────────┐
          │ ArUco homography · piece detection · tip offset │
          └───────────────────────┬─────────────────────────┘
                                  ▼
 game.py (minimax) ◄──────── main.py (pipeline + servo loop)
                                  │
                                  ▼
          ┌──────────────────── arm.py ─────────────────────┐
          │ kinematics · pulse calibration · serial link    │
          └───────────────────────┬─────────────────────────┘
                                  ▼  USB serial: "P p0 p1 p2 p3 p4 p5"
                        ESP32-S3 firmware (C++)
                                  ▼  I2C
                         PCA9685 → 6 × MG996
```

All decision-making runs in Python on the laptop. The ESP32 only receives six pulse widths and applies them, so the firmware rarely needs to change.

---

## Repository structure

```
robot-arm/
├── firmware/                  # ESP32-S3 PlatformIO project
│   ├── platformio.ini
│   └── src/main.cpp           # reads pulse commands over USB, drives the PCA9685
│
├── config.py                  # all constants: serial port, servo channels, pulse limits,
│                              # link lengths, camera IDs, HSV ranges, tolerances
├── arm.py                     # serial link, angle-to-pulse calibration, FK/IK, move_to(), grip()
├── vision.py                  # side camera: ArUco, homography, board and pieces
│                              # gripper camera: offset to target for visual servoing
├── game.py                    # tic-tac-toe board state, win check, minimax
├── main.py                    # entry point: modes, pipeline, visual servoing loop
│
├── calibration/               # generated calibration data
│   ├── side_camera.npz
│   ├── tip_camera.npz
│   └── servos.json
│
├── requirements.txt
└── README.md
```

| File | Responsibility |
|---|---|
| `config.py` | Plain Python constants with comments. Change numbers here, not in the logic. |
| `arm.py` | The only module that talks to the ESP32. Exposes `move_to(x, y, z)`, `grip()`, `release()`, `home()`. |
| `vision.py` | Camera capture, lens undistortion, ArUco detection, homography, HSV piece detection, gripper-camera offset. |
| `game.py` | Pure logic, no hardware. Testable on its own. |
| `main.py` | Command-line modes and the full pick-and-place pipeline. |

---

## Getting started

### MicroPython deployment

For the current ESP32 servo controller, install `mpremote` using `python -m pip install mpremote`.
Disconnect the GUI and close any serial monitor. Then run **Upload to ESP32** in VS Code
or run `python scripts/upload_esp32.py` from the repository root. Use `--port COM7` to override
the default port `COM10`. The script copies Python firmware to `/robot_arm/`.

See the [servo calibration setup](tools/servo-calibration/README.md#setup) for the full workflow.
The numbered setup below describes the planned PlatformIO application.

### Requirements

- Python 3.10 or newer
- [VS Code](https://code.visualstudio.com/) with the [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) extension
- The hardware listed above

### 1. Flash the firmware

1. Open the `firmware/` folder in VS Code with PlatformIO.
2. Connect the ESP32-S3 over USB-C.
3. Click **Upload** (→) in the PlatformIO status bar.
   - If the port is not found: hold **BOOT**, tap **RESET**, release **BOOT**, and upload again.
4. Open the serial monitor (plug icon). You should see `PCA9685 ready`.

### 2. Set up Python

```bash
git clone <repo-url>
cd robot-arm
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

`requirements.txt`:

```
opencv-contrib-python
numpy
pyserial
```

### 3. Configure

Edit `config.py`:

- `SERIAL_PORT`: for example `/dev/cu.usbmodem1101` on macOS or `COM5` on Windows
- `SIDE_CAMERA_ID`, `TIP_CAMERA_ID`: OpenCV camera indices
- Servo channels and pulse limits for your arm
- Link lengths measured with a ruler (cm)

---

## Usage

```bash
python main.py servo-test     # move individual servos to find pulse limits and centre positions
python main.py calibrate      # checkerboard lens calibration for both cameras
python main.py play           # play tic-tac-toe against a human
```

*(planned)* Additional tasks reuse the same pipeline with different task logic:

```bash
python main.py sort           # stretch goal: colour sorting
```

---

## Serial protocol

Text lines over USB serial at 115200 baud, ending with `\n`.

| Command | Meaning |
|---|---|
| `P p0 p1 p2 p3 p4 p5` | Set all six servo pulse widths in microseconds, channels 0–5 |
| `S ch pulse` | Set a single channel (used by `servo-test`) |

The firmware clamps every pulse to a safe range and ignores malformed lines. Example:

```
P 1500 1500 1500 1500 1500 1200
```

---

## How it works

Every task follows the same five stages:

1. **Detect** the board and objects with the side camera.
2. **Decide** the next move with the task logic (minimax for tic-tac-toe).
3. **Approach and align** above the object using the gripper camera.
4. **Grasp, lift, and transfer** the object.
5. **Align with the target and place** the object.

### Visual servoing rule

In stages 3 and 5:

1. The gripper camera measures the offset between the gripper and the target.
2. The offset is converted from pixels to centimetres.
3. The arm moves **50%** of the offset toward the target, waits to settle, then measures again.
4. It stops when the offset is **under 5 mm** or after **5 iterations**.

Moving only half the error each step prevents overshoot caused by camera latency and servo backlash.

### Tic-tac-toe mapping

| Pipeline | Tic-tac-toe |
|---|---|
| Input | Human's move (a board change that stays stable for 1 s) |
| Task logic | Minimax with alpha-beta pruning |
| Objects | X and O pieces |
| Goal positions | Board cells |

---

## Calibration

Run these in order the first time:

1. **Servo calibration** (`servo-test`): for each joint, record the pulse width at known angles and the safe minimum and maximum. Saved to `calibration/servos.json`.
2. **Lens calibration** (`calibrate`): show a printed checkerboard to each camera from several angles. Saved to `calibration/side_camera.npz` and `tip_camera.npz`.
3. **Board calibration** (automatic): at the start of each game, the side camera finds the 4 ArUco markers and computes the pixel-to-table homography. Moving the board only requires re-running this step.

---

## Evaluation

| Metric | How we measure it | Target |
|---|---|---|
| Positioning accuracy | 10 placements per cell, offset from cell centre, with vs. without servoing | < 1 cm (goal: < 5 mm) |
| Pick-and-place success | Grasped and released in the correct cell, over 50 attempts | > 90% |
| Board recalibration | Correct target positions after shifting or rotating the board | 9 of 10 tests |
| Execution time | From human move detected to piece released | < 15 s |
| Reliability | Full games completed without touching the robot | 4 of 5 games |

Results will be added here once testing is complete.

---

## Known challenges

| Challenge | Approach | Backup plan |
|---|---|---|
| No joint feedback, gear backlash | Approach targets from the same direction; correct with the gripper camera | Read each servo's internal potentiometer with the ESP32's analog inputs |
| Shoulder near its torque limit | Torque budget per joint, limited working radius, slow accelerations | Spring assist, or a 20 kg·cm servo |
| Camera-to-arm calibration error | Lens calibration, ArUco homography, piece-height correction | Fixed camera mount and a stored pose per cell |
| Unstable visual servoing | Settle before measuring, 50% gain, stop condition | Open-loop placement with a per-cell correction table |

---

## Roadmap

- [ ] **Week 1 (Nov 10–16):** assemble and wire the arm, servo calibration, mount cameras. *Milestone: every joint moves on command.*
- [ ] **Week 2 (Nov 17–23):** forward and inverse kinematics, test on 9 marked points, pick-and-place routine. *Milestone: repeatable open-loop pick-and-place.*
- [ ] **Week 3 (Nov 24–30):** ArUco homography, piece detection, visual servoing loop, minimax. *Milestone: closed-loop placement under 1 cm.*
- [ ] **Week 4 (Dec 1–7):** full games, tuning, evaluation data, demo. *Milestone: 5 test games, demo ready.*

### Stretch goals

- [ ] Move execution under 10 s
- [ ] Reliable detection under different lighting
- [ ] Detect failed grasps and retry automatically
- [ ] Colour sorting using the same pipeline

---

## Safety

- Test new code with servo horns detached or with a narrow pulse range first.
- Centre each servo (1500 µs) before attaching its horn.
- Keep hands clear of the arm while a program is running; unplug the 6 V supply to stop it immediately.
- If a servo becomes too hot to touch, stop and reduce the load or reach.

---

## Team

| Name | Focus |
|---|---|
| Vlad | |
| Ruslan | |
| Robert | |

Course: CMPUT 312, University of Alberta.
