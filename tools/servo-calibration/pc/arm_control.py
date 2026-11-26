"""Move the arm to coordinates typed in the terminal.

Runs on the PC. Talks to serial_bridge.py on the ESP32 (the same bridge the GUI uses),
solves the joint angles with kinematics.py, and converts them to pulses with
joint_calibration.json.

Coordinates are in cm from the middle of the base: x forward, y to the left, z up from the table.

Usage:
    python arm_control.py --port /dev/ttyACM0               (Windows: --port COM10)
    python arm_control.py --port /dev/ttyACM0 --camera 0    adds vision commands (v, look)

With --camera, the overhead camera sees the gripper tag (ID 4) and corrects the position:
it moves 50% of the remaining error each time and stops under 5 mm or after 5 corrections.

All servos are released (PWM off) when the program ends: after q, after an error, and after Ctrl+C.
Close the calibration GUI first; only one program can use the serial port.
"""

import argparse
import json
import math
import time

import serial

from joint_calibration import load_calibrations
from kinematics import forward_kinematics, inverse_kinematics

# Only needed for the vision commands; imported when --camera is given
cv2 = None
bd = None


# Set to True only after STRAIGHT_UP and DIRECTION below are filled in from the real arm.
# Until then only "where", "grip", "release" and "q" work.
CALIBRATED = False

# Joint order used by kinematics.py (theta1..theta5) and their motor numbers in the bridge
JOINT_NAMES = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]
JOINT_MOTORS = [1, 2, 3, 4, 5]
GRIPPER_MOTOR = 6

# Kinematics joint angles when the arm points straight up like a pole:
# base facing the board, shoulder vertical, elbow straight, gripper pointing up
STRAIGHT_UP_POSE = [0.0, 90.0, 0.0, 0.0, 0.0]

# PLACEHOLDERS: angle the GUI shows for each joint in that straight-up pose
STRAIGHT_UP_GUI_ANGLE = [90.0, 90.0, 90.0, 90.0, 90.0]

# PLACEHOLDERS: +1 if a bigger GUI angle moves the joint the positive way, -1 if the opposite way.
# Positive: base turns left, shoulder lifts up, elbow bends up, wrist tips the gripper up.
DIRECTION = [1, 1, 1, 1, 1]

# Safe kinematics angle range for each joint, in degrees (PLACEHOLDERS: check on the arm)
JOINT_MIN = [-80.0, 0.0, -150.0, -130.0, -90.0]
JOINT_MAX = [80.0, 130.0, 0.0, 30.0, 90.0]

# Gripper pulses in microseconds (PLACEHOLDERS: find them with "grip PULSE")
GRIPPER_OPEN_US = None
GRIPPER_CLOSED_US = None

# Board placement: centre cell position from the middle of the base, in cm
BOARD_CENTER_X = 19.0
BOARD_CENTER_Y = 0.0
CELL_CM = 6.0
HOVER_Z = 3.0

# Board axes in arm axes (board_detection.py frame -> arm frame), for the board placed as now:
# board x (tag 0 -> tag 1) points to the arm's right, board y (tag 0 -> tag 2) points toward the base
BOARD_X_IN_ARM = (0.0, -1.0)
BOARD_Y_IN_ARM = (-1.0, 0.0)

# Visual correction (README: move 50% of the error, stop under 5 mm or after 5 corrections)
SERVO_GAIN = 0.5
SERVO_TOLERANCE_MM = 5.0
SERVO_MAX_CORRECTIONS = 5

# Jaw tips minus the point under the gripper tag, in arm cm. Set it if the jaws end up
# consistently off-centre after a "v" move (tag not exactly above the jaws).
TIP_OFFSET_CM = (0.0, 0.0)

# Gripper angles to try, nearest to straight down first. Far or near cells may need a tilt.
PITCH_OPTIONS = [-90.0, -80.0, -100.0, -70.0, -110.0, -60.0, -120.0]

# Poses as kinematics joint angles
HOME_ANGLES = [0.0, 80.0, -90.0, -80.0, 0.0]
PARK_ANGLES = [0.0, 90.0, 0.0, 0.0, 0.0]    # straight up, so the arm balances when released

# Motion: the PC sends small steps so all joints move together
STEP_DEG = 3.0      # largest joint change per step
STEP_S = 0.1        # time between steps
SETTLE_S = 0.8      # wait after the last step so the bridge finishes the motion

# Command that starts the bridge from the MicroPython prompt (same as the GUI)
BRIDGE_START = ("import os, sys; sys.path.insert(0, '/robot_arm'); "
                "bridge = 'serial_bridge.py' if 'serial_bridge.py' in os.listdir('/robot_arm') "
                "else 'slider_control_v2.py'; exec(open('/robot_arm/' + bridge).read())")


class Bridge:
    """Serial link to serial_bridge.py. Messages from the ESP32 are lines starting with '@'."""

    def __init__(self, port):
        self.serial = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.25)
        self.serial.port = port

        # Keep DTR and RTS low so opening the port does not reset the ESP32
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.open()

        self.buffer = b""
        self.text_tail = ""
        self.frames = []
        self.errors = ""

    def send(self, line):
        self.serial.write((line + "\r").encode())

    def read(self):
        """Collect everything the ESP32 has sent so far."""

        waiting = self.serial.in_waiting
        if waiting:
            new_bytes = self.serial.read(waiting)
            self.buffer = self.buffer + new_bytes

            # Keep the last characters to spot the MicroPython prompt, which has no newline
            self.text_tail = (self.text_tail + new_bytes.decode(errors="replace"))[-2000:]

        while b"\n" in self.buffer:
            line, separator, rest = self.buffer.partition(b"\n")
            self.buffer = rest
            text = line.decode(errors="replace").strip()

            if "@" not in text:
                continue

            try:
                frame = json.loads(text[text.index("@") + 1:])
            except ValueError:
                continue

            # Error messages arrive in 30-character pieces; join them
            if frame[0] == "e":
                self.errors = self.errors + frame[1]
            else:
                self.frames.append(frame)

    def wait_for(self, kind, timeout):
        """Wait for a frame of the given kind ('h2', 'j', 'p', ...). Returns it, or None on timeout."""

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read()
            for index in range(len(self.frames)):
                if self.frames[index][0] == kind:
                    return self.frames.pop(index)
            time.sleep(0.02)
        return None

    def check_errors(self):
        # Give the ESP32 a moment to report a rejected command
        self.read()
        if self.errors:
            message = self.errors
            self.errors = ""
            raise RuntimeError("ESP32: " + message)

    def connect(self):
        """Talk to the bridge, starting it from the MicroPython prompt if it is not running."""

        self.send("hello2")
        if self.wait_for("h2", 1.5) is not None:
            print("Bridge already running")
            return

        print("Starting the bridge on the ESP32...")

        # Ctrl+C twice stops whatever is running and gives the MicroPython prompt
        self.serial.write(b"\x03\x03\r")
        deadline = time.monotonic() + 5
        while ">>>" not in self.text_tail:
            if time.monotonic() > deadline:
                raise RuntimeError("No MicroPython prompt from the ESP32")
            self.read()
            time.sleep(0.05)

        self.send(BRIDGE_START)

        # The bridge says hello when it starts; ask again in case the first one was missed
        for attempt in range(10):
            if self.wait_for("h2", 1.0) is not None:
                print("Bridge started")
                return
            self.send("hello2")

        raise RuntimeError("The bridge did not start. Check that /robot_arm/serial_bridge.py is on the ESP32.")

    def joint_limits(self, motor):
        # Reply: ['j', motor, min_us, max_us, center_us, current_us]
        self.send("joint {}".format(motor))
        frame = self.wait_for("j", 2.0)
        if frame is None:
            raise RuntimeError("No settings received for motor {}".format(motor))
        return frame[2], frame[3]

    def current_pulses(self):
        # Reply: ['p', pulse1, ..., pulse6], None for a motor with PWM off
        self.send("pose")
        frame = self.wait_for("p", 2.0)
        if frame is None:
            raise RuntimeError("No pose received from the bridge")
        return frame[1:]

    def set_pulse(self, motor, pulse, enabled):
        # A motor with PWM off must be enabled first; enable jumps straight to the pulse
        if enabled:
            self.send("move {} {}".format(motor, pulse))
        else:
            self.send("enable {} {}".format(motor, pulse))

    def release_all(self):
        # "stop" cancels motion and turns PWM off on every motor
        self.send("stop")
        time.sleep(0.2)

    def close(self):
        self.serial.close()


class Arm:
    def __init__(self, bridge, calibrations):
        self.bridge = bridge
        self.calibrations = calibrations

        # Pulse limits configured on the ESP32, per motor
        self.limits = {}
        for motor in JOINT_MOTORS + [GRIPPER_MOTOR]:
            self.limits[motor] = bridge.joint_limits(motor)

        # Which motors have PWM on, and the last commanded joint angles
        self.enabled = {}
        self.angles = None
        self.read_current_pose()

    def read_current_pose(self):
        """If every joint already has PWM on (for example from the GUI), start from its pulse."""

        pulses = self.bridge.current_pulses()
        for motor in JOINT_MOTORS + [GRIPPER_MOTOR]:
            self.enabled[motor] = pulses[motor - 1] is not None

        angles = []
        for index in range(len(JOINT_MOTORS)):
            pulse = pulses[JOINT_MOTORS[index] - 1]
            if pulse is None:
                self.angles = None
                return

            # A pulse outside the calibrated range cannot be turned into an angle
            try:
                gui_angle = self.calibrations[JOINT_MOTORS[index]].pwm_to_angle(pulse)
            except ValueError:
                self.angles = None
                return

            angles.append(gui_to_joint_angle(index, gui_angle))

        self.angles = angles

    def joint_angles_to_pulses(self, angles):
        """Check every joint and return its pulse. Raises ValueError if any joint is unsafe."""

        pulses = []
        for index in range(len(JOINT_MOTORS)):
            name = JOINT_NAMES[index]
            angle = angles[index]

            if angle < JOINT_MIN[index] or angle > JOINT_MAX[index]:
                raise ValueError("{} = {:.1f} deg is outside [{:.0f}, {:.0f}]".format(
                    name, angle, JOINT_MIN[index], JOINT_MAX[index]))

            # Kinematics angle -> GUI angle -> pulse from the measured calibration points
            gui_angle = joint_to_gui_angle(index, angle)
            motor = JOINT_MOTORS[index]
            try:
                pulse = self.calibrations[motor].angle_to_pwm(gui_angle)
            except ValueError:
                raise ValueError("{} needs GUI angle {:.1f}, outside the calibrated range".format(name, gui_angle))

            low, high = self.limits[motor]
            if pulse < low or pulse > high:
                raise ValueError("{} needs {} us, outside the ESP32 limits {}-{}".format(name, pulse, low, high))

            pulses.append(pulse)

        return pulses

    def send_pulses(self, pulses):
        for index in range(len(JOINT_MOTORS)):
            motor = JOINT_MOTORS[index]
            self.bridge.set_pulse(motor, pulses[index], self.enabled[motor])
            self.enabled[motor] = True

    def move_joints(self, target):
        """Move all joints together to the target angles in small steps."""

        # Check the end pose before anything moves
        final_pulses = self.joint_angles_to_pulses(target)

        # Unknown start: there is nothing to interpolate from, so jump once
        if self.angles is None:
            input("Start pose unknown; the arm will jump. Hold it near the target and press Enter...")
            self.send_pulses(final_pulses)
            self.angles = list(target)
            time.sleep(SETTLE_S)
            self.bridge.check_errors()
            return

        # The joint with the biggest change decides the number of steps
        largest_change = 0.0
        for index in range(len(target)):
            change = abs(target[index] - self.angles[index])
            if change > largest_change:
                largest_change = change

        steps = int(math.ceil(largest_change / STEP_DEG))
        if steps < 1:
            steps = 1

        start = list(self.angles)
        for step in range(1, steps + 1):
            fraction = step / steps

            pose = []
            for index in range(len(target)):
                pose.append(start[index] + (target[index] - start[index]) * fraction)

            self.send_pulses(self.joint_angles_to_pulses(pose))
            self.angles = pose
            time.sleep(STEP_S)
            self.bridge.check_errors()

        time.sleep(SETTLE_S)
        self.bridge.check_errors()

    def move_to(self, x, y, z, yaw):
        angles, pitch = self.solve(x, y, z, yaw)

        if pitch != -90.0:
            print("Gripper tilted to {:.0f} deg (straight down is out of reach here)".format(pitch))
        print("Angles: base {:.1f}, shoulder {:.1f}, elbow {:.1f}, wrist {:.1f}, roll {:.1f}".format(
            angles[0], angles[1], angles[2], angles[3], angles[4]))

        self.move_joints(angles)

    def solve(self, x, y, z, yaw):
        """Find joint angles, keeping the gripper as close to straight down as possible."""

        for pitch in PITCH_OPTIONS:
            try:
                angles = list(inverse_kinematics(x, y, z, yaw, pitch))
                self.joint_angles_to_pulses(angles)
            except ValueError:
                continue
            return angles, pitch

        raise ValueError("({:.1f}, {:.1f}, {:.1f}) cm is out of reach or outside the joint limits".format(x, y, z))

    def set_gripper(self, pulse):
        low, high = self.limits[GRIPPER_MOTOR]
        if pulse < low or pulse > high:
            raise ValueError("Gripper pulse must be {}-{} us".format(low, high))

        self.bridge.set_pulse(GRIPPER_MOTOR, pulse, self.enabled[GRIPPER_MOTOR])
        self.enabled[GRIPPER_MOTOR] = True
        self.bridge.check_errors()

    def release(self):
        self.bridge.release_all()
        for motor in self.enabled:
            self.enabled[motor] = False

        # A released arm sags, so the stored pose is no longer true
        self.angles = None
        print("All servos released")


def joint_to_gui_angle(index, angle):
    # Change from the straight-up pose, scaled by the joint's direction
    return STRAIGHT_UP_GUI_ANGLE[index] + DIRECTION[index] * (angle - STRAIGHT_UP_POSE[index])


def gui_to_joint_angle(index, gui_angle):
    return STRAIGHT_UP_POSE[index] + DIRECTION[index] * (gui_angle - STRAIGHT_UP_GUI_ANGLE[index])


def cell_position(row, col):
    """Centre of a cell in arm coordinates (cm).
    Row 0 is the far row (title side), col 0 is the left column (tag 0 side), seen from the base."""

    if row < 0 or row > 2 or col < 0 or col > 2:
        raise ValueError("Row and column must be 0, 1 or 2")

    x = BOARD_CENTER_X + CELL_CM * (1 - row)
    y = BOARD_CENTER_Y + CELL_CM * (1 - col)
    return x, y


def board_to_arm_vector(dx_mm, dy_mm):
    """A board displacement (mm) as an arm displacement (cm)."""
    dx_cm = (dx_mm * BOARD_X_IN_ARM[0] + dy_mm * BOARD_Y_IN_ARM[0]) / 10.0
    dy_cm = (dx_mm * BOARD_X_IN_ARM[1] + dy_mm * BOARD_Y_IN_ARM[1]) / 10.0
    return dx_cm, dy_cm


def arm_to_board_vector(dx_cm, dy_cm):
    """An arm displacement (cm) as a board displacement (mm). The axes are perpendicular,
    so this is the reverse of board_to_arm_vector."""
    dx_mm = (dx_cm * BOARD_X_IN_ARM[0] + dy_cm * BOARD_X_IN_ARM[1]) * 10.0
    dy_mm = (dx_cm * BOARD_Y_IN_ARM[0] + dy_cm * BOARD_Y_IN_ARM[1]) * 10.0
    return dx_mm, dy_mm


class Vision:
    """Overhead camera: finds the board and the gripper tag."""

    def __init__(self, camera_index):
        self.capture = cv2.VideoCapture(camera_index)
        if not self.capture.isOpened():
            raise RuntimeError("Could not open camera {}".format(camera_index))

        self.tracker = bd.BoardTracker(bd.load_layout())

        # Correction learned on earlier moves, used as the starting guess for the next cell
        self.learned_offset = [0.0, 0.0]

    def measure_tip(self, angles):
        """Board position (mm) of the jaw tips, seen through the gripper tag. None if not seen."""

        for attempt in range(10):
            # Skip old frames still waiting in the camera buffer
            for skipped in range(4):
                self.capture.grab()

            ok, frame = self.capture.read()
            if not ok:
                continue

            self.tracker.update(frame)
            if self.tracker.gripper is None:
                continue

            camera = self.tracker.camera()
            if camera is None:
                print("Board not found by the camera")
                return None

            # Where the tips should be (cm) and how the gripper is tilted
            tip_x, tip_y, tip_z = forward_kinematics(angles[0], angles[1], angles[2], angles[3])
            pitch = math.radians(angles[1] + angles[2] + angles[3])
            base = math.radians(angles[0])

            # The tag is TAG_TO_TIPS_MM back along the gripper from the tips
            tag_height_mm = tip_z * 10.0 + bd.TAG_TO_TIPS_MM * math.sin(-pitch)
            ground_x, ground_y = bd.gripper_ground_point(camera, self.tracker.gripper, tag_height_mm)

            # From the point under the tag to the tips: zero when straight down, along the arm when tilted
            reach_cm = bd.TAG_TO_TIPS_MM / 10.0 * math.cos(pitch)
            offset_x_cm = reach_cm * math.cos(base) + TIP_OFFSET_CM[0]
            offset_y_cm = reach_cm * math.sin(base) + TIP_OFFSET_CM[1]
            offset_x_mm, offset_y_mm = arm_to_board_vector(offset_x_cm, offset_y_cm)

            return ground_x + offset_x_mm, ground_y + offset_y_mm

        print("Gripper tag not seen in 10 frames")
        return None

    def close(self):
        self.capture.release()


def visual_move(arm, vision, row, col, z):
    """Hover over a cell, then correct with the camera until the tips are within 5 mm of its centre."""

    target_x_mm, target_y_mm = bd.cell_center_board(row, col)

    # Start from the measured board placement plus what earlier moves learned
    guess_x, guess_y = cell_position(row, col)
    x = guess_x + vision.learned_offset[0]
    y = guess_y + vision.learned_offset[1]

    for correction in range(SERVO_MAX_CORRECTIONS + 1):
        arm.move_to(x, y, z, 0.0)

        tip = vision.measure_tip(arm.angles)
        if tip is None:
            print("Stopping the correction; the arm stays here")
            return

        error_x_mm = target_x_mm - tip[0]
        error_y_mm = target_y_mm - tip[1]
        error_mm = math.sqrt(error_x_mm**2 + error_y_mm**2)
        print("Camera: tips at ({:.0f}, {:.0f}) mm on the board, {:.1f} mm from the cell centre".format(
            tip[0], tip[1], error_mm))

        if error_mm < SERVO_TOLERANCE_MM:
            break

        if correction == SERVO_MAX_CORRECTIONS:
            print("Stopped after {} corrections".format(SERVO_MAX_CORRECTIONS))
            break

        # Move only part of the error to avoid overshooting from camera delay and servo play
        step_x_cm, step_y_cm = board_to_arm_vector(error_x_mm, error_y_mm)
        x = x + SERVO_GAIN * step_x_cm
        y = y + SERVO_GAIN * step_y_cm

    # Remember the correction so the next cell starts closer
    vision.learned_offset[0] = x - guess_x
    vision.learned_offset[1] = y - guess_y
    print("Done: {:.1f} mm from the centre (learned offset {:.1f}, {:.1f} cm)".format(
        error_mm, vision.learned_offset[0], vision.learned_offset[1]))


def look(arm, vision):
    """Print where the camera sees the jaw tips, without moving."""

    if arm.angles is None:
        print("Pose unknown; move the arm first")
        return

    tip = vision.measure_tip(arm.angles)
    if tip is None:
        return

    cell = bd.cell_at_board(tip[0], tip[1])
    if cell is None:
        print("Tips at ({:.0f}, {:.0f}) mm on the board, outside the grid".format(tip[0], tip[1]))
    else:
        print("Tips at ({:.0f}, {:.0f}) mm on the board, cell {},{}".format(tip[0], tip[1], cell[0], cell[1]))


def print_help():
    print("\nCommands (cm and degrees):")
    print("  p x y z [yaw]      move the jaw tips to a point")
    print("  c row col [z]      hover over a cell (row 0 = far, col 0 = left), default z {:.0f}".format(HOVER_Z))
    print("  v row col [z]      same, then correct with the camera (needs --camera)")
    print("  look               where the camera sees the jaw tips (needs --camera)")
    print("  j b s e w r        move to joint angles")
    print("  home               home pose")
    print("  where              current joint angles and tip position")
    print("  open / close       gripper")
    print("  grip PULSE         gripper pulse in us, to find open and closed")
    print("  release            turn off all servos (the arm goes limp)")
    print("  q                  park, release and quit")


def needs_calibration():
    if not CALIBRATED:
        print("Locked: fill in STRAIGHT_UP_GUI_ANGLE and DIRECTION, then set CALIBRATED = True")
        return True
    return False


def read_numbers(parts, minimum, maximum):
    if len(parts) - 1 < minimum or len(parts) - 1 > maximum:
        raise ValueError("Wrong number of values for '{}'".format(parts[0]))

    numbers = []
    for word in parts[1:]:
        numbers.append(float(word))
    return numbers


def print_status(arm):
    if arm.angles is None:
        print("Pose unknown (servos released or not started yet)")
        return

    for index in range(len(JOINT_NAMES)):
        print("  {} {:.1f}".format(JOINT_NAMES[index], arm.angles[index]))

    x, y, z = forward_kinematics(arm.angles[0], arm.angles[1], arm.angles[2], arm.angles[3])
    print("Tip: x {:.1f}, y {:.1f}, z {:.1f} cm".format(x, y, z))


def command_loop(arm, vision):
    print_help()

    while True:
        try:
            line = input("\nCommand: ").strip().lower()
            if not line:
                continue

            parts = line.split()
            name = parts[0]

            if name == "q":
                return

            elif name == "where":
                print_status(arm)

            elif name == "release":
                arm.release()

            elif name == "grip":
                values = read_numbers(parts, 1, 1)
                arm.set_gripper(int(values[0]))

            elif name in ("open", "close"):
                if name == "open":
                    pulse = GRIPPER_OPEN_US
                else:
                    pulse = GRIPPER_CLOSED_US

                if pulse is None:
                    print("Set GRIPPER_OPEN_US and GRIPPER_CLOSED_US first (use 'grip PULSE' to find them)")
                    continue
                arm.set_gripper(pulse)

            elif needs_calibration():
                continue

            elif name == "p":
                values = read_numbers(parts, 3, 4)
                yaw = 0.0
                if len(values) == 4:
                    yaw = values[3]
                arm.move_to(values[0], values[1], values[2], yaw)

            elif name == "c":
                values = read_numbers(parts, 2, 3)
                z = HOVER_Z
                if len(values) == 3:
                    z = values[2]
                x, y = cell_position(int(values[0]), int(values[1]))
                print("Cell {},{} -> x {:.1f}, y {:.1f}, z {:.1f} cm".format(int(values[0]), int(values[1]), x, y, z))
                arm.move_to(x, y, z, 0.0)

            elif name in ("v", "look"):
                if vision is None:
                    print("Start with --camera N to use the camera")
                    continue

                if name == "look":
                    look(arm, vision)
                else:
                    values = read_numbers(parts, 2, 3)
                    z = HOVER_Z
                    if len(values) == 3:
                        z = values[2]
                    visual_move(arm, vision, int(values[0]), int(values[1]), z)

            elif name == "j":
                values = read_numbers(parts, 5, 5)
                arm.move_joints(values)

            elif name == "home":
                arm.move_joints(HOME_ANGLES)

            else:
                print("Unknown command")
                print_help()

        except ValueError as error:
            print("Error:", error)


def main():
    parser = argparse.ArgumentParser(description="Move the arm to typed coordinates")
    parser.add_argument("--port", required=True, help="ESP32 serial port, e.g. /dev/ttyACM0 or COM10")
    parser.add_argument("--camera", type=int, help="overhead camera index, enables v and look")
    args = parser.parse_args()

    vision = None
    if args.camera is not None:
        global cv2, bd
        import cv2
        import board_detection as bd
        vision = Vision(args.camera)

    calibrations = load_calibrations()
    for motor in JOINT_MOTORS:
        if motor not in calibrations:
            raise SystemExit("joint_calibration.json has no points for motor {}".format(motor))

    bridge = Bridge(args.port)
    arm = None
    try:
        bridge.connect()
        arm = Arm(bridge, calibrations)

        if not CALIBRATED:
            print("Not calibrated yet: only where, grip, release and q work.")

        command_loop(arm, vision)

        # Normal quit: stand the arm up before letting go
        if CALIBRATED and arm.angles is not None:
            print("Parking (hold the arm, it goes limp after this)...")
            arm.move_joints(PARK_ANGLES)

    finally:
        # Runs on q, errors and Ctrl+C, so the servos never stay powered after the program
        bridge.release_all()
        print("All servos released")
        bridge.close()
        if vision is not None:
            vision.close()


if __name__ == "__main__":
    main()
