# Forward and inverse kinematics for the diymore ROT3U arm (5 joints + gripper).
# Pure math with no hardware imports, so the same file runs in MicroPython on the
# ESP32 and in normal Python on the laptop.
#
# World frame: origin on the table directly below the base axis.
#   x forward, y to the left, z up. Lengths in cm, angles in degrees.
#
# Joint angles are geometric angles, not servo angles:
#   theta1  base yaw: 0 = arm points along +x, positive = turns left
#   theta2  shoulder: upper arm angle above horizontal (0 = horizontal, 90 = straight up)
#   theta3  elbow: forearm angle relative to the upper arm (0 = straight, negative = bends down)
#   theta4  wrist pitch: gripper angle relative to the forearm
#   theta5  wrist roll: 0 = jaws open along the arm's direction

import math

# Arm geometry in cm, measured servo shaft centre to shaft centre.
D1 = 11.8   # table surface up to the shoulder axis (measured)
A1 = 0.0    # shoulder axis sits directly above the base axis (measured)
L2 = 10.4   # shoulder axis to elbow axis (measured)
L3 = 9.9    # elbow axis to wrist pitch axis (measured)
L4 = 17.2   # wrist pitch axis to the jaw tips (measured)

# Default gripper direction: straight down, which means theta2 + theta3 + theta4 = -90
TOOL_PITCH = -90.0


def forward_kinematics(theta1, theta2, theta3, theta4):
    """Return the grasp point (x, y, z) in cm for joint angles in degrees."""

    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)
    t4 = math.radians(theta4)

    # Horizontal distance from the base axis, inside the arm's vertical plane
    r = (A1
         + L2 * math.cos(t2)
         + L3 * math.cos(t2 + t3)
         + L4 * math.cos(t2 + t3 + t4))

    # Height above the table
    z = (D1
         + L2 * math.sin(t2)
         + L3 * math.sin(t2 + t3)
         + L4 * math.sin(t2 + t3 + t4))

    # Rotate the arm's plane by the base angle to get x and y
    x = r * math.cos(t1)
    y = r * math.sin(t1)

    return x, y, z


def wrap_roll(angle):
    """Keep a roll angle within -90..90, since the gripper looks the same after a half turn."""

    while angle >= 90.0:
        angle = angle - 180.0

    while angle < -90.0:
        angle = angle + 180.0

    return angle


def inverse_kinematics(x, y, z, piece_yaw=0.0, tool_pitch=TOOL_PITCH):
    """Return (theta1, theta2, theta3, theta4, theta5) in degrees that put the jaw tips
    at (x, y, z) cm. tool_pitch is the gripper angle below horizontal (-90 = straight down).
    piece_yaw turns the jaws (degrees, measured from +x). Raises ValueError if unreachable."""

    # Base angle turns the arm's vertical plane toward the target
    theta1 = math.degrees(math.atan2(y, x))

    # Horizontal distance from the shoulder axis to the target, inside that plane
    r = math.sqrt(x**2 + y**2) - A1

    # The wrist axis sits L4 back along the gripper from the jaw tips
    pitch = math.radians(tool_pitch)
    r_wrist = r - L4 * math.cos(pitch)
    h_wrist = z - D1 - L4 * math.sin(pitch)

    # Law of cosines: elbow angle from the shoulder-to-wrist distance
    cos_theta3 = (r_wrist**2 + h_wrist**2 - L2**2 - L3**2) / (2.0 * L2 * L3)

    if cos_theta3 < -1.0 or cos_theta3 > 1.0:
        raise ValueError("({:.1f}, {:.1f}, {:.1f}) cm is out of reach".format(x, y, z))

    # Negative elbow angle = elbow-up solution, keeps the forearm clear of the table
    t3 = -math.acos(cos_theta3)

    # Shoulder = angle to the wrist point minus the extra angle the elbow bend adds
    t2 = (math.atan2(h_wrist, r_wrist)
          - math.atan2(L3 * math.sin(t3), L2 + L3 * math.cos(t3)))

    theta2 = math.degrees(t2)
    theta3 = math.degrees(t3)

    # Wrist pitch makes up the rest so the gripper ends at tool_pitch
    theta4 = tool_pitch - theta2 - theta3

    # Roll is measured from the arm's direction, so subtract the base rotation
    theta5 = wrap_roll(piece_yaw - theta1)

    return theta1, theta2, theta3, theta4, theta5
