"""Camera, ArUco tag and board settings for the vision code. Lengths in millimetres."""

# OpenCV index of the overhead camera
CAMERA_INDEX = 0

# PLACEHOLDER: height of the camera lens above the table (measure with a tape)
CAMERA_HEIGHT_MM = 462.5

# Marker family of every tag (ArUco_Board_Tags.pdf)
ARUCO_DICTIONARY = "DICT_4X4_50"

# Board tags: black square size, and how far a measured tag may be from it after setup
BOARD_TAG_MM = 48.0
BOARD_TAG_TOLERANCE_MM = 4.0

# Printed grid
CELL_MM = 60.0
GRID_CELLS = 3

# Tag on top of the gripper
GRIPPER_TAG_ID = 4

# PLACEHOLDER: black square size of the gripper tag
GRIPPER_TAG_MM = 30.0

# PLACEHOLDER: from the gripper tag's surface to the middle of the effector (the point between
# the jaws that the arm positions), measured along the gripper with the jaws closed on a piece
TAG_TO_EFFECTOR_MM = 162.5

# Board placement in the arm frame of robot/kinematics.py (x forward, y left, mm):
# centre of the middle cell, and the directions of the board axes
BOARD_CENTER_ARM_MM = (190.0, 0.0)
BOARD_X_IN_ARM = (0.0, -1.0)    # board x (tag 0 -> tag 1) points to the arm's right
BOARD_Y_IN_ARM = (-1.0, 0.0)    # board y (tag 0 -> tag 2) points toward the base

# Board tracking
MIN_BOARD_TAGS = 2              # fewer visible tags: keep the last good board position
MAX_REPROJECTION_PX = 4.0       # reject a board fit whose tag corners land further off than this
BOARD_MOVED_PX = 10.0           # report a board move when its centre shifts more than this
