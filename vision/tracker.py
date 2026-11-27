"""Live board tracking: keeps the latest good board position and finds the effector."""

import math
import time

from config import vision_config as cfg
from vision import board as board_geometry
from vision.camera import estimate_camera, locate_effector
from vision.markers import board_tags_only, create_detector, detect_tags


class BoardTracker:
    """Call update() with every camera frame. When the arm hides the board tags, the last good
    position is kept, since the board does not move while the arm works."""

    def __init__(self, layout):
        self.layout = layout
        self.detector = create_detector()
        self.homography = None          # pixels -> board mm
        self.board_tags = None          # board tags of the last good frame
        self.frame_size = None
        self.gripper_tag = None         # gripper tag corners in the latest frame, or None
        self.last_update_time = None
        self.status = "Board not found yet"

    def update(self, frame):
        """Process one frame. Returns all tags found in it."""

        tags = detect_tags(self.detector, frame)
        self.frame_size = (frame.shape[1], frame.shape[0])

        # The gripper tag is only trusted in the frame it was seen in
        self.gripper_tag = tags.get(cfg.GRIPPER_TAG_ID)

        board_tags = board_tags_only(tags)
        result = board_geometry.compute_homography(board_tags, self.layout)

        if result is None:
            self.status = self.holding_message(len(board_tags))
            return tags

        homography, error, used_ids = result
        if error > cfg.MAX_REPROJECTION_PX:
            self.status = "Rejected fit (error {:.1f} px), {}".format(error, self.holding_message(len(board_tags)))
            return tags

        self.report_board_move(homography)

        self.homography = homography
        self.board_tags = board_tags
        self.last_update_time = time.time()

        used_ids.sort()
        id_text = ""
        for tag_id in used_ids:
            id_text = id_text + str(tag_id) + " "
        self.status = "Tracking with tags {}| error {:.1f} px".format(id_text, error)

        return tags

    def report_board_move(self, homography):
        # Compare where the board centre appears now with the last good frame
        if self.homography is None:
            return

        center = board_geometry.GRID_MM / 2
        old_u, old_v = board_geometry.board_to_pixel(self.homography, center, center)
        new_u, new_v = board_geometry.board_to_pixel(homography, center, center)
        shift = math.sqrt((new_u - old_u)**2 + (new_v - old_v)**2)

        if shift > cfg.BOARD_MOVED_PX:
            print("Board moved by {:.0f} px, position updated".format(shift))

    def holding_message(self, visible_count):
        if self.homography is None:
            return "Board not found ({} tag(s) visible, need {})".format(visible_count, cfg.MIN_BOARD_TAGS)

        age = time.time() - self.last_update_time
        return "Holding last position from {:.0f} s ago ({} tag(s) visible)".format(age, visible_count)

    def camera(self):
        """Camera position from the last good board frame, or None."""
        if self.board_tags is None:
            return None
        return estimate_camera(self.board_tags, self.layout, self.frame_size[0], self.frame_size[1])

    def effector(self):
        """Effector seen in the latest frame (vision.camera.Effector), or None."""
        if self.gripper_tag is None:
            return None

        camera = self.camera()
        if camera is None:
            return None
        return locate_effector(camera, self.gripper_tag)
