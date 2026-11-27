"""Vision tests on a rendered scene: a tilted camera above the board and the gripper tag."""

import math
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import vision_config as cfg
from vision import board as board_geometry
from vision.camera import locate_effector, point_below
from vision.markers import board_tags_only, create_detector, detect_tags
from vision.tracker import BoardTracker


WIDTH = 1280
HEIGHT = 960
FOCAL = 900.0

# Board tag centres on the sheet (mm), outside the grid like the printed board
TAG_CENTERS = {0: (-30.0, -30.0), 1: (210.0, -30.0), 2: (-30.0, 210.0), 3: (210.0, 210.0)}


class Scene:
    """Pinhole camera CAMERA_HEIGHT_MM above the paper, looking at the board at an angle.
    Board z points into the table, so heights above the paper are negative z."""

    def __init__(self):
        self.matrix = np.array([[FOCAL, 0.0, WIDTH / 2], [0.0, FOCAL, HEIGHT / 2], [0.0, 0.0, 1.0]])
        position = np.array([110.0, 330.0, -cfg.CAMERA_HEIGHT_MM])
        look_at = np.array([90.0, 90.0, 0.0])

        forward = look_at - position
        forward = forward / np.linalg.norm(forward)
        right = np.cross(np.array([0.0, -1.0, 0.0]), forward)
        right = right / np.linalg.norm(right)
        down = np.cross(forward, right)

        rotation = np.vstack([right, down, forward])
        self.translation = -rotation @ position
        self.rotation_vector, jacobian = cv2.Rodrigues(rotation)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.ARUCO_DICTIONARY))

    def project(self, points):
        array = np.array(points, dtype=np.float64).reshape(-1, 1, 3)
        pixels, jacobian = cv2.projectPoints(array, self.rotation_vector, self.translation, self.matrix, None)
        return pixels.reshape(-1, 2).astype(np.float32)

    def paste(self, image, pattern, corners_3d):
        height, width = pattern.shape
        source = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
        warp = cv2.getPerspectiveTransform(source, self.project(corners_3d))
        warped = cv2.warpPerspective(pattern, warp, (WIDTH, HEIGHT))
        mask = cv2.warpPerspective(np.full(pattern.shape, 255, np.uint8), warp, (WIDTH, HEIGHT))
        image[mask > 0] = warped[mask > 0]

    def tag_image(self, tag_id):
        # 300 px black square inside a white margin, 380 px in total
        tag = cv2.aruco.generateImageMarker(self.dictionary, tag_id, 300)
        return cv2.copyMakeBorder(tag, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)

    def square(self, center_x, center_y, z, black_mm):
        # Corners of a tag image whose black square is black_mm wide
        half = black_mm / 2 * 380 / 300
        return [[center_x - half, center_y - half, z], [center_x + half, center_y - half, z],
                [center_x + half, center_y + half, z], [center_x - half, center_y + half, z]]

    def render(self, gripper=None, hidden_tags=()):
        """gripper: (x, y, tag height above the paper) in mm, or None."""

        image = np.full((HEIGHT, WIDTH), 110, np.uint8)
        self.paste(image, np.full((300, 300), 255, np.uint8),
                   [[-60, -60, 0], [240, -60, 0], [240, 240, 0], [-60, 240, 0]])

        for tag_id in TAG_CENTERS:
            if tag_id in hidden_tags:
                continue
            center_x, center_y = TAG_CENTERS[tag_id]
            self.paste(image, self.tag_image(tag_id), self.square(center_x, center_y, 0.0, cfg.BOARD_TAG_MM))

        if gripper is not None:
            x, y, tag_height = gripper
            self.paste(image, self.tag_image(cfg.GRIPPER_TAG_ID),
                       self.square(x, y, -tag_height, cfg.GRIPPER_TAG_MM))

        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    def grid_corner_clicks(self):
        clicks = {}
        for tag_id in board_geometry.GRID_CORNER_NEAR_TAG:
            corner_x, corner_y = board_geometry.GRID_CORNER_NEAR_TAG[tag_id]
            clicks[tag_id] = tuple(self.project([[corner_x, corner_y, 0.0]])[0])
        return clicks


def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


class VisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = Scene()
        frame = cls.scene.render()
        cls.board_tags = board_tags_only(detect_tags(create_detector(), frame))
        cls.layout = board_geometry.compute_layout(cls.board_tags, cls.scene.grid_corner_clicks())

    def tracker_for(self, frame):
        tracker = BoardTracker(self.layout)
        tracker.update(frame)
        return tracker

    def test_all_board_tags_found_and_measured_50_mm(self):
        self.assertEqual(sorted(self.board_tags), [0, 1, 2, 3])
        self.assertTrue(board_geometry.check_tag_sizes(self.layout))

    def test_cell_centres_land_on_the_true_pixels(self):
        tracker = self.tracker_for(self.scene.render())
        for row in range(3):
            for col in range(3):
                x, y = board_geometry.cell_center_board(row, col)
                found = board_geometry.board_to_pixel(tracker.homography, x, y)
                true = self.scene.project([[x, y, 0.0]])[0]
                self.assertLess(distance(found, true), 1.5)

    def test_camera_position_matches_the_scene(self):
        tracker = self.tracker_for(self.scene.render())
        position = tracker.camera().position.ravel()
        self.assertLess(abs(position[0] - 110.0), 5.0)
        self.assertLess(abs(position[1] - 330.0), 5.0)
        self.assertLess(abs(position[2] + cfg.CAMERA_HEIGHT_MM), 2.0)

    def test_effector_position_and_cross_on_the_effector(self):
        # Effector 30 mm above cell 2,0; the tag is TAG_TO_EFFECTOR_MM above it
        effector_x, effector_y, effector_height = 30.0, 150.0, 30.0
        tag_height = effector_height + cfg.TAG_TO_EFFECTOR_MM
        tracker = self.tracker_for(self.scene.render(gripper=(effector_x, effector_y, tag_height)))

        effector = tracker.effector()
        self.assertIsNotNone(effector)
        self.assertLess(distance((effector.board_x, effector.board_y), (effector_x, effector_y)), 3.0)
        self.assertLess(abs(effector.height_mm - effector_height), 8.0)
        self.assertEqual(board_geometry.cell_at_board(effector.board_x, effector.board_y), (2, 0))

        # The purple cross must sit on the middle of the effector, not on the tag
        true_pixel = self.scene.project([[effector_x, effector_y, -effector_height]])[0]
        tag_pixel = self.scene.project([[effector_x, effector_y, -tag_height]])[0]
        self.assertLess(distance(effector.pixel, true_pixel), 3.0)
        self.assertGreater(distance(tag_pixel, true_pixel), 20.0)

    def test_point_below_with_a_known_height(self):
        tracker = self.tracker_for(self.scene.render())
        pixel = self.scene.project([[60.0, 140.0, -160.0]])[0]
        x, y = point_below(tracker.camera(), pixel[0], pixel[1], 160.0)
        self.assertLess(distance((x, y), (60.0, 140.0)), 1.0)

    def test_board_kept_when_the_arm_hides_tags(self):
        tracker = self.tracker_for(self.scene.render())
        good = tracker.homography.copy()

        tracker.update(self.scene.render(hidden_tags=(1, 2, 3)))
        self.assertTrue(tracker.status.startswith("Holding last position"))
        self.assertTrue(np.allclose(tracker.homography, good))

        # Two tags are still enough to track
        tracker.update(self.scene.render(hidden_tags=(1, 3)))
        self.assertTrue(tracker.status.startswith("Tracking with tags 0 2"))

    def test_setup_clicks_in_any_order(self):
        clicks = self.scene.grid_corner_clicks()
        shuffled = [clicks[3], clicks[0], clicks[2], clicks[1]]
        assigned = board_geometry.assign_clicks_to_tags(shuffled, self.board_tags)
        for tag_id in clicks:
            self.assertEqual(assigned[tag_id], clicks[tag_id])
        self.assertIsNone(board_geometry.assign_clicks_to_tags([clicks[0], clicks[0]], self.board_tags))

    def test_layout_save_and_load(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "layout.json"
            board_geometry.save_layout(self.layout, path)
            self.assertEqual(board_geometry.load_layout(path), self.layout)

    def test_board_to_arm_frame(self):
        # Middle cell at BOARD_CENTER_ARM_MM; cell 0,0 is far-left from the base, cell 2,2 near-right
        center = board_geometry.cell_center_arm(1, 1)
        far_left = board_geometry.cell_center_arm(0, 0)
        near_right = board_geometry.cell_center_arm(2, 2)
        self.assertEqual(center, cfg.BOARD_CENTER_ARM_MM)
        self.assertEqual(far_left, (center[0] + cfg.CELL_MM, center[1] + cfg.CELL_MM))
        self.assertEqual(near_right, (center[0] - cfg.CELL_MM, center[1] - cfg.CELL_MM))

        arm_x, arm_y = board_geometry.board_to_arm(37.0, 121.0)
        self.assertEqual(board_geometry.arm_to_board(arm_x, arm_y), (37.0, 121.0))


if __name__ == "__main__":
    unittest.main()
