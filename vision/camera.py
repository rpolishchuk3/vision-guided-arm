"""Camera position from the board tags, and the 3D position of the effector from the gripper tag.

The lens is treated as distortion-free with its centre in the middle of the image. The focal
length is scaled until the camera height matches CAMERA_HEIGHT_MM, so no checkerboard is needed.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from config import vision_config as cfg
from vision.board import board_tag_points


@dataclass(frozen=True)
class CameraModel:
    matrix: np.ndarray          # 3x3 camera matrix (pixels)
    rotation: np.ndarray        # board axes -> camera axes
    translation: np.ndarray     # board origin in camera axes (mm)
    position: np.ndarray        # camera position in board mm

    @property
    def up_sign(self):
        """+1 or -1: the board z direction that points up toward the camera."""
        if self.position[2][0] < 0:
            return -1.0
        return 1.0


@dataclass(frozen=True)
class Effector:
    board_x: float              # board point under the middle of the effector (mm)
    board_y: float
    height_mm: float            # middle of the effector above the paper
    pixel: tuple                # where the middle of the effector appears in the image


def estimate_camera(board_tags, layout, frame_width, frame_height):
    """Where the camera is relative to the board, from the visible board tags, or None."""

    board_points, image_points, used_ids = board_tag_points(board_tags, layout)
    if len(used_ids) < cfg.MIN_BOARD_TAGS:
        return None

    # Board tag corners lie on the paper, at height 0
    object_array = []
    for point in board_points:
        object_array.append([point[0], point[1], 0.0])
    object_array = np.array(object_array, dtype=np.float64)
    image_array = np.array(image_points, dtype=np.float64)

    center_u = frame_width / 2.0
    center_v = frame_height / 2.0

    # Starting guess for the focal length in pixels
    focal = float(frame_width)

    for attempt in range(6):
        matrix = np.array([[focal, 0.0, center_u],
                           [0.0, focal, center_v],
                           [0.0, 0.0, 1.0]])

        ok, rotation_vector, translation = cv2.solvePnP(object_array, image_array, matrix, None)
        if not ok:
            return None

        # Camera position in board coordinates: -R^T * t
        rotation, jacobian = cv2.Rodrigues(rotation_vector)
        position = -rotation.T @ translation
        height = abs(float(position[2][0]))

        # The estimated height grows with the focal length, so scale it to the measured height
        if abs(height - cfg.CAMERA_HEIGHT_MM) < 1.0:
            break
        focal = focal * cfg.CAMERA_HEIGHT_MM / height

    return CameraModel(matrix, rotation, translation, position)


def project_board_point(camera, x, y, height_mm):
    """Pixel where a point height_mm above board point (x, y) appears."""

    point_board = np.array([[x], [y], [camera.up_sign * height_mm]])
    point_camera = camera.rotation @ point_board + camera.translation
    pixel = camera.matrix @ point_camera
    return float(pixel[0][0] / pixel[2][0]), float(pixel[1][0] / pixel[2][0])


def point_below(camera, u, v, height_mm):
    """A point seen at pixel (u, v) that is height_mm above the paper: the board point (mm)
    directly below it. Use this when the height is already known, for example from the arm."""

    # Camera ray through the pixel, first in camera axes, then in board axes
    ray_camera = np.linalg.inv(camera.matrix) @ np.array([[u], [v], [1.0]])
    ray_board = camera.rotation.T @ ray_camera

    # Follow the ray from the camera until it reaches that height
    plane_z = camera.up_sign * height_mm
    distance = (plane_z - camera.position[2][0]) / ray_board[2][0]
    x = camera.position[0][0] + distance * ray_board[0][0]
    y = camera.position[1][0] + distance * ray_board[1][0]
    return float(x), float(y)


def gripper_tag_position(camera, tag_corners):
    """3D centre of the gripper tag in board mm, from its known size (GRIPPER_TAG_MM)."""

    # Tag corners in the tag's own frame, in the order ArUco reports them
    half = cfg.GRIPPER_TAG_MM / 2.0
    tag_points = np.array([[-half, half, 0.0],
                           [half, half, 0.0],
                           [half, -half, 0.0],
                           [-half, -half, 0.0]])

    ok, rotation_vector, tag_center_camera = cv2.solvePnP(
        tag_points, tag_corners.astype(np.float64), camera.matrix, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None

    # Camera axes -> board axes: R^T * (p - t)
    return camera.rotation.T @ (tag_center_camera - camera.translation)


def locate_effector(camera, tag_corners):
    """Middle of the effector, assuming the gripper points straight down: TAG_TO_EFFECTOR_MM
    below the gripper tag. Returns an Effector, or None if the tag pose cannot be found."""

    tag = gripper_tag_position(camera, tag_corners)
    if tag is None:
        return None

    board_x = float(tag[0][0])
    board_y = float(tag[1][0])
    tag_height = camera.up_sign * float(tag[2][0])

    # Straight down from the tag to the middle of the effector
    height = tag_height - cfg.TAG_TO_EFFECTOR_MM
    pixel = project_board_point(camera, board_x, board_y, height)

    return Effector(board_x, board_y, height, pixel)
