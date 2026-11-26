"""Board geometry: tag layout, pixel <-> board conversion, cells and arm coordinates.

Board frame (mm): origin at the outer top-left corner of the 3x3 grid (next to tag 0),
x along the top edge toward tag 1, y along the left edge toward tag 2.
Cell row 0 is next to the title, col 0 next to tag 0.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

from config import vision_config as cfg


GRID_MM = cfg.CELL_MM * cfg.GRID_CELLS

# Outer grid corner next to each board tag, in board mm
GRID_CORNER_NEAR_TAG = {
    0: (0.0, 0.0),
    1: (GRID_MM, 0.0),
    3: (GRID_MM, GRID_MM),
    2: (0.0, GRID_MM),
}

# Fixed corner order for fitting the setup homography: clockwise from the top-left
SETUP_ORDER = (0, 1, 3, 2)

# Where each tag sits on the sheet, measured once by the setup
LAYOUT_PATH = Path(__file__).resolve().parents[1] / "config" / "board_layout.json"


def transform_points(matrix, points):
    """Apply a 3x3 homography to a list of (x, y) points."""

    array = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    result = cv2.perspectiveTransform(array, matrix)
    return result.reshape(-1, 2)


def assign_clicks_to_tags(clicks, board_tags):
    """Match each clicked grid corner to the nearest board tag, so the click order does not matter.
    Returns {tag_id: (u, v)}, or None if two clicks are nearest the same tag."""

    assigned = {}
    for click in clicks:
        nearest_id = None
        nearest_distance = None

        for tag_id in board_tags:
            # Distance from the click to the tag's centre
            center = board_tags[tag_id].mean(axis=0)
            dx = click[0] - center[0]
            dy = click[1] - center[1]
            distance = math.sqrt(dx**2 + dy**2)

            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_id = tag_id

        if nearest_id in assigned:
            return None
        assigned[nearest_id] = click

    return assigned


def compute_layout(board_tags, clicked_corners):
    """Where each board tag sits on the sheet (mm), from the 4 clicked grid corners.
    clicked_corners: {tag_id: (u, v)} pixel of the grid corner next to that tag."""

    image_points = []
    board_points = []
    for tag_id in SETUP_ORDER:
        image_points.append(clicked_corners[tag_id])
        board_points.append(GRID_CORNER_NEAR_TAG[tag_id])

    # Exact homography from the 4 clicked corners: pixels -> board mm
    pixels_to_board = cv2.getPerspectiveTransform(
        np.array(image_points, dtype=np.float32),
        np.array(board_points, dtype=np.float32))

    # Convert each tag's corners into board mm, so later frames only need the tags
    layout_tags = {}
    for tag_id in board_tags:
        corners_mm = transform_points(pixels_to_board, board_tags[tag_id])

        corner_list = []
        for corner in corners_mm:
            corner_list.append([round(float(corner[0]), 2), round(float(corner[1]), 2)])

        layout_tags[str(tag_id)] = corner_list

    return {
        "dictionary": cfg.ARUCO_DICTIONARY,
        "cell_mm": cfg.CELL_MM,
        "grid_cells": cfg.GRID_CELLS,
        "markers": layout_tags,
    }


def check_tag_sizes(layout):
    """Print each tag's measured side length. Returns True if all are close to BOARD_TAG_MM;
    a wrong size means a mis-clicked corner or a wrong print scale."""

    all_ok = True
    for key in sorted(layout["markers"].keys()):
        corners = layout["markers"][key]

        # Average of the 4 side lengths
        total = 0.0
        for index in range(4):
            next_index = (index + 1) % 4
            dx = corners[next_index][0] - corners[index][0]
            dy = corners[next_index][1] - corners[index][1]
            total = total + math.sqrt(dx**2 + dy**2)
        side = total / 4

        if abs(side - cfg.BOARD_TAG_MM) > cfg.BOARD_TAG_TOLERANCE_MM:
            print("Tag {}: {:.1f} mm (expected {:.0f})".format(key, side, cfg.BOARD_TAG_MM))
            all_ok = False
        else:
            print("Tag {}: {:.1f} mm, OK".format(key, side))

    return all_ok


def save_layout(layout, path=LAYOUT_PATH):
    with open(path, "w") as file:
        json.dump(layout, file, indent=2)
    print("Saved tag layout to", path)


def load_layout(path=LAYOUT_PATH):
    if not Path(path).exists():
        raise FileNotFoundError("No {} yet. Run python vision/viewer.py --setup first.".format(path))

    with open(path) as file:
        return json.load(file)


def board_tag_points(board_tags, layout):
    """Matching board points (mm) and image pixels for every visible tag measured in the layout."""

    board_points = []
    image_points = []
    used_ids = []

    for tag_id in board_tags:
        key = str(tag_id)
        if key not in layout["markers"]:
            continue

        for corner in range(4):
            board_points.append(layout["markers"][key][corner])
            image_points.append(board_tags[tag_id][corner])
        used_ids.append(tag_id)

    return board_points, image_points, used_ids


def compute_homography(board_tags, layout):
    """Fit pixels -> board mm from the visible board tags.
    Returns (homography, mean_error_px, used_ids), or None if too few tags are visible."""

    board_points, image_points, used_ids = board_tag_points(board_tags, layout)
    if len(used_ids) < cfg.MIN_BOARD_TAGS:
        return None

    image_array = np.array(image_points, dtype=np.float32)
    board_array = np.array(board_points, dtype=np.float32)

    # RANSAC ignores a corner that is badly off (threshold in board mm)
    homography, inlier_mask = cv2.findHomography(image_array, board_array, cv2.RANSAC, 3.0)
    if homography is None:
        return None

    # Check the fit: map board corners back to pixels and measure how far they land
    predicted = transform_points(np.linalg.inv(homography), board_points)

    total_error = 0.0
    for index in range(len(image_points)):
        dx = predicted[index][0] - image_points[index][0]
        dy = predicted[index][1] - image_points[index][1]
        total_error = total_error + math.sqrt(dx**2 + dy**2)

    return homography, total_error / len(image_points), used_ids


def pixel_to_board(homography, u, v):
    """Camera pixel of a point on the paper -> (x, y) on the board in mm."""
    point = transform_points(homography, [(u, v)])[0]
    return float(point[0]), float(point[1])


def board_to_pixel(homography, x, y):
    """Board (x, y) in mm on the paper -> camera pixel."""
    point = transform_points(np.linalg.inv(homography), [(x, y)])[0]
    return float(point[0]), float(point[1])


def cell_center_board(row, col):
    """Centre of a cell in board mm."""
    return cfg.CELL_MM * (col + 0.5), cfg.CELL_MM * (row + 0.5)


def cell_at_board(x, y):
    """(row, col) of the cell containing board point (x, y), or None if outside the grid."""

    if x < 0.0 or y < 0.0 or x >= GRID_MM or y >= GRID_MM:
        return None

    row = int(y // cfg.CELL_MM)
    col = int(x // cfg.CELL_MM)
    return row, col


def board_to_arm(x, y):
    """Board point (mm) -> arm frame (x forward, y left, mm), from BOARD_CENTER_ARM_MM and the axes."""

    # Offset from the board centre, turned into the arm's axes
    dx = x - GRID_MM / 2
    dy = y - GRID_MM / 2
    arm_x = cfg.BOARD_CENTER_ARM_MM[0] + dx * cfg.BOARD_X_IN_ARM[0] + dy * cfg.BOARD_Y_IN_ARM[0]
    arm_y = cfg.BOARD_CENTER_ARM_MM[1] + dx * cfg.BOARD_X_IN_ARM[1] + dy * cfg.BOARD_Y_IN_ARM[1]
    return arm_x, arm_y


def arm_to_board(arm_x, arm_y):
    """Arm frame point (mm) -> board mm. The board axes are perpendicular, so this reverses board_to_arm."""

    dx = arm_x - cfg.BOARD_CENTER_ARM_MM[0]
    dy = arm_y - cfg.BOARD_CENTER_ARM_MM[1]
    x = GRID_MM / 2 + dx * cfg.BOARD_X_IN_ARM[0] + dy * cfg.BOARD_X_IN_ARM[1]
    y = GRID_MM / 2 + dx * cfg.BOARD_Y_IN_ARM[0] + dy * cfg.BOARD_Y_IN_ARM[1]
    return x, y


def cell_center_arm(row, col):
    """Centre of a cell in the arm frame (mm), ready for robot.kinematics."""
    x, y = cell_center_board(row, col)
    return board_to_arm(x, y)
