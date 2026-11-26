"""Find the tic-tac-toe board on the table using its four ArUco markers.

The board sheet has four DICT_4X4_50 markers. With the title at the top:
    id 0 top-left, id 1 top-right, id 2 bottom-left, id 3 bottom-right.

The markers give a homography: a 3x3 matrix that converts camera pixels to
millimetres on the board, even when the camera looks at the board at an angle.

Board frame (mm): origin at the outer top-left corner of the 3x3 grid,
x along the top edge toward marker 1, y along the left edge toward marker 2.

A fifth tag (ID 4) on top of the gripper shows where the gripper is. The tag sits high
above the paper, so its position is corrected using the camera height (CAMERA_HEIGHT_MM).

Usage:
    python board_detection.py --setup      # once: click the 4 outer grid corners
    python board_detection.py              # live tracking from camera 0
    python board_detection.py --camera 1   # another camera
    python board_detection.py --image photo.jpg   # test on a still image
"""

import argparse
import json
import math
import os
import time

import cv2
import numpy as np


# Marker family printed on the board
ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_50

# Tag on top of the gripper
GRIPPER_MARKER_ID = 4

# PLACEHOLDER: height of the camera lens above the table, in mm (measure with a tape)
CAMERA_HEIGHT_MM = 600.0

# PLACEHOLDER: distance from the jaw tips to the gripper tag's surface, along the gripper, in mm.
# With the gripper pointing straight down this is how high the tag is above the jaw tips.
TAG_TO_TIPS_MM = 140.0

# Black square size from ArUco_Board_Tags.pdf, used to check the setup clicks
MARKER_MM = 50.0
MARKER_TOLERANCE_MM = 3.0   # about 4 pixels at a 640x480 overhead view

# Grid size printed on the sheet
CELL_MM = 60.0
GRID_CELLS = 3
GRID_MM = CELL_MM * GRID_CELLS

# Grid corner next to each marker, in board mm
GRID_CORNER_NEAR_MARKER = {
    0: (0.0, 0.0),
    1: (GRID_MM, 0.0),
    3: (GRID_MM, GRID_MM),
    2: (0.0, GRID_MM),
}

# Fixed corner order used when fitting the setup homography: clockwise from the top-left
SETUP_ORDER = [0, 1, 3, 2]

# Fewer than 2 visible markers gives an unreliable homography, so the last good one is kept
MIN_MARKERS = 2

# Reject a homography whose marker corners land further than this from where they were seen
MAX_REPROJECTION_PX = 4.0

# Print a message when the board centre jumps by more than this in the image
BOARD_MOVED_PX = 10.0

# Large images are shrunk to this size for display only
DISPLAY_MAX_PX = 1000

# Marker positions measured during setup are stored next to this file
LAYOUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "board_layout.json")

# Drawing colours (OpenCV uses blue, green, red)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (0, 255, 255)
CYAN = (255, 255, 0)


def create_detector():
    """Set up the ArUco detector, supporting both new (4.7+) and old OpenCV versions."""

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)

    if hasattr(cv2.aruco, "ArucoDetector"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    # Sub-pixel corner refinement makes the corners, and so the homography, more accurate
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    detector = {"dictionary": dictionary, "parameters": parameters, "new_api": None}
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector["new_api"] = cv2.aruco.ArucoDetector(dictionary, parameters)

    return detector


def detect_markers(detector, frame):
    """Return {marker_id: 4x2 array of corner pixels} for the board's markers in the frame.
    Corner order is the marker's own: top-left, top-right, bottom-right, bottom-left."""

    # Detection works on a grey image
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    if detector["new_api"] is not None:
        corners, ids, rejected = detector["new_api"].detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, detector["dictionary"], parameters=detector["parameters"])

    markers = {}
    if ids is None:
        return markers

    # OpenCV versions return ids as shape (N, 1) or (N,), so flatten to a plain list
    id_list = np.array(ids).flatten()

    for index in range(len(id_list)):
        marker_id = int(id_list[index])

        # Ignore markers that are not part of the board or the gripper
        if marker_id not in GRID_CORNER_NEAR_MARKER and marker_id != GRIPPER_MARKER_ID:
            continue

        # A repeated id means a false detection somewhere, so keep only the first one
        if marker_id in markers:
            continue

        markers[marker_id] = corners[index].reshape(4, 2).astype(np.float32)

    return markers


def transform_points(matrix, points):
    """Apply a 3x3 homography to a list of (x, y) points."""

    array = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    result = cv2.perspectiveTransform(array, matrix)
    return result.reshape(-1, 2)


def compute_layout(markers, clicked_corners):
    """Work out where each marker sits on the board, in mm, from the 4 clicked grid corners.
    clicked_corners: {marker_id: (u, v)} pixel of the grid corner next to that marker."""

    image_points = []
    board_points = []
    for marker_id in SETUP_ORDER:
        image_points.append(clicked_corners[marker_id])
        board_points.append(GRID_CORNER_NEAR_MARKER[marker_id])

    # Exact homography from the 4 clicked corners: pixels -> board mm
    pixels_to_board = cv2.getPerspectiveTransform(
        np.array(image_points, dtype=np.float32),
        np.array(board_points, dtype=np.float32))

    # Convert each marker's corners into board mm, so later frames only need the markers
    layout_markers = {}
    for marker_id in markers:
        # The gripper tag moves, so it is never part of the board layout
        if marker_id not in GRID_CORNER_NEAR_MARKER:
            continue

        corners_mm = transform_points(pixels_to_board, markers[marker_id])

        corner_list = []
        for corner in corners_mm:
            corner_list.append([round(float(corner[0]), 2), round(float(corner[1]), 2)])

        layout_markers[str(marker_id)] = corner_list

    return {
        "dictionary": "DICT_4X4_50",
        "cell_mm": CELL_MM,
        "grid_cells": GRID_CELLS,
        "markers": layout_markers,
    }


def check_marker_sizes(layout):
    """Print each marker's measured side length; far from 50 mm means a mis-clicked corner.
    Returns True if every marker is close to 50 mm."""

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

        if abs(side - MARKER_MM) > MARKER_TOLERANCE_MM:
            print("Marker {}: {:.1f} mm (expected {:.0f}) - check the clicks or the print scale".format(
                key, side, MARKER_MM))
            all_ok = False
        else:
            print("Marker {}: {:.1f} mm, OK".format(key, side))

    return all_ok


def save_layout(layout):
    with open(LAYOUT_PATH, "w") as file:
        json.dump(layout, file, indent=2)
    print("Saved marker layout to", LAYOUT_PATH)


def load_layout():
    if not os.path.exists(LAYOUT_PATH):
        raise SystemExit("No board_layout.json yet. Run with --setup first.")

    with open(LAYOUT_PATH) as file:
        return json.load(file)


def compute_homography(markers, layout):
    """Fit pixels -> board mm from all visible markers.
    Returns (homography, mean_error_px, used_ids), or None if too few markers are visible."""

    image_points = []
    board_points = []
    used_ids = []

    for marker_id in markers:
        key = str(marker_id)

        # Only board markers measured during setup can be used
        if marker_id not in GRID_CORNER_NEAR_MARKER or key not in layout["markers"]:
            continue

        # Each marker contributes its 4 corners
        for corner in range(4):
            image_points.append(markers[marker_id][corner])
            board_points.append(layout["markers"][key][corner])

        used_ids.append(marker_id)

    if len(used_ids) < MIN_MARKERS:
        return None

    image_array = np.array(image_points, dtype=np.float32)
    board_array = np.array(board_points, dtype=np.float32)

    # RANSAC ignores a corner that is badly off (threshold in board mm)
    homography, inlier_mask = cv2.findHomography(image_array, board_array, cv2.RANSAC, 3.0)
    if homography is None:
        return None

    # Check the fit: map board corners back to pixels and measure how far they land
    board_to_pixels = np.linalg.inv(homography)
    predicted = transform_points(board_to_pixels, board_points)

    total_error = 0.0
    for index in range(len(image_points)):
        dx = predicted[index][0] - image_points[index][0]
        dy = predicted[index][1] - image_points[index][1]
        total_error = total_error + math.sqrt(dx**2 + dy**2)

    mean_error = total_error / len(image_points)

    return homography, mean_error, used_ids


def pixel_to_board(homography, u, v):
    """Camera pixel -> (x, y) on the board in mm."""
    point = transform_points(homography, [(u, v)])[0]
    return float(point[0]), float(point[1])


def board_to_pixel(homography, x, y):
    """Board (x, y) in mm -> camera pixel."""
    point = transform_points(np.linalg.inv(homography), [(x, y)])[0]
    return float(point[0]), float(point[1])


def cell_center_board(row, col):
    """Centre of a cell in board mm. Row 0 is next to the title, col 0 next to marker 0."""
    return CELL_MM * (col + 0.5), CELL_MM * (row + 0.5)


def cell_at_board(x, y):
    """Return (row, col) of the cell containing board point (x, y), or None if outside the grid."""

    if x < 0.0 or y < 0.0 or x >= GRID_MM or y >= GRID_MM:
        return None

    row = int(y // CELL_MM)
    col = int(x // CELL_MM)
    return row, col


def board_marker_points(markers, layout):
    """Matching 3D board points (mm, on the paper) and image pixels for the visible board tags."""

    object_points = []
    image_points = []
    for marker_id in markers:
        key = str(marker_id)
        if marker_id not in GRID_CORNER_NEAR_MARKER or key not in layout["markers"]:
            continue

        for corner in range(4):
            board_corner = layout["markers"][key][corner]
            object_points.append([board_corner[0], board_corner[1], 0.0])
            image_points.append(markers[marker_id][corner])

    return np.array(object_points, dtype=np.float64), np.array(image_points, dtype=np.float64)


def estimate_camera(markers, layout, frame_width, frame_height):
    """Work out where the camera is relative to the board, from the visible board tags.
    Assumes no lens distortion and the lens centre in the middle of the image. The focal
    length is adjusted until the camera height matches CAMERA_HEIGHT_MM.
    Returns a dict (matrix, rotation, position in board mm) or None."""

    object_points, image_points = board_marker_points(markers, layout)
    if len(object_points) < 4 * MIN_MARKERS:
        return None

    center_u = frame_width / 2.0
    center_v = frame_height / 2.0

    # Starting guess for the focal length in pixels
    focal = float(frame_width)

    for attempt in range(6):
        matrix = np.array([[focal, 0.0, center_u],
                           [0.0, focal, center_v],
                           [0.0, 0.0, 1.0]])

        ok, rotation_vector, translation = cv2.solvePnP(object_points, image_points, matrix, None)
        if not ok:
            return None

        # Camera position in board coordinates: -R^T * t
        rotation, jacobian = cv2.Rodrigues(rotation_vector)
        position = -rotation.T @ translation
        height = abs(float(position[2][0]))

        # The estimated height grows with the focal length, so scale it to match the measured height
        if abs(height - CAMERA_HEIGHT_MM) < 1.0:
            break
        focal = focal * CAMERA_HEIGHT_MM / height

    return {"matrix": matrix, "rotation": rotation, "position": position}


def board_point_at_height(camera, u, v, height_mm):
    """A point seen at pixel (u, v) that is height_mm above the paper: return the board point
    (x, y in mm) directly below it. This removes the shift caused by looking at it from an angle."""

    # Direction of the camera ray through the pixel, first in camera axes, then in board axes
    ray_camera = np.linalg.inv(camera["matrix"]) @ np.array([[u], [v], [1.0]])
    ray_board = camera["rotation"].T @ ray_camera

    camera_x = float(camera["position"][0][0])
    camera_y = float(camera["position"][1][0])
    camera_z = float(camera["position"][2][0])
    ray_x = float(ray_board[0][0])
    ray_y = float(ray_board[1][0])
    ray_z = float(ray_board[2][0])

    # "Above the paper" is the side the camera is on
    if camera_z < 0:
        plane_z = -height_mm
    else:
        plane_z = height_mm

    # Follow the ray from the camera until it reaches that height
    distance = (plane_z - camera_z) / ray_z
    x = camera_x + distance * ray_x
    y = camera_y + distance * ray_y
    return x, y


def gripper_ground_point(camera, gripper_corners, tag_height_mm):
    """Board point (mm) directly below the centre of the gripper tag."""

    center = gripper_corners.mean(axis=0)
    return board_point_at_height(camera, float(center[0]), float(center[1]), tag_height_mm)


class BoardTracker:
    """Keeps the latest good homography. When the arm hides the markers,
    the last one is kept, since the board does not move while the arm works."""

    def __init__(self, layout):
        self.layout = layout
        self.detector = create_detector()
        self.homography = None
        self.last_update_time = None
        self.status = "Board not found yet"
        self.board_markers = None
        self.frame_size = None
        self.gripper = None

    def update(self, frame):
        markers = detect_markers(self.detector, frame)
        self.frame_size = (frame.shape[1], frame.shape[0])

        # The gripper tag is only trusted in the frame it was seen in
        self.gripper = None
        if GRIPPER_MARKER_ID in markers:
            self.gripper = markers[GRIPPER_MARKER_ID]

        result = compute_homography(markers, self.layout)

        if result is None:
            self.status = self.holding_message(len(markers))
            return markers

        homography, error, used_ids = result

        if error > MAX_REPROJECTION_PX:
            self.status = "Rejected fit (error {:.1f} px), {}".format(error, self.holding_message(len(markers)))
            return markers

        # Report when the board has been moved since the last good frame
        if self.homography is not None:
            old_u, old_v = board_to_pixel(self.homography, GRID_MM / 2, GRID_MM / 2)
            new_u, new_v = board_to_pixel(homography, GRID_MM / 2, GRID_MM / 2)
            shift = math.sqrt((new_u - old_u)**2 + (new_v - old_v)**2)
            if shift > BOARD_MOVED_PX:
                print("Board moved by {:.0f} px, position updated".format(shift))

        self.homography = homography
        self.last_update_time = time.time()

        # Keep the board tags of this good frame for estimating the camera position
        self.board_markers = {}
        for marker_id in markers:
            if marker_id in GRID_CORNER_NEAR_MARKER:
                self.board_markers[marker_id] = markers[marker_id]

        used_ids.sort()
        id_text = ""
        for marker_id in used_ids:
            id_text = id_text + str(marker_id) + " "
        self.status = "Tracking with markers {}| error {:.1f} px".format(id_text, error)

        return markers

    def camera(self):
        """Camera position from the last good board frame, or None."""
        if self.board_markers is None:
            return None
        return estimate_camera(self.board_markers, self.layout, self.frame_size[0], self.frame_size[1])

    def holding_message(self, visible_count):
        if self.homography is None:
            return "Board not found ({} marker(s) visible, need {})".format(visible_count, MIN_MARKERS)

        age = time.time() - self.last_update_time
        return "Holding last position from {:.0f} s ago ({} marker(s) visible)".format(age, visible_count)


def draw_markers(image, markers):
    for marker_id in markers:
        corners = markers[marker_id].astype(np.int32)
        cv2.polylines(image, [corners], True, GREEN, 2)

        # Label at the marker's first corner
        corner_u = int(corners[0][0])
        corner_v = int(corners[0][1])
        cv2.putText(image, "id " + str(marker_id), (corner_u, corner_v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)


def draw_grid(image, homography):
    """Draw the board grid and cell centres where the homography says they are."""

    # Grid lines: 4 vertical and 4 horizontal
    for index in range(GRID_CELLS + 1):
        offset = index * CELL_MM

        start = board_to_pixel(homography, offset, 0.0)
        end = board_to_pixel(homography, offset, GRID_MM)
        cv2.line(image, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), CYAN, 2)

        start = board_to_pixel(homography, 0.0, offset)
        end = board_to_pixel(homography, GRID_MM, offset)
        cv2.line(image, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), CYAN, 2)

    # Cell centres labelled row,col
    for row in range(GRID_CELLS):
        for col in range(GRID_CELLS):
            x, y = cell_center_board(row, col)
            u, v = board_to_pixel(homography, x, y)
            cv2.circle(image, (int(u), int(v)), 6, RED, -1)
            cv2.putText(image, "{},{}".format(row, col), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2)


def draw_text(image, lines):
    for index in range(len(lines)):
        cv2.putText(image, lines[index], (15, 35 + 32 * index),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, YELLOW, 2)


def display_scale(frame):
    # Shrink big images (phone photos) so they fit on screen
    height = frame.shape[0]
    width = frame.shape[1]
    largest = max(height, width)

    if largest <= DISPLAY_MAX_PX:
        return 1.0
    return DISPLAY_MAX_PX / largest


def show(window, image, scale):
    if scale != 1.0:
        image = cv2.resize(image, None, fx=scale, fy=scale)
    cv2.imshow(window, image)


class FrameSource:
    """Gives frames from a camera, or the same still image again and again."""

    def __init__(self, camera_index, image_path):
        self.capture = None
        self.image = None

        if image_path is not None:
            self.image = cv2.imread(image_path)
            if self.image is None:
                raise SystemExit("Could not read image " + image_path)
        else:
            self.capture = cv2.VideoCapture(camera_index)
            if not self.capture.isOpened():
                raise SystemExit("Could not open camera {}".format(camera_index))

    def read(self):
        if self.image is not None:
            return self.image.copy()

        ok, frame = self.capture.read()
        if not ok:
            raise SystemExit("Camera stopped sending frames")
        return frame

    def close(self):
        if self.capture is not None:
            self.capture.release()


def assign_clicks_to_markers(clicks, markers):
    """Match each clicked grid corner to the nearest marker, so the click order does not matter.
    Returns {marker_id: (u, v)}, or None if two clicks are nearest the same marker."""

    assigned = {}
    for click in clicks:
        nearest_id = None
        nearest_distance = None

        for marker_id in markers:
            # Distance from the click to the marker's centre
            center = markers[marker_id].mean(axis=0)
            dx = click[0] - center[0]
            dy = click[1] - center[1]
            distance = math.sqrt(dx**2 + dy**2)

            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_id = marker_id

        if nearest_id in assigned:
            return None
        assigned[nearest_id] = click

    return assigned


def run_setup(source):
    """Click the 4 outer grid corners in any order; the marker positions are then saved."""

    window = "Board setup"
    detector = create_detector()
    clicks = []
    state = {"scale": 1.0}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if len(clicks) >= 4:
            return

        # Convert from the shrunk display back to full image pixels
        click = (x / state["scale"], y / state["scale"])
        clicks.append(click)
        print("Corner {} clicked at pixel ({:.0f}, {:.0f})".format(len(clicks), click[0], click[1]))

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    print("Move the arm out of the camera view so all 4 markers are visible.")
    print("Click the 4 outer corners of the 3x3 grid, in any order.")
    print("Keys (press them in the camera window, not the terminal): s = save, r = redo, q = quit")

    while True:
        frame = source.read()
        state["scale"] = display_scale(frame)
        markers = detect_markers(detector, frame)

        image = frame.copy()
        draw_markers(image, markers)

        # Show the corners clicked so far
        for click in clicks:
            cv2.circle(image, (int(click[0]), int(click[1])), 6, RED, -1)

        visible = []
        for marker_id in sorted(markers.keys()):
            if marker_id in GRID_CORNER_NEAR_MARKER:
                visible.append(marker_id)

        lines = ["Board markers visible: {}".format(visible)]
        if len(clicks) < 4:
            lines.append("Click the outer grid corners: {} of 4".format(len(clicks)))
        else:
            lines.append("Press s to save, r to redo")
        draw_text(image, lines)

        show(window, image, state["scale"])
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            print("Quit without saving")
            return

        if key == ord("r"):
            clicks.clear()
            print("Clicks cleared")

        if key == ord("s"):
            if len(clicks) < 4:
                print("Click all 4 corners first")
                continue
            if len(visible) < len(GRID_CORNER_NEAR_MARKER):
                print("All 4 board markers must be visible to save; visible now:", visible)
                continue

            board_markers = {}
            for marker_id in visible:
                board_markers[marker_id] = markers[marker_id]

            corners = assign_clicks_to_markers(clicks, board_markers)
            if corners is None:
                print("Two clicks are next to the same marker. Press r and click each outer corner once.")
                continue

            layout = compute_layout(board_markers, corners)

            # Each printed marker is 50 mm, so a wrong click shows up as a wrong size
            if not check_marker_sizes(layout):
                print("Not saved. Press r and click the corners again, exactly on the line crossings.")
                continue

            save_layout(layout)
            return


def draw_gripper(image, tracker, tip_height_cm):
    """Mark the board point under the gripper (gripper pointing straight down) and return a status line."""

    if tracker.gripper is None:
        return "Gripper tag not seen"

    camera = tracker.camera()
    if camera is None or tracker.homography is None:
        return "Gripper tag seen, board not found"

    tag_height_mm = tip_height_cm * 10.0 + TAG_TO_TIPS_MM
    x, y = gripper_ground_point(camera, tracker.gripper, tag_height_mm)

    u, v = board_to_pixel(tracker.homography, x, y)
    cv2.drawMarker(image, (int(u), int(v)), (255, 0, 255), cv2.MARKER_CROSS, 30, 3)

    cell = cell_at_board(x, y)
    if cell is None:
        return "Gripper over ({:.0f}, {:.0f}) mm, outside the grid".format(x, y)
    return "Gripper over ({:.0f}, {:.0f}) mm, cell {},{}".format(x, y, cell[0], cell[1])


def run_tracking(source, tip_height_cm):
    """Track the board live. Click anywhere to print its board position and cell."""

    window = "Board tracking"
    tracker = BoardTracker(load_layout())
    state = {"scale": 1.0}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if tracker.homography is None:
            print("Board not found yet")
            return

        u = x / state["scale"]
        v = y / state["scale"]
        board_x, board_y = pixel_to_board(tracker.homography, u, v)
        cell = cell_at_board(board_x, board_y)

        if cell is None:
            print("Pixel ({:.0f}, {:.0f}) -> board ({:.1f}, {:.1f}) mm, outside the grid".format(
                u, v, board_x, board_y))
        else:
            print("Pixel ({:.0f}, {:.0f}) -> board ({:.1f}, {:.1f}) mm, cell {},{}".format(
                u, v, board_x, board_y, cell[0], cell[1]))

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    print("Click a point to see its board position. Press q to quit.")

    while True:
        frame = source.read()
        state["scale"] = display_scale(frame)
        markers = tracker.update(frame)

        image = frame.copy()
        draw_markers(image, markers)
        if tracker.homography is not None:
            draw_grid(image, tracker.homography)
        gripper_line = draw_gripper(image, tracker, tip_height_cm)
        draw_text(image, [tracker.status, gripper_line, "Click: board position | q: quit"])

        show(window, image, state["scale"])
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            return


def main():
    parser = argparse.ArgumentParser(description="Detect the tic-tac-toe board with ArUco markers")
    parser.add_argument("--setup", action="store_true", help="click the grid corners and save marker positions")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index (default 0)")
    parser.add_argument("--image", help="use a still image instead of the camera")
    parser.add_argument("--tip-height", type=float, default=3.0,
                        help="height of the jaw tips above the paper in cm, for the gripper marker (default 3)")
    args = parser.parse_args()

    source = FrameSource(args.camera, args.image)
    try:
        if args.setup:
            run_setup(source)
        else:
            run_tracking(source, args.tip_height)
    finally:
        source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
