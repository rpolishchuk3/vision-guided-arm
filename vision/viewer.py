"""Camera window for the board: one-time setup, and live tracking with the effector cross.

    python vision/viewer.py --setup           click the 4 outer grid corners once
    python vision/viewer.py                   live tracking from CAMERA_INDEX
    python vision/viewer.py --camera 1        another camera
    python vision/viewer.py --image photo.jpg test on a still image

Keys are read by the camera window, not the terminal.
"""

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import vision_config as cfg
from vision import board as board_geometry
from vision.markers import BOARD_TAG_IDS, board_tags_only, create_detector, detect_tags
from vision.tracker import BoardTracker


# Large images are shrunk to this size for display only
DISPLAY_MAX_PX = 1000

# Drawing colours (OpenCV uses blue, green, red)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (0, 255, 255)
CYAN = (255, 255, 0)
PURPLE = (255, 0, 255)


class FrameSource:
    """Frames from a camera, or the same still image again and again."""

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


def display_scale(frame):
    # Shrink big images (phone photos) so they fit on screen
    largest = max(frame.shape[0], frame.shape[1])
    if largest <= DISPLAY_MAX_PX:
        return 1.0
    return DISPLAY_MAX_PX / largest


def show(window, image, scale):
    if scale != 1.0:
        image = cv2.resize(image, None, fx=scale, fy=scale)
    cv2.imshow(window, image)


def draw_tags(image, tags):
    for tag_id in tags:
        corners = tags[tag_id].astype(np.int32)
        cv2.polylines(image, [corners], True, GREEN, 2)

        # Label at the tag's first corner
        cv2.putText(image, "id " + str(tag_id), (int(corners[0][0]), int(corners[0][1]) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)


def draw_grid(image, homography):
    """Draw the board grid and cell centres where the homography says they are."""

    # 4 vertical and 4 horizontal grid lines
    for index in range(cfg.GRID_CELLS + 1):
        offset = index * cfg.CELL_MM

        start = board_geometry.board_to_pixel(homography, offset, 0.0)
        end = board_geometry.board_to_pixel(homography, offset, board_geometry.GRID_MM)
        cv2.line(image, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), CYAN, 2)

        start = board_geometry.board_to_pixel(homography, 0.0, offset)
        end = board_geometry.board_to_pixel(homography, board_geometry.GRID_MM, offset)
        cv2.line(image, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), CYAN, 2)

    # Cell centres labelled row,col
    for row in range(cfg.GRID_CELLS):
        for col in range(cfg.GRID_CELLS):
            x, y = board_geometry.cell_center_board(row, col)
            u, v = board_geometry.board_to_pixel(homography, x, y)
            cv2.circle(image, (int(u), int(v)), 6, RED, -1)
            cv2.putText(image, "{},{}".format(row, col), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2)


def draw_effector(image, tracker):
    """Purple cross on the middle of the effector, and a status line about it."""

    if tracker.gripper_tag is None:
        return "Gripper tag not seen"

    effector = tracker.effector()
    if effector is None:
        return "Gripper tag seen, board not found"

    u = int(round(effector.pixel[0]))
    v = int(round(effector.pixel[1]))
    cv2.drawMarker(image, (u, v), PURPLE, cv2.MARKER_CROSS, 30, 3)

    arm_x, arm_y = board_geometry.board_to_arm(effector.board_x, effector.board_y)
    position = "board ({:.0f}, {:.0f}) mm, arm ({:.0f}, {:.0f}) mm, {:.0f} mm above paper".format(
        effector.board_x, effector.board_y, arm_x, arm_y, effector.height_mm)

    cell = board_geometry.cell_at_board(effector.board_x, effector.board_y)
    if cell is None:
        return "Effector outside the grid: " + position
    return "Effector over cell {},{}: {}".format(cell[0], cell[1], position)


def draw_text(image, lines):
    for index in range(len(lines)):
        cv2.putText(image, lines[index], (15, 35 + 32 * index), cv2.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2)


def run_setup(source):
    """Click the 4 outer grid corners in any order; the tag positions on the sheet are then saved."""

    window = "Board setup"
    detector = create_detector()
    clicks = []
    state = {"scale": 1.0}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or len(clicks) >= 4:
            return

        # Convert from the shrunk display back to full image pixels
        click = (x / state["scale"], y / state["scale"])
        clicks.append(click)
        print("Corner {} clicked at pixel ({:.0f}, {:.0f})".format(len(clicks), click[0], click[1]))

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    print("Move the arm out of the camera view so all 4 board tags are visible.")
    print("Click the 4 outer corners of the 3x3 grid, in any order.")
    print("Keys (in the camera window): s = save, r = redo, q = quit")

    while True:
        frame = source.read()
        state["scale"] = display_scale(frame)
        board_tags = board_tags_only(detect_tags(detector, frame))

        image = frame.copy()
        draw_tags(image, board_tags)
        for click in clicks:
            cv2.circle(image, (int(click[0]), int(click[1])), 6, RED, -1)

        visible = sorted(board_tags.keys())
        lines = ["Board tags visible: {}".format(visible)]
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
            if len(visible) < len(BOARD_TAG_IDS):
                print("All 4 board tags must be visible to save; visible now:", visible)
                continue

            corners = board_geometry.assign_clicks_to_tags(clicks, board_tags)
            if corners is None:
                print("Two clicks are next to the same tag. Press r and click each outer corner once.")
                continue

            layout = board_geometry.compute_layout(board_tags, corners)

            # Each printed tag is 50 mm, so a wrong click shows up as a wrong size
            if not board_geometry.check_tag_sizes(layout):
                print("Not saved. Press r and click the corners again, exactly on the line crossings.")
                continue

            board_geometry.save_layout(layout)
            return


def run_tracking(source):
    """Track the board live. Click a point on the paper to print its board and arm position."""

    window = "Board tracking"
    tracker = BoardTracker(board_geometry.load_layout())
    state = {"scale": 1.0}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if tracker.homography is None:
            print("Board not found yet")
            return

        u = x / state["scale"]
        v = y / state["scale"]
        board_x, board_y = board_geometry.pixel_to_board(tracker.homography, u, v)
        arm_x, arm_y = board_geometry.board_to_arm(board_x, board_y)
        cell = board_geometry.cell_at_board(board_x, board_y)

        text = "Pixel ({:.0f}, {:.0f}) -> board ({:.1f}, {:.1f}) mm, arm ({:.1f}, {:.1f}) mm".format(
            u, v, board_x, board_y, arm_x, arm_y)
        if cell is None:
            print(text + ", outside the grid")
        else:
            print(text + ", cell {},{}".format(cell[0], cell[1]))

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    print("Click a point on the paper to see its position. Press q to quit.")

    while True:
        frame = source.read()
        state["scale"] = display_scale(frame)
        tags = tracker.update(frame)

        image = frame.copy()
        draw_tags(image, tags)
        if tracker.homography is not None:
            draw_grid(image, tracker.homography)
        effector_line = draw_effector(image, tracker)
        draw_text(image, [tracker.status, effector_line, "Click: board position | q: quit"])

        show(window, image, state["scale"])
        if cv2.waitKey(30) & 0xFF == ord("q"):
            return


def main():
    parser = argparse.ArgumentParser(description="Board and effector tracking with the overhead camera")
    parser.add_argument("--setup", action="store_true", help="click the grid corners and save the tag layout")
    parser.add_argument("--camera", type=int, default=cfg.CAMERA_INDEX, help="OpenCV camera index")
    parser.add_argument("--image", help="use a still image instead of the camera")
    args = parser.parse_args()

    source = FrameSource(args.camera, args.image)
    try:
        if args.setup:
            run_setup(source)
        else:
            run_tracking(source)
    finally:
        source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
