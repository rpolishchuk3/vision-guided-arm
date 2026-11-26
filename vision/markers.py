"""ArUco tag detection, for both the new (4.7+) and the old OpenCV interface."""

import cv2
import numpy as np

from config import vision_config as cfg


# Board tags 0-3, plus the tag on the gripper
BOARD_TAG_IDS = (0, 1, 2, 3)
KNOWN_TAG_IDS = BOARD_TAG_IDS + (cfg.GRIPPER_TAG_ID,)


def create_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.ARUCO_DICTIONARY))

    if hasattr(cv2.aruco, "ArucoDetector"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    # Sub-pixel corners make every position computed from them more accurate
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    detector = {"dictionary": dictionary, "parameters": parameters, "new_api": None}
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector["new_api"] = cv2.aruco.ArucoDetector(dictionary, parameters)

    return detector


def detect_tags(detector, frame):
    """Return {tag_id: 4x2 array of corner pixels} for the board and gripper tags.
    Corner order is the tag's own: top-left, top-right, bottom-right, bottom-left."""

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

    tags = {}
    if ids is None:
        return tags

    # OpenCV versions return ids as shape (N, 1) or (N,), so flatten to a plain list
    id_list = np.array(ids).flatten()

    for index in range(len(id_list)):
        tag_id = int(id_list[index])

        # Ignore tags that belong to something else
        if tag_id not in KNOWN_TAG_IDS:
            continue

        # A repeated id means a false detection somewhere, so keep only the first one
        if tag_id in tags:
            continue

        tags[tag_id] = corners[index].reshape(4, 2).astype(np.float32)

    return tags


def board_tags_only(tags):
    """The board tags from a detection, without the gripper tag."""

    board = {}
    for tag_id in tags:
        if tag_id in BOARD_TAG_IDS:
            board[tag_id] = tags[tag_id]
    return board
