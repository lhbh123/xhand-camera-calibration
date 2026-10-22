from pathlib import Path

import cv2
import numpy as np

from .geometry import from_pose


def make_board(spec):
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError("Install one opencv-contrib-python >=4.10,<4.12 package")
    if not (0 < spec["marker_length_m"] < spec["square_length_m"]):
        raise ValueError("Marker length must be positive and smaller than square length")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    return cv2.aruco.CharucoBoard(
        (spec["squares_x"], spec["squares_y"]),
        spec["square_length_m"], spec["marker_length_m"], dictionary,
    )


def detect(image, spec, board=None):
    board = make_board(spec) if board is None else board
    detector = cv2.aruco.CharucoDetector(board)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < spec.get("min_corners", 12):
        return None
    # Refine chess intersections directly, in addition to marker-based interpolation.
    # A small window avoids pulling corners toward nearby marker borders.
    cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
    object_points, image_points = board.matchImagePoints(corners, ids)
    obj = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(axis=0)) < 2:
        return None
    return {"object": obj, "image": np.asarray(image_points, dtype=np.float64).reshape(-1, 2),
            "corners": corners, "ids": ids}


def project(obj, T_camera_board, K, D):
    from scipy.spatial.transform import Rotation
    uv, _ = cv2.projectPoints(obj, Rotation.from_matrix(T_camera_board[:3, :3]).as_rotvec(),
                             T_camera_board[:3, 3], K, D)
    return uv.reshape(-1, 2)


def estimate_pose(observation, K, D):
    obj, img = observation["object"], observation["image"]
    ok, r, t = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError("PnP failed")
    T = from_pose(np.r_[t.ravel(), r.ravel()])
    if np.any((obj @ T[:3, :3].T + T[:3, 3])[:, 2] <= 0):
        raise ValueError("PnP placed board behind camera")
    errors = np.linalg.norm(project(obj, T, K, D) - img, axis=1)
    return T, float(np.sqrt(np.mean(errors ** 2)))


def observations(session_path, meta, rows):
    board = make_board(meta["board"])
    accepted, rejected = [], []
    width, height = meta["camera"]["width"], meta["camera"]["height"]
    for row in rows:
        filename = Path(session_path) / row["image"]
        image = cv2.imread(str(filename))
        if image is None:
            raise ValueError(f"Cannot read {filename}")
        if image.shape[:2] != (height, width):
            raise ValueError(f"Unexpected image dimensions: {filename}")
        obs = detect(image, meta["board"], board)
        if obs is None:
            rejected.append({"sample_id": row["sample_id"], "reason": "insufficient or collinear ChArUco corners"})
        else:
            accepted.append({**obs, "sample": row})
    return accepted, rejected
