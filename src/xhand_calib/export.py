"""Export calibrated rig data, without pretending it is the old eye/target format."""
from pathlib import Path

import cv2
import numpy as np

from .geometry import check_transform
from .solve import check_result_intrinsics, load_intrinsics
from .storage import check_board, check_camera, check_robot, read_json, stamp, write_json


def rectified_parameters(intr, alpha=0.):
    K = np.asarray(intr["K"], dtype=float)
    D = np.asarray(intr["D"], dtype=float)
    size = (intr["camera"]["width"], intr["camera"]["height"])
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, size, alpha, size, centerPrincipalPoint=False)
    return new_K, [int(x) for x in roi]


def rectify_image(image_path, intrinsics_path, out_path, alpha=0.):
    if not 0 <= alpha <= 1:
        raise ValueError("alpha must be between 0 and 1")
    intr, K, D = load_intrinsics(intrinsics_path)
    out = Path(out_path)
    sidecar = Path(str(out) + ".json")
    if out.exists() or sidecar.exists():
        raise ValueError("Rectified image/sidecar already exists")
    if out.suffix.lower() != ".png":
        raise ValueError("Use a .png output to avoid lossy calibration images")
    image = cv2.imread(str(image_path))
    size = (intr["camera"]["width"], intr["camera"]["height"])
    if image is None or image.shape[1::-1] != size:
        raise ValueError("Input image is unreadable or has a different resolution")
    new_K, roi = rectified_parameters(intr, alpha)
    fixed = cv2.undistort(image, K, D, None, new_K)
    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), fixed):
        raise RuntimeError("Failed to write rectified image")
    result = {"K": new_K, "D": [0.] * 5, "width": size[0], "height": size[1],
              "alpha": alpha, "valid_roi_xywh": roi, "cropped": False,
              "note": "The full-size image is retained; do not crop to ROI without updating cx/cy."}
    write_json(sidecar, result)
    return result


def export_rig(wrist_intrinsics, global_intrinsics, handeye_path, global_path,
               link_to_tcp=None, alpha=0.):
    if not 0 <= alpha <= 1:
        raise ValueError("alpha must be between 0 and 1")
    wrist, _, _ = load_intrinsics(wrist_intrinsics)
    glob, _, _ = load_intrinsics(global_intrinsics)
    hand, ext = read_json(handeye_path), read_json(global_path)
    if hand.get("type") != "handeye" or ext.get("type") != "global_extrinsics":
        raise ValueError("Unexpected extrinsic result type")
    check_camera(hand, wrist)
    check_camera(ext, glob)
    check_board(hand["board"], ext["board"])
    check_robot(hand["robot"], ext["robot"])
    if not np.allclose(hand["T_B_Q"], ext["T_B_Q"], atol=1e-9):
        raise ValueError("Global and wrist results reference different board/base estimates; recompute global")
    # Results must have been computed with these exact K/D values.
    for result, supplied in ((hand, wrist), (ext, glob)):
        check_result_intrinsics(result, supplied)
    cameras = {}
    for name, intr in (("wrist", wrist), ("global", glob)):
        K, roi = rectified_parameters(intr, alpha)
        cameras[name] = {
            "serial": intr["camera"]["serial"],
            "resolution": [intr["camera"]["width"], intr["camera"]["height"]],
            "fps": intr["camera"]["fps"],
            "raw": {"K": intr["K"], "D": intr["D"], "model": intr["distortion_model"]},
            "rectified": {"K": K, "D": [0.] * 5, "alpha": alpha,
                          "valid_roi_xywh": roi, "cropped": False},
        }
    cameras["global"]["T_B_Cg"] = check_transform(ext["T_B_Cg"])
    cameras["wrist"]["T_TCP_Cw"] = check_transform(hand["T_TCP_Cw"])
    cameras["wrist"]["reference_link"] = None
    cameras["wrist"]["T_link_Cw"] = None
    if link_to_tcp:
        link = read_json(link_to_tcp)
        if not link.get("verified"):
            raise ValueError("Link/TCP example is a placeholder; fill and set verified=true")
        if not link.get("reference_link"):
            raise ValueError("reference_link is required")
        cameras["wrist"]["reference_link"] = link["reference_link"]
        cameras["wrist"]["T_link_Cw"] = check_transform(link["T_link_tcp"]) @ cameras["wrist"]["T_TCP_Cw"]
    return {
        "schema_version": 1, "type": "calibrated_camera_rig", "created_utc": stamp(),
        "transform_convention": "T_A_B maps coordinates in B into A; metres; column vectors",
        "optical_axes": "+X right, +Y down, +Z forward (OpenCV)",
        "robot": hand["robot"], "board": hand["board"], "cameras": cameras,
        "ready_for_isaac_wrist": bool(link_to_tcp),
        "note": "Use examples/isaac_camera_rig.py; this is NOT the old eye/target camera JSON. Supply T_world_base explicitly.",
    }
