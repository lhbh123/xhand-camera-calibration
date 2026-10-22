"""Offline calibration. All positions are metres; optical axes follow OpenCV."""
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .board import estimate_pose, observations, project
from .geometry import check_transform, from_pose, inverse, mean_transform, pose_error, summary, to_pose
from .storage import check_board, check_camera, check_robot, load_session, read_json, stamp


def load_intrinsics(path):
    data = read_json(path)
    if data.get("type") != "opencv_intrinsics" or data.get("distortion_model") != "opencv_brown_conrady":
        raise ValueError("Expected an OpenCV intrinsic result, not RealSense factory coefficients")
    K = np.asarray(data["K"], dtype=float)
    D = np.asarray(data["D"], dtype=float)
    if K.shape != (3, 3) or D.size != 5 or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise ValueError("Invalid intrinsic matrix/distortion coefficients")
    if min(K[0, 0], K[1, 1]) <= 0 or not np.allclose(K[2], [0, 0, 1]):
        raise ValueError("Invalid intrinsic matrix")
    return data, K, D


def check_result_intrinsics(result, intr):
    snapshot = result.get("intrinsics")
    if snapshot is None or not np.allclose(snapshot["K"], intr["K"], rtol=0, atol=1e-10) or \
            not np.allclose(np.ravel(snapshot["D"]), np.ravel(intr["D"]), rtol=0, atol=1e-10):
        raise ValueError("Intrinsics changed after extrinsic calibration; recompute extrinsics")


def intrinsic_calibration(session, holdout_every=5):
    if holdout_every < 3:
        raise ValueError("holdout_every must be >= 3")
    path, meta, rows = load_session(session)
    if meta["kind"] != "intrinsics":
        raise ValueError("Use an intrinsics capture session")
    obs, rejected = observations(path, meta, rows)
    if len(obs) < 12:
        raise ValueError(f"Need >=12 detected views (recommended 30-50); got {len(obs)}")
    train = [o for i, o in enumerate(obs) if i % holdout_every]
    held = [o for i, o in enumerate(obs) if not i % holdout_every]
    size = (meta["camera"]["width"], meta["camera"]["height"])
    rms, K, D, _, _ = cv2.calibrateCamera(
        [o["object"].astype(np.float32) for o in train],
        [o["image"].astype(np.float32) for o in train], size, None, None,
    )
    if not np.isfinite(K).all() or not np.isfinite(D).all() or min(K[0, 0], K[1, 1]) <= 0:
        raise ValueError("Intrinsic calibration returned invalid parameters")
    details = []
    for group, views in (("train", train), ("holdout", held)):
        for view in views:
            _, err = estimate_pose(view, K, D)
            details.append({"sample_id": view["sample"]["sample_id"], "split": group,
                            "corners": len(view["object"]), "reprojection_rms_px": err})
    all_uv = np.concatenate([o["image"] for o in obs])
    warnings = []
    held_errors = [v["reprojection_rms_px"] for v in details if v["split"] == "holdout"]
    if max(held_errors) > 1:
        warnings.append("At least one holdout view exceeds 1 px RMS; inspect images and model")
    if not (0 <= K[0, 2] < size[0] and 0 <= K[1, 2] < size[1]):
        warnings.append("Principal point lies outside the image; inspect view coverage")
    if len(obs) < 25:
        warnings.append("Fewer than 25 detected views; collect more varied distances/tilts")
    if np.max(np.abs(D.ravel()[[0, 1, 4]])) > 1:
        warnings.append("Large radial distortion coefficients; inspect edge coverage and held-out images for overfitting")
    return {
        "schema_version": 1, "type": "opencv_intrinsics", "created_utc": stamp(),
        "source_session": str(path.resolve()), "camera": meta["camera"], "board": meta["board"],
        "distortion_model": "opencv_brown_conrady", "coefficient_order": ["k1", "k2", "p1", "p2", "k3"],
        "K": K, "D": D.ravel(), "training_rms_px": float(rms),
        "holdout_rms_px": summary(held_errors), "views": details, "rejected": rejected,
        "corner_bounds_normalized": [all_uv.min(axis=0) / size, all_uv.max(axis=0) / size],
        "warnings": warnings,
        "note": "Holdout PnP refits board pose only. It does not refit K or D; low RMS alone does not prove metric accuracy.",
    }


def pose_views(session, intrinsics_path, max_reprojection=2.):
    path, meta, rows = load_session(session)
    intr, K, D = load_intrinsics(intrinsics_path)
    check_camera(meta, intr)
    obs, rejected = observations(path, meta, rows)
    accepted = []
    for o in obs:
        try:
            T, error = estimate_pose(o, K, D)
        except (ValueError, cv2.error) as exc:
            rejected.append({"sample_id": o["sample"]["sample_id"], "reason": str(exc)})
            continue
        if error > max_reprojection:
            rejected.append({"sample_id": o["sample"]["sample_id"],
                             "reason": "PnP reprojection exceeds limit", "reprojection_rms_px": error})
        else:
            accepted.append({**o, "T_camera_board": T, "pnp_rms_px": error})
    return path, meta, intr, K, D, accepted, rejected


def motion_excitation(transforms):
    vectors = []
    for i, A in enumerate(transforms):
        for B in transforms[i + 1:]:
            vectors.append(Rotation.from_matrix((inverse(A) @ B)[:3, :3]).as_rotvec())
    v = np.asarray(vectors)
    s = np.linalg.svd(v, compute_uv=False)
    result = {"max_relative_rotation_deg": float(np.degrees(np.linalg.norm(v, axis=1).max())),
              "rotation_axis_singular_values": s.tolist(),
              "second_to_first_axis_ratio": float(s[1] / max(s[0], 1e-12))}
    if result["max_relative_rotation_deg"] < 15 or result["second_to_first_axis_ratio"] < .1:
        raise ValueError(f"Insufficient multi-axis rotation for hand-eye calibration: {result}")
    return result


def _robot_transform(view):
    row = view["sample"]
    if not row.get("stationary_checked") or "robot" not in row:
        raise ValueError("Hand-eye samples must include stationary_checked and robot.T_B_TCP")
    return check_transform(row["robot"]["T_B_TCP"])


def handeye_calibration(session, intrinsics_path, method="park", max_reprojection=2., refine=True):
    path, meta, intr, K, D, views, rejected = pose_views(session, intrinsics_path, max_reprojection)
    if meta["kind"] != "handeye" or meta["camera_name"] != "wrist":
        raise ValueError("Use a wrist handeye session")
    if not meta.get("robot", {}).get("tcp_configuration_verified"):
        raise ValueError("TCP configuration is not verified in session metadata")
    if len(views) < 10:
        raise ValueError(f"Need >=10 valid hand-eye views (recommended 25-40); got {len(views)}")
    A = [_robot_transform(v) for v in views]
    B = [v["T_camera_board"] for v in views]
    excitation = motion_excitation(A)
    methods = {"park": cv2.CALIB_HAND_EYE_PARK, "tsai": cv2.CALIB_HAND_EYE_TSAI,
               "horaud": cv2.CALIB_HAND_EYE_HORAUD, "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
               "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS}
    r, t = cv2.calibrateHandEye([a[:3, :3] for a in A], [a[:3, 3] for a in A],
                              [b[:3, :3] for b in B], [b[:3, 3] for b in B], method=methods[method])
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = r, t.ravel()
    check_transform(X)
    Y = mean_transform([a @ X @ b for a, b in zip(A, B)])
    initial = {"T_TCP_Cw": X.copy(), "T_B_Q": Y.copy()}
    refinement = {"enabled": refine}
    if refine:
        def residual(p):
            x, y = from_pose(p[:6]), from_pose(p[6:])
            result = []
            for a, v in zip(A, views):
                predicted = inverse(x) @ inverse(a) @ y
                result.append((project(v["object"], predicted, K, D) - v["image"]).ravel())
            return np.concatenate(result)
        opt = least_squares(residual, np.r_[to_pose(X), to_pose(Y)], loss="soft_l1",
                            f_scale=1., x_scale="jac", max_nfev=200)
        if not opt.success or not np.isfinite(opt.x).all():
            raise ValueError(f"Hand-eye refinement did not converge: {opt.message}")
        X, Y = from_pose(opt.x[:6]), from_pose(opt.x[6:])
        refinement.update({"success": bool(opt.success), "evaluations": opt.nfev,
                           "cost": float(opt.cost), "message": opt.message})
    rows = []
    for a, v in zip(A, views):
        predicted = inverse(X) @ inverse(a) @ Y
        if np.any((v["object"] @ predicted[:3, :3].T + predicted[:3, 3])[:, 2] <= 0):
            raise ValueError("Hand-eye refinement placed board behind the camera")
        err = np.linalg.norm(project(v["object"], predicted, K, D) - v["image"], axis=1)
        rows.append({"sample_id": v["sample"]["sample_id"],
                     **pose_error(Y, a @ X @ v["T_camera_board"]),
                     "predicted_reprojection_rms_px": float(np.sqrt(np.mean(err ** 2)))})
    return {
        "schema_version": 1, "type": "handeye", "created_utc": stamp(),
        "source_session": str(path.resolve()), "camera": meta["camera"], "board": meta["board"],
        "robot": meta["robot"], "intrinsics_source": str(Path(intrinsics_path).resolve()),
        "intrinsics": {"K": K, "D": D.ravel()},
        "T_TCP_Cw": X, "T_B_Q": Y, "method": method, "initial": initial,
        "refinement": refinement, "motion_excitation": excitation,
        "training_consistency": _consistency(rows), "views": rows, "rejected": rejected,
        "warnings": ["Training consistency is not independent validation. Collect a separate validation session."] +
                    (["Fewer than 25 valid hand-eye views"] if len(views) < 25 else []),
    }


def _consistency(rows):
    return {key: summary([r[key] for r in rows]) for key in
            ("translation_m", "rotation_deg", "predicted_reprojection_rms_px")}


def global_calibration(session, intrinsics_path, handeye_path, max_reprojection=2.):
    path, meta, intr, K, D, views, rejected = pose_views(session, intrinsics_path, max_reprojection)
    hand = read_json(handeye_path)
    if hand.get("type") != "handeye":
        raise ValueError("Expected hand-eye calibration result")
    if meta["kind"] != "global" or meta["camera_name"] != "global":
        raise ValueError("Use a global camera extrinsic session")
    check_board(meta["board"], hand["board"])
    if len(views) < 3:
        raise ValueError("Need >=3 global views of the same stationary board")
    # Global camera and board are both static: fit one camera->board transform.
    C_Q = mean_transform([v["T_camera_board"] for v in views])
    def residual(p):
        T = from_pose(p)
        return np.concatenate([(project(v["object"], T, K, D) - v["image"]).ravel() for v in views])
    opt = least_squares(residual, to_pose(C_Q), loss="soft_l1", f_scale=1., max_nfev=100)
    if not opt.success:
        raise ValueError(f"Global pose refinement failed: {opt.message}")
    C_Q = from_pose(opt.x)
    Y = check_transform(hand["T_B_Q"])
    G = Y @ inverse(C_Q)
    per_view = []
    for v in views:
        uv = project(v["object"], C_Q, K, D)
        error = np.linalg.norm(uv - v["image"], axis=1)
        per_view.append({"sample_id": v["sample"]["sample_id"],
                         **pose_error(G, Y @ inverse(v["T_camera_board"])),
                         "predicted_reprojection_rms_px": float(np.sqrt(np.mean(error ** 2)))})
    return {
        "schema_version": 1, "type": "global_extrinsics", "created_utc": stamp(),
        "source_session": str(path.resolve()), "camera": meta["camera"], "board": meta["board"],
        "intrinsics_source": str(Path(intrinsics_path).resolve()),
        "intrinsics": {"K": K, "D": D.ravel()},
        "handeye_source": str(Path(handeye_path).resolve()), "robot": hand["robot"],
        "T_B_Cg": G, "T_B_Q": Y, "views": per_view, "rejected": rejected,
        "training_consistency": _consistency(per_view),
        "warnings": ["Absolute accuracy inherits the hand-eye board/base estimate. Same static board required.",
                     "Repeated images at one fixed pose do not independently verify scale or all camera parameters."],
    }


def validate(session, intrinsics_path, result_path, max_reprojection=2.,
             max_translation=.005, max_rotation=2., overlay_dir=None):
    path, meta, intr, K, D, views, rejected = pose_views(session, intrinsics_path, max_reprojection)
    result = read_json(result_path)
    if meta["kind"] != "validation":
        raise ValueError("Collect a separate --kind validation session")
    if path.resolve() == Path(result["source_session"]).resolve():
        raise ValueError("Validation must not reuse the training session")
    if len(views) < 5:
        raise ValueError(f"Need >=5 detected validation views; got {len(views)}")
    check_camera(meta, result)
    check_result_intrinsics(result, intr)
    check_board(meta["board"], result["board"])
    Y = check_transform(result["T_B_Q"])
    rows = []
    if overlay_dir:
        overlay_dir = Path(overlay_dir)
        overlay_dir.mkdir(parents=True, exist_ok=False)
    for v in views:
        if result["type"] == "handeye":
            check_robot(meta["robot"], result["robot"])
            A = _robot_transform(v)
            X = check_transform(result["T_TCP_Cw"])
            predicted = inverse(X) @ inverse(A) @ Y
            actual_Y = A @ X @ v["T_camera_board"]
        elif result["type"] == "global_extrinsics":
            G = check_transform(result["T_B_Cg"])
            predicted = inverse(G) @ Y
            actual_Y = G @ v["T_camera_board"]
        else:
            raise ValueError("Expected handeye or global_extrinsics result")
        uv = project(v["object"], predicted, K, D)
        err = np.linalg.norm(uv - v["image"], axis=1)
        rows.append({"sample_id": v["sample"]["sample_id"], **pose_error(Y, actual_Y),
                     "predicted_reprojection_rms_px": float(np.sqrt(np.mean(err ** 2)))})
        if overlay_dir:
            image = cv2.imread(str(path / v["sample"]["image"]))
            for observed, expected in zip(v["image"], uv):
                if not np.isfinite(expected).all() or np.max(np.abs(expected)) > 1e6:
                    continue
                a, b = tuple(np.rint(observed).astype(int)), tuple(np.rint(expected).astype(int))
                cv2.circle(image, a, 3, (0, 220, 0), 1)
                cv2.drawMarker(image, b, (0, 0, 255), cv2.MARKER_CROSS, 7, 1)
                cv2.line(image, a, b, (255, 150, 0), 1)
            cv2.imwrite(str(overlay_dir / (v["sample"]["sample_id"] + ".png")), image)
    stats = _consistency(rows)
    # Every retained view must pass. Rejected views also fail the report, rather than hiding failures.
    passed = not rejected and stats["translation_m"]["max"] <= max_translation and \
        stats["rotation_deg"]["max"] <= max_rotation and \
        stats["predicted_reprojection_rms_px"]["max"] <= max_reprojection
    return {"schema_version": 1, "type": "validation", "created_utc": stamp(),
            "passed": bool(passed), "source_session": str(path.resolve()), "result_source": str(result_path),
            "thresholds": {"reprojection_rms_px": max_reprojection, "translation_m": max_translation,
                           "rotation_deg": max_rotation}, "statistics": stats,
            "views": rows, "rejected": rejected,
            "note": "Thresholds are user-adjustable checks, not an accuracy guarantee. Board must not have moved."}
