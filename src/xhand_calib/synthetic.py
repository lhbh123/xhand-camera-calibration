"""Rendered ChArUco sessions for offline checks; never represents real hardware accuracy."""
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .board import make_board, project
from .geometry import from_pose, inverse
from .storage import write_json


def generate(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(23)
    spec = {"frame_id": "synthetic_static_board", "squares_x": 7, "squares_y": 10,
            "square_length_m": .025, "marker_length_m": .018, "dictionary": "DICT_5X5_100", "min_corners": 12}
    board = make_board(spec)
    texture = board.generateImage((1400, 2000), marginSize=0, borderBits=1)
    robot = {"ip": "synthetic", "reference_frame": "configured_tcp",
             "tcp_offset_flange_to_tcp_m_rotvec": [0.] * 6, "tcp_configuration_verified": True}
    K_w = np.array([[760., 0, 476.], [0, 752., 357.], [0, 0, 1.]])
    K_g = np.array([[820., 0, 482.], [0, 815., 362.], [0, 0, 1.]])
    X = from_pose([.04, -.025, .065, .16, -.22, .08])
    Y = from_pose([.48, -.13, .72, .05, -.03, .2])
    CgQ = from_pose([-.075, -.12, .72, .22, -.18, .08])
    G = Y @ inverse(CgQ)

    def poses(n):
        result = []
        for _ in range(n):
            R = Rotation.from_euler("xyz", rng.uniform([-.5, -.5, -.35], [.5, .5, .35])).as_matrix()
            center = np.array([rng.uniform(-.105, .105), rng.uniform(-.065, .065), rng.uniform(.5, .82)])
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = center - R @ np.array([.0875, .125, 0.])
            result.append(T)
        return result

    def render(T, K):
        edge = np.array([[0, 0, 0], [.175, 0, 0], [.175, .25, 0], [0, .25, 0]], dtype=float)
        uv = project(edge, T, K, np.zeros(5)).astype(np.float32)
        H = cv2.getPerspectiveTransform(np.float32([[0, 0], [1400, 0], [1400, 2000], [0, 2000]]), uv)
        gray = cv2.warpPerspective(texture, H, (960, 720), flags=cv2.INTER_LINEAR, borderValue=255)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def session(name, camera, kind, transforms, K):
        path = root / name
        (path / "images").mkdir(parents=True)
        cam = {"serial": f"SYNTHETIC_{camera}", "width": 960, "height": 720, "fps": 30,
               "stream": "color", "model": "synthetic perspective camera"}
        metadata = {"schema_version": 1, "kind": kind, "camera_name": camera,
                    "camera": cam, "board": spec, "robot": robot if camera == "wrist" and kind != "intrinsics" else None}
        write_json(path / "session.json", metadata)
        with (path / "samples.jsonl").open("x") as f:
            for i, T in enumerate(transforms):
                row = {"sample_id": f"{i:04d}", "image": f"images/{i:04d}.png"}
                cv2.imwrite(str(path / row["image"]), render(T, K))
                if metadata["robot"]:
                    A = Y @ inverse(T) @ inverse(X)
                    row.update({"robot": {"T_B_TCP": A.tolist()}, "stationary_checked": True})
                f.write(json.dumps(row) + "\n")
    session("intrinsics_wrist", "wrist", "intrinsics", poses(32), K_w)
    session("intrinsics_global", "global", "intrinsics", poses(32), K_g)
    session("handeye", "wrist", "handeye", poses(26), K_w)
    session("global", "global", "global", [CgQ] * 5, K_g)
    session("validation_wrist", "wrist", "validation", poses(8), K_w)
    session("validation_global", "global", "validation", [CgQ] * 6, K_g)
    truth = {"K_wrist": K_w, "K_global": K_g, "T_TCP_Cw": X, "T_B_Q": Y, "T_B_Cg": G}
    write_json(root / "ground_truth.json", truth)
    return truth


def main():
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", required=True, help="New output directory")
    args = p.parse_args()
    generate(args.out)
    print(f"Synthetic sessions saved to {args.out}; these do not establish real-world accuracy.")


if __name__ == "__main__":
    main()
