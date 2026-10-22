import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, data):
    path = Path(path)
    if path.exists():
        raise ValueError(f"Refusing to overwrite {path}; use a new output path")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False, default=_json_default) + "\n")
    tmp.replace(path)


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def stamp():
    return datetime.now(timezone.utc).isoformat()


def load_session(path):
    path = Path(path)
    meta = read_json(path / "session.json")
    rows = [json.loads(line) for line in (path / "samples.jsonl").read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError(f"No samples in {path}")
    ids = [r["sample_id"] for r in rows]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate sample IDs")
    return path, meta, rows


def check_board(a, b):
    keys = ("frame_id", "squares_x", "squares_y", "square_length_m", "marker_length_m", "dictionary")
    if any(a.get(k) != b.get(k) for k in keys):
        raise ValueError("Board specifications/frame_id differ; a moved board requires a new extrinsic calibration")


def check_camera(meta, intrinsics):
    camera = meta["camera"]
    if camera["serial"] != intrinsics["camera"]["serial"]:
        raise ValueError("Camera serial differs from intrinsic calibration")
    for key in ("width", "height"):
        if camera[key] != intrinsics["camera"][key]:
            raise ValueError(f"Camera {key} differs from intrinsic calibration; do not silently resize")


def check_robot(a, b):
    keys = ("ip", "reference_frame", "tcp_offset_flange_to_tcp_m_rotvec")
    if any(a.get(k) != b.get(k) for k in keys):
        raise ValueError("Robot base/TCP configuration differs from hand-eye calibration")
