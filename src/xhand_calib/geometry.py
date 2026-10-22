import numpy as np
from scipy.spatial.transform import Rotation


def check_transform(value):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError("Transform must be a finite 4x4 matrix")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError("Invalid transform bottom row")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(R), 1, atol=1e-5):
        raise ValueError("Transform rotation is not in SO(3)")
    return T


def from_pose(pose):
    """UR [x,y,z,rx,ry,rz]: metres and rotation vector in radians."""
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError("Pose must contain six finite numbers")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(pose[3:]).as_matrix()
    T[:3, 3] = pose[:3]
    return T


def to_pose(T):
    T = check_transform(T)
    return np.r_[T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_rotvec()]


def inverse(T):
    T = check_transform(T)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def mean_transform(transforms):
    ts = np.asarray([check_transform(T) for T in transforms])
    if not len(ts):
        raise ValueError("No transforms to average")
    out = np.eye(4)
    out[:3, :3] = Rotation.from_matrix(ts[:, :3, :3]).mean().as_matrix()
    out[:3, 3] = ts[:, :3, 3].mean(axis=0)
    return out


def pose_error(reference, measured):
    d = inverse(reference) @ check_transform(measured)
    return {
        "translation_m": float(np.linalg.norm(d[:3, 3])),
        "rotation_deg": float(np.degrees(Rotation.from_matrix(d[:3, :3]).magnitude())),
    }


def summary(values):
    values = np.asarray(values, dtype=float)
    if not values.size:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {"count": int(values.size), "mean": float(values.mean()),
            "median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "max": float(values.max())}


def usd_camera_pose(T_world_optical):
    """OpenCV optical (+X right,+Y down,+Z forward) -> USD camera."""
    T = check_transform(T_world_optical)
    R = T[:3, :3] @ np.diag([1., -1., -1.])
    q = Rotation.from_matrix(R).as_quat()  # xyzw -> Isaac wxyz
    return T[:3, 3].copy(), q[[3, 0, 1, 2]]
