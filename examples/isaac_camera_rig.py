"""Isaac Sim 5.0 adapter: import ONLY after starting SimulationApp.

Independent of the optional RealSense/RTDE SDKs. See docs/SIM_INTEGRATION.md.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


class CalibratedCameraRig:
    def __init__(self, rig_path, scene, T_world_base, image_mode="rectified"):
        from isaacsim.sensors.camera import Camera
        from pxr import UsdGeom

        if image_mode not in ("raw", "rectified"):
            raise ValueError("image_mode must be raw or rectified")
        self.rig = json.loads(Path(rig_path).read_text())
        self.scene = scene
        self.T_world_base = self._check(T_world_base)
        if not np.isclose(UsdGeom.GetStageMetersPerUnit(scene.stage), 1.):
            raise ValueError("This adapter expects a stage measured in metres")
        wrist = self.rig["cameras"]["wrist"]
        if not self.rig["ready_for_isaac_wrist"] or wrist["T_link_Cw"] is None:
            raise ValueError("Export a verified --link-to-tcp transform before using the wrist adapter")
        self.link_index = scene.robot.body_names.index(wrist["reference_link"])
        self.T_link_camera = self._check(wrist["T_link_Cw"])
        self.cameras = {}
        for name, spec in self.rig["cameras"].items():
            width, height = spec["resolution"]
            cam = Camera(f"/World/CalibratedCameras/{name}", resolution=(width, height), frequency=spec["fps"])
            params = spec[image_mode]
            K = np.asarray(params["K"])
            # Consistent base projection; exact cx/cy/fx/fy are also passed to the lens model.
            focal = .02
            cam.set_focal_length(focal)
            cam.set_horizontal_aperture(focal * width / K[0, 0], maintain_square_pixels=False)
            cam.set_vertical_aperture(focal * height / K[1, 1], maintain_square_pixels=False)
            cam.set_opencv_pinhole_properties(fx=float(K[0, 0]), fy=float(K[1, 1]),
                                             cx=float(K[0, 2]), cy=float(K[1, 2]),
                                             pinhole=list(params["D"]))
            cam.set_clipping_range(.005, 100.)
            cam.initialize()
            self.cameras[name] = cam
        self.set_pose(self.cameras["global"], self.T_world_base @ self._check(self.rig["cameras"]["global"]["T_B_Cg"]))

    @staticmethod
    def _check(value):
        T = np.asarray(value, dtype=float)
        if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]):
            raise ValueError("Expected a finite homogeneous 4x4 transform")
        if not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(T[:3, :3]), 1.):
            raise ValueError("Expected a proper rotation matrix")
        return T

    @staticmethod
    def set_pose(camera, T_world_optical):
        q = Rotation.from_matrix(T_world_optical[:3, :3] @ np.diag([1., -1., -1.])).as_quat()
        camera.set_world_pose(position=T_world_optical[:3, 3], orientation=q[[3, 0, 1, 2]], camera_axes="usd")

    def update(self, env_id=0):
        """Call after physics state update and BEFORE rendering the frame."""
        data = self.scene.robot.data
        position = data.body_pos_w[env_id, self.link_index].detach().cpu().numpy()
        wxyz = data.body_quat_w[env_id, self.link_index].detach().cpu().numpy()
        T_world_link = np.eye(4)
        T_world_link[:3, :3] = Rotation.from_quat(wxyz[[1, 2, 3, 0]]).as_matrix()
        T_world_link[:3, 3] = position
        self.set_pose(self.cameras["wrist"], T_world_link @ self.T_link_camera)

    def rgb(self):
        """Call after scene.sim.render(); returned arrays are RGB, not OpenCV BGR."""
        result = {}
        for name, camera in self.cameras.items():
            rgba = camera.get_rgba()
            if rgba is None or not rgba.size:
                raise RuntimeError(f"No rendered frame from {name}; render/warm up first")
            result[name] = rgba[:, :, :3].copy()
        return result
