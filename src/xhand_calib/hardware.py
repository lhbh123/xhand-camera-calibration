"""Hardware SDKs are imported only when a hardware command is used.

No RTDE control interface or robot motion command is used anywhere in this repo.
"""
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np

from .board import detect, make_board
from .geometry import from_pose
from .storage import stamp, write_json


def rs_module():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError('Install hardware dependencies: pip install -e ".[hardware]"') from exc
    return rs


def list_devices():
    rs = rs_module()
    result = []
    for device in rs.context().query_devices():
        modes = set()
        for sensor in device.query_sensors():
            for p in sensor.get_stream_profiles():
                if p.stream_type() == rs.stream.color:
                    v = p.as_video_stream_profile()
                    modes.add((v.width(), v.height(), p.fps(), str(p.format())))
        result.append({
            "name": device.get_info(rs.camera_info.name),
            "serial": device.get_info(rs.camera_info.serial_number),
            "firmware": device.get_info(rs.camera_info.firmware_version),
            "color_modes": [dict(zip(("width", "height", "fps", "format"), m)) for m in sorted(modes)],
        })
    return result


class RealSense:
    def __init__(self, config):
        rs = rs_module()
        serial = config["serial"]
        if not serial or "REPLACE" in serial:
            raise ValueError("Set the camera serial in configs/local.json first")
        self.rs = rs
        self.pipeline = rs.pipeline()
        self.started = False
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, config["width"], config["height"], rs.format.bgr8, config["fps"])
        try:
            profile = self.pipeline.start(cfg)
            self.started = True
            device = profile.get_device()
            model = device.get_info(rs.camera_info.name)
            if config.get("expected_model", "") not in model:
                raise ValueError(f"Serial {serial} is {model}, expected {config['expected_model']}")
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            if (intr.width, intr.height) != (config["width"], config["height"]):
                raise ValueError("Resolved stream dimensions differ from the requested dimensions")
            settings = []
            for sensor in device.query_sensors():
                options = {}
                for name in ("exposure", "gain", "enable_auto_exposure", "white_balance", "enable_auto_white_balance"):
                    option = getattr(rs.option, name)
                    if sensor.supports(option):
                        options[name] = float(sensor.get_option(option))
                settings.append({"sensor": sensor.get_info(rs.camera_info.name), "options_at_start": options})
            self.metadata = {
                **config, "model": model, "stream": "color", "format": "bgr8",
                "firmware": device.get_info(rs.camera_info.firmware_version),
                "factory_intrinsics": {
                    "width": intr.width, "height": intr.height,
                    "fx": intr.fx, "fy": intr.fy, "cx": intr.ppx, "cy": intr.ppy,
                    "distortion_model": str(intr.model), "coefficients": list(intr.coeffs),
                },
                "sensor_settings": settings,
            }
            for _ in range(20):
                self.read()
        except BaseException:
            self.close()
            raise

    def read(self):
        frames = self.pipeline.wait_for_frames(5000)
        for _ in range(10):
            newer = self.pipeline.poll_for_frames()
            if not newer:
                break
            frames = newer
        frame = frames.get_color_frame()
        if not frame:
            raise RuntimeError("No RGB frame returned by RealSense")
        return np.asanyarray(frame.get_data()).copy(), {
            "camera_timestamp_ms": float(frame.get_timestamp()),
            "camera_timestamp_domain": str(frame.get_frame_timestamp_domain()),
            "camera_frame_number": int(frame.get_frame_number()),
            "host_receive_time_ns": time.time_ns(),
            "host_receive_monotonic_ns": time.monotonic_ns(),
        }

    def close(self):
        if self.started:
            self.pipeline.stop()
            self.started = False


class RobotReader:
    def __init__(self, config):
        if not config.get("tcp_configuration_verified"):
            raise ValueError("Read the active TCP offset on the pendant, fill it in, and set tcp_configuration_verified=true")
        if config.get("reference_frame") != "configured_tcp":
            raise ValueError("This collector records configured_tcp poses, not flange poses")
        if "REPLACE" in config["ip"]:
            raise ValueError("Set robot.ip before connecting")
        from_pose(config["tcp_offset_flange_to_tcp_m_rotvec"])
        try:
            import rtde_receive
        except ImportError as exc:
            raise RuntimeError('Install hardware dependencies: pip install -e ".[hardware]"') from exc
        self.config = config
        self.rtde = rtde_receive.RTDEReceiveInterface(config["ip"])

    def read(self):
        out = {
            "host_time_ns": time.time_ns(),
            "tcp_pose_m_rotvec": self.rtde.getActualTCPPose(),
            "joint_positions_rad": self.rtde.getActualQ(),
            "tcp_speed": self.rtde.getActualTCPSpeed(),
        }
        if any(np.asarray(out[k]).shape != (6,) or not np.isfinite(out[k]).all()
               for k in ("tcp_pose_m_rotvec", "joint_positions_rad", "tcp_speed")):
            raise RuntimeError("Invalid/incomplete RTDE state")
        out["T_B_TCP"] = from_pose(out["tcp_pose_m_rotvec"]).tolist()
        return out

    def is_stationary(self, before, after):
        cfg = self.config
        for sample in (before, after):
            speed = np.asarray(sample["tcp_speed"])
            if np.linalg.norm(speed[:3]) > cfg["max_linear_speed_m_s"]:
                return False
            if np.linalg.norm(speed[3:]) > cfg["max_angular_speed_rad_s"]:
                return False
        dq = np.asarray(after["joint_positions_rad"]) - before["joint_positions_rad"]
        dq = np.arctan2(np.sin(dq), np.cos(dq))
        return bool(np.max(np.abs(dq)) <= cfg["max_joint_change_rad"])

    def close(self):
        self.rtde.disconnect()


def capture(config, camera_name, kind, out, count=0, headless=False, interval=2.):
    if headless and count <= 0:
        raise ValueError("Headless capture requires --count > 0")
    if interval < 0.2:
        raise ValueError("--interval must be >= 0.2 seconds")
    if kind == "handeye" and camera_name != "wrist":
        raise ValueError("Hand-eye collection must use --camera wrist")
    if kind == "global" and camera_name != "global":
        raise ValueError("Global extrinsic collection must use --camera global")
    out = Path(out)
    if out.exists():
        raise ValueError(f"Session already exists: {out}. Use a new --out directory")
    board = make_board(config["board"])
    needs_robot = camera_name == "wrist" and kind in ("handeye", "validation")
    camera = robot = None
    saved = 0
    try:
        if needs_robot:
            robot = RobotReader(config["robot"])
        camera = RealSense(config["cameras"][camera_name])
        out.mkdir(parents=True)
        (out / "images").mkdir()
        metadata = {
            "schema_version": 1, "created_utc": stamp(), "kind": kind,
            "camera_name": camera_name, "camera": camera.metadata,
            "board": config["board"], "robot": config["robot"] if needs_robot else None,
            "image_coordinates": "raw color stream; no resize/crop/undistort",
            "robot_sampling": "stationary samples bracketing frame receipt; not hardware synchronized",
        }
        write_json(out / "session.json", metadata)
        print("SPACE: save stationary sample; Q/ESC: finish. Headless saves at --interval.", flush=True)
        next_save = time.monotonic() + interval
        with (out / "samples.jsonl").open("x") as stream:
            while count <= 0 or saved < count:
                image, frame_info = camera.read()
                obs = detect(image, config["board"], board)
                key = -1
                if not headless:
                    display = image.copy()
                    if obs is not None:
                        cv2.aruco.drawDetectedCornersCharuco(display, obs["corners"], obs["ids"])
                    label = f"{camera_name}: saved {saved}; corners {0 if obs is None else len(obs['ids'])}; SPACE save, Q quit"
                    cv2.putText(display, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 180, 0), 1)
                    cv2.imshow("xhand-calib", display)
                    key = cv2.waitKey(1) & 0xff
                    if key in (27, ord('q')):
                        break
                requested = time.monotonic() >= next_save if headless else key == ord(' ')
                if not requested:
                    continue
                next_save = time.monotonic() + interval
                before = after = None
                if robot is not None:
                    before = robot.read()
                    time.sleep(config["robot"]["settle_seconds"])
                    image, frame_info = camera.read()
                    after = robot.read()
                    if not robot.is_stationary(before, after):
                        print("Skipped: robot moved or exceeded speed limits. Stop before saving.", flush=True)
                        continue
                    obs = detect(image, config["board"], board)
                if obs is None:
                    print("Skipped: insufficient ChArUco corners. Change board distance/angle/lighting.", flush=True)
                    continue
                sample_id = f"{saved:04d}"
                relative = f"images/{sample_id}.png"
                if not cv2.imwrite(str(out / relative), image):
                    raise RuntimeError("PNG write failed")
                row = {"sample_id": sample_id, "image": relative, **frame_info,
                       "detected_corners": len(obs["ids"])}
                if robot is not None:
                    row.update({"robot": after, "robot_before": before, "stationary_checked": True})
                stream.write(json.dumps(row, allow_nan=False) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
                saved += 1
                print(f"Saved {saved}: {relative}", flush=True)
    finally:
        if camera is not None:
            camera.close()
        if robot is not None:
            robot.close()
        if not headless:
            cv2.destroyAllWindows()
        print(f"Saved {saved} samples to {out}", flush=True)
    return saved
