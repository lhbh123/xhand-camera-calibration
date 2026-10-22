import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from xhand_calib.cli import main
from xhand_calib.export import export_rig, rectify_image
from xhand_calib.geometry import from_pose, inverse, pose_error, usd_camera_pose
from xhand_calib.hardware import RobotReader
from xhand_calib.solve import global_calibration, handeye_calibration, intrinsic_calibration, motion_excitation, validate
from xhand_calib.storage import check_camera, check_robot, read_json, write_json
from xhand_calib.synthetic import generate


class GeometryTests(unittest.TestCase):
    def test_ur_rotvec_and_inverse(self):
        T = from_pose([.1, .2, .3, 0, 0, np.pi / 2])
        np.testing.assert_allclose(T @ inverse(T), np.eye(4), atol=1e-12)
        np.testing.assert_allclose(T @ [1, 0, 0, 1], [.1, 1.2, .3, 1], atol=1e-12)

    def test_optical_to_usd(self):
        from scipy.spatial.transform import Rotation
        p, wxyz = usd_camera_pose(np.eye(4))
        np.testing.assert_allclose(p, [0, 0, 0])
        R = Rotation.from_quat(wxyz[[1, 2, 3, 0]]).as_matrix()
        np.testing.assert_allclose(R @ [0, 0, -1], [0, 0, 1], atol=1e-12)

    def test_degenerate_motion_rejected(self):
        poses = [from_pose([0, 0, 0, 0, 0, x]) for x in np.linspace(0, .8, 12)]
        with self.assertRaisesRegex(ValueError, "Insufficient"):
            motion_excitation(poses)

    def test_mismatched_device_and_tcp_rejected(self):
        with self.assertRaises(ValueError):
            check_camera({"camera": {"serial": "A"}}, {"camera": {"serial": "B"}})
        with self.assertRaises(ValueError):
            check_robot({"reference_frame": "flange"}, {"reference_frame": "configured_tcp"})

    def test_stationary_guard_rejects_motion(self):
        reader = RobotReader.__new__(RobotReader)  # No SDK connection needed for guard math.
        reader.config = {"max_linear_speed_m_s": .002, "max_angular_speed_rad_s": .01,
                         "max_joint_change_rad": .002}
        before = {"tcp_speed": [0.] * 6, "joint_positions_rad": [0.] * 6}
        after = copy.deepcopy(before)
        self.assertTrue(reader.is_stationary(before, after))
        after["tcp_speed"][0] = .02
        self.assertFalse(reader.is_stationary(before, after))
        after = copy.deepcopy(before)
        after["joint_positions_rad"][2] = .01
        self.assertFalse(reader.is_stationary(before, after))

    def test_unverified_tcp_rejected_before_connecting(self):
        with self.assertRaisesRegex(ValueError, "TCP"):
            RobotReader({"tcp_configuration_verified": False})


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="xhand-calib-test-")
        cls.root = Path(cls.tmp.name)
        cls.data = cls.root / "sessions"
        cls.truth = generate(cls.data)
        cls.wrist = intrinsic_calibration(cls.data / "intrinsics_wrist")
        cls.glob = intrinsic_calibration(cls.data / "intrinsics_global")
        cls.wi, cls.gi = cls.root / "wrist.json", cls.root / "global_intrinsics.json"
        write_json(cls.wi, cls.wrist)
        write_json(cls.gi, cls.glob)
        cls.hand = handeye_calibration(cls.data / "handeye", cls.wi)
        cls.hp = cls.root / "handeye.json"
        write_json(cls.hp, cls.hand)
        cls.ext = global_calibration(cls.data / "global", cls.gi, cls.hp)
        cls.gp = cls.root / "global.json"
        write_json(cls.gp, cls.ext)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_intrinsics_recover_rendered_camera(self):
        for result, key in ((self.wrist, "K_wrist"), (self.glob, "K_global")):
            K = np.asarray(result["K"])
            truth = self.truth[key]
            self.assertLess(np.max(np.abs(K - truth)), 8.)
            self.assertLess(result["holdout_rms_px"]["max"], 1.)
            self.assertGreaterEqual(result["holdout_rms_px"]["count"], 5)

    def test_handeye_and_global_recover_ground_truth(self):
        for result, key in ((self.hand, "T_TCP_Cw"), (self.ext, "T_B_Cg")):
            err = pose_error(self.truth[key], result[key])
            self.assertLess(err["translation_m"], .01)
            self.assertLess(err["rotation_deg"], 1.)

    def test_independent_validation_and_overlays(self):
        for camera, intr, ext in (("wrist", self.wi, self.hp), ("global", self.gi, self.gp)):
            overlay = self.root / f"overlay_{camera}"
            result = validate(self.data / f"validation_{camera}", intr, ext, overlay_dir=overlay)
            self.assertTrue(result["passed"], result)
            self.assertEqual(len(list(overlay.glob("*.png"))), len(result["views"]))

    def test_wrong_transform_fails_validation(self):
        bad = copy.deepcopy(self.hand)
        bad["T_TCP_Cw"] = np.asarray(bad["T_TCP_Cw"]).copy()
        bad["T_TCP_Cw"][0, 3] += .03
        path = self.root / "wrong_handeye.json"
        write_json(path, bad)
        result = validate(self.data / "validation_wrist", self.wi, path)
        self.assertFalse(result["passed"])

    def test_export_requires_verified_link_and_preserves_K(self):
        rig = export_rig(self.wi, self.gi, self.hp, self.gp)
        self.assertFalse(rig["ready_for_isaac_wrist"])
        link = self.root / "link.json"
        write_json(link, {"reference_link": "wrist_3_link", "verified": False, "T_link_tcp": np.eye(4)})
        with self.assertRaisesRegex(ValueError, "placeholder"):
            export_rig(self.wi, self.gi, self.hp, self.gp, link)
        link2 = self.root / "link_verified.json"
        write_json(link2, {"reference_link": "wrist_3_link", "verified": True, "T_link_tcp": np.eye(4)})
        rig = export_rig(self.wi, self.gi, self.hp, self.gp, link2)
        self.assertTrue(rig["ready_for_isaac_wrist"])
        np.testing.assert_allclose(rig["cameras"]["wrist"]["T_link_Cw"], self.hand["T_TCP_Cw"])

    def test_rectify_and_no_overwrite(self):
        image = self.data / "validation_wrist/images/0000.png"
        out = self.root / "rectified.png"
        result = rectify_image(image, self.wi, out)
        self.assertEqual(cv2.imread(str(out)).shape[:2], (720, 960))
        self.assertFalse(result["cropped"])
        self.assertEqual(len(result["D"]), 5)
        with self.assertRaises(ValueError):
            rectify_image(image, self.wi, out)

    def test_cli_export_and_failed_validation_exit_code(self):
        out = self.root / "cli_rig.json"
        self.assertEqual(main(["export", "--wrist-intrinsics", str(self.wi), "--global-intrinsics", str(self.gi),
                               "--handeye", str(self.hp), "--global-result", str(self.gp), "--out", str(out)]), 0)
        self.assertEqual(read_json(out)["type"], "calibrated_camera_rig")
        report = self.root / "tight_validation.json"
        self.assertEqual(main(["validate", "--session", str(self.data / "validation_wrist"),
                               "--intrinsics", str(self.wi), "--result", str(self.hp), "--out", str(report),
                               "--max-translation", "0.0000001"]), 2)

    def test_moved_board_id_rejected(self):
        hand = copy.deepcopy(self.hand)
        hand["board"]["frame_id"] = "different_board_location"
        path = self.root / "moved_board.json"
        write_json(path, hand)
        with self.assertRaisesRegex(ValueError, "Board"):
            global_calibration(self.data / "global", self.gi, path)

    def test_changed_intrinsics_rejected(self):
        intr = copy.deepcopy(self.wrist)
        intr["K"] = np.asarray(intr["K"]).copy()
        intr["K"][0, 0] += 10
        path = self.root / "changed_intrinsics.json"
        write_json(path, intr)
        with self.assertRaisesRegex(ValueError, "Intrinsics changed"):
            export_rig(path, self.gi, self.hp, self.gp)

    def test_result_paths_are_not_loading_dependencies(self):
        hand = copy.deepcopy(self.hand)
        glob = copy.deepcopy(self.ext)
        hand["intrinsics_source"] = "/nonexistent/original_machine/wrist.json"
        glob["intrinsics_source"] = "/nonexistent/original_machine/global.json"
        h, g = self.root / "portable_hand.json", self.root / "portable_global.json"
        write_json(h, hand)
        write_json(g, glob)
        rig = export_rig(self.wi, self.gi, h, g)
        self.assertEqual(rig["type"], "calibrated_camera_rig")


if __name__ == "__main__":
    unittest.main(verbosity=2)
