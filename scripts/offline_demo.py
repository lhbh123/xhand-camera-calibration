"""Run the complete CLI on rendered synthetic ChArUco sessions, without hardware."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from xhand_calib.cli import main
from xhand_calib.geometry import pose_error
from xhand_calib.storage import read_json, write_json
from xhand_calib.synthetic import generate


def run():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", required=True, help="New output directory")
    args = p.parse_args()
    root = Path(args.out)
    if root.exists():
        p.error("Output directory already exists; use a new path")
    root.mkdir(parents=True)
    data = root / "sessions"
    truth = generate(data)
    result = root / "results"

    def invoke(*cmd):
        cmd = [str(x) for x in cmd]
        print("\npython -m xhand_calib " + " ".join(cmd), flush=True)
        status = main(cmd)
        if status:
            raise SystemExit(status)

    wi, gi = result / "wrist_intrinsics.json", result / "global_intrinsics.json"
    hand, glob = result / "handeye.json", result / "global_extrinsics.json"
    invoke("intrinsics", "--session", data / "intrinsics_wrist", "--out", wi)
    invoke("intrinsics", "--session", data / "intrinsics_global", "--out", gi)
    invoke("handeye", "--session", data / "handeye", "--intrinsics", wi, "--out", hand)
    invoke("global", "--session", data / "global", "--intrinsics", gi, "--handeye", hand, "--out", glob)
    for camera, intr, ext in (("wrist", wi, hand), ("global", gi, glob)):
        invoke("validate", "--session", data / f"validation_{camera}", "--intrinsics", intr,
               "--result", ext, "--out", result / f"validation_{camera}.json",
               "--overlays", result / f"overlays_{camera}")
    invoke("rectify", "--image", data / "validation_wrist/images/0000.png", "--intrinsics", wi,
           "--out", result / "wrist_rectified.png")
    invoke("export", "--wrist-intrinsics", wi, "--global-intrinsics", gi,
           "--handeye", hand, "--global-result", glob, "--out", result / "camera_rig.json")
    report = {
        "synthetic_only": True,
        "handeye_ground_truth_error": pose_error(truth["T_TCP_Cw"], read_json(hand)["T_TCP_Cw"]),
        "global_ground_truth_error": pose_error(truth["T_B_Cg"], read_json(glob)["T_B_Cg"]),
        "wrist_validation": read_json(result / "validation_wrist.json")["statistics"],
        "global_validation": read_json(result / "validation_global.json")["statistics"],
    }
    write_json(result / "demo_report.json", report)
    print(f"\nComplete: {result / 'demo_report.json'}")


if __name__ == "__main__":
    run()
