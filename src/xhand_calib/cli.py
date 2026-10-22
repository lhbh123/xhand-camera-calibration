import argparse
import json
from pathlib import Path
import sys

import cv2

from .storage import read_json, write_json


def parser():
    p = argparse.ArgumentParser(description="D405/D435 + UR5e camera calibration (robot state read only)")
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("devices", help="List RealSense serials and color modes")
    q = sub.add_parser("board", help="Generate a printable ChArUco board")
    q.add_argument("--config", required=True)
    q.add_argument("--out", required=True, help="New PNG output")
    q.add_argument("--pixels-per-square", type=int, default=200)

    q = sub.add_parser("robot-state", help="Read current configured TCP/joints; never commands motion")
    q.add_argument("--config", required=True)

    q = sub.add_parser("capture", help="Collect PNGs and optional stationary RTDE state")
    q.add_argument("--config", required=True)
    q.add_argument("--camera", choices=["wrist", "global"], required=True)
    q.add_argument("--kind", choices=["intrinsics", "handeye", "global", "validation"], required=True)
    q.add_argument("--out", required=True, help="New session directory; never overwrite")
    q.add_argument("--count", type=int, default=0, help="0 = manual until Q; positive = finish after N saves")
    q.add_argument("--headless", action="store_true")
    q.add_argument("--interval", type=float, default=2.)

    q = sub.add_parser("intrinsics", help="Fit K/D with an automatic holdout split")
    q.add_argument("--session", required=True)
    q.add_argument("--out", required=True)
    q.add_argument("--holdout-every", type=int, default=5)

    q = sub.add_parser("handeye", help="Solve configured-TCP <- wrist optical camera")
    q.add_argument("--session", required=True)
    q.add_argument("--intrinsics", required=True)
    q.add_argument("--out", required=True)
    q.add_argument("--method", choices=["park", "tsai", "horaud", "andreff", "daniilidis"], default="park")
    q.add_argument("--no-refine", action="store_true")
    q.add_argument("--max-reprojection", type=float, default=2.)

    q = sub.add_parser("global", help="Solve base <- global camera using the SAME static board")
    q.add_argument("--session", required=True)
    q.add_argument("--intrinsics", required=True)
    q.add_argument("--handeye", required=True)
    q.add_argument("--out", required=True)
    q.add_argument("--max-reprojection", type=float, default=2.)

    q = sub.add_parser("validate", help="Check an independent validation session")
    q.add_argument("--session", required=True)
    q.add_argument("--intrinsics", required=True)
    q.add_argument("--result", required=True)
    q.add_argument("--out", required=True)
    q.add_argument("--overlays", help="New directory: green observed circles, red predicted crosses")
    q.add_argument("--max-reprojection", type=float, default=2.)
    q.add_argument("--max-translation", type=float, default=.005, help="metres")
    q.add_argument("--max-rotation", type=float, default=2., help="degrees")

    q = sub.add_parser("rectify", help="Undistort one PNG, preserving full resolution")
    q.add_argument("--image", required=True)
    q.add_argument("--intrinsics", required=True)
    q.add_argument("--out", required=True)
    q.add_argument("--alpha", type=float, default=0.)

    q = sub.add_parser("export", help="Export raw/rectified camera rig and extrinsics")
    q.add_argument("--wrist-intrinsics", required=True)
    q.add_argument("--global-intrinsics", required=True)
    q.add_argument("--handeye", required=True)
    q.add_argument("--global-result", required=True)
    q.add_argument("--link-to-tcp", help="Verified T_link_tcp JSON; optional until simulation integration")
    q.add_argument("--alpha", type=float, default=0.)
    q.add_argument("--out", required=True)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if hasattr(args, "out") and Path(args.out).exists():
            raise ValueError(f"Output exists: {args.out}; choose a new path")
        if args.command == "devices":
            from .hardware import list_devices
            devices = list_devices()
            print(json.dumps(devices, indent=2))
            return 0 if devices else 1
        if args.command == "board":
            from .board import make_board
            spec = read_json(args.config)["board"]
            if args.pixels_per_square < 50 or Path(args.out).suffix.lower() != ".png":
                raise ValueError("Use a PNG output and >=50 pixels per square")
            board = make_board(spec)
            size = (spec["squares_x"] * args.pixels_per_square, spec["squares_y"] * args.pixels_per_square)
            image = board.generateImage(size, marginSize=0, borderBits=1)
            out = Path(args.out)
            if Path(str(out) + ".json").exists():
                raise ValueError("Board sidecar already exists")
            out.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(out), image):
                raise RuntimeError("Cannot write board image")
            write_json(str(out) + ".json", spec)
            print(f"Print board body at {spec['squares_x'] * spec['square_length_m'] * 1000:g} x "
                  f"{spec['squares_y'] * spec['square_length_m'] * 1000:g} mm. Verify actual square size.")
            return 0
        if args.command == "robot-state":
            from .hardware import RobotReader
            robot = RobotReader(read_json(args.config)["robot"])
            try:
                print(json.dumps(robot.read(), indent=2))
            finally:
                robot.close()
            return 0
        if args.command == "capture":
            from .hardware import capture
            n = capture(read_json(args.config), args.camera, args.kind, args.out,
                        args.count, args.headless, args.interval)
            return 0 if n and (not args.count or n >= args.count) else 1
        if args.command == "intrinsics":
            from .solve import intrinsic_calibration
            result = intrinsic_calibration(args.session, args.holdout_every)
        elif args.command == "handeye":
            from .solve import handeye_calibration
            result = handeye_calibration(args.session, args.intrinsics, args.method,
                                         args.max_reprojection, not args.no_refine)
        elif args.command == "global":
            from .solve import global_calibration
            result = global_calibration(args.session, args.intrinsics, args.handeye, args.max_reprojection)
        elif args.command == "validate":
            from .solve import validate
            result = validate(args.session, args.intrinsics, args.result, args.max_reprojection,
                              args.max_translation, args.max_rotation, args.overlays)
        elif args.command == "rectify":
            from .export import rectify_image
            rectify_image(args.image, args.intrinsics, args.out, args.alpha)
            print(f"Saved {args.out} and {args.out}.json")
            return 0
        elif args.command == "export":
            from .export import export_rig
            result = export_rig(args.wrist_intrinsics, args.global_intrinsics, args.handeye,
                                args.global_result, args.link_to_tcp, args.alpha)
        else:
            raise ValueError(args.command)
        write_json(args.out, result)
        print(f"Saved {args.out}")
        for warning in result.get("warnings", []):
            print("NOTE:", warning)
        if result.get("type") == "validation":
            print(json.dumps({"passed": result["passed"], "statistics": result["statistics"]}, indent=2))
            return 0 if result["passed"] else 2
        return 0
    except KeyboardInterrupt:
        print("Interrupted; completed capture samples remain on disk.", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError, ImportError, cv2.error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
