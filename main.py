"""CLI for the standalone RH56E2 SDK."""
import argparse
from contextlib import ExitStack
from datetime import datetime
from itertools import count
from zoneinfo import ZoneInfo
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent


def default_log_path(prefix, extension):
    stamp = datetime.now(ZoneInfo("Europe/Berlin")).strftime("%Y%m%d_%H%M%S_%f")
    return ROOT / "logs" / f"{prefix}_{stamp}.{extension}"


def parser():
    result = argparse.ArgumentParser(description="Inspire RH56E2 SDK: simulation and Modbus TCP")
    commands = result.add_subparsers(dest="command", required=True)
    for name in ("visualize", "demo"):
        sub = commands.add_parser(name)
        if name == "demo":
            sub.add_argument("mode", choices=("position", "velocity", "force", "cartesian"))
        sub.add_argument("--side", choices=("right", "left"), default="right")
        sub.add_argument(
            "--duration", type=float, default=None if name == "visualize" else 10.0,
            help="Simulation seconds; visualize defaults to running until closed, demo defaults to 10 seconds",
        )
        sub.add_argument("--headless", action="store_true")
        sub.add_argument("--record", nargs="?", const="auto", metavar="OUTPUT.mp4")
        sub.add_argument("--fps", type=int, default=25)
    for name in ("state", "hardware"):
        sub = commands.add_parser(name)
        sub.add_argument("--side", choices=("right", "left"), default="right")
        sub.add_argument("--host", required=(name == "hardware"))
        sub.add_argument("--port", type=int, default=6000)
        sub.add_argument("--device-id", type=int)
        if name == "state":
            sub.add_argument("--backend", choices=("mujoco", "modbus"), default="mujoco")
        else:
            sub.add_argument("--positions-rad", nargs=6, type=float, required=True)
            sub.add_argument("--speed-units", type=float, default=150)
            sub.add_argument("--force-units", type=float, default=200)
    return result


def run_simulation(args):
    from inspire_sdk import MujocoBackend
    from inspire_sdk.model import positive
    from example.demos import Demo
    from visualization.record_video import VideoRecorder, camera_for
    duration = positive(args.duration) if args.duration is not None else None
    if duration is not None and abs(duration * 100 - round(duration * 100)) > 1e-7:
        raise ValueError("--duration must be a multiple of 0.01 seconds")
    mode = args.mode if args.command == "demo" else "visualize"
    with ExitStack() as stack:
        backend = stack.enter_context(MujocoBackend(args.side, contact_fixture=(mode == "force")))
        demo = Demo(backend, mode) if mode != "visualize" else None
        viewer = None
        if not args.headless:
            import mujoco.viewer
            viewer = stack.enter_context(mujoco.viewer.launch_passive(backend.model, backend.data))
            camera = camera_for(backend)
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth = camera.azimuth
            viewer.cam.elevation = camera.elevation
        recorder = None
        if args.record:
            path = default_log_path(mode, "mp4") if args.record == "auto" else Path(args.record)
            recorder = stack.enter_context(VideoRecorder(backend, path, fps=args.fps))
        steps = count() if duration is None else range(round(duration / 0.01))
        for _ in steps:
            if viewer is not None and not viewer.is_running():
                break
            start = time.monotonic()
            if recorder:
                recorder.capture()
            if demo:
                demo.step(0.01)
            else:
                backend.advance(0.01)
            if viewer is not None:
                viewer.sync()
                time.sleep(max(0, 0.01 - (time.monotonic() - start)))
        summary = {"mode": mode, "side": args.side, "state": backend.read_state().to_dict()}
        if recorder:
            summary.update(video=str(recorder.path), frames=recorder.frame_count, fps=recorder.fps)
        if demo and demo.ik_results:
            summary["ik_errors_m"] = [r.errors_m for r in demo.ik_results]
        print(json.dumps(summary, indent=2))


def run_hardware(args):
    import numpy as np
    from inspire_sdk.modbus_tcp import ModbusTcpBackend
    from inspire_sdk.model import HandModel
    if not args.host:
        raise ValueError("--host is required for Modbus TCP")
    # Validate the entire command before connecting or sending any writes.
    if args.command == "hardware":
        positions = HandModel(args.side).positions(args.positions_rad)
        if not np.isfinite(args.speed_units) or not 1 <= args.speed_units <= 1000:
            raise ValueError("--speed-units must be 1..1000")
        if not np.isfinite(args.force_units) or not 0 <= args.force_units <= 1000:
            raise ValueError("--force-units must be 0..1000")
    with ModbusTcpBackend(args.host, port=args.port, device_id=args.device_id, side=args.side) as backend:
        if args.command == "hardware":
            backend._check_faults()
            backend.set_speed_limits(np.full(6, args.speed_units))
            backend.set_force_limits(np.full(6, args.force_units))
            backend.set_positions(positions)
        print(json.dumps(backend.read_state().to_dict(), indent=2))


def main(argv=None):
    args = parser().parse_args(argv)
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    if getattr(args, "headless", False) and getattr(args, "record", None):
        os.environ.setdefault("MUJOCO_GL", "osmesa")
    try:
        if args.command in ("visualize", "demo"):
            run_simulation(args)
        elif args.command == "hardware" or args.backend == "modbus":
            run_hardware(args)
        else:
            from inspire_sdk import MujocoBackend
            with MujocoBackend(args.side) as backend:
                print(json.dumps(backend.read_state().to_dict(), indent=2))
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
