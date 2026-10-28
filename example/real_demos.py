"""Reviewable hardware checks and small-motion demos; no I/O at import time.

Only --real constructs a Modbus client. Hardware demos require a viewer and
mirror measured poses; simulation previews use the normal torque-driven backend.
"""
from contextlib import ExitStack
import json
from pathlib import Path
import sys
import time

import numpy as np

from inspire_sdk import InspireHand, MotionCancelled, MujocoBackend
from inspire_sdk.general_control import OPEN_POSE_NATIVE, ZERO_POSE_NATIVE
from inspire_sdk.model import CHANNELS, FINGERS, HandModel, positive
from visualization.feedback_viewer import open_viewer, sync_feedback


def validate_connection(args):
    positive(args.timeout, "--timeout")
    if not args.host or not 1 <= args.port <= 65535:
        raise ValueError("--host must be nonempty and --port must be 1..65535")
    if args.device_id is not None and not 1 <= args.device_id <= 255:
        raise ValueError("--device-id must be 1..255")


def connect_hand(stack, args, scene):
    if args.real:
        return stack.enter_context(InspireHand.modbus_tcp(
            args.host, port=args.port, device_id=args.device_id, side=args.side, timeout_s=args.timeout))
    # The stack already owns the simulation backend.
    return InspireHand(scene)


def healthy(state):
    return state.error_codes is None or not np.any(state.error_codes)


def run_connection_check(args):
    """FC03 only in real mode, including cleanup: never call stop() here."""
    validate_connection(args)
    interval = positive(args.interval, "--interval")
    if args.samples < 1:
        raise ValueError("--samples must be >= 1")
    with ExitStack() as stack:
        scene = stack.enter_context(MujocoBackend(args.side))
        viewer = None if args.headless else open_viewer(stack, scene)
        hand = connect_hand(stack, args, scene)
        samples = []
        for index in range(args.samples):
            if viewer is not None and not viewer.is_running():
                break
            start = time.monotonic()
            state = hand.read_state()
            samples.append(state.to_dict())
            if args.real:
                sync_feedback(scene, state)
            if viewer is not None:
                viewer.sync()
            if index + 1 < args.samples:
                time.sleep(max(0, interval - (time.monotonic() - start)))
        ok = bool(samples) and all(not any(s["error_codes"] or []) for s in samples)
        diagnostics = hand.backend.read_motion_diagnostics() if args.real and args.diagnostics else None
        print(json.dumps({"backend": "modbus" if args.real else "mujoco", "read_only": True,
                          "host": args.host if args.real else None,
                          "port": args.port if args.real else None,
                          "device_id": (args.device_id if args.device_id is not None else
                                        (1 if args.side == "right" else 2)) if args.real else None,
                          "samples_read": len(samples), "healthy": ok, "states": samples,
                          "motion_diagnostics": diagnostics}, indent=2))
        return 0 if ok else 1


class SmallMotionDemo:
    """Two bounded waypoints relative to the measured starting pose.

    The Cartesian path pre-solves reachable FK-generated endpoints before demo
    commands, then slews their IK solutions with the same rad/s reference limit.
    """
    def __init__(self, hand, mode, initial_state, channels, amplitude, rate):
        self.hand, self.mode, self.rate = hand, mode, rate
        description = hand.backend.description
        self.reference = description.positions(np.clip(initial_state.positions_rad, description.lower, description.upper))
        offset = np.array([amplitude if c in channels else 0.0 for c in CHANNELS])
        self.waypoints = [np.clip(self.reference + sign * offset, description.lower, description.upper)
                          for sign in (1, -1)]
        self.lower_bound_rad = self.waypoints[1].copy()
        self.upper_bound_rad = self.waypoints[0].copy()
        self.ik_results = []
        if mode == "cartesian":
            fingers = [f for f in FINGERS if f in channels or (f == "thumb" and
                       any(c.startswith("thumb_") for c in channels))]
            for waypoint in self.waypoints:
                tips = hand.backend.kinematics.forward(waypoint)
                # Measured and commanded open poses differ on real hardware.
                # Keep the same measured seed for both endpoints, lock every
                # unselected channel, and enforce excursion bounds in the solver.
                result = hand.cartesian.solve(
                    {f: tips[f] for f in fingers}, seed_rad=self.reference,
                    active_channels=channels, lower_rad=self.lower_bound_rad, upper_rad=self.upper_bound_rad)
                changed = [channel for i, channel in enumerate(CHANNELS)
                           if channel not in channels and abs(result.positions_rad[i] - self.reference[i]) > 1e-9]
                exceeded = [channel for i, channel in enumerate(CHANNELS)
                            if result.positions_rad[i] < self.lower_bound_rad[i] - 1e-9 or
                            result.positions_rad[i] > self.upper_bound_rad[i] + 1e-9]
                if changed or exceeded:
                    raise ValueError(f"Cartesian IK preflight rejected: changed_locked_channels={changed}, "
                                     f"outside_excursion_bounds={exceeded}, seed_rad={self.reference.tolist()}, "
                                     f"solution_rad={result.positions_rad.tolist()}")
                self.ik_results.append(result)
            self.waypoints = [r.positions_rad for r in self.ik_results]
        if mode == "velocity":
            hand.velocity.set_targets(np.zeros(6))

    def update(self, elapsed_s, duration_s):
        target = self.waypoints[int(elapsed_s / 3.0) % 2]
        if self.mode == "velocity":
            # Retain the integrated reference, including across reversals. Values
            # are bounded by the waypoint, so prolonged runs cannot drift away.
            velocity = np.clip((target - self.hand.velocity.reference_rad) / duration_s,
                               -self.rate, self.rate)
            self.hand.velocity.set_targets(velocity, reset_reference=False)
            self.reference = self.hand.velocity.update(duration_s)
        else:
            self.reference += np.clip(target - self.reference, -self.rate * duration_s,
                                      self.rate * duration_s)
            self.hand.position.set_targets(self.reference)
        return self.reference.copy()


def validate_demo(args):
    validate_connection(args)
    positive(args.duration, "--duration")
    period = positive(args.period, "--period")
    if period < 0.02 or abs(period * 1000 - round(period * 1000)) > 1e-7:
        raise ValueError("--period must be >= 0.02 seconds and a multiple of 0.001")
    amplitude = positive(args.amplitude, "--amplitude")
    rate = positive(args.rate, "--rate")
    if amplitude > 0.3 or rate > 0.2:
        raise ValueError("Review demos require --amplitude <= 0.3 rad and --rate <= 0.2 rad/s")
    HandModel(args.side).velocities(np.full(6, rate))
    if not 1 <= args.speed_units <= 1000 or not 1 <= args.force_units <= 1000:
        raise ValueError("--speed-units and --force-units must be 1..1000")
    if args.real and args.headless:
        raise ValueError("Real motion requires the MuJoCo viewer; remove --headless")
    pose_rate = positive(args.pose_rate, "--pose-rate")
    if pose_rate > 0.2:
        raise ValueError("Review demos require --pose-rate <= 0.2 rad/s")
    positive(args.pose_timeout, "--pose-timeout")
    tolerance = positive(args.pose_tolerance, "--pose-tolerance")
    if tolerance > 0.05:
        raise ValueError("--pose-tolerance must be <= 0.05 rad")
    if not 1 <= args.pose_tolerance_units <= 50:
        raise ValueError("--pose-tolerance-units must be 1..50")


def run_motion_demo(args):
    validate_demo(args)  # Validate before creating a client or writing limits.
    from main import default_log_path
    path = default_log_path(f"{'real' if args.real else 'sim'}_{args.mode}", "jsonl") if args.log == "auto" else Path(args.log)
    path.parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        log = stack.enter_context(path.open("x", encoding="utf-8"))
        scene = stack.enter_context(MujocoBackend(args.side))
        viewer = None if args.headless else open_viewer(stack, scene)
        hand = connect_hand(stack, args, scene)
        state = hand.read_state()
        if not healthy(state):
            raise RuntimeError(f"Device faults {state.error_codes.tolist()}; demo rejected")
        if args.real:
            sync_feedback(scene, state)
        if viewer is not None:
            viewer.sync()
        log.write(json.dumps({"event": "start", "backend": "modbus" if args.real else "mujoco",
                              "options": vars(args), "state": state.to_dict(),
                              "open_native": list(OPEN_POSE_NATIVE), "zero_native": list(ZERO_POSE_NATIVE)}) + "\n")
        log.flush()
        motion_attempted = False
        ticks = 0
        stop_error = None
        zero_reached = False
        reason = "completed"
        failure = None
        failure_diagnostics = None
        phase = "setup"
        phase_begin = time.monotonic()
        pose_frame_begin = phase_begin
        pose_status = None
        next_progress = 0.0

        def cancelled():
            return viewer is not None and not viewer.is_running()

        def show_state(measured, reference):
            nonlocal state, pose_frame_begin
            state = measured
            if args.real:
                sync_feedback(scene, state)
            if viewer is not None:
                viewer.sync()
            log.write(json.dumps({"event": "sample", "phase": phase,
                                  "elapsed_s": time.monotonic() - phase_begin,
                                  "reference_rad": reference.tolist(), "pose_status": pose_status,
                                  "state": state.to_dict()}) + "\n")
            log.flush()
            if not args.real and phase != "demo":
                # The simulation backend advances physics without sleeping;
                # pace the opening/zero preview like the hardware display.
                time.sleep(max(0, args.period - (time.monotonic() - pose_frame_begin)))
                pose_frame_begin = time.monotonic()

        def start_phase(name):
            nonlocal phase, phase_begin, pose_frame_begin, pose_status, next_progress
            phase, phase_begin = name, time.monotonic()
            pose_frame_begin = phase_begin
            pose_status, next_progress = None, phase_begin
            log.write(json.dumps({"event": "phase", "phase": phase}) + "\n")
            log.flush()
            print(f"[{phase}] {args.mode}", file=sys.stderr)

        def show_pose_progress(feedback):
            nonlocal pose_status, next_progress
            pose_status = feedback
            now = time.monotonic()
            if args.real and (now >= next_progress or feedback["settled_samples"] >= feedback["required_samples"]):
                print(f"[{phase}] {feedback['source']}={feedback['actual']}, "
                      f"target={feedback['target']}, error={feedback['absolute_error']}, "
                      f"tolerance={feedback['tolerance']:g} {feedback['units']}, "
                      f"pending={feedback['blocking_channels']}, "
                      f"raw_forces={feedback['raw_forces']}, "
                      f"force_limits={feedback['commanded_force_limits']}, "
                      f"final_reference_sent={feedback['final_reference_sent']}, "
                      f"settled={feedback['settled_samples']}/{feedback['required_samples']}", file=sys.stderr)
                next_progress = now + 2.0

        pose_options = dict(rate_rad_s=args.pose_rate, period_s=args.period,
                            timeout_s=args.pose_timeout, tolerance_rad=args.pose_tolerance,
                            tolerance_units=args.pose_tolerance_units, on_state=show_state,
                            on_progress=show_pose_progress, cancelled=cancelled)
        try:
            # Initial feedback and the viewer precede ALL motion, including the
            # whole-hand opening. Cartesian preflight follows measured opening.
            if cancelled():
                raise MotionCancelled("Viewer closed before opening")
            motion_attempted = True
            if args.real:
                hand.backend.set_speed_limits(np.full(6, args.speed_units))
                hand.force.set_native_limits(np.full(6, args.force_units))
            start_phase("opening")
            state = hand.open_hand(**pose_options)
            if cancelled():
                raise MotionCancelled("Viewer closed after opening")
            if args.mode == "cartesian":
                start_phase("cartesian_preflight")
            demo = SmallMotionDemo(hand, args.mode, state, args.channels, args.amplitude, args.rate)
            log.write(json.dumps({"event": "demo_plan", "waypoints_rad": [q.tolist() for q in demo.waypoints],
                                  "seed_rad": demo.reference.tolist(), "active_channels": args.channels,
                                  "lower_bound_rad": demo.lower_bound_rad.tolist(),
                                  "upper_bound_rad": demo.upper_bound_rad.tolist(),
                                  "ik_errors_m": [r.errors_m for r in demo.ik_results]}) + "\n")
            log.flush()
            start_phase("demo")
            begin = time.monotonic()
            previous = begin
            while True:
                now = time.monotonic()
                elapsed = now - begin
                if cancelled():
                    raise MotionCancelled("Viewer closed during demo")
                if elapsed + 1e-9 >= args.duration:
                    break
                # Do not catch up after network/IK/rendering stalls.
                dt = min(args.period, now - previous) if ticks else args.period
                previous = now
                reference = demo.update(elapsed, max(dt, 1e-6))
                if not args.real:
                    scene.advance(args.period)
                state = hand.read_state()
                if not healthy(state):
                    raise RuntimeError(f"Device faults {state.error_codes.tolist()}; demo stopped")
                if demo.ik_results:
                    result = demo.ik_results[int(elapsed / 3.0) % 2]
                    for finger, point in result.requested_m.items():
                        scene.model.site(f"{finger}_target").pos[:] = point
                show_state(state, reference)
                ticks += 1
                time.sleep(max(0, args.period - (time.monotonic() - now)))
            if cancelled():
                raise MotionCancelled("Viewer closed before returning to zero")
            start_phase("returning_zero")
            state = hand.return_to_zero(**pose_options)
            zero_reached = True
        except MotionCancelled as exc:
            reason, failure = "cancelled", str(exc)
        except KeyboardInterrupt:
            reason, failure = "interrupted", "KeyboardInterrupt"
            raise
        except Exception as exc:
            reason, failure = "error", f"{type(exc).__name__}: {exc}"
            failure_diagnostics = getattr(exc, "diagnostics", None)
            raise
        finally:
            # Preserve the original failure or KeyboardInterrupt even if hold fails.
            # On success keep the final five-zero/thumb-open target. A new
            # measured hold could stop a few native units short of zero.
            hold_attempted = motion_attempted and not zero_reached
            if hold_attempted:
                try:
                    hand.stop()
                except (ValueError, RuntimeError, OSError) as exc:
                    stop_error = str(exc)
                    print(f"Position hold failed: {exc}", file=sys.stderr)
            log.write(json.dumps({"event": "finish", "phase": phase, "reason": reason,
                                  "error": failure, "ticks": ticks, "zero_reached": zero_reached,
                                  "failure_diagnostics": failure_diagnostics,
                                  "hold_attempted": hold_attempted, "hold_error": stop_error}) + "\n")
            log.flush()
        if stop_error:
            raise RuntimeError(f"Demo ended but position hold failed: {stop_error}")
        print(json.dumps({"mode": args.mode, "backend": "modbus" if args.real else "mujoco",
                          "reason": reason, "zero_reached": zero_reached,
                          "ticks": ticks, "log": str(path), "state": state.to_dict()}, indent=2))
