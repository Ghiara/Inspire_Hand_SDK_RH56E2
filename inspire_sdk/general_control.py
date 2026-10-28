"""Common state, hold and bounded whole-hand pose operations.

Named poses use DEVICE angle semantics: open = six native 1000 values;
zero = five native zeros with thumb_rotation held open at 1000.
Model zero radians instead represents open.
"""
import logging
import time

import numpy as np

from .model import CHANNELS, positive, vector


OPEN_POSE_NATIVE = (1000, 1000, 1000, 1000, 1000, 1000)
ZERO_POSE_NATIVE = (0, 0, 0, 0, 0, 1000)


class MotionCancelled(RuntimeError):
    """A caller cancelled a pose transition, for example by closing its viewer."""


class PoseTimeoutError(TimeoutError):
    """Position was not reached; preserve device evidence taken before holding."""

    def __init__(self, message, feedback):
        super().__init__(message)
        self.feedback = feedback.copy()
        self.diagnostics = None


class GeneralController:
    def __init__(self, backend):
        self.backend = backend

    def read_state(self):
        return self.backend.read_state()

    def advance(self, duration_s):
        self.backend.advance(duration_s)

    def _reset_control_state(self):
        """Subclasses discard their old mode's integrator/feedback state here."""

    def stop(self):
        """Discard this controller's pending control state and request a hold."""
        self._reset_control_state()
        self.backend.stop()

    @staticmethod
    def _ensure_healthy(state):
        if state.error_codes is not None and np.any(state.error_codes):
            raise RuntimeError(f"Device faults {state.error_codes.tolist()}; pose transition rejected")

    def open_hand(self, **kwargs):
        """Move all six channels to native ANGLE_SET=1000 and wait for feedback.

        kwargs are move_to_positions() options, including rate_rad_s, timeout_s,
        on_state and cancelled. No force calibration or hardware homing is run.
        """
        return self.move_to_positions(self.backend.description.from_registers(OPEN_POSE_NATIVE), **kwargs)

    def return_to_zero(self, **kwargs):
        """Bend five channels to native zero while holding thumb_rotation open.

        Native target is [0, 0, 0, 0, 0, 1000]. The thumb's root rotation target
        remains 1000 throughout the closing trajectory, including its first write.
        If rotation was moved away from open by another operation, restore that
        axis first with the other five channels held at measured positions.
        """
        if "hold_channels" in kwargs:
            raise ValueError("return_to_zero fixes thumb_rotation; hold_channels cannot be overridden")
        deadline = time.monotonic() + positive(kwargs.get("timeout_s", 30.0), "timeout_s")

        def move_options():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Return-to-zero timed out before the next pose transition")
            return {**kwargs, "timeout_s": remaining}

        cancelled = kwargs.get("cancelled")
        if cancelled is not None and cancelled():
            raise MotionCancelled("Pose transition cancelled")
        description = self.backend.description
        target = description.from_registers(ZERO_POSE_NATIVE)
        state = self.read_state()
        self._ensure_healthy(state)
        native_feedback = (getattr(self.backend, "native_position_feedback", False) or
                           getattr(state, "raw_angles", None) is not None)
        feedback = self._pose_feedback(
            state, target, target, native_feedback=native_feedback,
            tolerance_rad=positive(kwargs.get("tolerance_rad", 0.01), "tolerance_rad"),
            tolerance_units=positive(kwargs.get("tolerance_units", 20.0), "tolerance_units"))
        if "thumb_rotation" in feedback["blocking_channels"]:
            # Restore rotation before closing the fingers; never sweep rotation
            # toward the palm while the index finger is also bending inward.
            reference = (description.from_registers(state.raw_angles) if native_feedback else
                         description.positions(np.clip(state.positions_rad, description.lower, description.upper)))
            reference[CHANNELS.index("thumb_rotation")] = target[CHANNELS.index("thumb_rotation")]
            self.move_to_positions(reference, **move_options())
        return self.move_to_positions(target, hold_channels=("thumb_rotation",), **move_options())

    def _pose_feedback(self, state, target, reference, *, native_feedback, tolerance_rad, tolerance_units):
        """Compare device feedback in device units; model angles are display estimates."""
        if native_feedback:
            if getattr(state, "raw_angles", None) is None:
                raise RuntimeError("ANGLE_ACT feedback disappeared during the pose transition")
            actual = vector(state.raw_angles, name="raw_angles")
            if np.any(actual < 0) or np.any(actual > 1000) or np.any(actual != np.rint(actual)):
                raise ValueError("raw_angles must be integer ANGLE_ACT values within 0..1000")
            actual = actual.astype(int)
            target_values = self.backend.description.to_registers(target)
            reference_values = self.backend.description.to_registers(reference)
            tolerance, source, units = tolerance_units, "ANGLE_ACT", "native_units"
        else:
            actual = vector(state.positions_rad, name="positions_rad")
            target_values, reference_values = target, reference
            tolerance, source, units = tolerance_rad, "model_joint_feedback", "rad"
        errors = np.abs(actual - target_values)
        return {"source": source, "units": units, "target": target_values.tolist(),
                "actual": actual.tolist(), "absolute_error": errors.tolist(), "tolerance": tolerance,
                "raw_forces": (None if getattr(state, "raw_forces", None) is None else
                               np.asarray(state.raw_forces).tolist()),
                "commanded_force_limits": (None if getattr(self.backend, "force_limits_units", None) is None else
                                           np.asarray(self.backend.force_limits_units).tolist()),
                "blocking_channels": [channel for channel, error in zip(CHANNELS, errors) if error > tolerance],
                "final_reference_sent": bool(np.array_equal(reference_values, target_values))}

    @staticmethod
    def _pose_timeout(timeout, feedback):
        return PoseTimeoutError(
            f"Pose did not reach all six targets within {timeout:g} seconds; "
            f"source={feedback['source']}, units={feedback['units']}, "
            f"final_reference_sent={feedback['final_reference_sent']}, "
            f"blocking_channels={feedback['blocking_channels']}, "
            f"target={feedback['target']}, actual={feedback['actual']}, "
            f"absolute_error={feedback['absolute_error']}, tolerance={feedback['tolerance']}; "
            f"raw_forces={feedback['raw_forces']}, commanded_force_limits={feedback['commanded_force_limits']}",
            feedback)

    def _capture_timeout_diagnostics(self, error, state, peak_raw_forces):
        # A measured hold overwrites ANGLE_SET, so read the evidence first.
        snapshot = {"captured_before_hold": True, "feedback": error.feedback,
                    "peak_raw_forces": None if peak_raw_forces is None else peak_raw_forces.tolist(),
                    "error_codes": (None if getattr(state, "error_codes", None) is None else
                                    np.asarray(state.error_codes).tolist())}
        if hasattr(self.backend, "read_motion_diagnostics"):
            try:
                snapshot["device"] = self.backend.read_motion_diagnostics()
            except Exception as diagnostic_error:
                snapshot["diagnostic_error"] = str(diagnostic_error)
        error.diagnostics = snapshot
        error.args = (f"{error}; peak_raw_forces={snapshot['peak_raw_forces']}; "
                      f"device_diagnostics_before_hold={snapshot.get('device', snapshot.get('diagnostic_error'))}; "
                      "position not reached: inspect force thresholds, STATUS, empty-hand resistance and sensor offset; "
                      "do not treat a stopped motor as zero reached",)

    def move_to_positions(self, positions_rad, *, rate_rad_s=0.2, period_s=0.05,
                          timeout_s=30.0, tolerance_rad=0.01, settle_samples=3,
                          tolerance_units=20.0, on_state=None, on_progress=None, cancelled=None,
                          hold_channels=()):
        """Slew a six-axis pose and return its measured HandState once settled.

        The timeout covers the whole transition; reaching a write acknowledgement
        is not completion. Hardware compares raw ANGLE_ACT against native targets
        using tolerance_units; tolerance_rad applies to simulation. All axes must
        remain within tolerance for consecutive
        samples after the final reference was sent. Interpolated steps never exceed
        rate_rad_s * period_s, including after slow I/O. on_state(state, reference)
        can update a viewer/log. cancelled() aborts before the next motion write.
        on_progress(dict) receives per-axis arrival diagnostics before on_state.
        hold_channels must already be within arrival tolerance. Their references
        are fixed at the target from the first write, rather than interpolated.
        Exceptions after a write attempt trigger a best-effort measured hold.
        """
        description = self.backend.description
        target = description.positions(positions_rad)
        rate = positive(rate_rad_s, "rate_rad_s")
        description.velocities(np.full(6, rate))
        period = positive(period_s, "period_s")
        timeout = positive(timeout_s, "timeout_s")
        tolerance = positive(tolerance_rad, "tolerance_rad")
        native_tolerance = positive(tolerance_units, "tolerance_units")
        if native_tolerance > 1000:
            raise ValueError("tolerance_units must be <= 1000")
        if not isinstance(settle_samples, int) or isinstance(settle_samples, bool) or settle_samples < 1:
            raise ValueError("settle_samples must be a positive integer")
        if isinstance(hold_channels, str):
            raise ValueError("hold_channels must be a collection, not a single string")
        hold_channels = tuple(hold_channels)
        if any(channel not in CHANNELS for channel in hold_channels):
            raise ValueError(f"hold_channels must be a collection of names from {CHANNELS}")
        held_indices = [CHANNELS.index(channel) for channel in hold_channels]
        if hasattr(self.backend, "model"):
            timestep = self.backend.model.opt.timestep
            steps = round(period / timestep)
            if steps < 1 or abs(steps * timestep - period) > 1e-9:
                raise ValueError("period_s must be an integer multiple of the simulation timestep")

        self._reset_control_state()
        started = time.monotonic()
        motion_attempted = False
        peak_raw_forces = None

        def observe_force_sample(sample):
            nonlocal peak_raw_forces
            if getattr(sample, "raw_forces", None) is not None:
                forces = vector(sample.raw_forces, name="raw_forces")
                peak_raw_forces = forces if peak_raw_forces is None else np.maximum(peak_raw_forces, forces)

        try:
            if cancelled is not None and cancelled():
                raise MotionCancelled("Pose transition cancelled")
            state = self.read_state()
            self._ensure_healthy(state)
            observe_force_sample(state)
            # Soft simulation constraints can produce tiny measured overshoots.
            reference = description.positions(np.clip(state.positions_rad, description.lower, description.upper))
            native_feedback = (getattr(self.backend, "native_position_feedback", False) or
                               getattr(state, "raw_angles", None) is not None)
            feedback = self._pose_feedback(state, target, reference, native_feedback=native_feedback,
                                           tolerance_rad=tolerance, tolerance_units=native_tolerance)
            if any(channel in feedback["blocking_channels"] for channel in hold_channels):
                raise ValueError("Held channels must reach their targets before the remaining axes move")
            feedback["final_reference_sent"] = False
            last_sent = None
            settled = 0
            while True:
                if cancelled is not None and cancelled():
                    raise MotionCancelled("Pose transition cancelled")
                if time.monotonic() - started >= timeout:
                    raise self._pose_timeout(timeout, feedback)
                reference = description.positions(
                    reference + np.clip(target - reference, -rate * period, rate * period))
                reference[held_indices] = target[held_indices]
                if last_sent is None or not np.array_equal(reference, last_sent):
                    motion_attempted = True  # Includes ambiguous write acknowledgements.
                    self.backend.set_positions(reference)
                    last_sent = reference.copy()
                self.advance(period)
                state = self.read_state()
                self._ensure_healthy(state)
                observe_force_sample(state)
                feedback = self._pose_feedback(state, target, reference, native_feedback=native_feedback,
                                               tolerance_rad=tolerance, tolerance_units=native_tolerance)
                at_target = feedback["final_reference_sent"] and not feedback["blocking_channels"]
                settled = settled + 1 if at_target else 0
                feedback["settled_samples"] = settled
                feedback["required_samples"] = settle_samples
                feedback["held_channels"] = list(hold_channels)
                feedback["peak_raw_forces"] = None if peak_raw_forces is None else peak_raw_forces.tolist()
                if on_progress is not None:
                    on_progress(feedback)
                if on_state is not None:
                    on_state(state, reference.copy())
                if cancelled is not None and cancelled():
                    raise MotionCancelled("Pose transition cancelled")
                if time.monotonic() - started >= timeout:
                    raise self._pose_timeout(timeout, feedback)
                if settled >= settle_samples:
                    return state
        except (Exception, KeyboardInterrupt) as exc:
            try:
                if isinstance(exc, PoseTimeoutError):
                    self._capture_timeout_diagnostics(exc, state, peak_raw_forces)
            finally:
                # Even an interrupt while collecting diagnostics must hold.
                if motion_attempted:
                    try:
                        self.stop()
                    except Exception:
                        logging.getLogger(__name__).warning("Position hold failed after pose transition error", exc_info=True)
            raise
