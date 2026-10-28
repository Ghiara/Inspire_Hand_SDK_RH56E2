"""Pose-controller regression cases; uses only an in-memory backend."""
from types import SimpleNamespace

import numpy as np
import pytest

from inspire_sdk import (GeneralController, MotionCancelled, PoseTimeoutError, HandModel, InspireHand,
                         JointPositionController, JointVelocityController,
                         ForceController, TargetPositionController)
from inspire_sdk.general_control import ZERO_POSE_NATIVE


CONTROLLERS = (JointPositionController, JointVelocityController, ForceController, TargetPositionController)


@pytest.fixture
def pose_backend(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr("inspire_sdk.general_control.time.monotonic", lambda: clock.now)

    class Backend:
        def __init__(self):
            self.description = HandModel("left")
            self.actual = self.description.upper * 0.4
            self.target_rad = self.actual.copy()
            self.writes = []
            self.reads = 0
            self.holds = 0
            self.follow = True
            self.faults = np.zeros(6, dtype=int)

        def read_state(self):
            self.reads += 1
            return SimpleNamespace(positions_rad=self.actual.copy(), error_codes=self.faults.copy())

        def set_positions(self, positions_rad):
            self.target_rad = self.description.positions(positions_rad)
            self.writes.append(self.target_rad.copy())

        def advance(self, duration_s):
            clock.now += duration_s
            if self.follow:
                self.actual = self.target_rad.copy()

        def stop(self):
            self.holds += 1
            self.target_rad = self.actual.copy()

    return Backend()


@pytest.mark.parametrize("controller_type", CONTROLLERS)
def test_all_controllers_inherit_both_named_poses(pose_backend, controller_type):
    controller = controller_type(pose_backend)
    assert isinstance(controller, GeneralController)
    opened = controller.open_hand()
    np.testing.assert_array_equal(pose_backend.description.to_registers(opened.positions_rad), [1000] * 6)
    zero = controller.return_to_zero()
    np.testing.assert_array_equal(pose_backend.description.to_registers(zero.positions_rad), ZERO_POSE_NATIVE)
    np.testing.assert_array_equal(pose_backend.target_rad, pose_backend.description.from_registers(ZERO_POSE_NATIVE))


def test_pose_reference_is_slewed_and_feedback_settles(pose_backend):
    initial = pose_backend.actual.copy()
    samples = []
    GeneralController(pose_backend).open_hand(on_state=lambda state, reference: samples.append(state))
    references = [initial, *pose_backend.writes]
    for previous, current in zip(references, references[1:]):
        assert np.all(np.abs(current - previous) <= 0.2 * 0.05 + 1e-10)
    assert len(samples) >= len(pose_backend.writes) + 2  # Three settled feedback samples.


@pytest.mark.parametrize("options", [{"rate_rad_s": float("nan")}, {"rate_rad_s": 2},
                                     {"period_s": 0}, {"timeout_s": 0}, {"tolerance_rad": 0},
                                     {"tolerance_units": 0}, {"tolerance_units": 1001},
                                     {"settle_samples": 0}, {"settle_samples": True}])
def test_invalid_pose_options_do_not_read_or_write(pose_backend, options):
    with pytest.raises(ValueError):
        GeneralController(pose_backend).open_hand(**options)
    assert not pose_backend.writes and pose_backend.reads == 0


def test_pose_timeout_holds_instead_of_reporting_arrival(pose_backend):
    pose_backend.follow = False
    with pytest.raises(TimeoutError):
        GeneralController(pose_backend).open_hand(timeout_s=0.2)
    assert pose_backend.holds == 1


def test_cancelled_before_first_pose_write(pose_backend):
    with pytest.raises(MotionCancelled):
        GeneralController(pose_backend).return_to_zero(cancelled=lambda: True)
    assert not pose_backend.writes and pose_backend.holds == 0


def test_fault_feedback_prevents_pose_commands(pose_backend):
    pose_backend.faults[0] = 1
    with pytest.raises(RuntimeError, match="Device faults"):
        GeneralController(pose_backend).open_hand()
    assert not pose_backend.writes


def test_facade_named_pose_discards_velocity_and_force_state(pose_backend):
    hand = InspireHand(pose_backend)
    hand.velocity.reference_rad = pose_backend.actual.copy()
    hand.velocity.velocities_rad_s[:] = 0.1
    hand.force.reference_rad = pose_backend.actual.copy()
    hand.force.targets_n[:] = 1
    hand.force.enabled[:] = True
    hand.open_hand()
    assert hand.velocity.reference_rad is None and hand.force.reference_rad is None
    np.testing.assert_array_equal(hand.velocity.velocities_rad_s, 0)
    np.testing.assert_array_equal(hand.force.targets_n, 0)
    assert not np.any(hand.force.enabled)


def native_feedback_backend(pose_backend, monkeypatch, offset):
    original = pose_backend.read_state
    pose_backend.native_position_feedback = True

    def read_native_state():
        state = original()
        state.raw_angles = np.clip(pose_backend.description.to_registers(pose_backend.actual) + offset, 0, 1000)
        state.positions_rad = pose_backend.description.from_registers(state.raw_angles)
        return state

    monkeypatch.setattr(pose_backend, "read_state", read_native_state)
    return pose_backend


def test_observed_open_feedback_985_passes_native_tolerance(pose_backend, monkeypatch):
    # Exact regression of the observed endpoint: four fingers and thumb bend
    # report 999, thumb rotation 985, with all error codes zero.
    backend = native_feedback_backend(pose_backend, monkeypatch, np.array([-1, -1, -1, -1, -1, -15]))
    progress = []
    state = GeneralController(backend).open_hand(on_progress=progress.append)
    np.testing.assert_array_equal(state.raw_angles, [999, 999, 999, 999, 999, 985])
    assert progress[-1]["source"] == "ANGLE_ACT"
    assert progress[-1]["settled_samples"] == 3
    assert progress[-1]["blocking_channels"] == []
    assert state.positions_rad[5] == pytest.approx(0.0174615)
    assert state.positions_rad[5] > 0.01  # Old model-radian gate could never complete.


def test_native_arrival_ignores_model_angle_estimate(pose_backend, monkeypatch):
    backend = native_feedback_backend(pose_backend, monkeypatch, np.array([-1, -1, -1, -1, -1, -15]))
    original = backend.read_state

    def misleading_model_estimate():
        state = original()
        # Emulate an unreliable model/display estimate. Actual device samples,
        # rather than this vector, must decide whether the pose is reached.
        state.positions_rad = backend.description.upper * 0.5
        return state

    monkeypatch.setattr(backend, "read_state", misleading_model_estimate)
    state = GeneralController(backend).open_hand()
    assert np.all(state.positions_rad > 0.01)
    np.testing.assert_array_equal(state.raw_angles, [999, 999, 999, 999, 999, 985])


@pytest.mark.parametrize("offset, tolerance, expected_error", [
    (-15, 10, 15), (-80, 20, 80),
])
def test_native_outside_tolerance_still_times_out_with_axis_details(pose_backend, monkeypatch,
                                                                  offset, tolerance, expected_error):
    pose_backend.actual[:] = 0
    backend = native_feedback_backend(pose_backend, monkeypatch, np.array([0, 0, 0, 0, 0, offset]))
    with pytest.raises(TimeoutError) as error:
        GeneralController(backend).open_hand(timeout_s=1.0, tolerance_units=tolerance)
    message = str(error.value)
    assert "source=ANGLE_ACT" in message and "thumb_rotation" in message
    assert f"{expected_error}" in message and "final_reference_sent=True" in message
    assert backend.holds == 1


def test_native_zero_requires_five_bent_channels_and_open_thumb_rotation(pose_backend, monkeypatch):
    backend = native_feedback_backend(pose_backend, monkeypatch, np.array([1, 1, 1, 1, 1, 15]))
    state = GeneralController(backend).return_to_zero()
    np.testing.assert_array_equal(state.raw_angles, [1, 1, 1, 1, 1, 1000])
    np.testing.assert_array_equal(backend.description.to_registers(backend.target_rad), ZERO_POSE_NATIVE)


def test_hardware_missing_raw_angles_never_falls_back_to_model(pose_backend):
    pose_backend.native_position_feedback = True
    with pytest.raises(RuntimeError, match="ANGLE_ACT feedback disappeared"):
        GeneralController(pose_backend).open_hand()
    assert not pose_backend.writes


def test_return_zero_restores_thumb_rotation_before_closing_fingers(pose_backend):
    initial = pose_backend.actual.copy()
    GeneralController(pose_backend).return_to_zero()
    closing = False
    for reference in pose_backend.writes:
        if not np.allclose(reference[:5], initial[:5], atol=1e-10):
            closing = True
        if closing:
            # All subsequent closing commands retain the root's open target.
            assert reference[5] == 0
        else:
            np.testing.assert_allclose(reference[:5], initial[:5])
    assert closing


def test_return_zero_never_slews_open_thumb_rotation_toward_palm(pose_backend, monkeypatch):
    pose_backend.actual[5] = 0
    backend = native_feedback_backend(pose_backend, monkeypatch, np.array([0, 0, 0, 0, 0, -15]))
    final = GeneralController(backend).return_to_zero()
    assert final.raw_angles[5] == 985
    assert all(backend.description.to_registers(reference)[5] == 1000 for reference in backend.writes)
    np.testing.assert_array_equal(backend.description.to_registers(backend.target_rad), ZERO_POSE_NATIVE)


def test_fixed_axis_away_from_target_rejects_before_motion(pose_backend):
    with pytest.raises(ValueError, match="Held channels"):
        GeneralController(pose_backend).move_to_positions(
            pose_backend.description.from_registers(ZERO_POSE_NATIVE), hold_channels=("thumb_rotation",))
    assert not pose_backend.writes


def test_timeout_captures_command_and_force_evidence_before_hold(pose_backend, monkeypatch):
    pose_backend.actual = pose_backend.description.from_registers([368, 79, 147, 159, 0, 985])
    backend = native_feedback_backend(pose_backend, monkeypatch, np.zeros(6, dtype=int))
    backend.follow = False
    backend.force_limits_units = np.array([100] * 6)
    original_read = backend.read_state

    def with_force_sample():
        state = original_read()
        state.raw_forces = np.array([118, 122, 116, 115, 48, 3] if backend.reads == 1 else
                                    [66, 110, 80, 85, 48, 3])
        return state

    def before_hold():
        assert backend.holds == 0
        return {"angle_set": backend.description.to_registers(backend.target_rad).tolist(),
                "force_set": [100] * 6, "status_codes": [3, 3, 3, 3, 2, 2]}

    monkeypatch.setattr(backend, "read_state", with_force_sample)
    monkeypatch.setattr(backend, "read_motion_diagnostics", before_hold, raising=False)
    with pytest.raises(PoseTimeoutError) as error:
        GeneralController(backend).move_to_positions(
            backend.description.from_registers(ZERO_POSE_NATIVE), hold_channels=("thumb_rotation",), timeout_s=5)
    report = error.value.diagnostics
    assert report["captured_before_hold"]
    assert report["device"]["angle_set"] == list(ZERO_POSE_NATIVE)
    assert report["peak_raw_forces"][:4] == [118, 122, 116, 115]
    assert backend.holds == 1
    # The current target has now become a measured hold; the report preserves
    # the earlier zero target rather than mistaking the hold for the failed command.
    assert backend.description.to_registers(backend.target_rad).tolist() == [368, 79, 147, 159, 0, 985]


def test_diagnostic_interrupt_still_requests_hold(pose_backend, monkeypatch):
    pose_backend.follow = False

    def interrupted_read():
        raise KeyboardInterrupt

    monkeypatch.setattr(pose_backend, "read_motion_diagnostics", interrupted_read, raising=False)
    with pytest.raises(KeyboardInterrupt):
        GeneralController(pose_backend).open_hand(timeout_s=0.2)
    assert pose_backend.holds == 1
