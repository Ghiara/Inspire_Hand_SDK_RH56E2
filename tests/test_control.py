import json
import numpy as np
import pytest
from inspire_sdk import (HandModel, MujocoBackend, JointPositionController,
                         JointVelocityController, ForceController, TargetPositionController,
                         FINGERS, CHANNELS)


@pytest.fixture(params=["right", "left"])
def backend(request):
    with MujocoBackend(request.param) as backend:
        yield backend


def test_position_all_channels_and_mimics(backend):
    target = backend.description.upper * np.array([0.2, 0.35, 0.5, 0.65, 0.7, 0.8])
    JointPositionController(backend).set_targets(target)
    initial = backend.read_state().positions_rad
    np.testing.assert_allclose(initial, 0)  # Commands must not teleport the hand.
    backend.advance(3)
    state = backend.read_state()
    np.testing.assert_allclose(state.positions_rad, target, atol=1e-4)
    for name, (parent, multiplier, offset) in backend.description.mimics.items():
        assert abs(state.joint_positions_rad[name] - multiplier * state.joint_positions_rad[parent] - offset) < 1e-4
    assert len(state.joint_positions_rad) == 12
    assert len(state.joint_velocities_rad_s) == 12
    assert set(state.fingertip_positions_m) == set(FINGERS)
    np.testing.assert_allclose(state.contact_forces_n, 0)
    json.dumps(state.to_dict(), allow_nan=False)


@pytest.mark.parametrize("channel", CHANNELS)
def test_individual_channel_and_passive_joint(backend, channel):
    controller = JointPositionController(backend)
    i = CHANNELS.index(channel)
    controller.set_finger(channel, backend.description.upper[i] * 0.5)
    backend.advance(2)
    target = np.zeros(6)
    target[i] = backend.description.upper[i] * 0.5
    np.testing.assert_allclose(backend.read_state().positions_rad, target, atol=1e-4)
    controller.set_joint("index_dip", 0.5)
    assert backend.target_rad[3] == pytest.approx(0.5 / 1.0843)


@pytest.mark.parametrize("invalid", [[1]*5, [1]*7, [np.nan]*6, [np.inf]*6, [-0.1]*6, [10]*6, [[0]*6]])
def test_position_rejects_invalid_before_motion(backend, invalid):
    before = backend.target_rad.copy()
    with pytest.raises(ValueError):
        JointPositionController(backend).set_targets(invalid)
    np.testing.assert_array_equal(backend.target_rad, before)
    assert backend.data.time == 0


def test_velocity_tracking_reversal_zero_and_limit(backend):
    controller = JointVelocityController(backend)
    controller.set_targets([0.2]*6)
    for _ in range(100):
        controller.step(0.01)
    state = backend.read_state()
    np.testing.assert_allclose(state.velocities_rad_s, 0.2, atol=0.01)
    np.testing.assert_allclose(state.positions_rad, 0.2, atol=0.02)
    controller.set_targets([-0.2]*6)
    for _ in range(150):
        controller.step(0.01)
    backend.advance(0.5)
    np.testing.assert_allclose(backend.read_state().positions_rad, 0, atol=1e-4)
    controller.set_targets([0]*6)
    controller.step(0.1)
    np.testing.assert_allclose(backend.read_state().velocities_rad_s, 0, atol=1e-4)
    with pytest.raises(ValueError):
        controller.set_targets([2]*6)
    with pytest.raises(ValueError):
        controller.step(float("nan"))


def test_upper_limits_and_stop(backend):
    controller = JointVelocityController(backend)
    controller.set_targets(backend.description.max_velocity)
    for _ in range(250):
        controller.step(0.01)
    backend.advance(0.5)
    np.testing.assert_allclose(backend.read_state().positions_rad, backend.description.upper, atol=1e-4)
    controller.stop()
    before = backend.read_state().positions_rad
    backend.advance(0.5)
    np.testing.assert_allclose(backend.read_state().positions_rad, before, atol=1e-4)


def test_cartesian_all_fingers_reachable(backend):
    desired = backend.description.upper * [0.3, 0.4, 0.5, 0.6, 0.55, 0.65]
    targets = backend.kinematics.forward(desired)
    result = TargetPositionController(backend).set_targets(targets, tolerance_m=1e-5)
    assert result.reachable
    assert max(result.errors_m.values()) < 1e-5
    backend.advance(3)
    actual = backend.read_state().fingertip_positions_m
    for finger in FINGERS:
        assert np.linalg.norm(actual[finger] - targets[finger]) < 0.0005


def test_unreachable_and_projection_are_explicit(backend):
    controller = TargetPositionController(backend)
    before = backend.target_rad.copy()
    with pytest.raises(ValueError, match="Unreachable"):
        controller.set_targets({"index": [10, 10, 10]})
    np.testing.assert_array_equal(backend.target_rad, before)
    projected = controller.set_targets({"index": [10, 10, 10]}, project=True)
    assert not projected.reachable
    assert projected.errors_m["index"] > 1
    with pytest.raises(ValueError):
        controller.set_targets({"index": [np.nan, 0, 0]})


@pytest.mark.parametrize("side", ["right", "left"])
def test_force_all_fingers_contact_and_release(side):
    with MujocoBackend(side, contact_fixture=True) as backend:
        controller = ForceController(backend)
        controller.set_targets([1]*5)
        for _ in range(1000):
            controller.step(0.01)
        state = backend.read_state()
        np.testing.assert_allclose(state.contact_forces_n, 1, atol=0.15)
        assert np.all(state.positions_rad >= backend.description.lower - 0.0002)
        assert np.all(state.positions_rad <= backend.description.upper + 0.0002)
        controller.stop()
        JointPositionController(backend).set_targets([0]*6)
        backend.advance(3)
        np.testing.assert_allclose(backend.read_state().contact_forces_n, 0, atol=0.001)


def test_force_free_space_and_safety(backend):
    controller = ForceController(backend)
    with pytest.raises(ValueError):
        controller.set_targets([-1]*5)
    with pytest.raises(ValueError):
        controller.set_targets([6]*5)
    controller.set_targets([1]*5)
    for _ in range(500):
        controller.step(0.01)
    np.testing.assert_array_equal(backend.read_state().contact_forces_n, np.zeros(5))
    assert np.all(backend.target_rad <= backend.description.upper)


def test_native_mapping_and_side():
    right, left = HandModel("right"), HandModel("left")
    np.testing.assert_array_equal(right.to_registers(np.zeros(6)), [1000]*6)
    np.testing.assert_array_equal(right.to_registers(right.upper), [0]*6)
    np.testing.assert_allclose(right.from_registers([500]*6), right.upper * 0.5)
    np.testing.assert_array_equal(right.upper, left.upper)
    from inspire_sdk.kinematics import Kinematics
    r = Kinematics(right).forward(right.upper * 0.4)
    l = Kinematics(left).forward(left.upper * 0.4)
    for finger in FINGERS:
        np.testing.assert_allclose(l[finger], r[finger] * [1, -1, 1], atol=1e-6)


def test_close_rejects_commands(backend):
    backend.close()
    with pytest.raises(RuntimeError):
        backend.set_positions([0]*6)
    with pytest.raises(RuntimeError):
        backend.read_state()
