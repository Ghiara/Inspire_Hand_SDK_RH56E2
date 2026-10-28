"""Offline IK regressions: measured seeds, locked axes and excursion bounds.

These tests do not construct hardware clients. They are prepared for review and
have not been executed as part of the current code-only debugging request.
"""
import numpy as np
import pytest

from inspire_sdk import CHANNELS, InspireHand, MujocoBackend
from example.real_demos import SmallMotionDemo


@pytest.fixture(params=["left", "right"])
def hand(request):
    with MujocoBackend(request.param) as backend:
        yield InspireHand(backend)


def test_recorded_open_pose_does_not_reset_unselected_axes_to_commanded_zero(hand):
    backend = hand.backend
    state = hand.read_state()
    state.positions_rad = backend.description.from_registers([997, 997, 998, 998, 999, 984])
    np.testing.assert_array_equal(backend.target_rad, 0)  # Nominal open command.
    before = backend.target_rad.copy()
    demo = SmallMotionDemo(hand, "cartesian", state, ["index"], 0.12, 0.05)
    locked = [0, 1, 2, 4, 5]
    assert len(demo.ik_results) == 2
    for result in demo.ik_results:
        assert result.reachable and max(result.errors_m.values()) <= 0.001
        np.testing.assert_array_equal(result.positions_rad[locked], state.positions_rad[locked])
        assert np.all(result.positions_rad >= demo.lower_bound_rad)
        assert np.all(result.positions_rad <= demo.upper_bound_rad)
    np.testing.assert_array_equal(backend.target_rad, before)
    assert backend.data.time == 0  # Only FK/IK, no plant advance or motion command.


@pytest.mark.parametrize("channels", [["thumb_bend"], ["thumb_rotation"], ["thumb_bend", "thumb_rotation"]])
def test_partial_thumb_ik_cannot_move_unselected_axis(hand, channels):
    backend = hand.backend
    seed = backend.description.upper * 0.4
    desired = seed.copy()
    lower, upper = seed.copy(), seed.copy()
    for channel in channels:
        i = CHANNELS.index(channel)
        desired[i] += 0.03
        lower[i] -= 0.05
        upper[i] += 0.05
    target = backend.kinematics.forward(desired)["thumb"]
    result = hand.cartesian.solve({"thumb": target}, seed_rad=seed, active_channels=channels,
                                  lower_rad=lower, upper_rad=upper, tolerance_m=1e-5)
    assert result.reachable
    for i, channel in enumerate(CHANNELS):
        if channel not in channels:
            assert result.positions_rad[i] == seed[i]
    assert np.all(result.positions_rad >= lower) and np.all(result.positions_rad <= upper)


def test_reachable_in_full_model_is_rejected_when_outside_requested_excursion(hand):
    backend = hand.backend
    seed = backend.description.upper * 0.4
    desired = seed.copy()
    desired[3] += 0.3
    target = backend.kinematics.forward(desired)["index"]
    lower, upper = seed.copy(), seed.copy()
    lower[3] -= 0.05
    upper[3] += 0.05
    before = backend.target_rad.copy()
    with pytest.raises(ValueError, match="Unreachable"):
        hand.cartesian.set_targets({"index": target}, seed_rad=seed, active_channels=["index"],
                                   lower_rad=lower, upper_rad=upper, tolerance_m=1e-5)
    np.testing.assert_array_equal(backend.target_rad, before)
    projected = hand.cartesian.solve({"index": target}, seed_rad=seed, active_channels=["index"],
                                     lower_rad=lower, upper_rad=upper, tolerance_m=1e-5, project=True)
    assert not projected.reachable
    assert lower[3] <= projected.positions_rad[3] <= upper[3]


def test_equal_bounds_and_empty_active_set_keep_measured_seed(hand):
    seed = hand.backend.description.upper * 0.4
    target = hand.backend.kinematics.forward(seed)["index"]
    result = hand.cartesian.solve({"index": target}, seed_rad=seed, active_channels=[],
                                  lower_rad=seed, upper_rad=seed)
    np.testing.assert_array_equal(result.positions_rad, seed)
    assert result.reachable


def test_invalid_constraints_are_rejected_without_publishing(hand):
    seed = hand.backend.description.upper * 0.4
    target = hand.backend.kinematics.forward(seed)["index"]
    with pytest.raises(ValueError, match="active_channels"):
        hand.cartesian.solve({"index": target}, seed_rad=seed, active_channels=["unknown"])
    with pytest.raises(ValueError, match="lower_rad"):
        hand.cartesian.solve({"index": target}, seed_rad=seed, lower_rad=seed + 0.05, upper_rad=seed - 0.05)
    with pytest.raises(ValueError, match="seed_rad"):
        hand.cartesian.solve({"index": target}, seed_rad=seed, lower_rad=seed + 0.01)
    np.testing.assert_array_equal(hand.backend.target_rad, 0)
