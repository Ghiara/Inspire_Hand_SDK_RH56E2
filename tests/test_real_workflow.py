"""Offline checks: fake Modbus client, fake viewer and clock; no device access."""
from types import SimpleNamespace
import json

import numpy as np
import pytest

import main as cli
from example import real_demos
from inspire_sdk import InspireHand, JointVelocityController, MujocoBackend
from inspire_sdk.modbus_tcp import ModbusTcpBackend
from visualization.feedback_viewer import sync_feedback
from test_modbus import FakeClient


@pytest.fixture
def offline(monkeypatch):
    class FollowingClient(FakeClient):
        """Immediate arrival emulator for the opening/demo/zero lifecycle."""
        angles = [1000] * 6

        def read_holding_registers(self, **kwargs):
            response = super().read_holding_registers(**kwargs)
            if kwargs["address"] == 1546:
                response.registers = self.angles[:-1] if self.short_read else self.angles.copy()
            return response

        def write_registers(self, **kwargs):
            response = super().write_registers(**kwargs)
            if kwargs["address"] == 1486 and not self.error_response and not self.ack_mismatch:
                self.angles = list(kwargs["values"])
            return response

    client = FollowingClient()
    monkeypatch.setattr("inspire_sdk.modbus_tcp.ModbusTcpClient", lambda *a, **kw: client)

    class Clock:
        now = 0.0

        def monotonic(self):
            return self.now

        def sleep(self, duration):
            self.now += max(duration, 0.000001)

    clock = Clock()
    monkeypatch.setattr(real_demos.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(real_demos.time, "sleep", clock.sleep)

    class Viewer:
        closed = False
        running = True
        syncs = 0
        interrupt = False

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self.closed = True

        def is_running(self):
            return self.running

        def sync(self):
            self.syncs += 1
            if self.interrupt and self.syncs >= 2:
                raise KeyboardInterrupt

    viewer = Viewer()
    monkeypatch.setattr(real_demos, "open_viewer", lambda stack, scene: stack.enter_context(viewer))
    return SimpleNamespace(client=client, viewer=viewer, clock=clock)


def test_connection_check_only_reads_even_with_faults(offline, capsys):
    offline.client.fault = 2
    assert cli.main(["check", "--real", "--headless", "--samples", "2"]) == 1
    report = json.loads(capsys.readouterr().out)
    assert report["read_only"] and not report["healthy"]
    assert report["samples_read"] == 2
    assert report["states"][0]["velocity_source"] == "unavailable_first_sample"
    assert report["states"][1]["velocity_source"] == "finite_difference_of_angle_feedback"
    assert not offline.client.writes
    assert offline.client.closed


def test_extended_connection_check_is_still_read_only(offline, capsys):
    assert cli.main(["check", "--real", "--headless", "--samples", "1", "--diagnostics"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["motion_diagnostics"]["read_only"]
    assert "status_codes" in report["motion_diagnostics"]
    assert not offline.client.writes and offline.client.closed


def test_no_real_flag_never_creates_transport(monkeypatch, capsys, tmp_path, offline):
    def forbidden(*args, **kwargs):
        pytest.fail("Simulation attempted to create a hardware connection")
    monkeypatch.setattr("inspire_sdk.modbus_tcp.ModbusTcpClient", forbidden)
    assert cli.main(["check", "--headless", "--samples", "1"]) == 0
    assert json.loads(capsys.readouterr().out)["backend"] == "mujoco"
    assert cli.main(["demo-real", "position", "--headless", "--duration", "0.1",
                     "--log", str(tmp_path / "simulation.jsonl")]) == 0
    assert json.loads(capsys.readouterr().out)["backend"] == "mujoco"


@pytest.mark.parametrize("arguments", [
    ["--headless", "--real"], ["--duration", "nan"], ["--period", "0.001"],
    ["--period", "0.0205"], ["--amplitude", "0.31"], ["--rate", "0.21"],
    ["--speed-units", "0"], ["--force-units", "0"], ["--timeout", "nan"],
    ["--device-id", "0"], ["--port", "0"],
    ["--pose-rate", "0.21"], ["--pose-timeout", "0"], ["--pose-tolerance", "0.06"],
    ["--pose-tolerance-units", "0"], ["--pose-tolerance-units", "51"],
])
def test_invalid_demo_options_never_connect(monkeypatch, arguments):
    monkeypatch.setattr(real_demos, "connect_hand", lambda *a: pytest.fail("Unexpected connection"))
    assert cli.main(["demo-real", "position", *arguments]) == 1


@pytest.mark.parametrize("mode", ["position", "velocity", "cartesian"])
def test_mocked_hardware_demo_opens_runs_returns_zero_and_closes(offline, tmp_path, mode):
    log = tmp_path / f"{mode}.jsonl"
    assert cli.main(["demo-real", mode, "--real", "--duration", "0.15", "--log", str(log)]) == 0
    writes = offline.client.writes
    assert writes[:2] == [(1522, [100]*6, 1), (1498, [100]*6, 1)]
    assert writes[2] == (1486, [1000]*6, 1)  # Opening completes before the demo.
    assert writes[-1] == (1486, [0, 0, 0, 0, 0, 1000], 1)  # Thumb root stays open.
    assert all(values[5] == 1000 for address, values, _ in writes if address == 1486)
    assert offline.client.closed and offline.viewer.closed
    events = [json.loads(line) for line in log.read_text().splitlines()]
    assert events[0]["event"] == "start"
    assert events[0]["zero_native"] == [0, 0, 0, 0, 0, 1000]
    expected_phases = ["opening", *(["cartesian_preflight"] if mode == "cartesian" else []), "demo", "returning_zero"]
    assert [event["phase"] for event in events if event["event"] == "phase"] == expected_phases
    assert events[-1]["zero_reached"] and not events[-1]["hold_attempted"]
    assert events[-1]["hold_error"] is None
    assert events[-1]["ticks"] == 3
    for event in events:
        if event["event"] == "sample" and event["phase"] == "demo":
            np.testing.assert_allclose(np.array(event["reference_rad"])[[0, 1, 2, 4, 5]], 0)


def test_closed_viewer_before_motion_never_writes(offline, tmp_path):
    offline.viewer.running = False
    assert cli.main(["demo-real", "position", "--real", "--log", str(tmp_path / "closed.jsonl")]) == 0
    assert not offline.client.writes
    assert offline.client.closed


def test_keyboard_interrupt_holds_before_close(offline, tmp_path):
    offline.viewer.interrupt = True
    assert cli.main(["demo-real", "position", "--real", "--log", str(tmp_path / "ctrlc.jsonl")]) == 130
    assert offline.client.writes[-1] == (1486, [1000]*6, 1)
    assert offline.client.closed and offline.viewer.closed


def test_fault_at_start_prevents_limits_and_motion(offline, tmp_path):
    offline.client.fault = 1
    assert cli.main(["demo-real", "position", "--real", "--log", str(tmp_path / "fault.jsonl")]) == 1
    assert not offline.client.writes
    assert offline.client.closed


def test_ambiguous_position_write_attempts_hold(offline, monkeypatch, tmp_path):
    original = offline.client.write_registers

    def fail_first_angle(**kwargs):
        reply = original(**kwargs)
        if kwargs["address"] == 1486 and len(offline.client.writes) == 3:
            raise TimeoutError("lost angle write acknowledgement")
        return reply

    monkeypatch.setattr(offline.client, "write_registers", fail_first_angle)
    assert cli.main(["demo-real", "velocity", "--real", "--log", str(tmp_path / "timeout.jsonl")]) == 1
    assert offline.client.writes[-1] == (1486, [1000]*6, 1)
    # The general controller and runner may each request a measured hold, but
    # neither proceeds to the demo or to a closing/zero trajectory after failure.
    events = [json.loads(line) for line in (tmp_path / "timeout.jsonl").read_text().splitlines()]
    assert events[-1]["reason"] == "error" and not events[-1]["zero_reached"]
    assert not any(event.get("phase") == "returning_zero" for event in events)
    assert offline.client.closed


@pytest.mark.parametrize("side", ["left", "right"])
def test_feedback_mirror_contains_measured_mimics(side):
    client = FakeClient()
    with ModbusTcpBackend("fake", side=side, client=client) as hardware, MujocoBackend(side) as scene:
        state = hardware.read_state()
        state.positions_rad = scene.description.upper * 0.4
        # Passive positions must follow the same calibrated/model mapping.
        state.joint_velocities_rad_s = {n: 0.1 for n in scene.description.joints}
        sync_feedback(scene, state)
        displayed = scene.read_state()
        np.testing.assert_allclose(displayed.positions_rad, state.positions_rad)
        for name, value in scene.description.expand(state.positions_rad).items():
            assert displayed.joint_positions_rad[name] == pytest.approx(value)
        assert scene.data.time == 0  # Pose display does not step physics.
        assert not client.writes


@pytest.mark.parametrize("mode", ["position", "velocity", "cartesian"])
def test_small_motion_reference_stays_bounded_for_long_runs(mode):
    client = FakeClient()
    with ModbusTcpBackend("fake", client=client) as backend:
        hand = InspireHand(backend)
        initial = hand.read_state()
        demo = real_demos.SmallMotionDemo(hand, mode, initial, ["index"], 0.12, 0.05)
        for tick in range(1000):
            reference = demo.update(tick * 0.05, 0.05)
            assert 0 <= reference[3] <= 0.12 + 1e-9
            np.testing.assert_allclose(reference[[0, 1, 2, 4, 5]], 0, atol=1e-9)


def test_velocity_nonblocking_updates_preserve_reference():
    client = FakeClient()
    with ModbusTcpBackend("fake", client=client) as backend:
        controller = JointVelocityController(backend)
        controller.set_targets([0.1]*6)
        controller.update(0.1)
        reads_before = len(client.reads)
        controller.set_targets([-0.02]*6, reset_reference=False)
        assert len(client.reads) == reads_before
        np.testing.assert_allclose(controller.update(0.1), [0.008]*6)


def test_documented_tcp_unit_255(offline, capsys):
    assert cli.main(["check", "--real", "--device-id", "255", "--headless", "--samples", "1"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["device_id"] == 255
    assert all(unit == 255 for _, _, unit in offline.client.reads)
    assert not offline.client.writes


def test_viewer_closing_during_opening_holds(offline, monkeypatch, tmp_path):
    original_sync = offline.viewer.sync

    def close_on_motion():
        original_sync()
        if offline.viewer.syncs >= 2:
            offline.viewer.running = False

    monkeypatch.setattr(offline.viewer, "sync", close_on_motion)
    assert cli.main(["demo-real", "position", "--real", "--log", str(tmp_path / "close.jsonl")]) == 0
    assert offline.client.writes[-1] == (1486, [1000]*6, 1)
    assert offline.client.closed


def test_failed_hold_reports_failure_and_closes(offline, monkeypatch, tmp_path, capsys):
    original = offline.client.write_registers

    def fail_hold(**kwargs):
        reply = original(**kwargs)
        if kwargs["address"] == 1486 and kwargs["values"] == [1000]*6:
            raise TimeoutError("hold failed")
        return reply

    monkeypatch.setattr(offline.client, "write_registers", fail_hold)
    assert cli.main(["demo-real", "position", "--real", "--duration", "0.1",
                     "--log", str(tmp_path / "hold.jsonl")]) == 1
    assert "hold failed" in capsys.readouterr().err
    assert offline.client.closed


def test_viewer_closing_during_demo_does_not_return_zero(offline, monkeypatch, tmp_path):
    original_update = real_demos.SmallMotionDemo.update

    def close_during_demo(self, elapsed_s, duration_s):
        reference = original_update(self, elapsed_s, duration_s)
        offline.viewer.running = False
        return reference

    monkeypatch.setattr(real_demos.SmallMotionDemo, "update", close_during_demo)
    log = tmp_path / "demo_cancel.jsonl"
    assert cli.main(["demo-real", "position", "--real", "--log", str(log)]) == 0
    events = [json.loads(line) for line in log.read_text().splitlines()]
    assert events[-1]["reason"] == "cancelled"
    assert events[-1]["hold_attempted"] and not events[-1]["zero_reached"]
    assert not any(event.get("phase") == "returning_zero" for event in events)


def test_opening_timeout_never_starts_demo_or_zero(offline, monkeypatch, tmp_path):
    # Begin closed and withhold subsequent feedback: acknowledgements alone are
    # insufficient to enter the demo phase.
    offline.client.angles = [0] * 6
    original_read = offline.client.read_holding_registers

    def frozen_feedback(**kwargs):
        response = original_read(**kwargs)
        if kwargs["address"] == 1546:
            response.registers = [0] * 6
        return response

    monkeypatch.setattr(offline.client, "read_holding_registers", frozen_feedback)
    log = tmp_path / "opening_timeout.jsonl"
    assert cli.main(["demo-real", "position", "--real", "--pose-timeout", "0.2", "--log", str(log)]) == 1
    events = [json.loads(line) for line in log.read_text().splitlines()]
    assert [event["phase"] for event in events if event["event"] == "phase"] == ["opening"]
    assert events[-1]["reason"] == "error" and events[-1]["hold_attempted"]
    assert offline.client.closed


@pytest.mark.parametrize("side", ["right", "left"])
@pytest.mark.parametrize("channels", [["index", "middle"], ["thumb_bend", "thumb_rotation"],
                                     list(real_demos.CHANNELS)])
def test_cartesian_preflight_at_nonzero_pose(side, channels):
    client = FakeClient()
    with ModbusTcpBackend("fake", side=side, client=client) as backend:
        hand = InspireHand(backend)
        state = hand.read_state()
        state.positions_rad = backend.description.upper * 0.4
        backend.target_rad = state.positions_rad.copy()
        demo = real_demos.SmallMotionDemo(hand, "cartesian", state, channels, 0.12, 0.05)
        assert len(demo.ik_results) == 2
        assert all(result.reachable for result in demo.ik_results)
        assert not client.writes  # Both endpoints solved before any command.


def test_viewer_failure_precedes_connection(offline, monkeypatch, tmp_path):
    def broken_viewer(*args):
        raise RuntimeError("no display")
    monkeypatch.setattr(real_demos, "open_viewer", broken_viewer)
    monkeypatch.setattr(real_demos, "connect_hand", lambda *args: pytest.fail("Unexpected connection"))
    assert cli.main(["demo-real", "position", "--real", "--log", str(tmp_path / "display.jsonl")]) == 1
    assert not offline.client.reads and not offline.client.writes
