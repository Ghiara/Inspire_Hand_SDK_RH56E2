import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import pytest

ROOT = Path(__file__).resolve().parents[1]


def run(*args):
    env = dict(os.environ, PYTHONPATH="", MUJOCO_GL="osmesa")
    return subprocess.run([sys.executable, *args], cwd=ROOT, env=env, text=True,
                          capture_output=True, timeout=60)


@pytest.mark.parametrize("mode", ["position", "velocity", "force", "cartesian"])
def test_headless_demo_cli(mode):
    result = run("main.py", "demo", mode, "--headless", "--duration", "0.1")
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["mode"] == mode


@pytest.mark.parametrize("script", ["joint_pos_demo.py", "joint_velo_demo.py", "force_demo.py", "target_pos_demo.py"])
def test_example_scripts(script):
    result = run(f"example/{script}", "--headless", "--duration", "0.1")
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("arguments", [
    ["visualization/mujoco_viewer.py", "--headless", "--duration", "0.1"],
    ["main.py", "state"],
])
def test_viewer_and_state(arguments):
    result = run(*arguments)
    assert result.returncode == 0, result.stderr


def test_invalid_cli_never_connects_hardware():
    result = run("main.py", "hardware", "--host", "127.0.0.1", "--positions-rad", *["999"]*6)
    assert result.returncode == 1
    assert "mechanical limits" in result.stderr


def test_mp4_encoding_and_decode(tmp_path):
    import imageio.v2 as imageio
    video = tmp_path / "test.mp4"
    result = run("visualization/record_video.py", "position", "--duration", "0.4", "--record", str(video))
    assert result.returncode == 0, result.stderr
    summary = json.loads(result.stdout)
    assert summary["frames"] == 10
    assert video.exists() and video.stat().st_size > 1000
    with imageio.get_reader(video) as reader:
        assert reader.count_frames() == 10
        assert reader.get_data(0).shape == (480, 640, 3)
        assert reader.get_meta_data()["fps"] == 25


@pytest.mark.parametrize("interrupt", [False, True], ids=["window_close", "ctrl_c"])
def test_visualization_without_duration_runs_until_user_exit(monkeypatch, interrupt):
    import numpy as np
    import main as cli
    import inspire_sdk
    import mujoco.viewer

    assert cli.parser().parse_args(["visualize"]).duration is None
    assert cli.parser().parse_args(["demo", "position"]).duration == 10.0

    class Backend:
        description = SimpleNamespace(side="right")
        model = data = None
        steps = 0
        closed = False

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self.closed = True

        def advance(self, duration):
            self.steps += 1

        def read_state(self):
            return SimpleNamespace(to_dict=lambda: {})

    backend = Backend()

    class Viewer:
        cam = SimpleNamespace(lookat=np.zeros(3))
        closed = False

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self.closed = True

        def is_running(self):
            # Stay open past the previous default of 1000 steps (10 seconds).
            return backend.steps < 1100

        def sync(self):
            if interrupt and backend.steps == 1100:
                raise KeyboardInterrupt

    viewer = Viewer()
    monkeypatch.setattr(inspire_sdk, "MujocoBackend", lambda *args, **kwargs: backend)
    monkeypatch.setattr(mujoco.viewer, "launch_passive", lambda *args: viewer)
    monkeypatch.setattr(cli.time, "sleep", lambda seconds: None)
    assert cli.main(["visualize"]) == (130 if interrupt else 0)
    assert backend.steps == 1100
    assert backend.closed and viewer.closed
