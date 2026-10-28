"""Streaming MP4 recorder; can be used with any simulation control loop."""
from pathlib import Path
import sys
import imageio.v2 as imageio


def camera_for(backend):
    import mujoco
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [-0.005, 0, 0.11]
    camera.distance = 0.42
    camera.azimuth = -75 if backend.description.side == "right" else 75
    camera.elevation = -15
    return camera


class VideoRecorder:
    def __init__(self, backend, path, fps=25, width=640, height=480):
        import mujoco
        if not isinstance(fps, int) or not 1 <= fps <= 100:
            raise ValueError("fps must be an integer in 1..100")
        if width < 16 or height < 16 or width % 16 or height % 16:
            raise ValueError("width/height must be positive multiples of 16")
        self.path = Path(path).expanduser().resolve()
        if self.path.suffix.lower() != ".mp4":
            raise ValueError("Recording output must have an .mp4 extension")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.backend = backend
        self.fps = fps
        self.next_frame_s = 0.0
        self.frame_count = 0
        self.renderer = mujoco.Renderer(backend.model, height=height, width=width)
        self.writer = None
        try:
            self.writer = imageio.get_writer(str(self.path), fps=fps, codec="libx264", quality=7,
                                            pixelformat="yuv420p", macro_block_size=16)
        except Exception:
            self.renderer.close()
            raise
        self.camera = camera_for(backend)

    def capture(self):
        # Timestamp schedule prevents rounding drift and repeated-frame bursts.
        if self.backend.data.time + 1e-9 < self.next_frame_s:
            return
        self.renderer.update_scene(self.backend.data, camera=self.camera)
        self.writer.append_data(self.renderer.render())
        self.frame_count += 1
        self.next_frame_s = self.frame_count / self.fps

    def close(self):
        try:
            if self.writer is not None:
                self.writer.close()
                self.writer = None
        finally:
            self.renderer.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def main():
    # Forward CLI to the shared demo runner; parse before importing MuJoCo for headless GL.
    from main import main as run
    arguments = ["demo", *sys.argv[1:], "--headless"]
    if not any(arg == "--record" or arg.startswith("--record=") for arg in arguments):
        arguments.append("--record")
    return run(arguments)


if __name__ == "__main__":
    if __package__ in (None, ""):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
