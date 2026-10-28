"""Signed rad/s commands integrated into bounded position references.

Call step() periodically. Hardware SPEED_SET is a speed limit, not a signed
velocity servo. Software integration is intentionally shared across backends.
"""
import numpy as np
from .model import positive


class JointVelocityController:
    def __init__(self, backend):
        self.backend = backend
        self.velocities_rad_s = np.zeros(6)
        self.reference_rad = None

    def set_targets(self, velocities_rad_s):
        values = self.backend.description.velocities(velocities_rad_s)
        # Switching into velocity mode starts at measured position, not a stale target.
        self.reference_rad = np.clip(self.backend.read_state().positions_rad,
                                     self.backend.description.lower, self.backend.description.upper)
        self.velocities_rad_s = values
        return values.copy()

    def step(self, duration_s):
        duration_s = positive(duration_s)
        if self.reference_rad is None:
            raise RuntimeError("Call set_targets() before step()")
        description = self.backend.description
        reference = np.clip(self.reference_rad + self.velocities_rad_s * duration_s,
                            description.lower, description.upper)
        self.backend.set_positions(reference)
        self.reference_rad = reference
        self.backend.advance(duration_s)

    def stop(self):
        self.velocities_rad_s[:] = 0
        self.backend.stop()
        self.reference_rad = None
