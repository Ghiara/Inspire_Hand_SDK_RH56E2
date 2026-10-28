"""Signed rad/s commands integrated into bounded position references.

Call step() periodically. Hardware SPEED_SET is a speed limit, not a signed
velocity servo. Software integration is intentionally shared across backends.
"""
import numpy as np
from .model import positive
from .general_control import GeneralController


class JointVelocityController(GeneralController):
    def __init__(self, backend):
        super().__init__(backend)
        self.velocities_rad_s = np.zeros(6)
        self.reference_rad = None

    def set_targets(self, velocities_rad_s, *, reset_reference=True):
        """Set signed speeds; optionally preserve an external loop's reference."""
        values = self.backend.description.velocities(velocities_rad_s)
        # Switching into velocity mode starts at measured position, not a stale target.
        if reset_reference or self.reference_rad is None:
            self.reference_rad = np.clip(self.backend.read_state().positions_rad,
                                         self.backend.description.lower, self.backend.description.upper)
        self.velocities_rad_s = values
        return values.copy()

    def update(self, duration_s):
        """Publish one integrated reference without sleeping or advancing physics.

        External schedulers can call this at their own rate. Delayed cycles should
        use a bounded duration rather than catching up with a large position jump.
        """
        duration_s = positive(duration_s)
        if self.reference_rad is None:
            raise RuntimeError("Call set_targets() before step()")
        description = self.backend.description
        reference = np.clip(self.reference_rad + self.velocities_rad_s * duration_s,
                            description.lower, description.upper)
        self.backend.set_positions(reference)
        self.reference_rad = reference
        return reference.copy()

    def step(self, duration_s):
        self.update(duration_s)
        self.backend.advance(duration_s)

    def _reset_control_state(self):
        self.velocities_rad_s[:] = 0
        self.reference_rad = None
