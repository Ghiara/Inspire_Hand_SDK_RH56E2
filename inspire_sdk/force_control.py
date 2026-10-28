"""Simulation contact-force feedback and native hardware force-threshold control."""
import numpy as np
from .model import FINGERS, vector, positive


class ForceController:
    def __init__(self, backend):
        self.backend = backend
        self.targets_n = np.zeros(5)
        self.enabled = np.zeros(5, dtype=bool)
        self.reference_rad = None
        self.filtered_n = np.zeros(5)

    def set_targets(self, forces_n):
        if self.backend.read_state().contact_forces_n is None:
            raise NotImplementedError("Newton force feedback needs hardware calibration. Use set_native_limits() for Modbus thresholds.")
        forces = vector(forces_n, 5, "forces_n")
        if np.any(forces < 0) or np.any(forces > 5):
            raise ValueError("forces_n must be within 0..5 N per finger")
        self.reference_rad = np.clip(self.backend.read_state().positions_rad,
                                     self.backend.description.lower, self.backend.description.upper)
        self.targets_n = forces
        self.enabled = forces > 0
        self.filtered_n[:] = 0

    def step(self, duration_s):
        duration_s = positive(duration_s)
        if self.reference_rad is None:
            raise RuntimeError("Call set_targets() before step()")
        measured = self.backend.read_state().contact_forces_n
        alpha = 1 - np.exp(-duration_s / 0.03)
        self.filtered_n += alpha * (measured - self.filtered_n)
        velocity = np.clip(0.35 * (self.targets_n - self.filtered_n), -0.3, 0.3)
        velocity[~self.enabled] = 0
        description = self.backend.description
        reference = self.reference_rad.copy()
        reference[:5] += velocity * duration_s
        reference = np.clip(reference, description.lower, description.upper)
        self.backend.set_positions(reference)
        self.reference_rad = reference
        self.backend.advance(duration_s)

    def set_native_limits(self, force_units):
        values = vector(force_units, name="force_units")
        if np.any(values < 0) or np.any(values > 1000):
            raise ValueError("force_units must be within 0..1000")
        if not hasattr(self.backend, "set_force_limits"):
            raise NotImplementedError("Native force units are available only on Modbus hardware")
        self.backend.set_force_limits(values)

    def stop(self):
        self.backend.stop()
        self.reference_rad = None
