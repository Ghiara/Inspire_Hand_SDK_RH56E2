"""Reusable demos; the CLI supplies visualization, timing and recording."""
import numpy as np
from inspire_sdk import JointPositionController, JointVelocityController, ForceController, TargetPositionController


class Demo:
    def __init__(self, backend, mode):
        self.backend = backend
        self.mode = mode
        self.position = JointPositionController(backend)
        self.velocity = JointVelocityController(backend)
        self.force = ForceController(backend)
        self.cartesian = TargetPositionController(backend)
        self.phase = -1
        self.ik_results = []
        if mode == "force":
            self.force.set_targets(np.ones(5))
        elif mode == "velocity":
            self.velocity.set_targets(np.full(6, 0.25))

    def step(self, duration_s):
        now = self.backend.data.time
        if self.mode == "force":
            self.force.step(duration_s)
        elif self.mode == "velocity":
            phase = int(now / 2)
            if phase != self.phase:
                self.velocity.set_targets(np.full(6, 0.25 if phase % 2 == 0 else -0.25))
                self.phase = phase
            self.velocity.step(duration_s)
        elif self.mode in ("position", "cartesian"):
            phase = int(now / 3)
            if phase != self.phase:
                ratio = 0.35 if phase % 2 == 0 else 0.7
                target = self.backend.description.upper * ratio
                if self.mode == "position":
                    self.position.set_targets(target)
                else:
                    tips = self.backend.kinematics.forward(target)
                    self.ik_results.append(self.cartesian.set_targets(tips))
                self.phase = phase
            self.backend.advance(duration_s)
        else:
            raise ValueError(f"Unknown mode {self.mode}")
