"""Cartesian fingertip control in hand_base coordinates, measured in meters."""


class TargetPositionController:
    def __init__(self, backend):
        self.backend = backend

    def solve(self, targets_m, tolerance_m=0.001, project=False):
        return self.backend.kinematics.inverse(targets_m, self.backend.target_rad,
                                               tolerance_m, project)

    def set_targets(self, targets_m, tolerance_m=0.001, project=False):
        result = self.solve(targets_m, tolerance_m, project)
        self.backend.set_positions(result.positions_rad)
        if hasattr(self.backend, "model"):
            for finger, target in result.requested_m.items():
                self.backend.model.site(f"{finger}_target").pos[:] = target
        return result
