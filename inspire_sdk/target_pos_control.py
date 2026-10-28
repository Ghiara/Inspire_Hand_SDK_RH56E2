"""Cartesian fingertip control in hand_base coordinates, measured in meters."""
from .general_control import GeneralController


class TargetPositionController(GeneralController):
    def solve(self, targets_m, tolerance_m=0.001, project=False, *, seed_rad=None,
              active_channels=None, lower_rad=None, upper_rad=None):
        """Solve only, optionally from measured joints and with locked/bounded axes.

        The legacy default seeds from the previous command. Hardware demos
        explicitly pass measured joints so unselected channels keep that pose.
        """
        seed = self.backend.target_rad if seed_rad is None else seed_rad
        return self.backend.kinematics.inverse(
            targets_m, seed, tolerance_m, project, active_channels=active_channels,
            lower_rad=lower_rad, upper_rad=upper_rad)

    def set_targets(self, targets_m, tolerance_m=0.001, project=False, *, seed_rad=None,
                    active_channels=None, lower_rad=None, upper_rad=None):
        result = self.solve(targets_m, tolerance_m, project, seed_rad=seed_rad,
                            active_channels=active_channels, lower_rad=lower_rad, upper_rad=upper_rad)
        self.backend.set_positions(result.positions_rad)
        if hasattr(self.backend, "model"):
            for finger, target in result.requested_m.items():
                self.backend.model.site(f"{finger}_target").pos[:] = target
        return result
