"""FK and constrained IK use the same copied model and mimic constraints as simulation."""
from dataclasses import dataclass
import mujoco
import numpy as np
from scipy.optimize import least_squares
from .model import ACTIVE_JOINTS, CHANNELS, FINGERS, vector, positive


@dataclass
class IKResult:
    positions_rad: np.ndarray
    requested_m: dict
    achieved_m: dict
    errors_m: dict
    reachable: bool


class Kinematics:
    def __init__(self, description):
        self.description = description
        self.model = mujoco.MjModel.from_xml_path(str(description.scene_path))
        self.data = mujoco.MjData(self.model)
        self.addresses = {n: self.model.jnt_qposadr[self.model.joint(n).id]
                          for n in description.joints}

    def forward(self, positions_rad):
        for name, value in self.description.expand(positions_rad).items():
            self.data.qpos[self.addresses[name]] = value
        mujoco.mj_forward(self.model, self.data)
        return {finger: self.data.site(f"{finger}_tip").xpos.copy() for finger in FINGERS}

    def inverse(self, targets_m, seed_rad=None, tolerance_m=0.001, project=False, *,
                active_channels=None, lower_rad=None, upper_rad=None):
        tolerance_m = positive(tolerance_m, "tolerance_m")
        if not targets_m or any(finger not in FINGERS for finger in targets_m):
            raise ValueError(f"targets_m must map one or more of {FINGERS} to xyz meters")
        targets = {f: vector(p, 3, f"{f} target_m") for f, p in targets_m.items()}
        positions = self.description.positions(seed_rad if seed_rad is not None else np.zeros(6))
        lower = self.description.lower.copy() if lower_rad is None else self.description.positions(lower_rad)
        upper = self.description.upper.copy() if upper_rad is None else self.description.positions(upper_rad)
        if np.any(lower > upper):
            raise ValueError("IK lower_rad must not exceed upper_rad")
        if np.any(positions < lower) or np.any(positions > upper):
            raise ValueError("IK seed_rad must be inside the requested joint bounds")
        if active_channels is None:
            active = set(CHANNELS)
        else:
            if isinstance(active_channels, str):
                raise ValueError("active_channels must be a collection of channel names")
            active = set(active_channels)
            if not active.issubset(CHANNELS):
                raise ValueError(f"active_channels must contain only {CHANNELS}")
        # Four fingers trace 1D curves; the coupled thumb traces a 2D surface.
        # Like inspire-sdk's nearest-point operator, search only the reachable manifold.
        for finger, target in targets.items():
            indices = [4, 5] if finger == "thumb" else [FINGERS.index(finger)]
            # Unselected axes retain the caller's seed exactly. Equal bounds
            # also fix an axis; scipy requires strictly separated free bounds.
            indices = [i for i in indices if CHANNELS[i] in active and upper[i] > lower[i]]
            if not indices:
                continue
            lo, hi = lower[indices], upper[indices]
            span = hi - lo

            def residual(values):
                candidate = positions.copy()
                candidate[indices] = values
                return self.forward(candidate)[finger] - target

            # Multiple seeds cover folded fingers and thumb surface boundaries.
            seeds = [positions[indices], lo + span * 0.15, lo + span * 0.5, lo + span * 0.85]
            if len(indices) == 2:
                seeds += [lo + span * [0.15, 0.85], lo + span * [0.85, 0.15]]
            results = [least_squares(residual, np.clip(s, lo, hi),
                                     bounds=(lo, hi),
                                     ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=150)
                       for s in seeds]
            best = min(results, key=lambda r: np.linalg.norm(r.fun))
            positions[indices] = best.x
        achieved = self.forward(positions)
        errors = {f: float(np.linalg.norm(achieved[f] - p)) for f, p in targets.items()}
        reachable = all(e <= tolerance_m for e in errors.values())
        result = IKResult(positions, targets, {f: achieved[f] for f in targets}, errors, reachable)
        if not reachable and not project:
            raise ValueError(f"Unreachable fingertip target; residuals in meters: {errors}. Use project=True to project explicitly.")
        return result
