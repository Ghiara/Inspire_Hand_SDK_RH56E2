"""FK and constrained IK use the same copied model and mimic constraints as simulation."""
from dataclasses import dataclass
import mujoco
import numpy as np
from scipy.optimize import least_squares
from .model import ACTIVE_JOINTS, FINGERS, vector, positive


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

    def inverse(self, targets_m, seed_rad=None, tolerance_m=0.001, project=False):
        tolerance_m = positive(tolerance_m, "tolerance_m")
        if not targets_m or any(finger not in FINGERS for finger in targets_m):
            raise ValueError(f"targets_m must map one or more of {FINGERS} to xyz meters")
        targets = {f: vector(p, 3, f"{f} target_m") for f, p in targets_m.items()}
        positions = self.description.positions(seed_rad if seed_rad is not None else np.zeros(6))
        # Four fingers trace 1D curves; the coupled thumb traces a 2D surface.
        # Like inspire-sdk's nearest-point operator, search only the reachable manifold.
        for finger, target in targets.items():
            indices = [4, 5] if finger == "thumb" else [FINGERS.index(finger)]
            upper = self.description.upper[indices]

            def residual(values):
                candidate = positions.copy()
                candidate[indices] = values
                return self.forward(candidate)[finger] - target

            # Multiple seeds cover folded fingers and thumb surface boundaries.
            seeds = [positions[indices], upper * 0.15, upper * 0.5, upper * 0.85]
            if len(indices) == 2:
                seeds += [upper * [0.15, 0.85], upper * [0.85, 0.15]]
            results = [least_squares(residual, np.clip(s, 1e-9, upper - 1e-9),
                                     bounds=(np.zeros(len(indices)), upper),
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
