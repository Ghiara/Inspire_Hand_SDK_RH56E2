from dataclasses import dataclass, asdict
import numpy as np


@dataclass
class HandState:
    timestamp_s: float
    positions_rad: np.ndarray
    velocities_rad_s: np.ndarray
    joint_positions_rad: dict
    joint_velocities_rad_s: dict
    fingertip_positions_m: dict
    contact_forces_n: np.ndarray | None
    raw_forces: np.ndarray | None
    velocity_source: str
    force_source: str
    error_codes: np.ndarray | None = None
    raw_angles: np.ndarray | None = None
    position_source: str = "unspecified"

    def to_dict(self):
        def convert(value):
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, dict):
                return {k: convert(v) for k, v in value.items()}
            return value
        return convert(asdict(self))
