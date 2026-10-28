"""RH56E2 channel order and limits, taken from the vendored URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
CHANNELS = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")
FINGERS = ("pinky", "ring", "middle", "index", "thumb")
ACTIVE_JOINTS = ("pinky_joint", "ring_joint", "middle_joint", "index_joint",
                 "thumb_joint2", "thumb_joint1")


def vector(value, size=6, name="values"):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result.copy()


def positive(value, name="duration"):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


class HandModel:
    def __init__(self, side="right"):
        if side not in ("right", "left"):
            raise ValueError("side must be right or left")
        self.side = side
        self.urdf_path = ASSETS / f"rh56e2_{side}.urdf"
        self.scene_path = ASSETS / f"rh56e2_{side}.xml"
        if not self.urdf_path.exists():
            raise FileNotFoundError("Missing assets; run python -m visualization.build_assets")
        root = ET.parse(self.urdf_path).getroot()
        self.joints = {j.attrib["name"]: j for j in root.findall("joint")
                       if j.attrib["type"] == "revolute"}
        self.lower = np.array([float(self.joints[n].find("limit").attrib["lower"])
                               for n in ACTIVE_JOINTS])
        self.upper = np.array([float(self.joints[n].find("limit").attrib["upper"])
                               for n in ACTIVE_JOINTS])
        self.max_velocity = np.array([float(self.joints[n].find("limit").attrib["velocity"])
                                     for n in ACTIVE_JOINTS])
        self.mimics = {}
        for name, joint in self.joints.items():
            mimic = joint.find("mimic")
            if mimic is not None:
                parent = mimic.attrib["joint"]
                multiplier = float(mimic.attrib.get("multiplier", 1))
                offset = float(mimic.attrib.get("offset", 0))
                self.mimics[name] = (parent, multiplier, offset)
                i = ACTIVE_JOINTS.index(parent)
                limit = joint.find("limit")
                # The copied RH56E2 has positive, linear mimic ratios.
                self.upper[i] = min(self.upper[i], (float(limit.attrib["upper"]) - offset) / multiplier)
                self.max_velocity[i] = min(self.max_velocity[i], float(limit.attrib["velocity"]) / multiplier)

    def positions(self, positions_rad):
        positions = vector(positions_rad, name="positions_rad")
        if np.any(positions < self.lower - 1e-10) or np.any(positions > self.upper + 1e-10):
            raise ValueError(f"positions_rad exceed mechanical limits: {self.lower} .. {self.upper}")
        return np.clip(positions, self.lower, self.upper)

    def velocities(self, velocities_rad_s):
        velocities = vector(velocities_rad_s, name="velocities_rad_s")
        if np.any(np.abs(velocities) > self.max_velocity + 1e-10):
            raise ValueError(f"velocities_rad_s exceed URDF limits {self.max_velocity}")
        return velocities

    def expand(self, positions_rad):
        positions = self.positions(positions_rad)
        result = dict(zip(ACTIVE_JOINTS, positions))
        result.update({name: result[parent] * ratio + offset
                       for name, (parent, ratio, offset) in self.mimics.items()})
        return result

    def to_registers(self, positions_rad):
        # Native ANGLE_SET: 1000=open, 0=closed; model q=0 is open.
        return np.rint(1000 * (1 - (self.positions(positions_rad) - self.lower)
                                  / (self.upper - self.lower))).astype(int)

    def from_registers(self, registers):
        registers = vector(registers, name="angle registers")
        if np.any(registers < 0) or np.any(registers > 1000):
            raise ValueError("Invalid ANGLE_ACT feedback (expected 0..1000)")
        return self.lower + (1 - registers / 1000) * (self.upper - self.lower)
