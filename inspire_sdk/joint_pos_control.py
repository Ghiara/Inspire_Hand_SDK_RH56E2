"""Position commands for the six physical actuators, in radians."""
import numpy as np
from .model import CHANNELS, ACTIVE_JOINTS


class JointPositionController:
    def __init__(self, backend):
        self.backend = backend

    def set_targets(self, positions_rad):
        positions = self.backend.description.positions(positions_rad)
        self.backend.set_positions(positions)
        return positions

    def set_finger(self, channel, position_rad):
        if channel not in CHANNELS:
            raise ValueError(f"channel must be one of {CHANNELS}")
        # Preserve the previous command for other channels, including moving fingers.
        positions = self.backend.target_rad.copy()
        positions[CHANNELS.index(channel)] = position_rad
        return self.set_targets(positions)

    def set_joint(self, joint_name, position_rad):
        description = self.backend.description
        if joint_name not in description.joints or not np.isfinite(position_rad):
            raise ValueError("Expected a known revolute joint and a finite position_rad")
        limit = description.joints[joint_name].find("limit").attrib
        if not float(limit["lower"]) <= position_rad <= float(limit["upper"]):
            raise ValueError(f"{joint_name} target exceeds mechanical limits")
        # A passive-joint target moves its master and all coupled joints together.
        if joint_name in description.mimics:
            joint_name, multiplier, offset = description.mimics[joint_name]
            position_rad = (position_rad - offset) / multiplier
        return self.set_finger(CHANNELS[ACTIVE_JOINTS.index(joint_name)], position_rad)
