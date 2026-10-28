"""Torque-driven MuJoCo backend; commands never overwrite simulation qpos."""
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .model import HandModel, ACTIVE_JOINTS, FINGERS, vector, positive
from .state import HandState
from .kinematics import Kinematics


class MujocoBackend:
    def __init__(self, side="right", contact_fixture=False):
        self.description = HandModel(side)
        self.kinematics = Kinematics(self.description)
        if contact_fixture:
            root = ET.parse(self.description.scene_path).getroot()
            # Put a fixed sphere on each fingertip's closing path.
            fixture_points = self.kinematics.forward(self.description.upper * [0.5, 0.5, 0.5, 0.5, 0.5, 0.25])
            world = root.find("worldbody")
            for finger, position in fixture_points.items():
                ET.SubElement(world, "geom", name=f"fixture_{finger}", type="sphere",
                              pos=" ".join(map(str, position)), size="0.008", contype="2", conaffinity="1",
                              rgba="0.9 0.45 0.12 1")
            # Resolve mesh files against assets, including when XML is loaded from a string.
            for mesh in root.findall("asset/mesh"):
                mesh.set("file", str(self.description.scene_path.parent / mesh.attrib["file"]))
            self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        else:
            self.model = mujoco.MjModel.from_xml_path(str(self.description.scene_path))
        self.data = mujoco.MjData(self.model)
        self.q_addresses = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in ACTIVE_JOINTS])
        self.v_addresses = np.array([self.model.jnt_dofadr[self.model.joint(n).id] for n in ACTIVE_JOINTS])
        self.target_rad = np.zeros(6)
        self.reference_rad = np.zeros(6)
        self.closed = False
        self._body_fingers = {}
        for body_id in range(self.model.nbody):
            current = body_id
            while current:
                name = self.model.body(current).name
                finger = next((f for f in FINGERS if name.startswith(f + "_")), None)
                if finger:
                    self._body_fingers[body_id] = FINGERS.index(finger)
                    break
                current = self.model.body_parentid[current]
        mujoco.mj_forward(self.model, self.data)

    def _ensure_open(self):
        if self.closed:
            raise RuntimeError("Backend is closed")

    def set_positions(self, positions_rad):
        self._ensure_open()
        self.target_rad = self.description.positions(positions_rad)

    def advance(self, duration_s):
        self._ensure_open()
        duration_s = positive(duration_s)
        steps = int(round(duration_s / self.model.opt.timestep))
        if steps < 1 or abs(steps * self.model.opt.timestep - duration_s) > 1e-9:
            raise ValueError("duration_s must be an integer multiple of MuJoCo timestep (0.001s)")
        for _ in range(steps):
            dt = self.model.opt.timestep
            delta = np.clip(self.target_rad - self.reference_rad,
                            -self.description.max_velocity * dt, self.description.max_velocity * dt)
            self.reference_rad += delta
            q = self.data.qpos[self.q_addresses]
            velocity = self.data.qvel[self.v_addresses]
            # PD torque, capped well below the copied URDF's effort limit (10 Nm).
            self.data.ctrl[:] = np.clip(2.0 * (self.reference_rad - q) - 0.08 * velocity, -0.4, 0.4)
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def contact_forces(self):
        forces = np.zeros(5)
        wrench = np.zeros(6)
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            mujoco.mj_contactForce(self.model, self.data, i, wrench)
            for geom in (contact.geom1, contact.geom2):
                finger = self._body_fingers.get(int(self.model.geom_bodyid[geom]))
                if finger is not None:
                    forces[finger] += max(0, wrench[0])
        return forces

    def read_state(self):
        self._ensure_open()
        joint_positions = {n: float(self.data.qpos[self.model.jnt_qposadr[self.model.joint(n).id]])
                           for n in self.description.joints}
        joint_velocities = {n: float(self.data.qvel[self.model.jnt_dofadr[self.model.joint(n).id]])
                            for n in self.description.joints}
        return HandState(float(self.data.time), self.data.qpos[self.q_addresses].copy(),
                         self.data.qvel[self.v_addresses].copy(), joint_positions, joint_velocities,
                         {f: self.data.site(f"{f}_tip").xpos.copy() for f in FINGERS},
                         self.contact_forces(), None, "mujoco_qvel", "mujoco_contact_normal_sum",
                         position_source="mujoco_qpos")

    def stop(self):
        self._ensure_open()
        self.target_rad = np.clip(self.data.qpos[self.q_addresses], self.description.lower, self.description.upper)
        self.reference_rad = self.target_rad.copy()

    def close(self):
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
