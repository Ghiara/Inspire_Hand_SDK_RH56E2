"""Public SDK facade with explicit simulation/hardware factories."""
from .simulation import MujocoBackend
from .joint_pos_control import JointPositionController
from .joint_velo_control import JointVelocityController
from .force_control import ForceController
from .target_pos_control import TargetPositionController


class InspireHand:
    def __init__(self, backend):
        self.backend = backend
        self.position = JointPositionController(backend)
        self.velocity = JointVelocityController(backend)
        self.force = ForceController(backend)
        self.cartesian = TargetPositionController(backend)

    @classmethod
    def simulation(cls, side="right", contact_fixture=False):
        return cls(MujocoBackend(side, contact_fixture))

    @classmethod
    def modbus_tcp(cls, host, **kwargs):
        from .modbus_tcp import ModbusTcpBackend
        return cls(ModbusTcpBackend(host, **kwargs))

    def read_state(self):
        return self.backend.read_state()

    def advance(self, duration_s):
        self.backend.advance(duration_s)

    def stop(self):
        self.velocity.velocities_rad_s[:] = 0
        self.velocity.reference_rad = None
        self.force.reference_rad = None
        self.backend.stop()

    def close(self):
        self.backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
