"""Public SDK facade with explicit simulation/hardware factories."""
from .simulation import MujocoBackend
from .general_control import GeneralController
from .joint_pos_control import JointPositionController
from .joint_velo_control import JointVelocityController
from .force_control import ForceController
from .target_pos_control import TargetPositionController


class InspireHand:
    def __init__(self, backend):
        self.backend = backend
        self.general = GeneralController(backend)
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

    def _reset_controllers(self):
        for controller in (self.position, self.velocity, self.force, self.cartesian):
            controller._reset_control_state()

    def open_hand(self, **kwargs):
        """Move to six native 1000 values, waiting for measured arrival."""
        self._reset_controllers()
        return self.general.open_hand(**kwargs)

    def return_to_zero(self, **kwargs):
        """Bend five channels to native zero, keeping thumb_rotation open."""
        self._reset_controllers()
        return self.general.return_to_zero(**kwargs)

    def stop(self):
        self._reset_controllers()
        self.general.stop()

    def close(self):
        self.backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
