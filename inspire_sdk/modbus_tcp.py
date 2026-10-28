"""Modbus TCP transport adapted from inspire_demos' register map.

Six 16-bit channels are transferred in one FC16/FC03 operation, as in
inspire_demos/inspire_modbus.py. Addresses are device protocol addresses;
never divide by two or add a 40001 offset. Verify this profile on real hardware.
"""
from dataclasses import dataclass
import time
import numpy as np
from pymodbus.client import ModbusTcpClient
from .model import HandModel, vector, positive
from .kinematics import Kinematics
from .state import HandState


class ModbusError(RuntimeError):
    pass


@dataclass(frozen=True)
class RegisterMap:
    angle_set: int = 1486
    force_set: int = 1498
    speed_set: int = 1522
    angle_actual: int = 1546
    force_actual: int = 1582
    error: int = 1606


class ModbusTcpBackend:
    def __init__(self, host, port=6000, device_id=None, side="right", timeout_s=1.0,
                 registers=None, client=None):
        if not host:
            raise ValueError("An explicit hardware host is required")
        if not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be 1..65535")
        self.device_id = (1 if side == "right" else 2) if device_id is None else device_id
        if not isinstance(self.device_id, int) or not 1 <= self.device_id <= 247:
            raise ValueError("device_id must be 1..247")
        self.description = HandModel(side)
        self.kinematics = Kinematics(self.description)
        self.registers = registers or RegisterMap()
        self.client = client if client is not None else ModbusTcpClient(
            host, port=port, timeout=positive(timeout_s, "timeout_s"), retries=0)
        self.closed = False
        self._last_sample = None
        self._speed_initialized = False
        self._force_initialized = False
        try:
            if not self.client.connect():
                raise ModbusError(f"Cannot connect to {host}:{port}")
            self.target_rad = self.description.from_registers(self._read(self.registers.angle_actual, 6))
        except Exception:
            self.close()
            raise

    def _check(self, response):
        if response is None or response.isError():
            raise ModbusError(f"Modbus error response: {response}")
        return response

    def _read(self, address, count):
        if self.closed:
            raise ModbusError("Connection is closed")
        try:
            response = self._check(self.client.read_holding_registers(
                address=address, count=count, device_id=self.device_id))
            if len(response.registers) != count:
                raise ModbusError(f"Short register read at {address}: expected {count}")
            return response.registers
        except ModbusError:
            raise
        except Exception as exc:
            raise ModbusError(f"Read failed at {address}: {exc}") from exc

    def _write(self, address, values):
        if self.closed:
            raise ModbusError("Connection is closed")
        try:
            response = self._check(self.client.write_registers(
                address=address, values=[int(v) for v in values], device_id=self.device_id))
            if response.address != address or response.count != len(values):
                raise ModbusError("Mismatched FC16 write acknowledgement")
        except ModbusError:
            raise
        except Exception as exc:
            raise ModbusError(f"Write failed at {address}: {exc}") from exc

    def read_errors(self):
        packed = self._read(self.registers.error, 3)
        return np.array([byte for word in packed for byte in (word & 255, word >> 8)], dtype=int)

    def _check_faults(self):
        errors = self.read_errors()
        if np.any(errors):
            raise ModbusError(f"Hand reports errors {errors.tolist()}; motion rejected")

    def set_positions(self, positions_rad):
        positions = self.description.positions(positions_rad)
        values = self.description.to_registers(positions)
        self._check_faults()
        # Conservative native limits on first movement; connection is read-only.
        if not self._speed_initialized:
            self.set_speed_limits(np.full(6, 150))
        if not self._force_initialized:
            self.set_force_limits(np.full(6, 200))
        self._write(self.registers.angle_set, values)
        self.target_rad = positions.copy()

    def set_speed_limits(self, speed_units):
        values = vector(speed_units, name="speed_units")
        if np.any(values < 1) or np.any(values > 1000):
            raise ValueError("speed_units must be 1..1000; zero semantics are firmware dependent")
        self._write(self.registers.speed_set, np.rint(values))
        self._speed_initialized = True

    def set_force_limits(self, force_units):
        values = vector(force_units, name="force_units")
        if np.any(values < 0) or np.any(values > 1000):
            raise ValueError("force_units must be 0..1000")
        self._write(self.registers.force_set, np.rint(values))
        self._force_initialized = True

    def read_state(self):
        positions = self.description.from_registers(self._read(self.registers.angle_actual, 6))
        raw = np.asarray(self._read(self.registers.force_actual, 6), dtype=np.int32)
        raw = np.where(raw >= 32768, raw - 65536, raw)
        timestamp = time.monotonic()
        velocities = np.zeros(6)
        source = "unavailable_first_sample"
        if self._last_sample is not None:
            previous_time, previous_positions = self._last_sample
            elapsed = timestamp - previous_time
            if elapsed > 0:
                velocities = (positions - previous_positions) / elapsed
                source = "finite_difference_of_angle_feedback"
        self._last_sample = timestamp, positions.copy()
        joint_positions = self.description.expand(positions)
        joint_velocities = {name: 0.0 for name in joint_positions}
        from .model import ACTIVE_JOINTS
        joint_velocities.update(zip(ACTIVE_JOINTS, velocities))
        for name, (parent, multiplier, _) in self.description.mimics.items():
            joint_velocities[name] = joint_velocities[parent] * multiplier
        return HandState(timestamp, positions, velocities, joint_positions, joint_velocities,
                         self.kinematics.forward(positions), None, raw, source, "native_force_register_uncalibrated",
                         self.read_errors())

    def advance(self, duration_s):
        if self.closed:
            raise ModbusError("Connection is closed")
        time.sleep(positive(duration_s))

    def stop(self):
        # A position hold request, not an emergency stop or a device watchdog.
        self._check_faults()
        actual = self._read(self.registers.angle_actual, 6)
        positions = self.description.from_registers(actual)
        self._write(self.registers.angle_set, actual)
        self.target_rad = positions

    def close(self):
        if not self.closed:
            self.client.close()
            self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
