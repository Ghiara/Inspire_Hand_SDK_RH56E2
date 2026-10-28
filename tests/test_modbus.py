from types import SimpleNamespace
import numpy as np
import pytest
from inspire_sdk.modbus_tcp import ModbusTcpBackend, ModbusError, RegisterMap
from inspire_sdk import ForceController


class FakeClient:
    def __init__(self):
        self.reads = []
        self.writes = []
        self.connected = True
        self.closed = False
        self.error_response = False
        self.short_read = False
        self.fault = 0
        self.write_exception = False
        self.ack_mismatch = False

    def connect(self):
        return self.connected

    def read_holding_registers(self, *, address, count, device_id):
        self.reads.append((address, count, device_id))
        values = [1000]*6 if address == 1546 else [0]*count
        if address == 1582:
            values = [65535, 2, 3, 4, 5, 6]
        if address == 1606:
            values[0] = self.fault
        if self.short_read:
            values = values[:-1]
        return SimpleNamespace(registers=values, isError=lambda: self.error_response)

    def write_registers(self, *, address, values, device_id):
        self.writes.append((address, values, device_id))
        if self.write_exception:
            raise TimeoutError("test timeout")
        return SimpleNamespace(address=address + int(self.ack_mismatch), count=len(values),
                               isError=lambda: self.error_response)

    def close(self):
        self.closed = True


@pytest.fixture
def hardware():
    client = FakeClient()
    with ModbusTcpBackend("test", client=client) as backend:
        yield backend, client


def test_connection_read_only_and_default_motion_limits(hardware):
    backend, client = hardware
    assert not client.writes
    backend.set_positions(backend.description.upper * 0.25)
    assert client.writes == [(1522, [150]*6, 1), (1498, [200]*6, 1), (1486, [750]*6, 1)]
    assert (1606, 3, 1) in client.reads


def test_configured_limits_not_overwritten(hardware):
    backend, client = hardware
    backend.set_speed_limits([100]*6)
    ForceController(backend).set_native_limits([300]*6)
    backend.set_positions(backend.description.upper * 0.5)
    assert client.writes == [(1522, [100]*6, 1), (1498, [300]*6, 1), (1486, [500]*6, 1)]


@pytest.mark.parametrize("values", [[np.nan]*6, [-1]*6, [1]*5, [1e5]*6])
def test_invalid_hardware_command_has_no_writes(hardware, values):
    backend, client = hardware
    for method in (backend.set_positions, backend.set_speed_limits, backend.set_force_limits):
        with pytest.raises(ValueError):
            method(values)
    assert not client.writes


def test_state_units_and_estimated_velocity(hardware):
    backend, client = hardware
    first = backend.read_state()
    assert first.velocity_source == "unavailable_first_sample"
    assert first.contact_forces_n is None
    assert first.raw_forces[0] == -1
    second = backend.read_state()
    assert second.velocity_source == "finite_difference_of_angle_feedback"
    np.testing.assert_array_equal(second.velocities_rad_s, 0)
    with pytest.raises(NotImplementedError):
        ForceController(backend).set_targets([1]*5)


@pytest.mark.parametrize("failure", ["error_response", "short_read"])
def test_read_error_never_fabricates_state(hardware, failure):
    backend, client = hardware
    setattr(client, failure, True)
    with pytest.raises(ModbusError):
        backend.read_state()


def test_fault_blocks_all_motion(hardware):
    backend, client = hardware
    client.fault = 1
    with pytest.raises(ModbusError, match="reports errors"):
        backend.set_positions([0.1]*6)
    assert not client.writes


@pytest.mark.parametrize("failure", ["write_exception", "error_response", "ack_mismatch"])
def test_write_errors_not_success(hardware, failure):
    backend, client = hardware
    setattr(client, failure, True)
    with pytest.raises(ModbusError):
        backend.set_force_limits([100]*6)


def test_failed_connect_and_initial_read_close_transport():
    client = FakeClient()
    client.connected = False
    with pytest.raises(ModbusError):
        ModbusTcpBackend("test", client=client)
    assert client.closed
    client = FakeClient()
    client.short_read = True
    with pytest.raises(ModbusError):
        ModbusTcpBackend("test", client=client)
    assert client.closed


def test_side_device_id_and_stop():
    client = FakeClient()
    with ModbusTcpBackend("test", side="left", client=client) as backend:
        backend.stop()
    assert client.reads[0] == (1546, 6, 2)
    assert client.writes == [(1486, [1000]*6, 2)]
    assert client.closed
