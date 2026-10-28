"""Exercise real pymodbus MBAP/PDU encoding against a local TCP device emulator."""
from contextlib import contextmanager
import socket
import struct
import threading
import pytest
from inspire_sdk.modbus_tcp import ModbusTcpBackend, ModbusError


def receive_exact(connection, count):
    result = b""
    while len(result) < count:
        part = connection.recv(count - len(result))
        if not part:
            raise EOFError
        result += part
    return result


@contextmanager
def local_hand():
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    server.settimeout(5)
    records = []
    failures = []

    def serve():
        try:
            connection, _ = server.accept()
            with connection:
                connection.settimeout(5)
                angles = [1000]*6
                while True:
                    try:
                        transaction, protocol, length, unit = struct.unpack(">HHHB", receive_exact(connection, 7))
                        pdu = receive_exact(connection, length - 1)
                    except EOFError:
                        break
                    assert protocol == 0
                    function, address, count = struct.unpack(">BHH", pdu[:5])
                    if unit != 1:
                        reply = bytes([function | 0x80, 0x0B])
                    elif function == 3:
                        values = angles if address == 1546 else [0]*count
                        reply = bytes([3, count * 2]) + struct.pack(">" + "H"*count, *values)
                    elif function == 16:
                        assert pdu[5] == count * 2
                        values = list(struct.unpack(">" + "H"*count, pdu[6:]))
                        records.append((address, values, unit))
                        if address == 1486:
                            angles = values
                        reply = struct.pack(">BHH", 16, address, count)
                    else:
                        raise AssertionError(f"Unexpected function {function}")
                    connection.sendall(struct.pack(">HHHB", transaction, 0, len(reply) + 1, unit) + reply)
        except Exception as exc:
            failures.append(exc)
        finally:
            server.close()

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    try:
        yield server.getsockname()[1], records
    finally:
        worker.join(timeout=6)
        assert not worker.is_alive(), "Local device server did not finish"
        if failures:
            raise failures[0]


def test_real_tcp_read_write_and_register_order():
    with local_hand() as (port, records):
        with ModbusTcpBackend("127.0.0.1", port=port, timeout_s=0.5) as backend:
            assert records == []
            backend.set_positions(backend.description.upper * [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
            state = backend.read_state()
            assert state.positions_rad == pytest.approx(backend.target_rad)
            assert records == [(1522, [150]*6, 1), (1498, [200]*6, 1),
                               (1486, [900, 800, 700, 600, 500, 400], 1)]


def test_real_tcp_exception_response_is_rejected():
    with local_hand() as (port, records):
        with pytest.raises(ModbusError):
            ModbusTcpBackend("127.0.0.1", port=port, device_id=2, timeout_s=0.5)
        assert not records
