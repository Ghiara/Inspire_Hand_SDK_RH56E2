"""Offline checks only; these clients never open a socket."""
from types import SimpleNamespace

import pytest

import network_config as network


class Reply:
    def __init__(self, **fields):
        self.__dict__.update(fields)

    def isError(self):
        return False


class MemoryHand:
    def __init__(self, layout, address="192.168.11.210"):
        self.layout = layout
        self.words = network.encode_ip(address, layout)
        self.reads = []
        self.writes = []
        self.bad_ack = False
        self.save_bad_ack = False
        self.ignore_write = False

    def read_holding_registers(self, *, address, count, device_id):
        assert device_id == 255
        self.reads.append((address, count))
        offset = address - network.IP_ADDRESS
        if self.layout != "octet":
            assert offset % 2 == 0
            offset //= 2
        return Reply(registers=self.words[offset:offset + count])

    def write_registers(self, *, address, values, device_id):
        assert device_id == 255
        self.writes.append((address, list(values)))
        if address == network.SAVE_ADDRESS:
            return Reply(address=address, count=999 if self.save_bad_ack else len(values))
        if not self.ignore_write:
            self.words = list(values)
        return Reply(address=address, count=999 if self.bad_ack else len(values))


def options(**overrides):
    return SimpleNamespace(**{"host": "192.168.11.210", "device_id": 255,
                              "ip_layout": "auto", "set_ip": None, "port": 6000,
                              "timeout": 2.0, "save": False, **overrides})


@pytest.mark.parametrize("layout", network.LAYOUTS[1:])
def test_default_reads_actual_ip_without_any_writes(layout, capsys):
    hand = MemoryHand(layout)
    network.configure(hand, options())
    assert hand.writes == []
    assert "设备配置 IP: 192.168.11.210" in capsys.readouterr().out


@pytest.mark.parametrize("layout, expected", [
    ("packed-le", [0xA8C0, 0xD201]),
    ("packed-be", [0xC0A8, 0x01D2]),
    ("octet", [192, 168, 1, 210]),
])
def test_write_only_ip_fields_and_verify_readback(layout, expected, capsys):
    hand = MemoryHand(layout)
    network.configure(hand, options(set_ip="192.168.1.210"))
    assert hand.writes == [(1700, expected)]
    assert hand.reads[-1] == (1700, len(expected))
    assert "断电后重新上电" in capsys.readouterr().out


def test_config_different_from_connection_requires_known_layout():
    hand = MemoryHand("packed-le", "192.168.1.210")
    with pytest.raises(RuntimeError, match="无法唯一核实"):
        network.configure(hand, options(set_ip="192.168.1.211"))
    assert hand.writes == []


def test_pending_config_can_be_read_with_explicit_layout(capsys):
    hand = MemoryHand("packed-le", "192.168.1.210")
    network.configure(hand, options(ip_layout="packed-le"))
    assert "设备配置 IP: 192.168.1.210" in capsys.readouterr().out
    assert hand.writes == []


@pytest.mark.parametrize("failure", ["bad_ack", "ignore_write"])
def test_uncertain_write_stops_without_retry_or_flash_save(failure, capsys):
    hand = MemoryHand("packed-le")
    setattr(hand, failure, True)
    with pytest.raises(RuntimeError):
        network.configure(hand, options(set_ip="192.168.1.210"))
    assert len(hand.writes) == 1
    assert hand.writes[0][0] == 1700
    assert "结果尚未确认" in capsys.readouterr().err


def test_same_ip_does_not_write():
    hand = MemoryHand("packed-le")
    network.configure(hand, options(set_ip="192.168.11.210"))
    assert hand.writes == []


@pytest.fixture
def flash_waits(monkeypatch):
    waits = []
    monkeypatch.setattr(network.time, "sleep", waits.append)
    return waits


@pytest.mark.parametrize("layout, expected_ip, expected_save", [
    ("packed-le", [0xA8C0, 0xD201], 0x0001),
    ("packed-be", [0xC0A8, 0x01D2], 0x0100),
    ("octet", [192, 168, 1, 210], 0x0001),
])
def test_save_follows_ip_verification_and_avoids_factory_reset(layout, expected_ip, expected_save, flash_waits):
    hand = MemoryHand(layout)
    reads_at_save = []
    write_registers = hand.write_registers

    def record_write(**kwargs):
        if kwargs["address"] == 1005:
            reads_at_save.extend(hand.reads)
        return write_registers(**kwargs)

    hand.write_registers = record_write
    network.configure(hand, options(set_ip="192.168.1.210", save=True))
    assert hand.writes == [(1700, expected_ip), (1005, [expected_save])]
    assert reads_at_save[-2:] == [(1700, len(expected_ip))] * 2
    assert hand.reads[-1] == (1700, len(expected_ip))
    if layout != "octet":
        byte_order = "little" if layout == "packed-le" else "big"
        assert expected_save.to_bytes(2, byte_order) == bytes([1, 0])
    assert flash_waits == [2.0]


def test_same_pending_ip_still_sends_requested_save(flash_waits):
    hand = MemoryHand("packed-le", "192.168.1.210")
    network.configure(hand, options(ip_layout="packed-le", set_ip="192.168.1.210", save=True))
    assert hand.writes == [(1005, [1])]


def test_ip_readback_failure_prevents_global_save(flash_waits):
    hand = MemoryHand("packed-le")
    hand.ignore_write = True
    with pytest.raises(RuntimeError, match="IP 回读不匹配"):
        network.configure(hand, options(set_ip="192.168.1.210", save=True))
    assert hand.writes == [(1700, [0xA8C0, 0xD201])]
    assert flash_waits == []


def test_flash_failure_is_not_retried_or_reported_as_persistent(flash_waits, capsys):
    hand = MemoryHand("packed-le")
    hand.save_bad_ack = True
    with pytest.raises(RuntimeError, match="1005 写入应答"):
        network.configure(hand, options(set_ip="192.168.1.210", save=True))
    assert hand.writes == [(1700, [0xA8C0, 0xD201]), (1005, [1])]
    assert flash_waits == []
    assert "Flash 保存流程未确认完成" in capsys.readouterr().err


def test_save_requires_explicit_ip_before_any_connection(monkeypatch):
    def unexpected_network_access(*args):
        pytest.fail("invalid arguments must not inspect or connect to the network")

    monkeypatch.setattr(network, "show_local_network", unexpected_network_access)
    with pytest.raises(SystemExit) as error:
        network.main(["--save"])
    assert error.value.code == 2


def test_route_uses_device_interface_and_lists_aliases(monkeypatch, capsys):
    def ip_command(*args):
        if args == ("route", "get", "192.168.11.210"):
            return [{"dev": "enp3s0", "prefsrc": "192.168.11.212"}]
        assert args == ("address", "show")
        return [{"ifname": "enp3s0", "addr_info": [{"family": "inet", "local": "192.168.11.212", "prefixlen": 24},
                               {"family": "inet", "local": "192.168.1.212", "prefixlen": 24}]}]

    monkeypatch.setattr(network, "ip_command", ip_command)
    network.show_local_network("192.168.11.210")
    output = capsys.readouterr().out
    assert "enp3s0" in output and "本机源 IP: 192.168.11.212" in output
    assert "192.168.11.212/24, 192.168.1.212/24" in output


def test_wireless_default_route_does_not_hide_wired_interface(monkeypatch, capsys):
    def ip_command(*args):
        if args == ("address", "show"):
            return [{"ifname": "enxa0cec8fd2d3c", "addr_info": [
                        {"family": "inet", "local": "192.168.11.212", "prefixlen": 24}]},
                    {"ifname": "wlp59s0", "addr_info": [
                        {"family": "inet", "local": "131.159.203.137", "prefixlen": 19}]}]
        assert args == ("route", "get", "192.168.1.210")
        return [{"dev": "wlp59s0", "prefsrc": "131.159.203.137", "gateway": "131.159.223.254"}]

    monkeypatch.setattr(network, "ip_command", ip_command)
    network.show_local_network("192.168.1.210")
    output = capsys.readouterr().out
    assert "enxa0cec8fd2d3c: 192.168.11.212/24" in output
    assert "未发现匹配的本机直连 IPv4 网段" in output
    assert "系统选中访问 192.168.1.210 的网口: wlp59s0" in output
    assert "不能证明设备在此网口" in output


def test_network_only_never_connects_to_device(monkeypatch, capsys):
    monkeypatch.setattr(network, "show_local_network", lambda *args: None)
    assert network.main(["--network-only"]) == 0
    assert "未连接设备" in capsys.readouterr().out
