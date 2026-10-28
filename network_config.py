"""Read RH56E2 network settings; only --set-ip enables a device write.

Standalone Linux utility: no motion SDK imports, network changes or reboot.
The manual lists byte addresses 1700..1703 but does not specify their Modbus
packing. Auto mode verifies the layout against --host using reads only.
"""
import argparse
import ipaddress
import json
import math
import subprocess
import sys
import time


IP_ADDRESS = 1700
SAVE_ADDRESS = 1005
FLASH_SETTLE_S = 2.0
LAYOUTS = ("auto", "packed-le", "packed-be", "octet")


def ipv4(value):
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise argparse.ArgumentTypeError("请输入 IPv4 地址，例如 192.168.11.210") from exc
    if address.is_unspecified or address.is_multicast or address.is_loopback or int(address) == 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("请输入设备可用的单播 IPv4 地址")
    return str(address)


def bounded_int(low, high):
    def parse(value):
        number = int(value)
        if not low <= number <= high:
            raise argparse.ArgumentTypeError(f"必须在 {low}..{high} 之间")
        return number
    return parse


def timeout_value(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("超时必须是有限正数")
    return number


def parser():
    cli = argparse.ArgumentParser(
        description="读取实机 IP、到设备的本机网口/IP；仅 --set-ip 写入新 IP，随后提示重新上电。")
    cli.add_argument("--host", type=ipv4, default="192.168.11.210", help="当前连接地址（默认 %(default)s）；不扫描设备")
    cli.add_argument("--port", type=bounded_int(1, 65535), default=6000)
    cli.add_argument("--device-id", type=bounded_int(1, 255), default=255, help="TCP Unit ID（默认 %(default)s）")
    cli.add_argument("--timeout", type=timeout_value, default=2.0, help="通信超时秒数（默认 %(default)s），不自动重试")
    cli.add_argument("--set-ip", type=ipv4, help="指定时才写设备 IP，例如 192.168.1.210")
    cli.add_argument("--save", action="store_true",
                     help="IP 回读通过后请求保存当前可保存参数到 Flash（全局保存）；必须同时指定 --set-ip")
    cli.add_argument("--network-only", action="store_true", help="只查看电脑网口和路由，不连接设备")
    cli.add_argument("--ip-layout", choices=LAYOUTS, default="auto",
                     help="默认只读识别；手册未明确字节打包。显式指定需已核实固件编码")
    return cli


def ip_command(*arguments):
    result = subprocess.run(["ip", "-j", *arguments], check=True,
                            capture_output=True, text=True, timeout=5)
    return json.loads(result.stdout)


def ipv4_addresses(link):
    return [ipaddress.IPv4Interface(f"{item['local']}/{item['prefixlen']}")
            for item in link.get("addr_info", []) if item.get("family") == "inet"]


def show_local_network(host, new_ip=None):
    """Show all local links separately from the OS-selected route to the hand."""
    links = []
    try:
        links = ip_command("address", "show")
        print("本机网口列表（本机枚举，不代表已连接到灵巧手）:")
        for link in links:
            addresses = ", ".join(str(address) for address in ipv4_addresses(link)) or "无 IPv4 地址"
            print(f"  {link['ifname']}: {addresses}；状态 {link.get('operstate', 'UNKNOWN')}")
        for label, target in (("当前目标", host), ("拟设置目标", new_ip)):
            if target is None:
                continue
            matches = [link["ifname"] for link in links
                       if any(ipaddress.IPv4Address(target) in address.network for address in ipv4_addresses(link))]
            if matches:
                print(f"{label} {target} 的本机直连网段匹配: {', '.join(matches)}（不代表设备已响应）")
            else:
                print(f"{label} {target}: 未发现匹配的本机直连 IPv4 网段。"
                      "若灵巧手直连有线口，需在该网口配置对应网段的空闲地址。")
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print(f"无法枚举本机网口（Linux 需 iproute2）: {exc}", file=sys.stderr)
    try:
        routes = ip_command("route", "get", host)
        if not routes or not routes[0].get("dev"):
            raise ValueError("未找到可用路由")
        route = routes[0]
        interface = route["dev"]
        source = route.get("prefsrc") or route.get("src") or "未知"
        print(f"系统选中访问 {host} 的网口: {interface}；本机源 IP: {source}（尚未验证设备可达）")
        if route.get("gateway"):
            print(f"经网关: {route['gateway']}；这只是路由选择，不能证明设备在此网口。"
                  "若应直连有线口，请检查有线口的目标网段地址和路由。")
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print(f"无法完整读取本机网口信息（Linux 需 iproute2）: {exc}", file=sys.stderr)


def checked(response):
    if response is None or response.isError():
        raise RuntimeError(f"Modbus 返回错误: {response}")
    return response


def read_words(client, device_id, address, count):
    response = checked(client.read_holding_registers(address=address, count=count, device_id=device_id))
    if len(response.registers) != count:
        raise RuntimeError(f"地址 {address} 读取不完整，预期 {count} 个寄存器")
    return list(response.registers)


def write_words(client, device_id, address, values):
    response = checked(client.write_registers(address=address, values=values, device_id=device_id))
    if response.address != address or response.count != len(values):
        raise RuntimeError(f"地址 {address} 写入应答的地址/数量不匹配")


def save_flash(client, device_id, layout):
    # 1005 is SAVE, 1006 is RESET_PARA. On byte-addressed firmware the
    # single Modbus word contains both bytes: SAVE=1, RESET_PARA=0.
    # Do not shift the start address to 1004 (CLEAR_ERROR), or reuse an
    # adjacent command's old value. In octet layout only 1005 is written.
    value = 0x0100 if layout == "packed-be" else 1
    print(f"请求全局 Flash 保存: 地址 {SAVE_ADDRESS}，值 [{value}]；"
          "会保存当前可保存参数，不仅是 IP。", flush=True)
    write_words(client, device_id, SAVE_ADDRESS, [value])
    print(f"保存命令应答通过；预留 {FLASH_SETTLE_S:g} 秒等待，断电持久化仍需重启验证。", flush=True)
    # The manual provides no completion deadline. This is a grace period,
    # not evidence that programming finished. SAVE may self-clear, so a
    # readback of 1 is not a valid completion condition.
    time.sleep(FLASH_SETTLE_S)


def decode_ip(words, layout):
    if layout == "octet":
        if len(words) != 4 or any(not 0 <= word <= 255 for word in words):
            raise ValueError(f"不是四个单字节 IP 字段: {words}")
        octets = words
    else:
        if len(words) != 2:
            raise ValueError("packed IP 必须是两个 16-bit 寄存器")
        order = "little" if layout == "packed-le" else "big"
        octets = b"".join(word.to_bytes(2, order) for word in words)
    return str(ipaddress.IPv4Address(bytes(octets)))


def encode_ip(address, layout):
    octets = ipaddress.IPv4Address(address).packed
    if layout == "octet":
        return list(octets)
    order = "little" if layout == "packed-le" else "big"
    return [int.from_bytes(octets[index:index + 2], order) for index in (0, 2)]


def read_ip(client, device_id, layout):
    return decode_ip(read_words(client, device_id, IP_ADDRESS, 4 if layout == "octet" else 2), layout)


def identify_layout(client, device_id, host, requested):
    if requested == "octet":
        return requested, read_ip(client, device_id, requested)
    # Byte-addressed firmware should expose the second word at byte address
    # 1702. Check this independently before any packed write at 1700.
    words = read_words(client, device_id, IP_ADDRESS, 2)
    second = read_words(client, device_id, IP_ADDRESS + 2, 1)[0]
    packed = {layout: decode_ip(words, layout) for layout in ("packed-le", "packed-be")}
    if requested != "auto":
        if second != words[1]:
            raise RuntimeError("1702 的读取结果与 packed 布局不符，停止；未写入")
        return requested, packed[requested]
    matches = [layout for layout, address in packed.items() if address == host and second == words[1]]
    if len(matches) == 1:
        return matches[0], host
    # A firmware may expose one octet per 16-bit Modbus register instead.
    # These additional operations are reads only, never trial writes.
    octet_address = None
    if all(0 <= word <= 255 for word in (*words, second)):
        last = read_words(client, device_id, IP_ADDRESS + 3, 1)[0]
        try:
            octet_address = decode_ip([*words, second, last], "octet")
        except ValueError:
            pass
        if octet_address == host and not matches:
            return "octet", octet_address
    raise RuntimeError(
        f"无法唯一核实 IP 编码，未写入。1700 原始值={words}，1702={second}；"
        f"候选 packed={packed}，octet={octet_address}，连接地址={host}。"
        "若配置 IP 已与连接地址不同，请根据固件资料指定 --ip-layout 后重新读取。")


def configure(client, args):
    layout, current = identify_layout(client, args.device_id, args.host, args.ip_layout)
    print(f"设备配置 IP: {current}；寄存器编码: {layout}")
    if current != args.host:
        print(f"连接仍使用 {args.host}；配置 IP 与运行地址不同，可能存在待生效修改。")
    if args.set_ip is None:
        print("只读完成，未写入任何设备参数。")
        return
    if args.set_ip == current and not args.save:
        print("指定 IP 与配置值相同，未重复写入。")
        if current != args.host:
            print("请手动重新上电，并通过配置 IP 再次读取确认。")
        return
    if args.set_ip == current:
        print("指定 IP 与配置值相同，跳过 IP 重写；仍按 --save 请求保存。")
    else:
        values = encode_ip(args.set_ip, layout)
        print(f"写入设备 IP: {current} -> {args.set_ip}；地址 {IP_ADDRESS}，值 {values}", flush=True)
        try:
            write_words(client, args.device_id, IP_ADDRESS, values)
            actual = read_ip(client, args.device_id, layout)
            if actual != args.set_ip:
                raise RuntimeError(f"IP 回读不匹配: 期望 {args.set_ip}，实际 {actual}")
        except (Exception, KeyboardInterrupt):
            print("写入已尝试，结果尚未确认；未重试、未自动回滚，也未发送 Flash 保存。"
                  "请先用旧连接地址回读核对配置。", file=sys.stderr)
            print(f"python network_config.py --host {args.host} --port {args.port} "
                  f"--device-id {args.device_id} --timeout {args.timeout:g} --ip-layout {layout}", file=sys.stderr)
            raise
        print(f"写入应答和配置回读均通过: {actual}")
    if args.save:
        try:
            # Verify even when the requested IP was already present, before
            # issuing any global save, then check it again after the request.
            if read_ip(client, args.device_id, layout) != args.set_ip:
                raise RuntimeError("保存前 IP 回读不符，未发送保存命令")
            save_flash(client, args.device_id, layout)
            if read_ip(client, args.device_id, layout) != args.set_ip:
                raise RuntimeError("保存后 IP 回读不符")
        except (Exception, KeyboardInterrupt):
            print("Flash 保存流程未确认完成，未重试或回滚；当前 IP 配置可能已改变。"
                  "请保持供电并检查完整报错。", file=sys.stderr)
            raise
    else:
        print("未发送 Flash 保存指令；本次仅修改当前配置，可能在断电后恢复旧 IP。")
    print("以上仅确认当前配置回读，尚未验证断电保存和新地址可达。")
    print("请手动将灵巧手断电后重新上电，再确认新 IP 是否生效；本脚本不会自动重启。")
    print("电脑网口需能访问新网段；本脚本不修改本机网络。")
    print("重新上电后请自行运行以下只读命令，确认实际地址及断电持久化:")
    print(f"python network_config.py --host {args.set_ip} --port {args.port} "
          f"--device-id {args.device_id} --timeout {args.timeout:g} --ip-layout {layout}")


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if args.network_only and args.set_ip is not None:
        cli.error("--network-only 不可与 --set-ip 同时使用")
    if args.save and args.set_ip is None:
        cli.error("--save 必须与 --set-ip 同时使用；默认入口保持只读")
    print(f"当前连接目标: {args.host}:{args.port}；Unit ID: {args.device_id}")
    show_local_network(args.host, args.set_ip)
    if args.network_only:
        print("本机网络只读检查完成，未连接设备。")
        return 0
    try:
        from pymodbus.client import ModbusTcpClient
    except ImportError:
        print("缺少 pymodbus，请在 SDK 已安装依赖的环境中运行（requirements.txt: pymodbus==3.11.3）。", file=sys.stderr)
        return 1
    try:
        client = ModbusTcpClient(args.host, port=args.port, timeout=args.timeout, retries=0)
        try:
            if not client.connect():
                raise RuntimeError(f"无法连接 {args.host}:{args.port}；请检查设备供电、当前 IP 和网口路由")
            configure(client, args)
        finally:
            client.close()
    except KeyboardInterrupt:
        print("已中断。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"失败: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
