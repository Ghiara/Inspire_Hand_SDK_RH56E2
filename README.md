# Inspire_Hand_SDK_RH56E2

<p align="center">
  <strong>Author · Yuan Meng</strong><br>
  <a href="https://www.cit.tum.de/en/cit/home/">TUM School of Computation, Information and Technology (CIT)</a><br>
  Technical University of Munich (TUM)
</p>

Inspire RH56E2 灵巧手 Python SDK，支持左右手 MuJoCo 仿真与 Modbus TCP 真机控制，无需 ROS。
提供关节位置、带方向的关节速度、3D 指尖位置与仿真接触力反馈控制，以及状态读取、可视化和 MP4 录像。


**[SDK API 文档](https://ghiara.github.io/Inspire_Hand_SDK_RH56E2/)**



## 安装

推荐 Python 3.10，以下命令在 SDK 仓库根目录执行：

```bash
git clone https://github.com/Ghiara/Inspire_Hand_SDK_RH56E2.git
cd Inspire_Hand_SDK_RH56E2
conda create -n inspire-rh56e2 python=3.10
conda activate inspire-rh56e2
export PYTHONNOUSERSITE=1
export PYTHONPATH=
python -m pip install -r requirements.txt
```

每次新开终端后激活环境并设置上述两个环境变量，避免其他 Python 或 ROS 环境的包干扰。
模型与网格已随仓库提供。图形窗口需要桌面显示环境；无窗口仿真可使用 `--headless`。

## SDK demo CLI

以下命令均在仓库根目录、已激活的环境中执行。

### 1. 纯可视化模型（左手 / 右手）

```bash
python main.py visualize --side left
python main.py visualize --side right
```

窗口默认持续打开，关闭窗口或按 `Ctrl+C` 退出；可用 `--duration 30` 指定仿真时长。
此入口仅查看模型，不运行控制 demo，也不连接真机。

### 2. 纯仿真运行四种 demo（左手 / 右手）

```bash
# 左手：关节位置、关节速度、接触力反馈、3D 指尖目标
python main.py demo position  --side left --duration 6
python main.py demo velocity  --side left --duration 6
python main.py demo force     --side left --duration 10
python main.py demo cartesian --side left --duration 6

# 右手
python main.py demo position  --side right --duration 6
python main.py demo velocity  --side right --duration 6
python main.py demo force     --side right --duration 10
python main.py demo cartesian --side right --duration 6
```

默认显示 MuJoCo 窗口，省略 `--duration` 时运行 10 秒。位置 demo 每 3 秒切换目标，速度 demo
每 2 秒反向；力控 demo 自动加入接触球，五指目标各为 1 N；指尖 demo 使用 FK 生成可达目标并通过 IK 求解。
仿真以 1 ms 步进、每 10 ms 更新命令，`--duration` 须为正数且是 0.01 秒的整数倍。

```bash
# 无窗口仿真
python main.py demo position --side left --headless --duration 3
# 无窗口录像；省略 --record 后的路径时自动写入 logs/
python main.py demo force --side right --headless --duration 10 --record
python main.py demo cartesian --side left --headless --duration 6 --record logs/cartesian.mp4
```

录像默认 640×480、25 fps，可用 `--fps` 修改帧率。无窗口录像默认使用 OSMesa，需要系统提供对应运行库。

### 3. 真机运行三种 demo（左手）

电脑网口须能访问灵巧手所在网段。下面使用当前地址 `192.168.11.210`、端口 `6000`、TCP Unit ID `255`，
请按设备配置修改。先进行只读连接检查：

```bash
python main.py check --real --side left --host 192.168.11.210 --port 6000 --device-id 255
```

运行关节位置、关节速度与 3D 指尖目标 demo：

```bash
python main.py demo-real position  --real --side left --host 192.168.11.210 --port 6000 --device-id 255 --duration 6
python main.py demo-real velocity  --real --side left --host 192.168.11.210 --port 6000 --device-id 255 --duration 6
python main.py demo-real cartesian --real --side left --host 192.168.11.210 --port 6000 --device-id 255 --duration 6
```

三个 demo 均先全手张开，再运行所选通道的小幅动作，最后执行设备语义的回零：
五路 `ANGLE_SET=0`，拇指旋转保持 `1000`。**张开与回零会运动全手**，`--channels` 仅限制中间 demo 阶段。
窗口显示真机反馈姿态，真机运动不支持 `--headless`。省略 `--real` 时运行同一流程的仿真预览。
真机尚未提供经标定的 Newton 力反馈接口，因此这里提供三种真机 demo。

| 参数 | 默认值与含义 |
| --- | --- |
| `--channels` | `index`；可多选 `pinky ring middle index thumb_bend thumb_rotation` |
| `--duration / --period` | 中间动作阶段 6 秒墙钟时间 / 0.05 秒周期；周期至少 0.02 秒且为 1 ms 的整数倍 |
| `--amplitude / --rate` | 相对张开后反馈的最大偏移 0.12 rad / 参考速度 0.05 rad/s；上限分别为 0.3 / 0.2 |
| `--speed-units / --force-units` | 均为 200，六轴设备速度上限 / 原生力保护阈值，范围均为 1..1000 |
| `--pose-rate / --pose-timeout` | 张开与回零的参考速度 0.2 rad/s / 每阶段超时 30 秒 |
| `--pose-tolerance-units` | 真机六轴原始反馈容差 20，范围 1..50，最终参考发送后需连续 3 次到位 |
| `--timeout` | 每次通信请求超时 1 秒，不自动重试 |
| `--log` | 自动写入 `logs/real_模式_时间戳.jsonl`；可指定新路径，拒绝覆盖已有文件 |

例如，仅在中间动作阶段控制食指和中指：

```bash
python main.py demo-real position --real --side left --host 192.168.11.210 --device-id 255 --channels index middle --amplitude 0.1 --rate 0.04
```

`stop()` 是当前位置保持请求；关闭资源不等于急停。真机位置是由设备角度线性映射得到的模型估计，
速度控制由软件周期积分位置实现。

### 4. 灵巧手 IP 读取、修改与保存

`network_config.py` 是独立的网络配置入口，直接连接真机，无需 `--real`。
默认只读设备 IP，并显示本机网口与访问设备的路由；需要 Linux `iproute2` 来显示本机网络信息。
`--host` 始终填写本次连接使用的当前地址。

```bash
# 读取当前配置
python network_config.py --host 192.168.11.210 --port 6000 --device-id 255

# 修改 IP 并回读，不保存到 Flash
python network_config.py --host 192.168.11.210 --set-ip 192.168.1.210

# 修改 IP、回读并请求保存到 Flash
python network_config.py --host 192.168.11.210 --set-ip 192.168.1.210 --save

# 手动重新上电，并让电脑网口能访问新网段后，验证新 IP
python network_config.py --host 192.168.1.210
```

上面的两种写入方式按需选用。默认以只读方式识别 IP 寄存器编码；若已存在待生效配置而导致识别失败，
按已核实的固件编码指定 `--ip-layout packed-le`、`packed-be` 或 `octet`。
例如，确认设备为 `packed-le` 后，可保存已配置的目标 IP：

```bash
python network_config.py --host 192.168.11.210 --set-ip 192.168.1.210 --ip-layout packed-le --save
```

`--save` 必须与 `--set-ip` 同时使用；设备已配置为目标 IP 时会跳过重写，继续回读并保存。
保存指令作用于设备当前可保存的全局参数。脚本不修改电脑网口，也不自动重启；新地址生效与断电持久化需重新上电后确认。

更多参数：

```bash
python main.py --help
python main.py demo --help
python main.py demo-real --help
python network_config.py --help
```

## Python SDK

```python
from inspire_sdk import InspireHand

with InspireHand.simulation(side="left") as hand:
    hand.position.set_finger("index", 0.5)  # rad
    hand.advance(2.0)
    print(hand.read_state().to_dict())

    hand.stop()
    hand.velocity.set_targets([0, 0, 0, 0.1, 0, 0])  # rad/s
    for _ in range(100):
        hand.velocity.step(0.01)
    hand.stop()

    tips = hand.backend.kinematics.forward(hand.backend.description.upper * 0.4)
    result = hand.cartesian.set_targets(tips, tolerance_m=0.001)
    hand.advance(2.0)
    print(result.errors_m)
```

六轴顺序为 `[pinky, ring, middle, index, thumb_bend, thumb_rotation]`，五指顺序为
`[pinky, ring, middle, index, thumb]`。位置使用 rad，速度使用 rad/s，指尖坐标使用 m、位于 `hand_base` 坐标系。
模型 `0 rad` 表示展开，设备 `ANGLE_SET=1000` 表示展开。

位置与指尖目标发布后调用 `advance()`；速度与仿真力反馈需周期调用各自的 `step()`，其中已包含后端推进。
切换控制模式时先结束原循环并调用 `hand.stop()`。四指各一个主动弯曲轴，拇指两个主动轴，其余关节按 mimic 关系联动；
IK 只能求解可达曲线或曲面上的目标，默认拒绝不可达目标，可显式用 `project=True` 接受投影。

完整的方法签名、参数、默认值、返回值、异常、状态字段和后端差异见
**[SDK API 文档](https://ghiara.github.io/Inspire_Hand_SDK_RH56E2/)**。

## 目录

```text
inspire_sdk/      统一接口、controller、状态、模型、FK/IK 与仿真/真机后端
example/          四种仿真 demo、三种真机 demo 与连接检查
visualization/    模型生成、MuJoCo 窗口与录像
assets/           左右手 URDF、MJCF 与网格
docs/             静态 API 文档与 GitHub Pages 使用说明
tests/            控制、通信、CLI 与离线真机流程测试
third_party/      参考库信息与许可证
main.py           SDK demo CLI
network_config.py 独立 IP 读取、修改与保存 CLI
logs/             运行日志与录像，按需生成，不纳入 Git
```
