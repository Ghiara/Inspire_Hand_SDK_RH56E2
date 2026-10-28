# Inspire_Hand_SDK_RH56E2

Inspire RH56E2 控制 SDK。

- [ ] 与已有机械臂平台（Franka，KUKA iiwa）结合（仿真+实机）
- [ ] 灵巧手真机测试（TCP串口）
- [x] 2026/10/28：灵巧手仿真mujoco支持4种控制模式（关节位置，关节速度，指尖空间笛卡尔目标，接触力反馈）

默认示例运行 MuJoCo，真机通信后端采用
**Modbus TCP**。左右手均有本地模型，运行时不需要 ROS 或同级参考仓库。

已实现位置、带方向的速度、接触力反馈、3D 指尖位置控制，以及状态读取、可视化、MP4 录像。
仿真使用力矩驱动和 `mj_step`，命令不会直接修改仿真关节位置。

## 安装

使用 **conda** 管理环境，推荐 Python 3.10。首次安装时创建独立的 `inspire-rh56e2` 环境：

```bash
git clone https://github.com/Ghiara/Inspire_Hand_SDK_RH56E2.git
cd Inspire_Hand_SDK_RH56E2

conda create -n inspire-rh56e2 python=3.10 

conda activate inspire-rh56e2

pip install -r requirements.txt
```

## SDK demo CLI

以下命令在本目录、已激活的conda环境中运行。

```bash
# 查看模型：默认持续打开，关闭窗口或按 Ctrl+C 退出
PYTHONPATH= python main.py visualize
PYTHONPATH= python visualization/mujoco_viewer.py --side left

# 需要定时退出时，显式设置仿真时长（秒）
PYTHONPATH= python main.py visualize --duration 30

# 四种控制 demo，默认打开窗口
PYTHONPATH= python main.py demo position --duration 6
PYTHONPATH= python main.py demo velocity --duration 6
PYTHONPATH= python main.py demo force --duration 10
PYTHONPATH= python main.py demo cartesian --duration 6

# 无窗口运行和读取状态
PYTHONPATH= python main.py demo position --headless --duration 3
PYTHONPATH= python main.py state --side right

# 无窗口录制，省略输出路径时自动写入 logs/模式_时间戳.mp4
PYTHONPATH= python main.py demo force --headless --duration 10 --record
PYTHONPATH= python main.py demo cartesian --headless --duration 6 --record logs/cartesian.mp4
PYTHONPATH= python visualization/record_video.py position --duration 3 --record logs/position.mp4

# 各模式也有独立脚本；支持相同的 --headless、--side、--duration、--record、--fps 参数
PYTHONPATH= python example/joint_pos_demo.py --headless --duration 3
PYTHONPATH= python example/joint_velo_demo.py --headless --duration 3
PYTHONPATH= python example/force_demo.py --headless --duration 10
PYTHONPATH= python example/target_pos_demo.py --headless --duration 3

PYTHONPATH= python main.py --help
PYTHONPATH= python main.py demo --help
```

`visualize` 默认不限制时长，直到关闭 MuJoCo 窗口或按 Ctrl+C 退出；无窗口模式下按 Ctrl+C 退出。
`demo` 默认运行 10 秒。可选参数 `--duration` 指定仿真运行时长，单位秒，必须为正数且为 0.01 秒的整数倍。
仿真按 1 ms 步进，demo 每 10 ms 发布一次命令。
窗口模式按墙钟时间播放；无窗口模式尽快运行。录像默认 640×480、25 fps，流式写入 H.264 MP4，
按仿真时间取帧。CLI 输出 JSON 状态和录像路径；Ctrl+C 返回 130。

位置 demo 每 3 秒切换开合目标；速度 demo 每 2 秒反向；力控 demo 自动加入五个固定接触球，
每指目标 1 N；Cartesian demo 用 FK 生成可达目标，再经 IK 控制所有指尖。

## 单位、顺序与机械约束

RH56E2 有 **6 个主动自由度、12 个活动关节**。四指各一个主动弯曲轴，拇指有弯曲和旋转两个轴。
其余 6 个关节按原模型的 mimic 比例联动，不能作为独立电机控制。

| 下标 | 主动通道 | URDF 关节 | 范围 rad |
| --- | --- | --- | --- |
| 0 | pinky | pinky_joint | 0..1.4381 |
| 1 | ring | ring_joint | 0..1.4381 |
| 2 | middle | middle_joint | 0..1.4381 |
| 3 | index | index_joint | 0..1.4381 |
| 4 | thumb_bend | thumb_joint2 | 0..0.5864 |
| 5 | thumb_rotation | thumb_joint1 | 0..1.1641 |

- 位置使用 rad，`0` 是模型展开姿态；速度使用带正负号的 rad/s，正数为弯曲方向。
- 四指 DIP = 主动轴 × 1.0843；拇指 joint3/joint4 = joint2 × 0.8024/0.76123688。
- 限位从本地 URDF 读取，并合并被动关节的限位与速度约束。四指主动轴速度上限约 0.9223 rad/s，
  拇指两轴为 1 rad/s；位置参考按此限速，仿真电机限制为 ±0.4 Nm。
- 位置、速度、力、IK 在发布前均校验形状、有限值和范围。用户越界命令抛出 `ValueError`，
  速度积分和力反馈产生的内部目标则在机械边界饱和。MuJoCo 的软约束存在微小数值误差。
- `set_joint()` 可以用被动关节名指定目标，它会反算主动轴并带动整组联动关节；若超出主动轴可达范围也会拒绝。
- 5 指数组顺序是 `[pinky, ring, middle, index, thumb]`，与 6 电机数组不同。

## Python SDK

从本项目目录运行或将该目录加入 Python 搜索路径：

```python
import numpy as np
from inspire_sdk import InspireHand

with InspireHand.simulation(side="right") as hand:
    hand.position.set_finger("index", 0.5)
    hand.position.set_joint("middle_dip", 0.6)  # 自动换算联动主动轴
    hand.advance(2.0)
    state = hand.read_state()
    print(state.positions_rad, state.velocities_rad_s)

    hand.velocity.set_targets([0.1, 0, 0, 0, 0, 0])
    for _ in range(100):
        hand.velocity.step(0.01)
    hand.stop()

    targets = hand.backend.kinematics.forward(hand.backend.description.upper * 0.4)
    result = hand.cartesian.set_targets(targets, tolerance_m=0.001)
    hand.advance(2.0)
    print(result.errors_m)
```

显式进入对应控制循环：位置/3D 目标调用一次后用 `advance()` 推进；速度/力反馈需要周期调用
对应控制器的 `step()`，其中已包含后端推进。切换模式时结束之前的循环并调用 `hand.stop()`。
`close()` 关闭资源；真机 `stop()` 是当前位置保持请求，不是断电急停，也不提供硬件失联看门狗。

力反馈示例：

```python
from inspire_sdk import InspireHand

with InspireHand.simulation(contact_fixture=True) as hand:
    hand.force.set_targets([1, 1, 1, 1, 1])  # N，0..5 N，顺序为五指
    for _ in range(1000):
        hand.force.step(0.01)
    print(hand.read_state().contact_forces_n)
    hand.stop()
    hand.position.set_targets([0]*6)  # 显式张开释放接触
    hand.advance(3)
```

该模式低通滤波接触力，依据力误差调整有界的位置参考；没有接触时读数为 0，最终受机械限位约束。
目标为 0 的通道不继续积分，保持当前位置；释放接触使用显式张开命令。
接触力是每个手指所有接触的法向力总和，未模拟真实触觉阵列，也不等同于电机力矩。

## 3D 指尖目标

`hand.cartesian.set_targets({"index": [x, y, z], "thumb": [x, y, z]})` 接受部分或全部手指目标，
单位 m，坐标系为固定的 `hand_base`。四指仅能沿各自一维可达曲线运动，拇指沿二维曲面运动，
因此不能到达任意空间点。

FK 与 IK 使用同一份 RH56E2 模型和全部 mimic 关系；IK 使用多初值有界最小二乘。
超出 `tolerance_m` 的目标默认拒绝，且不发布任何运动命令。需要投影时显式传
`project=True`，返回的 `IKResult` 包含请求点、投影点、每指误差和 `reachable` 标志。
该求解不规划避障路径。

按任务要求保留 `inspire_sdk/3d_target_pos_control.py`；由于 Python 常规 import 语句不支持数字开头的模块名，
正常导入使用 `inspire_sdk.TargetPositionController` 或 `inspire_sdk.target_pos_control`。

## 状态反馈

| 字段 | 仿真 | 真机 |
| --- | --- | --- |
| positions_rad / velocities_rad_s | MuJoCo 的 6 轴 qpos/qvel | 由 ANGLE_ACT 映射的模型角度 / 差分估计速度 |
| joint_positions_rad / joint_velocities_rad_s | 12 个活动关节的实际仿真值 | 6 轴反馈及按 mimic 推算的被动关节值 |
| fingertip_positions_m | 五指 site 实际坐标 | 由当前模型角度计算的 FK 估计 |
| contact_forces_n | 五指接触法向力，N | `None`，等待硬件力标定 |
| raw_forces | `None` | FORCE_ACT 的六路有符号原始值 |
| error_codes | `None` | 六路设备错误码 |
| velocity_source / force_source | 明确传感器来源 | 明确标记差分估计及未标定力值 |

真机第一次状态读取的速度返回零，并标记 `unavailable_first_sample`。之后速度为两次位置采样差分，
没有虚构独立的速度反馈寄存器。`timestamp_s` 在仿真中是仿真时间，真机中是本机 monotonic 时间。
`state.to_dict()` 可直接序列化为 JSON。

## Modbus TCP 真机接口

```bash
# 只读：连接、查询、断开，不发送运动或初始化写命令
PYTHONPATH= python main.py state --backend modbus --host 192.168.11.210 --port 6000 --device-id 1

# 发布六轴位置；IP、端口和 device-id 必须按实际设备配置
PYTHONPATH= python main.py hardware --host 192.168.11.210 --port 6000 --device-id 1 \
  --positions-rad 0.1 0.1 0.1 0.1 0.1 0.1 --speed-units 150 --force-units 200
```

```python
from inspire_sdk import InspireHand

with InspireHand.modbus_tcp("192.168.11.210", port=6000, device_id=1, side="right") as hand:
    print(hand.read_state().to_dict())
    hand.force.set_native_limits([200]*6)  # 设备原始单位，不是 N
    hand.position.set_finger("index", 0.2)
```

默认端口 6000 沿用 `inspire_demos`；设备也可能配置成 502，可通过参数覆盖。默认右手 ID=1、左手 ID=2。
`RegisterMap` 可由 Python 构造参数 `registers=` 替换。沿用的地址如下：

| 名称 | 地址 | 长度 |
| --- | --- | --- |
| ANGLE_SET | 1486 | 6 × 16-bit |
| FORCE_SET | 1498 | 6 × 16-bit |
| SPEED_SET | 1522 | 6 × 16-bit |
| ANGLE_ACT | 1546 | 6 × 16-bit |
| FORCE_ACT | 1582 | 6 × 16-bit |
| ERROR | 1606 | 3 × 16-bit，拆成 6 个低字节优先的错误码 |

ANGLE_SET 使用 `1000=展开、0=弯曲`，与模型角度做线性映射；这些 rad 是模型估计值，
尚未对实物的传动非线性标定。SPEED_SET 为 1..1000 的速度上限；FORCE_SET 为 0..1000
的设备原始阈值。首次运动前默认设速度 150、力阈值 200，已由用户设置的限制会保留。
连接本身只读；每次位置命令前检查设备错误，通信异常、短读、错误响应或写应答不匹配均抛出异常。
写操作不做自动重试。

真机速度控制使用周期软件位置积分，实际 rad/s 受设备带宽和速度参数标定影响。真机 Newton 力反馈控制
会抛出 `NotImplementedError`；当前可用的是原生阈值加位置命令，等待实机力标定后才能接入统一的 N 接口。
硬件 CLI 的成功表示命令已收到应答，不代表运动已经完成。尚未连接实际 RH56E2 验证固件地址和参数。


## 目录

```text
assets/          原始 RH56E2 模型副本、便携 URDF、MJCF、转换后的网格
inspire_sdk/     统一接口、安全检查、四种控制器、状态、FK/IK、两个后端
visualization/   模型构建、MuJoCo viewer、流式视频记录
example/         四种模式的独立 demo 和共用控制流程
tests/           仿真、通信、CLI、录像测试
third_party/     参考库提交、复用说明及许可证
logs/            本地视频与运行记录，按需生成，不纳入 Git
main.py          CLI 统一入口
requirements.txt 固定版本的直接依赖
```
