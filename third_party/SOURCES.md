# 参考库与复用边界

| 来源 | 本地提交 | 复用方式 |
| --- | --- | --- |
| [robot-descriptions-common](https://github.com/fiveages-sim/robot-descriptions-common) | `06a126f55305bdd2d6e50c4f74561c1a1778933e` | RH56E2 原始模型、网格、机械限位、mimic 比例；保留仓库 LICENSE |
| [inspire_demos](https://github.com/TechShare-inc/inspire_demos) | `1e64e674b5919b6168d61d0498e2bd9d173da39e` | 沿用 `inspire_serial.py` 寄存器地址及 `inspire_modbus.py` 六通道批量读写、8-bit 状态拆包方式；MIT 见 inspire_demos_LICENSE |
| [inspire_hands](https://github.com/Sentdex/inspire_hands) | `ae549d9fd0bbad3ff53e1f86e175cf31528ee231` | 核对 `examples/inspire_hand_modbus_proper.py` 的通道顺序、ANGLE_SET 方向与寄存器地址；MIT 见 inspire_hands_LICENSE |
| [inspire-sdk](https://github.com/NYU-robot-learning/inspire-sdk) | `679f495df1f403b819beaa2017f79097b5187f0d` | 参考 `hand_operator.py` 的指尖可达流形→主动电机→联动关节处理流程 |

Modbus 主干保留原库的地址与批量通信逻辑，补充范围校验、device_id、错误响应、写应答校验和连接清理。
没有沿用原库中越界值按位截断、吞掉错误或把通信失败作为成功返回的行为。

`inspire-sdk` 的回归系数来自另一套 RH56 模型，关节范围与当前 RH56E2 Xacro 不一致。
因此本项目保留最近可达点搜索思路，使用当前 URDF 的直接 FK 和有界最小二乘重新求解；
没有复制其回归代码、模型或训练系数。仿真 PD、接触力反馈和速度积分为本项目新增逻辑。

实现时核对的 API 文档：

- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html)
- [PyModbus 3.11.3 client](https://pymodbus.readthedocs.io/en/v3.11.3/source/client.html)

依赖版本固定在 requirements.txt，实际调用接口也由安装后的 Python 签名和 TCP 回环测试验证。
