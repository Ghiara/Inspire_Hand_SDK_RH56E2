# RH56E2 模型来源与构建

原始来源：`../robot-descriptions-common/gripper/inspire_description`，仓库
<https://github.com/fiveages-sim/robot-descriptions-common>，本地提交
`06a126f55305bdd2d6e50c4f74561c1a1778933e`。

`inspire_description/meshes/RH56E2/`、`inspire_description/xacro/`、原始 README、package.xml
及仓库 LICENSE 均保留副本。上游提供的是 Xacro，而非已展开的独立 URDF。

`rh56e2_right.urdf`、`rh56e2_left.urdf` 是展开后的便携 URDF；`.xml` 是由同一份
URDF 的关节变换、轴、质量、惯量和 mimic 比例构建的 MuJoCo 场景。
`converted/{right,left}` 是 MuJoCo 可读的 OBJ 网格：从原始 GLB/STL 导出，保留场景节点变换，
并烘焙左右手的镜像缩放。右手 direction=-1，左手 direction=1。

相对上游的仿真设置：去掉可选腕部法兰、固定 hand_base 于原点、重力设为零，
使用 implicitfast、1 ms 步长；加入 6 个受限力矩电机、6 组 mimic 等式约束、
5 个指尖 site、目标标记、光源、地面。电机力矩上限 0.4 Nm，位置参考受 URDF 速度限幅。
每个原始 visual 网格用作一个凸包碰撞体。关闭手自身碰撞，保留手与外部物体接触。
这些设置适合控制算法验证，未标定电机、传动、柔性皮肤或触觉阵列。

指尖定义为最末端力传感器 patch 的坐标原点：四指 `*_force_sensor_3`，拇指
`thumb_force_sensor_4`。这使 FK、IK、状态反馈与可视化使用一致定义。

在项目目录运行 `python -m visualization.build_assets`，仅依赖 assets 内的副本重建。
需要重新从原始库复制时才传入：

```bash
python -m visualization.build_assets --source ../robot-descriptions-common/gripper/inspire_description
```
