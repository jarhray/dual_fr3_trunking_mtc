# 双 FR3 线槽走线任务

`dual_fr3_trunking_mtc` 根据关键点 YAML 生成双臂走线任务，使用 MoveIt Task Constructor（MTC）规划并执行准备、夹持、跟随和让位动作。机器人模型与控制环境由 [dual_fr3_moveit_config](../dual_fr3_moveit_config/README.md) 提供，线缆物理由 [dual_fr3_maniskill](../dual_fr3_maniskill/README.md) 提供。

当前支持完整任务预检和按阶段执行已规划轨迹；`seat_edge` 的物理压线仍是占位逻辑，没有力控或视觉闭环。

## 构建

需要 ROS 2 Humble、MoveIt 2、MTC Python 绑定及工作区的 Franka 依赖。在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dual_fr3_trunking_mtc \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

每个新终端都需加载 ROS 和 `install/setup.bash`。使用 ManiSkill 前，还需完成[仿真环境安装](../dual_fr3_maniskill/docs/setup.md)。

## 使用方法

以下入口都会启动所需的 MoveIt 和机器人环境，同一 ROS 域内选择一个运行。

### 1. 查看关键点和任务调度

```bash
ros2 launch dual_fr3_trunking_mtc demo.launch.py simulation_backend:=fake
```

此入口只生成任务摘要和 RViz 标记，不调用 MTC 规划或执行运动。在 RViz 中添加 `MarkerArray`，话题选择 `/dual_fr3_trunking_planner/markers`，固定坐标系使用 `world`。

### 2. 只做 MTC 规划

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=fake plan:=true execute:=false
```

终端输出规划结果和阶段诊断；该入口不发布上面的关键点标记。只检查阶段构造时，可同时设置 `plan:=false execute:=false`。

**`mtc_prototype.launch.py` 当前默认 `plan:=true execute:=true`。只想预览时必须显式传入 `execute:=false`。**

### 3. 在仿真中执行

Gazebo 执行：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=gazebo execute:=true
```

ManiSkill 执行，并在准备阶段生成线缆：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor load_cable:=true execute:=true \
  maniskill_python:="$PWD/.venv/bin/python"
```

ManiSkill 流程为：完整路径预检 → 张开 → 按关键点提前定位 USB/线缆 → 接近准备位 → 接触闭合 → 解除定位 → 稳定验证 → 规划附着 → 下降与走线。默认在两次闭合和下降前等待确认，按 Enter 继续，输入 `q` 中止；无可用终端输入时会中止。自动仿真可加 `preparation_interactive:=false`。

ManiSkill 在闭合前创建 USB。`load_cable:=false` 不创建线缆，但保留双臂准备、下降和原后续 MTC 轨迹，作为 USB 搬运调试；`maniskill_cable:=false` 只运行机器人；`execute:=false` 不会生成物体。详见 [MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md)。

本次接触夹持验收使用 `cable_solver:=rope_actor` 和 USB-only；MPM 后续完善。全局默认仍是 `mpm`，因此推荐命令显式选择 Rope-Actor。仅 USB 的完整命令见 [MTC 使用说明](../dual_fr3_maniskill/docs/mtc_cable.md#启动)。
参数、模型差异和验证方法见[线缆建模方式](../dual_fr3_maniskill/docs/cable_backends.md)。

### 4. 使用自己的路径

复制并编辑 [config/keypoints.yaml](config/keypoints.yaml)，在上述任一入口追加：

```bash
keypoints_file:="$PWD/src/dual_fr3_trunking_mtc/config/keypoints.yaml"
```

关键点默认使用 `left_fr3_link0`，位置单位为米。`in_slot` 相同的相邻点生成跟随段，发生变化时生成 `seat_edge` 段。点级 `rpy` 不受支持，工具朝向由路径和任务参数计算。字段、夹爪配置和调参方法见[任务配置](docs/configuration.md)。

### 5. 连接真机

先只规划，将下面的示例 IP 替换为实际地址：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=real \
  left_robot_ip:=192.168.1.2 right_robot_ip:=192.168.2.2 \
  plan:=true execute:=false
```

核对路径和夹爪参数后，改为 `execute:=true`。真机执行默认先对两个夹爪进行 Homing，夹爪应为空；若已手动完成 Homing，使用 `home_grippers_before_execute:=false grippers_homed:=true`。详细行为见[执行与排查](docs/execution.md)。

## 常用参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `simulation_backend` | `gazebo` | `gazebo`、`maniskill`、`fake`、`real` 四选一 |
| `keypoints_file` | 包内 `config/keypoints.yaml` | 任务关键点 |
| `preparation_enabled` | `true` | 启用准备动作；带线缆的 ManiSkill 任务需要保留 |
| `preparation_height` | `0.05` | 初始悬停高度和下降距离，单位米 |
| `preparation_interactive` | `true` | 准备阶段终端确认 |
| `motion_velocity_scaling` / `motion_acceleration_scaling` | 均为 `0.2` | 运动速度和加速度比例 |
| `cable_solver` | `mpm` | ManiSkill 线缆模型：`mpm` 或 `rope_actor` |
| `maniskill_cable` | `true` | 启用 ManiSkill USB 接触夹持场景 |
| `load_cable` | `true` | `false` 不创建线缆，保留原双臂 MTC 轨迹以调试 USB 搬运 |
| `use_rviz` / `maniskill_viewer` | 均为 `true` | 分别控制 RViz 和 ManiSkill 窗口 |

完整参数可查询：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py --show-args
```

## 详细文档

- [任务配置](docs/configuration.md)：关键点、坐标、朝向、夹爪参数和规划参数。
- [执行与排查](docs/execution.md)：执行流程、ROS 接口、失败诊断和逐段工具。
- [架构说明](ARCHITECTURE.md)：启动链、模块职责和扩展位置。
- [MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md)：接触夹持、定位释放和滑移监测。
