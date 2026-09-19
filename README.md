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

完整任务入口会启动 MoveIt 和机器人环境，同一 ROS 域内选择一个运行；独立插入入口连接已有环境。

### 1. 查看关键点和任务调度

```bash
ros2 launch dual_fr3_trunking_mtc demo.launch.py simulation_backend:=fake
```

此入口只生成任务摘要和 RViz 标记，不调用 MTC 规划或执行运动。在 RViz 中添加 `MarkerArray`，话题选择 `/dual_fr3_trunking_planner/markers`，固定坐标系使用 `world`。

### 2. 只做 MTC 规划

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=fake insertion_enabled:=false plan:=true execute:=false
```

终端输出规划结果和阶段诊断；该入口不发布上面的关键点标记。只检查阶段构造时，可同时设置 `plan:=false execute:=false`。

**`mtc_prototype.launch.py` 当前默认 `plan:=true execute:=true`。只想预览时必须显式传入 `execute:=false`。**

### 3. 在仿真中执行

Gazebo 执行：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=gazebo insertion_enabled:=false execute:=true
```

推荐默认入口为 ManiSkill + Rope-Actor + 线缆，执行完整任务：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill cable_solver:=rope_actor load_cable:=true execute:=true \
  maniskill_python:="$PWD/.venv/bin/python"
```

准备自动完成：整段规划预检 → 张开 → 定位 USB/线缆 → 接近 → 双爪闭合 → 解除世界固定 → 稳定验证。随后用实测抓姿更新规划附着，验证缓存的下降、走线和孔前接近轨迹；通过后只等待一次 Enter，开始任务。输入 `q` 中止；无可用终端输入时中止。自动仿真可加 `preparation_interactive:=false`。

从准备到孔前均使用 MoveIt；末端右臂释放退出/回位、左臂接近和完整插入直线预检一起检查，不能只凭走线可规划就开始执行。缓存接近完成后自动调用孔前局部闭环微调，在缺省 2 mm / 3° 范围内修正实际 USB 尖端。推荐 YAML 中达到 0.2 mm / 0.5° 并稳定 0.1 s 后，以 2 mm/s 进入反馈插入；局部入口做一次 MoveIt 状态检查，后续运行有界本地 IK，保留 PhysX 接触和载荷/抓持/关节保护。设置 `insertion.local_collision_check: per_step` 可恢复逐短步校验，参数见[孔前微调说明](../dual_fr3_maniskill/docs/insertion_parameters.md#孔前局部闭环微调2026-09-19)。实测抓姿漂移或环境变化仍可能使执行停止，预规划不代替运行时验证。

ManiSkill 在闭合前创建 USB。`load_cable:=false` 不创建线缆，但保留双臂准备、下降和原后续 MTC 轨迹，作为 USB 搬运调试；`maniskill_cable:=false` 只运行机器人；`execute:=false` 不会生成物体。详见 [MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md)。

MTC 入口默认 `rope_actor`，其余独立物理入口保留原 `mpm` 默认。历史接触夹持记录使用 Rope-Actor + USB-only，不代表带线缆场景的本轮结果。仅 USB 的完整命令见 [MTC 使用说明](../dual_fr3_maniskill/docs/mtc_cable.md#启动)。
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
  simulation_backend:=real insertion_enabled:=false \
  left_robot_ip:=192.168.1.2 right_robot_ip:=192.168.2.2 \
  plan:=true execute:=false
```

核对路径和夹爪参数后，改为 `execute:=true`。真机执行默认先对两个夹爪进行 Homing，夹爪应为空；若已手动完成 Homing，使用 `home_grippers_before_execute:=false grippers_homed:=true`。详细行为见[执行与排查](docs/execution.md)。

### 6. 独立调用插入 skill

完整任务命令追加 `run_insertion:=false` 可在运输完成后保留插座和抓持。待原 MTC 进程结束、USB 仍稳定且世界固定已解除，在同一 ROS 域另开终端执行：

```bash
ros2 launch dual_fr3_trunking_mtc insertion_skill.launch.py
# 仅检查同一整段接近规划：追加 execute:=false
```

此入口不启动第二套仿真，不重新生成或抓取物体，也不等待 Enter。复用 `TerminalInsertion` 的右臂释放退出/回位 → 左臂对齐 → 反馈插入 → 固定 → 左臂释放退出/回位。场景需启用 `insertion_enabled`，MoveGroup 需加载 `move_group/ExecuteTaskSolutionCapability`；若使用自定义 YAML，传入与当前场景相同的 `cable_config:=...`。

完整返回后重复调用直接成功返回；进行中或失败后调用拒绝重放。失败按现有 `/usb_cable_demo/reset` 清理后重新准备夹持。独立调度时预先启用物理插座，见 [独立调用说明](../dual_fr3_maniskill/docs/usb_insertion.md#独立调用)。

## 常用参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `simulation_backend` | `maniskill`（MTC 入口） | `gazebo`、`maniskill`、`fake`、`real` 四选一 |
| `keypoints_file` | 包内 `config/keypoints.yaml` | 任务关键点 |
| `preparation_enabled` | `true` | 启用准备动作；带线缆的 ManiSkill 任务需要保留 |
| `preparation_height` | `0.05` | 初始悬停高度和下降距离，单位米 |
| `preparation_interactive` | `true` | 夹持解除固定、实测规划通过后，下降前一次确认 |
| `motion_velocity_scaling` / `motion_acceleration_scaling` | 均为 `0.2` | 运动速度和加速度比例 |
| `cable_solver` | `rope_actor`（MTC 入口） | ManiSkill 线缆模型：`mpm` 或 `rope_actor` |
| `maniskill_cable` | `true` | 启用 ManiSkill USB 接触夹持场景 |
| `load_cable` | `true` | `false` 不创建线缆，保留原双臂 MTC 轨迹以调试 USB 搬运 |
| `insertion_enabled` | `true`（ManiSkill 接触夹持模式） | 完整流程收尾插入；其他后端/纯机器人模式缺省 false |
| `run_insertion` | `true` | false：运输后保留已启用的插座，供独立 skill 调用 |
| `use_rviz` / `maniskill_viewer` | 均为 `true` | 分别控制 RViz 和 ManiSkill 窗口 |

末端动作修改位置见 [架构](ARCHITECTURE.md#终末插入边界)，孔尺寸、物理、反馈控制及成功/失败阈值见
[插入参数与覆盖关系](../dual_fr3_maniskill/docs/insertion_parameters.md)。当前推荐 YAML 的末端路径比例为 3.0，代码缺省为 1.5。
显式传入 `insertion_enabled` 仍优先于按后端计算的缺省值。

完整参数可查询：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py --show-args
```

## 详细文档

- [任务配置](docs/configuration.md)：关键点、坐标、朝向、夹爪参数和规划参数。
- [执行与排查](docs/execution.md)：执行流程、ROS 接口、失败诊断和逐段工具。
- [架构说明](ARCHITECTURE.md)：启动链、模块职责和扩展位置。
- [MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md)：接触夹持、定位释放和滑移监测。

本轮具体测试和仿真结果见 [工作流验证记录](../dual_fr3_maniskill/docs/insertion_workflow_validation.md)。

参数的作用、单位、消费文件及验证方法见 [参数索引](docs/parameters.md)；当前声明值和配置差异见 [默认值来源](docs/parameter_defaults.md)。
