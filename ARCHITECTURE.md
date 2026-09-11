# 双 FR3 走线任务架构

本文面向代码维护。运行命令见 [README](README.md)，参数见[任务配置](docs/configuration.md)，接口与排查见[执行说明](docs/execution.md)。

## 包的职责

| 包 | 职责 |
| --- | --- |
| `dual_fr3_trunking_mtc` | 关键点解析、双臂调度、MTC 阶段编译、准备搜索与执行 |
| `dual_fr3_moveit_config` | 双臂模型、规划组、MoveIt、后端启动与控制器映射 |
| `dual_fr3_maniskill` | 机器人物理、ROS 动作桥接、线缆模型及场景 |

物理包接收上层提供的最终 URDF/SRDF，不反向依赖 MoveIt 配置包。MTC 与 MoveGroup 使用同一套模型构造逻辑。

## 启动链

```text
mtc_prototype.launch.py
├─ dual_fr3_moveit_config/demo.launch.py
│  ├─ gazebo：Gazebo 与 ros2_control
│  ├─ maniskill：SAPIEN 与 ROS 动作桥接
│  └─ fake / real：左右独立 ros2_control 与夹爪节点
└─ trunking_readiness_gate.py
   ├─ 成功：启动 trunking_mtc_prototype.py
   └─ 失败：关闭本次 launch
```

就绪检查等待状态、控制器、MoveGroup 和夹爪。真机执行还进行 Homing 或要求已有回零确认。MTC 执行 capability 由启动文件加入 MoveGroup。

`demo.launch.py` 是另一个入口，只启动机器人环境和 `trunking_plan_node.py`，发布关键点、段分类与调度结果，不创建 MTC Task。

## 数据流

```text
关键点 YAML
  → planner：Keypoint[] → SegmentPlan[]
  → scheduler：TaskPlan
  → stages.compiler：准备与正式阶段的 MtcStageSpec[]
  → mtc.task_builder：MTC Task
  → preparation_search / planning：完整任务成功解
  → cached_execution：依次执行成功解的子轨迹
```

`MtcStageSpec` 是领域模型和 MTC 之间的可序列化接口，同时用于阶段话题和诊断。关键点的 `in_slot` 决定段分类，调度器决定双臂顺序，编译器决定具体运动与夹爪阶段。

准备搜索在相同 TCP 位姿下寻找不同 IK 关节配置，并检查后续完整路径。正常执行复用成功解，只有可恢复的执行失败才从实际状态重规划未完成阶段。

## 阶段语义

| 阶段 | 实现 |
| --- | --- |
| 准备定位 | OMPL 到两个初始关键点上方 |
| 准备夹持 | profile 对应的夹爪操作，保留交互确认 |
| 准备下降 | 两臂笛卡尔移动合并执行 |
| `straighten` | 原地调整朝向，再做笛卡尔平移 |
| `move_anchor` | leader 通过 OMPL 换锚点，检查 TCP 高度及路程 |
| `seat_edge` | 压线说明阶段、可选夹爪操作、leader 转向让位、follower 转向并用关节插值到目标 |
| `SimulationCable` | ManiSkill 准备闭合后生成线缆，并同步 USB 附着碰撞体 |

`seat_edge` 中的真实下压接触仍是占位，不代表完成了物理卡线。原地转向用 TCP 位置约束和路径偏差检查限制漂移。

## 模块与修改位置

路径均相对于 `dual_fr3_trunking_mtc/` Python 目录。

| 模块 | 职责与常见修改 |
| --- | --- |
| `models.py`、`planner.py` | 关键点、段类型、姿态规则和序列化 |
| `scheduler.py` | 双臂任务顺序与锚点调度 |
| `stages/specs.py`、`stages/compiler.py` | 新阶段字段和任务到阶段的映射 |
| `preparation.py` | 准备阶段与终端确认 |
| `gripper.py` | profile 解析、限值检查、后端夹爪动作 |
| `mtc/task_builder.py` | StageSpec 到 MTC 对象的构造 |
| `mtc/preparation_search.py` | IK 候选、组合选择与全程预检 |
| `mtc/planning.py` | 完整规划重试与成功解保留 |
| `mtc/cached_execution.py` | 缓存子轨迹执行和失败恢复 |
| `mtc/cartesian_validation.py`、`mtc/path_length.py` | TCP 路径约束检查 |
| `mtc/diagnostics.py` | 失败阶段诊断和统计 |
| `runtime/config.py` | 启动、CLI 与节点共享默认值 |
| `runtime/stage_publisher.py` | 阶段序列发布 |
| `readiness.py` | 一次性启动就绪检查 |
| `simulation_cable.py` | 线缆 spawn 服务和 USB 规划场景更新 |
| `ros_node.py`、`markers.py` | 可视化节点、重载和 RViz 标记 |
| `mtc_prototype.py` | 顶层编排与兼容入口 |

`mtc/executor.py` 保留逐阶段诊断实现；当前主执行流程使用 `cached_execution.py`。`segment_executor.py` 与固定动作脚本直接复用 MoveIt 控制接口，不能视为完整 MTC 流程的同义入口。

## ManiSkill 线缆边界

`maniskill_cable:=true` 时选择 `trunking_cable` 场景。仿真初始只有机器人；规划中按准备阶段纳入 USB 附着碰撞体，执行到 `SimulationCable` 时再调用 `/maniskill/cable/spawn` 并更新 MoveGroup 场景。

线缆物理由仿真包负责。MTC 不将柔性线缆转换成刚性障碍，不执行线缆状态估计、力控或视觉反馈。固定端与滑孔是理想约束，详见 [MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md)。

## 验证边界

`test/` 覆盖规划与调度、配置、准备搜索、路径约束、执行恢复、夹爪和后端启动。离线测试不代表真机或 GPU 完整任务成功。测试入口见 [ManiSkill 环境与验证](../dual_fr3_maniskill/docs/setup.md)，运行诊断见[执行说明](docs/execution.md)。
