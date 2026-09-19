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

准备搜索在相同 TCP 位姿下寻找不同 IK 关节配置，并检查后续完整路径。解除固定后更新实测抓姿，先验证剩余缓存轨迹和插入接近，成功则保留原轨迹；验证失败默认停止，仅显式 `replan_after_grasp=true` 才重规划。可恢复的实际执行失败仍按独立的次数上限从实际状态重规划未完成阶段。

## 阶段语义

| 阶段 | 实现 |
| --- | --- |
| 准备定位 | OMPL 到两个初始关键点上方 |
| 准备夹持 | profile 对应的夹爪操作；ManiSkill 解除固定并验证后，下降前统一确认 |
| 准备下降 | 两臂笛卡尔移动合并执行 |
| `straighten` | 原地调整朝向，再做笛卡尔平移 |
| `move_anchor` | leader 通过 OMPL 换锚点，检查 TCP 高度及路程 |
| `seat_edge` | 压线说明阶段、可选夹爪操作、leader 转向让位、follower 转向并用关节插值到目标 |
| `SimulationCable` | `spawn` 在张开准备后创建世界定位的 USB/可选线缆；`release_verify` 在实际接触和释放后稳定验证成功时才同步规划附着体 |

`seat_edge` 中的真实下压接触仍是占位，不代表完成了物理卡线。原地转向用 TCP 位置约束和路径偏差检查限制漂移。

## 模块与修改位置

路径均相对于 `dual_fr3_trunking_mtc/` Python 目录。

| 模块 | 职责与常见修改 |
| --- | --- |
| `task/models.py`、`task/planner.py` | 关键点、段类型、姿态规则和序列化 |
| `task/scheduler.py` | 双臂任务顺序与锚点调度 |
| `stages/specs.py`、`stages/compiler.py` | 新阶段字段和任务到阶段的映射 |
| `task/preparation.py` | 准备阶段与终端确认 |
| `execution/gripper.py` | profile 解析、限值检查、后端夹爪动作 |
| `mtc/task_builder.py` | StageSpec 到 MTC 对象的构造 |
| `mtc/preparation_search.py` | IK 候选、组合选择与全程预检 |
| `mtc/planning.py` | 完整规划重试与成功解保留 |
| `mtc/cached_execution.py` | 缓存子轨迹执行和失败恢复 |
| `mtc/cached_validation.py` | 实测场景中的缓存起点、限位和稠密碰撞检查；同步缓存场景的实测附着体 |
| `mtc/cartesian_validation.py`、`mtc/path_length.py` | TCP 路径约束检查 |
| `mtc/diagnostics.py` | 失败阶段诊断和统计 |
| `runtime/config.py` | 启动、CLI 与节点共享默认值 |
| `runtime/stage_publisher.py` | 阶段序列发布 |
| `nodes/readiness.py` | 一次性启动就绪检查 |
| `execution/simulation_cable.py` | 线缆 spawn 服务和 USB 规划场景更新 |
| `insertion_task/pipeline.py` | 终末动作编排、服务调用、阶段失败上报、临时 ACM 的恢复时机 |
| `insertion_task/motion.py` | 末端 MoveTo/MoveRelative 构造、测量初态规划重试、单次执行 |
| `insertion_task/planning_scene.py` | MoveIt 插座网格消息、安装碰撞对、USB 实测解除附着与夹持碰撞对 |
| `nodes/planner.py`、`nodes/markers.py` | 可视化节点、重载和 RViz 标记 |
| `nodes/prototype.py` | 顶层编排与兼容入口 |

`mtc/executor.py` 保留逐阶段诊断实现；当前主执行流程使用 `cached_execution.py`。`nodes/segment_executor.py` 与固定动作脚本直接复用 MoveIt 控制接口，不能视为完整 MTC 流程的同义入口。

## ManiSkill 线缆边界

`maniskill_cable:=true` 时选择 `trunking_cable` 场景。仿真初始只有机器人；规划中按准备阶段纳入 USB 附着碰撞体，执行到 `SimulationCable` 时再调用 `/maniskill/cable/spawn` 并更新 MoveGroup 场景。

线缆物理由仿真包负责。MTC 不将柔性线缆转换成刚性障碍，不执行线缆状态估计或视觉反馈。
当前接触夹持及 Rope-Actor 分体孔碰撞由物理包维护，不应套用早期固定夹持/理想滑孔版本的结论，详见 [MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md)。

## 终末插入边界

`TerminalInsertion.execute` 保留右爪释放、右臂退出回位、左臂接近、反馈插入、固定确认、左爪释放、左臂退出回位的顺序。

- `insertion_task/planning.py`：在候选运输解的末态构造连续 MTC 任务，包含右爪开度预览、右臂退出/回位、左臂接近及完整插入碰撞预检；按完整解 ID 缓存相连轨迹，预检段不执行。
- `mtc/preparation_search.py` / `mtc/planning.py`：通过 continuation validator 接受同时满足运输和孔前接近的候选。后续恢复规划也必须通过同一检查。
- `mtc/cached_execution.py` / `mtc/cached_validation.py`：release/verify 后以实测 USB 附着验证剩余缓存及原插入接近，通过后才等待唯一一次 Enter；原轨迹不变，缓存场景效果更新为实测附着。失败默认停止，仅显式允许时重规划，保持原笛卡尔目标及已选锚点关节末态；不重放闭爪/解除固定。等待后再次检查抓持。
- `insertion_task/motion.py`：插入后退出/回位的 CurrentState 规划和兼容运动辅助函数。
- `insertion_task/cli.py` / `insertion_skill.launch.py`：连接已有场景，从当前稳定夹持开始规划整段接近，然后复用相同终末流程。

插入前使用缓存 MoveIt 轨迹，随后调用 align 完成孔前局部闭环微调；仅在 2 mm / 3° 捕获范围内，以实测 USB 尖端和当前抓姿生成连续小步 IK，不调用 OMPL 重规划。推荐 YAML 的 `local_collision_check=entry` 只在局部入口校验实测状态，之后微调/插入连续执行本地 IK；`per_step` 保留原异步短步校验。两种模式均保留 PhysX 接触和载荷/抓持/关节保护。连续达到微调精度和低速条件后，start 仍按实际 USB 位姿检查原插入阈值。超出范围或检查失败则停止并保持夹爪。右臂退出仍属于 `right_return`，失败保持原上报语义。执行失败不自动重放部分轨迹。

物理包 `usb/geometry.py` 统一孔壁调整、USB 关键点及抓姿变换；MTC 保留三角网格消息构造，
`usb/scene.py` 保留凸分解、支撑反力采样和实体/约束生命周期。`usb/insertion.py` 的纯策略用仿真时间
滤波、推进及确认成功；`usb/bridge.py` 的具名服务处理负责控制权、忙检查和墙钟 heartbeat。
只有先记录 `inserted_unretained`、释放控制权后，物理步边界才创建保持约束；MTC 收到 retained 确认后才开左爪。
status 不续租；失败/取消仍归还驱动控制权，MTC 的 finally 负责取消和恢复临时碰撞许可。

推荐使用 `mtc_prototype.launch.py simulation_backend:=maniskill cable_solver:=rope_actor insertion_enabled:=true`，
USB-only 追加 `load_cable:=false`。参数来源、坐标系、默认值差异和调参验证路径见
[插入参数](../dual_fr3_maniskill/docs/insertion_parameters.md)，本轮结果见
[初轮重构验证](../dual_fr3_maniskill/docs/insertion_refactor_validation.md) 和
[本轮工作流验证](../dual_fr3_maniskill/docs/insertion_workflow_validation.md)。

## 验证边界

`test/` 覆盖规划与调度、配置、准备搜索、路径约束、执行恢复、夹爪和后端启动。离线测试不代表真机或 GPU 完整任务成功。测试入口见 [ManiSkill 环境与验证](../dual_fr3_maniskill/docs/setup.md)，运行诊断见[执行说明](docs/execution.md)。

## 目录边界与兼容

```text
dual_fr3_trunking_mtc/
├── task/            数据模型、关键点解析、调度、准备阶段定义
├── stages/          可序列化阶段规格与编译
├── mtc/             MTC 对象构造、整段规划、缓存执行及恢复
├── execution/       夹爪 action 与物理线缆服务客户端
├── insertion_task/  终末流程、连续接近规划、场景同步、独立 skill CLI
├── nodes/           ROS 入口装配、就绪门控、可视化、旧诊断执行器
├── runtime/         默认值、CLI 解析、日志和阶段发布
└── _compat/         旧根模块的兼容别名
```

`nodes/prototype.py` 只装配任务流程；参数定义/校验在 `runtime/arguments.py`，数值缺省在 `runtime/config.py`。
`task → stages → mtc` 传递领域计划与阶段规格；`execution` 处理外部动作，`insertion_task` 复用这些客户端。
`insertion_task/cli.py` 是独立 skill 入口，不启动第二个物理场景。

根包将 `_compat` 追加到模块搜索路径末尾。旧导入与新路径是同一模块对象，既有脚本与测试中的 monkeypatch 保持有效；
无需全局 import hook，也不预加载全部 ROS/物理模块。新代码使用上表的新路径。历史文档中的旧路径通过兼容目录查找。

全部参数的作用与消费文件见 [参数索引](docs/parameters.md)，各入口的当前数值见 [默认值来源](docs/parameter_defaults.md)。
本轮结构整理的测试与局限见 [验证记录](../dual_fr3_maniskill/docs/structure_validation.md)。
