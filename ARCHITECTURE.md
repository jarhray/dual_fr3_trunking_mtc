# dual_fr3_trunking_mtc 架构说明

本文描述 `dual_fr3_trunking_mtc` 当前代码结构、启动链、任务数据流、配置来源和模块边界。
它面向维护和扩展代码的开发者，不替代具体的真机操作与调试记录；真机状态和历史问题仍以
`HANDOFF.md` 为准。

## 1. 系统定位

本包位于双 FR3 系统的任务层，负责把线槽路径关键点转换为双臂协作动作，并进一步映射成
MoveIt Task Constructor（MTC）Stage。

本包负责：

- 读取和验证线槽关键点。
- 根据相邻关键点的 `in_slot` 状态判断段动作。
- 调度 leader 和 follower 的移动、保持与让位顺序。
- 生成与 ROS/MTC 对象解耦的 `MtcStageSpec` 序列。
- 将 StageSpec 转换为真正的 MTC Task。
- 执行准备动作、整体任务或逐 Stage 任务。
- 在启动 MTC 前检查控制器、关节状态、MoveGroup 和夹爪。

本包不负责：

- 机器人 URDF、SRDF、MoveIt controller 和 ros2_control 的主要配置。
- Gazebo 世界与机器人生成。
- 完整的视觉、力控、接触检测和线缆状态估计。
- 当前 `seat_cable_on_edge` 的真实接触操作；该 primitive 仍是占位描述。

上述机器人环境由 `dual_fr3_moveit_config` 提供。

## 2. 入口概览

包内有两个正式 launch 入口和两个诊断入口。

| 入口 | 用途 | 是否创建 MTC Task |
|---|---|---|
| `launch/mtc_prototype.launch.py` | 主 MTC 规划与执行入口 | 是 |
| `launch/demo.launch.py` | 关键点、线段和调度结果可视化 | 否 |
| `trunking_segment_executor.py` | 按 Stage 诊断控制器与规划结果 | 否，直接使用 MoveGroup/控制器接口 |
| `trunking_step_by_step.py` | 手动执行固定动作序列 | 否 |

`scripts/` 下的短脚本只是可执行入口，实际逻辑位于 Python 包内。

## 3. 主启动拓扑

`mtc_prototype.launch.py` 是当前主入口：

```text
mtc_prototype.launch.py
│
├─ use_gazebo=false
│  └─ include dual_fr3_moveit_config/launch/demo.launch.py
│
├─ use_gazebo=true
│  └─ include dual_fr3_moveit_config/launch/gazebo.launch.py
│
├─ start trunking_readiness_gate.py
│  └─ TrunkingReadinessGate.run()
│
└─ readiness 进程退出
   ├─ return code == 0
   │  └─ start trunking_mtc_prototype.py
   └─ return code != 0
      └─ 关闭整个 launch，不启动 MTC
```

MoveIt 环境和 readiness gate 同时开始启动。readiness gate 会持续等待依赖就绪，而不是
依赖固定延时。MTC 节点只会在 readiness 成功退出后启动。

### 3.1 MoveIt 资源构造

`dual_fr3_moveit_config.moveit_resources` 是 URDF/SRDF、kinematics 和 OMPL 参数的
唯一构造入口。`dual_fr3_moveit_config` 的 demo/Gazebo launch 和本包的 MTC
launch 都调用它，不再复制 xacro `Command`、YAML 读取与 request adapter
配置。

hardware 和 Gazebo 的 URDF xacro 不同，因此 MTC launch 保留两个条件互斥的
MTC Node，但它们只复用同一份命令行参数并从共享构造器取得参数。
`gazebo_effort`、`load_gripper` 和 `ee_id` 会同时传给 Gazebo MoveGroup 与 MTC
节点，避免两边加载不同的 robot model。

### 3.2 Readiness 检查

`readiness.py` 是一次性启动闸门，不参与后续任务调度。它检查：

- `/move_action`。
- 左右 `FollowJointTrajectory` Action Server。
- 左右 `controller_state` 是否存在且新鲜。
- `/joint_states` 是否包含 14 个机械臂关节且消息足够新。
- 启用夹爪时，对应后端的夹爪 Action Server。
- `execute=true` 时的 `/execute_task_solution`。
- 真机执行前的左右夹爪 Homing，或操作者已经 Homing 的明确确认。

任何检查失败都会返回非零退出码，阻止 MTC 节点启动。

## 4. 主任务数据流

主进程的静态数据流为：

```text
config/keypoints.yaml
        │
        ▼
planner.load_keypoints()
        │
        ▼
Keypoint[]
        │
        ▼
planner.build_segment_plans()
        │
        ▼
SegmentPlan[]
        │
        ▼
scheduler.build_task_plan()
        │
        ▼
TaskPlan
（keypoints + segments + steps + 初始双臂索引）
        │
        ├──────────────► JSON summary / RViz markers
        │
        ▼
stages.compiler.build_mtc_stage_specs(task_plan, preparation_config)
        │
        ▼
统一 MtcStageSpec[]
（preparation phase + formal phase）
        │
        ├──────────────► JSON / text stage sequence
        │
        ▼
mtc.preparation_search（启用 preparation 时）
  同一场景 → 多初值 IK 去重 → 候选组合 → 全程预检
        │
        ▼
mtc.task_builder.create_mtc_task()
        │
        ▼
MoveIt Task Constructor Task
        │
        ├─ task.plan()
        ├─ task.execute()                  整体执行
        └─ mtc.cached_execution            执行选定解，异常时恢复未完成阶段
```

这是单向的数据依赖：段动作只在 planner 中分类一次；scheduler 只消费已经分类的
`SegmentPlan[]`，不会再次读取 `Keypoint.in_slot` 判断动作；Stage compiler 只接收一个
`TaskPlan`，不再分别接收关键点、TaskStep 和初始索引。

对应的核心调用方式是：

```python
keypoints = load_keypoints(keypoints_file)
segments = build_segment_plans(keypoints)
task_plan = build_task_plan(
    segments,
    initial_leader_index=1,
    initial_follower_index=0,
)
stage_specs = build_mtc_stage_specs(
    task_plan,
    preparation_config=preparation_config,
)
```

旧调用需要按下面方式迁移：

| 旧接口 | 当前接口 |
|---|---|
| `build_task_schedule(keypoints, ...)` | `build_task_plan(build_segment_plans(keypoints), ...)` |
| `build_mtc_stage_specs(keypoints, task_steps, initial_...)` | `build_mtc_stage_specs(task_plan, ...)` |
| `create_mtc_task(node, keypoints, task_steps, initial_...)` | `create_mtc_task(node, task_plan, stage_specs, ...)` |

`build_task_schedule()` 名称暂时保留为轻量兼容视图，但它现在同样只接收 Segment，并只返回
`TaskPlan.steps` 的列表副本。新代码应使用 `build_task_plan()`，避免再次把统一规划对象拆散。

准备阶段由 `preparation.py` 编译为同一种 `MtcStageSpec`。它和正式 Stage 共用
`mtc/task_builder.py`；`mtc/preparation_search.py` 在任何准备 Stage 执行前搜索准备关节配置，
将 preparation 和 formal 作为同一条完整任务预检并保留成功解。

## 5. 分层和模块职责

### 5.1 领域模型与任务规划

| 文件 | 主要职责 |
|---|---|
| `models.py` | `Keypoint`、`SegmentPlan`、`TaskStep`、`TaskPlan` 和姿态方向公共函数 |
| `planner.py` | YAML 解析、段分类、路径采样、规划摘要 |
| `scheduler.py` | 校验连续 Segment 链并生成包含全局 `TaskStep` 顺序的 `TaskPlan` |

段分类的当前规则是：

```text
false -> false    straighten
true  -> true     straighten
false -> true     seat_edge
true  -> false    seat_edge
```

leader 默认是左臂，方向为 `reverse`；follower 默认是右臂，方向为 `forward`。

`TaskPlan` 是调度层之后的统一传递边界，包含：

- 从 Segment 链恢复出的有序关键点。
- planner 生成且 scheduler 实际消费的 Segment。
- scheduler 生成的 TaskStep。
- leader/follower 的初始关键点索引。

scheduler 会拒绝索引不连续、首尾不衔接或 action 不受支持的 Segment 链。这样进入
compiler 的 TaskPlan 不会出现关键点、Segment 和 TaskStep 分别传递后互不一致的问题。

### 5.2 Stage 描述层

`stages/` 不创建或执行 MTC Task，而是描述“应该有哪些动作”。

| 文件 | 主要职责 |
|---|---|
| `stages/specs.py` | 定义 `MtcStageSpec`，提供 dict、JSON 和文本序列化 |
| `stages/compiler.py` | 从 `TaskPlan` 编译出 preparation + formal 的统一 `MtcStageSpec[]` |

`MtcStageSpec` 是整个任务程序的统一中间表示，包含：

- 稳定的 `stage_key` 和顺序索引。
- action、actor、planning group 和 IK frame。
- 起终点索引、相对位移和目标 yaw。
- primitive、MTC Stage 类型和 planner 类型。
- 是否真正可执行。
- 可选夹爪 profile 与覆盖值。
- `preparation` 或 `formal` phase，以及是否需要操作者确认。
- `Merger` 等复合 Stage 的子 `MtcStageSpec`。

Stage 编译层只决定动作语义，不持有 MTC Task 生命周期。除 group、IK frame、方向和距离等
编译配置外，它的唯一规划数据输入是 `TaskPlan`。

### 5.3 MTC 适配层

| 文件 | 主要职责 |
|---|---|
| `mtc/task_builder.py` | 消费统一 StageSpec，创建 planner、路径约束和 MTC Task |
| `mtc/executor.py` | 保留的逐阶段诊断执行器，主入口不再用它先执行准备动作 |
| `mtc/preparation_search.py` | 捕获场景、多初值 IK 去重、对角遍历双臂候选，用固定准备关节目标预检全程；保留原模型加载器生命周期 |
| `mtc/planning.py` | 从 CurrentState 有限次完整规划，保留第一份成功解 |
| `mtc/cached_execution.py` | 按 MTC solution ID 提取同一成功解的阶段，执行原轨迹并恢复失败后的未完成部分 |
| `mtc/path_length.py` | 通过 FK 插值检查锚点 TCP 路程比例上限 |
| `mtc/cartesian_validation.py` | 检查时间参数化后的直线 TCP 偏差和原地转向位置漂移，包含 Merger 最终轨迹 |

当前 planner 映射：

| Stage 语义 | MTC 类型 | Planner |
|---|---|---|
| 原地转向 | `MoveTo` | `CartesianPath` |
| 沿路径移动 | `MoveRelative` | `CartesianPath` |
| follower 到 seat-edge 点 | `MoveTo` | `JointInterpolationPlanner` |
| leader 到下一个 anchor | `MoveTo` | `PipelinePlanner / OMPL` |
| 夹爪操作 | `MoveTo` 的手指关节表示 | 实际逐 Stage 执行时发送夹爪 Action |
| 双臂同步下降 | `Merger[MoveRelative x 2]` | `CartesianPath` |
| 物理卡线占位 | `InfoOnly` | 不执行 |

`direct_move_to_next_anchor` 增加 TCP 最大 z 路径约束和默认 1.5 倍直线距离的路程上限。
`CartesianPath` 启用 `cartesian_jump_threshold=2.0`，要求 `min_fraction=1.0`；
候选还必须通过默认 `cartesian_path_tolerance=0.01 m` 的密集 FK 位置检查。
检查失败返回无限 cost，禁止传播到完整成功解；正常规划与恢复规划共用检查。

### 5.4 运行时层

| 文件 | 主要职责 |
|---|---|
| `runtime/config.py` | launch、CLI 和节点共享的唯一默认值来源 |
| `runtime/stage_publisher.py` | 发布统一 StageSpec 程序的 JSON 和文本表示 |
| `mtc_prototype.py` | 参数解析、各层组装、顶层运行流程和旧 API 兼容 |

`mtc_prototype.py` 仍重导出常用符号，但规划接口已经统一为 `TaskPlan`：
`build_task_schedule()` 的输入也是 Segment，Stage compiler 和 MTC task builder 不再接受
分离的 `keypoints + task_steps`。新增代码应优先从 `stages/`、`mtc/` 或 `runtime/` 的实际
所属模块导入。

### 5.5 准备动作和夹爪

| 文件 | 主要职责 |
|---|---|
| `preparation.py` | 将初始接近、夹爪闭合和同步下降编译为 StageSpec，并处理确认输入 |
| `gripper.py` | profile 加载、安全限制、后端选择和 Action 调用 |

准备阶段固定顺序为：

```text
leader 移动到初始关键点上方
→ follower 移动到初始关键点上方
→ leader 夹爪闭合
→ follower 夹爪闭合
→ 双臂同步 Cartesian 下降
```

这五步不再拥有单独的 `PreparationStep`、MTC builder 或 executor。前两步是 `MoveTo`
spec，两次闭合是 `GripperOperation` spec，最后一步是带两个 `MoveRelative` 子 spec 的
`Merger`。因此预览、序列化、筛选、单步规划和失败即停逻辑与正式 Stage 完全共用。

夹爪 profile 来自 `config/gripper_profiles.yaml`。profile 的 `width` 始终表示两指总开口，
只有在映射为 MoveIt 手指关节目标时才除以二。

夹爪后端由运行环境唯一决定：

```text
use_gazebo=true         -> gazebo
use_gazebo=false 且
use_fake_hardware=true  -> fake
其他                    -> franka
```

## 6. 配置模型

`runtime/config.py` 中的 `TrunkingDefaults` 是包内默认值的唯一来源。它覆盖：

- 机器人环境和 IP。
- leader/follower group、IK frame 和方向。
- 运动缩放、Cartesian 步长和 anchor 路径高度。
- preparation 和夹爪 profile。
- plan/execute 模式。
- readiness 超时和状态新鲜度。
- 规划可视化参数。

配置传递关系为：

```text
TrunkingDefaults
├─ launch_default() -> DeclareLaunchArgument 默认值
├─ mtc_prototype._parse_args() 默认值
├─ PreparationConfig 默认值
├─ TrunkingReadinessGate 参数默认值
└─ TrunkingPlannerNode 参数默认值
```

用户在 launch 命令行提供的值仍具有最高优先级。配置文件路径需要安装空间中的实际 share
路径，因此由 launch 或入口函数动态解析，不写死在 `TrunkingDefaults` 中。

当前关键默认值：

```text
use_fake_hardware=true
use_gazebo=false
gazebo_effort=false
plan=true
execute=false
execute_stage_by_stage=true
preparation_enabled=true
preparation_height=0.05
anchor_max_path_z=0.6
leader_lead_distance=0.10
preparation_leader_gripper_profile=cable_tip
preparation_follower_gripper_profile=cable_tip
mtc_keep_alive_sec=30.0
```

## 7. 规划与执行模式

| `plan` | `execute` | `execute_stage_by_stage` | 行为 |
|---|---|---|---|
| false | false | 任意 | 只构造 Task 和 StageSpec，不调用规划 |
| true | false | 任意 | 含 preparation 时搜索准备姿态并预检全程，否则有限次完整规划；可发布成功解 |
| true | true | true | 先预检全程，再按阶段直接执行同一成功解，异常时重规划剩余部分 |
| true | true | false | 仅限无 preparation 且无夹爪 Stage，一次性执行成功解，失败后停止 |

逐 Stage 模式的关键性质：

- preparation 启用时，每臂最多 `preparation_ik_attempts=80` 个 IK 初值，保留 `preparation_ik_candidates=8` 个不同关节配置。TCP 目标不变，关节去重距离默认 `0.3 rad`。
- 候选组合按对角顺序遍历，最多 `preparation_candidate_attempts=2` 轮。只有 PipelinePlanner 阶段失败时，每组最多 `planning_attempts=10` 次完整预检；Cartesian 失败换下一组。搜索预算 `preparation_search_timeout=180` 秒在原生求解调用之间检查，可能被当前调用超出。
- 全部候选基于同一捕获场景的 FixedState 副本。准备 MoveTo 使用选中的关节目标，完整解通过所有现有约束后才开始执行。无成功解则不执行准备 Stage。
- 关闭 preparation 或恢复未完成后缀时，使用 `planning_attempts=10` 次规划上限。找到成功解后保留原 Task 和 solution。
- 通过完整解中的 solution ID 选择各阶段，不能独立选各阶段最便宜的解，否则可能不连接。
- 正常执行不再创建和规划新的 Task。
- 可恢复的终止错误触发最多 `execution_replan_attempts=2` 次恢复，每次从 CurrentState 规划未完成部分。
- 相对运动恢复使用首次成功解中的绝对终点；准备动作恢复保持原选中关节目标；约束不放宽。
- 取消、未知执行状态、夹爪失败和耗尽重试预算会停止。准备动作也适用同一异常恢复规则，已确认的阶段无需重复确认。
- 夹爪 Stage 直接通过选定后端发送 Action，不通过机械臂规划器执行。

## 8. ROS 接口

### 8.1 MTC Stage sequence

`runtime/stage_publisher.py` 发布：

```text
/dual_fr3_trunking_mtc_prototype/stage_sequence
/dual_fr3_trunking_mtc_prototype/stage_sequence_text
```

两者使用 `TRANSIENT_LOCAL` QoS。JSON 接口当前版本为 `interface_version=2`。

这里发布的是完整统一程序，包含 preparation 和 formal Stage。版本 2 新增 `phase`、
`confirmation_required` 和递归 `children` 字段。

### 8.2 规划可视化节点

`TrunkingPlannerNode` 发布：

```text
~/plan_summary
~/task_schedule
~/markers
```

并提供：

```text
~/replan
```

它属于 `demo.launch.py`，不会被 `mtc_prototype.launch.py` 启动。

## 9. 当前默认关键点对应的任务

`config/keypoints.yaml` 当前启用四个关键点：

```text
corner_1(true)
  -> corner_2(true)     straighten
  -> corner_4(true)     straighten
  -> entry_5(false)     seat_edge
```

使用默认初始索引时，调度结果为：

```text
1. leader: corner_2 -> entry_5，follower 保持 corner_1
2. follower: corner_1 -> corner_2
3. follower: corner_2 -> corner_4
4. terminal seat_edge
```

正式部分生成六个 StageSpec，其中四个可执行。terminal `seat_cable_on_edge` 仍是
`InfoOnly`，不会向机械臂发送运动命令。

## 10. 扩展代码时应该修改哪里

### 新增关键点字段

修改 `models.py` 和 `planner.load_keypoints()`，并为 YAML 验证补充测试。不要在 MTC builder
中直接解析 YAML。

### 新增段分类规则

修改 `planner.classify_segment()` 和 Segment 构造测试，再补 scheduler 对新 action 的处理。
scheduler 和 Stage 层都不应重新读取 `in_slot` 定义段分类规则。

### 新增任务 action 或 recipe

修改 `scheduler.py`，由 Segment action 生成新的 `TaskStep` 并装入 `TaskPlan`，然后在
`stages/compiler.py` 中增加对应的 StageSpec 编译逻辑。MTC 对象细节应放在
`mtc/task_builder.py`。

### 新增 MTC planner 或路径约束

修改 `mtc/task_builder.py`，不要把 MTC Python binding 引入 scheduler。

### 新增夹爪动作

优先增加 `gripper_profiles.yaml` profile。只有后端协议发生变化时才修改 `gripper.py`。

### 新增 ROS topic 或 service

放入 `runtime/` 或独立 ROS 节点，不要放进 Stage compiler。

## 11. 测试边界

当前测试重点覆盖：

- 关键点读取和段分类。
- Segment 链连续性及 scheduler 只消费 Segment action。
- 双臂调度顺序。
- StageSpec 编译、方向和让位距离。
- MTC planner 映射、anchor 路径约束和复合 `Merger` 构造。
- preparation/formal 统一索引、交互确认和同步下降。
- 夹爪 profile、安全限制和后端结果。
- readiness 所需关节与 controller topic。
- 逐 Stage fail-closed 行为。
- CLI、PreparationConfig 和 MTC builder 的默认值一致性。

常规验证命令：

```bash
source /opt/ros/humble/setup.bash
source /home/jerry/franka_ros2_ws/install/setup.bash

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q
ament_flake8 dual_fr3_trunking_mtc launch scripts test
ament_pep257 dual_fr3_trunking_mtc launch scripts test
```

Launch 文件可以在不启动机器人的情况下检查参数展开：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py --show-args
```

## 12. 已知架构边界

当前拆分已经隔离 Stage、MTC 和运行时职责，并完成
`Keypoint[] -> SegmentPlan[] -> TaskPlan -> 统一 MtcStageSpec[]` 的单向数据流。preparation
和正式任务已经共用 Stage IR、builder 和 executor；MoveIt 资源构造也已下沉到
`dual_fr3_moveit_config.moveit_resources`。仍有以下待改进点：

- `MtcStageSpec` 仍是统一的大型结构，不同 Stage 类型通过字符串和空字段区分。
- `stages/compiler.py` 仍集中包含所有 recipe，后续适合按 `straighten`、`seat_edge` 和
  `move_anchor` 拆分。
- 主进程同时管理 rclcpp MTC 节点和独立 rclpy publisher/夹爪上下文。
- Stage sequence 当前在规划/执行流程结束后发布；提前失败时不会发布。
- `seat_cable_on_edge` 尚未实现力控、接触检测或真实下压动作。

这些边界不影响当前模块职责，但在继续增加视觉、力控或更多走线 recipe 前应优先处理。
