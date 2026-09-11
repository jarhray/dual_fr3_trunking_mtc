# dual_fr3_trunking_mtc

后端统一通过 `simulation_backend` 选择，默认 `gazebo`；原 `use_gazebo`、
`use_fake_hardware` 和 `auto` 兼容逻辑已移除。
ManiSkill 后端使用 ManiSkill2 0.5.3 / SAPIEN 2.2.2，通过 `simulation_backend:=maniskill`
启动，保留当前 MTC 规划与执行流程。工作区 `.venv` 的安装脚本同时编译配套 MPM/Warp。
构建、运行和验证步骤见
[ManiSkill 使用说明](../dual_fr3_moveit_config/docs/maniskill.md)。

MTC 入口在 `simulation_backend:=maniskill` 时默认启用准备阶段线缆：两个夹爪
闭合成功后固定左手 USB、穿过右 TCP 滑孔，再执行双臂下降。运行
`ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py simulation_backend:=maniskill execute:=true`。
使用 `maniskill_cable:=false` 可关闭；`execute:=false` 不创建仿真线缆。
默认线径为适配现有研究手指孔的 2 mm，完整说明见
[MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md)。

`dual_fr3_trunking_mtc` 是一个面向双臂 FR3 线槽走线任务的 Python 优先规划包。

当前启动链、数据流、模块边界和扩展位置见
[ARCHITECTURE.md](ARCHITECTURE.md)。

计划新增的 D455 + SAM2 线缆视觉模块，其纯 Python 核心任务、离线测试、输入输出和
后续 ROS 2 适配边界见 [VISION_MODULE_HANDOFF.md](VISION_MODULE_HANDOFF.md)。

它依赖 `dual_fr3_moveit_config` 提供的双臂 FR3 MoveIt 场景、机器人模型、控制器和线槽碰撞体。本包当前关注的是任务层逻辑：根据线槽路径关键点生成“拉直/跟随/边缘卡线”等段级动作规划，并在 RViz 中发布可视化结果。

当前版本还不是完整执行闭环，也没有加入力控、视觉或真实插线细节。它先解决第一阶段问题：

```text
关键点 YAML
→ 根据 in_slot 判断每一段动作类型
→ 生成 SegmentPlan[]
→ scheduler 生成统一 TaskPlan
→ Stage compiler 从 TaskPlan 编译 preparation + formal 统一 StageSpec 程序
→ 发布 JSON 摘要、RViz marker 和 MTC stage sequence
```

## 设计思路

线槽走线任务不需要在每个中间位置都人工放点。更合适的做法是：

- 只在拐角、槽内/槽外变化点、边缘卡线点等位置放关键点
- 两个关键点之间由笛卡尔路径或障碍感知规划补齐
- 每个关键点用 `in_slot` 表示线当前是否处于线槽内
- 相邻两个关键点的 `in_slot` 是否变化，决定这一段的动作语义

段级规则如下：

```text
false -> false  拉直 / 槽外跟随
true  -> true   槽内跟随 / 槽内转向
false -> true   seat_edge / 把线卡到边缘上
true  -> false  seat_edge / 把线卡到边缘上
```

双臂分工：

- `leader`：始终抓住线的一端，负责锚定、牵引和障碍感知迁移
- `follower`：根据下一段语义执行拉直、跟随或边缘卡线动作

当前段级 recipe 会显式输出 `execution_order`：

```text
straighten:
  leader_move_to_segment_goal -> follower_follow_segment_path

seat_edge:
  leader_move_ahead_for_clearance -> follower_seat_cable_on_edge -> leader_hold_tension
```

当前用 follower 关节空间直接规划到卡线关键点近似 `seat_edge`。它能验证双臂调度和
关键点可达性，但还不包含下压、力控或接触检测，因此不等同于真机上的完整卡线动作。

## 包结构

```text
dual_fr3_trunking_mtc/
  config/
    keypoints.yaml              # 示例关键点配置
  dual_fr3_trunking_mtc/
    models.py                   # Keypoint / SegmentPlan / TaskPlan 数据模型
    planner.py                  # 关键点解析、段分类、段级 recipe 生成
    scheduler.py                # SegmentPlan[] -> TaskPlan 全局任务调度
    stages/
      specs.py                  # 可序列化的 MtcStageSpec
      compiler.py               # TaskPlan -> MtcStageSpec
    mtc/
      task_builder.py           # MtcStageSpec -> MTC Task
      executor.py               # 保留的逐阶段诊断执行器
      preparation_search.py     # 多准备关节姿态候选和完整后续路径预检
      planning.py               # 完整规划重试并保存成功解
      cached_execution.py       # 执行成功解，异常时重规划未完成部分
    runtime/
      config.py                 # launch/CLI/节点共享的默认配置
      stage_publisher.py        # stage sequence ROS 发布
    mtc_prototype.py            # 兼容入口和顶层流程编排
    preparation.py              # 准备 StageSpec 编译和交互确认
    readiness.py                # 启动就绪闸门
    gripper.py                  # 夹爪 profile 与后端控制
    markers.py                  # RViz marker 构造
    ros_node.py                 # ROS 2 节点
  launch/
    demo.launch.py              # 启动 dual_fr3_moveit_config 和本规划节点
    mtc_prototype.launch.py     # 启动 MoveIt 环境和 MTC 原型节点
  scripts/
    trunking_plan_node.py       # 可执行入口
    trunking_mtc_prototype.py   # MTC 原型可执行入口
  test/
    test_planner.py             # 纯 Python 规划逻辑测试
```

## 关键点配置

默认配置文件是：

```text
config/keypoints.yaml
```

示例：

```yaml
default_frame: left_fr3_link0
keypoints:
  - name: entry_0
    frame_id: left_fr3_link0
    position: [0.40, 0.00, 0.00]
    in_slot: false
    role: pull

  - name: slot_entry_2
    frame_id: left_fr3_link0
    position: [0.55, 0.02, 0.30]
    in_slot: true
    role: seat_edge
```

坐标系约定：

- 场景构建、碰撞体、线槽模型仍由 `dual_fr3_moveit_config` 放在 `world` 下
- 任务关键点、末端目标位姿、段级路径 recipe 默认使用左臂坐标系 `left_fr3_link0`
- 当前包不会在 `world` 和 `left_fr3_link0` 之间自动做 tf2 坐标转换
- 同一段的起点和终点必须处于同一个 `frame_id`，否则会直接报错，避免错误插值

字段说明：

- `default_frame`：默认任务坐标系，通常使用 `left_fr3_link0`
- `name`：关键点名称
- `frame_id`：该关键点所在坐标系
- `position`：关键点位置 `[x, y, z]`
- `in_slot`：线是否已经在线槽内
- `role`：人工标注的语义标签，目前主要用于阅读和调试

关键点不包含方向。夹爪 roll/pitch 是任务级工具姿态，yaw 由当前点与相邻点的
连线自动生成，再按每只机械臂的方向参数决定正反。`forward` 表示沿关键点索引递增
方向，`reverse` 表示朝相反方向。默认 leader 使用 `reverse`，即由下一个点指向
上一个点；follower 使用 `forward`，即由上一个点指向下一个点。YAML 中出现点级
`rpy` 会直接报错，避免位置数据携带隐藏姿态。当前版本只依赖 `in_slot` 做动作判断，
其他语义以后再扩展。

## 运行方式

先构建包：

```bash
colcon build --packages-select dual_fr3_trunking_mtc --symlink-install
source install/setup.bash
```

启动规划可视化 demo：

```bash
ros2 launch dual_fr3_trunking_mtc demo.launch.py
```

这个 launch 会同时启动：

- `dual_fr3_moveit_config/launch/demo.launch.py`
- `dual_fr3_trunking_mtc` 的 Python 规划节点 `trunking_plan_node.py`

它的用途是验证“关键点读取 + 任务调度 + RViz marker + ROS topic 输出”。也就是说，
它会发布小球、路径线、`plan_summary` 和 `task_schedule`，但不会创建 MTC task，
也不会调用 MTC planner。

也可以指定自己的关键点文件：

```bash
ros2 launch dual_fr3_trunking_mtc demo.launch.py \
  keypoints_file:=/absolute/path/to/keypoints.yaml
```

常用 launch 参数：

- `keypoints_file`：关键点 YAML 路径
- `simulation_backend`：选择 `gazebo`、`maniskill`、`fake` 或 `real`，默认 `gazebo`
- `fake_sensor_commands`：fake hardware 下是否启用 fake sensor commands，默认 `true`
- `use_rviz`：是否启动 RViz，默认 `true`
- `load_gripper`：是否加载 gripper，默认 `true`
- `ee_id`：末端执行器 ID，默认 `franka_hand`
- `left_robot_ip` / `right_robot_ip`：真实硬件 IP
- `task_frame`：关键点 YAML 未显式写 `default_frame` 时使用的任务坐标系，默认 `left_fr3_link0`
- `keypoint_marker_scale`：关键点小球尺寸，默认 `0.02`
- `marker_z_offset`：仅用于 RViz 调试显示的 z 方向抬高量，默认 `0.0`
- `publish_labels`：是否显示关键点名称标签，默认 `true`

启动 MTC 原型 demo：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  plan:=false \
  execute:=false
```

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=real \
  left_robot_ip:=192.168.1.2 \
  right_robot_ip:=192.168.2.2 \
  start_gripper:=true \
  plan:=true \
  execute:=true
```

这个 launch 会同时启动：

- `dual_fr3_moveit_config/launch/demo.launch.py`，由 `simulation_backend` 选择后端（默认 Gazebo）
- MTC 原型节点 `trunking_mtc_prototype.py`

它的用途是验证“scheduler 的 TaskPlan 能否被映射成 MoveIt Task Constructor stage”。
它不会启动 `trunking_plan_node.py`，也不会发布 RViz 关键点小球；它会自己读取
同一个 `keypoints.yaml`，在进程内部调用 planner/scheduler，然后创建 MTC task。

`mtc_prototype.launch.py` 额外做了几件 `demo.launch.py` 没有做的事：

- 通过 `dual_fr3_moveit_config.moveit_resources` 给 MTC Python 节点传入
  `robot_description`、`robot_description_semantic`、kinematics 和 OMPL 参数；
  MoveGroup 与 MTC 不再各自复制 xacro/YAML 构造逻辑
- 根据 task schedule 创建 MTC `CurrentState` 和 action primitive stage
- 发布 MTC stage sequence，作为后续执行模块的接口
- 启动 MoveIt demo 时给 `move_group` 额外加载
  `move_group/ExecuteTaskSolutionCapability`，这样 `execute:=true` 时
  MTC 的 `task.execute()` 才能连接到 `/execute_task_solution` action server
- 先启动 `trunking_readiness_gate.py`，等待左右轨迹控制器 Action、左右
  `controller_state`、完整且新鲜的 14 个机械臂关节状态和 MoveGroup；全部通过后才启动
  MTC 节点，不再依赖固定启动延时
- readiness 是一次性启动闸门，不参与 MTC 规划或后续 stage 调度。真机还等待
  `/move`、`/grasp`，并在 `execute:=true` 时依次 Homing 两个夹爪；Gazebo/ManiSkill/fake
  则等待各自唯一的 `GripperCommand` 接口。任一检查失败都不会启动 MTC
- 启用 preparation 时，在准备动作开始前搜索不同准备关节姿态，并预检准备到正式任务结束的全部路径。找到成功解后，`execute:=true` 按阶段直接执行这份完整 solution；只有执行异常才从实际状态重规划未完成部分。含准备动作或夹爪操作时要求 `execute_stage_by_stage:=true`
- 通过 `simulation_backend:=gazebo` 可切换到 `dual_fr3_moveit_config/launch/gazebo.launch.py`
  ，让 RViz 直接跟随 Gazebo 里的实际运动显示

当前手动规定的 action primitive 是：

```text
preparation phase（preparation.py 编译为统一 MtcStageSpec）：
  MoveTo(PipelinePlanner / OMPL RRTConnect): leader 到第二个关键点上方 0.05 m
  MoveTo(PipelinePlanner / OMPL RRTConnect): follower 到第一个关键点上方 0.05 m
  键盘确认后 GripperProfile(cable_tip): leader 明确执行配置中的 Move/Grasp
  键盘确认后 GripperProfile(cable_tip): follower 明确执行配置中的 Move/Grasp
  键盘确认后 Merger[MoveRelative(CartesianPath) x 2]: 双臂同步向下 0.05 m

seat_edge:
  InfoOnly(seat_cable_on_edge): 先执行物理卡线动作（当前为占位，不发运动命令）
  可选 GripperOperation: 从目标关键点 metadata.gripper 读取 actor/profile/override
  MoveTo(PoseStamped, CartesianPath): leader 保持 TCP xyz 不变，原地转向路径切向
  MoveRelative(CartesianPath): leader 沿卡线目标点的路径切向前移 10 cm
  MoveTo(PoseStamped, CartesianPath): follower 保持 TCP xyz 不变，按配置的路径方向转向
  MoveTo(JointInterpolationPlanner): follower 直接规划到卡线目标点

straighten:
  MoveTo(PoseStamped, CartesianPath): 夹爪保持 TCP xyz 不变，按配置的路径方向转向
  MoveRelative(CartesianPath): Cartesian 移动到下一个关键点

move_anchor:
  MoveTo(PipelinePlanner / OMPL RRTConnect): leader 绕障规划到下一个 anchor 关键点
```

准备动作和后续目标都不读取点级方向：roll/pitch 使用任务级工具姿态，yaw 根据路径
几何及当前 actor 的 `orientation_direction` 生成。后续独立转向使用当前关键点 xyz 和目标 yaw 的 PoseStamped，
由 CartesianPath 保持转向过程中 TCP 的 xyz 不变。规划器分别跟踪 leader/follower 当前 yaw，只执行到目标方向所需的最短
转角，已经对准的转向 stage 不发送轨迹。`forward` 沿关键点递增方向，`reverse`
朝相反方向；默认 leader=`reverse`、follower=`forward`。leader 的 10 cm 让位只在 `seat_edge` 中出现，
`straighten` 和 `move_anchor` 的动作逻辑保持不变。

它和 `dual_fr3_moveit_config/launch/demo.launch.py` / `gazebo.launch.py` 的关系是：

```text
dual_fr3_trunking_mtc/demo.launch.py
  -> include dual_fr3_moveit_config/launch/demo.launch.py
  -> start trunking_plan_node.py

dual_fr3_trunking_mtc/mtc_prototype.launch.py
  -> include dual_fr3_moveit_config/launch/demo.launch.py
     -> simulation_backend 选择 Gazebo（默认）、ManiSkill、fake 或真机
  -> start trunking_readiness_gate.py
  -> readiness checks passed 后 start trunking_mtc_prototype.py
```

所以它们是两个并列入口，不是互相调用关系。一般不要在两个终端同时启动
`demo.launch.py` 和 `mtc_prototype.launch.py`，因为二者都会启动一套
`dual_fr3_moveit_config` demo，容易重复启动 `move_group`、控制器和 RViz。

实际使用建议：

- 只想看关键点小球、路径线、调度文本：用 `demo.launch.py`
- 想验证 MTC stage 是否生成、后续执行模块应该接什么接口：用 `mtc_prototype.launch.py`
- 想在 MTC 原型阶段同时看 Gazebo 里的实际运动：用 `mtc_prototype.launch.py simulation_backend:=gazebo`
- 当前阶段做“方案 B / MTC 原型”：优先用 `mtc_prototype.launch.py`
- 上真实机械臂前：保持 `execute:=false`，先确认 stage sequence 和 MTC plan 结果

规划失败时，会输出 `[planning-failure]` 阶段摘要：每个阶段包含编号、名称、
准备/正式阶段、actor、规划组、规划器、关键点及目标坐标。状态含义：

- `HAS_SOLUTION`：该阶段已有局部候选解，不代表整个任务成功或已执行。
- `FAILED`：该阶段只有失败候选；后续行给出实际原因。
- `NO_RESULT`：没有返回候选，可能受前序阶段或超时影响，尚不能判定该阶段失败。
- `SKIPPED`：说明性阶段或不产生 MTC 运动的夹爪 hold 操作。

失败摘要和路径校验拒绝信息显示为红色，重试提示显示为黄色。launch 默认启用颜色；
单独运行脚本时自动根据终端判断。保存纯文本日志可在启动前设置
`TRUNKING_LOG_COLOR=never` 或 `NO_COLOR=1`，强制彩色可设 `TRUNKING_LOG_COLOR=always`。
这些设置控制本包 Python 日志，MoveIt/FCL 原生日志仍使用自己的格式。

`PATH_LENGTH_LIMIT` 会同时给出实际 TCP 路径长度、允许长度和直线距离；
`CARTESIAN_PATH_DEVIATION` 给出实际偏差及容差。这些原因也保存在 MTC solution comment
中。`INVALID_MOTION_PLAN` 会检查保留下来的失败轨迹，输出发现的碰撞轨迹点和
TCP 高度超限；旧版 MoveIt Python 无法读取碰撞对象名称时，通过相邻 FCL 原生日志输出。
诊断只检查存储轨迹的采样点，最多 4096 点，不改变规划场景、轨迹或约束；
未定位到原因时会明确提示查看原生日志，不将其解释为无碰撞。
搜索超时或重试耗尽时，`[planning-summary]` 汇总各阶段在多少次尝试中出现过各类拒绝。
同时按阶段顺序显示结果统计，范围为本次搜索中失败的完整任务规划尝试：

- `with_result=A/B`：该阶段纳入统计的 B 次尝试中，有 A 次返回了候选结果。
- `has_solution`：至少返回一个局部可行解的尝试次数，即使同时存在被拒绝的分支。
- `failed_only`：只返回失败候选的尝试次数。
- `no_result`：没有返回候选的尝试次数；可能受前序阶段或超时影响，不计作失败。
- `skipped` / `unavailable`：无运动阶段或无法读取状态的次数，不计入 `with_result`。
- `failed_only/with_result`：只在已有结果的尝试中计算失败比例；无结果时显示 `n/a`。

不要用阶段的拒绝次数除以总尝试次数来比较规划难度：前面的失败会减少后续阶段被规划
的机会。同一次尝试也可能产生多种拒绝原因，原因计数之和不一定等于失败尝试数。

`mtc_prototype.launch.py` 常用参数：

- `keypoints_file`：关键点 YAML 路径
- `task_frame`：关键点 YAML 未显式写 `default_frame` 时使用的任务坐标系
- `initial_leader_index`：任务调度器的 leader 初始关键点 index，默认 `1`
- `initial_follower_index`：任务调度器的 follower 初始关键点 index，默认 `0`
- `preparation_enabled`：是否在统一 Stage 程序前部加入 preparation phase，默认 `true`
- `preparation_height`：leader/follower 初始悬停高度及随后同步下降距离，默认 `0.05` m
- `preparation_interactive`：夹爪闭合和双臂下降前是否等待键盘确认，默认 `true`
- `preparation_ik_candidates`：每臂最多保留的不同准备 IK 解数，默认 `8`
- `preparation_ik_attempts`：每臂最多尝试的 IK 初值数，默认 `80`
- `preparation_ik_timeout`：每个 IK 初值的求解时间上限，默认 `0.05` 秒
- `preparation_min_joint_distance`：同臂候选之间的最小关节向量欧氏距离，默认 `0.3` rad，用于去重
- `preparation_candidate_attempts`：遍历全部候选组合的最多轮数，默认 `2`；每组遇到 OMPL 失败时还受 `planning_attempts` 控制
- `preparation_search_timeout`：包含场景获取、IK 采样和路径预检的搜索时间预算，默认 `180.0` 秒，在原生求解调用之间检查，当前调用可能超过剩余预算
- `gripper_profiles_file`：夹爪 profile 和不可突破的安全上限配置，默认
  `config/gripper_profiles.yaml`
- `preparation_leader_gripper_profile`：leader 准备抓取 profile，默认 `cable_tip`
- `preparation_follower_gripper_profile`：follower 准备抓取 profile，默认 `cable_tip`
- `leader_group`：leader 使用的 MoveIt planning group，默认 `left_fr3_arm`
- `follower_group`：follower 使用的 MoveIt planning group，默认 `right_fr3_arm`
- `leader_ik_frame`：leader TCP / IK frame，默认 `left_fr3_hand_tcp`
- `follower_ik_frame`：follower TCP / IK frame，默认 `right_fr3_hand_tcp`
- `leader_orientation_direction`：leader TCP yaw 相对路径的方向，`forward` 沿关键点递增方向、`reverse` 反向，默认 `reverse`
- `follower_orientation_direction`：follower TCP yaw 相对路径的方向，取值含义同上，默认 `forward`
- `cartesian_step_size`：笛卡尔规划采样步长，默认 `0.005` m
- `cartesian_jump_threshold`：相邻 IK 解的关节距离相对于全程平均值的检测倍数，默认 `2.0`，必须为有限值且大于 `1.0`；它不是关节速度上限。原默认 `1.5` 会将当前 FR3 场景中一条连续直线路径在约 90% 处截断，实际 TCP 偏离仍由独立 FK 检查拒绝
- `cartesian_path_tolerance`：笛卡尔直线段 TCP 偏离起终点线段、或原地转向 TCP 偏离起始位置的上限，默认 `0.01` m
- `motion_velocity_scaling`：MTC 运动速度缩放，默认 `0.2`
- `motion_acceleration_scaling`：MTC 运动加速度缩放，默认 `0.2`
- `anchor_max_path_z`：`direct_move_to_next_anchor` 的 OMPL 路径中，leader TCP 在关键点坐标系下允许的最大 z，默认 `0.6` m
- `anchor_max_path_length_ratio`：该阶段 TCP 累计路程与实际起始 TCP 到目标关键点的三维直线距离之比上限，默认 `1.5`，必须为有限值且不小于 `1.0`
- `leader_lead_distance`：`seat_edge` 前 leader 沿路径切向让位的距离，默认 `0.10` m
- `tool_roll`：任务级 TCP roll，默认 `π`，使夹爪朝向工作面
- `tool_pitch`：任务级 TCP pitch，默认 `0.0`
- `trajectory_execution_duration_scaling`：`move_group` 执行 watchdog 的时长放宽系数，默认 `10.0`
- `trajectory_execution_goal_margin`：`move_group` 执行 watchdog 的固定时间余量，默认 `5.0`
- `plan`：是否调用 MTC `task.plan()`，默认 `true`
- `execute`：是否执行 MTC solution，默认 `false`
- `planning_attempts`：完整规划或恢复规划的尝试次数上限，默认 `10`。准备候选搜索中，如果失败阶段均为 PipelinePlanner，则用该上限对当前组合重试；Cartesian 失败则换下一组。全程仍受 `preparation_search_timeout` 约束，与单次 OMPL 请求内部的尝试次数不同
- `execution_replan_attempts`：执行过程中，已知失败阶段后的恢复重规划次数上限，默认 `2`，`0` 表示不自动恢复；仅 `execute_stage_by_stage:=true` 生效
- `execute_stage_by_stage`：默认 `true`，按阶段执行选定完整解中的原轨迹，不再逐阶段重新规划
- `start_gripper`：是否启动左右夹爪驱动；真机执行包含夹爪 stage，因此必须为 `true`
- `readiness_timeout`：等待控制器、Action Server 和机器人状态的总超时，默认 `60.0` 秒
- `state_max_age`：`/joint_states` 和控制器状态允许的最大消息年龄，默认 `0.5` 秒
- `home_grippers_before_execute`：真机执行前是否自动依次 Homing 两个夹爪，默认 `true`
- `grippers_homed`：关闭自动 Homing 时，由操作者显式确认两个夹爪已经手动 Homing；默认 `false`
- `mtc_keep_alive_sec`：MTC 节点完成后继续保留 topic publisher 的时间，默认 `30.0`
- `simulation_backend`：统一选择执行后端，支持 `gazebo`、`maniskill`、`fake`、`real`，默认 `gazebo`
- `gz_args`：传给 Gazebo 的参数，默认 `empty.sdf -r`
- `gazebo_effort`：是否在 Gazebo URDF 中暴露 effort command interface，默认
  `false`；MoveGroup 和 MTC 节点共用该值

例如，在 Gazebo 中执行默认准备流程和正式任务：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=gazebo \
  plan:=true \
  execute:=true \
  preparation_height:=0.15 \
  preparation_interactive:=true
```

两次上方定位会自动规划并执行。随后终端会依次提示闭合 leader、闭合 follower
以及双臂同步下降；每次按 Enter 继续，输入 `q` 中止。终端没有可用标准输入时，
交互模式会安全中止，不会自动执行确认动作。设置
`preparation_interactive:=false` 可关闭这三个确认。

夹爪执行由 `dual_fr3_trunking_mtc/gripper.py` 统一封装。profile 中的 `width`
始终表示两指总开口：真机 backend 对 `move`/`grasp` 分别调用 Franka 官方 Action；
Gazebo/ManiSkill 使用 `/gripper_cmd`；fake hardware 使用 `/gripper_action`。后端在启动时由
`simulation_backend` 唯一确定，不会因某个 Action Server 暂时离线而
静默切换语义。profile 中的示例参数只用于初始联调，真机执行前必须按实际线径、
线槽尺寸和安全夹持力完成标定。

某个 `seat_edge` 需要夹爪操作时，可在目标关键点启用：

```yaml
metadata:
  gripper:
    enabled: true
    actor: follower
    profile: trunk_edge
    # action: grasp  # 可选，覆盖 profile action
    # width: 0.018   # 可选，仍受全局 safety 限制
```

`execute:=false` 时该操作以手指关节 `MoveTo` 表示，供碰撞检查和 RViz 预览；
`execute:=true execute_stage_by_stage:=true` 时由夹爪 backend 显式执行。包含这类
`seat_edge` 操作时不允许旧的整条 solution 一次性执行模式。

启用 preparation 时，预览和实际执行使用同一套“多准备姿态候选＋后续路径预检查”：

1. 在准备动作前捕获一次完整 PlanningScene，所有候选从该场景副本开始比较。
2. 固定原来的准备 TCP 位置和姿态，先用当前关节作为 IK 初值，再在关节限位内随机采样初值。
   用 FK 验证目标误差，并按关节距离去重；改变的是肘部、腕部等关节配置，不是关键点。
3. 对左右臂候选做对角顺序遍历：`L1/F1 → L1/F2 → L2/F1 → …`，先检查组合状态是否有效，
   再把选中 IK 解作为两个准备 MoveTo 的关节目标，规划全部 preparation 和 formal Stage。
   双臂下降、后续直线和转向、碰撞、高度、TCP 路程比例和轨迹偏差检查均包含在预检内。
4. 如果失败来自 OMPL 规划、碰撞后检或锚点路程检查，当前组合最多尝试 `planning_attempts` 次，
   避免把一次随机路径失败当成准备姿态不可用。Cartesian 后续检查失败时换下一组，
   整轮失败后在预算内重试。找到第一份完整成功解立即结束搜索，
   保留原 Task、完整 solution 和准备关节目标；所有候选失败或预算耗尽则停止，不启动准备动作。
5. `execute:=false` 仅发布该解。实际执行按阶段发送同一解中的原轨迹，在原确认点等待输入，
   不在执行准备动作时重新求 IK；双臂下降仍执行 MTC `Merger` 合并后的同步轨迹。

默认每臂最多 8 个候选，即最多 64 组；这版实现选择首个完整可行解，不搜索最优解，
也不保证有限次采样一定找到已有的可行姿态。搜索时间预算可能先于遍历完成而耗尽。
readiness 中既有的夹爪 Homing 仍发生在 MTC 节点启动之前，不属于这里的准备 Stage。

`test/test_fr3_cartesian_regression.py` 使用本包场景的实际 FR3、研究夹爪和线槽碰撞模型，
复现 `corner_1 → corner_2` 在旧 `cartesian_jump_threshold=1.5` 下的 `0.901639` 截断，
验证 `2.0` 下直线、原地转向和下一段直线全部通过。相对阈值基于关节增量平均值，
路径末端增量逐渐增大不等同于关节速度超限。这个离线回归把另一臂停在 ready，
完整双臂任务仍必须通过候选搜索及所有碰撞检查。

例如，把 anchor 阶段整个 OMPL 路径中的 leader TCP 高度限制为 `0.15 m`：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  anchor_max_path_z:=0.15
```

该高度使用 anchor 关键点的 `frame_id`（当前为 `left_fr3_link0`）测量，只约束
leader TCP 原点，不会错误地要求机械臂的所有连杆都低于这个高度。起点和终点本身也必须
满足该限制，否则 OMPL 不会找到路径。

锚点迁移同时默认限制 `TCP 路程 <= 1.5 × 起点到目标点的直线距离`。
起点来自该阶段规划轨迹的实际起始状态，不使用 YAML 中的名义起始关键点；
目标关键点先转换到机器人模型坐标系，因此前面的让位动作以及逐 stage 重新规划都能
正确计算上限。可用 `anchor_max_path_length_ratio:=1.5` 调整比例。

长度检查在 MTC 阶段内部执行，全链预览和逐 stage 执行都会生效。FR3 关节轨迹按
不超过 `0.005 rad` 的关节步长插值后，通过正运动学累计 TCP 平移距离，属于数值近似；
比较时只允许 `1e-6 m` 的数值误差，不包含原地转动角度。起终点重合时也不会额外
放宽上限。日志会显示实际路程、直线距离和长度上限。

笛卡尔段额外启用关节跳变检测，要求完整路径（`min_fraction=1.0`）。跳变导致的
部分路径不会作为成功解执行。对时间参数化后的轨迹，以不超过 `0.005 rad` 的关节
步长插值并做正运动学检查：直线移动检查到有限起终点线段的距离，原地转向检查
到起始 TCP 位置的距离；默认超过 `10 mm` 就拒绝该候选。正常规划和异常恢复都使用
这些检查，日志包含 `Cartesian TCP line/in-place deviation ... accepted/REJECTED`。
该检查是对规划轨迹的数值检查，不包含控制器样条插值和真机跟踪误差；也不限制
TCP 位置基本不变时肘部、腕部的累计运动。OMPL 锚点迁移及关节插值的入槽段仍按
各自规划语义执行，不应用直线位置限制。
超长或无法检查的解会标为失败，不会进入可执行解；重试受准备候选预算或 `planning_attempts` 限制，
不会自动放宽比例。现有高度与碰撞检查仍保留。

如需让 leader 恢复旧版的正向朝向，可运行：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  leader_orientation_direction:=forward
```

调试阶段推荐：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  plan:=false \
  execute:=false \
  mtc_keep_alive_sec:=120.0
```

如果你想直接在 RViz 里看 Gazebo 跑起来的效果：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=gazebo \
  plan:=true \
  execute:=true \
  motion_velocity_scaling:=0.1 \
  motion_acceleration_scaling:=0.1
```

Gazebo 的位置控制响应比规划轨迹慢时，可以继续增大上述两个 watchdog 参数；
它们只决定 MoveIt 等待控制器完成的时间，不会提高机械臂速度。真机首次验证仍应使用
`execute:=false`，确认规划后再使用更保守的真机执行超时参数。

## 逐段执行诊断

最简单的逐动作运行方式是：

```bash
source install/setup.bash
ros2 run dual_fr3_trunking_mtc trunking_step_by_step.py
```

脚本会自动启动 `dual_fr3_moveit_config gazebo.launch.py`，等待右臂控制器开始发布
状态后再显示动作列表；退出脚本时会关闭它启动的 Gazebo。Gazebo 输出保存在
`/tmp/dual_fr3_trunking_gazebo.log`。

脚本只调用 `DualFR3LinearController` 已有的 `move_to_ompl()` 和
`move_to_cartesian()`。按回车执行当前动作，输入 `s` 跳过，输入 `q` 退出；使用
`--start-step 6` 可以直接从指定编号开始，前提是机械臂已经处于该动作要求的起始位姿。
逐动作脚本和 `trunking_segment_executor.py` 都支持同名参数
`--leader-orientation-direction`、`--follower-orientation-direction`。

当 MTC 能规划完整 solution、但整条轨迹执行失败时，可以绕过 MTC 的整任务执行接口，
直接复用 `dual_fr3_moveit_config/scripts/fr3_controller.py` 和
`fr3_controller_lin.py` 的 MoveGroup/Cartesian 控制接口逐段运行。这个诊断器不会修改
`dual_fr3_moveit_config`。

终端 1 启动 Gazebo：

```bash
ros2 launch dual_fr3_moveit_config gazebo.launch.py
```

终端 2 source 工作区后先查看编号：

```bash
source install/setup.bash
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py --list
```

完整逐段执行并在第一处失败时停止：

```bash
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py --confirm-each
```

只复现一个 stage，例如固定 xyz 转向：

```bash
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py \
  --start-stage 13 --stop-stage 13
```

从某一步继续执行：

```bash
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py --start-stage 13
```

当前默认任务的机械臂运动顺序为：

```text
00 follower: close gripper                            Gripper
01 leader:   close gripper                            Gripper
02 follower: current -> corner_1                       OMPL
03 leader:   current -> corner_2                       OMPL
04 leader:   corner_2 -> entry_5                       OMPL
06 follower: corner_1 -> corner_2                      Cartesian
08 follower: corner_2 -> corner_4                      Cartesian
09 follower: seat cable on final edge                 Placeholder
```

每段都会打印执行前/后的 TCP xyz/rpy、调用方式、目标位姿，以及位置和四元数方向误差。
MoveIt 返回失败，或者 action 返回成功但实际 TCP 没到目标，都会标记该 stage 为
`FAILED`。`--skip-gripper` 可暂时排除夹爪 action server，`--plan-only` 可只规划选中的
单个 stage。诊断器默认禁止 Cartesian 路径自动退回 OMPL，以免掩盖真正失败的直线段；
需要与原脚本的自动回退行为对照时，可以添加 `--allow-cartesian-fallback`。

诊断器的关节空间段默认明确使用 Gazebo MoveIt 配置中存在的
`RRTConnectkConfigDefault`，而不是现有控制脚本硬编码的 Pilz `PTP`。可以通过
`--ompl-planner-id` 和 `--planning-time` 调整。

关节空间段会把 MoveGroup 规划和 `/execute_trajectory` 执行拆成两个请求，分别打印
`planning result` 和 `execution result`，用于区分路径规划失败与控制器执行失败。

逐段诊断器会同时等待左右控制器状态，并自动兼容 Gazebo 的旧话题和真机的命名空间
话题。如果任一侧状态没有更新，诊断器会立即报错，不再把控制循环停滞误报成某个
运动 stage 超时。`--allow-no-controller-state` 只保留作特殊诊断旁路，正常真机执行
不应使用。

当只想验证 MTC 是否能尝试规划，但绝不执行：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  plan:=true \
  execute:=false \
  mtc_keep_alive_sec:=120.0
```

如果要在 RViz 中看执行效果，先使用 fake hardware：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=fake \
  fake_sensor_commands:=true \
  use_rviz:=true \
  plan:=true \
  execute:=true \
  mtc_keep_alive_sec:=120.0
```

这个命令先搜索准备姿态，并在执行前预检包含准备和正式动作的完整任务。
一旦得到有效解，就保存并执行这份解中相互连接的 stage 子轨迹，保留原关节轨迹及时间信息。
正常完成一个 stage 后直接执行已保存的下一段，不会重新调用规划器。
只有关闭 preparation 且不含夹爪 Stage 时，才允许 `execute_stage_by_stage:=false` 直接发送整份解，失败后停止。在 fake hardware 下，RViz 应该能看到双臂状态随控制器执行结果更新。真实硬件调试前
不要直接把 `execute` 打开；应先确认 `/dual_fr3_trunking_mtc_prototype/stage_sequence_text`
中的 stage 顺序、group、ik frame 和相对位移都符合预期。

默认执行保留 MoveIt 的起点校验和执行监控。只有返回明确的终止错误
（`INVALID_MOTION_PLAN`、`MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE`、
`CONTROL_FAILED`、`TIMED_OUT`）才会在 `execution_replan_attempts` 上限内恢复：
跳过已完成阶段，从新的 `CurrentState` 规划失败阶段及其后续阶段。
相对移动会使用成功解保存的绝对 TCP 终点，以 Cartesian `MoveTo` 继续到原目标，
避免中途停止后再次走完整相对位移。高度、长度和碰撞约束始终保留。
取消、通信异常、未知错误和夹爪失败会停止；已完成的夹爪动作不会重放。
准备阶段保留原确认点。准备动作出现可恢复错误时，同样重规划未完成后缀，并保持预检选中的准备关节目标；不会重新随机选择准备姿态。

真机 `execute:=true` 时默认会自动 Homing 两个夹爪，因此夹爪中必须为空。如果已经手动
完成 Homing，并且需要先放入线缆再开始任务，应使用：

```bash
home_grippers_before_execute:=false grippers_homed:=true
```

如果两个参数都为 `false`，就绪门会拒绝启动 MTC。

如果 RViz 中没有运动，先看终端里有没有：

```text
[mtc_prototype] planning succeeded on attempt ...; retaining this solution
[mtc_prototype] executing ... cached stages from the successful plan
```

并确认 MTC 执行 action server 存在：

```bash
ros2 action list | grep execute_task_solution
```

如果终端出现：

```text
Failed to connect to the 'execute_task_solution' action server
```

说明 `move_group` 没有加载 MTC 执行 capability。请使用
`dual_fr3_trunking_mtc mtc_prototype.launch.py` 启动，而不是单独启动
`dual_fr3_moveit_config demo.launch.py` 后再手动运行 MTC 节点；前者会自动给
`move_group` 传入 `move_group/ExecuteTaskSolutionCapability`。

如果只看到 `planning failed`，说明还没有产生可执行 solution，`execute:=true`
也不会让机械臂运动。当前 `straighten` 先用 `CartesianPath` 原地转向，
再用 `CartesianPath` 做平移；如果某个 Cartesian stage 无法完成，可以先检查当前
姿态、关键点间距和碰撞约束。

## ROS 接口

规划节点名：

```text
/dual_fr3_trunking_planner
```

发布的话题：

```text
/dual_fr3_trunking_planner/plan_summary
/dual_fr3_trunking_planner/task_schedule
/dual_fr3_trunking_planner/markers
```

`plan_summary` 是 JSON 字符串，包含关键点和段级动作分类。示意：

```json
{
  "segments": [
    {
      "index": 0,
      "start": "entry_0",
      "goal": "corner_1",
      "action": "straighten",
      "path_type": "cartesian",
      "execution_order": [
        "leader_move_to_segment_goal",
        "follower_follow_segment_path"
      ]
    },
    {
      "index": 1,
      "start": "corner_1",
      "goal": "slot_entry_2",
      "action": "seat_edge",
      "path_type": "hybrid",
      "execution_order": [
        "leader_move_ahead_for_clearance",
        "follower_seat_cable_on_edge",
        "leader_hold_tension"
      ]
    }
  ]
}
```

`task_schedule` 是全局双臂执行调度，不是单段分类。它描述 leader 和 follower
在整个任务中的推进逻辑。示意：

```json
{
  "task_schedule": [
    {
      "index": 0,
      "action": "seat_edge",
      "leader": {"mode": "move_ahead_for_clearance", "hold": "corner_1"},
      "follower": {"mode": "seat_edge", "from": "entry_0", "to": "corner_1"},
      "execution_order": [
        "leader_move_ahead_for_clearance",
        "follower_seat_cable_on_edge",
        "leader_hold_tension"
      ]
    },
    {
      "index": 1,
      "action": "move_anchor",
      "leader": {"mode": "move_to_anchor", "from": "corner_1", "to": "entry_5"},
      "follower": {"mode": "hold_position", "hold": "corner_1"},
      "execution_order": [
        "follower_hold_position",
        "leader_move_to_anchor",
        "leader_establish_tension"
      ]
    }
  ]
}
```

终端查看全局调度时，优先 echo 短文本话题：

```bash
ros2 topic echo /dual_fr3_trunking_planner/task_schedule \
  --qos-durability transient_local
```

`plan_summary` 保留完整 JSON，主要给程序消费。

MTC 原型输出：

```text
/dual_fr3_trunking_mtc_prototype/stage_sequence
/dual_fr3_trunking_mtc_prototype/stage_sequence_text
```

查看它映射出来的 MTC stage 序列：

```bash
ros2 topic echo /dual_fr3_trunking_mtc_prototype/stage_sequence_text \
  --qos-durability transient_local
```

服务：

```text
/dual_fr3_trunking_planner/replan
```

调用后会重新读取关键点文件并发布新的规划摘要和 marker：

```bash
ros2 service call /dual_fr3_trunking_planner/replan std_srvs/srv/Trigger {}
```

节点会默认开启 `auto_reload`，关键点文件发生变化时会自动重新加载。

## RViz 可视化

节点会发布 `visualization_msgs/MarkerArray`：

- 关键点 marker：显示每个关键点
- 段 marker：显示关键点之间的插值路径

颜色含义：

- 橙色关键点：`in_slot: false`
- 绿色关键点：`in_slot: true`
- 蓝色段：`straighten`
- 红色段：`seat_edge`

## 当前实现边界

当前包已经实现：

- YAML 关键点读取
- `in_slot` 段级动作分类
- 段级 waypoint 插值
- leader/follower 全局任务调度
- 单向规划数据流：`Keypoint[] -> SegmentPlan[] -> TaskPlan -> MtcStageSpec[]`；
  scheduler 不再重复分类 Segment，Stage compiler 不再接收分离的关键点和 TaskStep
- 统一 Stage 程序：preparation 和 formal 共用 `MtcStageSpec`、MTC builder、
  逐 Stage executor 和 JSON/text 序列化；双臂同步下移由递归子 Stage 表示的
  MTC `Merger` 构造
- MTC 原型正式 stage 构建：`seat_edge` 展开为 leader Cartesian 让位 + follower turn/direct MoveTo，
  `straighten` 展开为
  Cartesian in-place turn + Cartesian move，`move_anchor` 展开为
  PipelinePlanner / OMPL RRTConnect MoveTo
- 终段 `seat_edge`：当 leader 已在最后关键点、follower 位于倒数第二个关键点时，
  只保留卡线占位动作，跳过现有两臂让位、转向和移动
- 规划摘要发布
- MTC stage sequence v2 JSON/text 发布，包含 `phase`、确认标记和复合子 Stage
- RViz marker 发布
- launch 集成 `dual_fr3_moveit_config`，并复用共享 MoveIt 资源构造器
- 每个 stage 都会带稳定的 `stage_key`，格式由 `step/action/actor/from_keypoint/to_keypoint/primitive`
  组合生成，排查问题时优先看这个，不要硬记 stage 编号

当前包暂未实现：

- 可用于真实硬件的完整双臂执行安全策略
- 夹爪状态反馈和完整开合策略
- 力控 / 视觉 / 接触检测
- 真实插线槽微搜索
- 线缆物理模型或闭环线状态估计

换句话说，现在这一版是“任务规划骨架”和“路径语义验证层”，不是最终执行控制器。

## 后续扩展方向

建议后续按这个顺序推进：

1. 在当前 `SegmentPlan` 上接入实际运动执行接口
2. 对 `straighten` 段实现 follower 笛卡尔跟随
3. 将当前 `seat_edge` 的路径跟随近似扩展为 pre-seat、short-push 和接触确认
4. 加入 leader/follower 的真实末端偏置
5. 接入夹爪动作
6. 再考虑力控、视觉或边缘卡线微搜索

如果后续坚持 Python 优先，可以先复用 `dual_fr3_moveit_config/scripts/fr3_controller.py` 和 `fr3_controller_lin.py` 中已有的 MoveIt action / Cartesian path 客户端。

## 开发提示

只验证 Python 逻辑时，可以运行：

```bash
PYTHONPATH=src/dual_fr3_trunking_mtc python3 -m py_compile \
  src/dual_fr3_trunking_mtc/dual_fr3_trunking_mtc/*.py
```

构建验证：

```bash
colcon build --packages-select dual_fr3_trunking_mtc --symlink-install
```
