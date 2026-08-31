# dual_fr3_trunking_mtc

`dual_fr3_trunking_mtc` 是一个面向双臂 FR3 线槽走线任务的 Python 优先规划包。

计划新增的 D455 + SAM2 线缆视觉模块，其纯 Python 核心任务、离线测试、输入输出和
后续 ROS 2 适配边界见 [VISION_MODULE_HANDOFF.md](VISION_MODULE_HANDOFF.md)。

它依赖 `dual_fr3_moveit_config` 提供的双臂 FR3 MoveIt 场景、机器人模型、控制器和线槽碰撞体。本包当前关注的是任务层逻辑：根据线槽路径关键点生成“拉直/跟随/边缘卡线”等段级动作规划，并在 RViz 中发布可视化结果。

当前版本还不是完整执行闭环，也没有加入力控、视觉或真实插线细节。它先解决第一阶段问题：

```text
关键点 YAML
→ 根据 in_slot 判断每一段动作类型
→ 生成 leader/follower 全局任务调度
→ 映射成 MTC 原型 primitive
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
    models.py                   # Keypoint / SegmentPlan 数据模型
    planner.py                  # 关键点解析、段分类、段级 recipe 生成
    scheduler.py                # leader/follower 全局任务调度
    mtc_prototype.py            # MTC 原型 primitive 构建和 stage sequence 发布
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
连线方向自动生成。YAML 中出现点级 `rpy` 会直接报错，避免位置数据携带隐藏姿态。
当前版本只依赖 `in_slot` 做动作判断，其他语义以后再扩展。

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
- `use_fake_hardware`：是否使用 fake hardware，默认 `true`
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
  use_fake_hardware:=false \
  left_robot_ip:=192.168.1.2 \
  right_robot_ip:=192.168.2.2 \
  start_gripper:=true \
  plan:=true \
  execute:=true
```

这个 launch 会同时启动：

- 默认情况下：`dual_fr3_moveit_config/launch/demo.launch.py`
- MTC 原型节点 `trunking_mtc_prototype.py`

它的用途是验证“scheduler 输出能否被映射成 MoveIt Task Constructor stage”。
它不会启动 `trunking_plan_node.py`，也不会发布 RViz 关键点小球；它会自己读取
同一个 `keypoints.yaml`，在进程内部调用 planner/scheduler，然后创建 MTC task。

`mtc_prototype.launch.py` 额外做了几件 `demo.launch.py` 没有做的事：

- 给 MTC Python 节点显式传入 `robot_description`、`robot_description_semantic`
  和 `kinematics.yaml`，让 `task.loadRobotModel(node)` 能加载双臂模型
- 根据 task schedule 创建 MTC `CurrentState` 和 action primitive stage
- 发布 MTC stage sequence，作为后续执行模块的接口
- 启动 MoveIt demo 时给 `move_group` 额外加载
  `move_group/ExecuteTaskSolutionCapability`，这样 `execute:=true` 时
  MTC 的 `task.execute()` 才能连接到 `/execute_task_solution` action server
- 先启动 `trunking_readiness_gate.py`，等待左右轨迹控制器 Action、左右
  `controller_state`、完整且新鲜的 14 个机械臂关节状态和 MoveGroup；全部通过后才启动
  MTC 节点，不再依赖固定启动延时
- 真机 `execute:=true` 时默认依次 Homing 两个夹爪；任一 Homing 失败都不会启动 MTC
- `execute:=true` 默认逐 executable stage 重新规划并执行，每段使用真实机器人状态作为下一段起点；设置 `execute_stage_by_stage:=false` 可复现旧的整条 solution 一次性执行方式
- 通过 `use_gazebo:=true` 可切换到 `dual_fr3_moveit_config/launch/gazebo.launch.py`
  ，让 RViz 直接跟随 Gazebo 里的实际运动显示

当前手动规定的 action primitive 是：

```text
initialize:
  MoveTo(left_fr3_hand/right_fr3_hand): follower、leader 在任何机械臂运动前先闭合夹爪
  MoveTo(JointInterpolationPlanner): follower 使用任务工具姿态和路径 yaw 到第一个关键点
  MoveTo(JointInterpolationPlanner): leader 使用任务工具姿态和路径 yaw 到第二个关键点

seat_edge:
  InfoOnly(seat_cable_on_edge): 先执行物理卡线动作（当前为占位，不发运动命令）
  MoveTo(PoseStamped, CartesianPath): leader 保持 TCP xyz 不变，原地转向路径切向
  MoveRelative(CartesianPath): leader 沿卡线目标点的路径切向前移 10 cm
  MoveTo(PoseStamped, CartesianPath): follower 保持 TCP xyz 不变，原地朝卡线目标点转向
  MoveTo(JointInterpolationPlanner): follower 直接规划到卡线目标点

straighten:
  MoveTo(PoseStamped, CartesianPath): 夹爪保持 TCP xyz 不变，原地转向下一个关键点
  MoveRelative(CartesianPath): Cartesian 移动到下一个关键点

move_anchor:
  MoveTo(JointInterpolationPlanner): leader 直接规划移动到下一个 anchor 关键点
```

初始化和后续目标都不读取点级方向：roll/pitch 使用任务级工具姿态，yaw 根据路径
几何生成。后续独立转向使用当前关键点 xyz 和目标 yaw 的 PoseStamped，
由 CartesianPath 保持转向过程中 TCP 的 xyz 不变。规划器分别跟踪 leader/follower 当前 yaw，只执行到目标方向所需的最短
转角，已经对准的转向 stage 不发送轨迹。普通点朝向下一点，最后一点沿上一段的进入
方向。leader 的 10 cm 让位只在 `seat_edge` 中出现，
`straighten` 和 `move_anchor` 的动作逻辑保持不变。

它和 `dual_fr3_moveit_config/launch/demo.launch.py` / `gazebo.launch.py` 的关系是：

```text
dual_fr3_trunking_mtc/demo.launch.py
  -> include dual_fr3_moveit_config/launch/demo.launch.py
  -> start trunking_plan_node.py

dual_fr3_trunking_mtc/mtc_prototype.launch.py
  -> default include dual_fr3_moveit_config/launch/demo.launch.py
  -> use_gazebo:=true 时 include dual_fr3_moveit_config/launch/gazebo.launch.py
  -> start trunking_readiness_gate.py
  -> readiness checks passed 后 start trunking_mtc_prototype.py
```

所以它们是两个并列入口，不是互相调用关系。一般不要在两个终端同时启动
`demo.launch.py` 和 `mtc_prototype.launch.py`，因为二者都会启动一套
`dual_fr3_moveit_config` demo，容易重复启动 `move_group`、控制器和 RViz。

实际使用建议：

- 只想看关键点小球、路径线、调度文本：用 `demo.launch.py`
- 想验证 MTC stage 是否生成、后续执行模块应该接什么接口：用 `mtc_prototype.launch.py`
- 想在 MTC 原型阶段同时看 Gazebo 里的实际运动：用 `mtc_prototype.launch.py use_gazebo:=true`
- 当前阶段做“方案 B / MTC 原型”：优先用 `mtc_prototype.launch.py`
- 上真实机械臂前：保持 `execute:=false`，先确认 stage sequence 和 MTC plan 结果

`mtc_prototype.launch.py` 常用参数：

- `keypoints_file`：关键点 YAML 路径
- `task_frame`：关键点 YAML 未显式写 `default_frame` 时使用的任务坐标系
- `initial_leader_index`：任务调度器的 leader 初始关键点 index，默认 `1`
- `initial_follower_index`：任务调度器的 follower 初始关键点 index，默认 `0`
- `align_initial_poses`：执行任务最开始是否先把 follower/leader 直接规划到初始关键点，默认 `true`
- `leader_group`：leader 使用的 MoveIt planning group，默认 `left_fr3_arm`
- `follower_group`：follower 使用的 MoveIt planning group，默认 `right_fr3_arm`
- `leader_ik_frame`：leader TCP / IK frame，默认 `left_fr3_hand_tcp`
- `follower_ik_frame`：follower TCP / IK frame，默认 `right_fr3_hand_tcp`
- `motion_velocity_scaling`：MTC 运动速度缩放，默认 `0.2`
- `motion_acceleration_scaling`：MTC 运动加速度缩放，默认 `0.2`
- `leader_lead_distance`：`seat_edge` 前 leader 沿路径切向让位的距离，默认 `0.10` m
- `tool_roll`：任务级 TCP roll，默认 `π`，使夹爪朝向工作面
- `tool_pitch`：任务级 TCP pitch，默认 `0.0`
- `trajectory_execution_duration_scaling`：`move_group` 执行 watchdog 的时长放宽系数，默认 `10.0`
- `trajectory_execution_goal_margin`：`move_group` 执行 watchdog 的固定时间余量，默认 `5.0`
- `plan`：是否调用 MTC `task.plan()`，默认 `true`
- `execute`：是否执行 MTC solution，默认 `false`
- `start_gripper`：是否启动左右夹爪驱动；真机执行包含夹爪 stage，因此必须为 `true`
- `readiness_timeout`：等待控制器、Action Server 和机器人状态的总超时，默认 `60.0` 秒
- `state_max_age`：`/joint_states` 和控制器状态允许的最大消息年龄，默认 `0.5` 秒
- `home_grippers_before_execute`：真机执行前是否自动依次 Homing 两个夹爪，默认 `true`
- `grippers_homed`：关闭自动 Homing 时，由操作者显式确认两个夹爪已经手动 Homing；默认 `false`
- `mtc_keep_alive_sec`：MTC 节点完成后继续保留 topic publisher 的时间，默认 `30.0`
- `use_gazebo`：是否切换到 Gazebo 版本的 MoveIt 启动文件，默认 `false`
- `gz_args`：传给 Gazebo 的参数，默认 `empty.sdf -r`

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
  use_gazebo:=true \
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
  use_fake_hardware:=true \
  fake_sensor_commands:=true \
  use_rviz:=true \
  plan:=true \
  execute:=true \
  mtc_keep_alive_sec:=120.0
```

这个命令会让 MTC 原型先规划，然后默认按 executable stage 逐段重新规划并调用
`task.execute()`。每段完成后下一段从真实机器人状态开始；如需复现旧的整条 solution
执行方式，增加 `execute_stage_by_stage:=false`。在 fake hardware 下，RViz 应该能看到双臂状态随控制器执行结果更新。真实硬件调试前
不要直接把 `execute` 打开；应先确认 `/dual_fr3_trunking_mtc_prototype/stage_sequence_text`
中的 stage 顺序、group、ik frame 和相对位移都符合预期。

默认逐 stage 执行是 fail-closed：任一 stage 规划失败、执行返回失败或抛出异常时，
立即终止整个后续 stage 序列，不会继续使用理想状态执行下一段。

真机 `execute:=true` 时默认会自动 Homing 两个夹爪，因此夹爪中必须为空。如果已经手动
完成 Homing，并且需要先放入线缆再开始任务，应使用：

```bash
home_grippers_before_execute:=false grippers_homed:=true
```

如果两个参数都为 `false`，就绪门会拒绝启动 MTC。

如果 RViz 中没有运动，先看终端里有没有：

```text
[mtc_prototype] planning succeeded with ...
[mtc_prototype] executing ... MTC stages with real-state replan between stages
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
- MTC 原型 stage 构建：开头先展开为 follower/leader close-gripper，随后可选 `initialize` 直接规划到初始关键点；
  `seat_edge` 再展开为 leader Cartesian 让位 + follower turn/direct MoveTo，
  `straighten` 展开为
  Cartesian in-place turn + Cartesian move，`move_anchor` 展开为
  JointInterpolationPlanner direct MoveTo
- 终段 `seat_edge`：当 leader 已在最后关键点、follower 位于倒数第二个关键点时，
  只保留卡线占位动作，跳过现有两臂让位、转向和移动
- 规划摘要发布
- MTC stage sequence JSON/text 发布
- RViz marker 发布
- launch 集成 `dual_fr3_moveit_config`
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
