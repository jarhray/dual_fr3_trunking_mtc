# dual_fr3_trunking_mtc 交接文档

> 最后更新：2026-08-31。`dual_fr3_moveit_config` 中真机控制器命名空间改造造成的
> Gazebo 回归已经修复；后续优先复核真机夹爪限位并继续逐 stage 验证。

## 1. 项目定位

这是双 FR3 机械臂在线槽/线缆路径上协同走线的任务层包，不是完整的真机接触操作程序。

- `leader` 固定为左臂：`left_fr3_arm`。
- `follower` 固定为右臂：`right_fr3_arm`。
- 默认任务坐标系：`left_fr3_link0`。
- 工作空间为 `/home/jhr/ws_franka`，MoveIt/MTC 系统依赖由用户自行安装。
- `dual_fr3_trunking_mtc` 和 `dual_fr3_moveit_config` 当前都有未提交修改；不要假设
  依赖包已经回滚，也不要使用 `git reset --hard` 或覆盖用户修改。

## 2. 当前最重要结论

当前真机启动链已经能够连接两台 FR3、两侧夹爪、MoveGroup 和左右轨迹控制器。
Gazebo 已恢复为单 controller manager，并使用独立的 MoveIt controller 映射；
左右臂和两侧物理仿真夹爪均已完成 Action 闭环验证。

目前仍保留逐动作脚本：

```bash
source /opt/ros/humble/setup.bash
source /home/jhr/ws_franka/install/setup.bash
ros2 run dual_fr3_trunking_mtc trunking_step_by_step.py
```

脚本会启动 `dual_fr3_moveit_config gazebo.launch.py`，然后复用
`DualFR3LinearController`、`move_to_ompl()` 和 `move_to_cartesian()`。
控制器助手会优先使用真机的 `gripper_action`，在 Gazebo 下自动选择
`gripper_cmd`。

- 回车：执行。
- `s`：跳过。
- `q`：退出。
- 如果 Gazebo 已经启动：`ros2 run dual_fr3_trunking_mtc trunking_step_by_step.py --no-gazebo`。

逐动作脚本、逐 stage 诊断器和 MTC 入口的 leader 默认让位距离统一为 `0.10` m。逐动作脚本也支持通过 `--leader-lead-distance` 覆盖。

MTC 入口适合看 stage 顺序和整体规划，不应当作为当前可靠的完整执行器：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  use_gazebo:=true plan:=true execute:=false
```

`execute:=true` 默认按 executable stage 逐段重新规划和执行，每段完成后由 `CurrentState` 读取真实机器人状态。这样可以避免把整条理想化轨迹一次性交给 `move_group`，降低双臂中间姿态碰撞和状态漂移导致的失败。若需复现旧行为，可显式设置 `execute_stage_by_stage:=false`；`planning succeeded` 仍只代表生成了 solution，不代表整条轨迹已经安全执行。

MTC 启动入口已经增加 fail-closed 就绪门：

- 等待 `/move_action` 和左右 `FollowJointTrajectory` Action Server。
- 等待左右 `controller_state`。
- 要求 `/joint_states` 包含 14 个机械臂关节，且消息年龄不超过
  `state_max_age`。
- `execute:=true` 时等待 `/execute_task_solution` 和夹爪 Action。
- 真机默认依次 Homing 两个夹爪；任一失败都不启动 MTC。
- 已删除固定 `TimerAction(6s)`，只在就绪进程以返回码 0 退出后启动 MTC。
- 任一 stage 规划失败、执行失败或抛出异常，立即终止所有后续 stage。

## 3. 数据流和主要文件

```text
config/keypoints.yaml
  -> planner.py       关键点解析、段分类、路径方向
  -> scheduler.py     leader/follower 调度
  -> mtc_prototype.py MTC stage 和 task
  -> move_group/controllers
```

主要文件：

- `config/keypoints.yaml`：点位置、frame、`in_slot`、角色语义。
- `dual_fr3_trunking_mtc/planner.py`：解析、分类、路径方向、summary。
- `dual_fr3_trunking_mtc/scheduler.py`：动作调度。
- `dual_fr3_trunking_mtc/mtc_prototype.py`：`MtcStageSpec` 和 MTC task。
- `dual_fr3_trunking_mtc/readiness.py`：双臂控制器、Action、14 关节状态和
  夹爪 Homing 的 fail-closed 就绪门。
- `dual_fr3_trunking_mtc/segment_executor.py`：逐 stage 诊断执行器。
- `scripts/trunking_step_by_step.py`：最简单的逐动作执行器。
- `scripts/trunking_mtc_prototype.py`：MTC 脚本入口。
- `scripts/trunking_readiness_gate.py`：就绪门可执行入口。
- `dual_fr3_trunking_mtc/ros_node.py`：关键点、线段、summary、schedule 发布。
- `launch/demo.launch.py`：MoveIt demo 加关键点可视化。
- `launch/mtc_prototype.launch.py`：MoveIt/Gazebo 加 MTC 节点。

与当前 Gazebo 回归直接相关的依赖包文件：

- `dual_fr3_moveit_config/config/moveit_controllers.yaml`
- `dual_fr3_moveit_config/config/gazebo_ros2_controllers.yaml`
- `dual_fr3_moveit_config/config/ros2_controllers.yaml`
- `dual_fr3_moveit_config/config/dual_fr3.urdf.xacro`
- `dual_fr3_moveit_config/launch/demo.launch.py`
- `dual_fr3_moveit_config/launch/gazebo.launch.py`

## 4. 关键点约定

当前默认点位于 `config/keypoints.yaml`：

```text
corner_1 [0.604,  0.733, 0.150]  in slot
corner_2 [0.604,  0.133, 0.150]  in slot
corner_4 [0.364,  0.133, 0.150]  in slot
entry_5  [0.364, -0.070, 0.150]  out of slot
```

`entry_0` 和 `corner_3` 当前在 YAML 中被注释，不参与规划。不要用旧文档中的 6 点
坐标覆盖当前用户点位。

约定：

- 相邻点必须在同一 frame。
- 点只描述位置和语义，不描述方向。
- YAML 中不要添加 `rpy`，加载器会拒绝。
- 默认工具姿态为 `roll=pi`、`pitch=0`，yaw 由相邻点连线和 actor 的
  `orientation_direction` 共同计算。
- `forward` 表示沿关键点索引递增方向，`reverse` 表示反向。leader 默认
  `reverse`（下一个点指向上一个点），follower 默认 `forward`（上一个点指向
  下一个点）。
- `false -> false` 和 `true -> true` 分类为 `straighten`。
- `false -> true` 或 `true -> false` 分类为 `seat_edge`。

## 5. 调度逻辑

默认准备位置索引：`initial_leader_index=1`、`initial_follower_index=0`。
按当前 YAML 中启用的关键点，这对应 leader=`corner_2`、follower=`corner_1`。

正式任务开始前的准备动作位于独立模块
`dual_fr3_trunking_mtc/preparation.py`，顺序固定为：

1. leader：使用 OMPL 从当前状态移动到 `corner_2` 上方 `0.15 m`。
2. follower：使用 OMPL 从当前状态移动到 `corner_1` 上方 `0.15 m`。
3. 等待键盘确认，闭合 leader 夹爪。
4. 等待键盘确认，闭合 follower 夹爪。
5. 等待键盘确认，通过 MTC `Merger` 让双臂 TCP 沿 Cartesian 直线同步向下
   `0.15 m`。

`execute:=true` 时这五步逐步从最新机器人状态规划和执行，准备完成后再规划正式
任务。`execute:=false` 时准备动作和正式任务仍会组合成一条 MTC 任务用于 RViz
预览。

后续调度规则是：当 follower 的下一段属于 `straighten` 且 leader 已在前方时，先让 leader 跳到下一个 anchor，再由 follower 追赶拉直；`seat_edge` 则先执行 leader 让位和 follower 卡线。当前是运动学近似，不包含下压、接触检测、力控、视觉或线缆张力反馈。

## 6. 当前 MTC stage 映射

准备动作文件：`dual_fr3_trunking_mtc/preparation.py`；正式任务 stage 文件：
`dual_fr3_trunking_mtc/mtc_prototype.py`。

```text
leader/follower initial approach:  MoveTo + PipelinePlanner (OMPL RRTConnect)
leader/follower close gripper:     MoveTo(hand group) + JointInterpolationPlanner
dual Cartesian descent:            Merger[MoveRelative(CartesianPath) x 2]
seat_cable_on_edge:                 InfoOnly 占位（当前不发运动命令）
leader_move_ahead_for_seat_edge:   MoveRelative + CartesianPath
turn_gripper_to_next_keypoint:     MoveTo(PoseStamped) + CartesianPath
cartesian_move_to_next_keypoint:   MoveRelative + CartesianPath
direct_move_to_next_anchor:        MoveTo + PipelinePlanner (OMPL RRTConnect，TCP z 上限可配置)
direct_move_to_seat_edge_keypoint: MoveTo + JointInterpolationPlanner
```

夹爪只在准备阶段各闭合一次；后续所有机械臂 stage 默认依赖夹爪保持闭合，
不再在每个动作前重复发送闭合命令。

终段特殊规则：当 leader 已在最后关键点、follower 在倒数第二个关键点时，
先保留 `seat_cable_on_edge` 卡线占位动作，然后跳过当前代码中的 leader 让位、
follower 转向和 follower 移动。

原地转向目标使用当前转弯关键点 xyz，以及按 actor 的 `orientation_direction`
计算的路径 yaw，意图是保持 TCP xyz 不变，只改变姿态。它比旧的
`MoveRelative + TwistStamped(angular.z=...)` 语义更明确，但仍可能因 Cartesian IK、
碰撞或奇异位形失败。

## 7. 已知失败证据

### 7.1 已修复的 Gazebo 回归：控制器名称分流缺失

回归原因是：

- 真机 `demo.launch.py` 被改成两个 controller manager：
  `/left/controller_manager` 和 `/right/controller_manager`。
- 真机轨迹 Action 因此是：
  `/left/left_fr3_arm_controller/follow_joint_trajectory` 和
  `/right/right_fr3_arm_controller/follow_joint_trajectory`。
- 共享的 `config/moveit_controllers.yaml` 随之被改成了
  `/left/left_fr3_arm_controller`、`/right/right_fr3_arm_controller`。
- 但 `gazebo.launch.py` 和 `gazebo_ros2_controllers.yaml` 仍使用单个根命名空间
  `/controller_manager`，创建的是 `/left_fr3_arm_controller`、
  `/right_fr3_arm_controller`。

现已新增 `config/moveit_controllers_gazebo.yaml`，由 `gazebo.launch.py` 专用；
`demo.launch.py` 继续使用原 `moveit_controllers.yaml`，真机双 controller manager
结构和名称未改动。

当前按不同后端分开 MoveIt 控制器映射：

```text
真机/双 controller manager:
  /left/left_fr3_arm_controller
  /right/right_fr3_arm_controller

Gazebo/单 controller manager:
  left_fr3_arm_controller
  right_fr3_arm_controller
```

不要把 Gazebo 映射重新合入真机文件，否则会再次破坏已经连通的真机执行链。

### 7.2 最新真机执行证据

2026-08-29 使用：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  use_fake_hardware:=false \
  left_robot_ip:=192.168.1.2 \
  right_robot_ip:=192.168.2.2 \
  start_gripper:=true plan:=true execute:=true
```

已确认成功：

- 两台 FR3 硬件连接成功。
- 左右 arm controller 激活。
- MoveGroup 加载 `ExecuteTaskSolution` capability。
- 两侧夹爪连接和 Homing 成功。
- 就绪门通过。
- stage `[00] initial_follower_close_gripper` 执行成功。

随后 stage `[01] initial_leader_close_gripper` 规划失败：

```text
Start state is out of bounds!
```

当时 `/joint_states` 中左夹爪两个关节均为
`0.040015940140588784` m，而原模型上限为 `0.04` m。当前工作树中的
`research_franka_hand.xacro` 已把上限改为 `0.041` m，但尚无重新真机验证证据，
不能把问题标记为已解决。

### 7.3 较早的 Gazebo 碰撞证据

较早运行命令：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  use_gazebo:=true plan:=true execute:=true leader_lead_distance:=0.10
```

当时 MTC 已生成 solution，多个 controller goal 也成功到达；随后 MoveIt 因双臂
碰撞停止：

```text
Trajectory component '15/19' is invalid for waypoint 24 out of 72
Found a contact between 'left_fr3_link6' and 'right_fr3_link7'
Stopping execution because the path to execute became invalid
Goal was aborted or canceled
```

排查时必须区分：

1. `task.plan()` 失败：没有 solution。
2. controller 执行失败：solution 存在但目标未完成。
3. 执行中碰撞：MoveIt 取消后续轨迹。
4. 起始状态越界：真实 joint state 与 URDF/SRDF 限位不一致。
5. 原地旋转失败：Cartesian IK、碰撞或中间姿态不可行。

Octomap 没有 3D sensor plugin、RViz 没有 `/recognize_objects`、实时内存锁失败和
`ff_velocity_scale` deprecation 都不是上述真机停止的直接原因。

## 8. 两个诊断执行器

### 逐动作脚本

`trunking_step_by_step.py` 直接调用用户验证过的控制器接口。原地旋转读取实时 TCP pose：

```python
controller.move_to_cartesian(
    arm, current.x, current.y, current.z,
    roll, pitch, target_yaw, frame_id=frame_id
)
```

### 逐 stage 诊断器

先启动 Gazebo：

```bash
ros2 launch dual_fr3_moveit_config gazebo.launch.py
```

再运行：

```bash
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py --list
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py \
  --confirm-each
```

测试单个 stage：

```bash
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py \
  --start-stage 13 --stop-stage 13 --confirm-each
```

编号必须以当前 `--list` 为准。诊断时不要先开启 `--allow-cartesian-fallback`；该选项只适合“尽量跑完”，不适合判断 Cartesian 本身是否成功。诊断器会输出执行前后 TCP xyz/rpy、位置/姿态误差，并可区分规划和执行失败。

`segment_executor.py` 现在要求左右两侧 controller state 都更新，并同时订阅两种命名：

```text
真机：/left/left_fr3_arm_controller/controller_state
      /right/right_fr3_arm_controller/controller_state
Gazebo：/left_fr3_arm_controller/controller_state
       /right_fr3_arm_controller/controller_state
```

这项兼容只解决诊断器的状态观察，不会修复 MoveIt
`FollowJointTrajectory` controller 映射错误。

## 9. ROS 话题和入口

规划节点 `/dual_fr3_trunking_planner` 发布：

- `/dual_fr3_trunking_planner/plan_summary`
- `/dual_fr3_trunking_planner/task_schedule`
- `/dual_fr3_trunking_planner/markers`

服务：`/dual_fr3_trunking_planner/replan`。

MTC 节点发布：

- `/dual_fr3_trunking_mtc_prototype/stage_sequence`
- `/dual_fr3_trunking_mtc_prototype/stage_sequence_text`

MTC 日志中的 `stage_key` 由 task step、action、actor、起止点和 primitive 构成；同时会打印 actor、group、frame、vector、yaw、执行顺序。

`trunking_readiness_gate.py` 根据 `use_gazebo` 选择控制器命名：

- `use_gazebo:=false`：等待真机/分命名空间控制器。
- `use_gazebo:=true`：等待 Gazebo 根命名空间控制器。

它还会等待 `/move_action`、`/execute_task_solution`（执行时）、左右夹爪 Action、
`/joint_states` 和两个 controller state。默认超时后关闭 launch，不会继续启动 MTC。

## 10. 重要参数

`launch/mtc_prototype.launch.py` 的主要默认值：

```text
use_gazebo=false, plan=true, execute=false
execute_stage_by_stage=true, start_gripper=true
initial_leader_index=1, initial_follower_index=0
leader_group=left_fr3_arm, follower_group=right_fr3_arm
leader_ik_frame=left_fr3_hand_tcp, follower_ik_frame=right_fr3_hand_tcp
leader_orientation_direction=reverse, follower_orientation_direction=forward
motion_velocity_scaling=0.2, motion_acceleration_scaling=0.2
leader_lead_distance=0.10
tool_roll=pi, tool_pitch=0.0
preparation_enabled=true, preparation_height=0.15
preparation_interactive=true
trajectory_execution_duration_scaling=10.0
trajectory_execution_goal_margin=5.0
readiness_timeout=60.0, state_max_age=0.5
home_grippers_before_execute=true, grippers_homed=false
mtc_keep_alive_sec=30.0
```

`leader_lead_distance:=0.10` 是让位距离，不是速度参数。

如需让 leader 恢复旧版的正向朝向：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  leader_orientation_direction:=forward
```

## 11. 推荐排查顺序

1. 只启动 `dual_fr3_moveit_config gazebo.launch.py`，不要同时启动 MTC。
2. 检查 `/controller_manager/list_controllers` 中五个 Gazebo controller 是否 active。
3. 用 `ros2 action list` 记录 Gazebo 实际的两个
   `follow_joint_trajectory` Action 名称。
4. 检查 move_group 日志中 `Added FollowJointTrajectory controller for ...` 的名称，
   必须与第 3 步逐字一致。
5. 在 RViz 中对左右臂分别 plan + execute，确认 Gazebo 模型和 `/joint_states` 都更新。
6. 再运行 `mtc_prototype.launch.py use_gazebo:=true plan:=false execute:=false`，
   确认就绪门通过。
7. 然后才恢复 MTC `plan:=true execute:=false` 和逐 stage 仿真。
8. 仿真闭环恢复后，再处理真机夹爪上限和后续动作。

每次测试前确认机器人实际状态符合动作前置状态。不要在动作执行到一半后，直接运行假设机器人仍在初始状态的 stage。

## 12. 后续 agent 任务优先级

### P0（已完成）：恢复 Gazebo，同时保留真机控制器命名空间

- 不要删除或复杂化 `trunking_step_by_step.py`。
- 不要用单一 `moveit_controllers.yaml` 强行同时描述两种不相同的 controller 名称。
- 建议拆分真机/`demo.launch.py` 与 Gazebo 的 MoveIt controller YAML，并让两个
  launch 明确选择各自文件。
- 保持真机名称：`/left/left_fr3_arm_controller`、
  `/right/right_fr3_arm_controller`。
- 保持 Gazebo 名称：`left_fr3_arm_controller`、
  `right_fr3_arm_controller`，除非完整地把 Gazebo controller manager、spawner、
  joint state 和所有客户端一起迁移到命名空间。
- 检查 `trunking_step_by_step.py`、`segment_executor.py` 和
  `mtc_prototype.launch.py` 对修复后两套名称的兼容。

P0 验收条件：

1. `gazebo.launch.py` 能稳定启动，五个 controller 为 active。
2. MoveIt 配置的两个 controller 与 `ros2 action list` 完全一致。
3. RViz 分别执行左右臂轨迹时 Gazebo 和 `/joint_states` 同步变化。
4. `mtc_prototype.launch.py use_gazebo:=true` 的就绪门能够通过。
5. 真机 launch 的命名空间配置没有被回退或覆盖。

2026-08-31 验证结果：五个 Gazebo controller 均为 active；MoveIt 加载
`left_fr3_arm_controller`、`right_fr3_arm_controller`、`left_franka_gripper`、
`right_franka_gripper`；左右臂小幅轨迹均成功；夹爪从 0.04 m 分别移动到约
0.0 m 和 0.02 m，`/joint_states` 中两侧 mimic finger 同步；readiness gate 通过。

### P1：复核真机夹爪限位修复

- 当前 `research_franka_hand.xacro` 的 finger upper limit 已从 `0.04` 改为
  `0.041`，但未重新真机验证。
- 验证 Homing 后 `/joint_states`、MoveIt robot model 和 MTC CurrentState 一致。
- 确认 stage `[00]`、`[01]` 都能依次闭合后，再允许机械臂运动 stage 开始。

### P2：继续验证逐 stage MTC

逐 stage 重新规划已经实现。保存完整 `--list` 和运行日志，对照 stage index；明确
后续失败属于规划、controller、状态越界、姿态误差还是碰撞。不要只增加规划时间或
watchdog。

### P3：解决双臂碰撞

重点检查 `left_fr3_link6` 与 `right_fr3_link7`。优先调整关键点、leader 让位距离或调度顺序，不要关闭碰撞检测。

### P4：补全真机安全策略

就绪门和失败即停已经实现，但仍需确认速度/加速度限制、初始关节状态、IP、frame、
急停、碰撞保护、接触检测和力控策略。完成真机安全复核前不要继续扩大动作范围。

## 13. 不要重复踩坑

- 不要把关键点硬编码成固定方向，方向必须由路径计算。
- 不要把 `planning succeeded` 当成整条轨迹安全执行完成。
- 不要把所有失败都归因于随机性。
- 不要恢复 `TwistStamped` 原地旋转，除非已验证参考系和旋转中心。
- 不要为了跑完流程而关闭双臂碰撞检测。
- 不要同时启动多套 Gazebo/MoveIt；多个 `/move_action` server 会导致目标分发不确定。
- 不要把真机和 Gazebo 的 controller 名称写进同一份无法条件化的配置后假设二者兼容。
- 修 Gazebo 时不要破坏已验证的真机双 controller manager 结构。
- 不要绕过 readiness gate 来掩盖 Action 或状态话题名称错误。

## 14. 构建和测试

```bash
cd /home/jhr/ws_franka
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_fr3_moveit_config dual_fr3_trunking_mtc \
  --symlink-install
source install/setup.bash
```

```bash
cd /home/jhr/ws_franka
source /opt/ros/humble/setup.bash
PYTHONPATH=/home/jhr/ws_franka/src/dual_fr3_trunking_mtc:$PYTHONPATH \
  python3 -m pytest -q src/dual_fr3_trunking_mtc/test
```

当前已验证：22 个 Python 测试通过，`ament_flake8`、`ament_pep257`、
`ament_lint_cmake` 通过，`dual_fr3_trunking_mtc` 单包构建成功，launch 参数解析成功。
完整 `colcon test` 还会遇到仓库原有 copyright 缺失，以及受限环境不能下载 ROS XML
schema 的 `xmllint` 失败。这些不等于 Gazebo 运行验证通过；当前 Gazebo 回归尚未修复。

## 15. 修改边界

两个仓库都有用户需要保留的未提交修改。开始工作前分别执行：

```bash
git -C /home/jhr/ws_franka/src/dual_fr3_moveit_config status --short
git -C /home/jhr/ws_franka/src/dual_fr3_trunking_mtc status --short
```

本轮在 `dual_fr3_trunking_mtc` 中新增或修改的关键内容：

- 新增 `readiness.py` 和 `trunking_readiness_gate.py`。
- `mtc_prototype.launch.py` 用就绪进程退出事件替换固定 Timer。
- `demo.launch.py`/`mtc_prototype.launch.py` 继续传递 `start_gripper`。
- `segment_executor.py` 同时检查左右 controller state，并兼容两套话题名称。
- `mtc_prototype.py` 任一 stage 失败或异常后立即终止后续 stage。
- 新增 readiness 和失败即停测试，CMake 注册 pytest。

`dual_fr3_moveit_config` 当前未提交的高风险修改包括：

- `demo.launch.py` 从单 controller manager 拆成 `/left`、`/right` 两套 manager。
- `moveit_controllers.yaml` 改为真机命名空间 controller 名称；这是 Gazebo 回归的
  直接配置冲突点。
- `ros2_controllers.yaml` 改成通配命名空间结构。
- `dual_fr3.urdf.xacro` 改用 `franka_description` 中的 ros2_control xacro，并增加
  `load_left_ros2_control`/`load_right_ros2_control`。
- `research_franka_hand.xacro` finger upper limit 从 `0.04` 改为 `0.041`，尚未真机复测。

下一个 agent 被授权为修复 Gazebo 而修改 `dual_fr3_moveit_config`，但必须保持改动最小，
先做真机/Gazebo controller 配置分流，不要回滚整个包，也不要覆盖与控制器无关的用户
改动。修复后同时记录 Gazebo Action 名称、MoveIt 加载名称和真机名称，避免再次互相
覆盖。
