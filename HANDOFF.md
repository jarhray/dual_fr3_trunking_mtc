# dual_fr3_trunking_mtc 交接文档

## 1. 项目定位

这是双 FR3 机械臂在线槽/线缆路径上协同走线的任务层包，不是完整的真机接触操作程序。

- `leader` 固定为左臂：`left_fr3_arm`。
- `follower` 固定为右臂：`right_fr3_arm`。
- 默认任务坐标系：`left_fr3_link0`。
- 依赖包 `dual_fr3_moveit_config` 用户已经回滚，后续默认不得修改。

## 2. 当前最重要结论

当前实际运动优先使用逐动作脚本：

```bash
source /home/jerry/ws_moveit/install/setup.bash
source /home/jerry/franka_ros2_ws/install/setup.bash
ros2 run dual_fr3_trunking_mtc trunking_step_by_step.py
```

脚本会启动 `dual_fr3_moveit_config gazebo.launch.py`，然后直接复用用户已经验证过的 `DualFR3LinearController`、`move_to_ompl()` 和 `move_to_cartesian()`。每次只执行一个动作，执行完成后重新读取 TF/joint state，再规划下一动作。

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
- `dual_fr3_trunking_mtc/segment_executor.py`：逐 stage 诊断执行器。
- `scripts/trunking_step_by_step.py`：最简单的逐动作执行器。
- `scripts/trunking_mtc_prototype.py`：MTC 脚本入口。
- `dual_fr3_trunking_mtc/ros_node.py`：关键点、线段、summary、schedule 发布。
- `launch/demo.launch.py`：MoveIt demo 加关键点可视化。
- `launch/mtc_prototype.launch.py`：MoveIt/Gazebo 加 MTC 节点。

## 4. 关键点约定

当前默认点位于 `config/keypoints.yaml`：

```text
entry_0  [0.604, 0.900, 0.030]  out of slot
corner_1 [0.604, 0.500, 0.050]  in slot
corner_2 [0.604, 0.373, 0.030]  in slot
corner_3 [0.364, 0.373, 0.030]  in slot
corner_4 [0.364, 0.133, 0.030]  in slot
entry_5  [0.364, 0.050, 0.030]  out of slot
```

约定：

- 相邻点必须在同一 frame。
- 点只描述位置和语义，不描述方向。
- YAML 中不要添加 `rpy`，加载器会拒绝。
- 普通点朝下一个点，最后一点朝上一段的进入方向。
- 默认工具姿态为 `roll=pi`、`pitch=0`，yaw 由相邻点连线计算。
- `false -> false` 和 `true -> true` 分类为 `straighten`。
- `false -> true` 或 `true -> false` 分类为 `seat_edge`。

## 5. 调度逻辑

默认初始化索引：`initial_leader_index=1`、`initial_follower_index=0`。
按当前 YAML 中启用的关键点，这对应 leader=`corner_2`、follower=`corner_1`。

初始化顺序固定为：

1. follower：先闭合夹爪。
2. leader：再闭合夹爪。
3. follower：当前状态 -> `corner_1`。
4. leader：当前状态 -> `corner_2`。

后续调度规则是：当 follower 的下一段属于 `straighten` 且 leader 已在前方时，先让 leader 跳到下一个 anchor，再由 follower 追赶拉直；`seat_edge` 则先执行 leader 让位和 follower 卡线。当前是运动学近似，不包含下压、接触检测、力控、视觉或线缆张力反馈。

## 6. 当前 MTC stage 映射

文件：`dual_fr3_trunking_mtc/mtc_prototype.py`。

```text
close_gripper_at_start:             MoveTo(hand group) + JointInterpolationPlanner
seat_cable_on_edge:                 InfoOnly 占位（当前不发运动命令）
move_to_initial_keypoint:          MoveTo + JointInterpolationPlanner
leader_move_ahead_for_seat_edge:   MoveRelative + CartesianPath
turn_gripper_to_next_keypoint:     MoveTo(PoseStamped) + CartesianPath
cartesian_move_to_next_keypoint:   MoveRelative + CartesianPath
direct_move_to_next_anchor:        MoveTo + JointInterpolationPlanner
direct_move_to_seat_edge_keypoint: MoveTo + JointInterpolationPlanner
```

夹爪只在初始化阶段各闭合一次；后续所有机械臂 stage 默认依赖夹爪保持闭合，
不再在每个动作前重复发送闭合命令。

终段特殊规则：当 leader 已在最后关键点、follower 在倒数第二个关键点时，
先保留 `seat_cable_on_edge` 卡线占位动作，然后跳过当前代码中的 leader 让位、
follower 转向和 follower 移动。

原地转向目标使用当前转弯关键点 xyz 和下一段 yaw，意图是保持 TCP xyz 不变，只改变姿态。它比旧的 `MoveRelative + TwistStamped(angular.z=...)` 语义更明确，但仍可能因 Cartesian IK、碰撞或奇异位形失败。

## 7. 已知失败证据

最近一次运行：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  use_gazebo:=true plan:=true execute:=true leader_lead_distance:=0.10
```

日志显示 MTC 已生成 solution，多个 controller goal 也成功到达；随后 MoveIt 因双臂碰撞停止：

```text
Trajectory component '15/19' is invalid for waypoint 24 out of 72
Found a contact between 'left_fr3_link6' and 'right_fr3_link7'
Stopping execution because the path to execute became invalid
Goal was aborted or canceled
```

必须区分：

1. `task.plan()` 失败：没有 solution。
2. controller 执行失败：solution 存在但目标未完成。
3. 执行中碰撞：MoveIt 取消后续轨迹。
4. 原地旋转失败：Cartesian IK、碰撞或中间姿态不可行。

当前 MTC 主要用 `JointInterpolationPlanner` 和 `CartesianPath`，不能把所有失败都归因于 OMPL 随机性。真正的核心差异是 MTC 整链执行没有真实状态闭环，且双臂中间姿态存在碰撞风险。

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

## 10. 重要参数

`launch/mtc_prototype.launch.py` 的主要默认值：

```text
use_gazebo=false, plan=true, execute=false
initial_leader_index=1, initial_follower_index=0
leader_group=left_fr3_arm, follower_group=right_fr3_arm
leader_ik_frame=left_fr3_hand_tcp, follower_ik_frame=right_fr3_hand_tcp
motion_velocity_scaling=0.2, motion_acceleration_scaling=0.2
leader_lead_distance=0.10
tool_roll=pi, tool_pitch=0.0
align_initial_poses=true
trajectory_execution_duration_scaling=10.0
trajectory_execution_goal_margin=5.0
mtc_start_delay=6.0, mtc_keep_alive_sec=30.0
```

`leader_lead_distance:=0.10` 是让位距离，不是速度参数。

## 11. 推荐排查顺序

1. 确认没有旧 Gazebo、RViz、`move_group` 或 controller 残留。
2. 用 `demo.launch.py` 检查小球、线段、关键点位置和 frame。
3. 用 MTC `plan:=true execute:=false` 检查 stage 是否完整。
4. 用诊断器 `--list` 确认 stage 编号和 actor。
5. 单独执行初始化 stage `0`、`1`。
6. 单独执行原地旋转 stage，编号以当前 `--list` 为准。
7. 记录旋转前后的 TCP xyz/rpy。
8. 单独执行碰撞前后的 stage，检查两臂实际姿态。
9. 单 stage 稳定后再连续执行多个 stage。

每次测试前确认机器人实际状态符合动作前置状态。不要在动作执行到一半后，直接运行假设机器人仍在初始状态的 stage。

## 12. 后续 agent 任务优先级

### P0：保留基线

- 不要删除或复杂化 `trunking_step_by_step.py`。
- 继续直接复用 `fr3_controller.py` 和 `fr3_controller_lin.py`。
- 默认不要修改 `dual_fr3_moveit_config`。

### P1：定位失败 stage

- 保存完整 `--list` 输出和运行日志。
- 对照 MTC stage index 和诊断器 stage index。
- 明确失败属于规划、controller、姿态误差还是碰撞。

### P2：改善 MTC 执行模型

不要只增加规划时间或 watchdog。合理方向是让 MTC 负责 stage 顺序，执行层按 stage/segment 分批执行；每段完成后读取真实 joint state/TF，使用真实状态作为下一段 start state，并重新碰撞检查。这应接近逐步脚本的闭环行为。

### P3：解决双臂碰撞

重点检查 `left_fr3_link6` 与 `right_fr3_link7`。优先调整关键点、leader 让位距离或调度顺序，不要关闭碰撞检测。

### P4：真机准备

仿真闭环稳定前不要连接真机。真机前还需确认速度/加速度限制、初始关节状态、IP、frame、controller 名称、急停、碰撞保护、接触检测和力控策略。

## 13. 不要重复踩坑

- 不要把关键点硬编码成固定方向，方向必须由路径计算。
- 不要把 `planning succeeded` 当成整条轨迹安全执行完成。
- 不要把所有失败都归因于随机性。
- 不要恢复 `TwistStamped` 原地旋转，除非已验证参考系和旋转中心。
- 不要为了跑完流程而关闭双臂碰撞检测。
- 不要同时启动多套 Gazebo/MoveIt；多个 `/move_action` server 会导致目标分发不确定。
- 不要修改 `dual_fr3_moveit_config` 来掩盖任务层问题。

## 14. 构建和测试

```bash
cd /home/jerry/franka_ros2_ws
source /home/jerry/ws_moveit/install/setup.bash
colcon build --packages-select dual_fr3_trunking_mtc --symlink-install
source install/setup.bash
```

```bash
cd /home/jerry/franka_ros2_ws/src/dual_fr3_trunking_mtc
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q test
ament_flake8 dual_fr3_trunking_mtc test
```

当前已验证：15 个测试通过，`ament_flake8` 无错误，单包构建成功。

## 15. 修改边界

当前任务包内的主要改动包括 MTC stage 映射、逐动作执行器、逐 stage 诊断器、关键点/规划/调度和测试。

`dual_fr3_moveit_config` 是依赖包，用户已经回滚过它。后续 agent 默认不得修改；如果必须修改，修改前必须说明具体文件、原因、逻辑变化以及对 Gazebo 和真机的影响。
