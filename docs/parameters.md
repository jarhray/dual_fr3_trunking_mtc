# 参数索引

推荐入口和独立插入调用见 [README](../README.md)。
动作修改先看 [架构](../ARCHITECTURE.md)，参数加载顺序见 [任务配置](configuration.md)。
主入口 launch → runtime/arguments.py → runtime/config.py 补缺省；显式 launch/CLI 值优先。
`mtc_keep_alive_sec` 是 launch 名称，转给 CLI `--keep-alive-sec`。独立 segment executor 是诊断入口，不能替代整段 MTC 预检。
每项的**当前值、代码缺省和声明文件**逐项列在 [默认值与来源](parameter_defaults.md)。同名项保留各入口的差异，不能把代码缺省当成 YAML 覆盖后的值。下面各表明确消费文件、单位、作用及关联；验证命令相对于工作区根目录。改 YAML 后重启消费者；这些文件字段不是动态 ROS 参数接口。

## 启动、后端与就绪

**消费代码：** [launch/mtc_prototype.launch.py](../launch/mtc_prototype.launch.py)、[launch/demo.launch.py](../launch/demo.launch.py)、[dual_fr3_trunking_mtc/runtime/config.py](../dual_fr3_trunking_mtc/runtime/config.py)、[dual_fr3_trunking_mtc/nodes/readiness.py](../dual_fr3_trunking_mtc/nodes/readiness.py)。

**验证：** `test_config.py`、`test_backend_launch.py`、`test_readiness.py`；`ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py --show-args`。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `simulation_backend` | 枚举 | 主 MTC 默认 maniskill；demo 可视化入口仍 gazebo，real/fake 保留原映射。 |
| `fake_sensor_commands` | 布尔 | mock 硬件传感器命令通道，不用于 ManiSkill 物理反馈。 |
| `left_robot_ip` | IP | 真机左臂连接地址，仿真不连接该机器人。 |
| `right_robot_ip` | IP | 真机右臂连接地址，与左侧分开配置。 |
| `load_gripper` | 布尔 | 模型加载夹爪；接触夹持任务要求启用。 |
| `start_gripper` | 布尔 | 启动/检查夹爪控制接口，需和 load_gripper 配套。 |
| `ee_id` | 模型标识 | 末端型号选择，决定 TCP/手指模型。 |
| `use_rviz` | 布尔 | 启动 RViz 窗口，仅影响显示和性能。 |
| `gz_args` | 命令参数串 | Gazebo 启动世界与运行选项，仅 gazebo 后端使用。 |
| `gazebo_effort` | 布尔 | Gazebo 使用力矩或位置控制接口，不修改 ManiSkill 控制方式。 |
| `maniskill_python` | 解释器路径 | 物理 Python，优先环境 MANISKILL_PYTHON，否则工作目录 .venv/bin/python。 |
| `maniskill_viewer` | 布尔 | 物理查看器；MTC launch 默认 true。 |
| `maniskill_cable` | 布尔 | 启用 trunking 接触夹持场景；false 为纯机器人场景。 |
| `load_cable` | 布尔 | 默认 true；false 为 USB-only 调试，运输运动仍保留。 |
| `cable_solver` | mpm/rope_actor | MTC launch 默认 rope_actor，向下层显式传递。 |
| `cable_config` | 文件路径 | MoveIt/物理共享的普通场景 YAML；空值选简化 2 mm。 |
| `cable_trace_dir` | 目录 | Rope 子步诊断，默认空；日志影响墙钟耗时。 |
| `insertion_enabled` | 布尔 | ManiSkill 接触夹持模式默认启用插座与插入前连续预检；显式值优先。 |
| `run_insertion` | 布尔 | 运输后自动执行终末插入；false 保留已启用场景供独立 skill 调用。 |
| `readiness_timeout` | 墙钟 s | 状态/控制器/MoveGroup/夹爪就绪总等待上限。 |
| `state_max_age` | 墙钟 s | 关节/控制器状态允许的最大陈旧时间。 |
| `home_grippers_before_execute` | 布尔 | 执行前夹爪 Homing 开关，真机需遵循就绪门控。 |
| `grippers_homed` | 布尔 | 操作员已确认夹爪回零；不执行自动 Homing 时的确认输入。 |
| `allow_no_controller_state` | 布尔 | 仅诊断 readiness CLI 可允许无控制器状态；不是主入口默认。 |

## 任务几何与调度

**消费代码：** [dual_fr3_trunking_mtc/task/models.py](../dual_fr3_trunking_mtc/task/models.py)、[dual_fr3_trunking_mtc/task/planner.py](../dual_fr3_trunking_mtc/task/planner.py)、[dual_fr3_trunking_mtc/task/scheduler.py](../dual_fr3_trunking_mtc/task/scheduler.py)、[dual_fr3_trunking_mtc/stages/compiler.py](../dual_fr3_trunking_mtc/stages/compiler.py)。

**验证：** `test_planner.py`、`test_scheduler.py`、`test_mtc_prototype.py`；再整段 MoveIt 预检。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `keypoints_file` | YAML 路径 | 关键点输入；文件显式 frame 优先于 task_frame。 |
| `task_frame` | TF frame | 缺省关键点坐标系，不会自动转换已有显式坐标。 |
| `default_frame` | TF frame | 关键点文件的默认 frame；缺省回退 task_frame。 |
| `keypoints[].name` | 字符串 | 点的可读名称，进入阶段和日志。 |
| `keypoints[].frame_id` | TF frame | 该点显式坐标系，同段两端必须一致。 |
| `keypoints[].position` | m，frame_id/default_frame | [x,y,z] 目标位置，影响路径和可达性。 |
| `keypoints[].in_slot` | 布尔 | 相邻点相同生成 straighten，变化生成 seat_edge；不要写字符串布尔。 |
| `keypoints[].role` | 标签 | 阅读/诊断标签，不决定段分类。 |
| `keypoints[].metadata.gripper.enabled` | 布尔 | 启用目标点的附加夹爪操作，要求逐阶段执行。 |
| `keypoints[].metadata.gripper.actor` | leader/follower | 该夹爪操作属于哪只任务臂。 |
| `keypoints[].metadata.gripper.profile` | profile 名 | 使用哪个安全范围内的夹爪配置，可配合逐字段覆盖。 |
| `initial_leader_index` | 从 0 开始的索引 | leader 初始点，须与 follower 及路径长度合法配对。 |
| `initial_follower_index` | 从 0 开始的索引 | follower 初始点，决定最初段和准备位置。 |
| `leader_group` | MoveIt group | 默认左臂 leader；ManiSkill 接触流程要求既有左右分工。 |
| `follower_group` | MoveIt group | 默认右臂 follower，与 leader_group 配对。 |
| `leader_ik_frame` | TCP frame | leader 规划目标工具坐标，不是机器人基座。 |
| `follower_ik_frame` | TCP frame | follower 规划目标工具坐标。 |
| `leader_orientation_direction` | forward/reverse | leader yaw 沿点索引正向或反向路径切线，初始 USB 朝向使用同一值。 |
| `follower_orientation_direction` | forward/reverse | follower yaw 方向，独立于 leader。 |
| `tool_roll` | rad | 路径目标 roll，默认 π，与由切线生成的 yaw 配套。 |
| `tool_pitch` | rad | 路径目标 pitch，默认 0；修改后要重新检查全程 IK。 |
| `leader_lead_distance` | m，路径切向 | seat_edge 阶段 leader 向前让位距离。 |

## 准备、运动规划与执行保护

**消费代码：** [dual_fr3_trunking_mtc/runtime/arguments.py](../dual_fr3_trunking_mtc/runtime/arguments.py)、[dual_fr3_trunking_mtc/mtc/preparation_search.py](../dual_fr3_trunking_mtc/mtc/preparation_search.py)、[dual_fr3_trunking_mtc/mtc/task_builder.py](../dual_fr3_trunking_mtc/mtc/task_builder.py)、[dual_fr3_trunking_mtc/mtc/cached_execution.py](../dual_fr3_trunking_mtc/mtc/cached_execution.py)、[dual_fr3_trunking_mtc/task/preparation.py](../dual_fr3_trunking_mtc/task/preparation.py)。

**验证：** `test_preparation_search.py`、`test_cached_execution.py`、`test_path_length.py`、`test_fr3_cartesian_regression.py`；启用插入后检查 continuation 预检。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `preparation_enabled` | 布尔 | 编译准备定位、闭爪和下降；接触夹持场景要求 true。 |
| `preparation_height` | m，点坐标系 +Z | 初始点上方准备偏移及同步下降距离。 |
| `preparation_interactive` | 布尔 | 接触夹持验证、解除固定、实测后续规划通过后，一次 Enter 开始下降/运输。 |
| `preparation_leader_gripper_profile` | profile 名 | 左爪准备 profile；ManiSkill 目标宽度由 usb.finger_position 覆盖。 |
| `preparation_follower_gripper_profile` | profile 名 | 右爪准备 profile；ManiSkill 目标总开口覆盖为 0。 |
| `preparation_ik_candidates` | 个/臂 | 保留多少不同 IK 候选，越多搜索成本越大。 |
| `preparation_ik_attempts` | 次/臂 | 求 IK 候选尝试的初值数量。 |
| `preparation_ik_timeout` | s/次 | 单次 IK 时间预算，与候选数及总搜索时限关联。 |
| `preparation_min_joint_distance` | rad，关节向量范数 | 候选去重最小关节距离，避免相同姿态重复搜索。 |
| `preparation_candidate_attempts` | 轮 | 遍历双臂候选组合的轮数，单组合还受 planning_attempts 限制。 |
| `preparation_search_timeout` | 墙钟 s | 准备候选搜索总预算，在原生求解调用之间检查，不能打断原生调用。 |
| `cartesian_step_size` | m | MTC 笛卡尔路径采样步长，关联碰撞分辨率和计算量。 |
| `cartesian_jump_threshold` | 相对关节跳变倍数 | 必须 >1；拒绝 IK 分支跳跃，要求完整路径。 |
| `cartesian_path_tolerance` | m，TCP | 直线偏离或原地转向时漂移上限，关联采样和 IK。 |
| `motion_velocity_scaling` | 0–1 比例 | 主 MTC 速度缩放，不覆盖终末插入 motion 中固定 .08/.05。 |
| `motion_acceleration_scaling` | 0–1 比例 | 主 MTC 加速度缩放，影响时间参数化与跟踪。 |
| `anchor_max_path_z` | m，关键点 frame Z | 换锚点 TCP 最高路径限制。 |
| `anchor_max_path_length_ratio` | 无量纲 | 换锚点 TCP 路程/实际起终点距离上限，独立于 insertion.max_path_length_ratio。 |
| `planning_attempts` | 次 | 完整任务/恢复规划及候选内 pipeline 重试预算。 |
| `execution_replan_attempts` | 次 | 执行失败后未完成阶段的恢复规划上限，0 关闭。 |
| `plan` | 布尔 | false 仅构造任务；true 执行全程规划。 |
| `execute` | 布尔 | 是否执行成功解；独立插入 skill 的 false 也只规划，仍会同步规划场景。 |
| `execute_stage_by_stage` | 布尔 | 正常逐段执行同一成功解；false 仅支持无准备/夹爪阶段的整任务 action。 |
| `publish_solution` | 布尔 | 发布 MTC 成功解供 RViz 查看，不改变轨迹。 |
| `keep_alive_sec` | 墙钟 s | CLI 完成后保留节点用于查看解的时间。 |
| `mtc_keep_alive_sec` | 墙钟 s | launch 对 keep_alive_sec 的映射，非另一套超时。 |
| `trajectory_execution_duration_scaling` | 倍数 | MoveIt 允许执行时间相对规划时长的缩放，不改变仿真速度。 |
| `trajectory_execution_goal_margin` | s | MoveIt 执行期限的额外宽限，关联 duration_scaling。 |

## 夹爪 profile 与安全范围

**消费代码：** [config/gripper_profiles.yaml](../config/gripper_profiles.yaml)、[dual_fr3_trunking_mtc/execution/gripper.py](../dual_fr3_trunking_mtc/execution/gripper.py)、[dual_fr3_trunking_mtc/stages/compiler.py](../dual_fr3_trunking_mtc/stages/compiler.py)。

**验证：** `test_gripper.py`、准备阶段测试；真实接触确认并观察解除世界固定后的稳定状态。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `gripper_profiles_file` | YAML 路径 | 夹爪配置源，阶段/准备覆盖仍通过 safety 检查。 |
| `schema_version` | 整数 | 配置格式版本，目前仅 1。 |
| `safety.min_width` | m，总开口 | 允许的最小宽度，限制 profile 和阶段覆盖。 |
| `safety.max_width` | m，总开口 | 允许的最大宽度，必须符合夹爪行程。 |
| `safety.max_speed` | m/s | 允许的请求速度上限；物理桥接按每指速度执行。 |
| `safety.max_force` | N | grasp 请求力上限，独立于接触法向判据。 |
| `safety.default_timeout` | s，仿真/墙钟 | profile 未指定 timeout 时的客户端等待期限。 |
| `profiles.*.action` | move/grasp/hold | 选择 action 类型，hold 不发新夹爪运动。 |
| `profiles.*.width` | m，总开口 | 目标宽度；当前 cable_tip=.0045，ManiSkill 准备阶段另有实体宽度覆盖。 |
| `profiles.*.speed` | m/s | 请求移动速度，不能超过 safety.max_speed。 |
| `profiles.*.force` | N | grasp 力限制，不能超过 safety.max_force。 |
| `profiles.*.epsilon.inner` | m，总宽度差 | grasp 接受区间向内容差。 |
| `profiles.*.epsilon.outer` | m，总宽度差 | grasp 接受区间向外容差。 |
| `profiles.*.timeout` | s，仿真/墙钟 | 可选 action 等待时限，未设用 safety.default_timeout。 |
| `keypoints[].metadata.gripper.width` | m，总开口 | 可选覆盖 profile 宽度，仍受 safety 约束。 |

## 路径显示与自动重载

**消费代码：** [dual_fr3_trunking_mtc/nodes/planner.py](../dual_fr3_trunking_mtc/nodes/planner.py)、[dual_fr3_trunking_mtc/nodes/markers.py](../dual_fr3_trunking_mtc/nodes/markers.py)。

**验证：** `test_config.py`、`test_planner.py`；demo 中查看 Marker，不能当作运动验证。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `samples_per_segment` | 点数 | 可视化每段采样数量，与 MTC 笛卡尔采样步长无关。 |
| `publish_markers` | 布尔 | 是否发布 RViz 路径标记。 |
| `auto_reload` | 布尔 | 监视关键点文件变化并重载可视化计划，不改执行中的 MTC 缓存轨迹。 |
| `reload_period_sec` | s | 文件检查周期，仅用于可视化节点。 |
| `keypoint_marker_scale` | m | 关键点标记球直径。 |
| `segment_line_width` | m | 路径线标记宽度。 |
| `marker_z_offset` | m，标记 frame Z | 仅显示偏移，不移动实际任务目标。 |
| `publish_labels` | 布尔 | 发布关键点/段文字标签。 |

## 独立逐段诊断入口（不是默认流程）

**消费代码：** [dual_fr3_trunking_mtc/nodes/segment_executor.py](../dual_fr3_trunking_mtc/nodes/segment_executor.py)。

**验证：** 使用该脚本 --help / --list 检查阶段；参数仅作用于这个旧诊断执行器，不影响 mtc_prototype 默认路径。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `list` | CLI 开关 | 只列出诊断阶段，不发送运动。 |
| `start_stage` | 索引 | 从指定诊断阶段开始，需自行满足起始状态。 |
| `stop_stage` | 索引 | 在指定诊断阶段停止，配合 start_stage 定位问题。 |
| `plan_only` | CLI 开关 | 仅规划，不推进实际机器人状态，后段不等同整任务连续解。 |
| `confirm_each` | CLI 开关 | 诊断入口逐段确认，与主 MTC 单次 Enter 设置分开。 |
| `continue_on_failure` | CLI 开关 | 诊断失败后继续后段，不是主 MTC 故障恢复策略。 |
| `skip_gripper` | CLI 开关 | 跳过诊断夹爪动作，不能用来验证接触夹持。 |
| `planning_time` | s | 单段 MoveGroup 求解时限。 |
| `ompl_planner_id` | 规划器 ID | 该入口使用的 OMPL planner，与配置中注册名称一致。 |
| `position_tolerance` | m，TCP | 诊断位置目标/到位容差。 |
| `orientation_tolerance` | rad | 诊断姿态目标/到位容差。 |
| `velocity_scale` | 比例 | 单段速度缩放，名称不同于主 MTC motion_velocity_scaling。 |
| `acceleration_scale` | 比例 | 单段加速度缩放。 |
| `cartesian_max_step` | m | 诊断 Cartesian 路径最大采样间隔。 |
| `cartesian_min_fraction` | 0–1 比例 | 该入口接受的 Cartesian 完成比例，主 MTC 仍要求完整路径。 |
| `allow_cartesian_fallback` | CLI 开关 | 诊断允许 Cartesian 失败转 OMPL，默认主流程不因此改变。 |
| `settle_time` | 墙钟 s | 诊断到位后等待稳定时间。 |
| `verification_timeout` | 墙钟 s | 诊断实测位姿验证期限。 |


## 终末插入共享字段

消费位置：[insertion_task](../dual_fr3_trunking_mtc/insertion_task)。字段作用及约束与 [物理侧参数索引](../../dual_fr3_maniskill/docs/parameters.md) 相同；这里列出 MTC 直接读取的字段。验证：test_terminal_insertion.py、test_insertion_skill.py。

| 参数 | 单位 / 坐标 | 作用、预期影响及关联 |
| --- | --- | --- |
| `insertion.approach_lift_m` | m，world +Z | 左臂孔前接近的可选抬高，0 省略；属于整段 MoveIt 预检。 |
| `insertion.clearance_yz_m` | m，插座 Y/Z，每侧 | 孔尺寸=USB 金属截面+2×间隙；与接触壳厚度分开验证。 |
| `insertion.collision_mesh` | 包 share 相对路径 | 插座碰撞 CAD；MoveIt 三角网格与 PhysX 凸体共享孔壁调整。 |
| `insertion.hole_center_m` | m，插座 CAD | 孔口中心，X 必须为 0；影响目标、测量和力矩参考点。 |
| `insertion.max_path_length_ratio` | 无量纲 | TCP 路程/起终点直线距上限；推荐 YAML 3.0，代码回退 1.5。 |
| `insertion.planning_attempts` | 次 | 插入接近/回位规划预算；独立于主任务 planning_attempts。 |
| `insertion.planning_timeout_s` | s | 终末 MoveTo 的规划超时；withdraw 不额外设置此项。 |
| `insertion.release_after_retention` | 布尔 | 固定确认后开左爪；不影响先行右爪释放。 |
| `insertion.retain_after_success` | 布尔 | 成功后创建保持约束；false 停在未固定成功状态。 |
| `insertion.retreat_m` | m，本侧 TCP -Z | 右臂开爪后、左臂固定开爪后的退出距离。 |
| `insertion.return_after_release` | 布尔 | 左爪释放后退出回位；不影响先行右臂回位。 |
| `insertion.timeout_s` | 仿真 s | 反馈阶段总时限；暂停仿真不代替 heartbeat 保护。 |

ManiSkill 夹爪客户端 timeout 以仿真时间计，同时有 `max(120,30*timeout)` 墙钟保护；其他后端等待采用墙钟时间。metadata.gripper 仅支持 enabled/actor/profile/action/width，其他字段不会作为 profile 覆盖生效。

| 参数 | 单位 / 坐标 | 作用及关联 |
| --- | --- | --- |
| `keypoints[].metadata.gripper.action` | move/grasp/hold | 可选覆盖 profile 的动作类型，见 stages/compiler.py 和 execution/gripper.py；验证 test_gripper.py。 |

## 代码固定的规划设置

| 设置 | 当前值 | 位置、影响及验证 |
| --- | --- | --- |
| OMPL_PIPELINE_NAME / OMPL_PLANNER_ID | move_group / RRTConnectkConfigDefault | `mtc/task_builder.py`，选择已有 OMPL 配置；整段规划测试 |
| OMPL_NUM_PLANNING_ATTEMPTS / OMPL_MOVE_TO_TIMEOUT | 5 次 / 5 s | 同上，单次 pipeline 的内部预算，与外层 planning_attempts 不同 |
| ANCHOR_PATH_CONSTRAINT_MIN_Z / XY_SIZE | −2 m / 4 m | 同上，路径约束包围盒下界和 XY 尺寸，Z 上界仍为 anchor_max_path_z |
| 终末接近 Cartesian step / jump / velocity / acceleration | .001 m / 2 / .08 / .08 | `insertion_task/planning.py`、`motion.py`，独立于主任务 .2 缩放 |
| 终末退出 step / jump / velocity / acceleration | .001 m / 2 / .05 / .05 | 同上；右臂先退、左臂固定开爪后退，检查缓存轨迹与完整场景 |

MTC launch 还转发 [ManiSkill 参数索引](../../dual_fr3_maniskill/docs/parameters.md) 中的全部 `perception_*` 选项；
其声明、环境缺省和覆盖逻辑位于 `dual_fr3_maniskill/ros/launch.py`，不在 MTC 再定义一份。
