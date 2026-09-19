# 执行、接口与排查

常用启动命令见 [README](../README.md)，任务参数见[任务配置](configuration.md)。

## 规划与执行顺序

`mtc_prototype.launch.py` 同时启动机器人环境和就绪检查。检查通过后才启动 MTC，失败则关闭本次 launch。检查内容包括 MoveGroup、双臂轨迹 action、双臂控制器状态、14 个机械臂关节及对应夹爪接口；执行时还等待 `/execute_task_solution`。

默认 `readiness_timeout=60.0` 秒，`state_max_age=0.5` 秒。它是一次性的启动检查，不是后续运动监控器。

启用准备动作时，MTC 从同一规划场景搜索不同准备关节姿态，并预检从准备到正式任务结束的完整路径。成功后保存这份解，`execute_stage_by_stage:=true` 按顺序执行原轨迹，不在每个正常完成的阶段后重新规划。

只有关闭准备动作且没有要求逐阶段执行的夹爪或仿真线缆操作时，才可使用 `execute_stage_by_stage:=false` 一次提交整份解。

| 设置 | 行为 |
| --- | --- |
| `plan:=false execute:=false` | 构造阶段和任务，不求解、不执行 |
| `plan:=true execute:=false` | 完整规划预检，不运动 |
| `plan:=true execute:=true` | 规划成功后执行，当前默认模式 |

ManiSkill 接触夹持准备自动定位并闭爪；随后解除世界固定、验证抓持、更新实测 USB 附着位姿，并验证剩余的原缓存轨迹（启用插入时也包括原插入接近和碰撞预检段）。通过后保留原关节路径、速度和时间参数，在下降前只等待一次 Enter。按 Enter 开始任务、输入 `q` 中止；`preparation_interactive:=false` 跳过确认，适用于自动仿真。其他后端保留原闭合前确认。

抓取后验证默认不调用运动规划器。验证检查实测起点与缓存起点的关节差（上限 0.01）、关节限位、双臂和附着体碰撞；检查全部缓存路点，并把相邻点细分到每个关节步长不超过 0.002（转动关节 rad、移动关节 m）。这是离散碰撞验证，不是连续碰撞检测。未运动的关节保留实测值；后续夹爪动作按实际开度到目标开度检查。插入接近还须满足原 USB 对齐阈值。验证成功后同步缓存场景的附着体，避免执行时恢复预设抓姿。

`replan_after_grasp:=false` 为默认值：缓存验证不通过就停止，不继续下降或搬运。明确允许验证失败后重规划时，追加：

```bash
replan_after_grasp:=true
```

即使开启此项，验证通过也始终复用原轨迹；只有验证不通过才从实测状态重规划未完成阶段，仍使用原目标和 `planning_attempts` 次数上限。不会重放抓取。读取实测场景失败、缺少附着体或验证异常时直接停止，不把未知状态当成可重规划的碰撞失败。日志以 `[cached-validation]` 给出具体阶段和失败位置。

ManiSkill USB 场景在机械臂接近前按关键点准备目标 `spawn` 并固定，随后张开接近，闭合动作结束后仍须通过持续双指接触、解除世界定位、释放后稳定验证，才能提交规划附着和搬运。`load_cable:=false` 不创建线缆，但保留双臂准备、夹持验证、下降及原正式 MTC 运动，用于 USB 搬运调试，不代表完成线缆布线。详见[接口与状态](../../dual_fr3_maniskill/docs/mtc_cable.md)。

## 真机夹爪回零

真机执行默认 `home_grippers_before_execute:=true`，会依次执行两个夹爪 Homing，夹爪内部应为空。若已手动回零并放入线缆，启动参数使用：

```bash
home_grippers_before_execute:=false grippers_homed:=true
```

两项都为 `false` 时，就绪检查拒绝启动 MTC。准备阶段需要 `start_gripper:=true`。夹爪接口和测试命令见[夹爪说明](../../dual_fr3_moveit_config/docs/gripper_action_test.md)。

## 失败恢复

明确的轨迹终止错误，如 `CONTROL_FAILED`、`TIMED_OUT`、`INVALID_MOTION_PLAN` 和 `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE`，可在 `execution_replan_attempts` 上限内恢复。恢复从实测状态开始，只规划失败阶段和后续阶段。

`execution_replan_attempts` 与 `replan_after_grasp` 独立：前者控制实际执行失败后的恢复，后者仅控制抓取后的缓存验证失败。两项都关闭时使用 `execution_replan_attempts:=0 replan_after_grasp:=false`。

相对移动使用原成功解保存的绝对 TCP 终点，避免中途停止后再次走完整段位移；准备阶段保留已选中的关节目标。已完成的夹爪和线缆生成操作不会重放。

取消、通信异常、未知错误、夹爪失败会停止执行。初次线缆生成失败时不会继续下降。完整规划没有成功解时，即使 `execute:=true` 也不会执行运动。

## ROS 接口

| 入口 | 接口 | 类型与内容 |
| --- | --- | --- |
| 关键点可视化 `demo.launch.py` | `/dual_fr3_trunking_planner/plan_summary` | `std_msgs/msg/String`，关键点和段分类 JSON |
| 同上 | `/dual_fr3_trunking_planner/task_schedule` | `std_msgs/msg/String`，全局调度 JSON |
| 同上 | `/dual_fr3_trunking_planner/markers` | `visualization_msgs/msg/MarkerArray` |
| 同上 | `/dual_fr3_trunking_planner/replan` | `std_srvs/srv/Trigger`，重新读取并规划 |
| MTC 入口 | `/dual_fr3_trunking_mtc_prototype/stage_sequence` | `std_msgs/msg/String`，阶段序列 JSON |
| MTC 入口 | `/dual_fr3_trunking_mtc_prototype/stage_sequence_text` | `std_msgs/msg/String`，可读阶段文本 |

查看已发布的阶段序列：

```bash
ros2 topic echo /dual_fr3_trunking_mtc_prototype/stage_sequence_text \
  std_msgs/msg/String --qos-durability transient_local --once
```

MTC 完成后发布节点默认仅保留 30 秒，可在启动时加 `mtc_keep_alive_sec:=120.0` 延长查看时间。可视化节点默认每秒检查文件更新，也可手动重载：

```bash
ros2 service call /dual_fr3_trunking_planner/replan std_srvs/srv/Trigger '{}'
```

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| MTC 节点没有启动 | 查看就绪检查输出，检查状态新鲜度、控制器命名空间和 action 是否存在 |
| 无法连接 `execute_task_solution` | 使用本包 MTC launch，它会加载 `move_group/ExecuteTaskSolutionCapability` |
| RViz 没有关键点球体 | MTC 入口不发布该标记；使用可视化入口或单独运行规划节点 |
| 规划成功但不运动 | 核对 `execute`、准备阶段确认和终端执行错误 |
| ManiSkill 没有线缆 | 核对 `maniskill_cable`、`load_cable`、准备开关及闭合前 spawn 返回结果；只规划不会生成；USB-only 本就不创建线缆 |
| 动作超时或状态停滞 | 检查 `/clock`、控制器状态和仿真错误；关闭窗口不等于提升物理步进速度 |

```bash
ros2 action list -t
ros2 topic echo /joint_states --once
```

规划失败摘要中，`HAS_SOLUTION` 只代表阶段局部有候选解；`FAILED` 表示只有失败候选；`NO_RESULT` 表示尚无候选，可能受前序失败影响；`SKIPPED` 表示说明或无运动阶段。

`PATH_LENGTH_LIMIT` 报告 TCP 路程上限，`CARTESIAN_PATH_DEVIATION` 报告 TCP 偏离。`INVALID_MOTION_PLAN` 的诊断会检查保留轨迹中的碰撞和高度；这是采样检查，未定位到原因时仍需查 MoveIt/FCL 日志。

`[planning-summary]` 按尝试汇总阶段结果。`failed_only/with_result` 只针对已返回候选的尝试，不应把无结果的阶段直接算作失败。

保存无颜色日志可在启动前设置 `TRUNKING_LOG_COLOR=never` 或 `NO_COLOR=1`，只影响本包 Python 日志。

## 逐段诊断工具

这些工具用于单独复现机器人运动，不替代包含准备夹持和线缆生成的完整 MTC 流程。阶段编号随配置变化，先列出当前任务再选择编号。

终端一启动机器人环境：

```bash
ros2 launch dual_fr3_moveit_config gazebo.launch.py
```

终端二加载 ROS 和工作区后运行：

```bash
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py --list
ros2 run dual_fr3_trunking_mtc trunking_segment_executor.py --confirm-each
```

使用 `--start-stage N --stop-stage N` 选择单个阶段，`--plan-only` 只规划；实际执行某个中间阶段前，机器人必须已处于相应起始状态。默认禁止笛卡尔失败自动回退为关节空间路径，便于定位直线运动问题。

另一个固定动作交互脚本会自行启动并清理 Gazebo，因此单独使用：

```bash
ros2 run dual_fr3_trunking_mtc trunking_step_by_step.py
```

按 Enter 执行、`s` 跳过、`q` 退出；Gazebo 日志位于 `/tmp/dual_fr3_trunking_gazebo.log`。更多开关用各脚本的 `--help` 查询。

ManiSkill 夹爪结果等待使用 profile 的仿真时间超时；墙钟保护为 `max(120, 30 × profile.timeout)` 秒。其他后端仍使用原墙钟超时。动作结束后仍须通过双指接触、释放定位和稳定验证，才能开始搬运。

末端插入现可通过 `insertion_enabled:=true` 启用；几何、反馈、保持、双臂释放回位、参数及实际验收边界见 [USB 插入说明](../../dual_fr3_maniskill/docs/usb_insertion.md)。早期验证记录中“未实现插入”的说明仅适用于当时版本。
