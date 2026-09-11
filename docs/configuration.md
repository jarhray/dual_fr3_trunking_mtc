# 任务配置

启动和构建见 [README](../README.md)。本页说明关键点和 `mtc_prototype.launch.py` 的主要参数，默认值来自 [runtime/config.py](../dual_fr3_trunking_mtc/runtime/config.py)。

## 关键点与坐标

先复制 [keypoints.yaml](../config/keypoints.yaml)，再通过 `keypoints_file:=/绝对路径/keypoints.yaml` 加载。配置格式示例：

```yaml
default_frame: left_fr3_link0
keypoints:
  - name: corner_1
    position: [0.604, 0.733, 0.05]
    in_slot: true
    role: pull
  - name: corner_2
    position: [0.604, 0.133, 0.05]
    in_slot: true
    role: pull
  - name: entry_5
    position: [0.364, -0.07, 0.10]
    in_slot: false
    role: seat_edge
```

这是格式示例，实际路径可达性需用 MTC 预检确认。

| 字段 | 含义 |
| --- | --- |
| `default_frame` | 文件默认坐标系；省略时采用 `task_frame` |
| `name` | 关键点名称 |
| `frame_id` | 可选，覆盖该点坐标系 |
| `position` | `[x, y, z]`，单位米 |
| `in_slot` | 布尔值，表示线是否在槽内；控制段分类 |
| `role` | 阅读和调试标签，不决定当前段分类 |
| `metadata.gripper` | 可选的卡线阶段夹爪操作 |

相邻点 `in_slot` 相同生成 `straighten`，变化生成 `seat_edge`。不要给布尔值加引号。任务层不自动进行 tf2 转换，同一段两端必须在同一坐标系下。

场景以 `world` 放置，任务默认以左臂基座 `left_fr3_link0` 表示。修改 `task_frame` 不会转换 YAML 中已写好的坐标，也不会覆盖显式的 `default_frame`。候选槽位草图见 [layout.md](../layout.md)，其中采样高度不代表当前任务高度。

关键点不接受 `rpy`。任务级 `tool_roll=π`、`tool_pitch=0`；yaw 根据路径切向计算，`forward` 沿点索引递增方向，`reverse` 朝相反方向。默认 leader 为 `reverse`，follower 为 `forward`。

## 双臂与准备动作

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `initial_leader_index` / `initial_follower_index` | `1` / `0` | 双臂初始关键点索引，从 0 开始 |
| `leader_group` / `follower_group` | `left_fr3_arm` / `right_fr3_arm` | 左右臂任务分工 |
| `leader_ik_frame` / `follower_ik_frame` | `left_fr3_hand_tcp` / `right_fr3_hand_tcp` | 目标 TCP |
| `preparation_enabled` | `true` | 编译准备阶段 |
| `preparation_height` | `0.05` m | 初始关键点上方偏移及同步下降距离 |
| `preparation_interactive` | `true` | 两次闭合和下降前确认 |
| `preparation_leader_gripper_profile` | `cable_tip` | leader 准备夹持配置 |
| `preparation_follower_gripper_profile` | `cable_body` | follower 准备夹持配置 |

默认准备流程先定位 leader，再定位 follower，然后依次闭合并同步下降。ManiSkill 线缆模式要求准备动作、双 Franka 夹爪和默认左右 TCP 分工；其准备宽度根据线缆配置覆盖为左侧 `2 × usb.finger_position`、右侧 `0`。详情见 [MTC 线缆接口](../../dual_fr3_maniskill/docs/mtc_cable.md)。

## 夹爪配置

`gripper_profiles_file` 默认指向 [gripper_profiles.yaml](../config/gripper_profiles.yaml)。`width` 始终是两指**总开口**，单位米；`speed` 单位 m/s，`force` 单位 N。配置提供 `move`、`grasp` 和 `hold` 动作，所有覆盖值都受 `safety` 限制。

默认 `cable_tip` 使用 `grasp`，开口 0.005 m、速度 0.02 m/s、力 10 N；`cable_body` 使用 `grasp`，开口 0 m、速度 0.02 m/s、力 12 N。这些是联调初值，真机需按实际工件标定。

要在某个 `seat_edge` 目标点插入夹爪动作，在该点加入：

```yaml
metadata:
  gripper:
    enabled: true
    actor: follower
    profile: trunk_edge
    # width: 0.018  # 可选覆盖总开口，仍受 safety 限制
```

`execute:=false` 时用手指关节目标参与规划预览；实际执行通过对应后端的夹爪 action 完成。此类任务应保留 `execute_stage_by_stage:=true`。

## 规划与重试参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `cartesian_step_size` | `0.001` m | 笛卡尔采样步长 |
| `cartesian_jump_threshold` | `2.5` | 相对关节跳变检测倍数，必须大于 1 |
| `cartesian_path_tolerance` | `0.01` m | 直线偏离或原地转向时 TCP 漂移上限 |
| `motion_velocity_scaling` / `motion_acceleration_scaling` | `0.2` / `0.2` | 速度与加速度比例 |
| `leader_lead_distance` | `0.10` m | `seat_edge` 中 leader 沿切向让位距离 |
| `anchor_max_path_z` | `0.5` m | leader 换锚点路径的 TCP 高度上限，以关键点坐标系计 |
| `anchor_max_path_length_ratio` | `1.5` | 换锚点 TCP 路程 / 起终点直线距离上限 |
| `planning_attempts` | `10` | 完整规划或恢复规划的尝试上限 |
| `execution_replan_attempts` | `2` | 可恢复执行失败后的重规划上限，0 表示关闭 |

启用准备动作时，先搜索不同关节姿态并检查整条后续路径：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `preparation_ik_candidates` | `8` | 每臂最多保留的 IK 候选数 |
| `preparation_ik_attempts` | `80` | 每臂最多尝试的 IK 初值数 |
| `preparation_ik_timeout` | `0.05` s | 单次 IK 时间上限 |
| `preparation_min_joint_distance` | `0.3` rad | 同臂候选关节向量去重距离 |
| `preparation_candidate_attempts` | `2` | 遍历候选组合的最多轮数 |
| `preparation_search_timeout` | `180.0` s | 搜索总预算，在原生求解调用之间检查 |

这些候选保持相同 TCP 目标，改变肘部、腕部等关节配置。PipelinePlanner 失败可在同一组合内重试；Cartesian 失败会转向其他组合。单次原生求解可能超过剩余预算。

执行时长放宽参数 `trajectory_execution_duration_scaling=10.0` 和 `trajectory_execution_goal_margin=5.0` 控制 MoveIt 超时判定，不控制物理仿真速度。

完整参数及更多调试开关以启动文件为准：

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py --show-args
```
