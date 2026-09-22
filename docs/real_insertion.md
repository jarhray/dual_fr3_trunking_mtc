# 从 ManiSkill 迁移独立插入动作

本实现保留原来的“孔前接近 → 局部对齐 → 沿孔轴小步前进 → 阻力减速/停止”流程。
仿真和真机调用同一个 `InsertionPolicy`；真机使用 FR3 已有的关节反馈、内部外力估计和夹爪开度，不要求新增传感器。
这是一版已完成离线验证的开发实现，尚未连接真机验证控制切换、时序和接触行为。

## 已实现的边界

| 部分 | 实现 |
| --- | --- |
| 共享策略 | `dual_fr3_maniskill/usb/insertion.py`，默认保留原仿真行为 |
| 共用估计观测 | `dual_fr3_maniskill/usb/observation.py`，标定关系、时效、开度、力坐标变换 |
| 真机局部闭环 | `insertion_task/real_session.py`、`local_kinematics.py`、`real_runtime.py` |
| 关节目标执行器 | `dual_fr3_moveit_config/JointTargetController`，effort PD 跟踪、超时停止、会话握手 |
| 独立调度 | `insertion_skill.launch.py backend:=real`，复用接近规划与服务序列 |
| 记录与回放 | `real_runtime.record_path`、`replay_insertion.py`，回放策略决策，不发送机器人指令 |

插入沿用现有三个包的分工：`dual_fr3_maniskill` 保存共享策略与仿真适配，
`dual_fr3_trunking_mtc/insertion_task` 负责任务编排和真机局部闭环，
`dual_fr3_moveit_config` 负责机器人环境、控制接口与执行器。执行器源码位于
`src/controllers/`，ROS 消息位于 `msg/`，没有单独的插入控制器包。
既有服务和话题名称保持不变；消息类型由 `dual_fr3_moveit_config/msg/JointTarget`
和 `dual_fr3_moveit_config/msg/JointTargetState` 提供。

真机和估计观测仿真最终只报告 `estimated_reached`：估计深度达标且保持，
`physical_success_verified` 始终为 false。真机还必须收到执行器新鲜的 `STOPPED` 状态。
第一阶段保持左夹爪闭合和执行器接管，不执行仿真的固定约束、松爪或左臂回位。
固定抓姿是假设，夹爪开度不能证明没有微小滑移；本实现没有自动搜孔。

完整走线入口 `mtc_prototype.launch.py` 的真机自动收尾暂未接入插入。
目前在已有双臂夹持/走线完成后，使用下述独立入口；不要在同一双臂上同时运行其他运动任务。

## 1. 构建与配置自检

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install \
  --packages-up-to dual_fr3_trunking_mtc \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash

/usr/bin/python3 install/dual_fr3_trunking_mtc/lib/dual_fr3_trunking_mtc/real_insertion_runtime.py \
  --config "$PWD/src/dual_fr3_trunking_mtc/config/real_insertion.example.yaml" --check-config
```

示例输出中 `config_valid: true` 只表示结构合法；模板仍应显示 `calibration: unverified`
和 `limits_verified: false`。这个命令不初始化 ROS 通信。

复制 [真机配置模板](../config/real_insertion.example.yaml) 和
[执行器配置模板](../../dual_fr3_moveit_config/config/left_insertion_controller.example.yaml)
作为实际配置。保留原模板，逐项填写：

- `world_T_socket`：插座 CAD 原点在机器人世界坐标中的位姿。
- `tcp_T_usb`：当前固定抓姿下，插头 CAD 原点相对 `left_fr3_hand_tcp` 的位姿。
- `tip_in_usb_m`、`hole_center_m`、实际孔深与间隙：须与使用的网格几何一致。
- `world_T_base`、`ee_T_tcp`：libfranka 基座/末端坐标到世界/TCP 的转换；运行时会与 URDF 正运动学交叉检查。
- 开度范围、标定不确定度、力偏置和方向、速度/力矩/延迟限值：按实际装配与无接触测试结果填写。

变换采用列向量，`A_T_B` 将 B 坐标变换到 A 坐标；估计关系为
`world_T_usb = world_T_tcp @ tcp_T_usb`。插入沿插座坐标 **−X**，反向 **+X** 力视为阻力。
力偏置 `wrench_bias` 用世界坐标表达、参考点为 K，先扣偏置再乘 `wrench_sign`，随后换算到孔口。
模板的速度/阻力系数意味着约 2.5 N 时前进速度已降为零，低于 5 N 轴向故障阈值；
这不是实际连接器的推荐参数，需要用实际插入力判断是否会过早停滞。

## 2. 先在仿真中使用同一种观测

将 [估计观测配置片段](../../dual_fr3_maniskill/config/insertion_calibrated_estimate.example.yaml)
的 `insertion` 字段合并到现有完整线缆 YAML，填写对应标定数据；片段不能单独作为线缆场景加载。
用合并后的完整 YAML 启动原 ManiSkill 流程，再调用独立插入。

该模式的控制位姿来自标定关系，真实物体状态只放入 `evaluation_only` 用于离线比较。
仿真的接触力仍来自仿真代理，真机来自 FR3 内部估计，两者误差与带宽并不相同。
先验证相同配置下的对齐、减速、卡滞和到位保持，再进行真机接触调试。

## 3. 真机只读观测与规划

前提是已有真机 ROS/MoveIt 环境、原控制器和夹爪节点运行，已完成实际夹持，
MoveGroup 加载 `move_group/ExecuteTaskSolutionCapability`，且当前没有其他运动任务。
本入口不会启动第二套硬件，也不会做夹爪 Homing。

```bash
ros2 launch dual_fr3_trunking_mtc real_insertion_runtime.launch.py \
  config:=/absolute/path/real_insertion.yaml

# 在另一个已 source 的终端读取反馈
ros2 service call /real/usb/insertion/status std_srvs/srv/Trigger '{}'

# 仅规划右臂释放退出/回位、左臂孔前接近与插入碰撞预检
ros2 launch dual_fr3_trunking_mtc insertion_skill.launch.py \
  backend:=real cable_config:=/absolute/path/real_insertion.yaml execute:=false
```

只读 runtime 不切换控制器或发布运动目标。规划会更新 MoveIt 中的插头附着和插座碰撞对象。
检查 `observation_valid`、位姿、夹爪开度、力方向和反馈年龄；接口名可在 `real_runtime` 中覆盖。
右臂与右夹爪分别检查原始话题新鲜度，不把聚合 `/joint_states` 刷新视为右侧源仍正常。
两端配置指纹必须一致，修改配置后需重启 runtime。

## 4. 标定及无接触验证后启用独立执行

实际校验完成后才在自己的配置中设置 `calibration.verified`、`grasp_assumption_valid`、
`real_runtime.limits_verified`，以及执行器的 `enabled`、`calibration_verified`、
`runtime_limits_verified`。不确定度须满足配置限制；仅改布尔开关不代表标定完成。

以下命令是后续调试入口，本次未执行：

```bash
ros2 launch dual_fr3_trunking_mtc real_insertion_runtime.launch.py \
  config:=/absolute/path/real_insertion.yaml allow_execution:=true \
  load_executor:=true executor_config:=/absolute/path/left_insertion_controller.yaml

# 确认执行器已加载为 inactive 后，在另一终端启动独立流程
ros2 launch dual_fr3_trunking_mtc insertion_skill.launch.py \
  backend:=real cable_config:=/absolute/path/real_insertion.yaml execute:=true
```

执行顺序为：完整规划 → 运行前检查 → 右爪打开 → 右臂退出/回位 → 左臂孔前接近并停稳
→ 严格切换左臂控制器 → 实测关节首包握手 → 共用对齐/插入策略 → 执行器停稳并保持。
局部目标逐步经过 MoveIt 碰撞检查，只允许插头与插座的接触对；碰撞场景中的几何位姿需与标定一致。
局部循环默认 50 Hz，但不是已验证的实时保证；IK、服务或反馈超过期限会停止。

主动请求停止：

```bash
ros2 service call /real/usb/insertion/cancel std_srvs/srv/Trigger '{}'
ros2 service call /real/usb/insertion/status std_srvs/srv/Trigger '{}'
```

停止请求响应不等于已停稳，应检查 `executor_state` 和 `stop_acknowledged`。
执行器仍保持 effort 接管；停止不是卸力、回撤或急停。反馈无效时不会宣称已停稳。
没有自动重试、自动切回旧轨迹控制器或自动松爪。
恢复前需处理当前接触状态，确保后继控制器从当前姿态保持、没有旧轨迹等待执行，再重新启动会话。
执行器行为和反馈丢失边界见 [控制器说明](../../dual_fr3_moveit_config/docs/insertion_controller.md)。

## 5. 记录与离线回放

每次尝试使用独立的 `real_runtime.record_path`，例如 `/tmp/insertion_attempt_001.jsonl`。
其中 `policy_event` 记录策略观测和调用顺序，`runtime_snapshot` 记录执行器状态。

```bash
/usr/bin/python3 install/dual_fr3_trunking_mtc/lib/dual_fr3_trunking_mtc/replay_insertion.py \
  --config /absolute/path/real_insertion.yaml \
  --input /tmp/insertion_attempt_001.jsonl --output /tmp/insertion_policy_replay.jsonl
```

回放复现策略计算、对齐累计步长和估计到位决策，不复现硬件运动、碰撞服务或执行器停止确认。
外部故障和最终是否停稳应同时查看原始 `runtime_snapshot`；策略回放结果不代表物理插入成功。
