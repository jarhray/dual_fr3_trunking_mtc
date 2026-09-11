# 维护文档入口

当前使用与维护说明已按主题整理。旧的逐次调试结论、临时日志路径和待办列表不再作为当前功能说明；需要追溯时可查看本文件的 Git 历史。

| 任务 | 文档 |
| --- | --- |
| 构建、启动、预览和执行 | [README](README.md) |
| 修改关键点、朝向、夹爪和规划参数 | [任务配置](docs/configuration.md) |
| 查就绪检查、执行恢复和 ROS 接口 | [执行与排查](docs/execution.md) |
| 了解模块边界和新增阶段的位置 | [架构说明](ARCHITECTURE.md) |
| 核对机器人坐标与后端控制器 | [MoveIt 后端说明](../dual_fr3_moveit_config/docs/backends.md) |
| 安装仿真依赖并运行检查 | [ManiSkill 环境与验证](../dual_fr3_maniskill/docs/setup.md) |
| 维护准备阶段线缆 | [MTC 线缆接口](../dual_fr3_maniskill/docs/mtc_cable.md) |

更新功能时，默认值以 `runtime/config.py` 和 launch 为准，配置字段以解析代码为准。离线测试、GPU 物理、完整仿真执行与真机验证分别记录，避免把某一层的成功推断为完整任务成功。
