设计文档入口
============

请从 [磁控导管系统详细设计](ROS_SYSTEM_ARCHITECTURE.md) 开始阅读。这是上位机、树莓派GCA和Windows磁驱动器的系统级主方案，版本为v0.2详细设计草案。

针对当前树莓派上的推进与旋转开发，接着阅读 [基于ROS 2的双轴实现方案](GCA_ROS2_IMPLEMENTATION_PLAN.md)。该方案结合旧代码审查和三个agent的交叉讨论，明确ROS 2正式运行入口、20项旧问题的修复要求、接口、并发与验收路径。

启动后续工程实现时，使用 [工程生成与代码编写Goal Prompt](GCA_ROS2_IMPLEMENTATION_GOAL_PROMPT.md)，按SDK／后端迁移、ROS 2与算法、无实物闭环、接线说明和详细文档五个阶段执行，并按证据验收。

| 文档 | 内容 |
|---|---|
| [系统详细设计](ROS_SYSTEM_ARCHITECTURE.md) | 需求、部署、模块职责、ROS接口、Windows协议、控制权、任务状态机、时间预算、配置、验收和工作包 |
| [ROS 2双轴实现方案](GCA_ROS2_IMPLEMENTATION_PLAN.md) | 当前Pi运行基线、旧代码问题与回归矩阵、Lifecycle/Action、双轴控制、停止监督、包布局与开发阶段 |
| [工程实现Goal Prompt](GCA_ROS2_IMPLEMENTATION_GOAL_PROMPT.md) | 可直接执行的五阶段开发任务、模块化与注释要求、代码／测试／文档产物、完成门槛 |
| [GCA设备核心设计](GCA_ARCHITECTURE.md) | 推进闭环、旋转开环、本地采集与输出、PID、脉冲后端、设备驱动和标定 |

这些文档描述待实现的软件设计，问题清单不表示缺陷已修复。Windows外部接口存在已由用户确认；具体SDK/API、实测电流能力和硬件性能参数仍需核实。文中的示例数据不作为实机运行参数。
