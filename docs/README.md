# 文档导航

当前实现依据 [原始实现方案](prebuild_docs/GCA_ROS2_IMPLEMENTATION_PLAN.md)，结合 [系统架构](prebuild_docs/ROS_SYSTEM_ARCHITECTURE.md) 与 [GCA 原理](prebuild_docs/GCA_ARCHITECTURE.md)。原文保存在 `prebuild_docs/`；下列文档描述实际代码与运行入口。

| 读者／顺序 | 文档 | 解决的问题 |
|---|---|---|
| 所有人，先读 | [状态与证据](IMPLEMENTATION_STATUS.md) | 哪些已经实现和运行，哪些仍待实物测量 |
| 新使用者 1 | [根 README](../README.md) | 安装、构建、无硬件运行 |
| 新使用者 2 | [用户指南](USER_GUIDE.md) | 控制权、参考、ARM、动作、停止与恢复 |
| 新使用者 3 | [仿真与测试](SIMULATION_AND_TESTING.md) | 如何读原始记录、曲线、错误和测试报告 |
| 手柄使用者 | [手柄控制启用与测试指南](TELEOP_USER_GUIDE.md) | 区分真实舵机与原双轴模拟入口；终端读取、动作与偏移文本记录、停止退出与故障处理 |
| 手柄维护者 | [JF/JT追踪](TELEOP_TRACEABILITY.md) | 修复源码、具体回归和软件/实体/HIL证据边界 |
| 手柄接入设计 | [Xbox按键约定](TELEOP_DESIGN.md) | 原始左摇杆/LT/B与右侧磁场预留约定；当前操作读手柄指南 |
| 舵机使用者 | [真实舵机快速操作](SERVO_USER_GUIDE.md#真实舵机快速操作) | 两个终端启动、LT＋左杆正反转与调速、B 后恢复、快速重复操作、停止退出和文本记录 |
| 舵机接入设计 | [舵机 ROS 2 与位移闭环方案](SERVO_ROS2_INTEGRATION_PLAN.md) | 手柄独立调速、纯 Python 配置、仅舵机接入及后续位移 PID 的设计依据；实际结果见状态文档 |
| 舵机开发任务 | [真实舵机接入 goal prompt](SERVO_ROS2_IMPLEMENTATION_GOAL_PROMPT.md) | 实现范围、配置与接口、有限验证及软件／实机完成标准；保存 prompt 不代表开始执行 |
| 手柄实施准备 | [实现与可控性验证方案](TELEOP_IMPLEMENTATION_PLAN.md) | 实现依据、原始JF缺陷与JT01–JT20预声明判据；执行状态见状态文档 |
| 手柄开发任务 | [实现goal prompt](TELEOP_IMPLEMENTATION_GOAL_PROMPT.md) | 本轮多agent实施的任务、约束和完成标准；实际结果另由报告确认 |
| 维护者 1 | [架构](ARCHITECTURE.md) | 模块、状态写入、I/O 所有权和停止路径 |
| 维护者 2 | [ROS 接口](ROS_INTERFACES.md) | 单位、IDL、权限、Action 和错误语义 |
| 维护者 3 | [开发指南](DEVELOPMENT.md) | 扩展、调试、测试、普通安装与代码规范 |
| 联调者 1 | [配置与标定](CONFIGURATION_AND_CALIBRATION.md) | 配置版本、参数范围、参考和比例 |
| 联调者 2 | [SDK 迁移](SDK_MIGRATION.md) | 旧到新映射、设备适配器与能力门控 |
| 联调者 3 | [硬件接线](HARDWARE_WIRING.md) | 引脚映射、电气条件、资料缺口 |
| 联调者 4 | [HIL 验收](HIL_ACCEPTANCE.md) | H01–H03 的设备、步骤、仪器与证据 |
| 所有人 | [第三方许可](THIRD_PARTY_NOTICES.md) | 依赖版本、来源与未明确的项目许可 |

软件测试通过不能代替电气接线、脉冲实测或停止距离测量。当前默认入口只启用模拟设备；硬件模板中的 `null` 和 `verified: false` 是明确拒绝启用的条件。

仅舵机实现已完成必要软件验证和空载手柄实机联调：上推反转、下推正转、LT 持续使能，指令上限 80 rpm、加速度 40 rpm/s，最近实测约 ±45.8 rpm。当前接收器为 BFM 模式，输入使用显式 `joydev_hold`。无线资格仍未完成，两项专项失效测试按用户决定移至后续；前次偶发中断保留为未复现。外部编码器／PID 仅预留接口，位置能力关闭。完整证据与范围见[实现状态](IMPLEMENTATION_STATUS.md)，不同阶段的测试数量不累加。
