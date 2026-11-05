# 文档导航

当前实现依据 [原始实现方案](prebuild_docs/GCA_ROS2_IMPLEMENTATION_PLAN.md)，结合 [系统架构](prebuild_docs/ROS_SYSTEM_ARCHITECTURE.md) 与 [GCA 原理](prebuild_docs/GCA_ARCHITECTURE.md)。原文保存在 `prebuild_docs/`；下列文档描述实际代码与运行入口。

| 读者／顺序 | 文档 | 解决的问题 |
|---|---|---|
| 所有人，先读 | [状态与证据](IMPLEMENTATION_STATUS.md) | 哪些已经实现和运行，哪些仍待实物测量 |
| 新使用者 1 | [根 README](../README.md) | 安装、构建、无硬件运行 |
| 新使用者 2 | [用户指南](USER_GUIDE.md) | 控制权、参考、ARM、动作、停止与恢复 |
| 新使用者 3 | [仿真与测试](SIMULATION_AND_TESTING.md) | 如何读原始记录、曲线、错误和测试报告 |
| 维护者 1 | [架构](ARCHITECTURE.md) | 模块、状态写入、I/O 所有权和停止路径 |
| 维护者 2 | [ROS 接口](ROS_INTERFACES.md) | 单位、IDL、权限、Action 和错误语义 |
| 维护者 3 | [开发指南](DEVELOPMENT.md) | 扩展、调试、测试、普通安装与代码规范 |
| 联调者 1 | [配置与标定](CONFIGURATION_AND_CALIBRATION.md) | 配置版本、参数范围、参考和比例 |
| 联调者 2 | [SDK 迁移](SDK_MIGRATION.md) | 旧到新映射、设备适配器与能力门控 |
| 联调者 3 | [硬件接线](HARDWARE_WIRING.md) | 引脚映射、电气条件、资料缺口 |
| 联调者 4 | [HIL 验收](HIL_ACCEPTANCE.md) | H01–H03 的设备、步骤、仪器与证据 |
| 所有人 | [第三方许可](THIRD_PARTY_NOTICES.md) | 依赖版本、来源与未明确的项目许可 |

软件测试通过不能代替电气接线、脉冲实测或停止距离测量。当前默认入口只启用模拟设备；硬件模板中的 `null` 和 `verified: false` 是明确拒绝启用的条件。
