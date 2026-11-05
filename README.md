# GCA ROS 2 双轴运动工程

树莓派上的磁控导管／导丝轴向推进与绕轴旋转软件。推进由外部编码器反馈形成位置外环，输出舵机速度；旋转由有限 PUL/DIR 波形驱动，累计角度来自已确认脉冲，属于开环估计。

正式入口是 ROS 2：客户端 → `system_coordinator` → `gca_control_node` → 本地控制与设备 I/O。当前交付范围为 **GCA-only**。Windows 磁驱动与联合实验编排保留设计边界，没有用 GCA 动作替代磁场／电流控制。

本轮无硬件工程交付已完成：200项软件测试全部通过，五包独立普通安装及完整ROS仿真通过。逐项证据和已观测的调度限制见 [工程状态与证据](docs/IMPLEMENTATION_STATUS.md)。实机身份、电气接口、波形和停止性能仍按 H01–H03 独立验收，当前均为NOT_RUN。

## 无硬件快速运行

环境基线：Pi 4B、ARM64、Ubuntu 24.04、Python 3.12、ROS 2 Jazzy。下面命令均从项目根目录执行：

```bash
bash tools/bootstrap_dev.sh
bash tools/verify_software.sh
bash tools/run_simulation_suite.sh --output artifacts/simulation/latest
```

`bootstrap` 只准备项目 `.venv`。`verify` 串行构建五个包、运行静态检查和测试，再做独立普通安装与实际 ROS 进程仿真。最后一个入口可单独重复演示；失败返回非零码，报告仍保留。三个入口默认都不访问真实硬件。

查看 `artifacts/verification/latest/report.json` 和 `artifacts/simulation/latest/report.json`。仿真每个场景有客户端结果、原始状态 JSONL／CSV、曲线和 ROS 日志。报告中的 `physical_hil: NOT_RUN` 表示未作实机验收。

## 手动运行

构建完成后，每个新终端先执行：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
```

终端 A：`ros2 launch system_bringup simulation.launch.py`。启动保持 DISARMED，不会自动设参考或运动。

终端 B：`ros2 run gca_ros gca_cli shell`，按 [用户指南](docs/USER_GUIDE.md) 领取控制权、设参考、ARM、运动和停止。也可运行 `ros2 run gca_ros gca_cli demo --scenario basic --output artifacts/manual-demo`，显式执行模拟双轴动作。

## 目录与阅读路径

| 位置 | 作用 |
|---|---|
| `ros2_ws/src/gca_system_interfaces` | 编译生成的 msg／srv／action 定义 |
| `ros2_ws/src/gca_core/gca` | 唯一控制核心、设备适配器、独立仿真与记录器 |
| `ros2_ws/src/gca_ros` | Lifecycle、Action 桥接及 CLI |
| `ros2_ws/src/system_coordinator` | 控制会话、许可与系统动作入口 |
| `ros2_ws/src/system_bringup` | launch、仿真配置、禁止直接启用的硬件配置模板 |
| `tests`、各包 `test` | 综合回归、ROS 进程集成、算法与设备测试 |
| `tools` | 环境、构建、验证和报告入口 |
| `legacy` | 只读参考；不参与包发现、运行时导入或测试收集 |
| `docs/prebuild_docs` | 实现前的设计与旧代码审查原文 |
| `artifacts` | 可再生成的运行证据，不能作为源码或实测标定 |

新使用者读 [文档导航](docs/README.md) 和 [用户指南](docs/USER_GUIDE.md)；维护者读 [架构](docs/ARCHITECTURE.md)、[ROS 接口](docs/ROS_INTERFACES.md) 和 [开发指南](docs/DEVELOPMENT.md)；准备接实物时读 [迁移说明](docs/SDK_MIGRATION.md)、[接线说明](docs/HARDWARE_WIRING.md) 和 [HIL 验收](docs/HIL_ACCEPTANCE.md)。第三方依赖与项目许可状态见 [许可说明](docs/THIRD_PARTY_NOTICES.md)。
