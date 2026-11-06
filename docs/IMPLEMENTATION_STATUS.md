# 工程实现状态与证据

## 新增功能：ROS 2 手柄动作与偏移文本记录

2026-11-06按用户明确的新功能要求，扩展 `record_directory`：输入事件记录原始杆量、中心偏移、曲线前后归一化值、映射动作与门控后的速度请求；10 Hz 记录系统反馈及相对有效基线的推进位移、旋转估计角变化。参考变化或反馈失效不会沿用旧基线，旋转估计不标为实测角度。操作及 `.txt` 转换见[手柄记录指南](TELEOP_USER_GUIDE.md#将输入动作与模拟偏移写入文本文件)。

仅重新构建安装 `gca_teleop`，运行 8 项定向测试（包含 3 项新增记录测试），全部通过；静态与格式检查通过。一次合成输入的 ROS 冒烟检查在修正测试脚本启动顺序后通过：[报告](../artifacts/teleop_recording/smoke-fzv4cgkd/report.json)、[文本样例](../artifacts/teleop_recording/smoke-fzv4cgkd/joystick.txt)。该次保留 143 条输入、60 条可用反馈采样，观测到推进和旋转偏移，记录器未报告丢弃或写入错误；STOP 确认、DISARM、释放控制权和进程退出均成功。初次检查因合成输入未启动而等待 READY 超时的记录保留于 `artifacts/teleop_recording/smoke-omcxk5m3/`。

以上仅验证新增记录链路，不代表新的实体手柄操作记录或完整工程验收；未重跑全套测试，原完整验收目标继续暂停。下面的 491／436 项结果与源码指纹属于此前阶段，不覆盖本次增量。

## 上一阶段：已暂停进一步验收

2026-11-06按用户要求停止追加与重复测试，转入项目总结和下一功能规划。后台验收在全新普通安装阶段收到SIGINT，子进程清理完成、无残留；不自动重启。该阶段源码指纹为 `5115a27789a5c119fb504e5ed52b2a6e5f38b3fb06ca768d8fa312af5b514153`。

| 当前状态 | 已确认结果 |
|---|---|
| 已实现 | ROS 2手柄推进/旋转、同owner管理、输入质量/停车门控；独立于ROS的终端输入监视器 |
| 当前构建与测试 | 静态检查、格式检查、六包构建通过；491项pytest与436项包级测试均0失败/错误/跳过，两组数量重叠 |
| 本次修复 | 推进速度PI、旋转相位/有限边沿、输入原截止、STOP/ARM保护；追加修复心跳时间快照误判及owner/代次切换竞态 |
| 独立手柄读取 | `python3 tools/joystick_monitor.py`无需ROS；加已标定profile显示LX/LY/LT/B、速度意图与门控原因，仅读取、不驱动执行器 |
| 实体证据 | 15步只读映射通过；实体→simulation仅确认短暂推进，旋转及持续双轴操作未通过 |
| 尚待验证 | 本次新源码的全新普通安装与完整DDS/模型复验未完成；无线资格、执行器H01–H03仍NOT_RUN |

当前[验证报告](../artifacts/verification/latest/report.json)记录已通过步骤与用户主动中断，整体`passed=false`不代表491项测试失败，也不能写成整体验收通过。[暂停记录](../artifacts/joystick_monitor/current_verification.json)保留停止原因；[手柄操作指南](TELEOP_USER_GUIDE.md)给出独立脚本与ROS入口。后续只针对新功能及受影响接口做必要验证，完整回归留到发布或重要集成节点，不再为整理文档重复运行。

## 历史已通过快照：手柄初版

以下455项及其完整仿真结果属于2026-11-05快照，不覆盖之后的独立监视器与协调器修复；以首节作为当前状态。



**该历史快照的无执行器硬件软件验收曾通过。** 2026-11-05 21:09（Asia/Shanghai）完成的统一运行包含455项必需软件测试、六包普通安装、旧仿真回归，以及手柄9个真实ROS/DDS场景和23个独立模型场景。实体完整映射已有只读证据；实体输入→simulation另行验证，无线资格及执行器H01–H03仍NOT_RUN。

### 初版实现与历史证据

`gca_teleop` 已实现只读joydev与显式合成源、严格profile、映射/门控、单一MANUAL owner、容量1异步派发、独立STOP本机代理与安装入口。正式路径不绕过协调器/双层许可。JF01–JF07已写入核心：外部编码器速度PI、真实dt/抗饱和、连续旋转phase/余量、初态、非均匀有限边沿、原输入截止及STOP/ARM闸门。源码与精确测试定位见 [追踪表](TELEOP_TRACEABILITY.md)，操作见 [手柄指南](TELEOP_USER_GUIDE.md)。

| 本轮证据 | 实际结果与范围 |
|---|---|
| [统一验证报告](../artifacts/verification/pre-monitor-20261105T130942Z/report.json) | `passed=true`；10步退出码全为0；每步清理完成且无残留 |
| [完整pytest](../artifacts/verification/pre-monitor-20261105T130942Z/pytest.xml) | 455项，0失败/错误/跳过；JUnit 166.667s，命令墙钟169.760s |
| [colcon包级结果](../artifacts/verification/pre-monitor-20261105T130942Z/colcon-results.log) | 402项，0失败/错误/跳过；与455项重叠，不累加 |
| [普通安装资源检查](../artifacts/verification/pre-monitor-20261105T130942Z/installed-resources.log) | 六包独立普通安装；源码树外检查安装模块、三个手柄入口、launch与配置，命令13.819s |
| [旧仿真套件](../artifacts/simulation/pre-monitor-20261105T130942Z/report.json) | 5个真实ROS场景及5个独立模型场景均通过 |
| [手柄完整套件](../artifacts/teleop/pre-monitor-20261105T130942Z/report.json) | 9个真实ROS/DDS场景全部通过，`full_acceptance=true`；[模型JUnit](../artifacts/teleop/pre-monitor-20261105T130942Z/model/junit.xml)23项全通过 |
| [最终bootstrap复核](../artifacts/teleop_implementation/bootstrap-final/report.json) | 实际执行 `bash tools/bootstrap_dev.sh` 退出0，前后环境与依赖记录一致 |

本次独立普通安装为 `artifacts/clean-install/20261105T125134568374Z/install`。统一源码清单 [sources.json](../artifacts/verification/pre-monitor-20261105T130942Z/sources.json) 的摘要为 `838e3e5e4ca9a49bc043eedb1b65d75f53029c35fd26076062a0ebb8c6c7b2cb`；验收期间源码与安装输入未变化。正式入口从工程根运行 `bash tools/verify_software.sh` 和 `bash tools/run_teleop_suite.sh --output artifacts/teleop/latest`。

[独立模型报告](../artifacts/teleop/pre-monitor-20261105T130942Z/model/summary.json)使用teleop-controllability-v3：23场景覆盖正反推进、滑移、噪声/延迟、死区、饱和解除、细动/跨段旋转、失步对照和长期采样。长时场景实际运行两次1800s刺激，各再观察1.5s、保留18018行；0.1s采样及中立边界的哈希均为 `bc9c3ff88c23f9506b29018b11dfeab59e9b9af64fe3c8930aa942e72d717ebd`。最大30s周期线漂移约0.000397mm（门槛0.02mm），角漂移1脉冲；稳定后1s线漂移约3.95e-10mm，无失步plant与完成计数最大差约4.69e-12脉冲。这里是确定性模型时间与数值真值，不是Pi实时或机械精度实测。

开发专项与失败现场保留在 `artifacts/teleop_implementation/`。核心teleop当前53项已包含在本次455项中，包括 [正负软限位及未结算预约4项专项](../artifacts/teleop_implementation/core/reservation_tests.xml)；早期49项、双60s或单次1800s快照不再用于证明本轮完整验收。独立plant的瞬态超调仍如实保留，不把稳态门槛通过写成无超调。

统一验证中断时曾留下测试启动的独立ROS子组，现场已按身份清理并保留在 `artifacts/verification/pre-reservation-regression-20261105T1050Z/`。`tools/verify_software.py` 现通过本次命令的继承标识与PID/starttime跟踪后代，超时、中断或结束时进行有界清理；正常父进程退出但有残留也判验证失败。对应 [7项清理回归](../artifacts/teleop_implementation/verification-cleanup.xml) 已实际通过（0失败/错误/跳过，5.797s），覆盖独立子组、升级终止、不影响无关进程和身份边界。这些回归也已纳入本次455项统一验收；原专项报告保留供复核。

已识别`20bc:506d`北通2.4GHz接收器，可读joydev。实体15步原始输入已采集，schema2映射重算复核PASS，证据和15份原始文件SHA见 [mapping.schema2.validation.json](../artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.validation.json)。LX=0、LY=1且反向、LT=5、B=1；右侧预留axes[2,3,4]/buttons[7,14]，8/9未分配。该通过范围仅为真实只读映射。操作者明确表示模式不清楚，记录标签`operator_reported_unknown_2.4GHz`；无线资格仍NOT_RUN，默认未核实profile继续null锁定。实体输入→simulation首启因schema2重算约0.18–0.20s超过reader100ms预算而失败，未开始motion；[attempt1](../artifacts/teleop_implementation/input/physical_simulation.attempt1/report.json)现场保留。早先的后续尝试已结束，[report.json](../artifacts/teleop_implementation/input/physical_simulation/report.json)明确为passed=false / NOT_PASSED_NO_MOTION_OBSERVED：保留的输入raw最大event_seq=0、changed_frames=0，两个stage完整DDS的接受速度/编码器速度/脉冲均0。原因尚未确定，reader健康不证明无线在线。已显式STOP确认、release，launch退出0，errors=[]；输入日志有界轮转可能淘汰早期帧，完整observations.jsonl保留。清理成功不把未观察到动作改为通过。之后重新打开输入设备的 [LT初始状态快照](../artifacts/teleop_implementation/input/lt_held_snapshot.json) 读到轴5为+32767、附带按钮8为1，但event_seq=0且无变化事件；它只证明该次打开时的缓存基线，不能追溯证明上一轮已接收到按下事件或发生运动。右侧RT/RB/右杆只预留磁场，本轮没有磁场驱动或联合停机实现。

第三轮 [physical_simulation.attempt3](../artifacts/teleop_implementation/input/physical_simulation.attempt3/report.json) 已关闭，结论为 `NOT_PASSED_OPERATOR_ACTION_UNCONFIRMED`：十分钟等待窗口结束前未收到所请求的LT完成回复，raw只有2个INIT基线、0个变化事件，DDS接受输出、编码器速度与确认脉冲均0。报告保留 `OPERATOR_WINDOW_EXPIRED_NO_ASSUMED_ACTION`，不能把经过时间当作用户已操作，也不能判实体运动通过。已确认STOP、接受release、launch退出0，所有该轮模拟ROS节点已关闭。

随后两次只读采集均已关闭。[live_lt_probe](../artifacts/teleop_implementation/input/live_lt_probe/report.json) 只保存到4个LX变化，没有LT变化；[live_lt_probe_restart_1](../artifacts/teleop_implementation/input/live_lt_probe_restart_1/report.json) 共保存123个变化记录，其中实际LT新按达到+32767、再松开至−15540，归一化0.262871低于释放阈值0.4。此次只确认按下/释放阈值跨越，未在该次按下后观测到完全释放端点；不能把它扩展为运动链路或无线资格通过。

第四轮 [实体输入→simulation报告](../artifacts/teleop_implementation/input/physical_simulation.attempt4/report.json) 已关闭，[分阶段评估](../artifacts/teleop_implementation/input/physical_simulation.attempt4/assessment.json) 为 `passed=false / NOT_PASSED_NO_MOTION_OBSERVED`：两个推进stage的接受速度、编码器速度及确认脉冲均0，保留的 [16个基线/变化帧](../artifacts/teleop_implementation/input/physical_simulation.attempt4/changed_input.jsonl) 无丢弃，关键观察也无丢弃。第一stage `physical_linear_forward` 记录LT新按seq4，约161ms后才观测到首个LY变化（reader观察时间差160.866216ms），已超过0.15s的event_age期限；第二stage `physical_linear_preposition` 虽收到用户完成回复，窗口内却没有真实变化事件。无变化不能证明无线在线，也不能判运动通过。STOP确认、release成功、launch退出0，errors为空只表示清理无报告错误，不改变未运动结论。

[并行只读输入对照](../artifacts/teleop_implementation/input/parallel_input_probe.attempt2/report.json) 已关闭：[评估](../artifacts/teleop_implementation/input/parallel_input_probe.attempt2/assessment.json) 确认原始 `/dev/input/js0` 与安装后NativeJoydev各16个变化事件按序一致，包含B索引1的多次按放；errors为空、native_health_failures=0。用户已确认B按放完成，但指示灯状态仍未提供。event4/event5读取因权限不足而跳过，未修改权限。该对照只确认已收到事件的解析一致性，不测试运动，也不证明无线在线或无线资格。

第五轮已关闭，[分阶段评估](../artifacts/teleop_implementation/input/physical_simulation.attempt5/assessment.json)保留650个基线/变化记录，变化与关键观察无丢弃，STOP/release成功、launch退出0。实体手柄确实触发了短暂模拟推进：接受输出最高1.6025722mm/s，编码器速度最高0.1322488mm/s，位置由0增至0.05mm，随后event_age到期停车。两次旋转试验已收到最高30°/s意图，但确认脉冲和模拟角度仍为0，整体标为`PARTIAL_BRIEF_LINEAR_MOTION_OBSERVED_ROTATION_NOT_CONFIRMED`，不判整体通过。最后一次旋转的DDS记录表明core在其最后接受的Jog期限到期后先进入STOPPING，teleop随后报告CONTEXT_CHANGED；更新的原始输入期限不证明已送达core。当前转入独立终端输入诊断，所有此次模拟节点和临时只读采集均已关闭。

默认event_age静态长保持会到期并要求释放/回中/新LT；qualified_state需真实匹配资格，当前未启用实体资格。旋转意图为长期平均率，段内峰值/空档分列；未知abort作废旋转参考并锁住双轴手柄，需同owner显式恢复。有限profile软件时序通过不代表机械连续加速度、真实角度或停止距离通过。

## 历史无硬件工程基线（本轮手柄改动前）

上一阶段无硬件工程基线曾通过： 2026-11-05 14:14（Asia/Shanghai）完成：200项软件测试全部通过，0失败/错误/跳过；colcon包级177项全部通过（与200项重叠，不累加）；五包构建、独立普通安装、源码树外入口检查、5个真实ROS仿真场景及5个确定性模型场景均通过。命令、退出码和逐用例证据见下表。该历史结果不包含实机验收，不覆盖本轮JF手动Jog扩展，也不消除已记录的调度限制。

上一阶段执行完整工程goal，设计原文保持在 [prebuild_docs](prebuild_docs/README.md)。所有已执行开发测试使用模拟设备或注入SDK／串口／波形客户端；没有打开真实串口、设置GPIO、启动实际波形或运动。**H01–H03全部NOT_RUN。**


以下五阶段表、200项及对应源码指纹为上一阶段记录，保留用于理解B01–B20/T01–T18和历史调度限制。**下表的latest路径现已被本轮运行覆盖/更新，不再是旧200项或旧指纹的归档证据。** 旧数字仅保留为上一阶段的文字记录；不要用本轮新报告反向支撑旧200数字。当前本轮最终结果只在本文首节由最终报告确认，不能把历史通过推广到新增手柄。

## 上一阶段五阶段交付状态

| 阶段 | 实际已落地内容 | 当前验证/边界 |
|---|---|---|
| 1 SDK／设备迁移 | DYNAMIXEL、编码器、有限波形真实适配器与工厂，严格帧解析、显式生命周期、锁、失败路径及注入测试 | 设备/协议73项通过，包含在最终200项中；不是实机适配验收 |
| 2 ROS 2／控制算法 | 已编译IDL、gca_core、gca_ros、协调器、bringup；本地I/O所有者、轨迹/PID、状态机、Action与控制权 | 控制/模型77项、ROS/权限26项及其它集成测试通过；五包普通安装及实际ROS入口通过 |
| 3 无实物闭环 | 独立plant、模拟执行器/编码器、故障注入、真实DDS测试、JSONL/CSV及离线曲线工具 | basic/cancel/jog/hold/faults全部PASS；确定性模型5场景PASS；原始记录和图表已生成 |
| 4 接线说明 | [接线](HARDWARE_WIRING.md)、[配置标定](CONFIGURATION_AND_CALIBRATION.md)、[HIL](HIL_ACCEPTANCE.md)、不可运行硬件模板 | 来源与未知项明确；没有实物验证，不能直接据候选图上电 |
| 5 说明与交付 | [文档导航](README.md)、架构/SDK/开发/用户/配置/HIL/许可、统一工具、本文追踪矩阵 | 三个统一入口均已实际执行；最终verify全部9个步骤退出0；文档本地链接已核对 |

环境基线已重新检查：Pi 4B／ARM64／Ubuntu24.04／Python3.12／ROS2 Jazzy。项目使用`.venv`和经PyPI哈希核对的固定pip wheel引导，补充依赖包括DYNAMIXEL SDK4.1.0、pigpio1.78及ruff；系统Python与legacy不改写。实际版本见 [环境记录](../artifacts/environment.json)。导入库不代表打开硬件。

## 上一阶段开发证据与统一验收入口

| 证据 | 位置/实际结果 | 如何解释 |
|---|---|---|
| 开发单元测试快照 | [development-unit.xml](../artifacts/development-unit.xml)：当时116测试、0失败/错误/跳过 | 早期源码快照；后续新增测试以最终JUnit为准 |
| 最终设备专项 | `test_devices_*.py`、`test_protocol_encoder.py`：73项通过 | 已安装真实适配器的注入测试，含真正官方SDK短帧路径；属于最终200项 |
| 最终核心专项 | 算法、plant、估计、runtime：77项通过 | 属于最终200项；不与包级结果重复累加 |
| ROS basic开发流程 | [development-basic-4/basic/report.json](../artifacts/simulation/development-basic-4/basic/report.json)：PASS；最终推进5.01mm、旋转估计19.985625deg，目标5mm/20deg | 实际ROS进程、DDS与模拟plant；不是机械精度实测 |
| 其它开发场景 | [development-scenarios/report.json](../artifacts/simulation/development-scenarios/report.json)：cancel/jog/hold/faults均PASS | 有CLI和启动退出码、DDS记录、曲线；仍是development报告 |
| 历史ROS调试 | [development-ros.xml](../artifacts/development-ros.xml)保留早期失败快照 | 历史失败不删除；不能用它代表修复后的最终状态，也不冒充全通过 |
| 最终统一验证 | [verification/latest/report.json](../artifacts/verification/latest/report.json)、[pytest.xml](../artifacts/verification/latest/pytest.xml) | passed=true；全部9步退出0；200项0失败/错误/跳过，pytest用时126.60s |
| colcon包级结果 | [colcon-results.log](../artifacts/verification/latest/colcon-results.log) | 177项，0失败/错误/跳过；包含真实bringup失败传播回归 |
| 最终仿真套件 | [simulation/latest/report.json](../artifacts/simulation/latest/report.json)及各场景子目录 | 5个ROS场景及5个模型场景全部PASS；各CLI和launch均退出0 |
| 独立普通安装 | `artifacts/clean-install/20261105T060700711551Z/install`、[资源检查日志](../artifacts/verification/latest/installed-resources.log) | 普通构建退出0，用时369.40s；源码树外只加载该install，五包、CLI、launch和配置资源均PASS |

上一阶段验收的文字记录指纹为 `4b007c29b3226d9ef2a9e5654fd95db9d6b0f275948574022768828fc01128b2`；当前 [sources.json](../artifacts/verification/latest/sources.json) 已随本轮更新，不能作为这份旧指纹的逐文件证据。统一工具已检查验收期间源码未变化。basic场景目标为5 mm/20°，最终推进反馈5.01 mm、旋转脉冲估计19.985625°；这是模型运行结果，不是实机精度。主入口均从工程根运行：

```bash
bash tools/bootstrap_dev.sh
bash tools/verify_software.sh
bash tools/run_simulation_suite.sh --output artifacts/simulation/latest
```

`verify_software.sh`串行构建、静态检查、必需pytest、colcon结果、独立普通安装、安装资源和完整仿真；每步保留命令、退出码与日志。源码专项验证可用于快速修复，整体完成仍须通过该统一路径。

## 追踪表读取方式

以下别名均指实际源码和测试；表中引用为具体函数或明确文件测试集合，最终执行状态由JUnit关联，不凭文件存在判PASS。

| 别名 | 文件/目录 |
|---|---|
| D | `ros2_ws/src/gca_core/gca/devices/` |
| P | `ros2_ws/src/gca_core/gca/protocols/` |
| R | `ros2_ws/src/gca_core/test/test_runtime_control.py` |
| C | `ros2_ws/src/gca_core/test/test_control_math.py` |
| S | `ros2_ws/src/gca_core/test/test_simulation_plant.py` |
| DD | `ros2_ws/src/gca_core/test/test_devices_dynamixel.py` |
| DE | `ros2_ws/src/gca_core/test/test_devices_encoder.py` |
| DP | `ros2_ws/src/gca_core/test/test_devices_pigpio.py` |
| DF | `ros2_ws/src/gca_core/test/test_devices_factory.py` |
| DX | `ros2_ws/src/gca_core/test/test_devices_dependencies.py` |
| PE | `ros2_ws/src/gca_core/test/test_protocol_encoder.py` |
| I | `tests/test_ros_integration.py`，实际ROS进程/DDS |
| O | `tests/test_observability.py` |
| SEC | `ros2_ws/src/gca_ros/test/test_security.py`与`test_permits.py` |
| AUTH | `ros2_ws/src/system_coordinator/test/test_authority.py` |

## B01–B20：旧问题到实现及回归

本表表示修复代码和回归已建立；物理效果涉及的H项保持NOT_RUN，最新软件全套状态仍由最终报告决定。

| 编号/旧问题 | 实现位置与修复行为 | 具体回归证据 | 实机边界 |
|---|---|---|---|
| B01 阻塞move_pos | D/dynamixel.py速度执行，runtime/control.py本地状态推进 | R::test_unarmed_rejected_and_full_closed_loop_uses_plant、test_owner_deadlines_stop_when_control_task_frozen、test_cancel_waits_for_stop_confirmation | H03停止时间待测 |
| B02 dt/积分饱和 | algorithms/control.py真实dt、有限积分、最终输出/量化残差一次回算 | C::test_pid_true_dt_and_quantization_residual_settled_once、test_p_only_does_not_create_hidden_integral_from_limiting、test_saturation_reversal_and_reset | 新增益不继承为实机参数 |
| B03 同轮重复采集 | D/encoder.py一次采集一个seq，estimation/linear.py不可变有效快照 | DE::test_open_is_read_only_and_each_valid_reply_has_new_sequence；S::test_estimation_repeated_samples_and_first_velocity_invalid | H01内部更新率待测 |
| B04 CRC宽度 | P/encoder.py固定两字节小端 | PE::test_crc_golden_width_and_standard_vector，覆盖84 00 | 黄金报文软件通过 |
| B05 编码器帧/符号 | 严格长度、站号/功能码/异常/CRC、请求关联及失败后重同步 | PE::test_malformed_never_becomes_measurement、test_exception_and_buffer_limit_are_explicit、test_signed_count_and_no_invented_wrap_rule；DE::test_bad_or_incomplete_response_requires_resync、test_unsolicited_old_reply_is_not_new_feedback | 未知真实协议禁用 |
| B06 SDK短帧假数值 | D/sdk_support.py在官方便利函数解码前核对完整参数长度 | DD::test_official_sdk_checked_packet_rejects_legal_crc_without_data、test_limits_timeout_and_invalid_measurement_are_explicit | 真实SDK离线执行，不是复制其实现做预期 |
| B07 步进内层不退出 | 独立停止标志、有限段、明确abort和有期限关闭 | R::test_cancel_waits_for_stop_confirmation；DP::test_abort_never_invents_partial_count | H02尾部脉冲待测 |
| B08 sleep/假stop | D/pigpio_engine.py有限硬件定时波形、明确idle/abort | DP::test_finite_wave_timing_and_confirm_only_after_observed_completion、test_abort_never_invents_partial_count | GPIO时序/机械余运动H02/H03 |
| B09 估计计数/截断 | 绝对目标舍入、预约/确认分离、回执绑定一次结算 | C::test_absolute_pulse_rounding_preserves_small_moves；R::test_pulse_receipts_are_bound_to_segment_and_settled_only_once、test_wrong_epoch_or_payload_cannot_settle_pulse；DP::test_short_unobserved_wave_cannot_invent_completion | 仍是开环角估计 |
| B10 启动自动移动/清零 | open/configure/enable分离、身份核对、编码器普通生命周期只读 | DD::test_construct_and_open_do_not_enable_or_configure、test_identity_failure_closes_port；DE::test_open_is_read_only_and_each_valid_reply_has_new_sequence、test_reconnect_changes_epoch_and_resets_sequence | 实物遗留输出H01 |
| B11 TCP空包/部分准入 | 正式入口替换为有类型ROS Action、完整目标原子准入 | R::test_both_axis_validation_prevents_partial_motion；I::test_source_conflict_and_atomic_two_axis_rejection、test_active_is_not_arm_and_direct_device_token_cannot_be_source_token | 不保留旧TCP运动服务器兼容 |
| B12 控制权/无界队列/去重 | source/device授权、短期许可、代次与序号高水位、停止独立容量 | R::test_relative_command_retries_do_not_move_twice、test_same_id_changed_payload_conflicts、test_highwater_survives_bounded_result_eviction、test_saturated_command_queue_cannot_block_stop_latch、test_critical_io_overflow_latches_fault_and_invalidates_count_evidence；SEC/AUTH | 能力字符串不替代DDS/本机权限 |
| B13 退出/deadman | 有期限停止先于关闭；zero和独立torque-disable分别尝试；Jog期限 | R::test_jog_deadman_and_short_ttl_stop、test_cleanup_failure_cannot_report_successful_close、test_unknown_stop_is_failed_even_if_zero_write_was_accepted；DD::test_failed_zero_still_attempts_independent_torque_disable；I::test_lifecycle_deactivate_stops_active_motion_without_ros_wait_deadlock | 实体手柄/线路拔出H03，接口测试不冒充实物适配 |
| B14 -1哨兵/假到位 | 显式任务状态、合法负目标、轨迹+有效速度+驻留 | I::test_single_axis_negative_target_and_relative_roundtrip；R::test_unarmed_rejected_and_full_closed_loop_uses_plant；S::test_estimation_repeated_samples_and_first_velocity_invalid | 0.08/0.15是模型阈值 |
| B15 日志无限增长 | observability/recorder.py有界异步轮转、丢样/错误可见；runtime有限记录环 | O全部；R::test_bounded_record_ring_reports_eviction；I::test_committed_success_cannot_be_canceled_and_slow_telemetry_does_not_block_control | 真机负载H03 |
| B16 自动杀占用进程 | 删除旧查杀代码，D/common.py资源flock，部署冲突明确失败 | DP::test_process_ownership_lock_prevents_second_client；静态审查新版无端口任意查杀路径 | 测试证明本工程锁冲突，不夸大第三方独占保证 |
| B17 import硬件副作用/持锁I/O | 构造无设备操作、lazy库、单I/O所有者 | DF::test_full_hardware_factory_constructs_all_real_adapters_without_io；DX全部；gca_ros/test/test_node_constructor.py::test_lifecycle_host_constructs_without_device_io | 默认无GPIO/SDK也可导入核心/仿真 |
| B18 模式未核对/None/重复读 | D/dynamixel.py逐步核对、稳定错误、每次采样一次读取 | DD::test_every_configuration_failure_prevents_enable、test_wrong_readback_refuses_enable、test_old_firmware_has_no_watchdog_capability_and_never_enables、test_watchdog_cannot_be_disabled_in_hardware_configuration | XM430-W350 model1020/fw≥38白名单；实物仍待确认 |
| B19 硬编码/无限位 | 完整RuntimeConfig及独立硬件schema、制动边界、必填物理值 | C::test_boundary_braking_includes_delay_and_output_dt、test_strict_configuration_rejects_ambiguous_values；DF::test_missing_physical_field_cannot_inherit_simulation、test_one_flag_cannot_enable_missing_hardware、test_schema_rejects_boolean_gpio_and_string_number、test_protocol_and_estimator_modulus_must_match | hardware.template的null故意拒绝ARM；H01/H03 |
| B20 多SDK/协议混用 | 单一安装包和固定官方依赖、明确型号与能力拒绝 | DD::test_unknown_model_never_connects_or_writes；DX全部；tools/installed_smoke.py | 普通安装结果见最终统一报告 |

## T01–T18：验证场景到实际测试

| 用例 | 实际测试入口 | 当前证据与范围 |
|---|---|---|
| T01 控制与到位 | C全部；R::test_unarmed_rejected_and_full_closed_loop_uses_plant；I::test_single_axis_negative_target_and_relative_roundtrip | 数学/独立plant已开发测试；最新整套见最终JUnit |
| T02 数据一致性 | DE全部；S::test_estimation_repeated_samples_and_first_velocity_invalid、test_encoder_preserves_frozen_sample_seq_and_age | 有效新采样、首次速度无效、冻结年龄不刷新 |
| T03 编码器协议 | PE全部、DE失败/迟到测试 | CRC、分片/粘连、异常码、位宽/符号/回绕及重同步 |
| T04 舵机协议与错误 | DD全部、DX缺依赖/SDK时限测试 | 已安装官方SDK短帧路径、注入读写/配置/错误 |
| T05 取消与退出 | R::test_cancel_waits_for_stop_confirmation、test_cleanup_failure_cannot_report_successful_close；I::test_competing_goals_and_immediate_cancel_have_one_terminal | 停止证据、未知失败、关闭失败不报成功 |
| T06 脉冲生命周期 | DP全部；S::test_pulse_acceptance_is_not_completion_and_abort_is_unknown；R::test_pulse_receipts_are_bound_to_segment_and_settled_only_once | 接受/确认、重复/迟到回执、中止未知 |
| T07 启动与参考 | DD生命周期/配置失败、DE重连、gca_ros构造测试 | 无自动运动/清零；设备epoch/参考/旧命令失效 |
| T08 输出监督 | R::test_owner_deadlines_stop_when_control_task_frozen、test_feedback_freeze_stops_and_does_not_renew_cached_sample；I::test_control_freeze_does_not_prevent_independent_io_stop | 设备所有者独立期限；不以遥测存活当控制健康 |
| T09 命令准入 | R::test_both_axis_validation_prevents_partial_motion、test_unselected_axis_zero_limits_are_allowed；I负位置/原子拒绝；C严格schema | 只校验所选轴，完整目标一次接受 |
| T10 会话与幂等 | R重复/冲突/结果淘汰/撤销代次测试；SEC/AUTH | 高水位及短期许可不因缓存淘汰或续租复活旧目标 |
| T11 队列与访问 | R::test_saturated_command_queue_cannot_block_stop_latch、test_critical_io_overflow_latches_fault_and_invalidates_count_evidence；I直接入口拒绝；SEC | 可控满队列、关键事件溢出及直接权限测试均通过，见最终JUnit |
| T12 观测负载 | O全部；R::test_bounded_record_ring_reports_eviction；I::test_committed_success_cannot_be_canceled_and_slow_telemetry_does_not_block_control | 慢/满盘、轮转、凭证/大小边界；新慢telemetry隔离测试结果见最终统一报告 |
| T13 部署冲突 | DP::test_process_ownership_lock_prevents_second_client；新版无任意杀进程路径审查 | 本工程资源锁失败可见；不声称能禁止任意第三方直接访问daemon |
| T14 包与副作用 | DX/DF、gca_ros构造测试；tools/installed_smoke.py | 普通安装、源码树外CLI/launch/配置及完整仿真均已实际通过 |
| T15 配置与限位 | C边界/严格schema；R::test_configuration_transaction_rejects_structural_change、test_config_cannot_claim_new_queue_capacity_was_applied；gca_ros/test/test_conversion.py | SI边界、结构配置拒绝、半更新版本禁止ARM |
| T16 Action竞态 | I::test_competing_goals_and_immediate_cancel_have_one_terminal、test_cancel_wins_over_deferred_hold_candidate、test_committed_success_cannot_be_canceled_and_slow_telemetry_does_not_block_control；R::test_cancel_before_start_clears_reservation | 两种受控终态交错窗口已写，**执行结果以最终统一报告为准**，不把一次顺利取消代替全部竞态 |
| T17 Lifecycle | I::test_lifecycle_deactivate_stops_active_motion_without_ros_wait_deadlock、test_active_is_not_arm_and_direct_device_token_cannot_be_source_token；R inactive/config测试 | active≠ARM，deactivate不等待自身ROS回调 |
| T18 HOLD与联动 | I::test_hold_source_expiry_stops_and_does_not_auto_rearm；R::test_hold_has_independent_context_and_feedback_expiry_exits | Action结果后保持上下文仍受租约/反馈监督；完整Windows联动属后续范围 |

纯控制算法不读取目标来伪造plant状态。S::test_plant_only_moves_after_accepted_output_and_has_lag、test_open_loop_can_miss_steps_without_fake_feedback分别验证推进滞后和开环失步模型；机械角真值只用于仿真评估，不回送为不存在的真实反馈。

## 多agent交叉审查记录

下列问题已经实现修正并建立回归/检查，不再列作当前未解决阻断：

- 真实适配器：官方SDK短帧CRC混入、未支持型号/旧固件误用、watchdog关闭、zero失败跳过torque-disable、编码器协议与展开模数不一致。
- 运行时：提交当刻重新检查整个脉冲段许可窗口；段ID/epoch/代次/载荷绑定与完成/abort去重；异常停止和清理失败不能仍报告成功。
- ROS并发：Action预留/Future顺序、取消与成功唯一终态、取消候选HOLD交接、生命周期独立门控及非选择轴检查。
- 工具与日志：停止指标使用IN_PROGRESS；非零退出/清理升级不能PASS；本次进程组清理包含已退出父进程留下的子进程；异常中断清理；递归拒绝凭证、记录大小不超文件上限。
- 覆盖补充：T11满队列和关键事件溢出、T12慢telemetry隔离、T16两种受控终态交错用例已经落地，执行状态仍以最终JUnit为准。

分工为设备/协议、控制/模型、ROS/协调器及主agent集成；互相复审停止、并发、独立plant与证据工具。agent口头通过不能代替命令结果。历史失败报告保留，用最终源码及最终报告说明解决状态。

### 调度观测与明确限制

本轮一次完整pytest记录为455项：447通过、3失败、5错误、0跳过，失败集中于ROS启动/状态发现、CLI运动启动、短进程握手及无效配置启动超时；现场保存在 [host-pressure归档](../artifacts/verification/host-pressure-20261105T2036Z/report.json)。同时观测到可用内存偏低、Pylance约占一半RAM，见 [现场快照](../artifacts/teleop_implementation/host_pressure/observed.json)。工作区 [.vscode/settings.json](../.vscode/settings.json) 仅为Python分析排除生成的artifacts、build、install、log目录；随后 [快照](../artifacts/teleop_implementation/host_pressure/after_excludes.json) 显示可用内存回升。相同代码、原断言与原超时下，8个失败/错误用例 [复测全部通过](../artifacts/teleop_implementation/host_pressure/recheck/pytest.xml)（60.390s，命令墙钟66.453s），[命令记录](../artifacts/teleop_implementation/host_pressure/recheck/report.json)已保留。没有终止用户IDE进程；这些关联观察不能确定每项失败的全部根因，也不能把447与8拼成整套PASS；之后本文首节所列的新统一运行独立通过，不能据此宣称最坏宿主调度性能已验证。

一次完整测试重跑在遥测阻塞场景出现约278 ms的控制/采样共同调度暂停，触发 `FEEDBACK_STALE`，随后确认停止。现场及199通过/1失败结果保存在 [历史失败证据](../artifacts/verification/development-telemetry-failure/report.json)。三次原期限定点复测均通过，控制记录最大间隔分别约23.37、20.55、22.37 ms，见 [诊断汇总](../artifacts/diagnostics/telemetry-scheduler/summary.json)；**未复现不等于已修复，暂停来源仍未确定**。该记录不支持硬实时或最坏停止时间保证。

本地输出TTL与过期反馈停车规则未放宽。当前Python/普通Linux部署在调度暂停时可能使任务失败并停车；实际负载下的运行连续性和停止性能仍需H03测量。后来另修正了原始控制记录中反馈年龄/租约剩余时间未更新的问题：日志与其单调时间戳对应，ROS快照仍在读取时重算年龄；冻结反馈回归核对日志时间差。集成测试在失败时也释放自身注入的遥测暂停，避免清理警告掩盖主要故障。

## H01–H03与尚待实物的具体资料

| 验收 | 状态 | 必须补齐的资料/测量 |
|---|---|---|
| H01 身份与低速标定 | NOT_RUN | 舵机真实型号/后缀/固件/TTL或RS-485、转换器和供电；编码器型号/电气接口/寄存器/响应迟到/符号回绕；方向、轮径比例、多点往返与载荷；实际参考方式 |
| H02 波形与实际角度 | NOT_RUN | 步进驱动型号、电平/光耦/端子、6400/25600细分冲突、传动比；高低/DIR时序、完整有效沿数、取消尾段；daemon平台/独占访问；外部机械角度与失步 |
| H03 停止与性能 | NOT_RUN | 已存在的禁止/急停回路、扭矩/使能失电行为；正常/断线/进程故障下停止时间与距离；最坏载荷误差/超调/抖动；预先约定性能阈值 |

硬件模板保持null/false；缺资料不能伪造默认值，也不阻止可无硬件完成的软件工作。当前旋转没有机械角反馈，软件无独立EN输出，也未验证实体手柄拔出；不能把接口或仿真行为扩大成这些实物能力。

## 交付范围与后续工作

上一阶段必需的软件构建、安装、测试、仿真及文档工作已有历史通过记录。本轮手柄扩展和JF01–JF07已实现，完整软件回归、普通安装及全路径仿真均已通过，具体证据见本文开头。历史失败保留，既有通过不代表已确定所有宿主调度暂停的原因；上文调度限制仍有效。之后修改源码、配置或依赖，应重新运行统一验证并形成新的指纹和报告。

后续需按实物资料完成H01–H03，确认实际设备、接线、比例、波形和停止性能后再启用hardware。实体只读映射已通过；实体输入→simulation仅确认短暂推进，旋转与持续双轴控制未通过，无线资格仍NOT_RUN。Windows磁驱动、联合实验、无线资格验收、跨机DDS权限策略、视觉/力反馈等仍是明确的后续范围，当前GCA-only不会代替这些能力。
