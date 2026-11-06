# 仿真、故障注入与证据

> 进一步测试已按用户要求暂停。当前新源码491项软件测试通过；本文既有完整DDS/模型结果是此前快照，本次全新安装与完整仿真复验尚未完成。当前状态见[实现状态](IMPLEMENTATION_STATUS.md)。

正式仿真启动两个真实 ROS 2 进程，CLI 经实际 DDS 的 Lifecycle、Service、Action 和 Topic 操作。替换的是设备与机械模型，rclpy 没有被 mock。另有固定模型时钟的算法报告；两者分开标记，避免把模型时间误当 Pi 的墙钟性能。

## 运行

```bash
cd /home/pi/mcrs_ros2_proj
bash tools/verify_software.sh
bash tools/run_simulation_suite.sh --output artifacts/simulation/latest
```

单场景调试可加 `--scenarios basic`，但不代替完整验证。工具使用独立测试 domain，记录本次命令与参数，从临时工作目录调用安装后的 launch／CLI；完成后只清理本次创建的进程组。客户端失败、节点异常退出或必须强制终止都会使场景失败。

Pi上运行完整套件时，生成安装树和报告的IDE索引可能增加宿主负载。工作区 [.vscode/settings.json](../.vscode/settings.json) 已用 `python.analysis.exclude` 排除 `artifacts/**` 与 `ros2_ws/build*/install*/log*` 生成目录，源代码仍参与分析。曾在内存压力现场出现启动超时，原代码/原超时下的8项复测通过；具体失败和资源快照见 [实现状态](IMPLEMENTATION_STATUS.md)。该复测不代替整套验收，也不构成最坏调度时间保证。

| ROS 场景 | 必须观察到的行为 |
|---|---|
| basic | 未ARM拒绝，显式参考／ARM后到5 mm及20°，STOP与终态 |
| cancel | 动作执行中原生取消，CANCELED且停止确认 |
| jog | 持续新输入时运动，deadman释放／输入到期后停止 |
| hold | 成功后outputs_held，独立保持上下文，随后STOP |
| faults | 推进反馈断开导致故障锁存，停止、清故障与显式恢复 |

`tests/test_ros_integration.py` 增补直接设备接口授权、会话冲突、单轴与负目标、并发准入、deactivate、源过期、控制冻结、受控取消／成功交错和慢遥测隔离。纯算法／设备／权限测试与真实进程测试互补，不能只跑顺利动作。

## 独立模型

`MechanicalPlant` 只接收实际被模拟执行器接受的速度 u 和输出脉冲，不读取命令目标或参考轨迹。推进模型为 `dv/dt=(u_eff-v)/tau`；对固定输入精确积分速度和位移，`u_eff` 包含饱和、死区和滑移。仿真默认 tau=0.04 s、执行器速度饱和30 mm/s、死区0.01 mm/s；这些数值不是机械标定。

编码器独立按原始采样时刻读取模型位置，含量化、固定种子噪声和可配置延迟；控制器只读该反馈。重复 seq 不重复更新速度或积分，第一帧及长间隔后的速度无效。外部参考是坐标变换，不能回写 plant 位置来假造到位。

旋转同时保存目标量化脉冲、预约、完成回执和独立机械角度。机械角度仅作评估；正式控制器没有使用真实系统不存在的角度传感器。注入失步可让估计角度与模型机械角度分离，软件不会凭输出脉冲自动发现所有失步。

## 固定模型时钟报告

完整 simulation suite 也执行 `tools/model_report.py`，输出到 `model/`：正常推进、25%滑移、零速API成功但模型仍运动、正常旋转、20%失步。复用正式轨迹、控制器、估计器、量化器及模拟设备，独立模型时钟按整数步推进；不会把这层报告标成完整 ROS 路径验证。

```bash
source tools/common.sh
gca_source_install
"$GCA_PYTHON" tools/model_report.py --output artifacts/model
```

模型报告显式记录 `clock=model_time`、seed=7、步长、参数与 `formal_ROS_path_tested_here=false`。原始 CSV／JSON 能重复生成；图表 PNG 可离线审阅。零速失败场景的 PASS 表示成功检测到“写零成功不代表已停”这一预期异常，不表示执行器停止了。

## 如何读报告

| 文件 | 内容 |
|---|---|
| `report.json` | 全套是否通过、配置哈希、版本、各场景退出码及模型报告 |
| `<scenario>/client/report.json` | 安装后CLI的Action结果与续租状态 |
| `<scenario>/states.jsonl` | 独立订阅器的完整ROS状态，含wall_s和boot_id |
| `<scenario>/states.csv` | 同一记录中的标量字段，便于离线分析 |
| `<scenario>/curves.png`／SVG | 目标／轨迹／反馈、请求／接受输出、旋转估计 |
| `<scenario>/launch.log`／`client.log` | 节点启动、调用与退出诊断 |
| `<scenario>/ros_logs/gca-records-*` | 本地控制高频记录，有限队列和轮转 |
| `model/junit.xml` | 手柄套件独立模型逐23场景JUnit；clock=model_time、criterion_version与实际full_duration属性，不把模型秒写作time耗时 |
| `model/summary.json` | 固定模型时间下的原始假设与场景指标 |

真实 ROS basic/HOLD 场景使用模拟阈值：最终推进误差≤0.12 mm、20°脉冲估计误差≤0.04°；后者包含6400脉冲／电机圈、17/30传动比对应的量化。模型推进报告使用自己的10 mm目标、误差≤0.1 mm、超调≤0.5 mm门槛。不同目标与时钟不混用指标。

`dds_observed_stop_response_s` 是订阅器第一次看见 IN_PROGRESS 到 CONFIRMED 的间隔，受发布周期与调度影响；它不是从物理故障发生起测得的最坏停止上界。模型报告的reached_at_model_s也不代表ROS进程耗时。

## 故障覆盖与限制

设备测试注入坏CRC、短帧、异常码、迟到响应、超时、写失败、错误读回、身份／固件错误、未知波形中止。核心测试覆盖反馈冻结／跳变、控制冻结但I/O继续、租约到期、旧epoch／代次／回执、队列满、已撤销权限、停止未知、部分清理失败。观测测试用事件屏障控制慢盘，避免依赖偶然调度；真实进程测试用实际墙钟期限。

`terminal_pause` 和 `telemetry_pause` 是 simulation-only 的桥接测试钩子：前者固定结果提交窗口，后者模拟ROS观测回调阻塞；它们不出现在硬件运行能力中。原始编码器冻结保持原seq和接收时间，不用不断发相同数值冒充新数据。

有限波形完整生命周期可以提供条件成立时的脉冲计数证据；中止无法知道部分边沿数则返回未知。模型无法证明 GPIO 电气兼容、真实波形连续性、导管滑移、真实角度、停止距离或整机控制精度。H01–H03 当前均 NOT_RUN，见 [HIL验收](HIL_ACCEPTANCE.md)。

## 手柄全路径与可控性报告

本轮新增正式入口：

```bash
bash tools/run_teleop_suite.sh --output artifacts/teleop/latest
```

脚本报告中的physical_input_mapping/physical_hil=NOT_RUN表示该脚本场景未使用实体输入或执行器，不覆盖另存的只读实体映射证据；真实输入→simulation试验也独立记录。

该工具从安装包启动 `teleop_simulation.launch.py`，合成脚本进入普通输入端口，经实际映射/门控、owner、本机IPC、双层许可和真实DDS送至核心；执行器/编码器为模拟设备，rclpy不被替换。场景为 linear、rotation、diagonal_reverse、b_stop、reader_freeze、disconnect、slow_arm、control_freeze、lt_release_restore。它不直接伪造Jog跳过输入层，也不读取owner凭证。工具完成后只清理本次进程组。

`--scenarios linear`、`--skip-model`仅用于开发：即使子集passed=true，`full_acceptance=false`仍不算完整交付。本轮 [verification/latest](../artifacts/verification/latest/report.json) 已完成455项软件回归及六包普通安装；[teleop/latest](../artifacts/teleop/latest/report.json)9个ROS/DDS场景、23个模型场景全部通过，`full_acceptance=true`，源码与安装输入未变化。结果需连同源码指纹、退出码、JUnit和普通安装探测一起阅读。

| 产物 | 含义 |
|---|---|
| `teleop/latest/report.json`、`junit.xml` | 场景结果、判据/脚本/源码摘要、完整性与源码未变化核对 |
| 各场景 `states.jsonl/CSV`、`teleop.jsonl`、`arm_phases.jsonl` | 真实DDS设备/输入/ARM交接观察，墙钟或同机单调时间 |
| 各场景 `input_records/`、命令/launch日志及曲线 | 原输入、归一化意图/质量、派发/管理时间线、退出与有界记录；`simulation_io.jsonl`为独立模拟I/O观测 |
| `model/junit.xml` | 独立模型逐23场景JUnit；clock=model_time、criterion_version与实际full_duration属性，不把模型秒写作time耗时 |
| `model/summary.json` | 正式核心runtime+模拟端口+独立plant的固定模型时间报告，非ROS耗时 |
| `model/<case>.csv/jsonl/json/png/svg` | 独立速度/位移/角真值、控制接受值、误差和图表 |
| `model/<case>_accepted_profiles.jsonl` | 接受段时间、全部整数µs边沿、段内率、代次及固定截止，可复核空档 |

模型报告使用 `gca.simulation.teleop_report.run(output, criteria=...)`，由统一工具传入安装的版本化判据。它逐步执行唯一runtime控制算法，再由实际模拟执行器接受输出驱动独立plant，不用目标回显当反馈。无worker线程的模型调度不是正式ROS并发证明；STOP/ARM竞态另由真实owner屏障与进程/DDS测试核验。

预声明判据 teleop-controllability-v3保留v1/v2原23场景及已有门槛：±2 mm/s、滑移0/10%/25%，2s后W=0.5s独立plant平均速度误差≤max(5%×目标绝对值,2编码器计数/W)；回中稳定后额外1s漂移≤2计数。噪声0.005mm、延迟0.02s分别及组合使用W=1s，额外误差预算0.02mm/s。低速0.1deg/s×10s完整结算，与受限速度独立积分相差≤1脉冲且必须非零；±30deg/s以跨段窗口评估5%或2脉冲/W。固定seed=7运行1800s模型，检查相位余量≤半脉冲、状态、重复性与有界记录/淘汰计数。

v2新增的6个事前声明短场景继续保留：独立plant死区0.01/0.5mm/s × 意图±0.25mm/s，2s后W=0.5s误差≤max(5%目标,2计数/W)=0.04mm/s；较大死区高于意图，必须实际跨过执行器非线性。饱和/解除场景在意图±2时先把实际接受能力限制到1mm/s，t=1s解除；解除后2s的W=0.5s误差仍≤0.1mm/s、积分贡献回落≤0.1mm/s，活动目标持续到窗口结束后，再检验回中漂移。接受速度、真实速度、死区/上限和积分分别入原始CSV/JSONL，不通过修改测量或提前清零积分制造恢复。原17场景保留，完整报告为23场景；本次统一安装验收已实际完成双1800s模型运行。

短开发证据 `artifacts/teleop_implementation/core/extended_models/summary.json` 中6场景按预声明门槛通过：0.5死区/0.25意图的速度误差约0.02038mm/s；饱和解除后稳态误差约9.15e-6mm/s、积分约0.01154mm/s。该模型同时观测瞬态采样速度峰值约2.954mm/s，不宣称低超调或实机性能；峰值、稳态与积分恢复分别报告，不用稳态5%推广为瞬态5%。

计划/预约/确认计数与独立机械角分列；固定25%失步对照中机械角与软件角分离而参考仍可valid，是开环不可观测的预期结果，不是检测失步通过。普通回中不补追旧位移；LT/B立即abort时若部分计数未知则参考失效，axis_mask=3锁定并需显式恢复。

旋转平均意图、段内峰值、段间空档和占空比都必须报告。源码级开发模型曾观测30deg/s意图对应约29.994deg/s跨段均速、约42.444deg/s段内峰值与10ms最大空档；这些是特定模型/调度结果，仅解释语义，最终值读取最新报告。整数时序oracle验证段内受限相位/导数和µs误差；不能据此宣称跨空档机械连续加速度或实物角度/停止距离已验证。

v3对long_sampling增加**两次完整同seed运行**：每次1800s刺激，随后0.5s零速稳定和1s观察；每run保留实际刺激/观察时长、row_count、checks与采样hash。hash覆盖明确列出的0.1s数值/状态采样及中立边界，排除UUID/host字段；它不是每个5ms I/O tick的全量哈希。两run hash须相同，首run原始CSV/JSONL/accepted profiles可复现，第二run不重复导出巨量raw。

线速度和角速度意图的共同周期为30s，以首个稳态周期端点t=30s作累计基准；相邻完整周期端点的独立plant漂移分别≤2编码器计数与1脉冲角，累计上限为实际周期间隔数N乘同一预算，并报告全程首末偏移。该累积推导不要求速度Jog回到初始位置或进入HOLD。完成且idle、reserved=0时软件计数对无失步plant误差≤1脉冲；活动段则检查机械真值位于confirmed到confirmed+signed reserved包络。末尾稳定后1s线漂移仍≤原2计数；相位余量/状态/日志容量同时检查。边界比较的预声明ULP数值舍入guard单列，不作物理性能余量。

本次 [v3完整模型报告](../artifacts/teleop/latest/model/summary.json) 两个run均完成1800s刺激及1.5s收尾观察，各保留18018行。两run采样哈希同为 `bc9c3ff88c23f9506b29018b11dfeab59e9b9af64fe3c8930aa942e72d717ebd`；每run最大30s周期线漂移0.000396833mm（阈值0.02mm），最大角漂移约1脉冲，末尾中立1s线漂移3.954725e-10mm、角漂移0。完成计数对无失步plant最大差4.689931e-12脉冲，最大phase余量0.499988221脉冲；控制环形记录保留4000条、淘汰176155条，容量行为如实报告。完整判据包含预声明浮点舍入guard，具体数值与全部checks见JSON；数值结果不能当作真实机械精度。

本次真实DDS套件的STOP观测汇总见 [主报告](../artifacts/teleop/latest/report.json) 的 `stop_latency`。从请求起算，owner受理P50/P95/最大为22.614/33.704/40.835ms（279样本），独立模拟I/O首次观测为65.230/89.821/112.970ms（278样本），DDS首次观测停止确认为191.966/888.738/1230.577ms（270样本）。不同阶段样本数与观察链路不同，不能把最后一个DDS到达时刻当机械停止时刻，也不声明硬实时上界；逐次原始时间线与缺失阶段保留在各场景报告中。

v3开发记录在 `artifacts/teleop_implementation/core/v3_short_periods/`：两次60s刺激+1.5s观察、各618个记录样本、hash完全一致；周期线漂移约2.415e-5mm，角漂移1脉冲，末尾中立线漂移约3.04e-10mm。`full_30_minutes=false`，不能用它或更早单次1800s报告代替当前双1800s完整验收。半脉冲fraction±.5从零速进入正反向时的最早交点已用精确c==0修复，并有真实runtime多边沿和独立相位oracle回归；非零微小加速度不被epsilon吞掉。

JF01–JF07、JT01–JT20到源码/测试的精确对应见 [手柄追踪表](TELEOP_TRACEABILITY.md)。实体只读映射向导、无线资格试验及其适用边界见 [手柄指南](TELEOP_USER_GUIDE.md)；它们既不能被合成测试代替，也不属于GCA执行器HIL。

simulation专用的 `ControlRuntime.simulation_evidence()` 通过 `/diagnostics` 的 `simulation_io_evidence` JSON给出 `evidence_kind=SIMULATED_ACCEPTED_IO_AND_PLANT`、available、sampled_ns/plant_updated_ns、独立plant位置/速度/机械角与接受速度、servo_enabled、pulse_idle/活动段原截止、input/jog/demand原截止、输出/控制序号及stop_serial。该出口只读，不advance/poll、不额外写设备；跨读元数据变化或快照暂不可用时available=false。control_freeze场景据此核对原期限后的实际模拟接受输出，而不是仅sleep再解除冻结或把停滞状态快照当停止证据。该方法仅在simulation提供有效证据，硬件返回available=false/SIMULATION_ONLY；不能当硬件传感器反馈。

当前推进控制在有效输入/反馈年龄内允许控制任务仅刷新未变速度需求的期限元数据，不因缺少新seq重算PI、重复write或重复量化回执。回中/反向/降幅仍按slew改变输出；控制冻结后该刷新停止，原deadline继续约束I/O。真实pigpio最终触发校验还预留声明发送RPC预算；注入测试验证不足时拒绝，不以此宣称实际RPC或机械延迟已有硬上界。
