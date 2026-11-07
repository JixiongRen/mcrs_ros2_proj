# 工程实现状态与证据

## 本轮交付结项（2026-11-07）

用户已明确“两项专项移到后续，本轮结项”。依据这一范围修订，[交付报告](../artifacts/servo_ros2/delivery-v5/report.json)将本轮标为完成：软件实现PASS、授权空载范围内的真实手柄舵机联调PASS、外部编码器及位置PID接口预留PASS。接口预留通过不表示编码器已接入或位置闭环已经完成。

最终参数为v5：指令上限79.921 rpm（不超过80 rpm）、变化率40 rpm/s、停车确认期限2秒；最近实测约±45.8 rpm。现行配置与最后一轮实机快照一致，87个安装源码及接口文件与工作区一致，最后的实机状态为零速、力矩关闭、DISARM和会话释放。结项核对复用已有测试及运行证据，没有追加测试，也未自动提交或推送。

“停止命令更新但保持遥测”及“受控总线静默验证watchdog”两项实机测试移至后续，状态保留`NOT_RUN`。无线资格仍未完成，沿用用户明确允许的joydev_hold空载运行范围。上一轮偶发中断保留原始证据，本次未复现，不宣称根因已消除。逐项范围与证据见[需求核对](../artifacts/servo_ros2/delivery-v5/requirements.json)及[用户范围修订](../artifacts/servo_ros2/delivery-v5/scope-revision.json)。

日常启用、正反转、B 后恢复和停止退出见[真实舵机快速操作](SERVO_USER_GUIDE.md#真实舵机快速操作)。以下按阶段保留历史记录，其中“尚未复测”“尚未验收”描述对应阶段，当前范围以本节结项说明为准。

## 单次复现结果：本轮正常，前次记为未复现的偶发中断（2026-11-07）

按用户“再测试一次，不能复现则判为意外中断”的要求，只增加了[一轮复现记录](../artifacts/servo_ros2/one-reproduction.8jkrvts9/report.json)，沿用v5参数及原16毫秒USB缓存设置。180秒记录窗口内，实测速度达到±45.8 rpm，记录了33次LT释放及3次B停止，没有串口错误、FAULT或进程意外退出。167次进程观察均保留舵机节点、协调器、手柄节点及launch存活证据，直到脚本开始显式清理。

用户确认“这次看起来正常，最后脚本自动退出触发停车”。最后一次停车由预声明窗口结束触发，已确认零速、力矩关闭、DISARM、释放会话，launch退出码0。前次事件据此记为“本次未复现的偶发中断”，保留原故障和原始输入记录，不宣称根因已查明或长期可靠性已证明。本轮手柄复现测试结束，不追加重复试验。原goal中的停止命令更新但维持遥测、受控总线静默两项实机专项仍为NOT_RUN；无线资格继续按用户授权不作为joydev_hold运行的前置条件。

## v5实机：快速重启及B恢复已有证据，反馈读取仍超时（2026-11-07）

[本组报告](../artifacts/servo_ros2/stop-budget-retest.w3zafzp8/report.json)仍为FAIL。实测速度达到±45.57 rpm，初始化读回速度限制raw349、加速度raw12、watchdog25、模式1与Drive Mode0。原停车超时没有重现；约45 rpm下停车确认耗时约1.24–1.33秒。10段WAIT_STOP中，前9段恢复新的JOG并被核心接受，其中7段在下一次LT释放前测得再次运动；另2段很快又释放LT。运动中B停车后1.316秒确认，之后新操作也产生实测运动。

最后一次LT释放后64.151毫秒出现`read_feedback / DEVICE_TIMEOUT / late DYNAMIXEL read`，错误记录时间比原80毫秒事务期限晚约0.885毫秒。此时已经重新按LT，但仍处于WAIT_STOP，尚未恢复运动；故障使本次重启取消。SDK返回成功、状态和长度校验通过，但没有原始字节接收完成时间，不能确定迟到来自串口传输、线程调度还是后续处理，不能据此忽略错误或随意延长期限。

最后已确认零速、力矩关闭、会话释放，节点退出码0，DEVICE_TIMEOUT保留。本轮不再追加实机测试。用户随后明确为“持续按住LT时突然停转”。整组复审未发现原始LT保持按下却被误判为释放，也未发现无STOP而自行关闭力矩；但日志中的原始输入不能证明操作者实际松手。存在一次LY保持到底而LT上报释放约128.5毫秒后恢复的片段，性质未明，不能将最后的串口故障直接认定为操作者所述事件。无线资格和原计划两项失效停车场景继续标为NOT_RUN，整体真实链路尚未验收。

## 80 rpm指令配置实测：停车确认期限不足（2026-11-07）

[v4实机记录](../artifacts/servo_ros2/restart-retest.e_kxq19l/report.json)未通过。反向实测达到约45.57 rpm；首次LT释放后约14毫秒进入STOPPING，约105毫秒的新鲜反馈确认力矩关闭，但约1.19秒才首次读到零速。原1秒停车期限先触发`STOP_TIMEOUT`并DISARM，后续快速重启及B恢复未完成。无串口错误。最终确认零速、力矩关闭、释放会话，进程退出码0，故障原因保留。

关闭力矩后轴位仍变化1101 counts（约96.77°），说明存在实际机械滑行，不能将全部间隔归为软件时延。v5只把`stop_timeout_s`改为2秒；STOP立即撤销输出、零速阈值、100毫秒驻留和原输入期限保持原样。对同一组真实样本进行[离线停车逻辑回放](../artifacts/servo_ros2/restart-retest.e_kxq19l/stop-replay.json)，旧配置在1.031秒进入FAULT，新配置在1.282秒确认停车；反馈缺失不能伪造停车的既有用例通过。2秒是本次约45 rpm观测后的候选确认预算，不会加快机械停车，也不代表实际80 rpm停车已验证。配置变更无需重建ROS包，须重新导出并重新配置；v5后续实机结果见本页顶部。

## 最新实机反馈与快速重启修订（2026-11-07）

[修订v3实机记录](../artifacts/servo_ros2/response-retest._hrdv9ie/report.json)已完成，操作者评价“基本正常”。记录中实测范围为−9.847至+9.847 rpm，正反方向接近满速的连续区间分别约36.65秒、29.04秒，未出现`JOG_EXPIRED`、串口错误或FAULT。LT/B均有停止事件；移动中的独立LT释放后，首次新鲜零速且力矩关闭反馈约0.49–0.59秒，确认约0.63–0.74秒。该时间是观测时间，不倒推精确机械停时。最后一次B与测试窗口结束STOP重叠，不作为独立B验收。最终已确认零速、关闭力矩、DISARM、释放会话，进程退出码0。

尚有两项交互问题：B后重新操作及快速松开再按LT时，新按下事件可能发生在STOP确认前，被就绪变化重置而丢失；另有一次`CONTEXT_CHANGED`在新非零意图发出前截断操作。当时执行端仍在零速，但已有较新的硬件样本；teleop持有的旧样本恰好越过250毫秒门限，属于上层遥测缓存重复参与反馈新鲜度判断的强证据。该次实机不标为完整通过。

按操作者新要求，v4配置将指令上限改为80 rpm（raw349，实际可表示值79.921 rpm），软件加速度40 rpm/s，固件Profile Acceleration raw12。厂家标称XM430-W350空载转速为12 V时46 rpm、14.8 V时57 rpm，因此80 rpm只表示软件指令上限，不是实机可达转速。原始实机记录仍对应10 rpm配置，不混入新参数结果。快速重启门控与本次参数调整完成后只进行相关定向检查及一次正式ROS模拟，实机验证另行记录。

v4[修订与验证记录](../artifacts/servo_ros2/restart-response-fix/report.json)：101项相关定向检查通过，核心及手柄包重建成功，69个源码模块与安装副本一致。快速重启在最新STOP屏障内保留新LT操作资格，确认后使用当时的输入；B释放本身不恢复运动，等待期间释放LT、按B或输入失效均取消操作。原核心反馈期限及停车判据保留。

正式模拟发现满速RPM经ROS单位往返转换后出现浮点尾差，已限制为四个ULP的舍入容差并在接纳时夹回精确上限，真实超限仍拒绝。第二次运行在初始化RPC超时，未进入运动，具体底层时序原因没有被记录；检查工具补充了生命周期服务实际只读响应的启动条件。[最终安装后10秒模拟](../artifacts/servo_ros2/restart-response-fix/ros-simulation-ready/report.json)十项检查通过，节点退出码0；两次失败记录保留。该模拟使用合成输入，实体held快速重启由软件用例覆盖，不能代替实机结果。**本轮软件验证结束；80 rpm指令配置、40 rpm/s加速度及快速重启修订尚未实机验收。**

## 历史实机：响应迟缓与指令到期（2026-11-07）

[本次实机报告](../artifacts/servo_ros2/permit-retest.np0hati9/report.json)为FAIL。已记录正负9.847 rpm，但操作者反馈不跟手并再次中断；首发停止原因为`JOG_EXPIRED`，未记录到串口错误。停止前输入观察仍更新，LT保持、B未按；seq683从申请许可到发布耗时48.353毫秒，随后36.483毫秒才首次反映在设备状态中，此时原输入期限仅剩38.755毫秒。seq685虽在旧期限前30.505毫秒发布，却未及时成为执行端新指令。现有记录缺少下游各阶段接收及拒绝信息，尚不能确定该条指令的具体延迟位置。

响应迟缓另有明确原因：原软件加速度为3 rpm/s，固件加速度raw 1约为3.576 rpm/s。记录中目标已变为+9.847 rpm时，受限输出仍接近零，随后约3.364秒才达到目标。末次反馈确认零速、力矩关闭，DISARM、会话释放及launch正常退出；持续运行和独立LT/B验收未完成。

本次软件修订集中在四处：软件加速度改为20 rpm/s、固件raw 6；舵机输入及输出期限统一为300毫秒，移除手柄发送端硬编码150毫秒；I/O按整轮耗时计算等待，不在写入及读取后再固定等待20毫秒；补充JOG各阶段拒绝原因和核心拒绝返回日志。原始样本及许可一经生成便不续期，LT/B独立STOP、单一在途许可及最新命令槽保留。默认qualified_state和旧双轴输入约束保留。

300毫秒按现有两个80毫秒事务及控制调度预留预算，不是实测最坏延迟或机械停车承诺。旧150毫秒快照不能满足新的完整读写周期校验，必须重新导出v3配置。[94项定向检查](../artifacts/servo_ros2/response-budget-fix/report.json)通过（核心50项、手柄与ROS边界44项），四包重建成功，69个源码模块与安装副本一致。[安装后10秒ROS模拟](../artifacts/servo_ros2/response-budget-fix/ros-simulation/report.json)十项检查通过，节点正常退出；模拟使用合成输入，连续held语义由定向用例覆盖。第一次边界测试因测试模块导入路径缺失而未能收集，修正环境后通过，原失败记录保留。软件验证结束，**修订后实机尚未复测，不能宣称已消除运动中断**。

## 上一组实机：持续反转时指令到期（2026-11-07）

通信锁补丁后的[实机复测](../artifacts/servo_ros2/joystick-retest.namyiyr5/report.json)仍未通过。执行约15.88秒后出现`JOG_EXPIRED`；此时LY仍为上推到底、LT按住、B松开，输入继续更新。本组没有记录到串口错误，实测范围为−9.847至0 rpm。最终已确认零速、力矩关闭、DISARM和会话释放，全部实机进程正常退出；LT/B与持续正反转未完成实机验收。

软件复现确认，协调器与执行端的运动许可服务原先与管理操作共用互斥回调组，管理等待会阻塞许可。已将两端许可服务改为独立回调组，保留现有权限、单次许可、截止时间与STOP屏障。[两个原版复现用例](../artifacts/servo_ros2/joystick-retest.namyiyr5/permit-reproducer.xml)均因等待管理回调而失败；分组修改后的31项相关回归通过。[一次35秒ROS模拟](../artifacts/servo_ros2/joystick-retest.namyiyr5/permit-sustained-simulation/report.json)在3至31.8秒区间的288个状态均为EXECUTING，并确认LT释放后归零。该模拟使用合成输入和模拟舵机，不代表实体手柄链路通过，也未覆盖随后补充的鉴权／许可登记原子性修改。

并发复审补齐了鉴权与许可登记的原子性：使用现有授权锁保护这两个短操作，防止撤销完成后重新残留旧代次许可；锁内没有设备操作或RPC等待。最终[33项定向回归](../artifacts/servo_ros2/joystick-retest.namyiyr5/permit-atomic-tests.xml)通过，两包重建成功且源码与安装副本一致。最终安装版本的[10秒正式ROS模拟](../artifacts/servo_ros2/joystick-retest.namyiyr5/permit-final-simulation/report.json)十项检查通过，包括正反转、调速及LT/B；节点正常退出。该阶段软件验证完成；随后实机仍失败，见本页最新记录，整体真实链路未验收。

## 上一组实机：正反转已实现，速度写入超时（2026-11-07）

[本组记录](../artifacts/servo_ros2/joystick-hardware.hqasexp2/report.json)显示，实测速度达到−9.847至+9.847 rpm，759条JOG请求持续约30.32秒；随后出现`write_velocity / DEVICE_TIMEOUT / late DYNAMIXEL write`。故障前LT仍按住、B未按下，输入持续更新。最终零速、力矩关闭、停止确认，实机与采集进程已退出；LT释放和B停车未能独立验收，本组不通过。

速度写入完成时已超过80毫秒绝对期限。设备状态记录存在约232毫秒间断，但写事务持有控制线程也需要的锁，因此不能仅凭间断判定整个进程暂停。一次35秒纯软件时序采集未复现同量级的舵机进程GC停顿；该模拟约6.5秒便出现JOG_EXPIRED，不是连续运动验收。现有证据尚不能区分本次串口／USB响应延迟与线程调度停顿，不放宽期限或忽略错误。

另外修复已确认的记录缺陷：待处理、执行中及已确认的故障原因不再被后续普通STOP覆盖。未新增状态字段，也未改变停车时序；[15项相关回归](../artifacts/servo_ros2/joystick-hardware.hqasexp2/fault-reason-tests.xml)通过。此修复用于保留故障依据，**不代表意外停车根因已解决**。

随后修复速度事务对控制更新的阻塞：STOP锁只覆盖最终许可检查与串口发送，等待ACK时释放锁，允许控制线程接纳新输入或撤销输出。发送前取消会释放SDK的总线占用标志，后续零速／关闭力矩仍可执行；迟到回执不能恢复已撤销输出。原有80毫秒事务期限、输入／租约／输出截止时间和报文校验保持不变。[69项定向回归](../artifacts/servo_ros2/joystick-hardware.hqasexp2/write-guard-tests.xml)通过，核心包构建成功，5个运行文件与安装副本一致，见[修复记录](../artifacts/servo_ros2/joystick-hardware.hqasexp2/write-guard-repair.json)。本补丁尚未开展实机运动复测，不能将软件回归通过视为持续转动已经通过。

安装后的一次[10秒ROS模拟](../artifacts/servo_ros2/joystick-hardware.hqasexp2/write-guard-ros-simulation/report.json)十项检查通过，涵盖正反转、杆量调速、LT/B及停止后不恢复，所有节点正常退出。本轮软件验证至此结束；该短模拟不替代持续实机验收，也不消除上文35秒时序采集中的未解决间断。

## 再次复测：运动前反馈读取故障（2026-11-07）

[补丁后的实机记录](../artifacts/servo_ros2/hardware-retest.1nsxzeft/after-budget-fix/report.json)表明，本次在READY约10.4秒后、手柄操作之前发生`read_feedback / SDK_ERROR -3002`，没有发布任何JOG；故障发生时没有运动输入期限。故而不能继续将本次不转动归因于旧运动指令的剩余时长。

审查确认SDK还有独立的提前超时：57600波特下，72字节反馈的SDK估计等待约48.41毫秒，短于项目配置的80毫秒。软件回归复现55毫秒到达的完整合法响应被提前拒绝。已将接收超时统一为调用者原有绝对期限，并将零超时忙轮询改为按剩余期限等待串口数据；不重发运动、不延长旧输出有效期，CRC、长度及身份检查保留。

独立于ROS的30秒只读采集取得625个轴反馈样本、共629笔READ，全部成功，最长31.81毫秒，见[报文记录](../artifacts/servo_ros2/hardware-retest.1nsxzeft/after-budget-fix/wire-readonly.json)。这不证明原始ROS故障报文一定有效；未捕获当时线缆数据，真实报文损坏仍不能排除。[38项适配层回归](../artifacts/servo_ros2/hardware-retest.1nsxzeft/after-budget-fix/sdk-tests.xml)通过，覆盖期限、完整合法帧、错误CRC及缺失尾部。

核心包已重建，源码与安装内容一致。修复后的[ROS实机反馈检查](../artifacts/servo_ros2/hardware-retest.1nsxzeft/after-budget-fix/sdk-idle-check-2/report.json)持续30秒，新增615个样本，无串口故障，全程零速、力矩关闭，launch正常退出。首次检查脚本在协调器状态尚未到达时申请会话而提前结束，其记录保留；正式反馈检查等待配置状态就绪，并未申请会话或ARM。

**当前结论：SDK修复、定向回归和实机反馈读取通过；手柄持续运动仍未验收。** 本次全部实机进程已关闭。以下为之前阶段证据。

## 实机复测：短暂低速后串口超时（2026-11-07）

本次实机复测未通过，操作者反馈“仍未观察到任何转动”。[原始记录与报告](../artifacts/servo_ros2/hardware-retest.1nsxzeft/report.json)显示：LY、LT均收到完整范围输入；发出了19条速度请求，最大反向请求为−9.847 rpm。舵机短暂接受到−1.603 rpm，反馈峰值为−1.374 rpm，轴位置净变化约−4.48°；进入EXECUTING约0.66秒后报`SDK_ERROR -3001`，随后的正向操作因FAULT而未执行。这不构成持续运动通过。

错误时间只比执行端旧输入期限晚0.124毫秒；更新的手柄指令已发布，但串口写回执等待仍持有控制锁。写入启动时旧输出剩余约24毫秒，低于既往约28毫秒的正常通信时间，存在本地期限截断交易的缺陷。当前没有逐字节串口记录，不能排除底层通信故障。已修复：写速度前必须剩余完整的既有80毫秒事务预算，不足则跳过旧输出；反馈读取使用独立预算；错误记录补充具体通信阶段和期限。未放宽任何旧指令期限，未新增配置或状态机。新增两个28毫秒读写时序场景，覆盖旧输出跳过、新输入执行及STOP／断供停车；[39项定向回归](../artifacts/servo_ros2/hardware-retest.1nsxzeft/repair-tests.xml)、核心包重建与安装一致性核对通过。此次补丁尚未实机验收。

本次已执行STOP、DISARM和释放会话；新反馈确认零速、力矩关闭，launch退出码为0，采集进程也已关闭。以下73项软件测试和ROS模拟属于复测前版本，不能据此宣称实机问题已解决。

## 上一轮代码修复：使能时序与输入调度（2026-11-07）

按用户要求，先完成多agent定位、代码修复和有限软件验证，实机进程已经关闭。本轮沿用现有状态机、授权链路和单一串口所有者，修改集中在锁范围、期限传递、响应解析与重复计算。

| 问题与依据 | 修复 |
|---|---|
| 停车后重新使能时，串口握手持有控制锁；日志中的后续指令2239、2241、2242已更新期限，但执行端仍使用首条期限，最终报SDK超时 | 耗时的零目标握手移到锁外，使用固定事务预算；最终使能前后检查当前已接纳输入、命令期限和STOP/会话身份。每条非零输出原有期限保持不变 |
| 初次ARM握手中接纳首条JOG会正常推进输出代次，却被使能检查误判撤销 | 删除使能阶段重复的输出代次约束，运动写出仍检查输出代次；增加并发回归 |
| SDK解填充后更新报文长度，但保留Python列表残尾，合法响应可能被本地长度检查误拒 | 在官方SDK完成CRC校验及解填充后裁去残尾；错误CRC和缺少数据的响应继续拒绝。软件缺陷已复现，既往实机每一条损坏响应的线缆报文未留存 |
| 手柄节点以250Hz重复检查，而默认舵机控制为25Hz；实际曾记录108.3ms读取停顿 | `joydev_hold`调度使用舵机控制周期与profile频率上限，去除重复节流和重复身份检查；输入线程仍独立读取并直接处理LT/B停车。该次读取停顿的直接原因尚未单独确认，不能仅凭负载推定 |
| 输入有效期在手柄层另有硬编码，零目标ARM还要求额外按键事件 | 通过既有舵机快照传递`manual_input_max_age_s`，同时服从输入profile更短的期限；完整中立INIT可用于零目标ARM，实际运动仍要求新的物理LT上升沿 |

[73项定向测试](../artifacts/servo_ros2/software-repair.1724qbqc/pytest.xml)、[核心与手柄包构建](../artifacts/servo_ros2/software-repair.1724qbqc/build.log)及[正式ROS模拟的10项检查](../artifacts/servo_ros2/software-repair.1724qbqc/ros-simulation/report.json)通过；72个源码模块与安装内容一致。模拟使用合成输入验证共用节点链路，`joydev_hold`的保持、期限和并发行为由定向测试覆盖。验证完成后停止追加，见[本轮报告](../artifacts/servo_ros2/software-repair.1724qbqc/report.json)。

**该阶段结论：软件修复与模拟验证通过；随后实机复测未通过，见上节。** 此前真实操作确有中断和不转动，保留失败记录；attempt5仅启动节点，未配置或使能设备便结束，不算运动验证。无线资格仍未完成。以下为此前各阶段记录，不覆盖最新源码。

## 本轮修订：LT持续操作、速度上限10 rpm

2026-11-07用户明确要求LT按住时持续操作，不设单次操作时间上限，LY决定方向与速度。代码已通过舵机配置快照的显式 `manual_input_mode=joydev_hold` 实现该语义；单一配置源 `config/servo_settings.py` 设置raw 43，即9.847 rpm，低于10 rpm，变化率3 rpm/s。持续有效reader观察允许静态杆量保持，0.15 s只约束reader观察与输出的新鲜度；会话、命令、输出期限和LT/B停止仍生效。

无线链路证据仍为 `UNKNOWN`，无线资格仍为 `NOT_RUN`；用户已取消其作为本轮空载持续操作的前置条件。默认 `qualified_state` 保留原资格约束，只有显式导出 `joydev_hold` 快照才采用新策略。舵机节点、协调器和手柄节点加载同一快照；管理顺序为configure、activate、claim、arm，不设置外部 `reference`。

| 本轮项目 | 当前实际结果 |
|---|---|
| 定向回归 | [67项JUnit](../artifacts/servo_ros2/bounded-jog-tx1awiz0/held-tests.xml)全部通过，0失败/错误/跳过，14.399s |
| 静态检查与构建 | 17个相关Python文件Ruff检查通过；[五包普通构建](../artifacts/servo_ros2/bounded-jog-tx1awiz0/held-build.log)通过，34.3s |
| 故障修复回归 | ARM重复事务移除后[32项回归](../artifacts/servo_ros2/bounded-jog-tx1awiz0/arm-fix-tests.xml)通过；SDK字节填充响应长度修复后[34项回归](../artifacts/servo_ros2/bounded-jog-tx1awiz0/packet-fix-tests.xml)通过，分别重建核心包；用例与初轮有重叠 |
| 真实速度控制 | 已读回速度模式1、raw 43限幅和watchdog 25，已观察到正反转；操作中出现JOG_EXPIRED及SDK_ERROR中断，随后一轮未转动。实机流程已结束；最新软件修复见上节，见[阶段报告](../artifacts/servo_ros2/bounded-jog-tx1awiz0/held-report.json) |
| 范围与记录 | [scope.json](../artifacts/servo_ros2/bounded-jog-tx1awiz0/scope.json)保存用户修订及已取代的旧范围；目录名 `bounded-jog-tx1awiz0` 沿用历史名称，不代表当前存在单次时长上限 |

本轮只验证新输入语义及受影响接口，没有重跑历史完整工程验收。下文259/63/13项舵机测试、491/436项及更早455项完整结果均属于此前源码快照，不能作为本次修改的验收结果。实际操作见[舵机指南](SERVO_USER_GUIDE.md)。

## 上一版：仅舵机 ROS 2 软件通过，真实链路尚未验收

2026-11-06依据[舵机接入方案](SERVO_ROS2_INTEGRATION_PLAN.md)与[实施任务](SERVO_ROS2_IMPLEMENTATION_GOAL_PROMPT.md)，已写入独立标准库配置脚本、原生 RPM 驱动、仅舵机 `FeedAxisRuntime`、正式 ROS 速度／状态接口、同 owner 管理和手柄策略。LY 上推反转、下推正转，幅度调速，LT/B 保留停止门控；LX 未分配。当前手柄模式不创建或使用外部位移编码器，未来机械标定、反馈端口与位置 PID 仅预留契约，位置任务明确拒绝。

该版记录独立保存于 [`artifacts/servo_ros2/implementation-xce7i_e4/`](../artifacts/servo_ros2/implementation-xce7i_e4/)。下列通过项及资格门控陈述均对应更改前版本，不覆盖当前 `joydev_hold`；各项不累加为完整工程或真实运动验收：

| 项目 | 当前结果与证据 |
|---|---|
| 受影响包构建 | 六包构建通过，见[构建日志](../artifacts/servo_ros2/implementation-xce7i_e4/build.log)；后续手柄修复另有定向重建 |
| 首轮定向测试与旧路径回归 | 259 项通过，见[软件报告](../artifacts/servo_ros2/implementation-xce7i_e4/software/report.json)及[JUnit](../artifacts/servo_ros2/implementation-xce7i_e4/software/pytest.xml)；覆盖当时源码，不代替后续修复复核 |
| 手柄 STOP／release 修复复核 | 四个受影响文件共 63 项通过，见[复核报告](../artifacts/servo_ros2/implementation-xce7i_e4/stop-release-recheck/report.json)；该报告中的后续模拟尝试未通过，最终链路结果见下一行 |
| 最终 STOP 回执与正式 ROS 模拟 | PASS；[最终复核报告](../artifacts/servo_ros2/implementation-xce7i_e4/stop-receipt-recheck/report.json)中 core 重建、13 项定向回归及 ROS 模拟退出码均为 0。[模拟报告](../artifacts/servo_ros2/implementation-xce7i_e4/ros-simulation-pass/report.json)十项检查全部通过，`hardware_tested=false`、`launch_exit_code=0`，三个子节点正常退出 |
| 最终源码／安装与静态检查 | [安装文件核对](../artifacts/servo_ros2/implementation-xce7i_e4/installed_sources_final.json)记录 73 项源码与安装内容一致；[静态结果](../artifacts/servo_ros2/implementation-xce7i_e4/static-final.json)的 40 个 Python 文件检查和格式检查均通过 |
| 新驱动真实只读识别 | PASS；一次探测取得三个新样本，见[只读报告](../artifacts/servo_ros2/implementation-xce7i_e4/readonly/report.json)。型号 1020、固件 46、模式 4，0 rpm、力矩关闭、硬件错误 0；单次轴反馈 READ 约 28–32 ms，WRITE 次数为 0 |
| 接收器模式确认 | PASS；2026-11-07 完成 BFM → PC360 → BFM 两次现场切换，见[模式确认记录](../artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2/assessment.json)。当前为 BFM，`20bc:506d / hid-generic`；PC360 实测为 `045e:028e / xpad` |
| 模式与映射关联 | PASS；新增 schema 3 来源包装，保留原标定原文并验证 BFM→PC360→BFM 观察。已生成无无线资格的 `event_age` profile，[安装后加载检查](../artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2/installed-binding-check.json)通过，当时资格模式的真实ARM仍拒绝 |
| 实体手柄资格 | NOT_RUN；当前模式已确认，但手柄断电、无线失联、接收器拔出各三次的资格检查尚未执行。长按 Back＋B 关机不提供准确的断电起点，不能用 B STOP 代替无线失联响应 |
| 真实速度模式 1 与停止验收 | NOT_RUN；未进行模式切换、真实 ROS 调速、无线失联停车或本轮运动 HIL；原双轴 H01–H03 状态不变 |

首轮 259 项、STOP／release 修复 63 项、零速回执修复 13 项存在重叠，不能相加。最终模拟确认上推负转速、下推正转速、不同幅度、速度限制、LT/B 停止、B 后不恢复以及外部测量不可用。结束时 `session_owned=false`、`armed=false`、`stop_confirmed=true`、`stop_progress=CONFIRMED`，记录器 dropped/errors 均为 0。以上是在正式安装后的 ROS 节点与模拟舵机上取得的证据。

初期检查工具的启动等待时序、持续 B／脚本结束与 release 的 STOP 竞争、零速回执与输出代次更新的竞争均已修复。后者曾使状态中的接受 RPM 未清零，但当时力矩已关闭、反馈归零，不能误述为真实恢复运动。历史失败保留在 [ros-simulation](../artifacts/servo_ros2/implementation-xce7i_e4/ros-simulation/report.json)、[ros-simulation-recheck](../artifacts/servo_ros2/implementation-xce7i_e4/ros-simulation-recheck/report.json)、[stop-release-recheck](../artifacts/servo_ros2/implementation-xce7i_e4/stop-release-recheck/report.json)、`ros-simulation-complete/` 和 `ros-simulation-verified/`，最终通过未放宽验收条件。该版软件验证已结束；当前只针对上述用户修订执行必要验证，不重新启动历史完整验收。

分项交付结论：**软件实现 PASS；后续接口 PASS，仅限已定义契约、关闭能力并拒绝位置任务；真实输入资格 NOT_RUN；真实速度模式 1 与相关 HIL NOT_RUN。** 后续接口通过不表示编码器驱动、真实位移闭环或 PID 整定完成。

只读时实际 Goal Velocity 仍为 raw 1023、watchdog 为 0；工具没有修改它们。当时的候选速度上限4.58 rpm及配置时限不是实测安全边界，历史位置模式约 10°／720° 试转也不替代本轮速度模式证据。整体真实链路尚未完成；操作命令、配置重载、记录含义与停止退出见[舵机指南](SERVO_USER_GUIDE.md)。

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
