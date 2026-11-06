# ROS 2 接口与语义

正式 IDL 位于 [`gca_system_interfaces`](../ros2_ws/src/gca_system_interfaces)。GCA-only 的公共动作入口是 `/system/execute_motion`，协调器代理 `/gca/execute_motion`。原系统设计的磁驱动联合实验 `/system/execute` 未在本轮实现，不接受“假磁驱动成功”。

## 接口列表

| 名称 | 类型 | 调用者与行为 |
|---|---|---|
| `/system/claim_control` | `ControlSession` service | 客户端领取唯一源会话，选择 AUTO／MANUAL／MAINTENANCE |
| `/system/renew_control` | `ControlSession` service | 同会话、同代次、递增序号续租 |
| `/system/acknowledge_generation` | `ControlSession` service | 当前owner确认ARM后的新代次，收回交接期受限STOP别名 |
| `/system/release_control` | `ControlSession` service | 撤销源会话及设备许可，触发停止 |
| `/system/get_motion_permit` | `ControlSession` service | 给一个命令上下文签发固定截止时间的一次性准入许可 |
| `/system/execute_motion` | `ExecuteMotion` action | 单轴／双轴绝对或相对定位 |
| `/system/manual_target` | `JogCommand` topic | MANUAL 模式的新鲜 deadman 速度输入 |
| `/system/{arm,disarm,reset_fault,request_stop,set_reference}` | `ManageDevice` service | 经会话授权的管理请求 |
| `/system/apply_config` | `ApplyConfig` service | 完整配置候选，校验与实际应用分别报告 |
| `/system/get_command_status` | `GetCommandStatus` service | 查询 command_id、拒绝原因及当前状态 |
| `/system/inject_fault` | `InjectFault` service | 只允许 simulation 注入故障 |
| `/system/state` | `GcaState` topic | 聚合设备快照 |
| `/gca/execute_motion` | `ExecuteMotion` action | 协调器持设备许可调用，公共源token不能替代 |
| `/gca/jog` | `JogCommand` topic | 协调器转发的有期限输入 |
| `/gca/execution_lease` | `ExecutionLease` service | 协调器专用：签发、续期、释放执行许可 |
| `/gca/get_motion_permit` | `ControlSession` service | 设备端独立签发一次性准入许可 |
| `/gca/{arm,disarm,reset_fault,request_stop,set_reference}` | `ManageDevice` service | 设备端再次授权与状态校验 |
| `/gca/{apply_config,get_command_status,inject_fault}` | 对应同名 IDL service | 设备端配置、查询及模拟故障 |
| `/gca/get_capabilities` | `GetCapabilities` service | 后端、可用能力、限制、未验证项 |
| `/gca/state` | `GcaState` topic | 带质量、年龄、版本和停止证据的状态 |
| `/teleop/state` | `TeleopState` topic | 输入门控/质量、会话状态、意图/已接受速度与有界派发队列；不含凭证 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 故障、版本与记录丢样／写盘错误 |
| `/gca_control_node/*` | 标准 Lifecycle／参数服务 | configure／activate／deactivate／cleanup，均不隐式ARM |

接口类型可通过 `ros2 interface show gca_system_interfaces/action/ExecuteMotion` 查看。生命周期、设备管理与源会话是不同状态维度，不能只看 node active 就认为可以运动。

## 单位、身份与时效

ROS 字段全部使用 SI：位置 m、角度 rad、速度 m/s 或 rad/s、加速度 m/s² 或 rad/s²、期限 s。CLI 的 mm／deg 是用户输入单位，转换后发送 SI；节点桥接再一次转换成核心 mm／deg。不得在适配器重复做这层换算。

`CommandContext` 包含 session_id、task_id、command_id、generation、sequence。UUID 用于关联与去重；sequence 在当前授权中递增。相同 command_id 不重复执行，不同载荷复用 ID 被拒绝，缓存淘汰不消除序号高水位。相对目标只解析一次。

源会话 token、设备执行 token、一次性运动许可分别代表不同用途。`get_motion_permit` 请求使用现有会话 token 和即将提交的完整 CommandContext，`lease_s` 表示短准入窗口，响应 token 是不透明值。Action／Jog 使用返回的许可；许可绑定上下文且截止时间来自签发端单调时钟。续租不能延长已经签发的许可。安装后的 `GcaClient` 自动处理这些步骤。

任务执行超时、源租约、设备租约、准入期限、输出 TTL、反馈最大年龄、停止期限相互独立。ROS 时间戳用于观测；截止时间与采样年龄使用本机单调时钟，不依赖网络双方时钟同步。

## ExecuteMotion

Goal 的 `target.axis_mask` 为 LINEAR=1、ROTATION=2、双轴=3；`target.mode` 为 ABSOLUTE=1、RELATIVE=2。`reference_id` 必须对应当前参考。所选轴的目标、速度及加速度必须有限并位于配置上限内；未选轴不继承旧 HOLD。双轴整体校验后才占用目标槽。

`end_policy`：UNSPECIFIED=0 被拒绝，STOP=1 到位后停止并核验，HOLD=2 到位后保留独立保持上下文。结果含：

| 字段 | 含义 |
|---|---|
| `outcome` | SUCCEEDED=1、CANCELED=2、FAILED=3 |
| `error_code`／`message` | 稳定诊断码与补充说明 |
| `final_state` | 终态提交时快照，仍需读有效性 |
| `outputs_held` | 成功后是否仍在受监督保持 |
| `stop_confirmed` | 是否已取得该次停止所需证据 |

Action feedback 包含 phase、state、elapsed_s、stop。原生拒绝只返回 accepted=false，详细原因通过 `get_command_status` 按 command_id 查询；不要因网络等待超时就用新 command_id 再做一次相对运动。

ROS Action 状态和业务结果需要一起读。例如授权过期导致中止，即使已停止，也不是一次到位成功。取消仅在 CANCELING 后提交 canceled；故障与取消交错保留失败原因。HOLD 成功后仍需 STOP／DISARM 或有效租约持续监督。

## 状态与停止证据

`GcaState` 中 `linear_valid`、`linear_velocity_valid`、`rotation_estimate_valid`、各轴 reference_valid 分别表明不同质量。`feedback_age_s` 来自原始接收时间；发布新消息不能把旧编码器采样变新。`boot_id` 区分进程重启，`device_epoch` 区分设备连接周期。

`confirmed_pulses` 与 `reserved_pulses` 不同。确认脉冲换算出的角度仍是开环估计；`can_measure_rotation=false` 明确没有实际角度闭环。中止计数未知时估计与累计参考失效，绝对／相对旋转定位不能继续靠旧数值运行。

`StopStatus.progress` 为 REQUESTED／IN_PROGRESS／CONFIRMED／UNKNOWN。CONFIRMED 需要反馈零速与脉冲 idle 等证据；写零速指令成功不构成停止确认。具体证据类型在 `evidence_kind`，真实能力验证见 HIL。`INITIAL_DISABLED` 表示启动禁能态，不是某次已发生运动的测量停车报告。

`requested_config_revision` 与 `applied_config_revision` 必须区分。管理服务 accepted 表示请求被受理，不保证异步设备动作已结束；应等待期望状态及相应版本。

## QoS 与队列

| 通道 | 策略 |
|---|---|
| `/gca/state`、`/system/state` | KEEP_LAST(1)、RELIABLE、TRANSIENT_LOCAL；只保存最新快照 |
| Jog topics | KEEP_LAST(1)、RELIABLE、VOLATILE；本地输入期限仍是有效性依据 |
| Action／Service | 标准可靠交互，应用等待有截止时间 |
| `/teleop/state` | `TeleopState` topic | 输入门控/质量、会话状态、意图/已接受速度与有界派发队列；不含凭证 |
| `/diagnostics` | 有界浅队列；不参与闭环 |
| 本地速度需求 | 容量1；旧需求覆盖，但不能覆盖独立STOP撤销 |
| 关键I/O回执／结果 | 有界；溢出故障锁存并作废不再可信的计数证据 |

记录与发布运行在控制线程之外。即使慢可靠发布阻塞 ROS 回调，本地输出 TTL、反馈年龄与停止门控继续工作。

## 常见错误码与权限边界

NOT_ARMED、NOT_ACTIVE、CONTROL_BUSY 表示前置状态不满足；INVALID_TOKEN、SESSION_EXPIRED、STALE_GENERATION、STALE_SEQUENCE 表示授权或顺序失效；ID_CONFLICT 表示同 ID 载荷不同；LINEAR_LIMIT／ROTATION_LIMIT 表示目标越界；FEEDBACK_STALE、OUTPUT_EXPIRED、DEVICE_TIMEOUT 表示监督触发；STOP_TIMEOUT／OUTPUT_STATE_UNKNOWN 表示停止证据不足。所有错误都要结合 state、有效性和 stop 字段处理，不能把错误替换成数值零。

业务会话不是 DDS 身份认证。默认只发现本机节点，ROS_DOMAIN_ID 也不是安全机制。原生 CancelGoal 没有自定义 token 字段，同一可信 ROS 图中的客户端可能调用它；硬件部署必须使用受控主机账户并显式声明可信本机，或配置并强制 DDS Security。跨机权限策略不属于已经验收的本轮功能。故障注入只在模拟后端开启。

撤销后只保留最后一个已认证会话的短期限受限管理记录，用于 STOP／DISARM／释放，以及 simulation 中清除故障注入。它不能续租、ARM、申请运动许可或设置参考／配置；出现新控制者后，旧记录立即不能影响新控制者。故障锁存仍只由显式 RESET 清除，普通停车不恢复运动授权。

## 手动输入质量与STOP屏障

`JogCommand.quality` 使用 `ManualInputQuality`：`source_kind/source_id/input_epoch` 标识来源；`event_seq/event_ns/event_time_source` 保留事件时间；`observation_seq/observation_ns` 表示真实reader观察进展；`valid_until_ns` 是固定原截止；`clock_domain_id` 为实际Linux boot ID，首期只比较同Pi同boot单调时间。`qualification_id/link_evidence` 标记资格依据；`episode_end_ns` 固定程序Jog的一次总时段。

| source_kind | 期限与准入 |
|---|---|
| EVENT_AGE | 真实GCA变化事件年龄；静态握持、右侧变化和心跳不能延期 |
| QUALIFIED_STATE_CONTRACT | 资格记录匹配的reader观察；不是新无线报告或任意RF故障检测保证 |
| SCRIPT_SAMPLE | 明确合成样本，仅simulation |
| PROGRAMMED_JOG | 旧CLI jog(duration)的一次固定episode，后续帧不延长终点 |

active Jog缺质量返回 `MANUAL_QUALITY_REQUIRED`；错误boot/未来或过期时间、回退序号、旧epoch重放及越过history容量均失败关闭。相同真实观察可携带递增命令seq，但固定截止不得改变。权限退休清理对应质量历史；普通STOP不清历史，避免旧epoch绕过。deadman=false保留受限停车路径。

每层取输入原截止、一次性nonce、租约和本地输出TTL的最小值；有限波形须整段落入该窗口，pigpio准备完成后发送前再次核验。控制线程冻结不让I/O获得now+TTL的新期限。`GcaState`新增 `stop_serial/input_valid_until_ns`、Jog受限速度、速度误差/积分和旋转相位/计划率；其中角速度仍是软件估计，不是机械角测量。

B、deadman释放和管理STOP在等待许可前清容量1待发槽并提升Jog epoch/停止闸门。seq≤STOP屏障的请求、旧许可与迟到ARM都不能复活；最终enable/速度写入/有限submit与serial串行化。跨代STOP别名仅限相同owner的受限交接期，不能用于续租、ARM、参考或普通运动；新owner出现后旧owner无权影响它。

本机 `gca_teleop_cli` 不额外claim ROS源会话；它经私有Unix socket请求既有owner执行管理，携带owner_instance、input_epoch、session/generation、reference/config预期上下文并去重。STOP使用独立socket/工作队列，只比较仍属当前owner/输入的停止必要上下文，避免慢ARM或配置变化使B失效。它是同用户本机接口，不提供跨机身份认证。完整命令见 [手柄指南](TELEOP_USER_GUIDE.md)。
