# ROS2 上位机位移 PID 闭环实施方案

本方案在现有舵机速度控制链上增加外部编码器位移闭环。上位机提交一个目标位置，树莓派根据编码器测得的实际位移修正舵机转速，直到到位并停车。手柄继续直接控制舵机 RPM，不采集外部编码器、不使用外部位置或速度修正杆量。

本方案对应的位置软件、统一启动入口和持久客户端已实现；使用命令见[位置用户指南](POSITION_USER_GUIDE.md)，软件、ROS 仿真、实机与跨机验证结果分别见[实现状态](POSITION_PID_IMPLEMENTATION_STATUS.md)。方案及实现经过运行时、编码器与 PID、ROS 接口三个 agent 的交叉讨论；原始任务约束保留在 [实施 goal prompt](POSITION_PID_ROS2_GOAL_PROMPT.md)。后续实机中，紧固后 100 mm、2 mm/s 推进通过，8 mm/s 回零超调后由测试脚本停车。本阶段按用户要求先归档当前实现，未修正的超调和期限错误码问题保留在状态文档中。

## 1. 实现边界

首期支持单轴推进的绝对位置、相对位移、上位机 STOP、Action 取消和有限执行超时。一次只执行一个任务，任务以停车结束；暂不做 HOLD、目标排队、运动中替换目标、导管绕轴旋转或磁场联动。

增加统一的显式模式开关 `control_mode:=manual|position`，选择手柄或上位机位置控制。保留现有手柄入口作为兼容封装，位置入口也调用同一装配逻辑；两者互斥运行，共用 `ServoControlNode`、`FeedAxisRuntime`、舵机驱动及 STOP 流程，不复制第二套执行框架。

| 启动装配 | 控制策略 | 外部编码器 | 输入及输出 |
|---|---|---|---|
| 现有手柄入口 | `MANUAL_SERVO_VELOCITY` | 不创建、不打开 | LY 决定 RPM，LT 使能，B 停车 |
| 新位置入口 | `POSITION_EXTERNAL_ENCODER` | configure 时开始读取 | AUTO 会话提交位移任务，位置 PID 输出 RPM |

手柄的开环指不使用外部编码器闭环；舵机自身的内部速度调节、轴状态读取和既有停止保护继续保留。编码器缺失不能影响手柄入口启动和使用。

切换流程为：STOP 确认 → DISARM → 释放会话 → 退出旧装配 → 启动另一装配。首期不做同进程热切换。新位置装配需显式建立参考，不能假定手柄期间机构没有移动。

### 1.1 模式控制开关

统一入口为 `system_bringup/feed_axis.launch.py`，必须显式指定 `control_mode` 和 `backend`。以下展示本机硬件入口的参数形式，路径占位符需替换为组合导出的匹配快照；手柄还需按[舵机指南](SERVO_USER_GUIDE.md)传入已标定 profile。完整可执行步骤见[位置用户指南](POSITION_USER_GUIDE.md)。

```bash
# 手柄开环控制
ros2 launch system_bringup feed_axis.launch.py \
  control_mode:=manual backend:=hardware \
  servo_config_file:=/实际路径/manual.json trusted_local_control:=true

# 上位机位移闭环控制
ros2 launch system_bringup feed_axis.launch.py \
  control_mode:=position backend:=hardware \
  servo_config_file:=/实际路径/position.json trusted_local_control:=true
```

| 控制开关 | 用户含义 | ControlSession.mode | 固定执行策略 |
|---|---|---|---|
| `manual` | 手柄控制 | `MANUAL` | `MANUAL_SERVO_VELOCITY` |
| `position` | 上位机位移控制 | `AUTO` | `POSITION_EXTERNAL_ENCODER` |

小写开关值与会话枚举不同，客户端不得将 `position` 直接传给已有会话服务。模式进入导出的不可变运行配置；launch 参数必须与快照匹配，否则在连接硬件前报错。协调器、执行节点和核心均核验固定策略：手柄模式拒绝 AUTO 位移任务，位置模式拒绝 MANUAL 控制会话及手柄 Jog。不会因收到另一类命令、手柄上线或编码器故障而自动切换。

新统一入口不为省略值选择模式；旧手柄 launch 可明确固定为 `manual`，保留旧快照的已知手柄语义。运行后拒绝通过 `ros2 param set` 改变模式。启动只完成装配，不自动 claim、reference、ARM 或运动。

`/system/state` 复用已有 `GcaState.control_policy` 显示实际模式，另显示当前控制权所有者及 armed 状态；不能只显示客户端上次选择的值。读取模式不需要新增服务。该开关首期提供启动时选择和停机后重启切换，不提供运动中的即时接管。

切换时，当前持权客户端先拒绝继续提交运动并发起 STOP，等待确认，再 DISARM、释放会话并关闭旧装配。若停车未确认或旧装配尚未释放设备，切换失败，新装配不得接管串口；不得通过强制杀进程或抢占锁伪造切换成功。新装配始终从未使能状态开始；返回手柄需释放 LT、杆量回中再按原流程使能，返回位置需重新建立参考。

## 2. 复用结构

```mermaid
flowchart TB
    PC[上位机持久客户端] --> ACT[/system/execute_motion]
    JOY[手柄 LY LT B] --> MAN[/system/servo_manual_target]
    ACT --> CO[system_coordinator 单一控制权]
    MAN --> CO
    CO --> NODE[ServoControlNode 生命周期与 Action]
    NODE --> CORE[FeedAxisRuntime 单一控制所有者]
    ENC[编码器独立 I/O 线程 仅位置装配] --> LATEST[最新有效样本与故障事件]
    LATEST --> POS[位置估计 轨迹 PID 到位判定]
    CORE --> POS
    POS --> DEMAND[原生 RPM 目标]
    CORE -->|手柄直接调速| DEMAND
    DEMAND --> IO[唯一舵机 I/O 所有者]
    IO --> SERVO[真实舵机或模拟舵机]
```

ROS 回调只做请求校验、提交和结果通知。控制线程处理任务状态、估计、轨迹和 PID；两个串口各由自己的 I/O 线程持有。串口等待、写盘和 DDS 发布不进入控制状态锁。

位置策略及外部反馈归入独立 `gca_position` 功能包，内部按职责分为策略、估计和采集模块。共享算法仍复用 `gca_core`；不把这些模块分别拆成 ROS 节点，也不引入通用插件框架。

### 2.1 ROS2 功能包划分

采用现有六个包加两个新包的划分，按可独立维护和部署的职责分包。功能包不等于节点或进程；高频反馈和 PID 继续在设备进程内通信，不额外绕经 DDS。

| 功能包 | 状态 | 责任及边界 |
|---|---|---|
| `gca_system_interfaces` | 保留 | 消息、Action、Service 定义；不放控制算法、驱动或业务代码 |
| `gca_core` | 保留 | 无 ROS 的领域契约、公共算法、执行状态、授权/STOP 基础、现有驱动与模拟基础；不依赖具体位置包或客户端 |
| `gca_position` | 新增 | 外部编码器采集组件、位置反馈与速度窗口、位置任务策略；只依赖核心契约和可复用实现，不导入 `rclpy` |
| `gca_ros` | 保留并收窄职责 | 设备侧 ROS 生命周期、Action 适配、消息转换和组件装配；不堆积 PID、编码器解析及上位机客户端业务 |
| `system_coordinator` | 保留 | 控制模式准入、唯一源会话、许可、任务代理和停止协调；不持有串口，不计算 PID |
| `gca_teleop` | 保留 | 手柄读取、映射、LT/B 门控和 MANUAL 客户端；不包含位置闭环 |
| `gca_client` | 新增 | 上位机持久客户端、线性指令 CLI、会话续租、取消与状态查询；不依赖服务端实现或硬件驱动 |
| `system_bringup` | 保留 | 统一模式开关、节点装配、配置路径、发现范围和退出管理；不承担控制算法或另一套权限逻辑 |

新增业务模块如下；位置组件在 `gca_position/assembly.py` 装配并由设备节点注入运行时：

```text
gca_position/
  strategy.py       目标、轨迹、PID、到位与策略重置
  feedback.py       核心计数估计的适配、软件参考和速度窗口
  acquisition.py    复用 SerialEncoder 的 I/O 所有者及最新样本交换
gca_client/
  client.py         持久会话、管理服务和 Action 客户端
  cli.py            上位机命令行入口
```

依赖方向必须明确：`gca_position → gca_core`；`gca_ros → gca_core/gca_position/gca_system_interfaces`。由 `gca_ros` 根据模式构造位置策略和采集组件，再按 `gca_core` 定义的 Protocol 注入 `FeedAxisRuntime`。`gca_core` 不导入 `gca_position` 的类或配置类型；手柄装配不构造位置组件。仍复用现有 `SerialEncoder` 和舵机驱动，不为拆包复制串口代码。

`gca_client` 只依赖 ROS 客户端库、接口包、生命周期消息等必要客户端依赖。现有 `GcaClient` 中的客户端逻辑迁入该包，所需消息转换归入客户端边界，不能继续导入 `gca_ros.conversion` 或为少量手柄字段依赖含驱动的 `gca_core`。旧 `gca_ros.cli` 只保留薄兼容转发及已有公开符号，允许 `gca_ros → gca_client` 的兼容依赖，但严禁反向依赖，避免循环。

此次只分离新增位置业务与上位机客户端，不全面搬迁已有驱动、旧双轴算法或重新划分全仓库。各包更新自己的 `package.xml`、安装入口和依赖声明；设备端组合构建，上位机可独立安装客户端及接口相关依赖。

### 2.2 现有代码调整位置

| 现有组件 | 用法及必要调整 |
|---|---|
| `gca/runtime/feed_axis.py` | 按固定策略处理位置任务、参考与手柄速度；保留舵机 I/O、授权、STOP 和有界队列 |
| `gca/domain/position.py` | 保留共享契约、`MechanicalMapping`、`resolve_braking_parameters`；具体位置策略在 `gca_position` 实现并注入 |
| `gca/devices/encoder.py` | 原驱动复用；位置专用采集所有者放在 `gca_position/acquisition.py` |
| `gca/estimation/linear.py` | 原计数展开及参考复用；有限速度窗口放在 `gca_position/feedback.py`，旧双轴默认行为不变 |
| `gca/algorithms/control.py`、`trajectory.py` | 复用 `PositionController`、`Trapezoid` 和限幅函数 |
| `gca_ros/servo_node.py`、`conversion.py` | 按策略处理 ARM、Action、能力、状态和停止证据 |
| `system_coordinator/node.py` | 按配置放开 AUTO 位置任务，复用会话、许可、取消和终态提交 |
| `gca_ros/cli.py`、`gca_client` | 客户端逻辑迁入新包，旧 CLI 兼容；新增明确线性用法，避免双轴默认值进入位置任务 |
| `system_bringup` | 实现统一 `control_mode` 入口，原 launch 作为薄封装，不复制节点装配逻辑 |

## 3. 配置与参数

人工配置仍只来自 `config/servo_settings.py` 和 `config/encoder_settings.py`。`tools/export_feed_axis_config.py` 调用这两个脚本的配置函数，生成 schema 3、含修订号及摘要的位置运行快照；不增加第三份手工参数表。

快照组合舵机运行参数、舵机位移参数、编码器参数，以及互相一致的 `control_mode` 和执行策略。模式由导出工具的显式选项传入，不另设容易漂移的 Python 模式常量。扩展现有 `gca/servo_config.py` 的严格类型和摘要校验，手柄旧快照继续按原语义读取；位置配置不能因缺字段而悄悄降级。仍通过 `servo_config_file` 交给设备节点和协调器，运行节点不执行配置脚本。改参数后重新导出并重启，不做在线修改。

| 参数组 | 本轮使用值 |
|---|---|
| 舵机机构 | 半径 6 mm，直驱，`37.6991118 mm/输出轴圈`，正推进对应负 RPM |
| 编码器 | `BRT38-ROM-4096-D1800-RT1-IP68`；`/dev/ttySC1`、38400、8N1、站号 1、功能码 03、总计数寄存器 0～1 |
| 编码器换算 | 半径 10.89 mm、直连、4096 count/圈、循环 7,372,800；`0.01670505078 mm/count`；正推进计数增加 |
| 编码器时序 | 读取 0.02 s、事务超时 0.05 s、反馈有效期 0.1 s、连接静默等待 0.1 s |
| 异常速度阈值 | `1.5 × motion.max_linear_mm_s`，当前 15 mm/s；仅用于位置策略读数合理性判断 |
| 位置 PID | `kp=2`、`ki=0`、`kd=0`、`kaw=2`、积分限幅 1 mm/s、最大样本间隔 0.2 s |
| 位置运动限制 | 最大速度 10 mm/s、最大加速度 10 mm/s² |
| 行程及余量 | 软件行程 −50～1000 mm；两端各保留 10 mm，首期允许目标为 −40～990 mm |
| 到位条件 | 最终误差 ≤1 mm、外部测量速度绝对值 ≤0.1 mm/s，持续 0.3 s |
| 制动计算 | 实测减速度和延迟仍为 `None`，使用已授权缺省值 5 mm/s²、0.5 s，并记录来源 |
| 估计参数 | 编码器脚本的 `estimation.velocity_window_s=0.2`，仅位置策略使用 |

编码器已有一次约 2 秒、100 次成功查询的[耗时记录](../artifacts/encoder_readonly/timing-7x7x_4_p/report.json)：中位数 8.03 ms、最大 13.25 ms，20 ms 查询节拍无漏周期。此记录支持初始采集参数，不需要在实现前重新做串口压力测试。

舵机控制线程暂沿用现有 0.04 s 周期，编码器仍按 0.02 s 采集。控制线程消费容量为 1 的最新样本，可以跳过中间序号，但不能重复积分；实际 dt 按被消费样本的原时间计算。真实舵机事务吞吐独立于这些目标周期，不宣称舵机已经达到 50 Hz 更新。

## 4. 位置参考与上位机接口

位置入口 configure 后保持 DISARMED，编码器已经采集。操作流程为：configure、activate → 领取 AUTO 会话并持续续租 → 显式设置线性参考 → 等待约 0.2 秒速度窗口有效且静止 → ARM → 提交位移任务。`GcaClient.set_linear_reference()` 会等待新参考及有效速度窗口；未建立参考前不应等待 `linear_valid=true`。

软件参考把当前编码器位置映射到指定坐标，例如 0 mm；不写编码器硬件置零寄存器。同一装配运行期间，多个目标共用这一固定坐标系。绝对 20 mm 表示固定零点前方 20 mm；相对 +5 mm 在正式接纳任务时，由新鲜实测位置解析为一次固定的绝对目标，执行中不反复累加。

退出位置装配、设备重连或编码器 epoch 改变时，旧任务、参考和许可失效；恢复后显式重建参考、ARM 和新任务，不续跑历史目标。即使圈数可断电记忆，也不自动恢复软件任务坐标。

复用以下现有接口；模式由第 1.1 节的显式启动开关选择，不新增位移 Topic 或热切换服务：

| 用途 | 系统入口 |
|---|---|
| 控制权 | `/system/claim_control`、`renew_control`、`release_control` |
| 参考与使能 | `/system/set_reference`、`arm`、`disarm` |
| 任务 | `/system/execute_motion`，使用现有 `ExecuteMotion` Action |
| 停止与结果 | `/system/request_stop`、Action cancel、`get_command_status` |
| 状态及能力 | `/system/state`、`/gca/get_capabilities`；无 `/system/get_capabilities` |

Action 限定 `axis_mask=LINEAR`、`mode=ABSOLUTE/RELATIVE`、`end_policy=STOP`，旋转目标为零。必须有有效 reference、命令身份、会话和有限的接纳/执行超时；任务忙时拒绝新任务。相对目标解析后的范围也必须校验。重复命令复用现有去重语义，不重复执行。

ROS 边界使用 m、m/s、m/s²，核心使用 mm、mm/s、mm/s²；设备侧转换只在 `gca_ros/conversion.py` 完成。上位机客户端的用户单位在 `gca_client` 边界转换为 ROS 单位，不调用服务端转换模块。`GcaClient.make_linear_goal()`／`move_linear()` 显式发送 LINEAR 和零旋转目标，并检查能力上限：例如 +5 mm 对应 `linear_m=0.005`，全局速度/加速度上限分别对应 `0.01 m/s`、`0.01 m/s²`。兼容保留的 `make_goal()` 双轴默认值不用于位置入口。

首期上位机使用 ROS2 客户端；不另建 TCP 网关。位置 launch 显式提供本机与局域网发现配置，双方 ROS_DOMAIN_ID 一致；不继续无条件强制 LOCALHOST。授权与许可保留，远程客户端不发送被当作树莓派单调时钟的时间戳。树莓派本机联调通过只证明本机链路，跨机通信结果单列。

## 5. 闭环计算

处理顺序为：有效原始计数 → 回绕展开 → 软件参考位置 → 有限窗口速度 → 梯形轨迹 → 位置 PID → 限速、限加速度、边界制动 → 有符号 RPM → 唯一舵机 I/O 所有者。

```text
x_mm = reference_position_mm + encoder.direction_sign × encoder.mm_per_count × (unwrapped_count − reference_count)
v_requested = v_ref + kp × (x_ref − x_mm) + I − kd × v_measured
rpm = servo.direction_sign × 60 × v_limited / servo.mm_per_shaft_revolution
```

`reference_count` 是建立参考时的连续计数，`reference_position_mm` 是当时指定的坐标值，不能固定假设为零。当前 `ki=kd=0`，先采用位置比例项加轨迹速度前馈。保留现有积分、测量微分及抗饱和接口，不再叠加外部速度 PI。软件限幅回算一次，驱动实际接受 RPM 换算成 mm/s 后只回算额外量化残差；重复或旧代次回执不能再次回算。

速度和加速度同时受目标请求与配置上限约束；输出的时间间隔使用实际控制输出间隔。最终舵机寄存器量化也须遵守位置速度上限，不能先限幅再向上舍入出一个超限目标；复用现有原生速度量化实现，在位置路径传入适用限制，不改变手柄语义。边界制动调用 `resolve_braking_parameters(...).limit_velocity()`，不要照搬旧双轴代码中使用指令加速度作为制动能力的路径。

## 6. 速度估计与到位

一个编码器计数约 0.016705 mm，20 ms 差分对应约 0.835 mm/s。短测中读数在 95135、95136 之间波动，直接差分可能使 0.1 mm/s 的停稳条件一直不成立。

位置策略使用约 0.2 s 的有界速度窗口：以展开后的位置差除以实际时间差，位置本身仍随新样本更新；不要固定按 10 帧或固定分母计算。窗口未形成、采样失效、断档或参考/epoch 改变时，速度无效并重建窗口。不得以速度填零或放宽用户阈值通过到位判定。旧双轴估计默认行为不变。

正常任务结束分四步：

1. 轨迹结束且最终误差 ≤1 mm，目标速度归零并按限制减速；此时尚未成功。进入正式 STOP 前，若误差再次超限，可继续修正同一目标。
2. 仅新鲜、序号不同、速度窗口有效的样本，连续满足误差 ≤1 mm、速度绝对值 ≤0.1 mm/s 共 0.3 s。按样本时间累计，条件失败就清零，不用重复快照凑驻留时间。
3. 进入既有 STOP，清除目标、积分和待发输出，等待实际停止证据。
4. 新鲜舵机停止证据与新鲜编码器最终位置/速度均满足要求后，才返回 SUCCEEDED、`outputs_held=false`。正式 STOP 后若最终条件失败，报告失败，不自动重新运动。

取消、上位机 STOP、执行超时、失去会话或编码器异常均直接进入 STOP，不等待上述驻留。舵机已经停车而编码器失效时，可以报告舵机停车证据，但不能报告线位移已到达或任务成功。

## 7. 授权与通信时序

`FeedAxisRuntime.arm()`、`_valid_output()`、`_enable_allowed()`、最终写入检查及 `ServoControlNode._arm_runtime()` 已按固定策略分支。位置分支检查 AUTO 会话、位置任务、参考和编码器反馈，不需要手柄 quality 或 LT；手柄分支保留既有输入期限。

保留以下独立时间约束：

| 约束 | 当前预算及处理 |
|---|---|
| 编码器采集 | 20 ms 查询、50 ms 超时，单个未完成事务 |
| 编码器新鲜度 | 100 ms；使用原采样时间，查询起始时间保留用于保守计龄和诊断 |
| 舵机事务 | 每次完整 WRITE/ACK 或 READ 最多 80 ms，不因编码器剩余年龄而截短 |
| 舵机遥测有效期 | 沿用 250 ms，不被编码器的 100 ms 覆盖 |
| 输出有效期 | 沿用固定 300 ms 和租约约束；重复发布不续期 |

每个位置 RPM 目标携带它实际使用的编码器样本时间、seq、epoch 和 reference_id。最终发送前分别检查原样本新鲜度、当前编码器健康、任务/授权代次及 STOP 屏障；不能用新样本的时间为旧 PID 输出续期。

不要要求“完整 80 ms 舵机 ACK 必须塞进编码器样本的 100 ms 窗口”：样本等待超过 20 ms 就可能无法发出命令，重演此前的间断问题。100 ms 用于开始事务前的新鲜度判定；已开始事务按原有界截止结束，期间可以置 STOP 撤销标志，结束后优先零速/关力矩。100 ms 不是物理停车时间承诺。

编码器 I/O 故障需要明确发布并撤销位置输出，不能只检查旧 `LinearEstimator.state.valid`；现有估计器收到无效样本时可能返回旧状态。舵机 I/O 独立检查位置输出引用的原反馈是否过期，即使控制线程停滞也不能继续使用旧目标。参考和设备代次不匹配的输出与回执均拒绝。

## 8. 状态与操作边界

位置模式的 `GcaState.linear_*`、参考、通用位置反馈年龄和样本身份来自外部编码器；嵌套 `ServoState` 保留舵机轴数据和独立年龄。`StopStatus.servo_confirmed` 与 `linear_confirmed` 分别来自舵机和编码器，不相互替代。观测过期会撤销相应证据，协调器不保留已经过期的停止确认标志。

手柄模式保持外部位移无效，不发布推算位置冒充测量。AUTO 模式下不要求手柄开机、LT 或 B。现有 B 停车依赖手柄的 MANUAL 会话，首期不增加跨所有者权限；AUTO 通过上位机 STOP 或 Action cancel 停止，不能在指南中宣称 B 已能停止 AUTO。

日志只增加诊断闭环必需的数据：目标和实测位置、误差、窗口速度及有效性、编码器 seq/epoch/原时间、所用 PID 样本、限幅前后速度、请求/接受/实测 RPM、控制策略、制动参数来源、终态及停止证据。沿用有界异步 JSONL，不输出授权 token。

## 9. 实施顺序与有限验证

实现已按共享契约、配置与位置组件、运行时、ROS 接口和持久客户端的顺序完成。模拟位移由实际接受的舵机输出经过机构模型产生，不直接赋值为目标；`tools/verify_position_simulation.py` 使用一次启动及固定滑移工况检查闭环行为。实际结果与证据以[实现状态](POSITION_PID_IMPLEMENTATION_STATUS.md)为准，不以本节列出的验证范围代替通过记录。

只安排以下验证，不按函数数目堆叠测试：

- 配置、方向、单位、绝对/相对目标、边界、参考与重复命令；复用现有算法测试。
- 将模式未指定/非法、开关与快照不匹配、错误来源命令拒绝、切换后未使能及无旧任务恢复合并到上述用例；检查新包无循环依赖、客户端无需导入设备包、旧 CLI 兼容，不额外增加实机测试轮次。
- 新样本更新、双向回绕、单计数噪声下的停稳、失效后窗口重建。
- 编码器过期/错误/epoch 改变、租约失效、取消与成功竞争、旧输出及回执拒绝；使用软件注入。
- 100 ms 样本判定与完整 80 ms 舵机事务的分离；MANUAL 在编码器不存在时仍运行。
- 一次正式安装后的 ROS 仿真启动，合并正反向绝对/相对任务、滑移修正和取消/STOP，脚本有有限总超时；通过后结束。
- 现场条件具备时最多一组短程实机：建议相对 +5 mm 后绝对回到 0 mm，速度 2 mm/s、加速度不超过 5 mm/s²，每个任务最多 10 s。开始前只确认驱动轮与测量轮作用于同一被测对象、行程可用且操作者在场；不沿用“舵机空载”推定机构已接合。若编码器没有相应变化或发生故障立即结束并分析记录，不反复试到成功。

实机专项失败只重测修复所影响的项目；不重做手柄映射、无线资格、长时间稳定性测试或已延期的总线静默专项。软件、ROS 仿真、实机和跨机通信分别报告结果；没有现场条件的项目列为 NOT_RUN，不把软件完成写成实机闭环已验收。
