# 手柄输入与模拟执行器操作指南

> 当前实现已完成，491项软件测试通过；进一步普通安装与完整集成复验已按用户要求暂停。当前状态见[实现状态](IMPLEMENTATION_STATUS.md)，本页可直接使用独立终端输入工具。

`gca_teleop` 已有只读joydev、合成输入、映射/门控、单一会话owner、本机管理CLI和ROS转发实现。本轮普通安装、455项软件回归及完整手柄仿真已通过，证据见 [实现状态](IMPLEMENTATION_STATUS.md)。实体15步映射的schema2复核已通过，后续LT只读测试也已确认实际按下和释放阈值跨越；该次按下后的完全释放端点未观测，不能据此认定运动或无线资格通过。

第四轮实体输入→simulation已关闭，两个推进stage均未产生运动；具体输入时间线与历史尝试见实现状态。[原始joydev与安装后NativeJoydev的只读对照](../artifacts/teleop_implementation/input/parallel_input_probe.attempt2/assessment.json) 已结束：各16个变化事件按序一致，含B键多次按放；灯状态仍未报告，event4/5权限不可读且未更改权限。第五轮复测也已结束：[评估](../artifacts/teleop_implementation/input/physical_simulation.attempt5/assessment.json)确认短暂模拟推进，编码器位置增加0.05mm；两次旋转试验虽收到最高30°/s意图，确认脉冲仍为0，不能判旋转通过。所有模拟节点已停止，继续使用独立终端工具定位输入；只读对照通过不能代替运动或无线资格。操作者仍报告设备模式不清楚，无线资格NOT_RUN，默认未核实profile保持锁定。所有下述运动示例均连接simulation。

## 按键与启动门控

| 输入 | GCA 意图 |
|---|---|
| LY 上/下 | 推进/退出速度，推进为正 |
| LX 左/右 | 负/正旋转速度，物理方向还须标定 |
| LT | 双轴 deadman；按住才允许输入，不等于 ARM，不调节速度 |
| B | 优先软件 STOP；释放 B 不自动恢复 |
| RT、RB、RX/RY、RS | 预留磁场；当前不产生 GCA 意图，也不刷新左侧事件期限 |

实体轴号、符号、中心和量程从只读向导得到，不能照搬合成 profile。每次启动、重连、重新领取会话、参考/配置改变、STOP 或输入失效后，都要观察到 **LT 释放且左杆回中，再有一次新的 LT 按下**。还需 active、有效双轴参考、已应用配置和显式 ARM。首期手柄固定 `axis_mask=3`，一轴参考失效会锁住整个手柄入口，不静默改成单轴。

回中表示零速度意图：按限幅减速，清除历史积分/未提交整脉冲积压；它不会追赶旧位移或进入 HOLD。LT 松开和 B 则沿原 STOP 路径立即撤销并中止有限波形，不等待温和排空；未知中止可能使旋转参考失效。

## 独立终端读取与操作判定（无需 ROS 2）

在工程目录直接运行，使用系统 Python，不需要 `source` ROS、colcon 构建或启动模拟执行器：

```bash
python3 tools/joystick_monitor.py
```

默认只读原始轴/按键编号与数值，不猜测任何手柄映射。自动发现唯一匹配的joydev设备；多个设备时拒绝歧义选择。按键变化和状态更新直接显示在当前终端，按 `Ctrl-C` 退出。初始化基线单独标记，不能算作一次真实按下。

使用已完成实体标定的配置，观察LX/LY、LT、B与门控判定：

```bash
python3 tools/joystick_monitor.py \
  --profile artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml
```

这条配置引用本机已采集的原始标定证据。换手柄或配置不匹配时，仍可观察原始数据，但不能据此得到有效运动判定。配置读取需要PyYAML（当前工程环境已具备）；没有ROS Python依赖。诊断上下文仅用于本地计算，不领取会话、不ARM、不发送推进/旋转/磁场命令。

日志分别展示原始输入、映射结果、门控意图与输入年龄。即使静态摇杆仍显示非零，`event_age`达到0.15s后也会撤销运动意图；原始状态显示不会因此停止。`READY/ACTIVE/STOP`均为**离线输入判定**，不是设备已使能或已停车的反馈。reader健康只说明本地读取过程，不证明无线链路在线。

常用日志字段：

| 字段 | 含义 |
|---|---|
| `INIT_BASELINE` / `CHANGE` / `CACHE` | 初始缓存、真实值变化、没有新变化的观察 |
| `axes` / `buttons` / `changed` | 完整原始数组及此次变化编号 |
| `LX` / `LY` / `LT` / `B` | 按profile解释的输入；LX/LY已应用死区与曲线 |
| `gate` / `decision` / `reason` | 离线门控、JOG/STOP等判定及拦截原因 |
| `v` / `w` | 门控后速度意图，单位mm/s与deg/s；不是执行器反馈 |
| `raw_age` / `gca_age` / `reader_age` | 原始事件、最近GCA有效事件及本地读取心跳的年龄 |
| `radio=UNKNOWN` / `log_dropped` | 无线在线状态未获证明；终端日志丢弃总数 |

先单独拨动左杆，确认轴值；再观察LT按放和B按放；最后观察组合输入。无需在聊天中逐次回复，直接在自己的终端操作、看日志。可以限定时长并保存结构化终端输出：

```bash
python3 tools/joystick_monitor.py \
  --profile artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml \
  --duration 30 --jsonl > /tmp/joystick-monitor.jsonl
```

日志输出与采集分离并有容量限制。终端/重定向阻塞时应查看丢日志计数，不能把有丢弃的显示当成完整原始证据。脚本复用与ROS节点相同的`NativeJoydev`、映射和状态机；源码入口为[joystick_monitor.py](../tools/joystick_monitor.py)，实现为[monitor.py](../ros2_ws/src/gca_teleop/gca_teleop/monitor.py)。

## 安装环境与只读识别

每个终端先执行：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
```

首次安装和修改后的完整检查由工程根的 `bash tools/bootstrap_dev.sh`、`bash tools/verify_software.sh` 完成。下面查询只打开输入设备读取元数据，不 claim、不 ARM，不启动 GCA 或 pigpiod，也不发送震动：

```bash
ros2 run gca_teleop gca_teleop_probe inspect \
  --profile "$(ros2 pkg prefix --share gca_teleop)/config/beitong_unverified.yaml" \
  --output /tmp/teleop-identity.json
```

已观测接收器为 `20bc:506d`、北通 2.4GHz，当前可读 joydev；元数据不能证明实际按键映射或无线连接活跃。当前完整只读映射证据见 `artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.validation.json`（15份原始文件SHA）；不要把默认profile的 `mapping=null` 改成猜测索引来解除锁定。

## 可重复的纯软件操作

先建立一份合成脚本；轴号只适用于安装的 `synthetic.yaml`：

```bash
cat > /tmp/teleop-example.json <<'JSON'
{
  "schema_version": 1,
  "duration_s": 3.0,
  "events": [
    {"at_s": 0.0, "axes": {"0": 0, "1": 0, "2": -32768}, "buttons": {"1": 0}},
    {"at_s": 0.4, "axes": {"2": 32767}},
    {"at_s": 0.6, "axes": {"1": -16000}},
    {"at_s": 1.6, "axes": {"1": 0}},
    {"at_s": 2.0, "axes": {"2": -32768}}
  ]
}
JSON
```

终端 A：

```bash
ros2 launch system_bringup teleop_simulation.launch.py \
  profile_file:="$(ros2 pkg prefix --share gca_teleop)/config/synthetic.yaml" \
  script_file:=/tmp/teleop-example.json \
  record_directory:=/tmp/teleop-example-records
```

普通 launch 不 configure、activate、claim、设参考或 ARM。合成输入也默认暂停；加载脚本本身不会运动。终端 B 通过同一个 owner 的本机管理代理操作：

```bash
ros2 run gca_teleop gca_teleop_cli configure
ros2 run gca_teleop gca_teleop_cli activate
ros2 run gca_teleop gca_teleop_cli claim
ros2 run gca_teleop gca_teleop_cli reference --linear-mm 0 --rotation-deg 0
ros2 run gca_teleop gca_teleop_cli arm
ros2 run gca_teleop gca_teleop_cli script-start
ros2 run gca_teleop gca_teleop_cli status
```

按 JSON 的时间线，脚本完成“释放/回中→新 LT→推进→回中→释放 LT”。这是显式合成的 `SCRIPT_SAMPLE`，新采样可以维持软件测试意图；它不证明实体静态握持仍有新无线报告。

停止与退出：

```bash
ros2 run gca_teleop gca_teleop_cli stop --wait
ros2 run gca_teleop gca_teleop_cli script-stop
ros2 run gca_teleop gca_teleop_cli disarm
ros2 run gca_teleop gca_teleop_cli release
```

随后在终端 A Ctrl-C。`stop --wait`同时要求本次stop epoch/回执/serial的`stop_confirmed=true`与CONFIRMED，不能用缓存的旧CONFIRMED提前退出；单次 `stop` 返回受理不等于物理停止。代理默认 socket 为 `/tmp/gca-teleop-<uid>/control.sock`，STOP 使用独立 `.stop` socket。需要自定义时，launch 的 `socket_path:=...` 与 CLI 的 `--socket ...` 保持一致；CLI 全局参数放在子命令之前。目录须属于当前用户且为私有权限，不能复用其它用户路径。

**不要同时运行 `gca_cli shell` 再 claim。** 控制权由持续运行的 teleop owner 持有并续租，管理 CLI 通过同 UID 的本机代理复用它，不读取或输出 token。冲突、旧 owner/epoch/参考/配置上下文会被拒绝，不能通过反复 claim 绕过。

当前已生成的实体event_age profile仅允许输入→simulation，操作者准备好重新只读核对并开始新的输入→simulation试验时，用同一安装环境启动：

```bash
ros2 launch system_bringup teleop_simulation.launch.py \
  profile_file:="$PWD/artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml" \
  record_directory:=/tmp/teleop-physical-simulation
```

再用同一owner CLI执行configure、activate、claim、静止reference和ARM；实体保持LT释放/左杆回中，确认门控允许后新按LT再操作，B随时请求软件STOP。此命令仍明确使用simulation；不设置script_file，不在真实输入失效时切换合成源。各轮真实结果见[实现状态](IMPLEMENTATION_STATUS.md)：第五轮仅确认短暂推进，持续双轴操作尚未验收；reader健康不能证明radio在线，本页命令不是成功记录。实际轴号仅对应这次证据：LX0、LY1(sign=-1)、LT5、B1；右侧axes[2,3,4]/buttons[7,14]预留，8/9未分配。

## 两种实体输入模式

`event_age` 是默认保守模式。只有 GCA 相关的真实有效变化才更新意图年龄；静态保持、重复缓存、右侧控件变化、读线程心跳、许可回执和定时发布都不能续期。默认 `input_max_age_s=0.15`，静态推杆并持 LT 超过期限会停车，并要求释放 LT/回中/新按下。这个限制是当前可交付行为，不能靠周期伪造事件消除。

`qualified_state` 允许保持经过资格确认的驱动状态。它要求当前设备身份、驱动/内核、操作者确认的接收器模式、映射与资格原始记录一致；reader 必须持续推进真实观察。原事件时间仍保留，观察时间独立记录。缺记录、错配、冻结、断开、INIT/重同步或证据失效时停车，不能自动降级为 event_age 后继续运动。合成资格测试不改变实体资格，也不声称能检测任意无线故障。当前无序列号接收器若被同元数据设备在同一端口替换，软件身份摘要不能保证区分，需操作者重新确认/资格验证。

当前资格 **NOT_RUN**，操作者报告设备模式不清楚（记录标签`operator_reported_unknown_2.4GHz`），未声称XInput/DInput已确定；只允许保守的输入→simulation。LT阈值和B原始事件已有只读证据，第五轮仅观察到短暂模拟推进，旋转确认脉冲为0，不能声称实体手柄持续双轴控制已验收。该轮和临时采集均已关闭，现在可使用独立终端工具检查输入。`hardware` 还独立要求 GCA 的身份、接线、标定及 H01–H03；“有输入资格”不能代替执行器验收。

## 完整映射与资格向导

向导只读输入，不启动 GCA。记录操作者对当前模式的真实陈述；不清楚时明确记录`operator_reported_unknown_2.4GHz`，不能把USB元数据猜成已确定的XInput/DInput。该未知标签不是模式资格通过。下例由操作者输入记录标签：

```bash
read -r -p '输入已核实的手柄/接收器模式标签: ' TELEOP_DEVICE_MODE
ros2 run gca_teleop gca_teleop_probe mapping \
  --profile "$(ros2 pkg prefix --share gca_teleop)/config/beitong_unverified.yaml" \
  --device-mode "$TELEOP_DEVICE_MODE" \
  --capture-s 3 \
  --output /tmp/teleop-mapping.json \
  --profile-output /tmp/teleop-mapped.yaml
```

依次采 LX 左/中/右、LY 上/中/下、LT 释放/按下、B 按/松、RT、RB、RX、RY、RS。每步在提示采样开始后只操作指定控件；中心步骤需在窗口内从偏移回中，不能只有 INIT/缓存。完整证据包含原始事件、身份、映射与摘要；已存在文件不覆盖。向导成功输出的是 **未激活** profile，不能自己 claim 或 ARM。

需要长期静态保持时，先为无线/reader失效试验预声明 D。以下示例的 `0.15` 是演示输入界限，操作者应根据独立测试计划预先选择，不是已确认的本接收器指标：

```bash
ros2 run gca_teleop gca_teleop_probe qualify \
  --profile /tmp/teleop-mapped.yaml \
  --max-disconnect-s 0.15 \
  --output /tmp/teleop-qualification.json \
  --profile-output /tmp/teleop-qualified.yaml
```

按向导进行各场景重复试验并作真实操作声明；缺一项、超时或身份变化均不合格。D 从操作者发出断链动作的本机提示/声明开始，不是软件直接测得的 RF 失联时刻。资格是有适用边界的工程证据，USB 接收器仍在线不证明无线手柄在线。

应用新 profile/切换模式必须先 `stop --wait`、`disarm`，再执行：

```bash
ros2 run gca_teleop gca_teleop_cli input-config /tmp/teleop-mapped.yaml
```

新输入 epoch、资格复核和释放/回中门控重新建立；参考/授权状态以 `status` 为准，必要时显式重建后 ARM。配置变化不会恢复旧意图。用于 qualified_state 时换成资格向导实际成功生成的文件，不能只改 `mode` 字符串。

## 未知中止后的同 owner 恢复

LT/B 会中止有限段；没有可证明的部分计数时 `rotation_reference_valid=false`。即使推进编码器仍有效，手柄双轴入口也保持锁定：

1. 松开 LT、左杆回中，`stop --wait`，检查 `stop_progress`；UNKNOWN 不能当作已停继续执行。
2. 同一个代理执行 `disarm`。有 FAULT 时先处理原因；模拟故障可通过 `simulation-fault NAME false` 清除，再在已确认停止后 `reset`。RESET 不自动恢复参考或 ARM。
3. 确认实际当前位置/角度，显式 `reference --linear-mm ... --rotation-deg ...`。真实机械角未知时不能填写猜测数值来恢复。
4. 显式 `arm`，读取成功和最新状态；再释放/回中并重新按 LT。原目标、旧 seq、旧许可和慢 ARM 回执都不能恢复运动。

`simulation-fault` 仅接受 `arm_pause/control_freeze/telemetry_pause/encoder_freeze`，只用于模拟诊断；真实故障恢复按 [HIL](HIL_ACCEPTANCE.md)。如 owner 退出或源租约失效，先确认停止，再用同一代理显式 release/claim；不要把旧凭证文件带到新会话。

## 旋转速度与状态解释

旋转摇杆给长期平均角速度意图。单有限波形的提交/轮询有空档，相位补偿可使 **段内峰值高于摇杆平均意图**，始终受核心 `max_rotation_deg_s`、计划加速度与整数时序限制；teleop 的映射上限不能直接当成每个边沿的瞬时上限。能力不足时不得越过核心上限追赶，实际平均率应由跨段窗口评价。有限 profile 的段内计划约束不证明跨空档机械连续加速度。

`/teleop/state`、`/system/state` 和 CLI `status` 分别给门控/质量/队列、设备反馈与停止证据。角速度和角度是接受脉冲时序/确认计数的 **软件估计**；`can_measure_rotation=false`，失步可能不可观测。详见 [测试和判据](SIMULATION_AND_TESTING.md) 与 [JF/JT追踪](TELEOP_TRACEABILITY.md)。
