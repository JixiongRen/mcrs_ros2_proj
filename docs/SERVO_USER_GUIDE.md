# 手柄控制舵机：启动、正反转与停止

本指南对应仅舵机入口 `servo_simulation.launch.py` 和 `servo_hardware.launch.py`。左杆上推使推进舵机反转，下推使其正转；杆量控制舵机轴转速。外部位移编码器与步进旋转轴不参与这一模式，因此不要求毫米参考，也不提供真实推进距离或导管旋转角度。

当前使用 `manual_input_mode="joydev_hold"`：持续按住 LT 即可操作，不设单次时长上限；松开 LT 或按 B 请求 STOP。指令上限为 **80 rpm**（寄存器量化后为 79.921 rpm），软件加速度为 **40 rpm/s**。最近空载实测约为 ±45.8 rpm，指令上限不等于电机实际能够达到的转速。

日常使用直接按下面的[真实舵机快速操作](#真实舵机快速操作)执行。需要查看输入而不驱动真实舵机时，使用[模拟舵机流程](#实体手柄控制模拟舵机)；调参见[配置入口](#配置入口与环境)。

本轮空载手柄联调已完成，[交付报告](../artifacts/servo_ros2/delivery-v5/report.json)及历史中断记录见[实现状态](IMPLEMENTATION_STATUS.md)。无线资格仍为 `NOT_RUN`；“停止命令更新但保持遥测”和“总线静默验证 watchdog”两项专项测试按用户决定移至后续。这些结果不包含真实位移闭环验收。

## 真实舵机快速操作

以下命令适用于本机已连接并供电的 XM430-W350（ID 1、57600 baud），以及已标定的北通阿修罗 3S 接收器。手柄当前应为 BFM 模式；模式切换会改变设备身份和映射。先结束占用同一串口或手柄管理入口的旧程序，保持 LT、B 松开，左摇杆回中。工程须已完成[日常构建](DEVELOPMENT.md#日常构建)。

### 1. 终端 A：启动真实舵机并保存记录

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install

python3 config/servo_settings.py export artifacts/servo_config/hardware.json \
  --backend hardware --manual-input-mode joydev_hold
SERVO_CONFIG="$PWD/artifacts/servo_config/hardware.json"
SERVO_PROFILE="$PWD/artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2/mapping.mode-confirmed.profile.yaml"
mkdir -p artifacts/servo_ros2
SERVO_RUN_DIR="$(mktemp -d "$PWD/artifacts/servo_ros2/manual-hardware.XXXXXX")" && \
printf '本次记录目录：%s\n' "$SERVO_RUN_DIR" && \
test -r "$SERVO_CONFIG" && test -r "$SERVO_PROFILE" && \
ros2 launch system_bringup servo_hardware.launch.py \
  servo_config_file:="$SERVO_CONFIG" \
  profile_file:="$SERVO_PROFILE" \
  trusted_local_control:=true \
  record_directory:="$SERVO_RUN_DIR"
```

终端 A 保持运行。`trusted_local_control:=true` 使用本机信任设置，标准 launch 限制 ROS 发现到本机；同一份配置会传给舵机节点、协调器和手柄节点。`mktemp` 模板末尾的 `XXXXXX` 必须保留，否则无法生成记录目录。

上面的实体 profile 及其引用的原始记录保存在本机 `artifacts/`，不随 Git 克隆。缺失时按[映射向导](TELEOP_USER_GUIDE.md#完整映射与资格向导)取得真实记录，并把 `SERVO_PROFILE` 改为对应路径；不能用合成输入配置代替实体标定。

### 2. 终端 B：领取控制权并使能

另开终端，保持 LT、B 松开、左杆回中，执行：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install

ros2 run gca_teleop gca_teleop_cli configure && \
ros2 run gca_teleop gca_teleop_cli activate && \
ros2 run gca_teleop gca_teleop_cli claim && \
ros2 run gca_teleop gca_teleop_cli arm && \
ros2 run gca_teleop gca_teleop_cli status
```

这组命令用于新启动且尚未领取会话的程序，**不需要 `reference`**。`configure` 会核对设备并配置速度模式、限制和 watchdog；`arm` 允许后续手柄操作，启动节点本身不会使舵机转动。

开始操作前，确认状态中 `backend="hardware"`、`control_profile="servo"`、`session_owned=true`、`armed=true`、`servo.servo_valid=true` 和 `intent.gate="READY"`，初始 `servo.measured_servo_rpm` 应为零。未接外部编码器时，`linear_valid=false`、`position_mm=null` 属于正常状态。

若报 `SESSION_ALREADY_OWNED`，先执行 `status`；这表示当前手柄节点已经领取控制权，不要重复 `claim`。已有会话且已 ARM、门控 READY 时直接操作；已有会话但未 ARM 时，释放 LT、B 并回中后单独执行 `arm`。

### 3. 操作正反转和调速

| 手柄操作 | 舵机行为 |
|---|---|
| 按住 LT，左杆向上推 | 反向旋转，速度请求为负值 |
| 按住 LT，左杆向下推 | 正向旋转，速度请求为正值 |
| 增大上推或下推幅度 | 增大转速幅值；按死区和曲线映射，不保证半程就是半速 |
| 保持 LT 和杆量不变 | 持续按当前方向、速度请求运行，无单次时间上限 |
| 按住 LT，让左杆回中 | 目标速度归零，按配置加速度减速 |
| 松开 LT | 请求 STOP，撤销运动并关闭力矩，等待实际停车确认 |
| 按下 B | 请求 STOP；松开 B 不自动恢复此前运动 |
| 左杆左右、右侧扳机和摇杆等输入 | 当前仅舵机模式不分配运动功能，右侧输入继续预留给磁场控制 |

正反方向以舵机速度寄存器的正负号为准，不依据观察轴端的位置改称顺时针或逆时针。外部位移编码器不参与上述手动调速。

首次操作保持空载，完成下面一组即可，不必重复完整测试套件：

1. 按住 LT，上推半程观察反转，再增加幅度观察加速。
2. 松开 LT、左杆回中，确认停车；重新按住 LT 并下推，观察正转。
3. 运动时按 B，确认停车；松开 B 和 LT、左杆回中，再重新按 LT 操作，确认可以恢复。
4. 最后松开全部输入，按下一节退出。

快速再次操作若显示 `WAIT_STOP`，表示新一次 LT 已识别，正在等待上一轮实际停车；保持本次 LT 和所需杆量，确认后会按当前杆量启动，无需反复重按。实测约 45 rpm 时停车确认曾需约 1.3 秒，配置期限为 2 秒。释放 LT 或按 B 会立即发出停止请求，但机械停车需要时间。

**B 停车后必须松开 B 和 LT、让左杆回中，再按一次 LT。**只松开 B 而保持此前的 LT，不会重新启动。若出现 `FAULT` 或停止超时，保存本次记录并查看错误，不按普通 `WAIT_STOP` 继续等待运动恢复。

上述标准 launch 没有定时退出。此前实机试验结束时的自动停车来自限时采集脚本；日常启动会持续运行，直到主动停止或发生故障。

### 4. 停止和退出

先松开 LT、B，让左杆回中；在终端 B 执行：

```bash
ros2 run gca_teleop gca_teleop_cli stop --wait && \
ros2 run gca_teleop gca_teleop_cli disarm && \
ros2 run gca_teleop gca_teleop_cli release && \
ros2 run gca_teleop gca_teleop_cli status
```

确认 `stop_confirmed=true`、`stop_progress="CONFIRMED"`、`armed=false`、`session_owned=false`、实测零速且力矩关闭，再在终端 A 按 `Ctrl-C`。若返回超时或 `UNKNOWN`，不能把请求成功当作已经停车。

记录保存在终端 A 打印的目录中：`device/` 为舵机状态，`teleop/` 为输入、动作判定和管理记录，都是可直接打开的 JSONL 文本。查看字段含义见[记录的单位与有效性](#记录的单位与有效性)。

## 配置入口与环境

舵机数值统一在 [`config/servo_settings.py`](../config/servo_settings.py) 修改。它只依赖标准库，不需要 ROS，显示和导出不访问串口：

```bash
cd /home/pi/mcrs_ros2_proj
python3 config/servo_settings.py show --manual-input-mode joydev_hold
python3 config/servo_settings.py export artifacts/servo_config/simulation.json \
  --backend simulation --manual-input-mode joydev_hold
python3 config/servo_settings.py export artifacts/servo_config/hardware.json \
  --backend hardware --manual-input-mode joydev_hold
```

以上命令使用默认 `--section runtime`，JSON 是带版本与 SHA-256 摘要的运行快照，不另行手改。硬件和模拟快照的 `backend` 不同，不能互换。舵机节点、协调器和手柄节点必须加载**同一路径、同一内容**；标准launch将 `servo_config_file` 同时传给这三个节点。能力响应的配置摘要、后端或速度上限不匹配时，手柄拒绝 ARM。

后续上位机位移闭环所需的驱动轮半径、传动比、PID 及运动限制，在同一脚本的 `position_settings_payload()` 中集中定义，可用 `python3 config/servo_settings.py show --section position` 查看。外部编码器参数另由 `python3 config/encoder_settings.py show` 查看。未知值保留 `None` 和 TODO；完整字段、单位、导出命令与标定关系见[上位机位移闭环参数草稿](CONFIGURATION_AND_CALIBRATION.md#上位机位移闭环参数草稿)。这两份草稿不能作为当前 `servo_config_file`，填写或导出不会启用 PID，也不会改变手柄控制。

刹车减速度与延迟为 `None` 时，参数解析分别采用 `braking_defaults` 中的 `5 mm/s²`、`0.5 s`；显示和导出的 `effective_braking` 会标明使用值及来源。填写实测值后自动优先使用该项实测值，原始 TODO 不会被缺省值覆盖。具体规则见[刹车实测值未知时的缺省策略](CONFIGURATION_AND_CALIBRATION.md#刹车实测值未知时的缺省策略)，当前手柄运行不受这两项 TODO 影响。

`joydev_hold`的reader观察期限由舵机快照中的`manual_input_max_age_s`决定，当前为0.30秒；原profile中的`event_age`期限仍用于原事件模式，不用于连续观察。`qualified_state`仍服从profile与舵机配置中更短的期限。每条观察保留原始截止时间，不在转发时续期，也不限制持续按住LT的总时间。`joydev_hold`调度按`control_period_s`运行，并服从profile更低的发送频率上限；默认40毫秒一次。

当前脚本中的指令上限raw349对应79.921 rpm，不超过用户规定的80 rpm；变化率40 rpm/s、固件加速度raw12、事务期限 0.08 s、反馈年龄 0.25 s、输出期限 0.30 s、停止确认期限 2 s、watchdog raw 25 等是当前空载联调参数，不代表所有失效场景的安全边界已经测定。`manual_axis_to_servo_sign=-1` 表示映射后 LY 上正下负到舵机方向的唯一转换；当前配置校验不接受改为正号。位移草稿已按用户确认填写 6 mm 半径、直驱、上推为推进正方向及 PID 初值，每转推进量按无滑动几何模型计算约为 37.6991 mm；这不是实测标定，也尚未接入闭环。当前运行快照中的 `mechanical_mapping`、`position_pid` 必须保持 `null`。

加速度raw12约为42.92 rpm/s，依据[ROBOTIS Profile Acceleration定义](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#profile-acceleration108)。软件目标从零达到79.921 rpm约需2秒；这是限速器计算值，不是实测响应时间。厂家标称XM430-W350的空载转速为12 V时46 rpm、14.8 V时57 rpm；指令上限80 rpm不能提高电机本身的速度能力。一次速度写入与反馈读取分别使用80毫秒事务预算，I/O周期20毫秒按整轮计算，事务已超过周期时不追加等待。配置校验要求完整读写周期与控制周期能容纳于输出及输入期限。

300毫秒是停止更新后的指令新鲜度上限，不是保证机械停车完成的时间；LT释放和B按下仍直接请求STOP，不等待此期限。已有v2快照的150毫秒预算不满足新校验，须重新导出当前v5快照并重启，不能直接复用旧JSON。

ROS 操作前需按[日常构建](DEVELOPMENT.md#日常构建)构建当前代码及消息；每个终端分别加载安装环境：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
```

修改 Python/IDL 源码需要重新构建。只修改配置脚本中的数值不需要编译 ROS，但必须重新导出、停止并重新配置，具体流程见文末。

## 独立只读检查：不使用 ROS

此工具读取当前寄存器和轴反馈，不改变模式、不写零速、不停止设备，也不使能力矩。先退出占用同一串口的 ROS 节点或独立程序；设备锁和串口独占冲突时应结束已有使用者，不能删除锁或绕过独占。

```bash
cd /home/pi/mcrs_ros2_proj
mkdir -p artifacts/servo_ros2
SERVO_PROBE_DIR="$(mktemp -d "$PWD/artifacts/servo_ros2/probe.XXXXXX")"
.venv/bin/python tools/servo_probe.py \
  --config-file "$PWD/artifacts/servo_config/hardware.json" \
  --output-dir "$SERVO_PROBE_DIR/readonly" --samples 3
```

输出为 `$SERVO_PROBE_DIR/readonly/report.json`。`--output-dir` 必须尚不存在，工具不覆盖历史报告。检查 `status`、`write_instructions_sent`、设备身份、`registers`、`samples` 与 `readonly_disconnected`。读取成功只证明本次通信和反馈可用，不表示参数、负载或真实运动已经验收。

## 实体手柄控制模拟舵机

这条流程连接实体手柄，舵机后端固定为 `simulation`。先关闭旧双轴 launch、其它舵机 launch 和占用默认管理 socket 的手柄节点；同一时刻只运行一个 `/gca/*` 执行端。

### 终端 A：启动并记录

```bash
SERVO_CONFIG="$PWD/artifacts/servo_config/simulation.json"
SERVO_PROFILE="$PWD/artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2/mapping.mode-confirmed.profile.yaml"
mkdir -p artifacts/servo_ros2
SERVO_RUN_DIR="$(mktemp -d "$PWD/artifacts/servo_ros2/manual-sim.XXXXXX")"
test -r "$SERVO_CONFIG" && test -r "$SERVO_PROFILE" && \
ros2 launch system_bringup servo_simulation.launch.py \
  servo_config_file:="$SERVO_CONFIG" \
  profile_file:="$SERVO_PROFILE" \
  record_directory:="$SERVO_RUN_DIR"
```

本机实体 profile 及引用的映射原始证据不进入 Git，新克隆可能没有这些文件。缺失时按[映射向导](TELEOP_USER_GUIDE.md#完整映射与资格向导)取得真实记录；不能用 `synthetic.yaml` 解释实体手柄。省略 `profile_file` 使用的是未核实模板，不会自动启用本机映射。

### 终端 B：通过同一 owner 启用模拟舵机

新启动且尚未领取会话时执行：

```bash
ros2 run gca_teleop gca_teleop_cli configure && \
ros2 run gca_teleop gca_teleop_cli activate && \
ros2 run gca_teleop gca_teleop_cli claim && \
ros2 run gca_teleop gca_teleop_cli arm && \
ros2 run gca_teleop gca_teleop_cli status
```

这里**不执行 `reference`**。若已经持有会话，先查询 `status`，不要再次 `claim`；`SESSION_ALREADY_OWNED` 表示当前 owner 已领取，并非必须重启。不要另开 `gca_cli shell` 争用控制权。

完整初始化读数确认LY回中、LT和B松开后，即可显式ARM，无需先额外按B或拨杆制造事件。该中立状态只能用于零目标使能；运动仍须在READY后新按LT。普通启动不自动ARM。

保持 LT、B 松开，LY 回中，确认 `backend="simulation"`、`control_profile="servo"`、`control_policy="MANUAL_SERVO_VELOCITY"`、`session_owned=true`、`armed=true`、`servo.servo_valid=true` 和 `intent.gate="READY"`。`linear_valid=false`、`rotation_reference_valid=false`、`position_mm=null` 是本模式的正常状态，不应设置假参考来消除它们。

### 操作与反馈

| 操作 | 应出现的舵机请求 |
|---|---|
| LT 松开时拨动 LY | 不允许非零运动 |
| 新按 LT 后上推 LY | 负 RPM；幅度增加时速度幅值非递减 |
| 新按 LT 后下推 LY | 正 RPM；幅度增加时速度幅值非递减 |
| LY 回中 | 零速目标，按配置变化率减速，不追赶历史位移 |
| 松开 LT 或按 B | 独立请求 STOP；松开 B 不恢复旧运动 |
| 操作 LX 或右侧预留输入 | 不改变舵机速度请求；LX 不阻止 LY 回中 |

当前入口由舵机快照中的 `manual_input_mode="joydev_hold"` 显式启用持续操作。实体profile仍负责身份与映射，不能仅凭其历史 `event_age` 标签判断该舵机模式的保持行为。只要LT持续按住、reader持续产生有效观察，静态杆量可以持续控制舵机；没有单次episode截止。0.30 s限制作用于reader观察和输出的新鲜度，控制权、命令及输出期限仍独立生效。每次停止或输入失效后，释放LT、LY回中，再按下LT；若状态已DISARMED，则先显式ARM。

快速连续操作时，可以在上一轮STOP尚未确认期间松开LT、让左杆回中，再按下LT并推动左杆。`WAIT_STOP`表示已经识别了这一轮新操作，仍在等待上一轮停车确认；确认后按**当时的摇杆位置**运行，无需再松开重按。等待期间松开LT、按B、输入失效或控制权变化会取消本次操作资格。程序不会保存并追赶等待前的速度指令。

B停车后，先松开B和LT、让左杆回中，再重新按LT操作。只松开B、继续保持旧LT按下状态不会恢复。舵机必须完成新鲜零速、力矩关闭及驻留确认才能重新运动；本机10 rpm试验的停车确认约0.65–0.74秒；约45 rpm试验的记录到约1.28秒才具备确认条件。v5的2秒预算用于等待真实停止，不是额外冷却时间，也不会使机械滑行更快结束。

设备运行时继续执行250毫秒反馈有效期。手柄节点使用设备上报的有效性，并要求上层状态序号持续前进；超过500毫秒不前进则禁止输出，同序号重复消息不能延长这段期限。界面仍显示实际样本年龄，不把遥测缓存延迟当作新的硬件采样。

观察现有两个话题即可，不需要再启动手柄读取进程：

```bash
ros2 topic echo /teleop/state
```

```bash
ros2 topic echo /system/state
```

前者查看 `requested_servo_rad_s`、`measured_servo_rad_s`、`servo_valid`、门控与停止原因；后者查看 `servo.requested_velocity_rad_s`、`accepted_velocity_rad_s`、`measured_velocity_rad_s`、`raw_position_count`、`torque_enabled`、`sample_seq`、`sample_age_s`。请求、实际接受值和测量值含义不同；请求非零不能独立证明运动。

### 停止和退出

先松开 LT、B，LY 回中，在终端 B 执行：

```bash
ros2 run gca_teleop gca_teleop_cli stop --wait && \
ros2 run gca_teleop gca_teleop_cli disarm && \
ros2 run gca_teleop gca_teleop_cli release && \
ros2 run gca_teleop gca_teleop_cli status
```

确认 `stop_confirmed=true`、`stop_progress="CONFIRMED"`、`armed=false`、`session_owned=false`，再在观察和 launch 终端按 `Ctrl-C`。超时或 `UNKNOWN` 应保留原因，不能标成已停止。软件退出使用有界清理；不要在正式停止之前终止所有 ROS 进程，令管理回执无处返回。

## 一次可重复的正式 ROS 模拟检查

此工具保留合成输入路径的检查入口，使用安装后的launch、协调器、手柄owner和模拟舵机。它自动执行10 s脚本、停止与释放，不访问真实串口。脚本使用 `SCRIPT_SAMPLE`，应单独导出 `qualified_state` 的simulation快照；该脚本不验证 `joydev_hold` 的实体持续输入：

```bash
mkdir -p artifacts/servo_ros2
SERVO_CHECK_DIR="$(mktemp -d "$PWD/artifacts/servo_ros2/software-check.XXXXXX")"
python3 config/servo_settings.py export "$SERVO_CHECK_DIR/simulation.json" \
  --backend simulation --manual-input-mode qualified_state
"$GCA_PYTHON" tools/servo_simulation_check.py \
  --config-file "$SERVO_CHECK_DIR/simulation.json" \
  --output "$SERVO_CHECK_DIR/result"
```

工具默认使用 ROS domain 213 和独立管理 socket；输出目录必须尚不存在。检查 `result/report.json` 的 `passed`、各项 `checks`、`errors`、`launch_exit_code`，并保留 `launch.log`、`states.jsonl` 与 `records/`。该报告的 `hardware_tested=false` 明确表示纯软件检查。通过后停止追加测试；失败只核对相应错误，不反复执行整个历史套件。

## 手柄模式与映射记录

本轮空载持续操作使用显式 `joydev_hold` 快照；无线资格未完成不阻止该模式的ARM，LT/B及停止后重新门控仍生效。2026-11-07 的[只读模式确认](../artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2/assessment.json)记录了 BFM（`20bc:506d / hid-generic`）→ PC360（`045e:028e / xpad`）→ BFM，当前保持 BFM。

已生成[关联模式确认的 profile](../artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2/mapping.mode-confirmed.profile.yaml)。其 schema 3 映射记录保存原 schema 2 文件原文、模式观察和操作者回复及摘要；先按旧身份完整重放映射，再验证同端口往返切换，以及除模式标签和设备路径外的身份一致性。原标定及其未知模式标签未修改。该映射profile仍为 `event_age`，`qualification_file` 为空；更改前的[安装后检查](../artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2/installed-binding-check.json)确认可以加载映射，当时按资格模式进行的真实ARM返回 `HARDWARE_INPUT_NOT_QUALIFIED`。本次持续输入由显式舵机快照选择，不把映射记录改写为已通过无线资格。

下面的独立命令可依据已保存的记录生成另一份新文件；只读取手柄身份，不运行 ROS 节点或执行器。输出不得覆盖原文件。目前仅支持已验证的阿修罗 3S BFM→PC360→BFM 关联，不将任意模式名称推定为有效。

```bash
source tools/common.sh
gca_source_install
MODE_OBSERVATION="$PWD/artifacts/servo_ros2/implementation-xce7i_e4/input-mode-confirm-ugna0rh2"
MODE_OUTPUT="$(mktemp -d "$PWD/artifacts/servo_ros2/mode-binding.XXXXXX")"
.venv/bin/python -m gca_teleop.mode_confirmation \
  --profile "$PWD/artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml" \
  --observation-directory "$MODE_OBSERVATION" \
  --output "$MODE_OUTPUT/mapping.json" \
  --profile-output "$MODE_OUTPUT/profile.yaml"
```

配置导出省略 `--manual-input-mode` 时，默认仍为 `qualified_state`；该默认路径仍须依据[资格向导](TELEOP_USER_GUIDE.md#完整映射与资格向导)完成预声明的手柄断电、无线失联、接收器拔出检查。向导任一试验失败或采集异常即保存报告并退出，不继续下一项，也不生成资格 profile。`qualified_state` 必须由匹配真实证据的流程产生；修改 mode 字符串、使用脚本输入或将手柄改称 `PROGRAMMED_JOG` 都不能取得资格。

操作者确认本手柄通过长按 Back＋B 关机。长按过程、B 本身触发的 STOP 和无线失联响应是不同事件；不能按假定的长按时长移动计时起点。聊天中的“已关机”只能提供关机完成时间的上界：该回复之后仍残留非零输入超过 D，可以证明不满足 D；此前已清零或断开则不能仅凭回复时间证明 D 内响应。完整时限资格仍需记录可信起点及其误差。

真实启动命令见[真实舵机快速操作](#真实舵机快速操作)。当前速度模式已完成空载手柄正反转联调，历史位置模式 720° 试转不计入本轮结果。配置成功后的正常退出保留速度模式 1、关闭力矩，不自动切回模式 4；watchdog 锁存或硬件故障需要显式恢复，普通 configure 不自动清锁。

## 记录的单位与有效性

`record_directory` 下分为 `device/` 与 `teleop/`。JSONL 文件可直接用文本编辑器读取；记录队列有界，检查丢弃与错误计数后再判断是否完整。

| 内容 | 关键字段或单位 |
|---|---|
| 原始与映射输入 | LY 原始计数、中心偏移、归一化值；`servo_rpm_before_gate` 是门控前请求 |
| 门控意图 | `intent.servo_rpm`、控制策略、输入年龄、LT/B 与停止原因 |
| 舵机反馈 | `requested_servo_rpm`、`accepted_servo_rpm`、`measured_servo_rpm`，单位 rpm |
| 轴位置与质量 | `raw_position_count`、`servo_shaft_angle_rad`、`sample_seq`、`device_epoch`、`feedback_age_s`、`servo_valid` |
| 输出与故障 | `torque_enabled`、`hardware_error`、`watchdog_raw`、`error_code`、停止进度 |

舵机轴角度按原始位置计数 `count × 2π / 4096` 表示，属于舵机寄存器坐标；模式变化、重新连接或设备代次改变后不能直接相减当作连续位移。RPM 与 rad/s 的关系为 `rad/s = rpm × 2π / 60`。本模式不计算毫米推进量，也不把舵机轴角写入导管旋转估计；日志中的这些外部量为 `null`，ROS 对应有效标志为 false。

## 修改配置与常见拒绝

调参顺序是：确认 STOP → DISARM → release → 退出 launch → 修改 `config/servo_settings.py` 和配置修订号 → 重新导出快照 → 重新启动并 configure、activate、claim，满足模式条件后显式 ARM。数值调整不需要 ROS 编译；代码或 IDL 修改仍需要构建。

设备层也要求 cleanup 后重新 configure；若只对设备做 lifecycle cleanup，手柄节点仍可能持有旧快照，所以日常操作采用整组重启，保证三个节点摘要一致。`servo_config_file` 参数不能在线更换，`/system/apply_config` 不接收舵机 JSON 热更新。

| 拒绝或状态 | 处理依据 |
|---|---|
| `SERVO_CAPABILITY_CONFIG_MISMATCH` | 核对三个节点的实际快照、摘要、后端和速度上限；重导出并整组重启 |
| `HARDWARE_INPUT_NOT_QUALIFIED`／硬件输入质量拒绝 | 默认 `qualified_state` 路径要求真实资格；本轮 `joydev_hold` 应核对三个节点是否加载同一显式快照 |
| `EXTERNAL_POSITION_CONTROL_UNAVAILABLE`／`EXTERNAL_ENCODER_REQUIRED` | 纯舵机模式不设置毫米／步进参考 |
| `EXTERNAL_ENCODER_POSITION_UNSUPPORTED`／`POSITION_CONTROL_UNAVAILABLE` | 本轮未实现上位机位移闭环，不能改发伪毫米速度规避 |
| `WATCHDOG_LATCHED` | 保存锁存与停止证据，按明确恢复流程处理；不通过重新 configure 隐式清除 |
| `DEVICE_BUSY`、串口独占失败 | 结束已有合法设备使用者，不并发试转或删除锁 |
| `WAIT_NEUTRAL` | 松开 B 和 LT，让左杆回中；收到有效中立输入后，再新按 LT |
| `WAIT_STOP` | 正在等待上一轮停车确认；保持新一轮 LT 和杆量，确认后按当前输入运行；超过期限报错时保留记录 |
| B 停车后没有恢复 | 松开 B 之外还须释放 LT、左杆回中，再新按 LT；不能沿用停车前的 LT 按下状态 |
| `GCA_INPUT_EXPIRED` | 释放LT、LY回中并检查输入模式：`joydev_hold` 核对reader观察是否推进，原 `event_age` 核对有效变化事件年龄 |
| `SESSION_ALREADY_OWNED` | 查询现有 owner 状态，继续当前会话，不重复 claim |

上位机位置策略的反馈类型、机械换算与 PID 接口已预留；真实外部编码器、标定、位置任务和 PID 硬件整定仍未实施，参见[接入方案](SERVO_ROS2_INTEGRATION_PLAN.md)。
