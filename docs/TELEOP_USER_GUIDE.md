# 手柄控制启用与测试指南

本页按“检查实体输入 → 启动 ROS 2 模拟执行器 → 显式启用 → 短程操作 → 停止退出”的顺序说明使用方法。左摇杆控制推进与旋转，LT 为运动许可条件，B 请求软件 STOP。当前实体手柄使用 `event_age` 模式，**左侧有效输入静止约 0.15 秒即到期停车**，暂不支持未经资格确认的持续静态握持。

当前可运行独立输入监视器和“实体手柄 → 模拟执行器”链路。实体映射已有记录；历史操作只确认短暂模拟推进，旋转确认脉冲为 0，持续双轴控制尚未验收。无线资格和真实执行器 H01–H03 均为 `NOT_RUN`。此前 491 项软件测试通过；新增文本记录功能另经 8 项定向测试与合成输入 ROS 冒烟检查验证，完整集成复验仍暂停，详细结果及历史记录见[实现状态](IMPLEMENTATION_STATUS.md)。

首次使用依次阅读[独立终端检查](#独立终端读取与操作判定无需-ros-2)、[环境准备](#安装环境与只读识别)和[实体手柄启用流程](#实体手柄控制模拟执行器)。不接手柄的软件演示见[合成输入示例](#可重复的纯软件操作)。日常使用无需重新运行完整验收套件；每项短程检查取得明确结果后即可结束，失败时保留原因，不反复重试直至出现一次成功。

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
cd /home/pi/mcrs_ros2_proj
python3 tools/joystick_monitor.py
```

默认只读原始轴/按键编号与数值，不猜测任何手柄映射。自动发现唯一匹配的joydev设备；多个设备时拒绝歧义选择。按键变化和状态更新直接显示在当前终端，按 `Ctrl-C` 退出。初始化基线单独标记，不能算作一次真实按下。

使用已完成实体标定的配置，观察LX/LY、LT、B与门控判定：

```bash
python3 tools/joystick_monitor.py \
  --profile artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml
```

这条配置引用本机已采集的原始标定证据。`artifacts/` 不进入 Git；新克隆的工程不会自带该文件。配置与其引用的映射 JSON、原始采集文件须完整保留，复制一份 YAML 不足以通过证据校验。缺少记录或更换设备时，按后文[映射向导](#完整映射与资格向导)重新采集，不能用 `synthetic.yaml` 解释实体输入，也不能把默认未核实配置的空字段改成猜测值。

换手柄或配置不匹配时，仍可观察原始数据，但不能据此得到有效运动判定。配置读取需要 PyYAML（当前工程环境已具备）；没有 ROS Python 依赖。诊断上下文仅用于本地计算，不领取会话、不 ARM、不发送推进、旋转或磁场命令。

日志分别展示原始输入、映射结果、门控意图与输入年龄。即使静态摇杆仍显示非零，`event_age` 达到 0.15 秒后也会撤销运动意图；原始状态显示不会因此停止。`gate=READY/ACTIVE` 和 `decision=JOG/STOP` 均为**离线输入判定**，不是设备已使能或已停车的反馈。reader 健康只说明本地读取过程，不证明无线链路在线。

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

使用已标定配置时，按以下顺序各检查一次；直接在终端操作和观察，无需在聊天中逐次确认：

| 操作 | 应观察到的输入与判定 |
|---|---|
| 松开 LT、B，左杆回中 | `LX=0`、`LY=0`、`LT` 接近 0、`B=0`；证据与读取均有效时进入 `gate=READY` |
| LT 保持松开，左杆向上、下、左、右拨动后回中 | 上/下对应 `LY` 正/负，左/右对应 `LX` 负/正；`v`、`w` 保持零 |
| 左杆回中，LT 扣下再松开 | `LT` 从接近 0 变到接近 1；跨过 0.6 时 `LT_held=1`，低于 0.4 时恢复 0 |
| 重新释放、回中后，上推左杆并扣下 LT，短暂连续改变上推幅度 | 输入有效期间可见 `decision=JOG`、`v>0`；只右推时应为 `w>0`。这是离线意图，不代表电机运动 |
| 按下 B，再松开 | 按下时 `B=1`、`reason=B_STOP`，速度意图归零；松开后须重新释放 LT、回中并新按 LT |
| 左侧保持中立，分别操作 RT、RB、右摇杆和 RS | 变化只出现在原始值及 `reserved` 中，不产生 GCA 速度意图 |

出现 `INIT_BASELINE` 只说明读取到初始状态；必须看到操作对应的 `CHANGE` 才能确认新事件。若只有原始数组、`gate=RAW_UNMAPPED` 和 `mapped=UNAVAILABLE`，说明当前使用了默认原始读取方式，尚未传入完整映射。

检查到数据与操作一致即可按 `Ctrl-C` 退出；需要保留一段记录时，可限定时长并保存结构化输出：

```bash
python3 tools/joystick_monitor.py \
  --profile artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml \
  --duration 30 --jsonl > /tmp/joystick-monitor.jsonl
```

日志输出与采集分离并有容量限制。终端/重定向阻塞时应查看丢日志计数，不能把有丢弃的显示当成完整原始证据。脚本复用与ROS节点相同的`NativeJoydev`、映射和状态机；源码入口为[joystick_monitor.py](../tools/joystick_monitor.py)，实现为[monitor.py](../ros2_ws/src/gca_teleop/gca_teleop/monitor.py)。

## 安装环境与只读识别

只运行前述独立脚本时可跳过本节。ROS 2 操作要求工程已经构建安装；每个新终端先执行：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
```

首次准备环境时执行 `bash tools/bootstrap_dev.sh`。如果提示安装目录缺失，或者修改过需要用于 ROS 运行的源码，按[开发指南的日常构建](DEVELOPMENT.md#日常构建)完成普通构建后，重新打开终端并执行上面三条命令。独立脚本读取源码，ROS 命令使用安装目录；修改源码后只重新 `source` 不会更新已复制的安装文件。启动手柄不要求先运行 `verify_software.sh`。

需要确认设备身份时，下面查询只打开输入设备读取元数据，不 claim、不 ARM，不启动 GCA 或 pigpiod，也不发送震动：

```bash
ros2 run gca_teleop gca_teleop_probe inspect \
  --profile "$(ros2 pkg prefix --share gca_teleop)/config/beitong_unverified.yaml" \
  --output /tmp/teleop-identity.json
```

已观测接收器为 `20bc:506d`、北通 2.4GHz，当前可读 joydev；元数据不能证明实际按键映射或无线连接活跃。当前完整只读映射证据见 `artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.validation.json`（15份原始文件SHA）；不要把默认profile的 `mapping=null` 改成猜测索引来解除锁定。

## 实体手柄控制模拟执行器

以下使用本机已经标定的北通接收器配置，**执行器后端固定为 `simulation`**。真实电机入口目前不在本流程内。先退出独立监视器和此前的模拟会话，保持 LT、B 松开、左杆回中；每个终端均先完成上节的环境加载。

### 1. 终端 A：启动节点

```bash
TELEOP_PROFILE="$PWD/artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml"
TELEOP_RECORD_DIR="$(mktemp -d /tmp/gca-teleop-physical.XXXXXX)"
test -r "$TELEOP_PROFILE" && \
ros2 launch system_bringup teleop_simulation.launch.py \
  profile_file:="$TELEOP_PROFILE" \
  record_directory:="$TELEOP_RECORD_DIR"
```

配置文件不存在时，此命令不会启动节点。缺少引用的证据、设备不匹配或映射校验失败时，输入门控拒绝运动；根据后文故障表处理。此处不设置 `script_file`，也不使用 `script-start`。默认 launch 使用未核实配置，省略 `profile_file` 不会自动采用本机标定结果。

终端 A 保持运行。节点启动本身不执行 configure、activate、claim、设参考或 ARM。`TELEOP_RECORD_DIR` 是本次手柄原始输入、判定和管理事件的记录目录，运行结束后可在该终端用 `printf '%s\n' "$TELEOP_RECORD_DIR"` 查看路径。

### 2. 终端 B：显式启用

以下通过手柄节点持有的同一控制会话操作；`&&` 确保某一步报错后不继续执行后续步骤：

```bash
ros2 run gca_teleop gca_teleop_cli configure && \
ros2 run gca_teleop gca_teleop_cli activate && \
ros2 run gca_teleop gca_teleop_cli claim && \
ros2 run gca_teleop gca_teleop_cli reference --linear-mm 0 --rotation-deg 0 && \
ros2 run gca_teleop gca_teleop_cli arm && \
ros2 run gca_teleop gca_teleop_cli status
```

依次完成配置、激活、领取 MANUAL 控制权、设置模拟初始参考、ARM 和查询状态。这里的两个零只用于新启动且静止的模拟执行器，不是实机标定命令。不要同时通过 `gca_cli shell` 领取另一份控制权。

ARM 后继续松开 LT、B 并让左杆回中。读取 `status`，确认 `backend="simulation"`、`source="joydev"`、`session_owned=true`、`armed=true`、`linear_valid=true`、`rotation_reference_valid=true`，且 `intent.gate="READY"`。这些状态未满足时先查看 `intent.reason`、`health.reason`、`last_error`，不直接开始操作。`status` 的 `accepted=true` 只表示查询成功。

### 3. 终端 C：观察意图与反馈

```bash
ros2 topic echo /teleop/state
```

重点看 `gate_state`、`gate_reason`、`deadman`、`armed`、`requested_linear_m_s`、`requested_rotation_rad_s`、`applied_linear_m_s`、`estimated_rotation_rad_s` 和 `stop`。ROS 速度字段使用 m/s、rad/s；独立监视器的速度使用 mm/s、deg/s，管理 CLI 的位置和角度使用 mm、deg。

`requested_*` 是输入请求，不能单独证明模拟执行器运动。每项操作前后在终端 B 查询 `status`，对比 `position_mm`、`rotation_estimated_deg`；需要核对旋转确认计数时，在观察终端结束当前 echo，再运行：

```bash
ros2 topic echo /system/state
```

此话题的 `confirmed_pulses`、`rotation_estimated_rad` 是脉冲计数及软件角度估计，`linear_position_m` 是模拟编码器位置。启用 `record_directory` 后，手柄节点也会按 10 Hz 记录最近的系统反馈，字段和偏移基线见下节；不能用角速度意图非零认定旋转已完成。

### 4. 做一次短程功能检查

每次运动检查都从 **LT 松开、左杆回中、`armed=true`、`READY`** 开始。先在 LT 松开时轻推左杆，再扣下 LT，并短暂连续改变左杆幅度；不要先扣住 LT 等待几秒才推杆。静态保持超过 0.15 秒发生 `GCA_INPUT_EXPIRED` 属于当前设计行为；它不是持续握持测试通过。

| 检查 | 操作 | 判定依据 |
|---|---|---|
| 未按 LT 的运动抑制 | LT 松开，拨动左杆后回中 | 不产生非零运动请求，模拟位置不变化 |
| 推进与后退 | 分别上推、下推并配合新 LT 按下 | 线速度请求分别为正、负；同时记录模拟位置是否发生对应变化 |
| 正反旋转 | 分别右推、左推并配合新 LT 按下 | 角速度请求分别为正、负；以确认脉冲或估计角度变化判断模拟运动是否发生 |
| LT 释放 | 有效运动期间先松开 LT，再回中 | 运动意图撤销并请求 STOP；按下节命令确认停止 |
| B 软件 STOP | 有效运动期间按 B，再松开 LT、B 并回中 | 出现 `B_STOP` 并请求停止；释放 B 不恢复原运动 |

LT 释放、B 和输入超时都可能使设备退出 ARM；中止旋转段还可能使旋转参考失效。进行下一项前先确认停止并查询状态：需要时显式 `arm`，参考失效时按后文[恢复流程](#未知中止后的同-owner-恢复)处理。不要连续执行上述动作而忽略中间的门控状态。

每项保留一次明确结果即可。请求非零但位置或脉冲没有变化时，应记录为“已收到意图，运动未确认”，结束该项；不要把反复试出的偶发运动写成通过。若已在输入检查阶段发现数据缺失，先停止本轮，不继续驱动模拟执行器。无线失联资格与实机停止距离不在这组日常检查中。

### 5. 停止并退出

松开 LT、B，左杆回中，在终端 B 执行：

```bash
ros2 run gca_teleop gca_teleop_cli stop --wait && \
ros2 run gca_teleop gca_teleop_cli disarm && \
ros2 run gca_teleop gca_teleop_cli release
```

只有当前停止请求的 `stop_confirmed=true` 且 `stop_progress="CONFIRMED"` 才算确认。超时或 `UNKNOWN` 时保留错误，按恢复流程处理，不能写成已停。确认后在终端 C 和 A 分别按 `Ctrl-C`。每次启动生成独立记录目录，不覆盖上一轮。

### 常见现象与处理

| 现象或原因 | 含义与处理 |
|---|---|
| `Package 'gca_teleop' not found` 或找不到可执行入口 | 确认已构建当前源码，并在当前终端执行 `source tools/common.sh`、`gca_source_install` |
| 本机代理 socket 不存在或连接失败 | 先确认终端 A 的手柄节点仍在运行；自定义 socket 时，CLI 的 `--socket` 与 launch 的 `socket_path` 必须一致 |
| `MAPPING_UNVERIFIED` 或映射证据错误 | 检查是否用了未核实配置，以及映射 JSON 和原始文件是否齐全、身份是否匹配；按映射向导修复证据，不手填猜测索引 |
| 只有 `INIT_BASELINE` / `CACHE`，没有动作对应的 `CHANGE` | 当前记录不能证明新输入已到达；返回独立监视器检查连接和事件，不继续运动测试 |
| `NOT_ARMED_READY` | 检查控制会话、激活、参考、ARM、故障与停止确认状态 |
| `RELEASE_AND_CENTER` / `NEW_LT_EDGE_REQUIRED` | 在当前上下文中释放 LT、左杆回中，确认 `READY` 后重新按下 LT |
| `GCA_INPUT_EXPIRED` | 左侧有效事件超过 0.15 秒；先停止、恢复门控。不能用右侧按键刷新期限，也不能直接改成未经资格确认的 `qualified_state` |
| `READER_STALE` 或输入断开 | 本地读取失效；先停止并保留日志，恢复输入后重新建立门控 |
| 有旋转请求但 `confirmed_pulses` 不变 | 只确认意图到达，尚未确认旋转；记录当前门控、停止原因和参考状态后结束该项 |

## 将输入动作与模拟偏移写入文本文件

本功能同时记录两种偏移：**摇杆相对标定中心的偏移**，以及**模拟执行器相对本次有效反馈基线的推进位移与旋转角度变化**。沿用上面的实体手柄启动、显式启用和停止步骤，不需要第二个手柄读取进程。

### 启动记录

终端 A 使用以下命令代替上面的启动命令，将记录保存在工程的 `artifacts/joystick_sessions/` 下。每次生成新目录：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
mkdir -p artifacts/joystick_sessions
TELEOP_RECORD_DIR="$(mktemp -d "$PWD/artifacts/joystick_sessions/run.XXXXXX")"
printf '本次记录目录：%s\n' "$TELEOP_RECORD_DIR"
ros2 launch system_bringup teleop_simulation.launch.py \
  profile_file:="$PWD/artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml" \
  record_directory:="$TELEOP_RECORD_DIR"
```

终端 B 按[显式启用步骤](#2-终端-b显式启用)依次执行 configure、activate、claim、reference、arm，确认 `READY` 后操作手柄。完成后按[停止并退出](#5-停止并退出)结束。仅启动记录不会自动 ARM。

文件名为 `control-00.jsonl` 至 `control-03.jsonl`，内容是 UTF-8 文本，每行一个 JSON 对象，可直接在编辑器打开。输入事件和反馈采样分行记录，使用同机 `monotonic_ns` 关联；它们不是同一时刻的同步测量。

### 字段含义

| 记录种类／字段 | 含义 |
|---|---|
| `kind="input"` / `frame` | 原始轴与按键数组、变化编号、事件时间、输入代次；`event_seq=0` 的初始化不能视为新操作 |
| `mapped.sticks.lx/ly.offset_raw` | 原始轴值减标定中心值，未改变驱动轴方向 |
| `mapped.sticks.lx/ly.normalized_before_deadzone` | 按标定范围及方向归一化到 −1～1，尚未应用死区 |
| `mapped.sticks.lx/ly.mapped_after_curve` | 应用死区和响应曲线后的映射量；LY 正为推进，LX 正为正向旋转 |
| `mapped.controls` | 映射后的 LX、LY、LT、B 与左杆是否回中 |
| `mapped.actions`、`*_before_gate` | 按杆量得到的动作名称及速度，**尚未通过 LT、ARM、期限等门控** |
| `intent`、`requested_actions` | 门控后的 JOG／STOP／NONE、原因、推进 mm/s 与旋转 deg/s，以及对应的动作名称；不是执行器回执 |
| `kind="motion_sample"` / `feedback` | 10 Hz 观察到的最近系统状态；保留原始 `state_seq`、ROS 时间戳和 `feedback_age_s`，重复序号不是新反馈 |
| `feedback.linear_position_mm` | 模拟编码器位置；同时查看 `linear_valid` |
| `feedback.rotation_estimated_deg` | 由脉冲估计的角度；同时查看 `rotation_valid`，`rotation_measured=false` 表示没有实测旋转角 |
| `feedback.linear_displacement_mm`、`rotation_displacement_deg` | 相对各轴日志基线的位移和角度变化，不是对摇杆速度做积分 |
| `feedback.linear_baseline`、`rotation_baseline` | `[基线状态序号, 基线位置或角度]`；首次有效反馈建立基线，设备重启、设备代次或参考改变后重建。某轴参考或反馈失效时，其偏移为 `null`，恢复后重新建立该轴基线 |
| `feedback.applied_linear_mm_s`、`linear_velocity_mm_s` | 模拟舵机接受的速度输出与模拟编码器速度；编码器速度另有 `linear_velocity_valid` 标志 |
| `feedback.confirmed_pulses`、`reserved_pulses` | 已确认旋转脉冲与尚未确认的预约脉冲；不能把预约数当已执行数 |
| `feedback.stop_progress`、`stop_reason`、`fault_code` | 停止进度、原因及故障码 |
| `recorder.dropped`、`errors` | 截至该次采样的记录丢弃和写入错误数量；非零时不能认为日志完整 |

例如 LT 未按下时，上推左杆可以出现 `mapped.actions.linear="推进"`，但 `intent.linear_mm_s=0`、`requested_actions.linear="零速"`；这表示识别了操作，但运动门控没有放行。若请求非零而反馈位移仍为零，只能认定动作请求已产生。

`mapped.available=false` 或 `feedback.available=false` 时读取对应 `reason`，不能把缺失值补成零。反馈有效标志由系统提供；记录时间不是传感器采样时间，判断过期状态须结合原时间戳、序号及反馈年龄。10 Hz 采样可能漏掉更短的运动过程，不能用于测量最坏停止延迟。

### 转为便于阅读的 `.txt` 文件

先停止节点，随后在终端 A 执行。该命令读取本次目录内的 JSONL，按记录时间排序并生成缩进文本，不覆盖同名文件：

```bash
python3 - "$TELEOP_RECORD_DIR" <<'PY'
import json
import sys
from pathlib import Path

directory = Path(sys.argv[1])
files = sorted(directory.glob("control-*.jsonl"))
if not files:
    raise SystemExit("没有找到本次记录文件")
records = [json.loads(line) for path in files for line in path.read_text().splitlines()]
records.sort(key=lambda record: record["monotonic_ns"])
output = directory / "joystick.txt"
with output.open("x", encoding="utf-8") as stream:
    for record in records:
        stream.write(json.dumps(record, ensure_ascii=False, indent=2) + "\n\n")
print(output)
PY
```

记录器沿用容量限制：最多四个文件、每个约 2 MB，循环覆盖旧文件；队列容量为 1024。日常只做短程检查，结束后及时保存结果。循环覆盖不计入 `dropped`，转换后的文本只包含仍保留的记录，不能据此声称保存了无限时长的完整操作过程。

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
