# 指令下发模式：上位机位移闭环操作指南

位置模式由上位机发送目标，树莓派用外部编码器测量位移并修正舵机转速。手柄模式仍直接控制 RPM，不读取外部编码器。功能包职责及控制细节见[实施方案](POSITION_PID_ROS2_PLAN.md)，验证范围见[实现状态](POSITION_PID_IMPLEMENTATION_STATUS.md)。

本指南中的“上位机”也可以是树莓派本机运行的客户端。当前入口是 ROS2 的 `gca_client position-shell`，不是手柄管理工具 `gca_teleop_cli`。`control_mode:=position` 选择指令下发，`control_mode:=manual` 选择手柄；同一时刻只启动一套装配。

2026-11-09 已完成一次 100 mm、2 mm/s 推进，最终编码器位置 99.479 mm，满足当前 1 mm 容差；随后 8 mm/s 回零出现超调，测试脚本请求停车后停在 −1.854 mm。本阶段先保留这些结果，8 mm/s 回零尚未通过稳定到位验证。下文操作示例采用 2 mm/s，完整记录和当前限制见[实机状态](POSITION_PID_IMPLEMENTATION_STATUS.md)。

## 准备环境与配置

在工程目录执行：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
mkdir -p artifacts/position_config
python3 tools/export_feed_axis_config.py artifacts/position_config/simulation.json \
  --control-mode position --backend simulation
```

导出工具只读取 `config/servo_settings.py` 和 `config/encoder_settings.py`，不连接设备。导出内容是含摘要的运行快照；不要手改 JSON。原来的两个 `*_position_draft` 参数草稿不能直接作为运行快照。

编码器读取周期为 20 ms，查询超时为 50 ms，反馈有效期为 100 ms；位置速度估计采用 `estimation.velocity_window_s=0.2` 秒的实际时间窗口。建立参考后需重新形成该窗口；客户端 `reference` 命令会等待有效速度反馈，不把未就绪的速度当作零。

## 启动位置模式

第一个终端：

```bash
export ROS_DOMAIN_ID=31
POSITION_RECORD_DIR="$(mktemp -d "$PWD/artifacts/position_config/run.XXXXXX")"
ros2 launch system_bringup feed_axis.launch.py \
  control_mode:=position backend:=simulation \
  servo_config_file:="$PWD/artifacts/position_config/simulation.json" \
  record_directory:="$POSITION_RECORD_DIR"
```

`control_mode`、`backend` 必须显式填写并与快照一致。启动不自动领取控制权、建立参考或使能。位置模式不启动手柄节点。

第二个终端先进入同一工程、执行 `source tools/common.sh` 和 `gca_source_install`，再执行：

```bash
export ROS_DOMAIN_ID=31
ros2 run gca_client gca_client position-shell
```

在 `position>` 提示符下逐行操作：

```text
configure
activate
claim
reference 0
arm
relative 5 2 5 10
wait
move 0 2 5 10
wait
state
quit
```

`claim` 领取 AUTO 控制权并在这个客户端运行期间自动续租；不需要手工拼接 token。`reference 0` 把当前编码器位置定义为软件坐标 0 mm，不写编码器置零寄存器，也不移动机构。可以指定非零参考值。

`move` 是绝对位置，`relative` 是相对本次任务接纳时实测位置的位移。它们后面的四个数依次为：**目标或位移 mm、最大速度 mm/s、最大加速度 mm/s²、执行超时 s**。后三个数省略时分别为 2、5、10。一个运行期间各次任务共用同一参考；相对移动不会修改参考零点。

`move` 或 `relative` 提交后立即返回提示符；`accepted=true` 只表示任务已接纳。一次只允许一个任务。在提示符可用时，可以输入 `state` 查看位置、`cancel` 取消当前 Action，或输入 `stop` 请求停车；`cancel` 和 `stop` 均不要求先到达目标。

`wait` 会阻塞当前终端，直到结果返回；等待期间不能在同一提示符输入 `stop`。需要随时手动停车时，提交后先保留提示符，用 `state` 观察，再输入 `stop` 或 `cancel`。在 `wait` 期间按 Ctrl+C 会进入客户端退出清理，尝试停车、解除使能及释放会话；应查看退出结果，不能把按键本身当作停车确认。

成功结果必须包含 `outcome=1`、`stop_confirmed=true`、`outputs_held=false`，最终编码器位置和速度满足要求。取消的 `outcome=2`，失败为 3。`quit`、正常 EOF 或退出处理会请求停车、解除使能并释放控制权。

`state` 显示实际 `control_policy`、会话、使能、参考、位置和速度有效性。另一个不持有控制权的终端也可只读查询：

```bash
ros2 run gca_client gca_client status
```

需要核对完整停车证据时，在相同 ROS 环境与 domain 中读取：

```bash
ros2 topic echo --once /gca/state
```

位置模式应显示 `control_policy: POSITION_EXTERNAL_ENCODER`。停车后应离开 `EXECUTING`，`stop.linear_confirmed` 和 `stop.servo_confirmed` 均为 true，反馈有效且速度为零；退出后还应为 `armed: false`、`servo.torque_enabled: false`。当前简要 `state` 不显示这两个独立停止标志，历史 `stop: CONFIRMED` 也可能在新任务运动中暂时保留，不能仅凭该字符串判断已经停车。

正常任务停车后可以提交下一任务。反馈故障、失去会话或设备重连后不得续跑旧任务；先查看故障和停车状态，恢复通信，再显式重建参考、使能并提交新任务。`reset` 仅请求复位故障，不恢复旧目标。

## 位移指令、速度与执行期限

| 命令示例 | 含义 |
|---|---|
| `reference 0` | 把当前位置定义为本次零点；不会移动，也不是自动寻零 |
| `relative 100 2 5 120` | 从任务接纳时的位置向正方向推进 100 mm，速度上限 2 mm/s、加速度上限 5 mm/s²，最多执行 120 秒 |
| `relative -5 2 5 120` | 从任务接纳时的位置后退 5 mm，其余参数同上 |
| `move 20 2 5 120` | 移到当前固定坐标系中的 +20 mm；不是再推进 20 mm |
| `move 0 2 5 120` | 返回当前软件零点；不是寻找原点开关或机械端点 |
| `cancel` | 请求取消本次任务并等待结果，成功取消的 outcome 为 2 |
| `stop` | 通过当前 AUTO 会话请求停车；不自动重新提交原目标 |
| `quit` | 停车、解除使能、释放会话并退出客户端 |

所有速度、加速度参数填写正的上限；运动方向由目标位置或相对位移的正负决定。推进正方向沿用手柄上推方向，对应舵机负 RPM；回退对应正 RPM。加减速和临近目标时速度可以低于填写的上限。

执行期限是整条运动任务的时间预算。100 mm / 2 mm/s 的匀速段理论上就需要 50 秒，另有加减速、误差修正及停车，因此长程示例显式使用 120 秒，不使用省略参数时的 10 秒默认值。实际 100 mm 推进耗时约 60.46 秒。

延长执行期限不延长通信和反馈时限：编码器反馈仍须在 100 ms 内，舵机反馈仍须在 250 ms 内，控制输出有效期仍为 300 ms；会话由持久客户端自动续租。这些参数不能混作一个“超时时间”。

## 零点保持与重启后的恢复

同一运行期间的所有 `move` 和 `relative` 共用一个参考；不要在每次移动前重新执行 `reference 0`。需要往返时，保持 launch 与 `position-shell` 运行，先完成推进，再提交 `move 0 ...`。

程序重启、设备重连或退出再进入位置模式后，软件参考需要重新建立。以下两种恢复情况具有不同的前提：

- 已把机构放回已知的物理零点：在未使能状态执行 `reference 0`。
- 当前物理位置已知：在未使能状态执行 `reference <已知位置mm>`。例如本轮经操作者确认导丝仍处于上次结束位置，才用 `reference 99.47857739534447` 恢复坐标；这个数不是通用常量，不能用于另一轮未知位置。

编码器计数保留不等于导丝零点必然保留。测量轮脱离导丝或被单独转动后，旧原始计数与导丝位置的对应关系已经改变；不能直接沿用旧计数推算回零。本轮旧零点计数因此作废，依据操作者确认的导丝位置重新建立参考。普通 `position-shell` 不会自动从历史日志恢复零点。

## 切换手柄与上位机控制

当前持权客户端先执行 `stop`，确认停车，再 `disarm`、`release`、`quit`；随后在启动终端按 Ctrl+C，等待旧装配退出并释放串口。停止未确认时不能用强制杀进程或抢占串口代替切换。

切换到手柄模式时导出另一份快照：

```bash
python3 tools/export_feed_axis_config.py artifacts/position_config/manual.json \
  --control-mode manual --backend simulation --manual-input-mode joydev_hold
ros2 launch system_bringup feed_axis.launch.py \
  control_mode:=manual backend:=simulation \
  servo_config_file:="$PWD/artifacts/position_config/manual.json" \
  profile_file:="$PWD/artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml"
```

手柄入口的配置、使能和 LT/B 操作沿用[舵机指南](SERVO_USER_GUIDE.md)。切回手柄时需释放 LT、摇杆回中，再按该指南使能。切回位置模式时重新建立参考。运行中不能通过 `ros2 param set` 更改模式，也不会因另一种指令到达而自动切换。上位机模式使用 `stop` 或 Action cancel；手柄 B 没有跨所有者停止 AUTO 任务的权限。

## 真实设备入口

终端 A：先退出此前占用舵机或编码器的装配，再从项目根目录执行。运行用户需要访问 `/dev/ttySC1` 和配置中的舵机串口；加入 `dialout` 后应重新登录，使新终端取得组权限。驱动轮与测量轮应压紧同一运动对象，实际行程应容纳本次目标。

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
export ROS_DOMAIN_ID=31
mkdir -p artifacts/position_config
python3 tools/export_feed_axis_config.py artifacts/position_config/hardware.json \
  --control-mode position --backend hardware
POSITION_RECORD_DIR="$(mktemp -d "$PWD/artifacts/position_config/hardware.XXXXXX")" || exit 1
printf '本次记录目录：%s\n' "$POSITION_RECORD_DIR"
ros2 launch system_bringup feed_axis.launch.py \
  control_mode:=position backend:=hardware \
  servo_config_file:="$PWD/artifacts/position_config/hardware.json" \
  trusted_local_control:=true \
  record_directory:="$POSITION_RECORD_DIR"
```

终端 B：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
export ROS_DOMAIN_ID=31
ros2 run gca_client gca_client position-shell
```

在 `position>` 内逐条输入，观察每一步是否成功；不要把整段盲目粘贴为自动运动脚本：

```text
configure
activate
claim
reference 0
arm
state
relative 100 2 5 120
```

这里的 `reference 0` 以“当前机构位置就是本次起点”为前提。提交后可以用 `state` 观察，或用 `wait` 等待结果；只有结果满足到位及停车要求后，才继续下一条指令。在同一会话中回零使用 `move 0 2 5 120`。本轮 2 mm/s、100 mm 推进有通过记录；这不等于所有方向、距离和速度组合均已验收。

结束时在终端 B 输入 `quit`，确认清理没有错误后，再在终端 A 按 Ctrl+C 退出 launch。不要另开客户端重复 `claim`；`SESSION_ALREADY_OWNED` 表示已有会话，优先回到原来的 `position-shell`。

以上使用本机受信控制，保持 `LOCALHOST` 发现范围。硬件与仿真使用相同命令，但必须分别导出匹配后端的快照；当前几何比例、PID 初值及制动缺省值的依据见[参数说明](CONFIGURATION_AND_CALIBRATION.md)。

位置速度上限为 10 mm/s，加速度上限为 10 mm/s²；扣除两端余量后允许目标为 −40～990 mm。正常完成需要误差不超过 1 mm、外部测量速度不超过 0.1 mm/s，连续 0.3 秒，然后取得停车证据。这些要求不等于已经证实的机构精度。

## 当前故障与已知限制

| 现象或错误 | 含义与处理 |
|---|---|
| `SESSION_ALREADY_OWNED` | 已有控制会话。使用原持权客户端，或由它停车、退出后再领取 |
| `REFERENCE_REQUIRED` 或线性参考无效 | 尚未建立有效坐标，或重连使参考失效；确认物理位置后重新设置参考 |
| `ENCODER_FEEDBACK_STALE` 等反馈错误 | 当前反馈不能用于控制；检查串口、供电与记录，不能靠延长任务期限消除 |
| `OUTPUT_EXPIRED` | 当前错误码也可能来自任务执行期限到期，不能仅凭名字认定串口掉线；须结合任务期限、输出更新和原反馈时间判断 |
| `TRIAL_POSITION_ENVELOPE` | 本轮专用测试脚本的观察边界被越过，不是通用节点故障码；普通 shell 没有这个专用检查 |
| 8 mm/s 回零超调 | 本轮越过 −1 mm 测试下界后停止，最终 −1.854 mm；未达到稳定到位条件，也没有验证继续运行能否自行收敛 |
| 舵机在转，编码器位移很小 | 检查驱动轮与导丝、测量轮的接触关系；舵机理论行程不能替代实际编码器反馈 |

测试脚本中的 −1～101 mm 界限仅用于那次观察，不能当作生产运行的全局行程配置。当前全局参数仍是 `linear_min_mm=-50`、`linear_max_mm=1000`，两端各保留 10 mm 余量。任务到期的错误码区分、较高速回零的减速与超调控制仍待后续改进；本次文档整理没有修改 PID、半径或期限保护逻辑。

## 上位机单独部署与局域网连接

上位机首期使用 ROS2 客户端；只构建接口与客户端两个包即可，不需要安装舵机驱动或 `gca_core`。在装有 ROS2 Jazzy 的上位机复制或检出工程后：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src/gca_system_interfaces src/gca_client --ignore-src -r -y
colcon build --packages-select gca_system_interfaces gca_client
source install/local_setup.bash
export ROS_DOMAIN_ID=31
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
ros2 run gca_client gca_client position-shell
```

树莓派启动命令增加 `discovery_range:=SUBNET`，双方使用相同 `ROS_DOMAIN_ID`；本机测试保留默认 `LOCALHOST`。以上网络设置可以用于仿真；真实硬件的跨机控制还须配置现有 SROS2 Enforce 安全部署，不能把本机受信选项用于局域网硬件控制。发现范围只决定 ROS 网络可见范围，不替代已有会话与许可。上位机位置命令不携带用于与树莓派单调时钟比较的客户端时间。本轮没有第二台 ROS 主机，跨机通信未验证。

## 记录与软件复现

指定 `record_directory` 后，设备记录位于该目录的 `device/`，采用有界异步 JSONL。记录包含编码器原始计数、原时间及序号、参考、目标和实际位置、速度及有效性、PID 输出与接受输出、制动参数来源、STOP 原因和任务终态；授权 token 不写入记录。

有限仿真脚本只启动一次 ROS 装配，使用 0.85 的固定模拟传动比例制造滑移，合并正反向绝对/相对任务、取消和 STOP：

```bash
source tools/common.sh
gca_source_install
python3 tools/verify_position_simulation.py \
  --output "$PWD/artifacts/position_pid/simulation-$(date +%Y%m%d-%H%M%S)"
```

输出目录必须尚不存在。脚本只生成仿真配置，不连接真实硬件；保存启动日志、状态样本和 `report.json`，并设置有限总时长。正式验证通过后无需反复运行。
