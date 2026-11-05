# 配置、版本事务与标定

独立位移测量可用离线工具处理：`source tools/common.sh` 后执行 `"$GCA_PYTHON" tools/calibrate_from_measurements.py --input measurements.csv --output artifacts/calibration`。CSV列为 `count_start,count_end,measured_displacement_mm`，推进位移为正、退出为负，必须来自独立仪器而非目标值。工具拟合有符号比例、正反行程统计和残差，只写报告，不连接设备、不自动修改配置或verified。计数跨回绕时应先依据已核实规则整理连续计数；不能直接拿跨epoch的差值拟合。

配置源码是 `gca/config.py` 的 `RuntimeConfig` 及 `gca/devices/common.py` 的三个设备配置类；正式设备装配在 `gca/devices/factory.py`。下表描述当前实际代码，不把设计草案中的未来字段视为已实现。

当前默认配置是仿真模型。**硬件配置没有可直接运行的物理默认值**：`system_bringup/config/hardware.template.yaml` 中的null和false故意使configure/ARM失败。安装依赖、打开串口或手动翻转verified标志均不等于接线、标定或HIL通过。

## 文件结构、加载与单位

```yaml
runtime:
  backend: simulation
  revision: simulation-v1
  # RuntimeConfig字段；仿真可省略非关键值并使用模型默认。
# 硬件文件另有devices，加载时合入RuntimeConfig.hardware。
# devices:
#   servo: {...}
#   encoder: {...}
#   pulse: {...}
```

`gca_ros`读取节点参数 `config_file`（为空时兼容 `configuration_file`），解析 `runtime`和顶层 `devices`。硬件模式要求全部非simulation运行字段显式出现，未知键、错误类型、NaN/Infinity、非法范围直接拒绝；例如布尔值不能冒充GPIO号或队列容量。完整硬件模板已包含所有必需字段。

**配置文件的运行/核心量使用mm、deg、秒；ROS运动消息使用m、rad、秒。**配置文件不会因为由ROS加载就自动变成SI单位。只有 `gca_ros/conversion.py` 的消息边界转换一次；寄存器整数/RPM与GPIO编号只在设备边界出现。所有内部期限为本地单调纳秒，跨机器不直接比较这些数值。

从工程根，在已完成环境准备和构建后可运行实际存在的入口：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source ros2_ws/install/local_setup.bash
ros2 launch system_bringup simulation.launch.py
```

加载自建仿真文件时使用已存在的launch参数：

```bash
ros2 launch system_bringup simulation.launch.py config_file:=/绝对路径/你的仿真配置.yaml
```

上面示例中的路径须替换成实际文件；不是待填写的硬件配置。本轮不执行真实硬件launch。普通launch保持DISARMED，控制顺序见 [用户指南](USER_GUIDE.md)。

## RuntimeConfig完整字段表

“模型默认”只用于仿真/软件资源策略；硬件物理项必须另有数据来源。所有数值还必须有限，整数类型严格校验。

### 执行、期限和资源

| 字段 | 单位/类型 | 模型默认 | 当前校验与含义 |
|---|---|---:|---|
| backend | 字符串 | simulation | 仅simulation/hardware；不自动回退 |
| revision | 字符串 | simulation-v1 | 非空、最长128；有效变更需新版本 |
| control_period_s | s | 0.01 | >0；本地控制调度，不承诺100Hz物理闭环 |
| encoder_period_s | s | 0.02 | >0且小于feedback_max_age_s；有效采样率须测量 |
| io_period_s | s | 0.005 | >0；I/O轮询/停止检查间隔 |
| feedback_max_age_s | s | 0.20 | >0；原始有效采样的最大年龄，也约束估计dt |
| output_ttl_s | s | 0.10 | >0且大于pulse_segment_s；普通输出期限 |
| io_timeout_s | s | 0.05 | >0；一轮设备事务预算；适配器还落实底层超时 |
| stop_timeout_s | s | 1.0 | >0；等待停止证据的总期限，超时UNKNOWN |
| lease_max_s | s | 5.0 | >0；执行授权允许的最大续租长度 |
| queue_capacity | 条数/整数 | 64 | >0；命令/可靠I/O事件队列容量，满队列行为可见 |
| history_capacity | 条数/整数 | 256 | >0；有界命令结果/去重记录；淘汰不放宽序号高水位 |
| hardware_verified | bool | false | hardware模式必须true且hardware非空；不能替代逐设备证据 |
| hardware | 映射 | {} | 仅真实装配时使用servo/encoder/pulse；YAML顶层devices合入这里 |

期限之间不使用统一“心跳超时”替代。`output_ttl_s`、授权租约、反馈年龄和任务执行超时负责不同问题；状态/遥测活跃不会刷新旧样本或旧运动的授权。

### 行程、速度和标定

| 字段 | 单位 | 模型默认 | 当前校验与含义 |
|---|---|---:|---|
| linear_min_mm | mm | -100.0 | 必须小于linear_max_mm；软件参考坐标中的下界 |
| linear_max_mm | mm | 100.0 | 目标与运动过程上界；不能替代真实机械限位 |
| rotation_min_deg | deg | -720.0 | 必须小于rotation_max_deg；累计角下界 |
| rotation_max_deg | deg | 720.0 | 累计角上界，不按360°取模 |
| max_linear_mm_s | mm/s | 20.0 | >0；不超过舵机原始速度限制换算能力 |
| max_linear_mm_s2 | mm/s² | 100.0 | >0；参考轨迹/输出变化限制 |
| max_rotation_deg_s | deg/s | 180.0 | >0；开环旋转规划限制 |
| max_rotation_deg_s2 | deg/s² | 720.0 | >0；开环旋转加减速限制 |
| encoder_mm_per_count | mm/count | 0.01 | >0；需多点往返独立位移标定 |
| encoder_count_modulus | count/整数 | 0 | 0或>=2；0不做模回绕；启用模数必须与编码器协议一致 |
| max_encoder_speed_mm_s | mm/s | 100.0 | >0；不合理跳变检查；不是放宽真实最大速度的通道 |
| pulses_per_motor_rev | pulse/电机圈，整数 | 6400 | >0；依据驱动细分及实测，不依据目录名 |
| catheter_per_motor_ratio | deg/deg | 17/30 | >0；导管角/电机角；与相反定义不能混用 |
| pulse_segment_s | s | 0.03 | >0且严格小于output_ttl_s；整段在提交时也必须落入许可窗口 |

`encoder_count_modulus`若启用，应为经过手册核实的有效计数周期，实际使用至少为2。采样间隔内真实变化不得超过半周期，否则软件无法唯一展开。`hardware.encoder.modulus`用于原始值范围检查，runtime的modulus用于累计展开，二者应按同一协议填写。0、None、signed不是同一概念。

### 控制器和到位

控制输出为线速度：`v_ref + Kp*e + I - Kd*v_measured`。以下量纲与该公式一致，不能套用旧“PID输出位置增量”的增益。

| 字段 | 单位 | 模型默认 | 当前校验与含义 |
|---|---|---:|---|
| kp | 1/s | 3.0 | >=0；位置误差到速度贡献 |
| ki | 1/s² | 0.0 | >=0；积分速度贡献增长率，默认先P |
| kd | 无量纲 | 0.0 | >=0；作用于测量速度，默认关闭 |
| kaw | 1/s | 2.0 | >=0；最终受限输出的抗饱和跟踪 |
| integral_limit_mm_s | mm/s | 5.0 | >=0；积分项本身的速度贡献上限 |
| position_tolerance_mm | mm | 0.08 | >0；最终目标误差阈值，非实机精度承诺 |
| velocity_tolerance_mm_s | mm/s | 0.15 | >0；有效测量速度到位/停止阈值 |
| settle_s | s | 0.15 | >0；所有到位条件连续满足的驻留时间 |

每个新有效样本至多更新一次PID；首次/长间隔速度无效，不能填0满足到位。轨迹完成、最终误差、有效测量速度和驻留共同决定成功。异步输出回执按序号/代次只结算一次量化残差。

### 仿真专用参数

| 字段 | 单位/类型 | 默认 | 解释 |
|---|---|---:|---|
| simulation_seed | 整数 | 7 | 固定随机种子，便于复现噪声序列 |
| simulation_noise_mm | mm | 0.0 | >=0；编码器高斯噪声标准差，实际模型见simulation/devices.py |
| simulation_encoder_delay_s | s | 0.0 | >=0；模型采样延迟，原始采样时间不会刷新为到达时刻 |
| simulation_slip | 比例 | 0.0 | 0≤值<1；独立plant的运动传递损失 |

plant的默认一阶时间常数、死区、饱和和每脉冲角度还定义在 `simulation/plant.py`，不是设备标定。仿真只消费实际接受的输出，无权读取任务目标后直接修改测量值；模型结果不能替代真实负载/摩擦/弹性实验。

## 真实舵机配置：devices.servo

当前只支持经核实的 **XM430-W350、model_number=1020、固件≥38**。字段虽能描述其它数字，适配器仍会在写寄存器前拒绝不支持型号。物理连接、准确型号和HIL尚待确认。

| 字段 | 单位/类型 | 允许值/当前行为 | 硬件默认策略 |
|---|---|---|---|
| port | 字符串 | 非空且≤512字符，稳定设备路径；不盲试其它USB设备 | null，必填 |
| baudrate | bit/s，整数 | >0，且被官方SDK支持；与设备一致 | null，必填 |
| device_id | 整数 | 0–252，读/写关联目标 | null，必填 |
| model_number | 整数 | 连接后核对；当前只支持1020 | null，必填 |
| mm_per_revolution | mm/舵机圈 | 有限且>0，实测传动比例 | null，必填 |
| velocity_unit_rpm | rpm/raw | 当前白名单必须0.229 | null，必填；按型号核实 |
| max_velocity_raw | raw整数 | 1–1023，速度目标幅值上限 | null，必填 |
| watchdog_raw | 20ms/raw，整数 | 1–127；生产运行不允许0关闭 | null，必填 |
| direction_sign | ±1 | 只在设备映射应用一次 | null，工厂要求显式填写 |
| transaction_timeout_s | s | >0；工厂若省略则使用显式runtime.io_timeout_s；模板要求填写 | null，待测 |
| verified | bool | 必须true才允许open；必须关联确认记录 | false |

构造器的少量技术默认不是硬件验收值。模式事务临时写watchdog=0清锁存错误仅发生在扭矩禁止时，随后核对非零运行值。速度配置还需满足：`60*max_linear_mm_s/(mm_per_revolution*velocity_unit_rpm)`不超过raw限制（考虑半单位量化）。

## 真实编码器配置：devices.encoder

| 字段 | 单位/类型 | 允许值/含义 | 硬件默认策略 |
|---|---|---|---|
| port | 字符串 | 非空且≤512；路径不证明TTL/RS-485 | null，必填 |
| baudrate | bit/s，整数 | >0，匹配设备 | null，必填 |
| address | 整数 | 1–247，响应站号必须匹配 | null，必填 |
| register | 寄存器地址，整数 | ≥0；读取范围不得越过65535 | null，必填 |
| bits | bit，整数 | 16、32或64；寄存器数=bits/16 | null，必填 |
| signed | bool | 二进制补码解释与否；不是运动方向参数 | null，必填 |
| byteorder | 字符串 | big或little | null，工厂要求显式填写 |
| modulus | count整数或null | 如启用须>1且≤2^bits；原始值必须落在该范围 | null表示不额外检查模范围，须有协议依据 |
| quiet_s | s | >0；重连连续静默窗口，应覆盖最大迟到响应 | null，必填 |
| transaction_timeout_s | s | >0；工厂省略时映射runtime.io_timeout_s | null，待测 |
| verified | bool | 必须true才连接；不伪造设备可读身份能力 | false |

协议没有事务ID；失败后必须显式重连并完成静默重同步。协议适配器只读计数，不自动写硬件零位、方向或波特率。编码器如果没有可读身份寄存器，仅报告configured_identity_only。

## 真实波形配置：devices.pulse

| 字段 | 单位/类型 | 允许值/含义 | 硬件默认策略 |
|---|---|---|---|
| pulse_gpio | BCM整数 | 0–31，须与direction_gpio不同 | null，核线后填 |
| direction_gpio | BCM整数 | 0–31，须与pulse_gpio不同 | null，核线后填 |
| high_us | µs整数 | >0，满足驱动最小高电平 | null，按手册/测量 |
| low_us | µs整数 | >0，满足最小低电平 | null，按手册/测量 |
| direction_setup_us | µs整数 | >0，首有效沿前方向建立 | null，按手册/测量 |
| direction_hold_us | µs整数 | >0，最后有效沿后方向保持 | null，按手册/测量 |
| max_segment_s | s | >0；不大于runtime.output_ttl_s；不小于runtime.pulse_segment_s | null，待测 |
| max_segment_pulses | pulse，整数 | >0；显式内存/波形队列上限，超出后端容量时拒绝 | null，待测 |
| positive_direction_level | 电平整数 | 0或1，实测正方向 | null，必填 |
| host | 字符串 | 仅localhost或127.0.0.1；归一化数值IPv4、不做DNS；远程/IPv6拒绝 | localhost |
| port | TCP整数 | 1024–30000 | 8888，软件协议部署值 |
| socket_timeout_s | s | >0；省略时从runtime.io_timeout_s映射 | null，待测 |
| verified | bool | 接口、平台等已核实 | false |
| exclusive_wave_owner | bool | 本机锁与daemon访问受控，不能靠任意客户端自称独占 | false |
| waveform_hil_verified | bool | 有完整段波形/计数证据和H02记录 | false |

硬件宿主还要求强制DDS安全（ROS_SECURITY_ENABLE=true且ROS_SECURITY_STRATEGY=Enforce），或ROS_LOCALHOST_ONLY=1配合显式trusted_local_control=true的受控本机部署；仅设置发现范围不是认证。三个波形布尔条件均满足才允许真实波形open。单段时长包括方向前后时序，必须容纳全部高低电平时间。后端不是按elapsed time猜计数；中止未知作废参考。当前没有软件EN输出，也没有自动读取细分拨码。见 [接线说明](HARDWARE_WIRING.md)。

## ROS宿主附加参数

这些不是RuntimeConfig字段，改变需重启/重新构建节点，不能混进runtime映射。

| 参数 | 默认/约束 | 作用 |
|---|---|---|
| config_file / configuration_file | 空字符串；前者优先 | 完整YAML路径，实际推荐launch的config_file |
| coordinator_key | 启动时由launch共享的随机能力 | 设备管理授权，不写日志，不当作网络认证替代物 |
| trusted_local_control | false | 需同时ROS_LOCALHOST_ONLY=1；仅受控本机硬件部署，须核实用户/设备权限 |
| record_directory | 空字符串 | 记录输出目录；标准仿真launch会分配本次运行目录 |
| executor_threads | 默认3，正整数，由MultiThreadedExecutor校验 | ROS回调线程数，不代表控制采样率 |
| telemetry_hz | 10.0，0.5–100 | 遥测发布率，与PID有效采样率独立 |

不要用参数dump向不受控日志复制coordinator_key。查询公开控制参数可用真实命令 `ros2 param get /gca_control_node kp`。

协调器 `/system_coordinator` 还读取下列启动参数。它们控制源会话和下游调用，不是设备配置，也不能在线延长已发出的单次运动许可。

| 参数 | 默认/约束 | 作用 |
|---|---|---|
| profile | gca_only；其它值拒绝启动 | 明确本轮没有完整磁场实验执行器 |
| coordinator_key | 默认空值拒绝启动；由launch与设备节点共享随机值 | 领取设备执行许可所需的能力，不能写入记录或当作DDS身份认证 |
| source_lease_max_s | 5.0 s；有限且≥0.1 s | 源控制会话的最大租约；每次申请仍须在0.1 s至该上限之间 |
| executor_lease_s | 2.0 s；有限且0.1–5.0 s | 下游执行许可的单次期限，同时不得超过源会话剩余期限 |
| rpc_timeout_s | 2.0 s；有限且>0 | 等待下游服务的墙钟期限，超时失败；不改变本地输出TTL |

变更后重启协调器并重新领取会话。普通launch不传入这三个时间参数，使用上述已测试的软件默认值；实机期限仍须结合H03测量确定。

## 配置版本事务与允许的在线调整

当前只允许 `revision、kp、ki、kd、kaw、integral_limit_mm_s`在 **DISARMED、无活动任务/保持/Jog/预留、无待应用配置**时更新。严格说这是“禁用输出时重建控制器”，不是运动中热调参。其它字段包括采样周期、限位、比例、波形、队列和硬件接线，都需要 `RECONFIGURE_REQUIRED`：停止并确认、DISARM、生命周期deactivate/cleanup、修改完整文件，再configure/activate，重新建立失效参考并显式ARM。

公共服务为 `/system/apply_config`，协调器再调用设备服务 `/gca/apply_config`，类型均为 `gca_system_interfaces/srv/ApplyConfig`。可通过实际IDL查看字段：

```bash
ros2 interface show gca_system_interfaces/srv/ApplyConfig
```

请求包含 `context`、对应层级的授权 `token`、完整 `configuration_yaml`；不能把源控制端token冒充设备端token。配置字符串最多64 KiB，可包含 `runtime`和 `devices`；新revision必须与实际变更关联。已安装CLI提供 `gca_cli apply-config PATH`，使用私有session文件经协调器调用；Python客户端可在同一持续续租的会话中调用 `client.apply_config(yaml_text)`。交互shell的命令列表中没有apply。单次CLI调用需要现有会话仍有效，操作顺序见 [用户指南](USER_GUIDE.md)。

响应中的accepted只说明受理，不代表硬件/控制任务已应用。`on_set`做校验，`post_set`排队不可变候选，控制任务确认后更新applied版本。状态中的requested_config_revision与applied_config_revision必须一致后才允许ARM；不同则检查拒绝原因并重新配置，不能继续执行。ROS参数显示值也不替代实际applied版本。

部分配置失败不能提升实际应用版本。校准、接线或限制变更不在运行中逐项更新；保持旧有效配置或进入待重新配置状态，避免“显示新参数，硬件仍旧参数”的混合状态。

## 方向与比例标定步骤

以下为后续取得实机授权后的程序，本轮未执行。先完成 [H01–H03](HIL_ACCEPTANCE.md) 所需接线和停止能力检查。

1. **固定坐标。**推进插入为正；旋转明确观察方向和正向，累计角不取模。记录载荷、夹持状态和参考位置。软件参考设置不改变硬件零位，也不声称机械回零。
2. **核对推进方向。**在有限允许行程内低速检查舵机方向与编码器增量；`servo.direction_sign`只做一次输出方向映射。当前encoder_mm_per_count要求正值，signed仅表示原始数字格式。插入时计数若减少，应按设备手册进行显式维护方向调整或修改实际安装，不能写负比例绕过校验；如需软件反相，应增加经审查的单一映射后再验收。
3. **编码器比例。**从独立位移测量取得多个正/负行程及原始计数，计算/拟合 `Δx_mm/Δcount`。保留原始对，不只保存最终比值；记录往返差、载荷、滑移、重复性和不确定度。核对计数范围和回绕，采样跨越半周期时不能唯一展开。
4. **舵机传动比例。**用实测舵机转数与推进位移得到 `mm_per_revolution`。舵机理论轮径只能作初估，外部位移是闭环依据；滑移/柔顺会使两者不同。
5. **步进比例。**检查实际细分拨码，外部仪器计完整有效沿与电机圈数，再测导管角/电机角。分别确定P和r，不用两者错误互相抵消。旧6400/25600冲突和17/30须重新证实；测试正反和多圈。
6. **期限与停止余量。**测采样/事务延迟分布、取消尾段、实际减速和最大停止距离。按最坏工况设置反馈年龄、输出期限、段时长、watchdog和行程余量。公式估算仅用于设计，不能替代停止实验。
7. **整定控制。**先低速P，确认方向、比例、限幅和停止，再根据稳态误差引入I；D默认关闭。改变输出定义后不继承旧PID增益。利用目标/参考/测量/输出与P/I/D诊断解释超调，避免只靠最终曲线“看起来平滑”。
8. **版本化。**形成新的标定与完整配置版本，在DISARMED重新配置；旧reference_id和任务失效。不同机械安装、负载或方向变化需重新确认适用范围。

### 标定记录模板

```yaml
calibration_record_id: null
status: NOT_RUN
operator: null
date: null
software_build: null
configuration_revision: null
wiring_revision: null
device_models_serials_firmware: []
instruments_and_calibration: []
coordinate_directions: null
load_and_clamping: null
raw_pairs: []  # 时间、原始count、实测mm、舵机圈数、pulse、实测deg
fit_result: null
forward_reverse_residuals: []
uncertainty_and_range: null
stop_time_distance_measurements: []
approved_runtime_values: null
reference_id_after_reconfigure: null
reviewer: null
```

模型参数可用于确定性回归与故障注入；只有对应实物、环境和原始测量记录才能支撑实机标定。所有尚未测量的数值仍保持NOT_RUN/null，并由能力门控阻止实机ARM。
