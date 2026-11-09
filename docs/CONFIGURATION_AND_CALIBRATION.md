# 配置、版本事务与标定

独立位移测量可用离线工具处理：`source tools/common.sh` 后执行 `"$GCA_PYTHON" tools/calibrate_from_measurements.py --input measurements.csv --output artifacts/calibration`。CSV列为 `count_start,count_end,measured_displacement_mm`，推进位移为正、退出为负，必须来自独立仪器而非目标值。工具拟合有符号比例、正反行程统计和残差，只写报告，不连接设备、不自动修改配置或verified。计数跨回绕时应先依据已核实规则整理连续计数；不能直接拿跨epoch的差值拟合。

原双轴配置源码是 `gca/config.py` 的 `RuntimeConfig` 及 `gca/devices/common.py` 的三个设备配置类；正式设备装配在 `gca/devices/factory.py`。当前推进舵机入口使用下节的独立 Python 配置与 JSON 快照：manual 不要求外部编码器或步进设备，position 装配独立外部编码器。

当前默认入口是仿真模型。**原双轴硬件模板没有可直接运行的物理默认值**：`system_bringup/config/hardware.template.yaml` 中的null和false故意使configure/ARM失败。安装依赖、打开串口或手动翻转verified标志均不等于接线、标定或HIL通过。

## 仅舵机配置：独立脚本与快照

人工编辑入口是 [`config/servo_settings.py`](../config/servo_settings.py)，严格类型与摘要校验在 `gca/servo_config.py`。导入、显示和导出只使用标准库，不访问设备或 ROS：

```bash
python3 config/servo_settings.py show --manual-input-mode joydev_hold
python3 config/servo_settings.py export artifacts/servo_config/simulation.json \
  --backend simulation --manual-input-mode joydev_hold
python3 config/servo_settings.py export artifacts/servo_config/hardware.json \
  --backend hardware --manual-input-mode joydev_hold
```

以上为实体手柄持续控制配置；省略 `--manual-input-mode` 时仍默认 `qualified_state`，要求匹配资格，合成输入模拟也使用这一默认策略。两种策略不通过修改实体标定文件相互冒充。

快照含 schema、配置修订号、后端、设备参数及 SHA-256，不作为第二个手工配置来源。`servo_simulation.launch.py`／`servo_hardware.launch.py` 使用 `servo_config_file` 将同一文件同时交给设备、协调器和手柄；配置摘要与能力必须匹配。先 STOP 确认、DISARM、释放会话并退出节点，再修改数值、导出快照并重启整组节点重新 configure，无需编译 ROS 消息。设备的 cleanup/configure 不能替代手柄节点重载快照；文件路径参数不支持在线更换，原 `ApplyConfig` YAML 接口不接收此 JSON。

| 配置组 | 当前值或语义 | 证据边界 |
|---|---|---|
| 身份 | by-id 路径、57600、ID 1、型号 1020 | 本轮只读识别通过；每次连接仍核对实际设备 |
| 单位与方向 | 0.229 rpm/raw；`manual_axis_to_servo_sign=-1` | 上推反转的手柄约定，不是未来机构正位移标定 |
| 最大速度 | `max_velocity_raw=349`，即 79.921 rpm | 软件指令上限80 rpm；型号标称空载46 rpm@12 V、57 rpm@14.8 V，不能据指令宣称实测达到80 rpm |
| 加速度 | `max_acceleration_rpm_s=40.0`、`profile_acceleration_raw=12` | 当前空载联调参数，分别约束软件目标与设备响应，不代表已测得机械加速度边界 |
| 周期与期限 | 控制 0.04 s、I/O整轮目标周期 0.02 s、单事务 0.08 s、反馈年龄 0.25 s、输出和held观察 0.30 s、停止确认 2 s | 各自用途不同；校验预算关系不等于测得最坏时延 |
| 停止与总线保护 | `settle_s=0.10`、`velocity_tolerance_rpm=0.229`、`watchdog_raw=25` | LT/B 停车已有空载证据；两项专项失效测试后移，状态读取也能维持 watchdog |
| 位移闭环配置 | 旧字段 `mechanical_mapping=null`、`position_pid=null` 保留兼容 | schema 3 使用嵌套 `position`；manual 为 `null`，position 模式必须提供完整参数 |

历史[只读报告](../artifacts/servo_ros2/implementation-xce7i_e4/readonly/report.json)未写寄存器，实际读回模式 4、Goal Velocity 1023、watchdog 0，与候选快照不同是正常现象。当前真实入口configure会关闭力矩、按需切换模式1、读回限制并确认零目标。BFM映射模式已确认；本轮空载连续控制采用显式`joydev_hold`，无线资格仍未完成。

历史手柄参数修订 v3 修正了原150毫秒预算无法覆盖完整WRITE/ACK、READ及控制周期的问题。`joydev_hold`使用配置的300毫秒观察期限，既有样本及许可的截止时间不续期；LT/B独立STOP不等待该期限。原双轴profile与默认`qualified_state`的更严格输入约束保留。这里的参数修订号不同于下文的 `schema_version=3`；不满足现有预算校验的旧 JSON 仍须重新导出。

纯舵机不使用下文双轴 `mm_per_revolution`、编码器比例、PID 或脉冲字段。位移草稿中的舵机比例按用户确认的几何关系计算，尚无独立位移测量验证；舵机轴角不能替代外部位移反馈。完整启动、只读工具和记录说明见[舵机指南](SERVO_USER_GUIDE.md)。

## 上位机位移闭环参数草稿

两个独立配置入口提供位移闭环参数：[`config/servo_settings.py`](../config/servo_settings.py) 的 `position_settings_payload()` 保存舵机机械映射、PID 和运动限制；[`config/encoder_settings.py`](../config/encoder_settings.py) 的 `settings_payload()` 保存编码器通信记录、计数格式、机械映射和采集参数。它们只使用 Python 标准库，显示和导出不需要 ROS，不连接设备。

```bash
python3 config/servo_settings.py show --section position
python3 config/servo_settings.py export artifacts/position_config/servo.json --section position
python3 config/encoder_settings.py show
python3 config/encoder_settings.py export artifacts/position_config/encoder.json
```

修改源脚本中的值，再重新导出；JSON 不作为第二个手工配置来源。两个草稿分别带有 `kind="servo_position_draft"` 和 `kind="encoder_position_draft"`，**不能传入当前 `servo_config_file` 或当作原双轴设备配置使用**。舵机脚本省略 `--section` 时仍导出现有运行快照，`--backend` 与 `--manual-input-mode` 只作用于该运行区。草稿显示和导出仅用于保存参数，不表示这些参数已完成标定或通过运行配置校验。

当前手柄路径仍使用原生 RPM 控制，不依赖外部编码器。上位机位移 PID 通过下节的组合快照和显式 position 模式启用，不会因为参数已经填写就自动改变手柄模式。旧兼容字段 `mechanical_mapping`、`position_pid` 仍为 `null`，新位置配置放入 schema 3 的 `position`。

### 组合运行快照

组合导出工具是 [`tools/export_feed_axis_config.py`](../tools/export_feed_axis_config.py)。它只读取上述两个配置源，不连接设备；不要把单独导出的参数草稿直接传入 launch。

```bash
python3 tools/export_feed_axis_config.py artifacts/position_config/simulation.json \
  --control-mode position --backend simulation
python3 tools/export_feed_axis_config.py artifacts/position_config/hardware.json \
  --control-mode position --backend hardware
python3 tools/export_feed_axis_config.py artifacts/position_config/manual.json \
  --control-mode manual --backend simulation --manual-input-mode joydev_hold
```

schema 3 快照保留舵机运行区，并增加 `control_mode` 和 `position`。`position` 含 `mechanics`、`pid`、`motion`、`braking_defaults`、`effective_braking`、`encoder`。编码器通信、计数、换算、采集和速度窗口参数在运行快照的 `position.encoder` 中展开为明确字段；例如源脚本 `estimation.velocity_window_s` 对应 `position.encoder.velocity_window_s`。引脚与购买记录等不参与控制的元数据仍保留在源脚本，不写入另一套运行常量。

`manual` 快照的 `position=null`；`position` 快照必须包含完整位置参数，缺字段、未知字段、非法值、摘要不符或模式与配置不符均拒绝。执行策略由模式确定为 `MANUAL_SERVO_VELOCITY` 或 `POSITION_EXTERNAL_ENCODER`，不能由普通消息切换。兼容读取的 schema 1/2 快照只表示原有 manual 语义。

统一 `feed_axis.launch.py` 使用 `servo_config_file` 将快照交给设备和协调器；manual 还交给手柄节点。启动时必须显式传入匹配的 `control_mode` 和 `backend`；运行时不执行配置脚本，不能在线改模式或快照路径。使用与切换步骤见[位置用户指南](POSITION_USER_GUIDE.md)，验证结果见[实现状态](POSITION_PID_IMPLEMENTATION_STATUS.md)。

### 舵机参数与待确认项

当前位置模式已有有限实机记录，见[实现状态](POSITION_PID_IMPLEMENTATION_STATUS.md)：100 mm、2 mm/s 推进通过，8 mm/s 回零超调。下列参数仍保留本轮初值，未依据某一次舵机理论行程与编码器位移之比反向修改半径或 PID。软件参考不是配置常量；测量轮脱离导丝后恢复零点的前提见[用户指南](POSITION_USER_GUIDE.md#零点保持与重启后的恢复)。

以下路径均相对于 `position_settings_payload()` 返回值。用户已确认驱动轮半径 6 mm、输出轴直驱，并将现有摇杆上推的转向定义为推进正方向；该转向对应舵机负 RPM，所以位置转换系数为 `-1`。PID 已填入待实验调整的初值；`motion` 按用户指定填写，仅刹车减速度与刹车延迟保留 `None`（导出为 `null`）及 TODO。位置零点和参考标识在运行时显式建立，不把某次原始读数写成固定零点。

| 字段路径 | 当前值 | 单位 | 含义 |
|---|---|---|---|
| `mechanics.drive_wheel_radius_mm` | 6.0 | mm | 用户确认的驱动轮有效半径 |
| `mechanics.shaft_revolutions_per_wheel_revolution` | 1.0 | 舵机输出轴圈/驱动轮圈 | 直驱，不再计入舵机内部减速比 |
| `mechanics.mm_per_shaft_revolution` | `2π × 6 / 1 ≈ 37.6991` | mm/舵机输出轴圈 | 按无滑动前提计算，随半径和传动比更新；不是独立实测结果 |
| `mechanics.direction_sign` | −1 | +1 或 −1 | 正向推进转换为负 RPM，保持当前手柄上推方向 |
| `mechanics.calibration_id` | `geometry-r6mm-direct-v1` | 文本 | 当前几何比例及方向约定的版本标识 |
| `pid.kp` | 2.0 | 1/s | 比例增益初值；1 mm 位置误差产生 2 mm/s 的比例速度请求 |
| `pid.ki` | 0.0 | 1/s² | 积分暂时关闭，待稳态误差实测后整定 |
| `pid.kd` | 0.0 | 无量纲 | 测量速度阻尼暂时关闭，先确认测量噪声及延迟 |
| `pid.kaw` | 2.0 | 1/s | 抗积分饱和跟踪初值；`ki=0` 时不起作用 |
| `pid.integral_limit_mm_s` | 1.0 | mm/s | 积分项速度贡献上限初值；当前积分项保持为零 |
| `pid.dt_max_s` | 0.2 | s | PID 最大相邻有效样本间隔初值，不是期望采样周期 |
| `motion.linear_min_mm` | −50.0 | mm | 固定零点后方 50 mm；不是每条指令最多后退 50 mm |
| `motion.linear_max_mm` | 1000.0 | mm | 固定零点前方 1000 mm；多次移动共用这一上限 |
| `motion.max_linear_mm_s` | 10.0 | mm/s | 推进或后退每秒最多移动 10 mm |
| `motion.max_linear_mm_s2` | 10.0 | mm/s² | 每秒速度最多变化 10 mm/s；不代表实测刹车能力 |
| `motion.braking_deceleration_mm_s2` | `None` | mm/s² | 待测减速度；未知时使用下节的显式缺省值 |
| `motion.braking_delay_s` | `None` | s | 待测完整链路延迟；未知时使用下节的显式缺省值 |
| `motion.boundary_margin_mm` | 10.0 | mm | 两端各保留 10 mm，用于边界制动计算 |
| `motion.position_tolerance_mm` | 1.0 | mm | 距最终目标不超过 1 mm，满足到位的位置条件 |
| `motion.velocity_tolerance_mm_s` | 0.1 | mm/s | 外部编码器测得的推进速度绝对值不超过 0.1 mm/s，满足到位的速度条件 |
| `motion.settle_s` | 0.3 | s | 位置和速度条件同时连续满足 0.3 秒，且反馈持续有效，才确认到位 |

`calibration_id` 是用于记录和区分参数依据的标签，不是舵机寄存器或 PID 系数，也不参与数值计算。当前标签表示“6 mm 半径、直驱、无滑动几何模型，第 1 版”；它不表示已经通过实测标定。半径、传动关系、方向或标定依据改变时应更新标识，实测后可换成对应测量记录的标识。

PID 参数沿用 `PositionController` 的输出定义：`v_requested = v_ref + kp × (x_ref − x_measured) + I − kd × v_measured`，输出单位为 mm/s。目前从比例控制开始，避免尚未确认的积分累积、速度噪声及延迟影响整定；这组工程初值不是根据实机辨识得到的结果，不保证响应时间、超调或稳态误差。响应速度与稳定裕度需要结合系统响应权衡，参见 [MathWorks PID 整定说明](https://www.mathworks.com/help/control/getstart/pid-tuning-algorithm.html)。`dt_max_s` 在接入时须与编码器采样周期、反馈最大年龄及实际通信延迟一起确认，不能当作采样周期使用。

这些 `motion` 数值是用户指定的控制要求，不是实测性能结论。位置误差满足 1 mm 并不能单独判定任务完成：还需外部编码器测得的线速度绝对值不超过 0.1 mm/s，两个条件持续满足 0.3 秒。速度条件使用实际推进速度，不使用舵机 RPM 或舵机目标速度代替；编码器测量噪声能否支持该阈值仍待确认。`motion.settle_s` 用于位移任务到位判定，与现有运行区用于舵机停车确认的 `settle_s` 分开维护。

按现有 `boundary_velocity` 的计算方式，两端各扣除 10 mm 余量后，向外运动的速度上限在 −40 mm 和 990 mm 处降为零。这里是行程两端预留的距离，不是每条位移指令都少走 10 mm。位置运行时的允许目标范围同为 −40～990 mm，绝对目标和解析后的相对目标均检查这一范围。刹车减速度和完整链路延迟仍待测量，软件采用下节的显式缺省值。

### 刹车实测值未知时的缺省策略

`motion` 中的两项 `None` 继续表示“尚未测得”，不写入估计值冒充实测结果。另在同一脚本的 `braking_defaults` 中集中设置临时计算参数：

| 缺省字段 | 当前值 | 用途与依据 |
|---|---:|---|
| `braking_defaults.braking_deceleration_mm_s2` | 5.0 mm/s² | 暂按这一减速能力计算停车距离；比采用指令加速度 10 mm/s² 更早开始减速 |
| `braking_defaults.braking_delay_s` | 0.5 s | 暂为采样、通信和执行响应预留 0.5 秒；不是实测最坏时延 |

这是明确选择的工程初值，不是已验证的制动能力。每项分别解析：`motion` 中填写了有效数值，就优先使用该数值；仅当它为 `None` 时才采用对应缺省值。原始配置不被覆盖。零延迟是明确提供的零值，不会被当成未知；减速度为零、负延迟或非有限数等错误仍会报错，不用缺省值掩盖错误输入。

`show --section position` 和 `export --section position` 会同时输出 `motion`、`braking_defaults` 及派生的 `effective_braking`，位置草稿版本为 2。当前派生结果如下，不要另行手改：

```json
"effective_braking": {
  "deceleration_mm_s2": 5.0,
  "delay_s": 0.5,
  "deceleration_source": "default",
  "delay_source": "default"
}
```

`default` 表示采用缺省值，`configured` 表示采用 `motion` 中显式填写的值；来源标签不替代测量验证。缺省策略仍执行边界减速计算和 10 mm 余量限制，不是跳过制动保护。以 10 mm/s 为例，模型计算的延迟行程为 `10 × 0.5 = 5 mm`，减速行程为 `10² ÷ (2 × 5) = 10 mm`，加上边界余量共 25 mm；这个计算不能证明实机必然在该距离内停止。

纯 Python 的 [`resolve_braking_parameters`](../ros2_ws/src/gca_core/gca/domain/position.py) 实现参数选择、来源记录及边界计算。显示、组合导出与位置运行采用同一解析结果，这两项 `None` 不阻塞位置软件运行；实际制动能力仍需独立测量，不能把缺省值标为实测。

### 编码器已确认的通信记录

以下为本机已完成的只读采集设置，依据见[接口与读取报告](../artifacts/encoder_readonly/waveshare-b-setup-8uelsbj4/report.json)。这些记录证明当前参数能够取得合法响应，不证明编码器型号、计数回绕或毫米比例已经确认。

| 字段或配置组 | 当前记录 |
|---|---|
| `board.model`、`board.connector` | Waveshare RS485 CAN HAT (B)，RS485_1 |
| `board.driver`、`board.overlay` | `sc16is7xx`；`sc16is752-spi1,int_pin=24` |
| `board.spi_bus`、`board.chip_select` | SPI1，片选 0 |
| `board.miso_gpio_bcm`、`board.mosi_gpio_bcm`、`board.sclk_gpio_bcm` | BCM 19、20、21 |
| `board.chip_select_gpio_bcm`、`board.interrupt_gpio_bcm`、`board.crystal_hz` | BCM 18、24；14745600 Hz |
| `communication.interface`、`communication.protocol` | RS485，Modbus RTU |
| `communication.port`、`communication.baudrate` | `/dev/ttySC1`，38400 baud |
| `communication.bytesize`、`communication.parity`、`communication.stopbits` | 8、N、1 |
| `communication.address`、`communication.function_code` | 地址 1，功能码 3（读保持寄存器） |
| `communication.register_start`、`communication.register_count` | 从寄存器 0 开始读取 2 个寄存器 |
| `count_format.byte_order_in_register` | `big`，单个 16 位寄存器内部高字节在前；两个寄存器的排列顺序见下节 |

`board` 中的引脚均为 BCM 编号。该配置区仅记录已经安装的扩展板及设备树参数，脚本不会据此修改 GPIO、加载驱动或改写启动配置。当前舵机使用 USB 转串口，因此舵机运行脚本没有 GPIO 控制引脚。

### 编码器规格与计数格式

用户于 2026-11-09 提供了购买记录和机构尺寸：布瑞特磁电断电记忆编码器，购买接口为 RS485，多圈量程 1800 圈，单圈分辨率 12 bit；测量轮接触半径为 10.89 mm，直连编码器轴。随后提供了该设备的 V2.7 官方手册，并明确正向推进时编码器计数应增加。编码器草稿已更新为版本 3。购买名称中的 SSI、CAN 是名称文字，不据此认定当前设备同时支持这些接口；完整型号已由用户确认为 `BRT38-ROM-4096-D1800-RT1-IP68`。

| 字段路径 | 当前值 | 含义与依据 |
|---|---|---|
| `device.brand`、`device.purchase_description` | 布瑞特；保留用户提供的购买名称 | 硬件来源记录，不冒充具体型号 |
| `device.model` | `BRT38-ROM-4096-D1800-RT1-IP68` | 用户提供的完整型号，按原文记录 |
| `device.manual_version` | `V2.7` | 用户提供的官方手册版本 |
| `device.single_turn_resolution_bits` | 12 | 每圈分成 `2¹²` 份，来自购买规格 |
| `device.multi_turn_range_revolutions` | 1800 | 多圈测量量程，单位为编码器轴圈；不是每圈计数 |
| `communication.mode` | `query` | 主站发送一次查询，接收一次响应；对应手册模式 0，脚本不写设备模式寄存器 |
| `communication.crc_byte_order` | `little` | CRC 校验的低字节先发送，与计数数据的字节顺序不同 |
| `count_format.bits` | 32 bit | 两个 16 位寄存器组成总计数，不是每圈分辨率 |
| `count_format.counts_per_revolution` | 4096 count/圈 | 由 12 bit 计算，与厂家说明一致 |
| `count_format.word_order` | `high_word_first` | 先收到高 16 位，再收到低 16 位 |
| `count_format.signed` | `False` | 原始计数为无符号整数；反转仍可使读数减少 |
| `count_format.min_count`、`count_format.max_count` | 0、7,372,799 count | 包含两端的有效计数范围，上限由分辨率和圈数计算 |
| `count_format.count_modulus` | 7,372,800 count | 本轮按用户指定采用 1800 圈循环；未做跨越端点的实测 |
| `mechanics.measurement_wheel_radius_mm` | 10.89 mm | 用户确认的测量轮接触半径 |
| `mechanics.encoder_revolutions_per_wheel_revolution` | 1.0 | 测量轮一圈对应编码器轴一圈，用户确认直连 |
| `mechanics.direction_sign` | +1 | 按用户约定，正向推进时计数增加；这是软件位移换算系数 |
| `mechanics.mm_per_count` | 约 0.01670505 mm/count | 按用户选择，直接由轮径、传动比和分辨率计算理论比例 |
| `mechanics.calibration_id` | `geometry-r10.89mm-g1-n4096-s+1-v1` | 自动记录当前几何参数及方向，不表示实测标定通过 |

本节以用户提供的[布瑞特《RS485 通信协议多圈 V2.7》](<prebuild_docs/3764672290002 RS485说明书通信协议 多圈 V2.7.pdf>)为依据，替代此前查阅的 V2.3。下文页码采用正文印刷页码，PDF 页码比正文多 2 页。正文第 1～2 页列出 12 bit 对应 4096；第 7 页第 5.4.1.1 节说明寄存器 `0x0000～0x0001` 为无符号编码器总计数，示例 `00 01 76 3B` 对应 95803，明确高位在前。该节适用于所有多圈编码器，查询帧与本机已成功使用的 `01 03 00 00 00 02 C4 0B` 一致。

按同节的范围公式及购买规格，标称计数范围为 `0～1800 × 4096 − 1`，即 `0～7,372,799`，已写入 `min_count`、`max_count`。本轮按用户指定将 `count_modulus` 设为 `1800 × 4096 = 7,372,800`，随圈数及分辨率计算；采用循环规则的依据是用户本轮指定，不宣称 V2.7 明确说明了端点行为或已经完成跨界实测。32 位存储空间不表示应按 `2³²` 回绕。量程上限很大也不意味着回绕不会出现：靠近原始零点时反向移动，同样可能越过下端。

`device.model` 已按用户提供的信息填写为 `BRT38-ROM-4096-D1800-RT1-IP68`；其中的 4096、D1800 与现有每圈计数和多圈量程配置一致。型号来源是用户确认，不是套用手册中的通用命名示例。

`mechanics.direction_sign=+1` 表示软件把正计数变化换算成正推进位移。它不会让硬件自动改变计数方向，也不是手册第 5.4.9 节的寄存器取值：硬件寄存器 `0x0009` 的 `0/1` 分别表示从输出轴端观察时顺时针/逆时针递增。现有信息没有规定实际安装后的轴端观察方向，因此不把软件 `+1` 直接写入该寄存器。方向约定来自用户，本次没有追加运动验证。

### V2.7 手册中的通信与测速参数

| 手册内容 | 已确认的信息 | 对当前配置的作用 |
|---|---|---|
| 第 5.1、5.4.5 节：串口 | 8N1，出厂默认 9600 baud，可设置 38400 baud | 保留本机已成功读取的 38400，不恢复出厂速率 |
| 第 5.4.6 节：工作模式 | `0` 查询、`1` 自动回传计数、`5` 自动回传测速值 | 当前采集方式记为 `query`；模式寄存器标为仅写，这不是本次读回值 |
| 第 5.4.2、5.4.3 节：圈数和单圈值 | `0x0002` 为圈数，`0x0003` 为单圈值 | 当前规格分别为 `0～1799`、`0～4095`；位移读取仍使用总计数寄存器，无需拆成两次查询 |
| 第 2 页：内核刷新周期 | 50 μs | 设备内部刷新间隔，不能当作树莓派串口读取周期 |
| 第 5.4.7 节：自动回传周期 | 默认 50 ms；小于 20 ms 时修改其他参数容易失败 | 只适用于自动回传模式，不据此填写查询模式的 `sample_period_s` |
| 第 5.4.10 节：内部角速度采样时间 | `0x000A`，默认 100 ms，可配置，标为仅写 | 测速的统计时间窗，不是通信超时或位置读取周期，也不代表当前设备实值已知 |
| 第 5.4.14 节：角速度值 | `0x0020～0x0021`，32 位有符号整数 | 与无符号总计数分开解析；当前配置仍只读取位置，不启用这条测速路径 |

若后续使用厂家测速寄存器，设其有符号返回值为 `q`、实际配置的测速时间窗为 `T_s` 秒，则 `rpm = 60 × q / (4096 × T_s)`。返回值不是直接的 RPM，不能忽略时间窗，也不能把默认 100 ms 当成已读回的实际设置。V2.7 的无符号“角速度值 2.1”条目使用重叠地址且适用范围写为单圈编码器，不能据此替换本机的多圈位置或有符号测速接口。

手册帧格式表将 CRC 标成高字节在前，但示例帧和已有实机响应均是低字节在前：查询正文 `01 03 00 00 00 02` 算得 CRC 为 `0x0BC4`，发送顺序是 `C4 0B`。配置记录为 `crc_byte_order="little"`，现有协议实现无需修改。CRC 字节顺序和计数数据的高位在前是两回事。

### 编码器采集初值与限速联动

| 字段路径 | 当前值 | 设置依据及适用条件 |
|---|---|---|
| `acquisition.sample_period_s` | 0.02 s，即 50 Hz | 本轮约 2 秒、100 次独立查询成功，20 ms 查询节拍无漏周期 |
| `acquisition.transaction_timeout_s` | 0.05 s | 相对本次最大响应耗时 13.25 ms 留出余量；完整有效回复到达即返回 |
| `acquisition.feedback_max_age_s` | 0.1 s | 用户指定的初值；原采样时刻起超过 100 ms 的数据不再用于控制 |
| `acquisition.quiet_s` | 0.1 s | 用户指定的打开/重连静默等待初值；不是每次查询前等待 100 ms |
| `acquisition.max_speed_mm_s` | 15.0 mm/s，自动派生 | 舵机位移控制限速的 1.5 倍；50% 余量为工程初值，容纳瞬态与测量波动 |
| `estimation.velocity_window_s` | 0.2 s | 仅位置模式使用：窗口内展开位移差除以实际时间差；不是编码器读取周期 |

位置仍随新样本更新，速度使用约 0.2 秒的有界窗口。单计数约为 0.016705 mm，在 20 ms 差分时相当于 0.835 mm/s，容易把静止时单计数抖动误判为超过 0.1 mm/s；窗口估计降低这种量化影响。参考变化、采样断档、无效样本或连接代次变化会清空窗口，重新积累足够跨度前 `linear_velocity_valid=false`，不能把无效速度当作零速。旧双轴估计默认行为不变，手柄模式不使用这一窗口。

`feedback_max_age_s=0.1` 可以作为初始设置，但它不能独立于读取周期判断：正常读取间隔加上通信、调度延迟须小于 0.1 秒，否则会反复把反馈判为过期。因此旧测试采用的 0.2 秒读取间隔不能直接用于这组配置。以当前 10 mm/s 指令上限计算，100 ms 对应 1 mm 行程，已相当于到位位置容差；这个数值是反馈失效的时间界限，不保证位置控制精度，也不表示应每 100 ms 才更新一次位置。

本轮仅进行一次不使用 ROS2 的短时读取，使用现有 `SerialEncoder` 适配器，在 `/dev/ttySC1`、38400 baud、8N1、地址 1 上发送功能码 03 查询。100 次全部得到合法响应，失败 0 次，漏周期 0 次；实际采样约 1.988 秒，含打开、静默等待和关闭共约 2.103 秒。[原始报告](../artifacts/encoder_readonly/timing-7x7x_4_p/report.json)及[逐次记录](../artifacts/encoder_readonly/timing-7x7x_4_p/samples.jsonl)保留在项目中。耗时从查询开始计至完整响应通过 CRC 校验，不包含打开串口、静默等待或周期休眠。

| 本轮测量项 | 结果 |
|---|---:|
| 单次查询最小耗时 | 7.23 ms |
| 单次查询中位数 | 8.03 ms |
| 单次查询 P95 | 10.62 ms |
| 单次查询最大耗时 | 13.25 ms |
| 查询发起间隔最大值 | 20.10 ms |
| 有效反馈接收间隔最大值 | 25.54 ms |

因此读取周期初值采用已在本轮运行的 20 ms；查询超时初值采用 50 ms，约为本次最大耗时的 3.77 倍。正常响应无需等待到超时，仍可按 20 ms 周期查询；若单次响应超过一个周期，则不重叠发起查询，也不密集补发错过的周期。`20 ms + 50 ms < 100 ms` 为现有反馈有效期留出 30 ms 预算，但这不是实际调度延迟的保证。此次采集使用 80 ms 的探测超时，所有实际响应均在 13.25 ms 内完成；50 ms 是依据结果选出的初值，没有为此另开一轮实机测试。短测没有覆盖长期负载、通信故障或 ROS 调度，不把本次最大值当作最坏耗时保证。

`quiet_s=0.1` 可以暂作工程初值。打开或重连时，持续 100 ms 没有收到旧数据才开始查询；期间收到数据则重新计算静默时间。它和反馈有效期用途不同，只是本轮都选用了 0.1 秒，并未在代码中互相引用。手册未给出最晚回复延迟，现有记录也未验证这一延迟小于 100 ms，因此仍保留该项验证 TODO，但不将配置值留空。

编码器脚本通过标准库 `run_path` 读取同目录 `servo_settings.py` 的 `position_settings_payload()`，计算 `acquisition.max_speed_mm_s = 1.5 × motion.max_linear_mm_s`。当前为 `1.5 × 10 = 15 mm/s`；修改舵机位移限速后，重新显示或导出编码器配置就会自动更新，无需两处手工修改。系数 `speed_plausibility_factor=1.5` 集中在编码器脚本中，属于待实际噪声和响应记录检验的初值，不是厂家参数。配置源缺失会直接报错，不使用隐含缺省值。

15 mm/s 是位移闭环的读数合理性阈值，不会把运动指令上限从 10 mm/s 提高到 15 mm/s。它也不使用手柄模式的原生 RPM 上限计算：手柄控制仍不受外部编码器调节。现有估计器的跳变检查还包含两个计数的量化余量，因此这个阈值不是要求每个瞬时速度计算值都严格不超过 15 mm/s。

### 编码器参数填写状态

编码器 `settings_payload()` 返回的配置中已没有 `None` 字段。部分已填写参数另保留工程验证说明，不把“已填写”与“已实测”混为一项。

`None` 表示尚未确认，不能用 0 或 `False` 代替。计数有无符号、计数回绕周期、推进方向是三项不同信息；禁止由其中一项推断另外两项。寄存器返回四字节也不等于每圈有 2³² 个计数。

单个寄存器内部的字节顺序已按 [Modbus 规范第 4.2 节](https://www.modbus.org/file/secure/modbusprotocolspecification.pdf)填写为 `big`。两个寄存器怎样组成一个计数，则依据上述厂家手册填写为 `high_word_first`；这两项分别规定字节顺序和寄存器顺序。

购买规格和厂家说明已支持额定值 4096 count/圈。[一次手动单圈粗测](../artifacts/encoder_readonly/one-turn-w2m1v9vq/assessment.json)得到 4063，较额定值少约 0.81%；没有独立角度参考，该差异不用于修改额定分辨率，也不代表完成了精度标定。旧代码中的编码器轮半径约 10.9783 mm 不再沿用，当前采用用户明确提供的 10.89 mm；1800 圈量程已由购买规格确认，但仍与回绕行为分开记录。

已有两段连续采集共 1221 个有效样本，读取均未报错。两段原始记录的相邻接收间隔中位数分别约为 0.20058 s、0.20060 s，最大值约为 0.20554 s、0.20575 s。这说明目标 0.2 s 的采集周期可以工作，但不能直接作为当前闭环的配置：实际间隔既超过本轮设置的 0.1 s 反馈有效期，也会超过草稿中的 `pid.dt_max_s=0.2 s`。在 10 mm/s 时，0.2 s 间隔对应约 2 mm 行程，也不能单凭采集成功宣称已支持 1 mm 到位要求。原始记录见[手转采集](../artifacts/encoder_readonly/manual-rotation-bb7vmi19/samples.jsonl)及[单圈采集](../artifacts/encoder_readonly/one-turn-w2m1v9vq/samples.jsonl)。

[历史单次查询](../artifacts/encoder_readonly/waveshare-b-setup-8uelsbj4/read-001/report.json)约 0.0093 s 收到完整响应，只证明该次响应耗时。旧采集器的 1 s 是预设等待上限，不是测得的最坏耗时。本轮已根据上节的短时测量将 `transaction_timeout_s` 设为 0.05 s。V2.7 没有给出查询响应的最坏耗时，短测也没有覆盖迟到回复，因此 `quiet_s=0.1 s` 仍是用户指定的工程初值。手册中的最大机械转速 3000 RPM 是编码器自身的额定机械上限，本机构的读数异常阈值采用上节的位移限速及余量计算。

品牌、完整型号、额定规格、测量轮尺寸、传动关系和正向计数约定已获得，不需要重复提供。毫米比例采用用户选择的理论值，循环周期采用用户指定值；软件已为这组几何比例生成 `calibration_id`。读取周期和单次查询超时已依据本轮短测填入初值。

### 几何换算与实测标定的职责

舵机侧令 `R_s = mechanics.drive_wheel_radius_mm`、`G_s = mechanics.shaft_revolutions_per_wheel_revolution`，理想无滑移时的几何推进量为：

\[
c_{geometry}=\frac{2\pi R_s}{G_s}\quad[\mathrm{mm/舵机输出轴圈}]
\]

编码器侧令 `R_e = mechanics.measurement_wheel_radius_mm`、`G_e = mechanics.encoder_revolutions_per_wheel_revolution`、`N_e = count_format.counts_per_revolution`，理想几何比例为：

\[
k_{geometry}=\frac{2\pi R_e}{G_e N_e}\quad[\mathrm{mm/count}]
\]

代入当前编码器参数，测量轮一圈的理论行程为 `2π × 10.89 ≈ 68.42389 mm`，每个计数对应 `68.42389 / 4096 ≈ 0.01670505 mm`，即约 16.705 μm。按用户本轮选择，`mechanics.mm_per_count` 直接采用该计算结果；它随轮径、传动比和分辨率自动更新，不另行手工填写小数。这是无滑动、有效半径不变时的几何换算，不是实测位移精度。

两个传动比都是“轴转数 / 对应轮转数”，不能颠倒。按用户明确选择，舵机的 `mm_per_shaft_revolution` 与编码器的 `mm_per_count` 均直接采用无滑动几何模型，避免手工维护重复数值；轮面形变或滑动会使实际推进量偏离该模型。编码器的 `calibration_id` 包含半径、轴/轮传动比、每圈计数和方向，并以 `geometry` 标明理论来源。若进行独立实测标定，两项比例应分别依据舵机输出轴转数与独立位移、编码器连续计数与独立位移，不能用舵机目标值给编码器标定，再用该编码器结果反证舵机比例。

位置路径使用比例正值 `c = servo.mechanics.mm_per_shaft_revolution` 将 PID 线速度换为舵机 RPM：`rpm = servo.mechanics.direction_sign × 60 × velocity_mm_s / c`。当前 `c` 是几何计算值，不是位移测量结果。编码器完成计数解析与回绕展开后，用 `x = reference_position_mm + encoder.mechanics.direction_sign × encoder.mechanics.mm_per_count × (unwrapped_count − reference_count)` 得到参考坐标下的位置。两处方向分别对应执行器和传感器，不重复叠加手柄的方向系数。

位置参考必须在运行时根据新鲜有效反馈建立；当前寄存器值、某次试验初始值和设备硬件零位都不能自动充当任务坐标零点。后文继续描述原双轴 YAML 配置，不应将其中的仿真默认值直接填入上述草稿。

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

### Jog速度PI、输入质量与有限边沿

这些量与定位PID独立；当前模型值仅支撑无硬件回归，硬件文件必须显式填写并有整定/时序依据。它们不在允许运行中更新的字段集合内，变更需停止确认、DISARM和生命周期重新配置。

| 字段 | 单位 | 模型默认 | 含义 |
|---|---|---:|---|
| jog_velocity_kp | 无量纲 | 0.25 | 外部测量速度误差的比例贡献 |
| jog_velocity_ki | 1/s | 4.0 | 速度积分增益，不复用定位ki |
| jog_velocity_kaw | 1/s | 2.0 | 最终限幅与接受量化残差一次回算 |
| jog_integral_limit_mm_s | mm/s | 10.0 | Jog积分速度贡献上限 |
| jog_velocity_filter_s | s | 0.04 | 新速度样本低通时间常数；0关闭 |
| jog_dt_max_s | s | 0.2 | 真实控制/反馈dt允许预算，超预算停车，不密集追赶 |
| manual_input_max_age_s | s | 0.15 | 接收端输入年龄政策上限，不能替源改写截止 |
| jog_phase_gain_s | 1/s | 8.0 | 有限波形空档的有界相位误差修正 |
| pulse_high_us / pulse_low_us | µs整数 | 5 / 5 | profile每脉冲最小高/低时间 |
| direction_setup_us / direction_hold_us | µs整数 | 10 / 10 | profile方向前/后保护时间 |

推进Jog为 `v_target + Kp*(v_target-v_measured_filtered) + I`；新有效反馈seq和真实dt只积分一次，输出经过速度/加速度/边界再提交抗饱和，接受回执只补额外残差。回中清历史修正，不能把摇杆积分成需追赶的虚拟位置。模型噪声/滑移可帮助回归，实机增益仍需H03。

runtime四个脉冲时序字段须与 `devices.pulse.high_us/low_us/direction_setup_us/direction_hold_us` 一致，工厂不接受两套不同限制。保护时间须能放进有限段，配置的最大脉冲率须满足高低时间和有界profile容量。`edge_offsets_us` 为可选不可变整型µs上升沿表，None兼容原定位调用；给出profile时不回退均匀脉冲。完整段包括保护时间并满足所有原截止。

手柄旋转映射最大值是长期平均意图；段内相位修正可高于意图，但仍受runtime核心角速度/加速度限制。若硬件速度上限没有足够余量抵消单wave提交空档，需降低可达平均意图并报告不足，不能把核心限制放宽为未经验证的数值。段内计划率、段间空档和跨段均速分开评估；H02决定实物适用性。

### 手柄profile与证据文件

安装资源在 `gca_teleop/config/`：`beitong_unverified.yaml` 保留未知映射null/模式空值，`synthetic.yaml`及其JSON只用于simulation。profile主字段为 source、profile_id、mode、device、mapping、mapping_evidence、qualification_file、script_file；未知键/类型、非有限数和越界值拒绝。device包含USB身份、名称、轴/键能力和操作者明确的device_mode，不凭名称自动映射。

| 输入参数 | 合成/未核实模板值 | 解释 |
|---|---:|---|
| deadzone / response_exponent | 0.08 / 1.5 | 左轴各自独立死区与幅度曲线，不使用径向死区制造串扰 |
| lt_press / lt_release | 0.6 / 0.4 | 归一化扳机滞回；LT不乘速度幅度 |
| max_linear_mm_s / max_rotation_deg_s | 2 / 30 | 输入意图范围，不是实机已测安全速度 |
| dispatch_hz | 25 | 容量1最新输入派发；慢许可不无限排队 |
| input_max_age_s / reader_timeout_s | 0.15 / 0.10 | GCA事件有效期与reader存活分开 |
| poll_timeout_s / max_poll_events / max_poll_bytes | 0.02 / 64 / 512 | 有界只读采集与8字节joydev解析预算 |
| timestamp_uncertainty_s / max_epoch_s | 0.020 / 3600 | joydev时间映射不确定度与epoch上限，不把接收时间冒充事件时间 |

映射必须有LX/LY两端/中心、LT两端、B按松以及所有预留键的真实事件记录与摘要。当前证据schema_version=2：读取时依据有界原始全轨迹重新计算端点/中心、所选样本与selection_policy，再核对mapping；摘要用于完整性检查，单凭摘要不代表映射来自原记录。只读元数据和部分采集不够。qualified_state还需匹配设备/驱动/内核/模式/映射的资格记录，合成记录不能用于实体。两种模式及可复现向导见 [手柄指南](TELEOP_USER_GUIDE.md)。修改/切换profile仅在停止CONFIRMED且DISARMED时通过同owner `input-config`，新epoch和释放/回中门控重建，不自动恢复意图。

软件可控性门槛独立版本化在 `system_bringup/config/teleop_acceptance.yaml`（teleop-controllability-v3）。v3保留v1/v2全部23场景及原门槛；v2在原17场景上另预声明执行器死区0.01/0.5mm/s及目标±0.25mm/s；仍按2s后W=0.5s、5%或2计数/W及回中2计数评价。另以1mm/s实际接受上限限制±2意图，1s解除，解除后2s的W=0.5s误差仍≤0.1mm/s，积分贡献回落≤0.1mm/s。动态限制只改变独立plant能力，不改目标/传感器读数；新6场景并入完整23场景报告。v3另固定long_model共同周期30s、首稳态端点30s、每周期2counts/1pulse漂移及累计N倍预算、无失步计数/真值1pulse误差、0.5s最终稳定和两次完整同seed运行。末尾1s零速观察仍用原2counts；数值ULP比较guard与物理预算分开报告，不强制速度Jog位置锁定。它描述合成测试，不是硬件标定文件；更改门槛需预先声明版本、重新验证，不能为某次失败事后放宽。

### 仿真专用参数

| 字段 | 单位/类型 | 默认 | 解释 |
|---|---|---:|---|
| simulation_seed | 整数 | 7 | 固定随机种子，便于复现噪声序列 |
| simulation_noise_mm | mm | 0.0 | >=0；编码器高斯噪声标准差，实际模型见simulation/devices.py |
| simulation_encoder_delay_s | s | 0.0 | >=0；模型采样延迟，原始采样时间不会刷新为到达时刻 |
| simulation_slip | 比例 | 0.0 | 0≤值<1；独立plant的运动传递损失 |

plant的默认一阶时间常数、死区、饱和和每脉冲角度还定义在 `simulation/plant.py`，不是设备标定。仿真只消费实际接受的输出，无权读取任务目标后直接修改测量值；模型结果不能替代真实负载/摩擦/弹性实验。

## 真实舵机配置：devices.servo

当前只支持经核实的 **XM430-W350、model_number=1020、固件≥38**。字段虽能描述其它数字，适配器仍会在写寄存器前拒绝不支持型号。本机型号和通信已有只读证据；机构比例、速度模式和原双轴 HIL 尚未因此通过。以下字段属于旧双轴毫米适配层。

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

构造器的少量技术默认不是硬件验收值。毫米适配层复用原生驱动，普通 configure 遇到 watchdog 锁存时拒绝，不再自动清锁；显式恢复要求力矩关闭且无硬件错误。速度配置还需满足：`60*max_linear_mm_s/(mm_per_revolution*velocity_unit_rpm)`不超过raw限制（考虑半单位量化）。

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

硬件宿主还要求强制DDS安全（ROS_SECURITY_ENABLE=true且ROS_SECURITY_STRATEGY=Enforce），或ROS_LOCALHOST_ONLY=1配合显式trusted_local_control=true的受控本机部署；仅设置发现范围不是认证。三个波形布尔条件均满足才允许真实波形open。单段时长包括方向前后时序，必须容纳全部高低电平时间。真实pigpio在波形准备完成、wave_send触发前，再要求整型µs实际段时长加ceil(socket_timeout_s)的声明发送RPC预算能落入原截止；不足即拒绝，不缩段、不续期。正式runtime始终提供固定expires_ns；底层expires_ns=0只保留旧直接端口调用兼容，不代表正式运动无期限。该预算不是实测RPC/OS/电气启动上界，H02/H03仍需测量。后端不是按elapsed time猜计数；中止未知作废参考。当前没有软件EN输出，也没有自动读取细分拨码。见 [接线说明](HARDWARE_WIRING.md)。

## ROS宿主附加参数

这些不是RuntimeConfig字段，改变需重启/重新构建节点，不能混进runtime映射。

| 参数 | 默认/约束 | 作用 |
|---|---|---|
| config_file / configuration_file | 空字符串；前者优先 | 完整YAML路径，实际推荐launch的config_file |
| servo_config_file | 仅舵机入口必填 | 独立脚本生成的 JSON；设备与手柄同路径、同摘要，变更后重启 |
| coordinator_key | 启动时由launch共享的随机能力 | 设备管理授权，不写日志，不当作网络认证替代物 |
| trusted_local_control | false | 需同时ROS_LOCALHOST_ONLY=1；仅受控本机硬件部署，须核实用户/设备权限 |
| record_directory | 空字符串 | 记录输出目录；标准仿真launch会分配本次运行目录 |
| executor_threads | 默认3，正整数，由MultiThreadedExecutor校验 | ROS回调线程数，不代表控制采样率 |
| telemetry_hz | 10.0，0.5–100 | 遥测发布率，与PID有效采样率独立 |

不要用参数dump向不受控日志复制coordinator_key。查询公开控制参数可用真实命令 `ros2 param get /gca_control_node kp`。

协调器 `/system_coordinator` 还读取下列启动参数。它们控制源会话和下游调用，不是设备配置，也不能在线延长已发出的单次运动许可。

| 参数 | 默认/约束 | 作用 |
|---|---|---|
| profile | gca_only 或 servo | 分别为原双轴与仅舵机入口；均无磁场实验执行器 |
| coordinator_key | 默认空值拒绝启动；由launch与设备节点共享随机值 | 领取设备执行许可所需的能力，不能写入记录或当作DDS身份认证 |
| source_lease_max_s | 5.0 s；有限且≥0.1 s | 源控制会话的最大租约；每次申请仍须在0.1 s至该上限之间 |
| executor_lease_s | 2.0 s；有限且0.1–5.0 s | 下游执行许可的单次期限，同时不得超过源会话剩余期限 |
| handover_timeout_s | 2.0 s | ARM代次交接确认期限；超时停车/撤权，不默认成功 |
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
