# 设备 SDK 与协议迁移

本工程把旧实验脚本迁移为具有显式生命周期、统一领域端口和可注入依赖的设备适配器。正式运行入口为 ROS 2；设备适配器本身不依赖 ROS。当前完成了软件实现和离线测试，尚未进行实机接线、波形、方向或停止距离验收。

## 依赖与安装

从工程根目录运行 `bash tools/bootstrap_dev.sh`。补充依赖安装到项目 `.venv`，不在导入时启动串口、pigpio daemon 或 GPIO。实际加载版本和系统依赖见 `artifacts/environment.json`；固定补充依赖见 `requirements-dev.txt`。

| 依赖 | 版本 | 用途与上游 |
|---|---|---|
| dynamixel-sdk | 4.1.0 | [ROBOTIS 官方 Python SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python)，串口及 Protocol 2 CRC/字节填充 |
| pigpio | 1.78 | [官方 Python 客户端](https://github.com/joan2937/pigpio)，连接独立波形服务；安装客户端不等于 daemon 可用 |
| pyserial | 3.5 | [官方串口库](https://github.com/pyserial/pyserial)，编码器有界事务 |

DYNAMIXEL 和 pigpio 胶合代码针对上述版本审查；加载其它版本会明确拒绝，不静默使用未经审查 API。第三方许可与旧代码来源边界见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 旧代码到新版模块

源码位于 `ros2_ws/src/gca_core/gca/`；测试位于相邻 `test/`。

| 旧功能／来源 | 新模块 | 修复与依据 | 主要回归 |
|---|---|---|---|
| `use_PIDController/motor.py` 手写舵机帧、阻塞到位 | `devices/dynamixel.py`、`devices/sdk_support.py` | B01/B06/B18；使用官方 SDK，输出是受限速度，设备适配器不等待到位 | `test_devices_dynamixel.py`、`test_runtime_control.py` |
| `steering_func.Initialize` 盲试串口、自动运动到 0 | `DynamixelServo.open/configure/enable` | B10/B18；身份、禁用配置、ARM使能分离 | `test_devices_dynamixel.py` 的构造、身份及配置失败用例 |
| `encoder_func` CRC/读取/计数解释 | `protocols/encoder.py`、`devices/encoder.py` | B03–B05；精确分帧、固定两字节 CRC、真实采样序号、显式计数格式 | `test_protocol_encoder.py`、`test_devices_encoder.py` |
| `stepper_func`、`DM542/DMdriver` 逐脉冲 sleep | `devices/pigpio_engine.py` | B07–B09；有限定时段，接受／运行／完成／中止分离 | `test_devices_pigpio.py` 的完成、中止及期限用例 |
| 各目录复制 SDK、型号与协议表混合 | `devices/factory.py`、固定依赖 | B20；新包单一导入路径，旧目录不进入运行时 | `test_devices_factory.py`、`test_devices_dependencies.py`及普通安装验证 |
| 端口占用查杀、导入时运动 | 显式生命周期与 `DeviceLock` | B16/B17；冲突明确失败，不杀进程，不自动启用 | `test_devices_pigpio.py` 的进程锁用例、工厂无I/O用例 |

步进迁移是 **PUL/DIR 波形后端迁移**，旧目录未提供可确认的厂家步进 SDK。新版没有虚构 DM542 API，也没有把 RPi.GPIO 的安装当成定时能力证明。

## 公共端口与资源所有权

共同类型见 `gca/domain/models.py`，端口见 `gca/domain/ports.py`。每个设备仅由其运行时 I/O 所有者调用；多线程不得共享一个适配器执行事务。

- `ServoPort`：`open → configure → enable(True)`，随后 `write_velocity/read_state/stop`，最后 `enable(False)/close`。速度为 mm/s，deadline 为本地绝对单调纳秒。
- `EncoderPort`：`open → read(deadline_ns) → close`。返回原始计数、seq、设备epoch、请求开始与接收时间；位置比例和软件零点在上层估计模块处理。
- `PulseEngine`：`open → submit/poll/abort/is_idle → close`。submit 只接受一个有限段，未结算前不接受下一段；中止计数可以未知。

构造对象不连接设备。`open` 是显式硬件入口；本轮离线测试只将工厂替换为测试传输，没有打开实际设备。真实工厂 `hardware_devices(config, clock)` 只装配三个真实适配器，不连接，也不回退模拟。

设备适配器通过 `DeviceLock` 获取进程级 advisory flock。锁文件按用户与设备资源散列命名，文件关闭释放锁，不删除其它进程的锁或杀进程。串口使用规范路径；波形按服务端口取得保守的本机进程锁。此锁约束遵循本工程约定的进程，无法替代设备节点权限、受控账户和 daemon 访问限制。

## DYNAMIXEL 的实际行为

当前只支持官方控制表中 model_number=1020 的 XM430-W350，固件必须为v38或更新版本；其它型号即使可读ID也在写寄存器前拒绝。型号编号依据[官方e-Manual源文件](https://raw.githubusercontent.com/ROBOTIS-GIT/emanual/master/docs/en/dxl/x/xm430-w350.md)，watchdog固件门槛依据[官方说明](https://raw.githubusercontent.com/ROBOTIS-GIT/emanual/master/_includes/en/dxl/control_table_buswatchdog.md)。当前地址语义为该型号 Protocol 2 速度控制表：模式11、速度限制44、扭矩64、watchdog98、目标速度104、实测速度128。连接时读取型号0与固件6，并与配置型号比较；其它型号不得仅因能打开串口就假定兼容。官方语义参见 [XM430-W350 控制表](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-description)。

配置依次关闭并核对扭矩、显式清除可能锁存的watchdog错误、写并核对速度模式、限制、非零watchdog及零速目标。生产配置要求watchdog_raw为1–127，不允许0关闭；清错误时临时写0是受限配置事务，不是关闭保护的运行模式。每一步失败都保持不可 ARM；使能前再次核对零速。重连产生新 epoch，不恢复旧目标。`stop` 只发送有期限的零速需求；调用成功、零速目标读回均不等于实测停止。

`DeadlinePort` 用单调时钟替换 SDK 默认时间源，接收超时同时受绝对事务期限约束；写入设置 pyserial `write_timeout`。没有给官方 SDK 函数杜撰 `deadline` 参数，也没有用一个外层 future 掩盖无限等待。串口打开涉及操作系统设备驱动，仍须在 HIL 测量异常打开/关闭行为，Python 不能承诺操作系统冻结下的硬实时保证。

`CheckedPacketHandler` 在官方 `txRxPacket` 返回后、便利读取函数取数据前核对期望 ID、状态指令和完整参数长度，CRC及字节填充仍由官方 SDK 负责。这样可阻止旧的合法 CRC 短状态帧被误解为位置数据。实际 SDK 离线测试覆盖无参数状态帧 `ff ff fd 00 01 04 00 55 00 a1 0c`，包括一字节读请求也不能取到 CRC 字节。

## 编码器事务和失效

协议实现只提供配置后的功能03读取，不隐藏波特率写入、硬件清零或方向改变。CRC始终两字节小端；黄金请求 `01 03 00 24 00 02 84 00` 覆盖高字节零。完整帧校验地址、功能码、字节数、异常响应、长度与CRC后才解码。

每条串口至多一个请求在途。Modbus式响应没有事务ID，无法仅凭包内容证明迟到帧属于当前请求。出错后适配器要求显式重连；重连先清空输入并等待配置的连续静默窗口，窗口内有输入则重新计时，整段重同步也有总期限。`quiet_s` 必须来自最大迟到响应的设备实验，不能把默认等待值当证据。发现请求前已有字节、两份响应或尾部数据，明确失败，不重标为新采样。

`CountFormat` 明确位宽、字节序、有符号解释和可选模数。旧900／1800圈规则已移除。设备身份若无可读身份寄存器，只声明 `configured_identity_only`，不会声称已经硬件鉴别编码器型号。

## Pigpio 的完成与中止证据

`PigpioPulseEngine` 生成一次有限波形：方向建立、N个有效上升沿和规定高低时间、方向保持，全部纳入段时长。GPIO 使用 BCM 0–31。段时长和脉冲数有上限，没有 repeat、forever 或无限硬件队列。

只有以下条件全部满足，才按完整段结算：配置有对应 HIL 记录、daemon波形资源受限且独占、本机进程锁有效、连接epoch稳定、曾观察匹配wave_id正在运行、随后正常结束且没有中止或错误。`wave_send_once` 的返回数不是边沿数；单看 busy=false、请求数或经过时间均不累计。未观察到运行生命周期的短段明确报告估计无效；应调整段长/轮询预算后重新测量，不能凭时间猜测。

活动段 `abort` 不可知道已输出多少边沿，因此返回 `PulseAbort(None, idle)`，上层作废角度估计和相关参考。完整段 `poll` 只结算一次；机械角度始终是开环估计，不证明没有失步。重连、外部波形替换、调用失败和无法确认停止均不能沿用可信计数。

官方 pigpio 1.78 的普通构造器会创建无期限socket和通知线程。波形专用工厂初始化其已固定版本的 socket holder，使用带期限的连接与读写，随后调用官方公共波形方法；不建立无用通知线程，不提供 GPIO callback 功能。该私有初始化兼容性必须随版本升级重审。适配器没有自动启动 daemon；daemon生命周期、服务账户与独占访问由部署和HIL记录确认。

## 离线验收和仍待实物的事项

从工程根运行统一 `bash tools/verify_software.sh`。设备专项测试是 `test_devices_*.py` 与 `test_protocol_encoder.py`，覆盖真实适配器的注入 SDK／串口／波形路径、配置步骤失败、短帧、超时、负速度、重复段、竞争锁和未知中止。

软件测试使用测试专用 `verified=True` 配置，不代表真实装置已验证。实际接线、型号/固件兼容、细分、供电、电平、标定、波形/取消延迟和机械停止距离仍为 NOT_RUN，见 [接线说明](HARDWARE_WIRING.md) 和 [HIL 验收](HIL_ACCEPTANCE.md)。

当前pigpio后端仅支持本机：host只能为localhost或127.0.0.1，连接前统一为数值127.0.0.1并使用AF_INET socket，不经过DNS/getaddrinfo；远程主机、其它数值IP和IPv6明确拒绝。这样连接期限不被DNS解析绕过；远程波形后端需要单独审查后扩展。
