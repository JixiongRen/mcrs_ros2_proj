# 仿真、故障注入与证据

正式仿真启动两个真实 ROS 2 进程，CLI 经实际 DDS 的 Lifecycle、Service、Action 和 Topic 操作。替换的是设备与机械模型，rclpy 没有被 mock。另有固定模型时钟的算法报告；两者分开标记，避免把模型时间误当 Pi 的墙钟性能。

## 运行

```bash
cd /home/pi/mcrs_ros2_proj
bash tools/verify_software.sh
bash tools/run_simulation_suite.sh --output artifacts/simulation/latest
```

单场景调试可加 `--scenarios basic`，但不代替完整验证。工具使用独立测试 domain，记录本次命令与参数，从临时工作目录调用安装后的 launch／CLI；完成后只清理本次创建的进程组。客户端失败、节点异常退出或必须强制终止都会使场景失败。

| ROS 场景 | 必须观察到的行为 |
|---|---|
| basic | 未ARM拒绝，显式参考／ARM后到5 mm及20°，STOP与终态 |
| cancel | 动作执行中原生取消，CANCELED且停止确认 |
| jog | 持续新输入时运动，deadman释放／输入到期后停止 |
| hold | 成功后outputs_held，独立保持上下文，随后STOP |
| faults | 推进反馈断开导致故障锁存，停止、清故障与显式恢复 |

`tests/test_ros_integration.py` 增补直接设备接口授权、会话冲突、单轴与负目标、并发准入、deactivate、源过期、控制冻结、受控取消／成功交错和慢遥测隔离。纯算法／设备／权限测试与真实进程测试互补，不能只跑顺利动作。

## 独立模型

`MechanicalPlant` 只接收实际被模拟执行器接受的速度 u 和输出脉冲，不读取命令目标或参考轨迹。推进模型为 `dv/dt=(u_eff-v)/tau`；对固定输入精确积分速度和位移，`u_eff` 包含饱和、死区和滑移。仿真默认 tau=0.04 s、执行器速度饱和30 mm/s、死区0.01 mm/s；这些数值不是机械标定。

编码器独立按原始采样时刻读取模型位置，含量化、固定种子噪声和可配置延迟；控制器只读该反馈。重复 seq 不重复更新速度或积分，第一帧及长间隔后的速度无效。外部参考是坐标变换，不能回写 plant 位置来假造到位。

旋转同时保存目标量化脉冲、预约、完成回执和独立机械角度。机械角度仅作评估；正式控制器没有使用真实系统不存在的角度传感器。注入失步可让估计角度与模型机械角度分离，软件不会凭输出脉冲自动发现所有失步。

## 固定模型时钟报告

完整 simulation suite 也执行 `tools/model_report.py`，输出到 `model/`：正常推进、25%滑移、零速API成功但模型仍运动、正常旋转、20%失步。复用正式轨迹、控制器、估计器、量化器及模拟设备，独立模型时钟按整数步推进；不会把这层报告标成完整 ROS 路径验证。

```bash
source tools/common.sh
gca_source_install
"$GCA_PYTHON" tools/model_report.py --output artifacts/model
```

模型报告显式记录 `clock=model_time`、seed=7、步长、参数与 `formal_ROS_path_tested_here=false`。原始 CSV／JSON 能重复生成；图表 PNG 可离线审阅。零速失败场景的 PASS 表示成功检测到“写零成功不代表已停”这一预期异常，不表示执行器停止了。

## 如何读报告

| 文件 | 内容 |
|---|---|
| `report.json` | 全套是否通过、配置哈希、版本、各场景退出码及模型报告 |
| `<scenario>/client/report.json` | 安装后CLI的Action结果与续租状态 |
| `<scenario>/states.jsonl` | 独立订阅器的完整ROS状态，含wall_s和boot_id |
| `<scenario>/states.csv` | 同一记录中的标量字段，便于离线分析 |
| `<scenario>/curves.png`／SVG | 目标／轨迹／反馈、请求／接受输出、旋转估计 |
| `<scenario>/launch.log`／`client.log` | 节点启动、调用与退出诊断 |
| `<scenario>/ros_logs/gca-records-*` | 本地控制高频记录，有限队列和轮转 |
| `model/summary.json` | 固定模型时间下的原始假设与场景指标 |

真实 ROS basic/HOLD 场景使用模拟阈值：最终推进误差≤0.12 mm、20°脉冲估计误差≤0.04°；后者包含6400脉冲／电机圈、17/30传动比对应的量化。模型推进报告使用自己的10 mm目标、误差≤0.1 mm、超调≤0.5 mm门槛。不同目标与时钟不混用指标。

`dds_observed_stop_response_s` 是订阅器第一次看见 IN_PROGRESS 到 CONFIRMED 的间隔，受发布周期与调度影响；它不是从物理故障发生起测得的最坏停止上界。模型报告的reached_at_model_s也不代表ROS进程耗时。

## 故障覆盖与限制

设备测试注入坏CRC、短帧、异常码、迟到响应、超时、写失败、错误读回、身份／固件错误、未知波形中止。核心测试覆盖反馈冻结／跳变、控制冻结但I/O继续、租约到期、旧epoch／代次／回执、队列满、已撤销权限、停止未知、部分清理失败。观测测试用事件屏障控制慢盘，避免依赖偶然调度；真实进程测试用实际墙钟期限。

`terminal_pause` 和 `telemetry_pause` 是 simulation-only 的桥接测试钩子：前者固定结果提交窗口，后者模拟ROS观测回调阻塞；它们不出现在硬件运行能力中。原始编码器冻结保持原seq和接收时间，不用不断发相同数值冒充新数据。

有限波形完整生命周期可以提供条件成立时的脉冲计数证据；中止无法知道部分边沿数则返回未知。模型无法证明 GPIO 电气兼容、真实波形连续性、导管滑移、真实角度、停止距离或整机控制精度。H01–H03 当前均 NOT_RUN，见 [HIL验收](HIL_ACCEPTANCE.md)。
