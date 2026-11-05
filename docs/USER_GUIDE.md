# 无硬件操作指南

以下步骤使用安装后的入口和模拟设备。普通 launch 不会 ARM、设参考或开始运动；演示命令和交互 shell 会按操作者明确输入的命令执行模拟动作。

## 从新终端开始

先从项目根目录完成 `bash tools/bootstrap_dev.sh` 和 `bash tools/verify_software.sh`。每个操作终端执行：

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
gca_source_install
```

终端 A 启动节点：

```bash
ros2 launch system_bringup simulation.launch.py
```

终端 B 打开交互客户端：

```bash
ros2 run gca_ros gca_cli shell
```

shell 维持一个有期限且持续续租的控制会话。独立运行 `claim` 后再逐条启动新 CLI 进程，容易超过短租约，不适合手工操作。

## 一次完整操作

在 `gca>` 提示符内依次输入：

```text
configure
activate
claim MANUAL
reference 0 0
arm
move 5 20
state
move -1 -5 relative
state
jog 2 10 0.5
stop
disarm
release
quit
```

`reference` 的单位为 mm 和 deg，表示把当前位置赋给指定的软件参考，不驱动机构回原点，也不改编码器 EEPROM。只在静止且 DISARMED 时设置参考。参考完成后 ARM 才允许执行；active 本身不授权运动。

`move 5 20` 是绝对目标；`relative` 相对目标只在准入时转换一次。成功结果包含 `outcome=1`、最终位置／角度估计以及 `stop_confirmed=true`。旋转数值是已确认脉冲的换算，不能用它证明实际机械角度。`jog` 在所给时段内发送新鲜输入，结束后发送 deadman 释放；输入停止或续租失败会使旧速度到期。

业务状态：DISARMED → READY → EXECUTING → READY。请求停止时可见 REQUESTED／IN_PROGRESS，确认后变为 CONFIRMED。出现 FAULT 或 UNKNOWN 时先读错误和停止证据，不重复盲发运动。

## 保持、取消与单轴动作

shell 中 `move 5 20 hold` 成功后进入 HOLDING，`outputs_held=true` 表示保持仍受反馈和会话期限监督。输入 `stop` 结束保持，`disarm` 撤销使能。离开 shell 会先请求停止／DISARM，再释放会话；关闭输入后不能一直续旧 Jog。

标准演示会自行领取并释放会话，因此不要与已有 shell 同时占控制权。它们适合验证具体行为：

```bash
ros2 run gca_ros gca_cli demo --scenario cancel --output artifacts/manual-cancel
ros2 run gca_ros gca_cli demo --scenario hold --output artifacts/manual-hold
ros2 run gca_ros gca_cli demo --scenario jog --output artifacts/manual-jog
ros2 run gca_ros gca_cli demo --scenario faults --output artifacts/manual-faults
```

Python 客户端 `gca_ros.cli.GcaClient` 支持更完整的参数；所有调用仍经过协调器。下列程序只针对已启动的 simulation：

```python
import rclpy
from gca_ros.cli import GcaClient

rclpy.init()
client = GcaClient()
try:
    client.configure()
    client.activate()
    client.claim("AUTO")
    client.manage("set_reference")
    client.wait_state(lambda s: s.linear_reference_valid and s.rotation_reference_valid)
    client.manage("arm")
    client.wait_state(lambda s: s.armed)
    print(client.move(axis_mask=1, linear_mm=-1, rotation_deg=0))
    print(client.move(axis_mask=2, linear_mm=0, rotation_deg=10))
    print(client.move(linear_mm=30, rotation_deg=90, cancel_after=0.2))
    client.manage("disarm")
    client.wait_state(lambda s: not s.armed and s.stop.progress == "CONFIRMED")
finally:
    client.close()
    rclpy.shutdown()
```

`axis_mask=1` 选推进、2 选旋转、3 同时选两轴。单轴动作只要求该轴必要反馈，未选执行器仍须保持停止；例如旋转可不依赖推进编码器的新样本，但舵机输出失控仍触发整机停止。双轴目标一次准入，一轴非法不会只让另一轴执行。

## 故障与恢复

simulation shell 中可以输入 `fault encoder_disconnect`，再输入 `state` 查看质量变化。只有活动动作需要推进反馈时，它才因编码器丢失被中止。清除注入用 `fault encoder_disconnect off`。等待停止确认后 `reset`，重新 `reference`、`arm`，再发新任务；RESET 不复活旧目标。

`UNKNOWN` 表示现有证据不能确认停止或脉冲结算，不能直接当作“已停”。模拟用例会检验这条路径；实机如何隔离输出和恢复必须按 HIL 文档的设备能力执行。

## 状态、记录与报告

```bash
ros2 run gca_ros gca_cli status
ros2 topic echo /gca/state
ros2 topic echo /diagnostics
bash tools/run_simulation_suite.sh --output artifacts/simulation/latest
```

状态含 boot_id、原始反馈年龄、sample_seq、有效性、参考版本、脉冲估计质量、配置版本和停止证据。诊断还包含日志丢样／错误。`record_directory` 指定本地控制 JSONL 目录；launch 默认放到本次 ROS 日志目录下独立的 `gca-records-*`，文件有轮转上限。仿真报告的 `states.jsonl`／CSV 来自实际 DDS 订阅，`curves.png` 可离线查看。

## 常见失败

| 现象 | 检查和处理 |
|---|---|
| 找不到包或 executable | 新终端先 source common.sh 与 gca_source_install；修改后重新 build |
| Service unavailable／发现超时 | 两终端 ROS_DOMAIN_ID 一致，默认本机发现；查看 launch.log 是否某节点已退出 |
| 未 ARM／NOT_ACTIVE | configure → activate → claim → reference → ARM；查看 lifecycle_active 和 armed 两项 |
| CONTROL_BUSY | 先释放已有客户端；不要用第二个客户端绕过会话 |
| SESSION_EXPIRED／INVALID_TOKEN | 重新领取控制权；不要重用持久化旧 token 或旧 generation |
| 参考无效 | 确认静止、数据质量与配置，DISARMED 下显式 reference |
| CONFIG_REVISION_MISMATCH | 比较 requested 与 applied；处理拒绝原因，结构变更需要清理重配 |
| PROTOCOL／DEVICE_TIMEOUT | 查看设备错误；真实模式核对型号与接口，不能把失败值当 0 |
| STOP_TIMEOUT／UNKNOWN | 证据不足；保留日志，按既定停止／隔离与验收步骤处理 |
| 状态看似不更新 | 检查 QoS、boot_id、seq 与 feedback_age；旧快照不能作为运动许可 |

真实硬件配置不使用这份仿真流程的参数。先读 [接线](HARDWARE_WIRING.md)、[配置与标定](CONFIGURATION_AND_CALIBRATION.md) 和 [HIL](HIL_ACCEPTANCE.md)。硬件 launch 要求明确 YAML；本机可信部署需显式 `trusted_local_control:=true`，否则要求 Enforce DDS Security。应用 token 不验证原生 CancelGoal 调用者身份。
