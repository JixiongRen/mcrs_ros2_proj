# 工程实现状态与证据

**无硬件工程交付完成，最终统一软件验收通过。** 2026-11-05 14:14（Asia/Shanghai）完成：200项软件测试全部通过，0失败/错误/跳过；colcon包级177项全部通过（与200项重叠，不累加）；五包构建、独立普通安装、源码树外入口检查、5个真实ROS仿真场景及5个确定性模型场景均通过。命令、退出码和逐用例证据见下表。该结果不包含实机验收，也不消除下文已记录的调度限制。

本轮执行完整工程goal，设计原文保持在 [prebuild_docs](prebuild_docs/README.md)。所有已执行开发测试使用模拟设备或注入SDK／串口／波形客户端；没有打开真实串口、设置GPIO、启动实际波形或运动。**H01–H03全部NOT_RUN。**

## 五阶段交付状态

| 阶段 | 实际已落地内容 | 当前验证/边界 |
|---|---|---|
| 1 SDK／设备迁移 | DYNAMIXEL、编码器、有限波形真实适配器与工厂，严格帧解析、显式生命周期、锁、失败路径及注入测试 | 设备/协议73项通过，包含在最终200项中；不是实机适配验收 |
| 2 ROS 2／控制算法 | 已编译IDL、gca_core、gca_ros、协调器、bringup；本地I/O所有者、轨迹/PID、状态机、Action与控制权 | 控制/模型77项、ROS/权限26项及其它集成测试通过；五包普通安装及实际ROS入口通过 |
| 3 无实物闭环 | 独立plant、模拟执行器/编码器、故障注入、真实DDS测试、JSONL/CSV及离线曲线工具 | basic/cancel/jog/hold/faults全部PASS；确定性模型5场景PASS；原始记录和图表已生成 |
| 4 接线说明 | [接线](HARDWARE_WIRING.md)、[配置标定](CONFIGURATION_AND_CALIBRATION.md)、[HIL](HIL_ACCEPTANCE.md)、不可运行硬件模板 | 来源与未知项明确；没有实物验证，不能直接据候选图上电 |
| 5 说明与交付 | [文档导航](README.md)、架构/SDK/开发/用户/配置/HIL/许可、统一工具、本文追踪矩阵 | 三个统一入口均已实际执行；最终verify全部9个步骤退出0；文档本地链接已核对 |

环境基线已重新检查：Pi 4B／ARM64／Ubuntu24.04／Python3.12／ROS2 Jazzy。项目使用`.venv`和经PyPI哈希核对的固定pip wheel引导，补充依赖包括DYNAMIXEL SDK4.1.0、pigpio1.78及ruff；系统Python与legacy不改写。实际版本见 [环境记录](../artifacts/environment.json)。导入库不代表打开硬件。

## 已有开发证据与统一验收入口

| 证据 | 位置/实际结果 | 如何解释 |
|---|---|---|
| 开发单元测试快照 | [development-unit.xml](../artifacts/development-unit.xml)：当时116测试、0失败/错误/跳过 | 早期源码快照；后续新增测试以最终JUnit为准 |
| 最终设备专项 | `test_devices_*.py`、`test_protocol_encoder.py`：73项通过 | 已安装真实适配器的注入测试，含真正官方SDK短帧路径；属于最终200项 |
| 最终核心专项 | 算法、plant、估计、runtime：77项通过 | 属于最终200项；不与包级结果重复累加 |
| ROS basic开发流程 | [development-basic-4/basic/report.json](../artifacts/simulation/development-basic-4/basic/report.json)：PASS；最终推进5.01mm、旋转估计19.985625deg，目标5mm/20deg | 实际ROS进程、DDS与模拟plant；不是机械精度实测 |
| 其它开发场景 | [development-scenarios/report.json](../artifacts/simulation/development-scenarios/report.json)：cancel/jog/hold/faults均PASS | 有CLI和启动退出码、DDS记录、曲线；仍是development报告 |
| 历史ROS调试 | [development-ros.xml](../artifacts/development-ros.xml)保留早期失败快照 | 历史失败不删除；不能用它代表修复后的最终状态，也不冒充全通过 |
| 最终统一验证 | [verification/latest/report.json](../artifacts/verification/latest/report.json)、[pytest.xml](../artifacts/verification/latest/pytest.xml) | passed=true；全部9步退出0；200项0失败/错误/跳过，pytest用时126.60s |
| colcon包级结果 | [colcon-results.log](../artifacts/verification/latest/colcon-results.log) | 177项，0失败/错误/跳过；包含真实bringup失败传播回归 |
| 最终仿真套件 | [simulation/latest/report.json](../artifacts/simulation/latest/report.json)及各场景子目录 | 5个ROS场景及5个模型场景全部PASS；各CLI和launch均退出0 |
| 独立普通安装 | `artifacts/clean-install/20261105T060700711551Z/install`、[资源检查日志](../artifacts/verification/latest/installed-resources.log) | 普通构建退出0，用时369.40s；源码树外只加载该install，五包、CLI、launch和配置资源均PASS |

最终源码指纹为 `4b007c29b3226d9ef2a9e5654fd95db9d6b0f275948574022768828fc01128b2`，逐文件哈希见 [sources.json](../artifacts/verification/latest/sources.json)。统一工具已检查验收期间源码未变化。basic场景目标为5 mm/20°，最终推进反馈5.01 mm、旋转脉冲估计19.985625°；这是模型运行结果，不是实机精度。主入口均从工程根运行：

```bash
bash tools/bootstrap_dev.sh
bash tools/verify_software.sh
bash tools/run_simulation_suite.sh --output artifacts/simulation/latest
```

`verify_software.sh`串行构建、静态检查、必需pytest、colcon结果、独立普通安装、安装资源和完整仿真；每步保留命令、退出码与日志。源码专项验证可用于快速修复，整体完成仍须通过该统一路径。

## 追踪表读取方式

以下别名均指实际源码和测试；表中引用为具体函数或明确文件测试集合，最终执行状态由JUnit关联，不凭文件存在判PASS。

| 别名 | 文件/目录 |
|---|---|
| D | `ros2_ws/src/gca_core/gca/devices/` |
| P | `ros2_ws/src/gca_core/gca/protocols/` |
| R | `ros2_ws/src/gca_core/test/test_runtime_control.py` |
| C | `ros2_ws/src/gca_core/test/test_control_math.py` |
| S | `ros2_ws/src/gca_core/test/test_simulation_plant.py` |
| DD | `ros2_ws/src/gca_core/test/test_devices_dynamixel.py` |
| DE | `ros2_ws/src/gca_core/test/test_devices_encoder.py` |
| DP | `ros2_ws/src/gca_core/test/test_devices_pigpio.py` |
| DF | `ros2_ws/src/gca_core/test/test_devices_factory.py` |
| DX | `ros2_ws/src/gca_core/test/test_devices_dependencies.py` |
| PE | `ros2_ws/src/gca_core/test/test_protocol_encoder.py` |
| I | `tests/test_ros_integration.py`，实际ROS进程/DDS |
| O | `tests/test_observability.py` |
| SEC | `ros2_ws/src/gca_ros/test/test_security.py`与`test_permits.py` |
| AUTH | `ros2_ws/src/system_coordinator/test/test_authority.py` |

## B01–B20：旧问题到实现及回归

本表表示修复代码和回归已建立；物理效果涉及的H项保持NOT_RUN，最新软件全套状态仍由最终报告决定。

| 编号/旧问题 | 实现位置与修复行为 | 具体回归证据 | 实机边界 |
|---|---|---|---|
| B01 阻塞move_pos | D/dynamixel.py速度执行，runtime/control.py本地状态推进 | R::test_unarmed_rejected_and_full_closed_loop_uses_plant、test_owner_deadlines_stop_when_control_task_frozen、test_cancel_waits_for_stop_confirmation | H03停止时间待测 |
| B02 dt/积分饱和 | algorithms/control.py真实dt、有限积分、最终输出/量化残差一次回算 | C::test_pid_true_dt_and_quantization_residual_settled_once、test_p_only_does_not_create_hidden_integral_from_limiting、test_saturation_reversal_and_reset | 新增益不继承为实机参数 |
| B03 同轮重复采集 | D/encoder.py一次采集一个seq，estimation/linear.py不可变有效快照 | DE::test_open_is_read_only_and_each_valid_reply_has_new_sequence；S::test_estimation_repeated_samples_and_first_velocity_invalid | H01内部更新率待测 |
| B04 CRC宽度 | P/encoder.py固定两字节小端 | PE::test_crc_golden_width_and_standard_vector，覆盖84 00 | 黄金报文软件通过 |
| B05 编码器帧/符号 | 严格长度、站号/功能码/异常/CRC、请求关联及失败后重同步 | PE::test_malformed_never_becomes_measurement、test_exception_and_buffer_limit_are_explicit、test_signed_count_and_no_invented_wrap_rule；DE::test_bad_or_incomplete_response_requires_resync、test_unsolicited_old_reply_is_not_new_feedback | 未知真实协议禁用 |
| B06 SDK短帧假数值 | D/sdk_support.py在官方便利函数解码前核对完整参数长度 | DD::test_official_sdk_checked_packet_rejects_legal_crc_without_data、test_limits_timeout_and_invalid_measurement_are_explicit | 真实SDK离线执行，不是复制其实现做预期 |
| B07 步进内层不退出 | 独立停止标志、有限段、明确abort和有期限关闭 | R::test_cancel_waits_for_stop_confirmation；DP::test_abort_never_invents_partial_count | H02尾部脉冲待测 |
| B08 sleep/假stop | D/pigpio_engine.py有限硬件定时波形、明确idle/abort | DP::test_finite_wave_timing_and_confirm_only_after_observed_completion、test_abort_never_invents_partial_count | GPIO时序/机械余运动H02/H03 |
| B09 估计计数/截断 | 绝对目标舍入、预约/确认分离、回执绑定一次结算 | C::test_absolute_pulse_rounding_preserves_small_moves；R::test_pulse_receipts_are_bound_to_segment_and_settled_only_once、test_wrong_epoch_or_payload_cannot_settle_pulse；DP::test_short_unobserved_wave_cannot_invent_completion | 仍是开环角估计 |
| B10 启动自动移动/清零 | open/configure/enable分离、身份核对、编码器普通生命周期只读 | DD::test_construct_and_open_do_not_enable_or_configure、test_identity_failure_closes_port；DE::test_open_is_read_only_and_each_valid_reply_has_new_sequence、test_reconnect_changes_epoch_and_resets_sequence | 实物遗留输出H01 |
| B11 TCP空包/部分准入 | 正式入口替换为有类型ROS Action、完整目标原子准入 | R::test_both_axis_validation_prevents_partial_motion；I::test_source_conflict_and_atomic_two_axis_rejection、test_active_is_not_arm_and_direct_device_token_cannot_be_source_token | 不保留旧TCP运动服务器兼容 |
| B12 控制权/无界队列/去重 | source/device授权、短期许可、代次与序号高水位、停止独立容量 | R::test_relative_command_retries_do_not_move_twice、test_same_id_changed_payload_conflicts、test_highwater_survives_bounded_result_eviction、test_saturated_command_queue_cannot_block_stop_latch、test_critical_io_overflow_latches_fault_and_invalidates_count_evidence；SEC/AUTH | 能力字符串不替代DDS/本机权限 |
| B13 退出/deadman | 有期限停止先于关闭；zero和独立torque-disable分别尝试；Jog期限 | R::test_jog_deadman_and_short_ttl_stop、test_cleanup_failure_cannot_report_successful_close、test_unknown_stop_is_failed_even_if_zero_write_was_accepted；DD::test_failed_zero_still_attempts_independent_torque_disable；I::test_lifecycle_deactivate_stops_active_motion_without_ros_wait_deadlock | 实体手柄/线路拔出H03，接口测试不冒充实物适配 |
| B14 -1哨兵/假到位 | 显式任务状态、合法负目标、轨迹+有效速度+驻留 | I::test_single_axis_negative_target_and_relative_roundtrip；R::test_unarmed_rejected_and_full_closed_loop_uses_plant；S::test_estimation_repeated_samples_and_first_velocity_invalid | 0.08/0.15是模型阈值 |
| B15 日志无限增长 | observability/recorder.py有界异步轮转、丢样/错误可见；runtime有限记录环 | O全部；R::test_bounded_record_ring_reports_eviction；I::test_committed_success_cannot_be_canceled_and_slow_telemetry_does_not_block_control | 真机负载H03 |
| B16 自动杀占用进程 | 删除旧查杀代码，D/common.py资源flock，部署冲突明确失败 | DP::test_process_ownership_lock_prevents_second_client；静态审查新版无端口任意查杀路径 | 测试证明本工程锁冲突，不夸大第三方独占保证 |
| B17 import硬件副作用/持锁I/O | 构造无设备操作、lazy库、单I/O所有者 | DF::test_full_hardware_factory_constructs_all_real_adapters_without_io；DX全部；gca_ros/test/test_node_constructor.py::test_lifecycle_host_constructs_without_device_io | 默认无GPIO/SDK也可导入核心/仿真 |
| B18 模式未核对/None/重复读 | D/dynamixel.py逐步核对、稳定错误、每次采样一次读取 | DD::test_every_configuration_failure_prevents_enable、test_wrong_readback_refuses_enable、test_old_firmware_has_no_watchdog_capability_and_never_enables、test_watchdog_cannot_be_disabled_in_hardware_configuration | XM430-W350 model1020/fw≥38白名单；实物仍待确认 |
| B19 硬编码/无限位 | 完整RuntimeConfig及独立硬件schema、制动边界、必填物理值 | C::test_boundary_braking_includes_delay_and_output_dt、test_strict_configuration_rejects_ambiguous_values；DF::test_missing_physical_field_cannot_inherit_simulation、test_one_flag_cannot_enable_missing_hardware、test_schema_rejects_boolean_gpio_and_string_number、test_protocol_and_estimator_modulus_must_match | hardware.template的null故意拒绝ARM；H01/H03 |
| B20 多SDK/协议混用 | 单一安装包和固定官方依赖、明确型号与能力拒绝 | DD::test_unknown_model_never_connects_or_writes；DX全部；tools/installed_smoke.py | 普通安装结果见最终统一报告 |

## T01–T18：验证场景到实际测试

| 用例 | 实际测试入口 | 当前证据与范围 |
|---|---|---|
| T01 控制与到位 | C全部；R::test_unarmed_rejected_and_full_closed_loop_uses_plant；I::test_single_axis_negative_target_and_relative_roundtrip | 数学/独立plant已开发测试；最新整套见最终JUnit |
| T02 数据一致性 | DE全部；S::test_estimation_repeated_samples_and_first_velocity_invalid、test_encoder_preserves_frozen_sample_seq_and_age | 有效新采样、首次速度无效、冻结年龄不刷新 |
| T03 编码器协议 | PE全部、DE失败/迟到测试 | CRC、分片/粘连、异常码、位宽/符号/回绕及重同步 |
| T04 舵机协议与错误 | DD全部、DX缺依赖/SDK时限测试 | 已安装官方SDK短帧路径、注入读写/配置/错误 |
| T05 取消与退出 | R::test_cancel_waits_for_stop_confirmation、test_cleanup_failure_cannot_report_successful_close；I::test_competing_goals_and_immediate_cancel_have_one_terminal | 停止证据、未知失败、关闭失败不报成功 |
| T06 脉冲生命周期 | DP全部；S::test_pulse_acceptance_is_not_completion_and_abort_is_unknown；R::test_pulse_receipts_are_bound_to_segment_and_settled_only_once | 接受/确认、重复/迟到回执、中止未知 |
| T07 启动与参考 | DD生命周期/配置失败、DE重连、gca_ros构造测试 | 无自动运动/清零；设备epoch/参考/旧命令失效 |
| T08 输出监督 | R::test_owner_deadlines_stop_when_control_task_frozen、test_feedback_freeze_stops_and_does_not_renew_cached_sample；I::test_control_freeze_does_not_prevent_independent_io_stop | 设备所有者独立期限；不以遥测存活当控制健康 |
| T09 命令准入 | R::test_both_axis_validation_prevents_partial_motion、test_unselected_axis_zero_limits_are_allowed；I负位置/原子拒绝；C严格schema | 只校验所选轴，完整目标一次接受 |
| T10 会话与幂等 | R重复/冲突/结果淘汰/撤销代次测试；SEC/AUTH | 高水位及短期许可不因缓存淘汰或续租复活旧目标 |
| T11 队列与访问 | R::test_saturated_command_queue_cannot_block_stop_latch、test_critical_io_overflow_latches_fault_and_invalidates_count_evidence；I直接入口拒绝；SEC | 可控满队列、关键事件溢出及直接权限测试均通过，见最终JUnit |
| T12 观测负载 | O全部；R::test_bounded_record_ring_reports_eviction；I::test_committed_success_cannot_be_canceled_and_slow_telemetry_does_not_block_control | 慢/满盘、轮转、凭证/大小边界；新慢telemetry隔离测试结果见最终统一报告 |
| T13 部署冲突 | DP::test_process_ownership_lock_prevents_second_client；新版无任意杀进程路径审查 | 本工程资源锁失败可见；不声称能禁止任意第三方直接访问daemon |
| T14 包与副作用 | DX/DF、gca_ros构造测试；tools/installed_smoke.py | 普通安装、源码树外CLI/launch/配置及完整仿真均已实际通过 |
| T15 配置与限位 | C边界/严格schema；R::test_configuration_transaction_rejects_structural_change、test_config_cannot_claim_new_queue_capacity_was_applied；gca_ros/test/test_conversion.py | SI边界、结构配置拒绝、半更新版本禁止ARM |
| T16 Action竞态 | I::test_competing_goals_and_immediate_cancel_have_one_terminal、test_cancel_wins_over_deferred_hold_candidate、test_committed_success_cannot_be_canceled_and_slow_telemetry_does_not_block_control；R::test_cancel_before_start_clears_reservation | 两种受控终态交错窗口已写，**执行结果以最终统一报告为准**，不把一次顺利取消代替全部竞态 |
| T17 Lifecycle | I::test_lifecycle_deactivate_stops_active_motion_without_ros_wait_deadlock、test_active_is_not_arm_and_direct_device_token_cannot_be_source_token；R inactive/config测试 | active≠ARM，deactivate不等待自身ROS回调 |
| T18 HOLD与联动 | I::test_hold_source_expiry_stops_and_does_not_auto_rearm；R::test_hold_has_independent_context_and_feedback_expiry_exits | Action结果后保持上下文仍受租约/反馈监督；完整Windows联动属后续范围 |

纯控制算法不读取目标来伪造plant状态。S::test_plant_only_moves_after_accepted_output_and_has_lag、test_open_loop_can_miss_steps_without_fake_feedback分别验证推进滞后和开环失步模型；机械角真值只用于仿真评估，不回送为不存在的真实反馈。

## 多agent交叉审查记录

下列问题已经实现修正并建立回归/检查，不再列作当前未解决阻断：

- 真实适配器：官方SDK短帧CRC混入、未支持型号/旧固件误用、watchdog关闭、zero失败跳过torque-disable、编码器协议与展开模数不一致。
- 运行时：提交当刻重新检查整个脉冲段许可窗口；段ID/epoch/代次/载荷绑定与完成/abort去重；异常停止和清理失败不能仍报告成功。
- ROS并发：Action预留/Future顺序、取消与成功唯一终态、取消候选HOLD交接、生命周期独立门控及非选择轴检查。
- 工具与日志：停止指标使用IN_PROGRESS；非零退出/清理升级不能PASS；本次进程组清理包含已退出父进程留下的子进程；异常中断清理；递归拒绝凭证、记录大小不超文件上限。
- 覆盖补充：T11满队列和关键事件溢出、T12慢telemetry隔离、T16两种受控终态交错用例已经落地，执行状态仍以最终JUnit为准。

分工为设备/协议、控制/模型、ROS/协调器及主agent集成；互相复审停止、并发、独立plant与证据工具。agent口头通过不能代替命令结果。历史失败报告保留，用最终源码及最终报告说明解决状态。

### 调度观测与明确限制

一次完整测试重跑在遥测阻塞场景出现约278 ms的控制/采样共同调度暂停，触发 `FEEDBACK_STALE`，随后确认停止。现场及199通过/1失败结果保存在 [历史失败证据](../artifacts/verification/development-telemetry-failure/report.json)。三次原期限定点复测均通过，控制记录最大间隔分别约23.37、20.55、22.37 ms，见 [诊断汇总](../artifacts/diagnostics/telemetry-scheduler/summary.json)；**未复现不等于已修复，暂停来源仍未确定**。该记录不支持硬实时或最坏停止时间保证。

本地输出TTL与过期反馈停车规则未放宽。当前Python/普通Linux部署在调度暂停时可能使任务失败并停车；实际负载下的运行连续性和停止性能仍需H03测量。后来另修正了原始控制记录中反馈年龄/租约剩余时间未更新的问题：日志与其单调时间戳对应，ROS快照仍在读取时重算年龄；冻结反馈回归核对日志时间差。集成测试在失败时也释放自身注入的遥测暂停，避免清理警告掩盖主要故障。

## H01–H03与尚待实物的具体资料

| 验收 | 状态 | 必须补齐的资料/测量 |
|---|---|---|
| H01 身份与低速标定 | NOT_RUN | 舵机真实型号/后缀/固件/TTL或RS-485、转换器和供电；编码器型号/电气接口/寄存器/响应迟到/符号回绕；方向、轮径比例、多点往返与载荷；实际参考方式 |
| H02 波形与实际角度 | NOT_RUN | 步进驱动型号、电平/光耦/端子、6400/25600细分冲突、传动比；高低/DIR时序、完整有效沿数、取消尾段；daemon平台/独占访问；外部机械角度与失步 |
| H03 停止与性能 | NOT_RUN | 已存在的禁止/急停回路、扭矩/使能失电行为；正常/断线/进程故障下停止时间与距离；最坏载荷误差/超调/抖动；预先约定性能阈值 |

硬件模板保持null/false；缺资料不能伪造默认值，也不阻止可无硬件完成的软件工作。当前旋转没有机械角反馈，软件无独立EN输出，也未验证实体手柄拔出；不能把接口或仿真行为扩大成这些实物能力。

## 交付范围与后续工作

本轮必需的软件构建、安装、测试、仿真及文档工作均完成。历史失败保留，最终通过不代表已确定所有宿主调度暂停的原因；上文调度限制仍有效。之后修改源码、配置或依赖，应重新运行统一验证并形成新的指纹和报告。

后续需按实物资料完成H01–H03，确认实际设备、接线、比例、波形和停止性能后再启用hardware。Windows磁驱动、联合实验、实体手柄适配、跨机DDS权限策略、视觉/力反馈等仍是明确的扩展范围，当前GCA-only不会代替这些能力。
