# 手柄缺陷、实现与验证追踪

> 当前491项软件测试与436项包级测试通过；追加12项STOP竞态回归已纳入。进一步整套安装/仿真复验已按用户要求暂停，下面455项与9个DDS/23个模型的整套PASS描述属于此前快照，不能视作新增修复后的完整验收。

本表追踪本轮实际代码与测试，完整模型采用teleop-controllability-v3（原17场景+6个死区/饱和恢复场景，原门槛不放宽，另补双run长周期/neutral及计数真值检查），不用“文件存在”判定整体验收通过。本轮 [统一验收](../artifacts/verification/latest/report.json) 已通过：455项软件测试0失败/错误/跳过，六包普通安装与源码树外入口检查通过；[手柄完整套件](../artifacts/teleop/latest/report.json)9个真实ROS/DDS场景和[23个独立模型场景](../artifacts/teleop/latest/model/junit.xml)均通过。下表测试已纳入当前源码与报告，开发快照见 `artifacts/teleop_implementation/`。未执行的实体资格和HIL保持NOT_RUN。

## JF01–JF07

| 编号 | 实际修复源码 | 回归证据（精确测试名） |
|---|---|---|
| JF01 外部速度反馈与抗饱和 | `gca/algorithms/jog_velocity.py`；`runtime/control.py::_advance_motion/_output_velocity/_consume_io` | `test_control_teleop.py::test_velocity_pi_tracks_external_plant_slip_and_neutral_has_no_position_debt`、`test_pi_actual_feedback_dt_once_only_receipt_and_antiwindup`、`test_duplicate_feedback_does_not_integrate_velocity_pi_twice` |
| JF02 细动量化/相位丢失 | `algorithms/jog_rotation.py`；`runtime/control.py::_plan_jog_pulse`，planned/reserved/confirmed分离 | `test_subpulse_rotation_survives_many_segments_within_one_edge`、`test_reverse_discards_old_whole_pulse_debt_but_keeps_bounded_deceleration`；`test_jog_rotation_quantized_candidate_respects_boundary_reference`、`test_jog_rotation_unsettled_reservation_blocks_new_plan_even_when_engine_idle`；既有runtime段ID/代次/回执去重测试保留 |
| JF03 首入/再入初态 | `runtime/control.py::_handle_jog/_begin_stop`；速度与旋转planner reset | `test_first_input_and_reentry_start_with_real_dt_not_old_timestamp`、`test_jog_reentry_after_long_idle_has_fresh_initial_budget` |
| JF04 有限非均匀边沿与跨段率 | `domain/models.py::PulseSegment`；`algorithms/pulse_timing.py`；`simulation/devices.py::SimPulseEngine`；`devices/pigpio_engine.py::_wave` | `test_edge_time_oracle_matches_integrated_limited_phase_with_microsecond_budget`、`test_constant_rotation_does_not_lose_wave_handoff_time`、`test_simulated_profile_consumes_only_actual_rising_edges_not_uniform_time_fraction`；`test_devices_pigpio.py::test_nonuniform_edge_profile_is_emitted_exactly_and_never_uniformized`、`test_invalid_nonuniform_profiles_never_prepare_or_send`；半脉冲 `test_runtime_half_phase_from_rest_produces_ordered_edges_with_independent_oracle`、`test_half_phase_exact_start_is_earliest_crossing` |
| JF05 STOP、许可与ARM竞态 | core `stop/_latch_stop/_tick`与`io_owners.py`最终事务闸门；`gca_ros/node.py`；`system_coordinator/node.py`；teleop `dispatch.py/session.py/management.py` | core `test_stop_rejects_already_queued_jog_and_arm_without_fifo_dependency`、`test_stop_during_quality_validation_cannot_reopen_output_gate`；runtime `test_stop_wins_final_velocity_transaction_gate_before_sdk_write`、`test_stop_wins_final_pulse_transaction_gate_before_finite_submit`；设备/协调器 `test_*_stop_handover.py` 与teleop `test_dispatch.py/test_session_stop.py/test_management.py/test_cli_stop.py` |
| JF06 输入原截止贯穿I/O | `gca/domain/manual.py::QualityTracker`；各层Jog准入、dispatch固定deadline、velocity demand与PulseSegment.expires_ns、真实send前检查 | core `test_missing_quality_and_fixed_source_expiry_fail_closed_with_control_paused`；teleop `test_original_input_deadline_is_not_extended_by_slow_permit`；`test_manual_quality.py`、设备send前期限测试；真实DDS control_freeze场景 |
| JF07 未知中止锁定与显式恢复 | core段计数/参考有效性；teleop固定双轴mask、门控和同owner管理代理 | `test_unknown_abort_invalidates_rotation_reference_and_locks_axis_mask3`、`test_same_owner_explicit_reference_and_new_arm_restores_after_unknown_abort`；`test_jt04_jt05_stop_never_invents_release_or_resumes_held_lt` |

core文件路径前缀为 `ros2_ws/src/gca_core/gca/`；`test_control_teleop.py`、`test_runtime_control.py`、`test_devices_pigpio.py` 位于该包 `test/`。JF04的“加速度”指有限profile的受限相位/导数和整数µs时序；段间空档单列评估，不证明机械连续加速度。角度/角速度为开环估计，不能凭计数发现真实失步。

正、负旋转边界与未结算预约共4项补充回归已单独实际通过，见 [reservation_tests.xml](../artifacts/teleop_implementation/core/reservation_tests.xml)；验证候选边沿越限时无新预约/输出，以及真实模拟波形已结束但完成回执未消费时仍阻止再次规划。专项结果另存保留；这4项也包含在本次455项完整软件验收中。

## JT01–JT20

| 用例 | 主要实际源码/测试入口 | 证据边界 |
|---|---|---|
| JT01 只读joydev/失败关闭 | `gca_teleop/input/joydev.py`；`test_input_joydev.py`中的`test_jt01_*`、身份/权限测试 | 注入端口覆盖INIT、短读、回绕、积压、重同步、身份/权限；真实RF能力不由这些测试证明 |
| JT02 独立左右轴映射 | `mapping.py`；`test_input_pipeline.py::test_jt02_*` | 单调、奇对称、死区、斜向无串扰；实体15步只读映射schema2重算已PASS，见下方实体证据；不等于执行器物理方向验收 |
| JT03 右侧预留 | `mapping.py/state_machine.py`；`test_jt03_reserved_controls_do_not_change_intent_or_real_event_age` | 右侧既不改变GCA意图，也不刷新旧事件期限 |
| JT04 LT/B/回中 | `state_machine.py`；`test_jt04_*`及`test_jt04_jt05_*` | B优先，回中零速度，B/LT释放不自动恢复 |
| JT05 初态/重连/重门控 | `state_machine.py`；`test_jt05_*` | 释放/回中→新LT、epoch与ARM/参考变化、reader冻结 |
| JT06 event_age | `state_machine.py`；`test_jt06_deadzone_noise_does_not_renew_age`、`test_jt06_static_event_age_expires_despite_reader_progress` | 静态长保持到期是已定义行为，不用心跳/右侧/重复缓存续命 |
| JT07 qualified_state | `config.py/probe.py/state_machine.py`；`test_jt07_*`、qualification与hardware ARM前置测试 | 软件资格夹具为合成；实体资格仍NOT_RUN |
| JT08 质量/程序episode | `gca/domain/manual.py`；`test_manual_quality.py`；`test_jt08_*`；旧`GcaClient.jog` | boot/时间/序号/epoch与固定episode，保持旧CLI调用方式 |
| JT09 慢许可与容量1 | `dispatch.py`；`test_latest_slot_does_not_starve_valid_inflight_when_observation_or_renewal_advances`、`test_original_input_deadline_is_not_extended_by_slow_permit` | 不无限排队，不因25Hz新帧使有效在途请求永久饥饿 |
| JT10 STOP与派发/I/O交错 | `test_dispatch.py`；core STOP/quality/最终write和submit闸门回归；DDS b_stop | 屏障固定窗口，核对旧seq无新输出 |
| JT11 ARM交接与独立STOP | `gca_ros/test/test_device_stop_handover.py`、`system_coordinator/test/test_coordinator_stop_handover.py`；teleop `test_session_stop.py/test_management.py`；DDS slow_arm | 许可跨代别名、ACK/超时、迟到使能和STOP独立socket |
| JT12 控制冻结 | core原截止测试、`simulation_evidence`只读测试；DDS control_freeze | 依据模拟实际接受输出/plant与原期限，不凭遥测存活判运动授权 |
| JT13 推进速度可控性 | `test_control_teleop.py`、`simulation/teleop_report.py` | ±2、slip0/10/25%、噪声/延迟，以及`test_jt13_independent_plant_deadband_crossing_uses_feedback`的±.25/死区.01/.5；独立plant速度 |
| JT14 初始化/边界/饱和 | core首入、长空闲、反向/限位、重复反馈与PI抗饱和测试；JF02的正负软限位/未结算预约回归 | 增加`test_jt14_actual_actuator_saturation_then_release_unwinds_velocity_pi`和report saturation_release：cap1/1s后解除、活动目标下误差与积分恢复。真实dt；紧急撤销不套普通斜坡 |
| JT15 低速/换向量化 | core细动、phase余量/换向回归和model rotation场景 | .1deg/s×10s非零、积分误差≤1脉冲；planned不冒充confirmed |
| JT16 有限边沿/整段期限 | profile独立相位oracle、真实pigpio注入测试、accepted_profiles.jsonl | 不回退均匀脉冲；真实电气/拼接及机械效果H02 NOT_RUN |
| JT17 回中/未知中止/恢复 | core neutral/unknown abort/same-owner restore；输入新LT门控；DDS diagonal_reverse/b_stop | 双轴固定mask；显式参考确认、ARM，不静默降级 |
| JT18 退出/负载/未知 | teleop reader/dispatch/session/IPC关闭测试；既有runtime/recorder/ROS退出测试；`tests/test_verification_cleanup.py`；各DDS场景退出码和清理 | 验证工具清理本次拥有的独立子进程组，并检查PID/starttime身份；正常父进程退出但遗留子组也判失败。慢/坏IPC有界，不能把UNKNOWN当成功 |
| JT19 重复性/长期采样 | `test_fixed_seed_repeats_model_trace_without_wall_clock_or_uuid_dependence`；model long_sampling | v3双1800s已实际通过：各1801.5s观察、18018行，.1s采样hash一致，30s周期漂移/累计量化预算、无失步plant计数及末尾neutral检查通过；开发双60s测试`test_v3_two_short_periodic_runs_validate_recorded_hash_drift_and_truth`，不冒充完整时长；非Pi硬实时性能 |
| JT20 普通安装与旧回归 | `tools/verify_software.sh`、`installed_smoke.py`、`teleop_suite.py`及原B/T必测集合 | 普通安装后源码树外入口/配置、完整ROS与模型；子集开发通过不替代统一验收 |

验证工具的独立子组清理补充回归共7项已实际通过（0失败/错误/跳过，5.797s），见 [verification-cleanup.xml](../artifacts/teleop_implementation/verification-cleanup.xml)。真实短进程覆盖正常无残留、父进程提前零/非零退出但遗留子组、timeout和KeyboardInterrupt，并核对无关进程存活；另有PID复用身份拒绝及截断扫描保留已知身份的受控测试。这7项也已纳入本轮完整统一验收，当前总结果见主报告。

## 当前实体证据

只读身份记录见 `artifacts/teleop_implementation/input/current_identity.json`；15步实际采集位于 `artifacts/teleop_implementation/input/physical_mapping/`。最终schema2映射重算与摘要复核 [validation](../artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.validation.json) 为PASS，对应 [profile](../artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.profile.yaml) 和 [原始证据](../artifacts/teleop_implementation/input/physical_mapping/mapping.schema2.json)。LX0/LY1反向、LT5、B1；预留axes[2,3,4]/buttons[7,14]，按钮8/9未分配。

操作者明确表示模式不清楚，记录标签`operator_reported_unknown_2.4GHz`，不认定XInput/DInput；无线资格NOT_RUN。实体输入→simulation的 [首启失败](../artifacts/teleop_implementation/input/physical_simulation.attempt1/report.json) 未开始motion：schema2重算耗时超过reader预算。后续 [实际尝试报告](../artifacts/teleop_implementation/input/physical_simulation/report.json) 已结束，passed=false / NOT_PASSED_NO_MOTION_OBSERVED；保留输入raw的maxevent_seq=0/changed_frames=0，完整DDS两个stage接受/编码器速度与脉冲均0。无新事件原因未确定，reader存活不是radio在线证明；STOP/release与launch退出0、errors=[]仅证明本次清理完成。输入轮转可能淘汰早期raw，完整DDS observations保留。只读映射PASS不能代替实体运动链路通过，更不能替代GCA H01–H03。pending或候选文件不按文件名自动更新PASS。

之后的 [LT初始状态快照](../artifacts/teleop_implementation/input/lt_held_snapshot.json) 记录重新打开设备时LT轴5为+32767、按钮8为1，event_seq=0且changed_axes/changed_buttons为空。该快照只确认初始化基线，不是新一次LT按下事件，也不改变上一轮实体运动未通过的结论。

第三轮 [实体输入→simulation报告](../artifacts/teleop_implementation/input/physical_simulation.attempt3/report.json) 为 `passed=false / NOT_PASSED_OPERATOR_ACTION_UNCONFIRMED`。十分钟窗口到期时尚未收到LT完成回复，仅2个INIT基线且0个变化事件，DDS接受输出/编码器速度/确认脉冲均0；`OPERATOR_WINDOW_EXPIRED_NO_ASSUMED_ACTION` 明确不推定用户动作。STOP确认、release成功、launch退出0，该轮模拟ROS节点已关闭。后续 [live_lt_probe](../artifacts/teleop_implementation/input/live_lt_probe/report.json) 已关闭，只有4个LX变化、无LT变化。

重启后的 [LT只读报告](../artifacts/teleop_implementation/input/live_lt_probe_restart_1/report.json) 也已关闭，共123个变化记录：LT实按+32767后实放至−15540，归一化0.262871<释放阈值0.4，仅按下/释放阈值跨越通过；该次按下后全释放端点未观测。该probe不测试运动，也不验证无线资格。

第四轮 [实体→simulation报告](../artifacts/teleop_implementation/input/physical_simulation.attempt4/report.json) 已关闭，[分阶段评估](../artifacts/teleop_implementation/input/physical_simulation.attempt4/assessment.json) 保留具体未通过原因，两个推进stage的接受/编码器速度和确认脉冲均0，`passed=false`。16个基线/变化帧完整保留，changed/critical丢弃计数均0。第一stage的LT新按seq4与首个LY变化的reader观察间隔160.866216ms，event_age已过期；第二stage虽有用户完成回复，但窗口内0真实变化。STOP/release成功、launch0、errors=[]是清理证据，不能转为运动PASS。

[原始joydev与NativeJoydev只读对照](../artifacts/teleop_implementation/input/parallel_input_probe.attempt2/report.json) 已关闭，[评估](../artifacts/teleop_implementation/input/parallel_input_probe.attempt2/assessment.json) 确认raw/native各16个变化事件按序一致，包含B索引1的多次按放，errors=[]且native_health_failures=0。用户确认B按放完成，灯状态仍NOT_REPORTED；event4/5权限不可读如实跳过、未改权限。它不测试运动或无线资格。第五轮也已关闭，见[分阶段评估](../artifacts/teleop_implementation/input/physical_simulation.attempt5/assessment.json)：短暂模拟推进已观测到0.05mm编码器位移；两次旋转试验最高30°/s意图但确认脉冲始终为0，整体不判通过。650个基线/变化记录、关键DDS观察未丢弃，清理成功。无线资格与H01–H03保持NOT_RUN。

## 独立终端诊断与时间快照修复

`tools/joystick_monitor.py`和安装入口`gca_joystick_monitor`复用`NativeJoydev`、`map_input`、`TeleopPipeline`，不加载ROS或设备输出驱动。`test_monitor.py`覆盖原始状态与过期意图区分、B优先、右侧隔离、断连等待与重连门控、慢stdout和有限退出；`tests/test_joystick_monitor_entry.py`在隔离Python子进程中禁止ROS/串口/GPIO库导入，检验源码入口独立可运行。实际只读短运行记录见[monitor live](../artifacts/joystick_monitor/live_final/report.json)，只证明打开设备和基线读取，未要求用户按键，不能替代动作验收。

`node.py::_tick`原先在获得输入锁前记录时间。若输入线程随后更新心跳，旧时间会将新心跳误判为“来自未来”并触发READER_STALE。现在在输入锁内紧邻状态机读取采样时间，仍在意图计算前捕获STOP guard。`test_tick_clock_snapshot.py`受控安排guard期间/获锁前两种心跳推进；[修复前后报告](../artifacts/joystick_monitor/tick_clock/report.json)保存旧代码2失败和修复后通过证据，并继续拒绝真正未来和过期心跳。这证明一处误判竞态，不代表过去所有停车或高CPU均由它导致。

最终交叉复审另确认了协调器认证与提交之间的竞态。`system_coordinator/node.py::_fence_stop`现在在更新停车代次和待发槽的同一把锁内重新核验STOP权限；`_jog`最终提交也重新核验owner和屏障。停车工作以提交时固定的session、generation及唯一工作身份隔离，旧RPC不抑制新owner或新代次的STOP，旧finally只清理自己的工作。新增12项确定性回归位于`test_coordinator_stop_handover.py`：`test_authenticated_old_stop_cannot_mutate_replacement_owner`、`test_stop_fence_revalidation_preserves_bounded_existing_stop_authority`、`test_previous_owner_slow_deadman_stop_cannot_block_or_clear_new_owner_stop`、`test_jog_final_mailbox_commit_rechecks_owner_and_stop_barrier`及其参数化用例。[修复前后证据](../artifacts/joystick_monitor/stop_auth_race/report.json)保留9个旧窗口失败和随后同owner新代次的失败；最终协调器与设备STOP专项34项全部通过。当前owner的前代STOP别名与无新owner时受限退休STOP仍保留，未扩大其renew/ARM/reference权限。
