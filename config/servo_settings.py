#!/usr/bin/env python3
"""Single editable source of native servo settings; never opens hardware.

Run without ROS: python3 config/servo_settings.py show
Export: python3 config/servo_settings.py export OUTPUT.json --backend simulation
Future position parameters: python3 config/servo_settings.py show --section position

The position section is a preparation document, not a runtime servo snapshot.
Unknown values remain None until confirmed; exporting it does not enable PID.

Identity values were rechecked in servo_ros2/implementation-xce7i_e4/readonly.
Speed raw=349 gives a 79.921 rpm command ceiling. This is not a motor rating:
XM430-W350 no-load speed is 46 rpm at 12 V and 57 rpm at 14.8 V.
Timing/acceleration values below are explicit engineering candidates: the native feedback block took
about 28–32 ms per READ in three read-only samples at 57600 baud.
They must be measured during real-chain acceptance, not labelled HIL PASS.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from math import pi
from pathlib import Path

# The local schema also uses only the standard library.  This makes export
# independent of ROS installation, a virtual environment and package building.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/gca_core"))
from gca.domain.position import resolve_braking_parameters  # noqa: E402
from gca.servo_config import make_servo_settings, servo_settings_document  # noqa: E402


def settings_payload(backend: str, *, manual_input_mode: str = "qualified_state"):
    return {
        "schema_version": 2,
        "config_revision": "servo-held-v5"
        if manual_input_mode == "joydev_hold"
        else "servo-manual-v5",
        "backend": backend,
        "device": {
            "port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5WJAPT-if00-port0",
            "baudrate": 57600,
            "device_id": 1,
            "model_number": 1020,
            "velocity_unit_rpm": 0.229,
            "max_velocity_raw": 349,
            "watchdog_raw": 25,
            "profile_acceleration_raw": 12,
            "transaction_timeout_s": 0.08,
        },
        "manual_axis_to_servo_sign": -1,
        "max_acceleration_rpm_s": 40.0,
        "control_period_s": 0.04,
        "io_period_s": 0.02,
        "feedback_max_age_s": 0.25,
        # A write and feedback read each have an 80 ms transaction budget.
        # Allow control/ROS scheduling as well; never renew an issued deadline.
        "output_ttl_s": 0.30,
        # At about 45 rpm, fresh zero + dwell took 1.28 s after LT release.
        # This bounds confirmation only; STOP still revokes output immediately.
        "stop_timeout_s": 2.0,
        "settle_s": 0.10,
        "velocity_tolerance_rpm": 0.229,
        "lease_max_s": 5.0,
        "manual_input_max_age_s": 0.30,
        "manual_input_mode": manual_input_mode,
        "queue_capacity": 64,
        "history_capacity": 256,
        "mechanical_mapping": None,
        "position_pid": None,
    }


def position_settings_payload():
    """位移闭环参数草稿：几何比例及 PID 初值，当前手柄控制不读取。"""
    # 用户确认：驱动轮半径 6 mm、输出轴直驱；比例按无滑动前提计算。
    drive_wheel_radius_mm = 6.0
    shaft_revolutions_per_wheel_revolution = 1.0
    document = {
        "kind": "servo_position_draft",
        "schema_version": 2,
        "mechanics": {
            # 有效接触半径，单位 mm；修改上方定义即可更新下方几何比例。
            "drive_wheel_radius_mm": drive_wheel_radius_mm,
            # 舵机输出轴转数 / 驱动轮转数，不再计入舵机内部减速比。
            "shaft_revolutions_per_wheel_revolution": shaft_revolutions_per_wheel_revolution,
            # 无滑动时的几何计算值，单位 mm/输出轴圈，约 37.6991。
            # TODO: 实验核对轮面形变及滑动影响；此值不是独立实测结果。
            "mm_per_shaft_revolution": (
                2 * pi * drive_wheel_radius_mm / shaft_revolutions_per_wheel_revolution
            ),
            # 用户确认现有上推转向为推进正方向；该转向对应舵机负 RPM。
            # 这是位置坐标到 RPM 的转换，不修改现有手柄方向系数。
            "direction_sign": -1,
            # 参数依据的版本标识；geometry 表示无滑动几何模型，非实测标定。
            # TODO: 半径、传动关系或标定依据变化时更新标识。
            "calibration_id": "geometry-r6mm-direct-v1",
        },
        "pid": {
            # 输出采用现有 PositionController 约定：v_ref + kp*e + I - kd*v。
            # 以下为待实验整定的初值，不是已验证的快速/无超调参数。
            # TODO: 比例增益初值 2 /s；1 mm 误差产生 2 mm/s 的比例速度请求。
            "kp": 2.0,
            # TODO: 积分增益，单位 1/s²；初期关闭，稳态误差实测后再整定。
            "ki": 0.0,
            # TODO: 测量速度阻尼增益，无量纲；初期关闭，先确认噪声及延迟。
            "kd": 0.0,
            # TODO: 抗积分饱和跟踪增益初值 2 /s；ki=0 时不参与计算。
            "kaw": 2.0,
            # TODO: 积分项速度贡献上限初值 1 mm/s；ki=0 时积分项保持为零。
            "integral_limit_mm_s": 1.0,
            # TODO: 相邻有效样本间隔上限初值 0.2 s，非期望采样周期。
            # 须在接入时与编码器采样周期、反馈年龄及实际通信延迟共同确认。
            "dt_max_s": 0.2,
        },
        "motion": {
            # 以推进为正；运行时建立位置参考，不能把当前计数硬编码为零点。
            # 用户指定：固定零点后方 50 mm；不是每次指令的后退距离。
            "linear_min_mm": -50.0,
            # 用户指定：固定零点前方 1000 mm；连续移动共用这一位置上限。
            "linear_max_mm": 1000.0,
            # 推进或后退的速度绝对值上限，单位 mm/s。
            "max_linear_mm_s": 10.0,
            # 每秒速度最多变化 10 mm/s，单位 mm/s²；不是实测刹车能力。
            "max_linear_mm_s2": 10.0,
            # TODO: 边界制动计算可依赖的减速度正值，单位 mm/s²，须独立测量。
            # None 表示未知；计算时采用 braking_defaults，不覆盖此处的 None。
            "braking_deceleration_mm_s2": None,
            # TODO: 边界制动计算使用的采样、通信与执行延迟，单位 s。
            "braking_delay_s": None,
            # 两端各保留 10 mm；按边界制动公式，向外速度在 -40/990 mm 归零。
            "boundary_margin_mm": 10.0,
            # 到位的位置条件：距最终目标不超过 1 mm，并非保证实机精度。
            "position_tolerance_mm": 1.0,
            # 到位的速度条件：外部编码器测得的推进速度绝对值不超过 0.1 mm/s。
            # 不是舵机轴 RPM；阈值由用户指定，测量噪声影响仍需后续确认。
            "velocity_tolerance_mm_s": 0.1,
            # 有效反馈的位置和速度条件同时连续满足 0.3 s，才确认到位。
            "settle_s": 0.3,
        },
        "braking_defaults": {
            # 实测值未知时使用的工程初值，不是已经验证的实际制动能力。
            # 按 5 mm/s² 估算，比按指令加速度 10 mm/s² 计算更早开始减速。
            "braking_deceleration_mm_s2": 5.0,
            # 为采样、通信和执行暂留 0.5 s；不是测得的最坏响应时间。
            "braking_delay_s": 0.5,
        },
    }
    # 显示和导出同时列出实际采用值及来源；使用者不要手动编辑该派生区。
    document["effective_braking"] = asdict(
        resolve_braking_parameters(document["motion"], document["braking_defaults"])
    )
    return document


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("show", "export"))
    parser.add_argument("output", nargs="?")
    parser.add_argument(
        "--section",
        choices=("runtime", "position"),
        default="runtime",
        help="runtime: current servo snapshot; position: future calibration/PID draft",
    )
    parser.add_argument(
        "--backend",
        choices=("simulation", "hardware"),
        default="simulation",
        help="runtime section only",
    )
    parser.add_argument(
        "--manual-input-mode",
        choices=("qualified_state", "joydev_hold"),
        default="qualified_state",
        help="runtime section only",
    )
    args = parser.parse_args(argv)
    if (args.operation == "export") != (args.output is not None):
        parser.error("export requires OUTPUT.json; show takes no output path")
    if args.section == "position":
        document = position_settings_payload()
    else:
        settings = make_servo_settings(
            settings_payload(args.backend, manual_input_mode=args.manual_input_mode)
        )
        document = servo_settings_document(settings)
    rendered = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.operation == "show":
        print(rendered, end="")
    else:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered)
        if args.section == "position":
            print(f"{output.resolve()} (position draft; not a runtime configuration)")
        else:
            print(f"{output.resolve()} sha256={settings.sha256}")


if __name__ == "__main__":
    main()
