#!/usr/bin/env python3
"""编码器通信记录与未来位移闭环的配置草稿，仅依赖 Python 标准库。

查看：python3 config/encoder_settings.py show
导出：python3 config/encoder_settings.py export OUTPUT.json

本脚本不打开串口、不修改设备树或 GPIO、不写编码器寄存器。
导出内容不是当前 EncoderConfig，当前运行链不会加载此草稿。
None 表示尚未确认；填写参数不等于已经完成标定或启用闭环。
软件零点与参考标识由后续运行时建立，不把本次寄存器读数固化为零点。

通信依据：artifacts/encoder_readonly/waveshare-b-setup-8uelsbj4/report.json
短时测时：artifacts/encoder_readonly/timing-7x7x_4_p/report.json
单圈粗测：artifacts/encoder_readonly/one-turn-w2m1v9vq/assessment.json
硬件参数：用户于 2026-11-09 提供的购买记录、测量轮尺寸及直连关系。
计数格式：用户提供的布瑞特《RS485 通信协议多圈 V2.7》5.4.1.1 节。
手册：docs/prebuild_docs/3764672290002 RS485说明书通信协议 多圈 V2.7.pdf
"""

from __future__ import annotations

import argparse
import json
from math import pi
from pathlib import Path
from runpy import run_path


def settings_payload():
    """返回可编辑草稿；所有长度使用 mm，时间使用 s，方向以推进为正。"""
    single_turn_resolution_bits = 12  # 购买规格；每圈计数由 2**位数计算。
    multi_turn_range_revolutions = 1800  # 购买规格的多圈量程。
    counts_per_revolution = 1 << single_turn_resolution_bits
    count_modulus = counts_per_revolution * multi_turn_range_revolutions
    register_count = 2  # V2.7 5.4.1.1：0x0000、0x0001 两个 16 位寄存器。
    measurement_wheel_radius_mm = 10.89  # 用户提供的有效接触半径。
    encoder_revolutions_per_wheel_revolution = 1.0  # 用户确认直连。
    direction_sign = 1  # 用户约定：正向推进时计数增加。
    # 从同目录的纯 Python 舵机配置读取位移控制限速，避免手工重复维护 10 mm/s。
    # run_path 不执行脚本的 CLI 入口；配置缺失时报错，不使用隐含缺省值。
    servo_position = run_path(str(Path(__file__).with_name("servo_settings.py")))[
        "position_settings_payload"
    ]()
    speed_plausibility_factor = 1.5  # 工程初值，预留 50% 瞬态/测量波动余量。
    return {
        "kind": "encoder_position_draft",
        "schema_version": 3,
        "device": {
            "brand": "布瑞特",
            "purchase_description": "布瑞特磁电断电记忆RS485ssiCANModbus",
            "manual_version": "V2.7",
            # 用户提供的完整型号，按原文记录。
            "model": "BRT38-ROM-4096-D1800-RT1-IP68",
            "single_turn_resolution_bits": single_turn_resolution_bits,
            # 购买规格的多圈量程，单位为编码器轴圈；本轮按用户指定也作为循环圈数。
            "multi_turn_range_revolutions": multi_turn_range_revolutions,
        },
        "board": {
            # 以下是已安装扩展板及设备树的记录，仅展示，不会自动应用。
            "model": "Waveshare RS485 CAN HAT (B)",
            "connector": "RS485_1",
            "driver": "sc16is7xx",
            "overlay": "sc16is752-spi1,int_pin=24",
            "spi_bus": 1,
            "chip_select": 0,
            "miso_gpio_bcm": 19,
            "mosi_gpio_bcm": 20,
            "sclk_gpio_bcm": 21,
            "chip_select_gpio_bcm": 18,
            "interrupt_gpio_bcm": 24,
            "crystal_hz": 14745600,
        },
        "communication": {
            # 这些设置已收到合法的功能 03 响应；不包含硬件置零或方向写入。
            "interface": "RS485",
            "protocol": "Modbus RTU",
            # 主站逐次查询；对应 V2.7 5.4.6 的模式 0，不发送模式设置命令。
            # 不是设备模式寄存器的读回值；该寄存器在手册中标为仅写。
            "mode": "query",
            "port": "/dev/ttySC1",
            # 保留本机已成功读取的速率，不用手册的出厂默认 9600 覆盖。
            "baudrate": 38400,
            "bytesize": 8,
            "parity": "N",
            "stopbits": 1,
            "address": 1,
            "function_code": 3,
            "register_start": 0,
            "register_count": register_count,
            # 示例查询 CRC 数值为 0x0BC4，线上字节为 C4 0B，低字节先发。
            # 手册帧格式表的 CRC 高/低标签与示例不一致，以示例及已有实读为据。
            "crc_byte_order": "little",
        },
        "count_format": {
            # 两个寄存器共四字节；存储位宽与每圈 12 bit 分辨率不同。
            "bits": 16 * register_count,
            "byte_order_in_register": "big",
            # V2.7 5.4.1.1：00 01 76 3B 表示 95803，高位寄存器在前。
            "word_order": "high_word_first",
            # 同节明确为无符号整数；反转可以使读数减少，不要求原始数为负。
            "signed": False,
            # V2.7 5.4.1.1：0..(单圈分辨率*硬件圈数-1)，两端均包含。
            # 计数范围由手册确定；循环规则按用户本轮指定采用，未做跨界实测。
            "min_count": 0,
            "max_count": count_modulus - 1,
            # 用户指定按 1800 圈循环：1800*4096=7372800 count。
            # 即使离总量程上端很远，靠近原始零点反向移动也可能越过下端。
            "count_modulus": count_modulus,
            # 单位 count/圈；12 bit 对应 4096，厂家手册与购买规格一致。
            # 手动一圈粗测 4063 不作为额定分辨率，也不表示已完成精度标定。
            "counts_per_revolution": counts_per_revolution,
        },
        "mechanics": {
            # 用户确认的测量轮接触半径，单位 mm；不同于舵机驱动轮半径。
            "measurement_wheel_radius_mm": measurement_wheel_radius_mm,
            # 编码器轴圈数 / 测量轮圈数，无量纲；用户确认测量轮直连编码器轴。
            "encoder_revolutions_per_wheel_revolution": encoder_revolutions_per_wheel_revolution,
            # 用户明确约定：沿正方向推进时计数增加，软件位移方向系数为 +1。
            # 不据此推定轴端 CW/CCW；不写硬件方向寄存器 0x0009，不改原始计数。
            "direction_sign": direction_sign,
            # 用户选择采用理论比例，单位 mm/count；随半径、传动比、分辨率计算。
            # 无滑动且有效半径不变时成立，不是独立位移实测结果。
            "mm_per_count": (
                2
                * pi
                * measurement_wheel_radius_mm
                / (encoder_revolutions_per_wheel_revolution * counts_per_revolution)
            ),
            # geometry 标明理论依据；标签包含比例参数及方向，不宣称实测标定通过。
            "calibration_id": (
                f"geometry-r{measurement_wheel_radius_mm:g}mm-"
                f"g{encoder_revolutions_per_wheel_revolution:g}-"
                f"n{counts_per_revolution}-s{direction_sign:+d}-v1"
            ),
        },
        "acquisition": {
            # 初值 20 ms（50 Hz）：独立只读短测 100 次成功，无漏周期。
            # 实测事务中位数 8.03 ms、最大 13.25 ms，详见上方短时测时报告。
            # 手册的内核刷新 50 us、自动回传 50 ms、内部测速 100 ms 均非此参数。
            "sample_period_s": 0.02,
            # 用户指定的初值，单位 s；从原采样时刻算，不因重复发布而续期。
            # 读取间隔+通信及调度延迟须小于 0.1 s；10 mm/s 下此时长对应 1 mm。
            "feedback_max_age_s": 0.1,
            # 初值 50 ms，约为本次最大事务耗时 13.25 ms 的 3.77 倍，留调度余量。
            # 完整有效回复到达即返回，不固定等待 50 ms；一次只允许一个未完成查询。
            # 20 ms 周期 + 50 ms 超时 < 100 ms 反馈有效期；短测不代表最坏时延保证。
            "transaction_timeout_s": 0.05,
            # 用户指定的初值：打开/重连后连续 0.1 s 无旧数据再查询，单位 s。
            # 仅影响打开/重连，不在每次正常查询前等待；不与反馈有效期绑定。
            # TODO: 尚未验证最晚回复延迟小于此值，不能据此宣称已排除迟到回复。
            "quiet_s": 0.1,
            # 位移闭环下读数变化的合理速度上限，单位 mm/s；当前为 1.5*10=15。
            # 随舵机 motion.max_linear_mm_s 联动；不是放宽运动指令限速。
            # 手柄原生 RPM 控制不使用本阈值，不能据此限制手柄模式下的运动。
            "max_speed_mm_s": (
                speed_plausibility_factor * servo_position["motion"]["max_linear_mm_s"]
            ),
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("show", "export"))
    parser.add_argument("output", nargs="?")
    args = parser.parse_args(argv)
    if (args.operation == "export") != (args.output is not None):
        parser.error("export requires OUTPUT.json; show takes no output path")
    rendered = json.dumps(settings_payload(), ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.operation == "show":
        print(rendered, end="")
    else:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
        print(output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
