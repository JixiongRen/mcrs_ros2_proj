import RPi.GPIO as GPIO
import time


def set_motor_forward(DIR, PUL):
    """设置电机正转的逻辑

    Args:
        DIR (int): 方向引脚号
        PUL (int): 脉冲引脚号
    """
    GPIO.output(DIR, GPIO.LOW)  # 设置方向
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(0.0002)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(0.0002)


def set_motor_reverse(DIR, PUL):
    """设置电机反转的逻辑

    Args:
        DIR (int): 方向引脚号
        PUL (int): 脉冲引脚号
    """
    GPIO.output(DIR, GPIO.HIGH)  # 设置方向为反转
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(0.0002)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(0.0002)


def stop_motor(DIR, PUL):
    """停止电机的逻辑

    Args:
        DIR (int): 方向引脚号
        PUL (int): 脉冲引脚号
    """
    GPIO.output(DIR, GPIO.HIGH)  # 设置方向为反转


def get_pulse_number(Angle, pluse_per_round, Transmission_ratio=(17/30)):
    """
    根据旋转角度和每圈脉冲数计算需要发送的脉冲数
    Args:
        Angle: 旋转角度
        pluse_per_round: 每圈脉冲数
        Transmission_ratio: 传动比
    """
    return int((Angle/360*pluse_per_round)/Transmission_ratio)


def is_numeric(s):
    """
    判断输入字符串是否为数字
    Args:
        s: 待判断的字符串
    Returns: 判断结果
    """
    if s is not '':
        if s[0] == '-' and s[1:].replace('.', '', 1).isdigit():
            return True
        elif s.replace('.', '', 1).isdigit():
            return True
        return False
    return False