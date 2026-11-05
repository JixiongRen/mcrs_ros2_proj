import spidev

import motor


def analogInput(channel, spi):
    """读取位移传感器输出的模拟量信号
    Args:
        channel (int): MCP3008 模数转换芯片的输出通道
        spi (object): spi对象
    Returns:
        int: 数字量输出
    """
    spi.open(0, 0)
    spi.max_speed_hz = 9600
    adc = spi.xfer2([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

def Dis(data):
    """根据传感器输出的模拟量计算距离
    Args:
        data (int): 数字转换量 0~1023
    Returns:
        float: 距离
    """
    dis = (data * 1500) / float(1023)
    return dis

def use_position_mode(motor_id, rotation):
    motor.motor_id = motor_id
    currentpos = motor.read_position()
    goalpos = currentpos + rotation
    motor.move_pos(goalpos)