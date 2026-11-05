import serial
import time
import encoder_func

# 配置串口参数，根据你的设备和需要进行调整
SERIAL_PORT = '/dev/ttySC1'  # 树莓派上的串口设备路径
BAUDRATE = 38400          # 波特率，应与编码器设置匹配
TIMEOUT = 1                   # 读取超时时间

# 创建串口对象
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)


if ser.is_open:
    ser.close()
ser.open()
time.sleep(0.1)
if ser.isOpen():
    encoder_func.SET_ZERO_POSITION(ser)
    time.sleep(0.5)
    encoder_func.SET_DIRECTION('anticlockwise', ser)
    time.sleep(0.5)
    ser.close()

ser.open()
encoder_func.SET_ZERO_POSITION(ser)
time.sleep(0.5)
encoder_func.SET_DIRECTION('anticlockwise', ser)
time.sleep(0.5)
encoder_func.QUERY_ANGLE(ser)