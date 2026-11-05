import spidev
import threading
import time
import motor

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

def analogInput(channel, spi):
    """读取位移传感器输出的模拟量信号
    Args:
        channel (int): MCP3008 模数转换芯片的输出通道
        spi (object): spi对象
    Returns:
        int: 数字量输出
    """
    spi.open(0,0)
    spi.max_speed_hz = 9600
    adc = spi.xfer2([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    spi.close()
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

def distanceMeansure(channel, spi):
    data = analogInput(channel, spi)
    if data > 1023:
        dis_exec = 0
    else:
        dis_exec = round(Dis(data),2)
    feedback_distance = dis_exec
    return feedback_distance
    

# spi = spidev.SpiDev()
# channel = 1
# feedback_distance = 0
# data_lock = threading.Lock()
# exit_event = threading.Event()
# setPoint = None
# radius = 6  # mm
# motor_id = 1

# # 连接至DYNAMIXEL并清零
# while True:#Conecting Loop
#     motor.serial_port.port = '/dev/ttyUSB0'
#     motor.serial_port.baudrate = 57600
#     motor.serial_port.timeout = 10
#     try:
#         motor.serial_port.open()
#         break
#     except:
#         print('串口错误！')
# print('连接成功！')
# time.sleep(2)
# print("舵机清零中...")
# motor.position_mode()
# motor.move_pos(0)
# print('清零完成，准备就绪，开始')

# # motor.extended_position_mode()
# # time.sleep(2)
# # motor.move_pos(102400)
# target_distance = 100 # mm 


# motor.extended_position_mode()
# time.sleep(2)
# pid_controller = PIDController(Kp=0.6, Ki=0.00001, Kd=0.1)

# spi.open(0,0)
# spi.max_speed_hz = 9600

# while True:  
#     error = target_distance - distanceMeansure(1, spidev.SpiDev())
#     print("distance: ", distanceMeansure(1, spidev.SpiDev()))
#     if abs(error) < 0.1:
#         print("舵机已到达指定位置")
#         break 
#     output = pid_controller.calculate(error)
#     print("output: ", output)
#     rotation = 4096 * (output / (2 * 3.1415926 * radius))
#     print("current pos: ", motor.read_position())
#     motor.move_pos(motor.read_position()+round(rotation))
    
