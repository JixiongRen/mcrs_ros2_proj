import motor
import time
import pygame
import pgzrun
import RPi.GPIO as GPIO


"""
舵机部分
"""

'''
连接DYNAMIXEL
'''
while True:#Conecting Loop
    motor.serial_port.port = '/dev/ttyUSB0'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 1
    try:
        motor.serial_port.open()
        break
    except:
        print('串口错误！')

print('连接成功！')
time.sleep(2)
print('准备就绪，开始')

'''
控制舵机转动的函数
'''
def use_wheel_mode(motor_id, flag, goal_RPM=100):
    motor.motor_id = motor_id
    motor.velocity_mode()
    if flag > 0.5:
        motor.send_goal_RPM((-1)*goal_RPM)
        value1 = motor.read_position()
        print(value1)
    if -0.5<flag<0.5:
        motor.send_goal_RPM(0)
        value1 = motor.read_position()
        print(value1)
    if flag < -0.5:
        motor.send_goal_RPM(goal_RPM)
        value1 = motor.read_position()
        print(value1)


# """
# 步进电机部分
# """

# 设置GPIO模式
GPIO.setmode(GPIO.BOARD)

# 定义引脚
PUL = 29 # 步进脉冲
DIR = 31  # 步进方向

# 设置引脚为输出
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)


def set_motor_forward(flag):
    """
    设置电机正转的逻辑。
    """
    GPIO.output(DIR, GPIO.LOW)  # 设置方向
    for _ in range(50):  # 发送足够的脉冲来完成一次正转
    # while flag == 1:
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.002)

def set_motor_reverse(flag):
    """
    设置电机反转的逻辑。
    """
    GPIO.output(DIR, GPIO.HIGH)  # 设置方向为反转
    for _ in range(50):  # 发送足够的脉冲来完成一次反转
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.002)

def stop_motor(flag):
    """
    停止电机的逻辑。
    """
    GPIO.output(DIR, GPIO.HIGH)  # 设置方向为反转
    for _ in range(0):  # 发送足够的脉冲来完成一次反转
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.002)




# 初始化pygame
pygame.init()

# 设置手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

# # 设置帧率
clock = pygame.time.Clock()
frame_rate = 4800  # 例如，设置为60帧每秒

# PygameZero游戏循环
def draw():
    pass  # 这里不需要画面

def update():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PUL, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)
    
    # 获取摇杆的X轴值
    joystick_value_1 = joystick.get_axis(1)
    joystick_value_3 = joystick.get_axis(3)
    
    # 根据摇杆值判断舵机正反转
    if joystick_value_1 > 0.5:
        flag_1 = 1  # 正转
    elif joystick_value_1 < -0.5:
        flag_1 = -1  # 反转
    else:
        flag_1 = 0  # 不转
    
    # 根据摇杆值判断步进电机正反转
    if joystick_value_3 > 0.5:
        flag_3 = 1  # 正转
        set_motor_forward(flag_3)
        GPIO.cleanup()
    elif joystick_value_3 < -0.5:
        flag_3 = -1  # 反转
        set_motor_reverse(flag_3)
        GPIO.cleanup()
    else:
        flag_3 = 0  # 不转
        stop_motor(flag_3)
        GPIO.cleanup()

    # 调用舵机控制函数
    use_wheel_mode(1, flag_1)

# 运行PygameZero游戏循环
pgzrun.go()

