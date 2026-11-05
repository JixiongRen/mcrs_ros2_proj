import motor
import time
import pygame
import threading
import RPi.GPIO as GPIO
import os
import spidev 
from time import sleep

#***************函数*****************#
def print_at_position(x, y, text):
    """在固定的位置打印

    Args:
        x (int): x坐标
        y (int): y坐标
        text (str): 要打印的字符串
    """
    print(f"\033[{y};{x}H{text}", end='', flush=True)

def  clear_screen():
    """清屏函数，用于更新显示数据
    """
    # 获取终端宽度
    _, columns = os.popen('stty size', 'r').read().split()
    # 将光标移动到第1行第6列
    move_cursor = f"\033[1;6H"
    # 清除从第6列到屏幕右侧的内容
    clear_content = "\033[J"
    # 打印清屏命令
    print("\033c", end='', flush=True)
    # 打印移动光标和清除内容的命令
    print(move_cursor + clear_content, end='', flush=True)
    
def use_wheel_mode(motor_id, flag, goal_RPM=20):
    """控制舵机转动

    Args:
        motor_id (int): 舵机的id
        flag (int): 舵机运动状态的标志符号
        goal_RPM (int, optional): 目标速度. Defaults to 40.
    """
    global current_pos
    motor.motor_id = motor_id
    motor.velocity_mode()
    if flag == 1:
        motor.send_goal_RPM((-1)*goal_RPM)
        current_pos = motor.read_position()
        # print(value1)
    if flag == 0:
        motor.send_goal_RPM(0)
        current_pos = motor.read_position()
        # print(value1)
    if flag == -1:
        motor.send_goal_RPM(goal_RPM)
        current_pos = motor.read_position()
        # print(value1)
        
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
    
def analogInput(channel, spi):
    """读取位移传感器输出的模拟量信号

    Args:
        channel (int): MCP3008 模数转换芯片的输出通道
        spi (object): spi对象

    Returns:
        int: 数字量输出
    """
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

def normalize_num(num):
    """数据规范化处理

    Args:
        num (float): 距离

    Returns:
        str: 规范化的数据字符串
    """
    # 将数据转换为字符串并用小数点分割
    parts = str(num).split('.')
    # 如果数据只有整数部分
    if len(parts) == 1:
        return '{:04d}.00'.format(int(parts[0]))
    # 如果数据有小数部分
    else:
        return '{:03d}.{:02d}'.format(int(parts[0]), int(parts[1]))
    

def motor_control_thread():
    """线程函数：步进电机控制线程

    Args:
        exit_event (_type_): 线程退出事件
        joystick (object): 手柄对象
    """
    global flag_3
    pul_num = 0
    while not exit_event.is_set():
        joystick_value_3 = joystick.get_axis(3)
        flag_3 = 1 if joystick_value_3 > 0.5 else (-1 if joystick_value_3 < -0.5 else 0)
        if flag_3 != 0:
            if flag_3 > 0:
                set_motor_forward(DIR, PUL)
                pul_num += 1
            else:
                set_motor_reverse(DIR, PUL)
                pul_num -= 1
        else:
            stop_motor(DIR, PUL)
        rotation_angle = "{:.1f}".format(((pul_num/6400)*360)*(17/30))
        print_at_position(1, 6, f"旋转角度: {rotation_angle}°")
        time.sleep(0.001)
        
def steering_control_thread():
    """线程函数：舵机控制线程

    Args:
        exit_event (_type_): 线程退出事件
        joystick (object): 手柄对象
    """
    global flag_1
    global current_pos
    flag_1_old = 0
    use_wheel_mode(1, flag_1_old)
    while not exit_event.is_set():
        joystick_value_1 = joystick.get_axis(1)
        flag_1 = 1 if joystick_value_1 > 0.5 else (-1 if joystick_value_1 < -0.5 else 0)
        if flag_1 != flag_1_old:
            flag_1_old = flag_1
            use_wheel_mode(1, flag_1_old)
        current_dis = "{:.2f}".format((current_pos/4096)*2*3.1415926*6)
        try:
            print_at_position(1, 7, f"进动距离：{current_dis}mm")
        except Exception as e:
            pass
        #time.sleep(0.001)
        
def distance_meansure_thread():
    """传感器工作线程
    """
    global channel
    global shared_dis_exec
    global spi
    spi.open(0,0)
    spi.max_speed_hz = 9600
    while not exit_event.is_set():
        with lock:
            data = analogInput(channel, spi)
            if data > 1023:
                dis_exec = 0
            else:
                dis_exec = round(Dis(data),2)
            shared_dis_exec = dis_exec
            dis_for_display = normalize_num(shared_dis_exec)
            try:
                print_at_position(1, 8, f"实际进动距离：{dis_for_display}mm")
            except Exception as e:
                pass
            # time.sleep(0.1)
        
def clear_screen_thread():
    """线程函数：清屏线程

    Args:
        exit_event (_type_): 线程退出事件
    """
    while not exit_event.is_set():
        clear_screen()
        time.sleep(0.1)
        
#*****************主代码****************#

print("\033c", end='', flush=True)  # 清屏

# 连接至DYNAMIXEL并清零
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
print("舵机清零中...")
motor.position_mode()
motor.move_pos(0)
print('清零完成，准备就绪，开始')

# 设置步进电机参数

# 定义引脚
PUL = 29  # 步进脉冲
DIR = 31  # 步进方向

# 设置GPIO模式
GPIO.setmode(GPIO.BOARD)

# 设置引脚为输出
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

# 初始化pygame
pygame.init()

# 初始化手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 一些全局变量
flag_3 = 0
flag_1 = 0
current_pos = 0
exit_event = threading.Event()  # 用于通知线程退出的事件
channel = 0 # MCP3008模拟量输入通道1
spi = spidev.SpiDev()
shared_dis_exec = None
lock = threading.Lock()  # 创建一个锁对象

# 启动各个线程
motor_thread = threading.Thread(target=motor_control_thread)
motor_thread.start()

steering_thread = threading.Thread(target=steering_control_thread)
steering_thread.start()

distance_meansurement_thread = threading.Thread(target=distance_meansure_thread)
distance_meansurement_thread.start()

clear_screen_thread = threading.Thread(target=clear_screen_thread)
clear_screen_thread.start()

try:
    while not exit_event.is_set():
        pygame.event.pump()  # 处理pygame事件
        time.sleep(0.000001)
except KeyboardInterrupt:
    pass
finally:
    exit_event.set()
    motor_thread.join()
    steering_thread.join()
    distance_meansurement_thread.join()
    clear_screen_thread.join()
    GPIO.cleanup()  # 清理GPIO资源
    pygame.joystick.quit()
    pygame.quit()