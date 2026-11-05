'''
检测操作系统
'''
import os

if os.name == 'nt':
    # Windows操作系统
    import msvcrt
    # 定义getch函数，获取用户输入的一个字符
    def getch():
        return msvcrt.getch().decode()
else:
    # *Unix操作系统
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    # 定义getch函数，获取用户输入  
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

'''
设置舵机参数
'''

from dynamixel_sdk import *  #使用Dynamixel SDK库

#********* 定义电机型号 *********
#***** 每次只能选择一个型号 *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

# 设置控制表地址，一般默认的就是正确的  查阅：https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # 查手册
    DXL_MAXIMUM_POSITION_VALUE  = 500     # 查手册
    BAUDRATE                    = 57600
    ADDR_OPERATING_MODE         = 11               # Control table address is different in Dynamixel model
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    ADDR_PRESENT_POSITION       = 611
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    ADDR_PRESENT_POSITION       = 580
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'XL320':
    ADDR_TORQUE_ENABLE          = 24
    ADDR_GOAL_POSITION          = 30
    ADDR_PRESENT_POSITION       = 37
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
    BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

# DYNAMIXEL协议版本 (1.0 / 2.0)
# 在此处查看相关说明https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# 在此处更改舵机ID，所有DYNAMIXEL的出厂默认ID为1
DXL_ID                      = 1

# 使用实际分配给U2D2的端口
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0' # 树莓派只有一个USB串口？？？

TORQUE_ENABLE               = 1     # 启用转矩模式，就是解锁电机
TORQUE_DISABLE              = 0     # 电机锁定，不能工作。和上面形成互锁
DXL_MOVING_STATUS_THRESHOLD = 10    # 设定电机运动阈值，当电机与目标位置的差值小于此阈值，认为已达到指定位置
EXT_POSITION_CONTROL_MODE   = 5     # 速度控制模式（无尽）

dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]    # 设置全局移动范围