import dynamixel_sdk as dxl
import config
import threading
import time
from dynamixel_sdk import *

# 控制表的地址
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRO_GOAL_VELOCITY = 104
ADDR_PRO_PRESENT_VELOCITY = 128

# 控制表的数据长度
LEN_PRO_GOAL_VELOCITY = 4
LEN_PRO_PRESENT_VELOCITY = 4

# 舵机ID
DXL_ID = 1

# 舵机通信端口和波特率
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600

# 初始化舵机SDK
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(2.0)

# 打开舵机通信端口
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# 设置舵机通信波特率
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# 启用舵机转矩
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, config.ADDR_TORQUE_ENABLE, config.TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel torque has been successfully enabled")


# 设置目标速度
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, config.DXL_ID, ADDR_PRO_GOAL_VELOCITY, 50)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("set speed success")

# 读取当前速度
present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_VELOCITY)

if dxl_comm_result != dxl.COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("[ID:%03d] Present Velocity : %d" % (DXL_ID, present_velocity))

# 关闭舵机通信端口
portHandler.closePort()
