'''
初始化设备
'''

import config
from dynamixel_sdk import *

# 初始化PortHandler实例
# 设置端口路径
# 获取porthandlerlinux或porthandlerwindows的方法和成员
portHandler = PortHandler(config.DEVICENAME)

# 初始化 PacketHandler 实例
# 设置协议版本
# 获取 Protocol1PacketHandler 或 Protocol2PacketHandler 的方法和成员变量
packetHandler = PacketHandler(config.PROTOCOL_VERSION)

# 开启端口
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    config.getch()
    quit()

# 设置波特率
if portHandler.setBaudRate(config.BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    config.getch()
    quit()

# 设置舵机模式
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, config.DXL_ID, config.ADDR_OPERATING_MODE, config.EXT_POSITION_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode changed to extended position control mode.")

# 启用力矩操作，dxl_comm_result, dxl_error 分别接收执行结果和报错信息，后面的部分是通过通信协议(porthandler)将启动力矩的指令(TORQUE_ENABLE)传入到制定的舵机(DXL_ID)的指定地址(ADDR_TORQUE_ENABLE)。
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, config.DXL_ID, config.ADDR_TORQUE_ENABLE, config.TORQUE_ENABLE) 
if dxl_comm_result != COMM_SUCCESS:
    # 如果通信失败，打印通信结果
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result)) # getTxRxResult()方法用于获取通信结果
elif dxl_error != 0:
    # 如果报错，打印报错信息
    print("%s" % packetHandler.getRxPacketError(dxl_error)) # getRxPacketError()方法用于获取错误信息
else:
    # 如果通信成功，打印成功信息
    print("Dynamixel has been successfully connected")
    
