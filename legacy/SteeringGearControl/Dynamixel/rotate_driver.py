from dynamixel_sdk import *
import config
import initialize
import controller
import threading
import time

'''
*******程序结构********
while 1:
    if 退出键被按下:
        关闭端口
        退出程序
    if 正转控制键被按下:
        目标位置+500
        正转500
        初始位置=当前位置
    if 反转控制键被按下:
        目标位置-500
        反转500
        初始位置=当前位置
'''

# 获取当前位置
dxl_present_position, dxl_comm_result, dxl_error = initialize.packetHandler.read4ByteTxRx(initialize.portHandler, config.DXL_ID, config.ADDR_PRESENT_POSITION)
# 若失败打印信息
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % initialize.packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % initialize.packetHandler.getRxPacketError(dxl_error))
else:
    # dxl_goal_position = dxl_present_position
    dxl_goal_position = 10000
    flag_initial_position = dxl_present_position
    
running = True

while running:
    #print("上推左摇杆舵机1正转, 下推反转, 按下键盘3退出\n")
    
    joystick_device_path = "/dev/input/event2"  
    
    #创建手柄事件读取进程
    joystick_thread = threading.Thread(target=controller.read_joystick_y_axes, args=(joystick_device_path,))
    joystick_thread.daemon = True
    joystick_thread.start()
       
    # # 退出程序    
    # if config.getch() == '3':
    #     break
    
    time.sleep(0.5)
    
    flag = controller.joystick_y_value[1]   
    # 正转
    # if config.getch() == '1':
    if flag < -0.5:
        dxl_goal_position = dxl_goal_position + config.DXL_MAXIMUM_POSITION_VALUE
        # 写入目标位置, 设置当前位置为初始位置，防止舵机倒转回0
        dxl_comm_result, dxl_error = initialize.packetHandler.write4ByteTxRx(initialize.portHandler, config.DXL_ID, config.ADDR_GOAL_POSITION, dxl_goal_position)
        # 打印相关信息
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % initialize.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % initialize.packetHandler.getRxPacketError(dxl_error))       
        
        while 1:    
        # 读取当前位置
            dxl_present_position, dxl_comm_result, dxl_error = initialize.packetHandler.read4ByteTxRx(initialize.portHandler, config.DXL_ID, config.ADDR_PRESENT_POSITION)
            # 打印相关信息
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % initialize.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % initialize.packetHandler.getRxPacketError(dxl_error))
            #print("[舵机ID:%03d] 目标位置:%03d  当前位置:%03d" % (config.DXL_ID, dxl_goal_position-flag_initial_position, dxl_present_position-flag_initial_position))
            print("[舵机ID:%03d] 目标位置:%03d  当前位置:%03d" % (config.DXL_ID, dxl_goal_position, dxl_present_position))
            if not abs(dxl_goal_position - dxl_present_position) > config.DXL_MOVING_STATUS_THRESHOLD:
                break
         
    # 反转
    #if config.getch() == '2':
    if flag > 0.5 :
        dxl_goal_position = dxl_goal_position - config.DXL_MAXIMUM_POSITION_VALUE
        # 写入全局位置, 设置当前位置为初始位置，防止舵机倒转回0
        dxl_comm_result, dxl_error = initialize.packetHandler.write4ByteTxRx(initialize.portHandler, config.DXL_ID, config.ADDR_GOAL_POSITION, dxl_goal_position)
        # 打印相关信息
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % initialize.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % initialize.packetHandler.getRxPacketError(dxl_error)) 
        
        while 1:    
        # 读取当前位置
            dxl_present_position, dxl_comm_result, dxl_error = initialize.packetHandler.read4ByteTxRx(initialize.portHandler, config.DXL_ID, config.ADDR_PRESENT_POSITION)
            # 打印相关信息
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % initialize.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % initialize.packetHandler.getRxPacketError(dxl_error))
            print("[舵机ID:%03d] 目标位置:%03d  当前位置:%03d" % (config.DXL_ID, dxl_goal_position, dxl_present_position))
            if not abs(dxl_goal_position - dxl_present_position) > config.DXL_MOVING_STATUS_THRESHOLD:
                break

# 禁用舵机转矩：锁定
dxl_comm_result, dxl_error = initialize.packetHandler.write1ByteTxRx(initialize.portHandler, config.DXL_ID, config.ADDR_TORQUE_ENABLE, config.TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % initialize.packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % initialize.packetHandler.getRxPacketError(dxl_error))

# Close port
initialize.portHandler.closePort()