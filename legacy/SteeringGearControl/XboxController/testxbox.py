# import pygame
# import serial
# import time

# # 初始化手柄
# pygame.init()
# pygame.joystick.init()
# joystick = pygame.joystick.Joystick(0)
# joystick.init()

# # 初始化串口通信
# serial_port = serial.Serial('/dev/ttyUSB0', baudrate=57600, timeout=0.1)

# while True:
#     pygame.event.pump()

#     # 读取手柄输入
#     left_stick_x = joystick.get_axis(0)  # X轴
#     left_stick_y = joystick.get_axis(1)  # Y轴
    
#     print(left_stick_x, left_stick_y)

#     # # 将手柄输入映射到舵机角度范围
#     # # 根据您的需求进行映射

#     # # 发送控制指令到舵机
#     # command = f"#1 P{int(angle_value)}"  # 根据舵机通信协议调整
#     # serial_port.write(command.encode())

#     # 延时，根据需要调整
#     time.sleep(2)

# # 关闭串口
# serial_port.close()


import pygame
import time

# 初始化pygame
pygame.init()

# 初始化手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

try:
    while True:
        pygame.event.pump()  # 更新事件队列
        # 读取左摇杆的X和Y轴值
        left_stick_x = joystick.get_axis(0)
        left_stick_y = joystick.get_axis(1)

        # 读取右摇杆的X和Y轴值
        right_stick_x = joystick.get_axis(3)
        right_stick_y = joystick.get_axis(4)

        # 输出左摇杆和右摇杆的值
        print(f"Left Stick X: {left_stick_x}, Left Stick Y: {left_stick_y}")
        print(f"Right Stick X: {right_stick_x}, Right Stick Y: {right_stick_y}")
        time.sleep(3  )

except KeyboardInterrupt:
    pass

# 清理
joystick.quit()
