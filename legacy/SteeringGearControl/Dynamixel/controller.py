import evdev

def read_joystick_y_axes(device_path):
    left_y = 0.0
    right_y = 0.0
    global joystick_y_value
    try:
        device = evdev.InputDevice(device_path)
        print("读取到手柄！")
        joystick_y_value = [0,0]
        for event in device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_Y:
                    # 左摇杆Y轴数值
                    left_y = (event.value - 0) / 32768  # 将值缩放到范围[-1, 1]
                elif event.code == evdev.ecodes.ABS_RY:
                    # 右摇杆Y轴数值
                    right_y = (event.value - 0) / 32768  # 将值缩放到范围[-1, 1]

            if left_y != 0.0 or right_y != 0.0:
                joystick_y_value = [left_y, right_y]
    except Exception as e:
        print(f"无法读取手柄设备：{e}")
        joystick_y_value = [0,0]

if __name__ == "__main__":
    # 你需要知道手柄设备的路径，通常在/dev/input/目录下
    joystick_device_path = "/dev/input/event2"  # 将X替换为手柄设备的编号

    y_axes_values = read_joystick_y_axes(joystick_device_path)
    print("左摇杆Y轴数值:", y_axes_values[0])
    print("右摇杆Y轴数值:", y_axes_values[1])