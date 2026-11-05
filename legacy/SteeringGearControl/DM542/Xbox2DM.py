import evdev
import RPi.GPIO as GPIO
import time
import asyncio

# 找到Xbox手柄的输入设备
xbox_controller = evdev.InputDevice('/dev/input/event2')  # 请替换X为手柄的事件号


# 设置GPIO模式
GPIO.setmode(GPIO.BOARD)  # 或者 GPIO.setmode(GPIO.BOARD)

# 定义引脚
PUL = 29  # 步进脉冲
DIR =  31 # 步进方向


# 设置引脚为输出
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

# 定义电机引脚
PUL = 29  # 步进脉冲
DIR = 31  # 步进方向

# 将Y轴数值定义为速度
def map_y_axis_to_movement(y_value, min_pul=0, max_pul=500):
    """
    将Y轴数值映射到电机速度范围。步进电机的
    Args:
        y_value (int): Y轴数值，通常绝对值在0到32767之间。
        min_pul (int): 最小脉冲数，对应最小移动量。
        max_pul (int): 最大脉冲数，对应最大移动量。

    Returns:
        int: 映射后的脉冲量。
    """
    # 将Y轴数值映射到速度范围
    pul_range = max_pul-min_pul
    #pul = int(abs(y_value) * pul_range / 32768) + min_pul
    pul = 5
    return max(min(pul, max_pul), min_pul)  # 确保速度在范围内

def set_motor_forward(DIRpin, PULpin, pul):
    """
    设置电机正转的逻辑。
    """
        # 设置GPIO模式
    GPIO.setmode(GPIO.BOARD)  # 或者 GPIO.setmode(GPIO.BOARD)
    GPIO.output(DIRpin, GPIO.LOW)  # 设置方向
    for _ in range(pul):  # 发送足够的脉冲来完成一次正转
        GPIO.output(PULpin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(PULpin, GPIO.LOW)
        time.sleep(0.001)

def set_motor_reverse(DIRpin, PULpin, pul):
    """
    设置电机反转的逻辑。
    """
    
    # 设置GPIO模式
    GPIO.setmode(GPIO.BOARD)  # 或者 GPIO.setmode(GPIO.BOARD)
    GPIO.output(DIRpin, GPIO.HIGH)  # 设置方向为反转
    for _ in range(pul):  # 发送足够的脉冲来完成一次反转
        GPIO.output(PULpin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(PULpin, GPIO.LOW)
        time.sleep(0.001)

def stop_motor(DIRpin, PULpin):
    """
    停止电机的逻辑。
    """
        # 设置GPIO模式
    GPIO.setmode(GPIO.BOARD)  # 或者 GPIO.setmode(GPIO.BOARD)
    GPIO.output(DIRpin, GPIO.HIGH)  # 设置方向为反转
    for _ in range(0):  # 发送足够的脉冲来完成一次反转
        GPIO.output(PULpin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(PULpin, GPIO.LOW)
        time.sleep(0.001)

async def handle_controller_events(DIRpin, PULpin, xbox_controller):
    previous_y_value = 0  # 保存上一次的Y轴数值
    while True:
        event = await xbox_controller.async_read_one()
        if event and event.type == evdev.ecodes.EV_ABS and event.code == evdev.ecodes.ABS_Y:
            y_axis_value = event.value

            # 如果Y轴数值发生变化或保持不变，都会进入这个分支
            if y_axis_value != previous_y_value:
                # 更新电机控制
                pul = map_y_axis_to_movement(y_axis_value)
                if y_axis_value > 0:
                    set_motor_forward(DIRpin, PULpin, pul)
                elif y_axis_value < 0:
                    set_motor_reverse(DIRpin, PULpin, pul)
                else:
                    stop_motor(DIRpin, PULpin)
                
                previous_y_value = y_axis_value


async def main():
    GPIO.cleanup()
    # 启动事件处理循环
    controller_task = asyncio.create_task(handle_controller_events(DIR, PUL, xbox_controller))
    # 等待手柄事件处理完成（这里可以加入其他任务等待）
    await controller_task
    

if __name__ == "__main__":
    asyncio.run(main())