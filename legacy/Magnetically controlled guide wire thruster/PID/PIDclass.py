import time

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

def main():
    target_position = 100  # 目标位置（mm）
    current_position = 0  # 当前位置（mm）
    radius = 6  # 舵机转轴半径（mm）

    pid_controller = PIDController(Kp=0.1, Ki=0.01, Kd=0.05)  # 设置 PID 参数

    while True:
        # 这里假设传感器采集到的反馈位置为feedback_position
        feedback_position = 0  # 假设初始反馈位置为0
        error = target_position - feedback_position

        if abs(error) < 0.1:  # 如果位置误差很小，认为已经到达目标位置
            print("Reached target position.")
            break

        output = pid_controller.calculate(error)
        rotation = output / (2 * 3.141592653589793 * radius) * 4096  # 将 PID 输出转换为舵机位置变化
        current_position += rotation * (2 * 3.141592653589793 * radius) / 4096  # 更新当前位置

        # 模拟传感器采集时间
        time.sleep(0.1)

if __name__ == "__main__":
    main()
