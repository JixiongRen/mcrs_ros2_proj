import time

def print_at_position(x, y, text):
    # 使用 ANSI 转义码将光标移动到指定位置
    print(f"\033[{y};{x}H{text}", end='', flush=True)

# 设置循环次数
iterations = 10

for i in range(iterations):
    # 清屏
    print("\033c", end='', flush=True)
    
    # 获取当前时间
    current_time = time.strftime("%H:%M:%S")
    
    # 打印内容在指定位置
    print_at_position(10, 10, f"当前时间：{current_time}")
    
    # 等待一段时间
    time.sleep(1)
