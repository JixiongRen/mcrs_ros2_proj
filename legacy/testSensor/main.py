import os
import math
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# 创建数据总线
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# 创建 cs
cs = digitalio.DigitalInOut(board.CE0)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

# # create an analog input channel on pin 1
# chan1 = AnalogIn(mcp, MCP.P1)

while True:
    print('light: %d, %.3fV' % (
        chan0.value, chan0.voltage,
        #chan1.value, lmt86_v2t(chan1.voltage * 1000),
    ))
    time.sleep(0.5)