# import spidev 
# import time

# channel = 1
# spi = spidev.SpiDev()
# spi.open(0,0)
# spi.max_speed_hz = 9600
# while True:
#     adc = spi.xfer2([1,(8+channel)<<4,0])
#     data = ((adc[1]&3) << 8) + adc[2]
#     dis_exec = (data * 1500) / float(1023) * 1500
#     print("Temp_dis : {} mm".format(dis_exec))
#     time.sleep(0.1)

# Importing modules
import spidev # To communicate with SPI devices
from time import sleep  # To add delay
# Start SPI connection
spi = spidev.SpiDev() # Created an object
spi.open(0,0) 

# Read MCP3008 data
def analogInput(channel):
  spi.max_speed_hz = 9600
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data
# Below function will convert data to voltage
def Dis(data):
  dis = (data * 1500) / float(1023) * 1500
  return dis
 
# Below function will convert data to temperature.
# def Temp(data):
#   temp = ((data * 330)/float(1023))-50
#   temp = round(temp)
#   return temp
while True:
  temp_output = analogInput(0) # Reading from CH0
  temp_dis  = Dis(temp_output)
  print("Temp_dis : {} mm".format(temp_output))
  sleep(0.1)

