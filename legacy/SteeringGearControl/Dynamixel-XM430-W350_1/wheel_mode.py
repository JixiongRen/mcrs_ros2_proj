import motor
import time

while True:#Conecting Loop

    motor.serial_port.port = '/dev/ttyUSB1'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 1
    try:
        motor.serial_port.open()
        break
    except:
        print('Serial error')



print('connected')
time.sleep(2)

print('start')
#motor.move_pos(1000)


motor.motor_id = 1
#motor.position_mode()
motor.velocity_mode()
motor.send_goal_RPM(45)

time.sleep(1)

while True:


    value1 = motor.read_position()


    print(value1)
    if value1 == None:
        print('error detected')
        time.sleep(1)
        motor.velocity_mode()
        motor.send_goal_RPM(45)
    time.sleep(0.1)

