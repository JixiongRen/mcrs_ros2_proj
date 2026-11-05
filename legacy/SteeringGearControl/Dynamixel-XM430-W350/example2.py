import motor
import time
while True:#Conecting Loop

    motor.serial_port.port = 'COM8'
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


#default velocity limite is 350 or 80.15RPM
motor.reboot(1)
time.sleep(1)
motor.motor_id = 1

motor.extended_position_mode()


motor.LED_control('on')

while True:
    '''
    motor.LED_control(1,0)
    motor.send_goal_pos(1, 2048-1000)
    time.sleep(1)
    motor.LED_control(1, 1)
    motor.send_goal_pos(1,2048+1000)
    time.sleep(1)
    '''

    value1 = motor.read_current_milliamp()


    print(value1)
    if value1 == None:
        print('error detected')
        time.sleep(1)
        motor.velocity_mode()
        motor.send_goal_RPM(80)
    time.sleep(0.1)




