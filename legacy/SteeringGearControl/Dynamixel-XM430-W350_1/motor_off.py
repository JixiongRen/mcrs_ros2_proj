import motor
import time
while True:#Conecting Loop

    #motor.serial_port.port = 'COM6'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 1
    try:
        motor.serial_port.open()
        break
    except:
        print('Serial error')



print('connected')
time.sleep(2)

print('motor_off')


#default velocity limite is 350 or 80.15RPM
#reboot(1)
motor.motor_id = 1
#motor.position_mode()
motor.motor_off()




