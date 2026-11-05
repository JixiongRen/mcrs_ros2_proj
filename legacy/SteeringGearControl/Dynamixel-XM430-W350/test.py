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
motor.reboot(1)
time.sleep(1)

print('start')


motor.motor_id = 1
motor.extended_position_mode()


left = -1044479


print('pos',motor.read_position())
print('homing',motor.read_int(motor.control_dict['homing_offset'],4))

motor.send_homing_offset(left)

print('pos',motor.read_position())
print('homing',motor.read_int(motor.control_dict['homing_offset'],4))

motor.motor_off()
print('motor_off')
print('pos',motor.read_position())
print('homing',motor.read_int(motor.control_dict['homing_offset'],4))

motor.send_homing_offset(left)

print('pos',motor.read_position())
print('homing',motor.read_int(motor.control_dict['homing_offset'],4))


