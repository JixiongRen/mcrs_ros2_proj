import numpy
import PID
import matplotlib.pyplot as plt

set = 100
i=0
error = 100
out = 0
out_list = []
set_list = []
error_list = []
out_list.append(out)
Kp=0.99
Ki=0.0001
Kd=0.0005
pid_controller = PID.PIDController(Kp, Ki, Kd)
#pid_controller = PID.PIDController(Kp=0.99, Ki=0.0001, Kd=0.005)
while abs(error)>0.1 :
    error = set - out
    add = pid_controller.calculate(error)
    out = out + add
    out_list.append(out)
    set_list.append(set)
    error_list.append(error)
    i+=1

out = 150
out_list.append(out)
error = 50
set_list.append(set)
error_list.append(error)
while abs(error)>0.1:
    error = set - out
    add = pid_controller.calculate(error)
    out = out + add
    out_list.append(out)
    set_list.append(set)
    error_list.append(error)
    i+=1

plt.figure()
plt.grid(True, linestyle='--',alpha=0.2)
plt.title('Kp='+str(Kp)+' Ki='+str(Ki)+' Kd='+str(Kd))
plt.plot(out_list, label='out')
plt.plot(set_list,linestyle='--',alpha=0.5, label='set')
plt.plot(error_list, linestyle=':',label='error')
plt.legend()
plt.xlabel('Times / 1')
plt.ylabel('value / 1')
plt.show()


