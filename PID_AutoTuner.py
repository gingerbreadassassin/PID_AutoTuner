import PID_Controller

setpoint = 62.0
measurement = 75.0
deltat = 0.1
kp = 0.0 # 0.01 # 0.07 # 0.25
ki = 0.0 # 0.23 # 0.36
kd = 0.00
timeout = 1.0

output = PID_Controller.PID()

while True:
    infcount = 0
    time_to_stable_1 = output.control(setpoint, measurement, deltat, kp, ki, kd, timeout)
    # print(time_to_stable_1)
    if time_to_stable_1 is 'INF':
        infcount += 1
        ki += 0.00001
        print(ki)

    else:
        ki += 0.00001
        time_to_stable_2 = output.control(setpoint, measurement, deltat, kp, ki, kd, timeout)
        # print(time_to_stable_2)
        if time_to_stable_1 < time_to_stable_2:
            print(ki)
            break
