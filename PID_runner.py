"""
Just a script to execute the PID controller for manual testing
"""

from PID_Controller import PID

output = PID('y')

setpoint = 62.0
measurement = 75.0
deltat = 0.1
kp = 0.01197999999999992  # 0.01
ki = 0.23023000000000024  # 0.23
kd = 0.14960000000002344  # 0.122
timeout = 10.0

time_to_stable_1 = output.control(setpoint, measurement, deltat, kp, ki, kd, timeout)

print(time_to_stable_1)
