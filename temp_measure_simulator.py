"""
A virtual temperature sensor that creates a bounded exponential growth function based on input from PID_Controller.py
function: y=C(1-e^(-kt))
Where C = (difference between target and initial temperature) * (the same + 1) + 10
      k = -1/t * ln(1 - initial-temperature / C) @ t = 5 seconds

When the PID controller calls "gettemp", it passes the last measurement.
gettemp then figures out what time "t" this temperature would be, and calculates the instantaneous rate of change @ t
Finally, the sensor returns the last temperature + the rate of change - the PID controller's output times some constant
This constant's realistic accuracy is currently a matter of contention (i.e. TBD)
"""

from math import fabs, e
from numpy import log1p
from scipy.misc import derivative


class VirtualSensor:
    def __init__(self, setpoint, measurement):
        # set up variables for bounded exponential growth formula
        # note that t was arbitrarily chosen to equal 5.0 to calculate k
        self.c = fabs(setpoint - measurement) * (fabs(setpoint - measurement) + 1.0) + 10.0
        self.k = ((-1.0/5.0) * log1p((1.0-(measurement/self.c)-1.0)))

    # define bounded exponential growth formula (x = t)
    def f(self, x):
        return self.c * (1-e**(-self.k*x))

    def gettemp(self, control, lasttemp):
        t = -log1p(1.0 - (lasttemp/self.c) - 1.0) / self.k  # solve exponential function for t
        iroc = derivative(self.f, t)  # calculate instantaneous rate of change at t
        return lasttemp + iroc - control*0.15
