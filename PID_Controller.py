"""
PID controller built from Wikipedia's pseudocode on PID Control and improved upon using Brett Beauregard's guide:
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

CSV logging and a changing setpoint simulation may be toggled on when class PID is initialized, but default to off:
    e.g. controller = PID('y') for csv logging only, ('n','y') for changing setpoint only, ('y', 'y') for both
"""

from distutils.util import strtobool
import time
from temp_measure_simulator import VirtualSensor
from PID_CSV_Writer import ControlWriter


class PID:
    def __init__(self, togglecsv=None, togglechangingsetpoint=None):
        if togglechangingsetpoint is None:
            self.changing = False
        else:
            self.changing = strtobool(togglechangingsetpoint)
        if togglecsv is None:
            self.csvwriting = False
        else:
            self.csvwriting = strtobool(togglecsv)

    def control(self, setpoint, measurement, deltat, kp, ki, kd, timeout):

        # initialize the csv writer if desired
        if self.csvwriting:
            csvwriter = ControlWriter()

        # setup sensor
        sensor = VirtualSensor(setpoint, measurement)

        # initialize variables used in PID controller
        interval = 0.0
        integral = 0.0
        output = 0.0
        timeout += time.time()  # sets real-world time limit to kill an infinite loop
        counter = 0
        countermax = 10

        # initializes counters if a simulated change to setpoint is desired
        if self.changing:
            countermax = 100
            run_once = 0

        while counter < countermax:     # main loop

            lastmeasure = measurement
            measurement = sensor.gettemp(output, lastmeasure)
            error = setpoint - measurement
            integral += -ki*error*deltat
            if integral > 100.0:
                integral = 100.0
            elif integral < 0.0:
                integral = 0.0
            differential = (lastmeasure - measurement)/deltat
            output = kp * error + integral - kd * differential
            
            # Sanitize output for PWM duty cycle %
            if output > 100.0:
                output = 100.0
            elif output < 0.0:
                output = 0.0

            # if desired, write the current values to csv file
            if self.csvwriting:
                csvwriter.addrow(interval, deltat, setpoint, kp, ki, kd, measurement, error, integral, differential,
                                 output)

            interval += deltat  # increments of user's chosen delta T
            if round(measurement, 1) == round(setpoint, 1):  # counts how many iterations measured temp matches target
                counter += 1

            # simulate changing the target temperature if desired
            if self.changing:
                if counter == 25 and run_once == 0:  # change setpoint after 25 low error samples
                    setpoint += 10
                    run_once += 1
                if counter == 50 and run_once == 1:
                    setpoint += 10
                    run_once += 1
                if counter == 75 and run_once == 2:
                    setpoint -= 40
                    run_once += 1

            if time.time() > timeout:
                interval = 'INF'
                break

        return interval
