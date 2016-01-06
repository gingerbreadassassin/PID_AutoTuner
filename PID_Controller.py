from distutils.util import strtobool
import time
from math import fabs
from numpy import sign, log1p
from scipy.misc import derivative
# import csv


def measure(control, reading, iroc):  # will take reading from sensor later, simulates temperature fn
    return reading + iroc + control*0.15


def calc_err(target, measured_value):
    return target - measured_value


# def differential(err1, err2, delta):
#     return (err2 - err1)/delta


# def outputcalc(kp, ki, kd, error, integral, diffntl):
#     error *= kp
#     integral *= ki
#     diffntl *= kd
#     return error + integral + diffntl


class PID:
    @staticmethod
    def control(setpoint, measurement, delta, kp, ki, kd, timeout, csvwrite):
        if strtobool(csvwrite):
            import csv
            # open csv and write header row
            with open('pid.csv', 'w', encoding='utf8', newline='') as csvfile:
                pidwriter = csv.writer(csvfile, delimiter=',',
                                       quotechar='|', quoting=csv.QUOTE_MINIMAL)
                pidwriter.writerow(['Interval', 'Delta', 'Target', 'Kp',
                                   'Ki', 'Kd', 'Measured',
                                    'Error1', 'Error2',
                                    'Integral', 'Differential', 'Output'])

        # set up variables for bounded exponential growth formula y=C(1-e^(-kt))
        # note that t was arbitrarily chosen to equal 5.0 to calculate k
        c = fabs(measurement - setpoint) * fabs(measurement - setpoint + 1.0) + 10.0
        k = ((-1.0/5.0) * log1p((1.0-(measurement/c)-1.0)))
        e = 2.718281828

        # define bounded exponential growth formula
        def f(x):
            return c * (1-e**(-k*x))

        # initialize variables used in PID controller
        err1 = 0
        interval = 0.0
        err2 = calc_err(setpoint, measurement)
        integral = 0.0
        # diffntl = differential(err1, err2, delta)
        diffntl = -err2/delta
        output = kp * err2 + integral + kd * diffntl

        # limit controller output to (-100.0, 100.0) for use as PWM duty cycle percentage
        if fabs(output) > 100.0:
            output = 100.0 * sign(output)
        round(output, 1)

        if strtobool(csvwrite):
            # write values to csv
            with open('pid.csv', 'a', encoding='utf8', newline='') as csvfile:
                pidwriter = csv.writer(csvfile, delimiter=',',
                                       quotechar='|', quoting=csv.QUOTE_MINIMAL)
                pidwriter.writerow([interval, delta, setpoint, kp, ki, kd, measurement, err1, err2, integral, diffntl,
                                    output])
        err1 = err2
        counter = 0
        timeout += time.time()
        run_once = 0

        while counter < 100:
            interval += delta  # increments of user's chosen delta T

            if counter == 25 and run_once == 0:  # change setpoint after 25 'close' samples
                setpoint += 5
                ki += 0.01
                run_once = 1

            # solve exponential function for t
            t1 = -log1p(1.0 - (measurement/c) - 1.0) / k
            iroc = derivative(f, t1)  # calculate instantaneous rate of change at t
            lastmeasure = measurement
            measurement = measure(output, measurement, iroc)  # simulate temperature measurement
            err2 = calc_err(setpoint, measurement)
            integral += ki*err1*delta
            #diffntl = differential(err1, err2, delta)
            diffntl = -(measurement - lastmeasure)/delta
            output = kp * err2 + integral + kd * diffntl
            if fabs(output) > 100.0:
                output = 100.0 * sign(output)

            if strtobool(csvwrite):
                with open('pid.csv', 'a', encoding='utf8', newline='') as csvfile:
                    pidwriter = csv.writer(csvfile, delimiter=',',
                                           quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    pidwriter.writerow([interval, delta, setpoint, kp, ki, kd, measurement, err1, err2, integral,
                                        diffntl, output])
            err1 = err2
            if round(measurement, 2) == round(setpoint, 2):
                counter += 1
            if time.time() > timeout:
                interval = 'INF'
                break

        # print("It took " + str(interval) + " seconds to stabilize.")
        return interval
