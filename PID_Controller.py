import time
from math import fabs
from numpy import sign, log1p
from scipy.misc import derivative
# import csv


def measure(control, reading, iroc): # will take reading from sensor later, simulates temperature fn
    return reading + iroc + control*0.15


def calc_err(target, measured_value):
    return target - measured_value


def differential(err1, err2, delta):
    return (err2 - err1)/delta


def outputcalc(kp, ki, kd, error, integral, diffntl):
    error *= kp
    integral *= ki
    diffntl *= kd
    return error + integral + diffntl


class PID:

    def control(self, setpoint, measurement, delta, kp, ki, kd, timeout):
        # open csv and write header row
        # with open('pid.csv', 'wb') as csvfile:
        #     pidwriter = csv.writer(csvfile, delimiter=',',
        #                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #     pidwriter.writerow(['Interval', 'Delta', 'Target', 'Kp',
        #                        'Ki', 'Kd', 'Measured',
        #                        'Error1', 'Error2',
        #                        'Integral', 'Differential', 'Output'])

        # Get user input
        # print("Welcome to the PID simpulator.")
        # setpoint = float(input("Enter a desired temperature: "))
        # measurement = float(input("Enter an initial measurement: "))
        measure_prime = measurement
        # delta = fabs(float(input("Enter a delta T (change in time between samples): ")))
        # kp = float(input("Enter proportional gain: "))
        # ki = float(input("Enter integral gain: "))
        # kd = float(input("Enter differential gain: "))

        # set up variables for bounded exponential growth formula y=C(1-e^(-kt))
        # note that t was arbitrarily chosen to equal 5.0 to calculate k
        c = fabs(measure_prime - setpoint) * fabs(measure_prime - setpoint + 1.0) + 10.0
        k = ((-1.0/5.0) * log1p((1.0-(measure_prime/c)-1.0)))
        e = 2.718281828

        # define bounded exponential growth formula
        def f(x):
            return c * (1-e**(-k*x))

        # initialize variables used in PID controller
        err1 = 0
        interval = 0.0
        err2 = calc_err(setpoint, measurement)
        integral = 0.0
        diffntl = differential(err1, err2, delta)
        output = outputcalc(kp, ki, kd, err1, integral, diffntl)

        # limit controller output to (-100.0, 100.0) for use as PWM duty cycle percentage
        if fabs(output) > 100.0:
            output = 100.0 * sign(output)
        round(output, 1)

        # write values to csv
        # pidwriter.writerow([interval, delta, setpoint, kp, ki, kd, measurement, err1, err2, integral, diffntl, output])
        # output = 0.0
        err1 = err2
        counter = 0
        timeout += time.time()

        while counter < 100:
            interval += delta # increments of user's chosen delta T

            # solve exponential function for t
            t1 = -log1p(1.0 - (measurement/c) - 1.0) / k
            iroc = derivative(f, t1) # calculate instantaneous rate of change at t
            measurement = measure(output, measurement, iroc) # simulate temperature measurement
            err2 = calc_err(setpoint, measurement)
            integral += err1*delta
            diffntl = differential(err1, err2, delta)
            output = outputcalc(kp, ki, kd, err1, integral, diffntl)
            if fabs(output) > 100.0:
                output = 100.0 * sign(output)

            # pidwriter.writerow([interval, delta, setpoint, kp, ki, kd, measurement, err1, err2, integral, diffntl, output])
            err1 = err2
            if round(measurement, 2) == round(setpoint, 2):
                counter += 1
            if time.time() > timeout:
                interval = 'INF'
                break

        # print("It took " + str(interval) + " seconds to stabilize.")
        return interval

'''
try:
    main()

except KeyboardInterrupt:
    pass
'''
