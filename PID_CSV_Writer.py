"""
A simple script to separate the csv stuff from the PID controller
"""

import csv


class ControlWriter:

    def __init__(self):
        with open('pid.csv', 'w', encoding='utf8', newline='') as csvfile:
                pidwriter = csv.writer(csvfile, delimiter=',',
                                       quotechar='|', quoting=csv.QUOTE_MINIMAL)
                pidwriter.writerow(['Interval', 'Target', 'Measured', 'Proportional',
                                    'Integral', 'Differential', 'Output', 'Delta', 'Kp', 'Ki', 'Kd'])

    @staticmethod
    def addrow(interval, delta, setpoint, kp, ki, kd, measurement, error, integral, diffntl, output):
        with open('pid.csv', 'a', encoding='utf8', newline='') as csvfile:
                pidwriter = csv.writer(csvfile, delimiter=',',
                                       quotechar='|', quoting=csv.QUOTE_MINIMAL)
                pidwriter.writerow([interval, setpoint, measurement, error, integral, diffntl,
                                    output, delta, kp, ki, kd])
