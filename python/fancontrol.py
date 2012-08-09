#!/usr/bin/python
#
# -*- coding: utf-8 -*-
# vim: set ts=4 sw=4 et sts=4 ai:

import sys
import csv
import serial

# 9600 baud, 8 bits data, 1 stop bit and no parity
port = serial.Serial(sys.argv[1], 9600, timeout=1)

# Sensors are in °C
# Buck converter outputs are in range 0 to 100
# Fan tachometer output in RPM
FCD = """
sensor_a,sensor_b,sensor_c,sensor_d,
buck_output_1,buck_output_2,buck_output_3,buck_output_4,
tacho_fan1a,
tacho_fan1b,
tacho_fan2a,
tacho_fan2b,
tacho_fan3a,
tacho_fan3b,
tacho_fan4a,
tacho_fan4b
"""

# Sensors: (0 = not connected, 1 = use °C, 2 = use °F)

fan_sensor_mapping = {
 1: "Sensor A"
 2: "Sensor B"
 3: "Sensor C"
 4: "Sensor D"
 5: "Sensor A - Sensor D"
 6: "Sensor B - Sensor D"
 7: "Sensor C - Sensor D"
 8: "Manual control (in that case the minimum power entry controls the speed of the fan)"
}


class FanPair(object):
    # Fan types
    NONE = 0
    2WIRE = 1
    3WIRE = 2
    4WIRE = 5

    # Sensor modes
    SENSOR_A = 1
    SENSOR_B = 2
    SENSOR_C = 3
    SENSOR_D = 4

    MANUAL = 8


    def sensor(self, sensor=None, delta=False, manual=False):
        assert (manual is True) or (sensor is not None)

    def range(self, start_at, max_at):
        assert max_at >= start_at
        assert start_at >= 0

    @staticmethod
    def check_power(power):
        assert power >=0 and power <= 100

    def set_min_power(self, power):
        self.check_power(power)
        self._min_power = power

    def get_min_power(self):
        return self._min_power

    allowed_stopped = True




FCR = """
sensor_a,sensor_b,sensor_c,sensor_d,
fan1_min_power
fan1_sensor
fan1_temp_min_speed
fan1_temp_max_speed
fan1_allow_stopped
fan1a_type
fan1b_type
fan2_min_power
fan2_sensor
fan2_temp_min_speed
fan2_temp_max_speed
fan2_allow_stopped
fan2a_type
fan2b_type
fan3_min_power
fan3_sensor
fan3_temp_min_speed
fan3_temp_max_speed
fan3_allow_stopped
fan3a_type
fan3b_type
fan4_min_power
fan4_sensor
fan4_temp_min_speed
fan4_temp_max_speed
fan4_allow_stopped
fan4a_type
fan4b_type
"""


