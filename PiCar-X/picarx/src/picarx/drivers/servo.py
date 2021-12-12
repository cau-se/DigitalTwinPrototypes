# Copyright 2021 Alexander Barbie
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Union
from picarx.pwm import PWM, SunFounderPWMValues
from picarx.interfaces.actuators import ServoInterface, SunFounderClutchGear

class Servo(ServoInterface):    
    def __init__(self, pwm_pin):
        self.pwm = pwm_pin
        self.angle = 90

    @property
    def pwm(self):
        return self.__pwm

    @pwm.setter
    def pwm(self, pin_number):
        self.__pwm = PWM(pin_number)
        self.__pwm.period = 4095
        self.__pwm.prescaler =  8

    @property
    def angle(self):
        return self.__angle

    @angle.setter
    def angle(self, angle: Union[int, float]):
        if not (isinstance(angle, int) or isinstance(angle, float)):
            raise ValueError("Angle value should be int or float value, not {}".format(type(angle)))

        if angle < 0:
            angle = 0
        elif 180 < angle:
            angle = 180

        self.__angle = angle

    def angle_to_pulse_width(self, angle):
        pulse_width = angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value - SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value
        # I don't know why it works, but I haven't found another way
        pulse_width = pulse_width / SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * self.pwm.period
        return int(pulse_width)

    def pulse_width_to_angle(self, pulse_width):
        pulse_width = pulse_width / self.pwm.period * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value
        angle = (pulse_width - SunFounderClutchGear.MAXIMUM_PULSE.value) / (SunFounderClutchGear.MAXIMUM_PULSE.value - SunFounderClutchGear.MINIMUM_PULSE.value) * 180
        return int(angle)

    def rotate_by_angle(self, angle: Union[int, float]):
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180

        pulse_width = self.angle_to_pulse_width(angle)
        self.pwm.pulse_width(pulse_width)
        self.__angle = angle