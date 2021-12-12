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
from picarx.interfaces.actuators import ServoInterface, SunFounderClutchGear
from picarx.pwm import PWM
from twisted.internet import reactor
import time

class ServoEmulator(ServoInterface):

    def __init__(self, pwm_pin, frequency = 50):
        self.pwm = pwm_pin
        self.frequency = frequency # in Hertz

    @property
    def pwm(self):
        return self.__pwm

    @pwm.setter
    def pwm(self, pin_number):
        self.__pwm = PWM(pin_number)
        self.__pwm.period = 4095
        self.__pwm.prescaler =  8

    @property
    def frequency(self):
        return self.__frequency

    @frequency.setter
    def frequency(self, frequency):
        self.__frequency = 1/frequency

    @property
    def angle(self):
        return self.__angle

    @angle.setter
    def angle(self, angle: Union[int, float]):
        self.__angle = angle

    def angle_to_pulse_width(self, angle):
        pass

    def angle_to_pulse_width(self, angle):
        pulse_width = angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value - SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value
        # SunFounder used this line. I don't know why it works, but I haven't found another way
        pulse_width = pulse_width / SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * self.pwm.period
        return int(pulse_width)

    def pulse_width_to_angle(self, pulse_width):
        pulse_width = pulse_width / self.pwm.period * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value
        angle = (pulse_width - SunFounderClutchGear.MAXIMUM_PULSE.value) / (SunFounderClutchGear.MAXIMUM_PULSE.value - SunFounderClutchGear.MINIMUM_PULSE.value) * 180
        return int(angle)

    def __rotate_by_pulse_width(self, pulse_width):
        angle = self.pulse_width_to_angle(pulse_width)
        self.__rotate(angle)

    def __rotate_by_angle(self, angle):
        self.__rotate(angle)

    def __rotate(self, angle):
        print("Rotating by angle: {}".format(angle))

    def __read_i2c_value(self):
        try:
            while reactor.running:
                time_before = time.time()

                pulse_width = self.pwm.read()
                pulse_width = int.from_bytes(pulse_width.to_bytes(2, 'big'), 'little')
                self.__rotate_by_pulse_width(pulse_width)

                while abs(time.time() - time_before) < self.frequency:
                    time.sleep(0.001)  # precision here
        except KeyboardInterrupt:
            print("WTF")
            reactor.stop()

    def run(self):
        reactor.callInThread(self.__read_i2c_value)
        if not reactor.running:
            reactor.run()