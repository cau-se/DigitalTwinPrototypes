# Copyright 2022 Alexander Barbie
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

from abc import abstractmethod
from typing import Union
from picarx.interfaces.actuators import ClutchGearInterface, SunFounderClutchGear
from picarx.pwm import PWM
from twisted.internet import reactor
import time

class AbstractClutchGearEmulator(ClutchGearInterface):

    def __init__(self, pwm_pin, i2c_port, frequency = 50):
        self.pwm_pin = {'channel': pwm_pin, 'i2c_port': i2c_port}
        self.frequency = frequency # in Hertz

    @property
    def pwm_pin(self):
        return self.__pwm_pin

    @pwm_pin.setter
    def pwm_pin(self, config: dict):
        self.__pwm_pin = PWM(channel=config['channel'], i2c_port=config['i2c_port'])
        self.__pwm_pin.period = 4095
        self.__pwm_pin.prescaler =  8

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
        pulse_width = round(angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value - SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value)
        # I don't know why it works, but I haven't found another way
        pulse_width = pulse_width / SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * self.pwm_pin.period
        return round(pulse_width / 2)

    def pulse_width_to_angle(self, pulse_width):
        pulse_width = round(pulse_width * 2 * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value / self.pwm_pin.period)
        angle = (pulse_width - SunFounderClutchGear.MINIMUM_PULSE.value) * 180 / (SunFounderClutchGear.MAXIMUM_PULSE.value - SunFounderClutchGear.MINIMUM_PULSE.value)
        return round(angle)

    def rotate_by_pulse_width(self, pulse_width):
        angle = self.pulse_width_to_angle(pulse_width)
        self.rotate(angle)

    def rotate_by_angle(self, angle: Union[int, float]):
        self.rotate(angle)

    def rotate(self, angle):
        raise NotImplementedError("The method {} is not implemented.".format('__rotate'))

    def start(self):
        raise NotImplementedError("The method {} is not implemented.".format('start'))

    def stop(self):
        raise NotImplementedError("The method {} is not implemented.".format('stop'))

    @abstractmethod
    def read_i2c_value(self):
        raise NotImplementedError("The method {} is not implemented.".format('__read_i2c_value'))

class SimpleClutchGearEmulator(AbstractClutchGearEmulator):

    def __init__(self, pwm_pin, frequency = 50):
        super(SimpleClutchGearEmulator, self).__init__(pwm_pin, frequency)

    def rotate(self, angle):
        print("Rotating by angle: {}".format(angle))

    def read_i2c_value(self):
        try:
            while reactor.running:
                time_before = time.time()

                pulse_width = self.pwm_pin.read()
                pulse_width = int.from_bytes(pulse_width.to_bytes(2, 'big'), 'little')
                self.__rotate_by_pulse_width(pulse_width)

                while abs(time.time() - time_before) < self.frequency:
                    time.sleep(0.005)  # 50Hz
        except KeyboardInterrupt:
            reactor.stop()

    def start(self):
        reactor.callInThread(self.__read_i2c_value)
        if not reactor.running:
            reactor.run()

    def stop(self):
        reactor.stop()