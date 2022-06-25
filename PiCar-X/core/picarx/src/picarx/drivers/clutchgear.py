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

from typing import Union
from picarx.pwm import PWM
from picarx.interfaces.actuators import ClutchGearInterface, SunFounderClutchGear
from abc import abstractmethod


class AbstractClutchGearDriver(ClutchGearInterface):
    def __init__(self, pwm_pin: str, i2c_port: str):
        self.pwm_pin = {'channel': pwm_pin, 'i2c_port': i2c_port}
        self.angle = 90

    @property
    def pwm_pin(self):
        return self.__pwm_pin

    @pwm_pin.setter
    def pwm_pin(self, config: dict):
        self.__pwm_pin = PWM(
            channel=config['channel'], i2c_port=config['i2c_port'])
        self.__pwm_pin.period = 4095
        self.__pwm_pin.prescaler = 8

    @property
    def angle(self):
        return self.__angle

    @angle.setter
    def angle(self, angle: Union[int, float]):
        if not (isinstance(angle, int) or isinstance(angle, float)):
            raise ValueError(
                "Angle value should be int or float value, not {}".format(type(angle)))

        if angle < 0:
            angle = 0
        elif 180 < angle:
            angle = 180

        self.__angle = angle

    def angle_to_pulse_width(self, angle):
        pulse_width = round(angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value -
                            SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value)
        # I don't know why it works, but I haven't found another way
        pulse_width = pulse_width / \
            SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * self.pwm_pin.period
        return round(pulse_width / 2)

    def pulse_width_to_angle(self, pulse_width):
        pulse_width = round(
            pulse_width * 2 * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value / self.pwm_pin.period)
        angle = (pulse_width - SunFounderClutchGear.MINIMUM_PULSE.value) * 180 / \
            (SunFounderClutchGear.MAXIMUM_PULSE.value -
             SunFounderClutchGear.MINIMUM_PULSE.value)
        return round(angle)

    @abstractmethod
    def start(self):
        raise NotImplementedError(
            "The method {} is not implemented.".format('start'))

    @abstractmethod
    def stop(self):
        raise NotImplementedError(
            "The method {} is not implemented.".format('stop'))

    @abstractmethod
    def rotate(self, angle):
        raise NotImplementedError(
            "The method {} is not implemented.".format('rotate'))


class SimpleClutchGear(AbstractClutchGearDriver):

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str]):
        super(SimpleClutchGear, self).__init__(name, direction_pin, pwm_pin)

    def rotate(self, angle: Union[int, float]):
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180

        pulse_width = self.angle_to_pulse_width(angle)
        self.pwm_pin.pulse_width(pulse_width)
        self.angle = angle

    def start(self):
        pass

    def stop(self):
        pass
