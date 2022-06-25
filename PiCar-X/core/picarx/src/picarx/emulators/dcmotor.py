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

from picarx.gpio import Direction, GPIO
from picarx.pwm import PWM
from abc import ABCMeta, abstractmethod
from picarx.interfaces.actuators import MotorSide, TravelDirection


class AbstractMotorEmulator(metaclass=ABCMeta):

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], i2c_port: str = '\dev\i2c-1', motor_side: MotorSide = MotorSide.LEFT):
        self.name = name
        self.direction_pin = direction_pin
        self.pwm_pin = {'channel': pwm_pin, 'i2c_port': i2c_port}
        self.motor_side = motor_side
        self.speed = None
        self.direction = TravelDirection.FORWARD

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    @property
    def direction_pin(self):
        return self.__direction_pin

    @direction_pin.setter
    def direction_pin(self, pin_number):
        self.__direction_pin = GPIO().setup(pin_number, direction=Direction.OUT,
                                            callback=self.change_direction_listener, emulator=True)

    @property
    def pwm_pin(self):
        return self.__pwm_pin

    @pwm_pin.setter
    def pwm_pin(self, config: dict):
        self.__pwm_pin = PWM(
            channel=config['channel'], i2c_port=config['i2c_port'])
        self.pwm_pin.period = 4095
        self.pwm_pin.prescaler = 8

    @property
    def motor_side(self):
        return self.__motor_side

    @motor_side.setter
    def motor_side(self, motor_side: MotorSide):
        self.__motor_side = motor_side

    @property
    def direction(self):
        return self.__direction

    @direction.setter
    def direction(self, direction: TravelDirection):
        if direction is None:
            self.__direction = None
        else:
            if direction.value != self.motor_side.value:
                self.__direction = 1 # forward
            else:
                self.__direction = -1 # backward

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, speed: int):
        if speed is None:
            self.__speed = 0
            return

        if speed > 100 or speed < 15:
            raise ValueError(
                "Speed must be between 15 and 100, you entered {}".format(speed))

        self.__speed = speed
        self.pwm_pin.duty_cycle = speed

    @abstractmethod
    def change_direction_listener(self, event):
        raise NotImplementedError(
            "The method {} is not implemented.".format('change_direction_listener'))

    @abstractmethod
    def drive_with_speed(self, i2c_value: int):
        raise NotImplementedError(
            "The method {} is not implemented.".format('change_direction_listener'))

    @abstractmethod
    def start(self):
        raise NotImplementedError(
            "The method {} is not implemented.".format('start'))

    @abstractmethod
    def stop(self):
        raise NotImplementedError(
            "The method {} is not implemented.".format('stop'))


class MotorEmulator(AbstractMotorEmulator):

    def __init__(self, *args, **kwargs):
        super(MotorEmulator, self).__init__(*args, **kwargs)

    def change_direction_listener(self, event):
        if event.event_type is 'modified':
            direction = self.direction_pin.value
            if direction == TravelDirection.FORWARD.value:
                self.drive_forward()
            elif direction == TravelDirection.BACKWARD.value:
                self.drive_backwards()

    def drive_with_speed(self, i2c_value: int):
        percentage = int((i2c_value/4095) * 100)
        print("Moving with {} percent speed".format(percentage))

    def start(self):
        pass

    def stop(self):
        pass
