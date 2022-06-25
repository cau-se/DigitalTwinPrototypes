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

from picarx.interfaces.actuators import DCMotorInterface , TravelDirection, MotorSide
from picarx.gpio import Direction, GPIO
from picarx.pwm import PWM

class AbstractDCMotorDriver(DCMotorInterface):
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
        self.__direction_pin = GPIO().setup(pin_number, direction=Direction.OUT, callback=None)

    @property
    def pwm_pin(self):
        return self.__pwm_pin

    @pwm_pin.setter
    def pwm_pin(self, config: dict):
        self.__pwm_pin = PWM(channel=config['channel'], i2c_port=config['i2c_port'])
        self.__pwm_pin.period = 4095
        self.__pwm_pin.prescaler = 8

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
            self.__direction = direction
            if direction.value != self.motor_side.value:
                self.direction_pin.on() # forward
            else:
                self.direction_pin.off() # backward

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, speed: int):
        if speed is None:
            self.__speed = 0
            return

        if  speed in range(1,15) or speed > 100:
            raise ValueError("Speed must be 0 or between 15 and 100, you entered {}".format(speed))

        self.__speed = speed
        self.pwm_pin.duty_cycle = speed

    def drive_with_speed(self, speed: int):
        self.speed = speed

    @abstractmethod
    def start(self):
        raise NotImplementedError("The method {} is not implemented.".format('start'))

    @abstractmethod
    def stop(self):
        raise NotImplementedError("The method {} is not implemented.".format('stop'))


class DCMotor(AbstractDCMotorDriver):

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], motor_side: MotorSide = MotorSide.LEFT):
        super(DCMotor, self).__init__(name, direction_pin, pwm_pin, motor_side)

    def start(self):
        pass

    def stop(self):
        self.drive_with_speed(0)