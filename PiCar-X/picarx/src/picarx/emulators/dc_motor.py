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

from picarx.gpio import Direction, GPIO
from picarx.pwm import PWM
from enum import Enum
from abc import ABCMeta, abstractmethod

class TravelDirection(Enum):
    FORWARD = 0
    BACKWARD = 1

class MotorSide(Enum):
    LEFT = 0
    RIGHT = 1

class AbstractMotorEmulator(metaclass=ABCMeta):

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], motor_side: MotorSide = MotorSide.LEFT):
        self.name = name
        self.direction_pin = direction_pin
        self.pwm_pin = pwm_pin
        self.motor_side = motor_side
        self.speed = 0
        self.pwm_pin.period = 4095
        self.pwm_pin.prescaler = 8
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
        self.__direction_pin = GPIO.setup(pin_number, direction=Direction.OUT, callback=self.change_direction_listener, emulator=True)

    @property
    def pwm_pin(self):
        return self.__pwm_pin

    @pwm_pin.setter
    def pwm_pin(self, pin_number):
        self.__pwm_pin = PWM(pin_number)

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
            print("Motor_sode: {} - direction: {} - Pin Direction: {}".format(self.motor_side.value, direction.value, self.direction_pin.direction.value))
            if direction.value != self.motor_side.value:
                #print("Motor_side: {} - direction: {}".format(self.motor_side.value, direction.value))
                self.direction_pin.on() # forwards
            else:
                #print("Motor_side: {} - direction: {}".format(self.motor_side.value, direction.value))
                self.direction_pin.off() #backwards
        #self.__db.set('{}_travel_direction'.format(self.name), direction.value)

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, speed: int):
        if speed > 100 or speed < 0:
            raise ValueError("Speed must be between 0 and 100, you entered {}".format(speed))

        self.__speed = speed

    @abstractmethod
    def change_direction_listener(self, event):
        raise NotImplementedError("The method {} is not implemented.".format('change_direction_listener'))

    @abstractmethod
    def drive_forward(self, speed):
        raise NotImplementedError("The method {} is not implemented.".format('drive_forward'))

    @abstractmethod
    def drive_backwards(self, speed):
        raise NotImplementedError("The method {} is not implemented.".format('drive_backwards'))

    def drive_by_duty_cycle(self, percent):
        print("Motor_side: {} - direction: {}".format(self.motor_side.value, self.direction_pin.value))
        self.pwm_pin.duty_cycle = percent

    def stop(self):
        self.speed = 0
        self.pwm_pin.pulse_width(0)


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