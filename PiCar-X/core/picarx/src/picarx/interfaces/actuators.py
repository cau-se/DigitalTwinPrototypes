#!/usr/bin/env python3
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
from abc import ABCMeta, abstractmethod
from enum import Enum
from typing import Union

class TravelDirection(Enum):
    FORWARD = 1
    BACKWARD = 0

class MotorSide(Enum):
    LEFT = 0
    RIGHT = 1

class SunFounderClutchGear(Enum):
    PULSE_PERIOD = 2000 # Every 20ms
    MIN_ANGLE = 0 # Degree
    MAX_ANGLE = 180 # Degree
    NEUTRAL_PULSE_LENGTH = 1500 # 1.5ms, will turn to 90 degree
    MINIMUM_PULSE = 500 # 0.5ms
    MAXIMUM_PULSE = 2500 # 2.5ms
    SUNFOUNDER_RANDOM_DIVIDER = 20000

class DCMotorInterface(metaclass=ABCMeta):

    @abstractmethod
    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], motor_side: MotorSide = MotorSide.LEFT):
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def name(self): raise NotImplementedError("Subclasses should implement this!")

    @name.setter
    @abstractmethod
    def name(self, name): raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def direction_pin(self): raise NotImplementedError("Subclasses should implement this!")

    @direction_pin.setter
    @abstractmethod
    def direction_pin(self, pin_number): raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def pwm_pin(self): raise NotImplementedError("Subclasses should implement this!")

    @pwm_pin.setter
    @abstractmethod
    def pwm_pin(self, pin_number): raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def motor_side(self): raise NotImplementedError("Subclasses should implement this!")

    @motor_side.setter
    @abstractmethod
    def motor_side(self, motor_side: MotorSide): raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def direction(self): raise NotImplementedError("Subclasses should implement this!")

    @direction.setter
    @abstractmethod
    def direction(self, direction: TravelDirection): raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def speed(self): raise NotImplementedError("Subclasses should implement this!")

    @speed.setter
    @abstractmethod
    def speed(self, speed: int): raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def stop(self): raise NotImplementedError("Subclasses should implement this!")

class ClutchGearInterface(metaclass=ABCMeta):    
    @abstractmethod
    def __init__(self, pwm_pin, i2c_port): raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def pwm_pin(self): raise NotImplementedError("Subclasses should implement this!")

    @pwm_pin.setter
    @abstractmethod
    def pwm_pin(self, pin_number): raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def angle(self): raise NotImplementedError("Subclasses should implement this!")

    @angle.setter
    @abstractmethod
    def angle(self, angle: Union[int, float]): raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def angle_to_pulse_width(self, angle): raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def pulse_width_to_angle(self, pulse_width): raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def rotate(self, angle): raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def start(self): raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def stop(self): raise NotImplementedError("Subclasses should implement this!")