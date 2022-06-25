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

from enum import Enum, EnumMeta
from typing import Union
from twisted.internet import reactor
import os
import weakref
import time
import logging
from watchdog.events import FileSystemEventHandler
from datetime import datetime, timedelta
from watchdog.observers import Observer


BUFFER_LEN = 100

class GPIOEnumMeta(EnumMeta):
    def __contains__(cls, item):
        return item in [v.value for v in cls.__members__.values()]


class Active(Enum, metaclass=GPIOEnumMeta):
    ON = 1
    OFF = 0


class Value(Enum, metaclass=GPIOEnumMeta):
    HIGH = 1
    LOW = 0


class Direction(Enum, metaclass=GPIOEnumMeta):
    OUT = 'out'
    IN = 'in'


class Edge(Enum, metaclass=GPIOEnumMeta):
    RISING = 'rising'
    FALLING = 'falling'
    NONE = 'none'
    BOTH = 'both'


class Pin(FileSystemEventHandler):
    SYSFS_BASE_PATH = '/sys/class/gpio'
    SYSFS_GPIO_EXPORT = '/sys/class/gpio/export'
    SYSFS_GPIO_UNEXPORT = '/sys/class/gpio/unexport'
    SYSFS_GPIO_PIN_PATH = '/sys/class/gpio/gpio{}'
    SYSFS_GPIO_PIN_DIRECTION = '/sys/class/gpio/gpio{}/direction'
    SYSFS_GPIO_PIN_VALUE = '/sys/class/gpio/gpio{}/value'
    SYSFS_GPIO_PIN_EDGE = '/sys/class/gpio/gpio{}/edge'
    SYSFS_GPIO_PIN_ACTIVE_LOW = '/sys/class/gpio/gpio{}/active_low'

    _dict = {
        "BOARD_TYPE": 12,
    }

    GPIOPins = {
        "D0":  17,
        "D1":  18,
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,
        "D7":  4,
        "D8":  5,
        "D9":  6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  19,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 21,
    }

    def __init__(self, pin_number: Union[int, str], direction=Direction.OUT, edge=Edge.NONE, active=Active.ON, callback = None, emulator=False):
        self.__emulator = emulator
        self.pin_number = pin_number

        while not os.path.exists(Pin.SYSFS_GPIO_PIN_PATH.format(self.pin_number)):
            time.sleep(0.1)

        if not emulator:
            self.direction = direction
            self.edge = edge
            self.active_low = active
        else:
            self.__init_existing()

        self.value = None
        self.callback = callback
        self.last_modified = datetime.now()
        self._finalize = weakref.finalize(self, self.__unexport, self.pin_number)

    def __init_existing(self):
        logging.info("Reading Pin for Emulator")
        time.sleep(2)
        #self.edge = Edge(self.__read_path(
        #    Pin.SYSFS_GPIO_PIN_EDGE.format(self.pin_number)))
        #self.direction = Direction(self.__read_path(
        #    Pin.SYSFS_GPIO_PIN_DIRECTION.format(self.pin_number)))
        #self.active_low = Active(self.__read_path(
        #    Pin.SYSFS_GPIO_PIN_ACTIVE_LOW.format(self.pin_number)))

    def on_modified(self, event):
        if datetime.now() - self.last_modified < timedelta(seconds=1):
            return
        else:
            self.last_modified = datetime.now()

        if callable(self.callback):
            self.callback(event)

    def __read_path(self, path):
        if os.path.isdir(path):
            path_reader = open(path, 'r')
            value = path_reader.read()
            path_reader.seek(0)
            path_reader.close()
            return value
        else:
            raise ValueError("The path {} does not exist".format(path))

    @property
    def pin_number(self):
        return self.__pin_number

    @pin_number.setter
    def pin_number(self, pin_number: Union[int, str]):
        if isinstance(pin_number, str):
            if pin_number in self.GPIOPins.keys():
                self.__pin_number = self.GPIOPins[pin_number]
        else:
            self.__pin_number = pin_number

        if not self.__emulator:
            self.__export()

    @property
    def direction(self):
        return self.__direction

    @direction.setter
    def direction(self, direction):
        if direction.value in Direction:
            if not self.__emulator:
                with open(Pin.SYSFS_GPIO_PIN_DIRECTION.format(self.pin_number), "w") as pin:
                    pin.write(direction.value)
            self.__direction = Direction(direction)

    @property
    def edge(self):
        return self.__edge

    @edge.setter
    def edge(self, edge):
        if edge in Edge:
            if not self.__emulator:
                with open(Pin.SYSFS_GPIO_PIN_EDGE.format(self.pin_number), "w") as pin:
                    pin.write(edge)
            self.__edge = edge

    @property
    def value(self):
        with open(Pin.SYSFS_GPIO_PIN_VALUE.format(self.pin_number), 'r') as value_reader:
            value = value_reader.read()
            if value != '':
                value = int(value)
            return value

    @value.setter
    def value(self, val):
        if val is not None:
            if val in Value:
                with open(Pin.SYSFS_GPIO_PIN_VALUE.format(self.pin_number), 'w') as value_writer:
                    value_writer.write("{}".format(val))

    @property
    def callback(self):
        return self.__callback
    
    @callback.setter
    def callback(self, callback):
        if callable(callback) or callback is None:
            self.__callback = callback
        else:
            raise TypeError("Your callback is not callable.")

    @property
    def active_low(self):
        return self.__active_low

    @active_low.setter
    def active_low(self, active_low_mode):
        if active_low_mode in Active:
            if not self.__emulator:
                with open(Pin.SYSFS_GPIO_PIN_ACTIVE_LOW.format(self.pin_number), "w") as pin:
                    pin.write(active_low_mode)
            self.__active_low = active_low_mode

    @property
    def value_reader(self):
        return self.__value_reader

    @value_reader.setter
    def value_reader(self, path):
        if path is None:
            if hasattr(self, 'value_reader'):
                self.__value_reader.close()
            self.__value_reader = None
        else:
            self.__value_reader = open(path, "r+")

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def high(self):
        self.on()

    def low(self):
        self.off()

    def __export(self):
        if not self.__emulator:
            with open(Pin.SYSFS_GPIO_EXPORT, "w") as export:
                export.write('%d' % self.pin_number)
            logging.debug("Exported GPIO{}".format(self.pin_number))
        else:
            logging.debug("Did not export the pin {} since I am an emulator".format(self.pin_number))

    def __unexport(self, *args):
        if not self.__emulator:
            with open(Pin.SYSFS_GPIO_UNEXPORT, "w") as unexport:
                unexport.write('%d' % self.pin_number)
            logging.debug("Unexported GPIO{}".format(self.pin_number))
        else:
            logging.debug("I am an emulator, thus I will not unexport pin {}".format(self.pin_number))

    def start_watchdog(self):
        if callable(self.callback):
            observer = Observer()
            observer.schedule(self, self.SYSFS_GPIO_PIN_VALUE.format(self.pin_number), recursive=False)
            observer.start()
            self.running = True
            try:
                while reactor.running:
                    time.sleep(1)
            finally:
                observer.stop()
            observer.join()        
        else:
            raise TypeError("Callback is not defined.")

    @staticmethod
    def exists_already(pin_number):
        gpio_path = Pin.SYSFS_GPIO_PIN_PATH.format(pin_number)
        return os.path.isdir(gpio_path)


class GPIO(object):

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, '_instance'):
            instance = super(GPIO, cls).__new__(cls)
            instance.__pins = {}
            instance._running = True
            reactor.addSystemEventTrigger('before', 'shutdown', instance.stop)
            cls._instance = instance
        return cls._instance

    def stop(cls):
        cls._running = False
        cls.cleanup()

    def cleanup(cls):
        cls.__pins.clear()

    def setmode(cls, mode):
        cls.__mode = mode

    def getmode(cls):
        return cls.__mode

    def setup(cls, pin_number, direction=Direction.OUT, callback=None, emulator=False):
        if not emulator:
            new_pin = Pin(pin_number, direction=direction, callback=callback)
        else:
            new_pin = Pin(pin_number, callback=callback, emulator=True)
            
        cls.__pins.update({pin_number: new_pin})
        reactor.callInThread(new_pin.start_watchdog)
        return new_pin
