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
from picarx.i2c import I2C
from enum import Enum

class SunFounderPWMValues(Enum):
    DEFAULT_CLOCK = 72000000
    DEFAULT_PERIOD = 4095
    DEFAULT_PRESCALER = 8
    DEFAULT_I2C_ADRESS = 0x14 # int int: 20
    REGISTER_CHANNEL = 0x20 # in int: 32
    REGISTER_FREQUENCY = 0x30
    REGISTER_PRESCALER = 0x40
    REGISTER_ARR = 0x44
    DEFAULT_FREQUENCY = 50


class PWM(I2C):

    def __init__(self, channel, address=0x14, i2c_port=1, period=SunFounderPWMValues.DEFAULT_PERIOD.value, prescaler=SunFounderPWMValues.DEFAULT_PRESCALER.value, reg_chn=SunFounderPWMValues.REGISTER_CHANNEL.value, reg_fre=SunFounderPWMValues.REGISTER_FREQUENCY.value, reg_psc=SunFounderPWMValues.REGISTER_PRESCALER.value, reg_arr=SunFounderPWMValues.REGISTER_ARR.value, debug="critical"):
        super(PWM, self).__init__(i2c_port=i2c_port, address=address)
        self.debug = debug
        self.channel = channel
        self.register_channel = reg_chn  # Channel
        self.register_frequency = reg_fre  # Frequenz
        self.register_prescaler = reg_psc  # Prescalar
        self.register_arr = reg_arr  # I don't know what this is, but SunFounder used it

    @property
    def channel(self):
        return self.__channel

    @channel.setter
    def channel(self, channel: Union[str, int]):
        if isinstance(channel, str):
            if channel.startswith("P"):
                channel = int(channel[1:])
            else:
                raise ValueError(
                    "PWM channel should be between [P1, P14], not {0}".format(channel))
        self.__channel = channel

        try:
            self.write_byte(0x2C)
            self.write_byte(0)
            self.write_byte(0)
        except IOError:
            raise IOError("Can't send stuff.")

    @property
    def period(self):
        return self.__period

    @period.setter
    def period(self, period):
        period = int(period) - 1
        if period > 0:
            self.__period = period
            #self.register_arr.write(self.__period)

    @property
    def prescaler(self):
        return self.__prescaler

    @prescaler.setter
    def prescaler(self, prescaler):
        prescaler = int(prescaler) - 1
        if prescaler > 0:
            self.__prescaler = prescaler
            #self.register_prescaler.write(prescaler)

    @property
    def register_channel(self):
        return self.__register_channel

    @register_channel.setter
    def register_channel(self, register_address):
        self.__register_channel = self.create_register(register_address + self.channel, 2)

    @property
    def register_frequency(self):
        return self.__register_frequency

    @register_frequency.setter
    def register_frequency(self, register_address):
        self.__register_frequency = register_address
        #self.__register_frequency = self.create_register(register_address + int(self.channel / 4), 2)

    @property
    def register_prescaler(self):
        return self.__register_prescaler

    @register_prescaler.setter
    def register_prescaler(self, register_address):
        self.__register_prescaler = register_address
        #self.__register_prescaler = self.create_register(register_address  + int(self.channel / 4), 2)

    @property
    def register_arr(self):
        return self.__register_arr

    @register_arr.setter
    def register_arr(self, register_address):
        self.__register_arr = register_address
        #self.__register_arr = self.create_register(register_address + int(self.channel / 4), 2)

    def pulse_width(self, pulse_width: int):
        ''' I2C expects int in big endian. RapsberryPI works with little endian. Thus, we transform all ints from little to big endian.'''
        if pulse_width >= 0:
            #pulse_width = pulse_width.to_bytes(2, 'little')
            self.register_channel.write(pulse_width)

    @property
    def duty_cycle(self):
        return self.__duty_cycle * 100

    @duty_cycle.setter
    def duty_cycle(self, percent):
        if 0 <= percent <= 100:
            percent = percent / 100.0
            self.__duty_cycle = percent
            self.pulse_width(int(percent * self.period))
