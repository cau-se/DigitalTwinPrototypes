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

from typing import Union
from picarx.i2c import I2C
import time

class ADC(I2C):

    def __init__(self, channel, address=0x14, i2c_port=1):
        super(ADC, self).__init__(i2c_port=i2c_port, address=address)
        self.channel = channel
        self.register_channel = 0x40 + self.channel

    @property
    def channel(self):
        return self.__channel

    @channel.setter
    def channel(self, channel: Union[str, int]):
        if isinstance(channel, str):
            if channel.startswith("A"): 
                channel = int(channel[1:])
            else:
                raise ValueError("ADC channel should be between [A0, A7], not {0}".format(channel))
        channel = 7 - channel
        self.__channel = channel | 0x10

    @property
    def register_channel(self):
        return self.__register_channel

    @register_channel.setter
    def register_channel(self, register_address):
        self.__register_channel = self.create_register(register_address + self.channel, 2)

    def read(self):
        self.register_channel.write(0)
        time.sleep(1000/50)
        value = self.register_channel.read()

        return value

    @property
    def voltage(self):
        return self.read() * 3.3 / 4095