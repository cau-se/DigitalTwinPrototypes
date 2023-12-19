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

from picarx.adc import ADC

class Grayscale_Module(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    @property
    def channel_0(self):
        return self.__channel_0

    @channel_0.setter
    def channel_0(self, channel: str):
        self.__channel_0 = ADC(channel)

    @property
    def channel_1(self):
        return self.__channel_1

    @channel_1.setter
    def channel_0(self, channel: str):
        self.__channel_1 = ADC(channel)

    @property
    def channel_2(self):
        return self.__channel_2

    @channel_0.setter
    def channel_2(self, channel: str):
        self.__channel_2 = ADC(channel)

    @property
    def ref(self):
        return self.__ref

    @ref.setter
    def ref(self, ref: int):
        self.__ref = ref

    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.channel_0.read())
        adc_value_list.append(self.channel_1.read())
        adc_value_list.append(self.channel_2.read())
        return adc_value_list

    def get_line_status(self, greyscale_data: list):

        if greyscale_data[0] > self.ref and greyscale_data[1] > self.ref and greyscale_data[2] > self.ref:
            return 'stop'
            
        elif greyscale_data[1] <= self.ref:
            return 'forward'
        
        elif greyscale_data[0] <= self.ref:
            return 'right'

        elif greyscale_data[2] <= self.ref:
            return 'left'