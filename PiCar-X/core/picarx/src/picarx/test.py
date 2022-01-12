#!/usr/bin/env python3
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

from picarx.gpio import GPIO, Direction
import time
import logging
from twisted.internet import reactor

def print_func(value):
    print("Pin Value changed to: {}".format(value))
    logging.info("LOG: Pin Value changed to: {}".format(value))


def callback(event):
    print(f'Event type: {event.event_type}  path : {event.src_path}')
    print(event.is_directory) # This attribute is also available

def emulator_callback(event):
    print(f'EMULATOR SEES: Event type: {event.event_type}  path : {event.src_path}')

def somestuff():
    try:
        while reactor.running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("WTF")
        reactor.stop()

if __name__ == '__main__':
    gpio = GPIO()
    pin24 = gpio.setup(24, Direction.OUT, callback=callback)
    pin24_emualtor = gpio.setup(24, callback=emulator_callback, emulator=True)
    reactor.callInThread(somestuff)
    reactor.run()
    
    