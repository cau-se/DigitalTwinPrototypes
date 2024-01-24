#!/usr/bin/env python3

"""
MIT License

Copyright (c) 2021 Alexander Barbie

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import argparse
import signal
import sys
import threading
import time

import serial


class Options(object):

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "serial_port", help="The RS232 port the simulator should send data to, e.g., /dev/com0")
        parser.add_argument(
            "interval", default=None, type=float, help="The interval in which a line in the file should be read.")

        self.args = parser.parse_args(argv)

    def get_args(self):
        return vars(self.args)


class Observer(object):
    _observers = []

    def __init__(self):
        self._observers.append(self)
        self._observables = dict()

    def observe(self, event_name: str, callback) -> None:
        self._observables[event_name] = callback

    def get_observables(self):
        return self._observables


class Event(object):
    def __init__(self, name: str, data: dict, autofire: bool = True):
        self.name = name
        self.data = data
        if autofire:
            self.fire()

    def fire(self) -> None:
        for observer in Observer._observers:
            observables = observer.get_observables()
            if self.name in observables:
                observables[self.name](str(self.data))


class SimpleDriver(object):
    def __init__(self):
        self.__serial_port = None
        self.driver_thread = None

    def get_sample(self) -> None:
        if self.serial_port is not None:
            if self.serial_port.is_open:
                self.serial_port.write('GET_SAMPLE\r\n'.encode())

    @property
    def serial_port(self) -> serial.Serial:
        return self.__serial_port

    @serial_port.setter
    def serial_port(self, serial_port: str) -> None:
        if serial_port is None:
            self.__serial_port = None
        else:
            try:
                self.__serial_port = serial.Serial(serial_port, baudrate=9600, timeout=0)  # open serial port
            except serial.SerialException:
                raise IOError("Problem connecting to serial device.")

    def server(self) -> None:
        try:
            while True:
                received_message = self.serial_port.readline().decode().rstrip()
                if received_message is None or len(received_message) == 0:
                    continue
                elif received_message[0] == '#':
                    field, value = received_message[1:].split(':')
                    sample = {field: value}
                    Event('get_sample', sample)
                else:
                    print(received_message)
        except serial.SerialException:
            print('Thread was terminated.')

    def set_interval(self, interval):
        self.serial_port.write('PERIOD {}\r\n'.format(interval).encode())

    def start(self, serial_port, interval):
        self.driver_thread = threading.Thread(target=self.start_driver, args=(serial_port, interval,))
        self.driver_thread.start()

    def start_driver(self, serial_port, interval: float = 0) -> None:
        self.serial_port = serial_port
        time.sleep(0.5)
        if interval > 0:
            self.set_interval(interval)
        self.server()

    def stop_driver(self) -> None:
        #self.serial_port.cancel_read()
        #self.serial_port.cancel_write()
        self.serial_port.close()


class SomeSampleHandler(Observer):
    def __init__(self):
        super(Observer, self).__init__()

    def new_sample(self, sample: dict) -> None:
        print(sample)


def shutdown_handler():
    driver.stop_driver()
    print("Emulator was stopped")


if __name__ == "__main__":
    driver = None
    try:
        signal.signal(signal.SIGINT, shutdown_handler)
        options = Options(sys.argv[1:])
        driver = SimpleDriver()
        driver.start(options.args.serial_port, options.args.interval)
        sample_observer = SomeSampleHandler()
        sample_observer.observe('get_sample', sample_observer.new_sample)
        time.sleep(1)
        driver.get_sample()
    except (KeyboardInterrupt, SystemExit):
        pass
