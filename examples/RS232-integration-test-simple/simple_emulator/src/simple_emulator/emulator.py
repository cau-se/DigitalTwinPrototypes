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
import csv
import sys
from threading import Timer, Thread
import serial
import time

class Options(object):

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "serial_port", help="The RS232 port the simulator should send data to, e.g., /dev/com0")
        parser.add_argument(
            "filename", help="The path to a file, e.g., /path/to/file.txt")
        parser.add_argument(
            "interval", default=None, type=float, help="The interval in which a line in the file should be read.")

        self.args = parser.parse_args(argv)

    def get_args(self):
        return vars(self.args)


class RepeatingTimer(object):
    """SOURCE: https://stackoverflow.com/questions/24072765/timer-cannot-restart-after-it-is-being-stopped-in-python"""

    def __init__(self, interval, f, *args, **kwargs):
        self.interval = interval
        self.f = f
        self.args = args
        self.kwargs = kwargs

        self.timer = None

    def callback(self):
        self.f(*self.args, **self.kwargs)
        self.start()

    def is_alive(self):
        return self.timer.is_alive()

    def cancel(self):
        self.timer.cancel()

    def start(self):
        self.timer = Timer(self.interval, self.callback)
        self.timer.start()


class SimpleEmulator(object):
    def __init__(self):
        self.__serial_port = None
        self.__timer = None
        self.__file = None
        self.driver_thread = None
        self.__current_line = 1

    @property
    def file(self) -> list:
        return self.__file

    @file.setter
    def file(self, filename: str) -> None:
        if filename is not None:
            with open(filename, 'r') as datafile:
                reader = csv.reader(datafile)
                headers = next(reader)
                self.__file = [{h: x for (h, x) in zip(headers, row)} for row in reader]

    def get_sample(self) -> None:
        new_value = self.read_next()
        try:
            self.serial_port.write('#{}: {}\r\n'.format(list(new_value.keys())[0], list(new_value.values())[0]).encode())
        except serial.SerialException:
            self.timer = 0
            print("Port is closed can't write.")

    def read_next(self) -> dict:
        if self.__current_line + 1 >= len(self.file):
            self.__current_line = -1
        self.__current_line = self.__current_line + 1
        return self.file[self.__current_line]

    @property
    def serial_port(self) -> serial.Serial:
        return self.__serial_port

    @serial_port.setter
    def serial_port(self, serial_port: str) -> None:
        try:
            self.__serial_port = serial.Serial(serial_port, baudrate=9600, timeout=0)  # open serial port
        except serial.SerialException:
            raise IOError("Problem connecting to serial device.")

    def server(self) -> None:
        try:
            while True:
                received_command = self.serial_port.readline().decode().rstrip()
                if received_command is None or len(received_command) == 0:
                    continue
                elif received_command == 'GET_SAMPLE':
                    self.get_sample()
                elif received_command[:6] == 'PERIOD':
                    interval_command, value = received_command.split(' ')
                    value = float(value)
                    if self.timer is not None:
                        if self.timer.is_alive():
                            self.timer.cancel()
                    self.timer = value
                    self.serial_port.write('SET NEW PERIOD: {} (seconds)\r\n'.format(value).encode())
                else:
                    self.serial_port.write('Invalid Command\r\n'.encode())
        except serial.SerialException:
            #print('Serial port is already opened or does not exist.')
            pass

    def start(self, serial_port, filename, interval):
        self.driver_thread = Thread(target=self.start_emulator, args=(serial_port, filename, interval,))
        self.driver_thread.start()

    def start_emulator(self, serial_port: str, filename: str, interval: float = 0) -> None:
        self.serial_port = serial_port
        time.sleep(0.5)
        self.file = filename
        self.timer = interval

        self.server()

    def stop_emulator(self) -> None:
        self.timer = 0
        self.serial_port.cancel_read()
        self.serial_port.cancel_write()
        self.serial_port.close()

    @property
    def timer(self) -> RepeatingTimer:
        return self.__timer

    @timer.setter
    def timer(self, interval: float) -> None:
        if self.__timer is not None:
            if self.__timer.is_alive():
                self.__timer.cancel()
        if interval is None:
            self.__timer = None
        elif interval > 0:
            self.__timer = RepeatingTimer(interval, self.get_sample)
            self.__timer.start()
        else:
            self.__timer = None


if __name__ == "__main__":
    emulator = None
    try:
        options = Options(sys.argv[1:])
        emulator = SimpleEmulator()
        emulator.start(options.args.serial_port, options.args.filename, options.args.interval)
        print("Emulator was started")
    except KeyboardInterrupt:
        emulator.stop_emulator()
        print("Emulator was stopped")
        pass
