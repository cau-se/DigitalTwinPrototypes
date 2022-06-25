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

# from .basic import _Basic_class
from abc import ABCMeta, abstractmethod
from smbus2 import SMBus

class I2C(object):

    def __init__(self, i2c_port=1, address=0x14):    
        self.i2c_port = i2c_port # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        self.smbus = SMBus(self.i2c_port)
        self.address = address
        self.known_registers = {}

    @property
    def i2c_port(self):
        return self.__i2c_port

    @i2c_port.setter
    def i2c_port(self, port: int):
        self.__i2c_port = port

    @property
    def address(self):
        return self.__address
    
    @address.setter
    def address(self, address: bytes):
        self.__address = address

    def write_byte(self, data):
        return self.smbus.write_byte(self.address, data)
    
    def read_byte(self):
        return self.smbus.read_byte(self.address)

    def create_register(self, register_address, size):
        if size == 1:
            return ByteRegister(self.smbus, self.address, register_address)
        elif size == 2:
            return WordRegister(self.smbus, self.address, register_address)
        elif size == 4:
            return I2CBlockRegister(self.smbus, self.address, register_address)
        else:
            raise ValueError("The size of the register must be 1,2, or 4. You entered {}".format(size))

class Register(metaclass=ABCMeta):
    
    @abstractmethod
    def __init__(self, smbus, address, register_address):
        self.smbus = smbus
        self.address = address    
        self.register_address = register_address

    @abstractmethod
    def write(self, value):
        pass

    @abstractmethod
    def read(self):
        pass

    def __del__(self):
        self.clean_up()

    def clean_up(self):
        print("Cleaning register: {}".format(self.register_address))
        self.write(0)

class ByteRegister(Register):

    def __init__(self, smbus, address, register_address):
        super(ByteRegister, self).__init__(smbus, address, register_address)

    def write(self, value):
        return self.smbus.write_byte_data(self.address, self.register_address, value)

    def read(self):
        return self.smbus.read_byte_data(self.address, self.register_address)

class WordRegister(Register):
    def __init__(self, smbus, address, register_address):
        super(WordRegister, self).__init__(smbus, address, register_address)

    def write(self, value):
        value = value.to_bytes(2, 'little')
        value = int.from_bytes(value, 'big')
        return self.smbus.write_word_data(self.address, self.register_address, value)

    def read(self):
        read_value = self.smbus.read_word_data(self.address, self.register_address)
        value = read_value.to_bytes(2, 'big')
        return int.from_bytes(value, 'little')

class I2CBlockRegister(Register):
    def __init__(self, smbus, address, register_address):
        super(I2CBlockRegister, self).__init__(smbus, address, register_address)

    def write(self, value):
        return self.smbus.write_i2c_block_data(self.address, self.register_address, value)

    def read(self):
        return self.smbus.write_i2c_block_data(self.address, self.register_address)