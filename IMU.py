"""
A micropython module for the InvenSense MPU6050 sensor.

Adapted from Jeff Rowberg driver.
https://github.com/jrowberg/i2cdevlib

=============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
=============================================
"""
from machine import I2C, Pin

MPU6050_ADDRESS_AD0_LOW     = 0x68  # address pin low (GND)
MPU6050_ADDRESS_AD0_HIGH    = 0x69  # address pin high (VCC)
MPU6050_DEFAULT_ADDRESS     = MPU6050_ADDRESS_AD0_LOW


class MPU6050():
    """A micropython module for the InvenSense MPU6050 sensor."""

    def __init__(self, i2c=None, address=MPU6050_ADDRESS_AD0_LOW):
        """Init MPU6050 instance."""
        if isinstance(i2c, I2C):
            self.i2c = i2c
        else:
            self.i2c = I2C(sda=Pin(21), scl=Pin(22))  # ESP32 DEVKIT V1

        self.address = address
        self.buf = bytearray(1)

    # I2C
    def scan(self):
        """Scan all the I2C adresses."""
        adresses = self.i2c.scan()
        if adresses:
            for adress in adresses:
                print(adress)
        else:
            "No device found."

    def read_bit(self, r_address, bit_num):
        """Read a single bit from an 8-bit device register."""
        self.i2c.readfrom_mem_into(self.address, r_address, self.buf)
        b = self.buf[0]
        b & (1 << bit_num)
        b >>= bit_num
        return b

    def write_bit(self, r_address, bit_num, data):
        """Write a single bit in an 8-bit device register."""
        self.buf = self.read_byte(self.address, r_address)
        b = self.buf[0]
        b = (b | (1 << bit_num)) if (data != 0) else (b & ~(1 << bit_num))
        return self.write_byte(self.address, r_address, bytearray([b]))

    def read_bits(self, r_address, bit_start, length):
        """Read multiple bits from an 8-bit device register."""
        self.i2c.readfrom_mem_into(self.address, r_address, self.buf)
        mask = ((1 << length) - 1) << (bit_start - length + 1)
        b = self.buf[0]
        b &= mask
        b >>= (bit_start - length + 1)
        return b

    def write_bits(self, r_address, bit_start, length, data):
        """Write multiples bits in an 8-bit device register."""
        self.buf = self.read_byte(self.address, r_address)
        b = self.buf[0]
        mask = ((1 << length) - 1) << (bit_start - length + 1)
        data <<= (bit_start - length + 1)
        data &= mask
        b &= ~(mask)
        b |= data
        return self.write_byte(self.address, r_address, bytearray([b]))

    def read_byte(self, r_address):
        """Read single byte from an 8-bit device register."""
        self.i2c.readfrom_mem_into(self.address, r_address, self.buf)
        return self.buf

    def write_byte(self, r_address, data):
        """Write a single byte in an 8-bit device register."""
        self.i2c.writeto_mem(self.address, r_address, data)
        if (data == self.read_byte(self.address, r_address)):
            return True
        return False

    def read_bytes(self, r_address, length):
        """Read single byte from an 8-bit device register."""
        return self.i2c.readfrom_mem(self.address, r_address, length)
