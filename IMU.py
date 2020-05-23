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

MPU6050_ADDRESS_AD0_LOW  = 0x68  # address pin low (GND)
MPU6050_ADDRESS_AD0_HIGH = 0x69  # address pin high (VCC)
MPU6050_DEFAULT_ADDRESS  = MPU6050_ADDRESS_AD0_LOW

MPU6050_RA_GYRO_CONFIG      = 0x1B
MPU6050_RA_ACCEL_CONFIG     = 0x1C
MPU6050_RA_PWR_MGMT_1       = 0x6B

# MPU6050_RA_ACCEL_CONFIG
MPU6050_ACONFIG_XA_ST_BIT      = 7
MPU6050_ACONFIG_YA_ST_BIT      = 6
MPU6050_ACONFIG_ZA_ST_BIT      = 5
MPU6050_ACONFIG_AFS_SEL_BIT    = 4
MPU6050_ACONFIG_AFS_SEL_LENGTH = 2

# MPU6050_RA_GYRO_CONFIG
MPU6050_GCONFIG_XG_ST_BIT     = 7
MPU6050_GCONFIG_YG_ST_BIT     = 6
MPU6050_GCONFIG_ZG_ST_BIT     = 5
MPU6050_GCONFIG_FS_SEL_BIT    = 4
MPU6050_GCONFIG_FS_SEL_LENGTH = 2

# MPU6050_RA_PWR_MGMT_1
MPU6050_PWR1_DEVICE_RESET_BIT = 7
MPU6050_PWR1_SLEEP_BIT        = 6
MPU6050_PWR1_CYCLE_BIT        = 5
MPU6050_PWR1_TEMP_DIS_BIT     = 3
MPU6050_PWR1_CLKSEL_BIT       = 2
MPU6050_PWR1_CLKSEL_LENGTH    = 3

# CLOCK SOURCE
MPU6050_CLOCK_INTERNAL   = 0x00
MPU6050_CLOCK_PLL_XGYRO  = 0x01
MPU6050_CLOCK_PLL_YGYRO  = 0x02
MPU6050_CLOCK_PLL_ZGYRO  = 0x03
MPU6050_CLOCK_PLL_EXT32K = 0x04
MPU6050_CLOCK_PLL_EXT19M = 0x05
MPU6050_CLOCK_KEEP_RESET = 0x07

# ACCEL FULL SCALE
MPU6050_ACCEL_FS_2       = 0x00
MPU6050_ACCEL_FS_4       = 0x01
MPU6050_ACCEL_FS_8       = 0x02
MPU6050_ACCEL_FS_16      = 0x03

# GYRO FULL SCALE
MPU6050_GYRO_FS_250      = 0x00
MPU6050_GYRO_FS_500      = 0x01
MPU6050_GYRO_FS_1000     = 0x02
MPU6050_GYRO_FS_2000     = 0x03


class MPUException(OSError):
    """Exception for MPU devices."""

    pass


class MPU6050():
    """A micropython module for the InvenSense MPU6050 sensor."""

    def __init__(self, i2c=None, address=MPU6050_DEFAULT_ADDRESS):
        """Init MPU6050 instance."""
        if isinstance(i2c, I2C):
            self.i2c = i2c
        else:
            self.i2c = I2C(sda=Pin(21), scl=Pin(22))  # ESP32 DEVKIT V1

        self.address = address
        self.buf = bytearray(1)

    def initialize(self,
                   clksel=MPU6050_CLOCK_PLL_XGYRO,
                   fs_sel=MPU6050_GYRO_FS_250,
                   afs_sel=MPU6050_ACCEL_FS_2):
        """Prepare for general usage."""
        self.set_clock_source(clksel)
        self.set_full_scale_gyro_range(fs_sel)
        self.set_full_scale_accel_range(afs_sel)
        self.set_sleep_enable(False)

    def set_clock_source(self, clksel):
        """
        Set clock source setting.

        -----------------------------------------------
        CLK_SEL | Clock Source
        --------+--------------------------------------
        0       | Internal oscillator
        1       | PLL with X Gyro reference
        2       | PLL with Y Gyro reference
        3       | PLL with Z Gyro reference
        4       | PLL with external 32.768kHz reference
        5       | PLL with external 19.2MHz reference
        6       | Reserved
        7       | Stops the clock and keeps the timing generator in reset
        """
        if self.write_bits(MPU6050_RA_PWR_MGMT_1,
                           MPU6050_PWR1_CLKSEL_BIT,
                           MPU6050_PWR1_CLKSEL_LENGTH,
                           clksel):
            return True
        raise MPUException("Failed to set clock source")

    def get_clock_source(self):
        """Clock source setting."""
        return self.read_bits(self.address,
                              MPU6050_RA_PWR_MGMT_1,
                              MPU6050_PWR1_CLKSEL_BIT,
                              MPU6050_PWR1_CLKSEL_LENGTH)

    def set_full_scale_gyro_range(self, fs_sel):
        """
        Set full-scale gyroscope range.

        -------------------------
        FS_SEL | Full scale range
        -------------------------
        0      | +/- 250 deg/sec
        1      | +/- 500 deg/sec
        2      | +/- 1000 deg/sec
        3      | +/- 2000 deg/sec
        """
        if self.write_bits(self.address,
                           MPU6050_RA_GYRO_CONFIG,
                           MPU6050_GCONFIG_FS_SEL_BIT,
                           MPU6050_GCONFIG_FS_SEL_LENGTH,
                           fs_sel):
            return True
        raise MPUException("Failed to set full-scale Gyro range")

    def get_full_scale_gyro_range(self):
        """Get current full-scale gyroscope range setting."""
        return self.read_bits(self.address,
                              MPU6050_RA_GYRO_CONFIG,
                              MPU6050_GCONFIG_FS_SEL_BIT,
                              MPU6050_GCONFIG_FS_SEL_LENGTH)

    def set_full_scale_accel_range(self, afs_sel):
        """
        Set full-scale accelerometer range.

        --------------------------------
        SFS_SEL | Full scale accel range
        --------------------------------
        0       | +/- 2g
        1       | +/- 4g
        2       | +/- 8g
        3       | +/- 16g
        """
        if self.write_bits(self.address,
                           MPU6050_RA_ACCEL_CONFIG,
                           MPU6050_ACONFIG_AFS_SEL_BIT,
                           MPU6050_ACONFIG_AFS_SEL_LENGTH,
                           afs_sel):
            return True
        raise MPUException("Failed to set full-scale Accel range")

    def get_full_scale_accel_range(self):
        """Get current full-scale accelerometer range setting."""
        return self.read_bits(self.address,
                              MPU6050_RA_ACCEL_CONFIG,
                              MPU6050_ACONFIG_AFS_SEL_BIT,
                              MPU6050_ACONFIG_AFS_SEL_LENGTH)

    def set_sleep_enable(self, enabled):
        """
        Set sleep mode status.

        False | Disabled
        True  | Enabled
        """
        if self.write_bit(self.address,
                          MPU6050_RA_PWR_MGMT_1,
                          MPU6050_PWR1_SLEEP_BIT,
                          enabled):
            return True
        raise MPUException("Failed to set Sleep mode")

    def get_sleep_enable(self):
        """Sleep mode status."""
        return self.read_bit(self.address,
                             MPU6050_RA_PWR_MGMT_1,
                             MPU6050_PWR1_SLEEP_BIT)

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
