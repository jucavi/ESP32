"""
A micropython module for the InvenSense MPU6050 sensor.

Revision: 4.2
Release Date: 08/19/2013

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

MPU6050_RA_SMPLRT_DIV        = 0x19
MPU6050_RA_CONFIG            = 0x1A  # [5:3] EXT_SYNC_SET[2:0], [2:0] DLPF_CFG[2:0]
MPU6050_RA_GYRO_CONFIG       = 0x1B
MPU6050_RA_ACCEL_CONFIG      = 0x1C
MPU6050_RA_FIFO_EN           = 0x23
MPU6050_RA_I2C_MST_CTRL      = 0x24
MPU6050_RA_I2C_MST_STATUS    = 0x36
MPU6050_RA_INT_PIN_CFG       = 0x37
MPU6050_RA_INT_ENABLE        = 0x38
MPU6050_RA_INT_STATUS        = 0x3A
MPU6050_RA_ACCEL_XOUT_H      = 0x3B
MPU6050_RA_ACCEL_XOUT_L      = 0x3C
MPU6050_RA_ACCEL_YOUT_H      = 0x3D
MPU6050_RA_ACCEL_YOUT_L      = 0x3E
MPU6050_RA_ACCEL_ZOUT_H      = 0x3F
MPU6050_RA_ACCEL_ZOUT_L      = 0x40
MPU6050_RA_TEMP_OUT_H        = 0x41
MPU6050_RA_TEMP_OUT_L        = 0x42
MPU6050_RA_GYRO_XOUT_H       = 0x43
MPU6050_RA_GYRO_XOUT_L       = 0x44
MPU6050_RA_GYRO_YOUT_H       = 0x45
MPU6050_RA_GYRO_YOUT_L       = 0x46
MPU6050_RA_GYRO_ZOUT_H       = 0x47
MPU6050_RA_GYRO_ZOUT_L       = 0x48
MPU6050_RA_SIGNAL_PATH_RESET = 0x68
MPU6050_RA_USER_CTRL         = 0x6A
MPU6050_RA_PWR_MGMT_1        = 0x6B
MPU6050_RA_PWR_MGMT_2        = 0x6C
MPU6050_RA_FIFO_COUNTH       = 0x72
MPU6050_RA_FIFO_COUNTL       = 0x73
MPU6050_RA_FIFO_R_W          = 0x74
MPU6050_RA_WHO_AM_I          = 0x75

# CONFIG
MPU6050_CFG_EXT_SYNC_SET_BIT    = 5
MPU6050_CFG_EXT_SYNC_SET_LENGTH = 3
MPU6050_CFG_DLPF_CFG_BIT    = 2
MPU6050_CFG_DLPF_CFG_LENGTH = 3

# EXT_SYNC
MPU6050_EXT_SYNC_DISABLED     = 0x0
MPU6050_EXT_SYNC_TEMP_OUT_L   = 0x1
MPU6050_EXT_SYNC_GYRO_XOUT_L  = 0x2
MPU6050_EXT_SYNC_GYRO_YOUT_L  = 0x3
MPU6050_EXT_SYNC_GYRO_ZOUT_L  = 0x4
MPU6050_EXT_SYNC_ACCEL_XOUT_L = 0x5
MPU6050_EXT_SYNC_ACCEL_YOUT_L = 0x6
MPU6050_EXT_SYNC_ACCEL_ZOUT_L = 0x7

# DLPF_CFG
MPU6050_DLPF_BW_256 = 0x00
MPU6050_DLPF_BW_188 = 0x01
MPU6050_DLPF_BW_98  = 0x02
MPU6050_DLPF_BW_42  = 0x03
MPU6050_DLPF_BW_20  = 0x04
MPU6050_DLPF_BW_10  = 0x05
MPU6050_DLPF_BW_5   = 0x06

# GYRO_CONFIG
MPU6050_GCONFIG_XG_ST_BIT     = 7
MPU6050_GCONFIG_YG_ST_BIT     = 6
MPU6050_GCONFIG_ZG_ST_BIT     = 5
MPU6050_GCONFIG_FS_SEL_BIT    = 4
MPU6050_GCONFIG_FS_SEL_LENGTH = 2

# GYRO FULL SCALE
MPU6050_GYRO_FS_250  = 0x00
MPU6050_GYRO_FS_500  = 0x01
MPU6050_GYRO_FS_1000 = 0x02
MPU6050_GYRO_FS_2000 = 0x03

# ACCEL_CONFIG
MPU6050_ACONFIG_XA_ST_BIT      = 7
MPU6050_ACONFIG_YA_ST_BIT      = 6
MPU6050_ACONFIG_ZA_ST_BIT      = 5
MPU6050_ACONFIG_AFS_SEL_BIT    = 4
MPU6050_ACONFIG_AFS_SEL_LENGTH = 2

# ACCEL FULL SCALE
MPU6050_ACCEL_FS_2  = 0x00
MPU6050_ACCEL_FS_4  = 0x01
MPU6050_ACCEL_FS_8  = 0x02
MPU6050_ACCEL_FS_16 = 0x03

# FIFO_EN
MPU6050_TEMP_FIFO_EN_BIT  = 7
MPU6050_XG_FIFO_EN_BIT    = 6
MPU6050_YG_FIFO_EN_BIT    = 5
MPU6050_ZG_FIFO_EN_BIT    = 4
MPU6050_ACCEL_FIFO_EN_BIT = 3
MPU6050_SLV2_FIFO_EN_BIT  = 2
MPU6050_SLV1_FIFO_EN_BIT  = 1
MPU6050_SLV0_FIFO_EN_BIT  = 0

# I2C_MST_CTRL
MPU6050_MULT_MST_EN_BIT    = 7
MPU6050_WAIT_FOR_ES_BIT    = 6
MPU6050_SLV3_FIFO_EN_BIT   = 5
MPU6050_I2C_MST_P_NSR_BIT  = 4
MPU6050_I2C_MST_CLK_BIT    = 3
MPU6050_I2C_MST_CLK_LENGTH = 4

# I2C_MST_STATUS
MPU6050_MST_PASS_THROUGH_BIT  = 7
MPU6050_MST_I2C_SLV4_DONE_BIT = 6
MPU6050_MST_I2C_LOST_ARB_BIT  = 5
MPU6050_MST_I2C_SLV4_NACK_BIT = 4
MPU6050_MST_I2C_SLV3_NACK_BIT = 3
MPU6050_MST_I2C_SLV2_NACK_BIT = 2
MPU6050_MST_I2C_SLV1_NACK_BIT = 1
MPU6050_MST_I2C_SLV0_NACK_BIT = 0

# MST_CLK
MPU6050_CLOCK_DIV_348 = 0x0
MPU6050_CLOCK_DIV_333 = 0x1
MPU6050_CLOCK_DIV_320 = 0x2
MPU6050_CLOCK_DIV_308 = 0x3
MPU6050_CLOCK_DIV_296 = 0x4
MPU6050_CLOCK_DIV_286 = 0x5
MPU6050_CLOCK_DIV_276 = 0x6
MPU6050_CLOCK_DIV_267 = 0x7
MPU6050_CLOCK_DIV_258 = 0x8
MPU6050_CLOCK_DIV_500 = 0x9
MPU6050_CLOCK_DIV_471 = 0xA
MPU6050_CLOCK_DIV_444 = 0xB
MPU6050_CLOCK_DIV_421 = 0xC
MPU6050_CLOCK_DIV_400 = 0xD
MPU6050_CLOCK_DIV_381 = 0xE
MPU6050_CLOCK_DIV_364 = 0xF

# INT_PIN_CFG
MPU6050_INTCFG_INT_LEVEL_BIT       = 7
MPU6050_INTCFG_INT_OPEN_BIT        = 6
MPU6050_INTCFG_LATCH_INT_EN_BIT    = 5
MPU6050_INTCFG_INT_RD_CLEAR_BIT    = 4
MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT = 3
MPU6050_INTCFG_FSYNC_INT_EN_BIT    = 2
MPU6050_INTCFG_I2C_BYPASS_EN_BIT   = 1

# INT_ENABLE, INT_STATUS
MPU6050_INTERRUPT_FIFO_OFLOW_BIT    = 4
MPU6050_INTERRUPT_I2C_MST_INT_BIT   = 3
MPU6050_INTERRUPT_DATA_RDY_BIT      = 0

# USR_CTRL
MPU6050_USERCTRL_FIFO_EN_BIT        = 6
MPU6050_USERCTRL_I2C_MST_EN_BIT     = 5
MPU6050_USERCTRL_I2C_IF_DIS_BIT     = 4
MPU6050_USERCTRL_FIFO_RESET_BIT     = 2
MPU6050_USERCTRL_I2C_MST_RESET_BIT  = 1
MPU6050_USERCTRL_SIG_COND_RESET_BIT = 0

# SIGNAL_PATH_RESET
MPU6050_PATHRESET_GYRO_RESET_BIT  = 2
MPU6050_PATHRESET_ACCEL_RESET_BIT = 1
MPU6050_PATHRESET_TEMP_RESET_BIT  = 0

# USER_CTRL
MPU6050_USERCTRL_FIFO_EN_BIT        = 6
MPU6050_USERCTRL_I2C_MST_EN_BIT     = 5
MPU6050_USERCTRL_I2C_IF_DIS_BIT     = 4
MPU6050_USERCTRL_DMP_RESET_BIT      = 3
MPU6050_USERCTRL_FIFO_RESET_BIT     = 2
MPU6050_USERCTRL_I2C_MST_RESET_BIT  = 1
MPU6050_USERCTRL_SIG_COND_RESET_BIT = 0

# PWR_MGMT_1
MPU6050_PWR1_DEVICE_RESET_BIT = 7
MPU6050_PWR1_SLEEP_BIT        = 6
MPU6050_PWR1_CYCLE_BIT        = 5
MPU6050_PWR1_TEMP_DIS_BIT     = 3
MPU6050_PWR1_CLKSEL_BIT       = 2
MPU6050_PWR1_CLKSEL_LENGTH    = 3

# CLKSEL
MPU6050_CLOCK_INTERNAL   = 0x00
MPU6050_CLOCK_PLL_XGYRO  = 0x01
MPU6050_CLOCK_PLL_YGYRO  = 0x02
MPU6050_CLOCK_PLL_ZGYRO  = 0x03
MPU6050_CLOCK_PLL_EXT32K = 0x04
MPU6050_CLOCK_PLL_EXT19M = 0x05
MPU6050_CLOCK_KEEP_RESET = 0x07

# PWR_MGMT_2
MPU6050_PWR2_LP_WAKE_CTRL_BIT    = 7
MPU6050_PWR2_LP_WAKE_CTRL_LENGTH = 2
MPU6050_PWR2_STBY_XA_BIT         = 5
MPU6050_PWR2_STBY_YA_BIT         = 4
MPU6050_PWR2_STBY_ZA_BIT         = 3
MPU6050_PWR2_STBY_XG_BIT         = 2
MPU6050_PWR2_STBY_YG_BIT         = 1
MPU6050_PWR2_STBY_ZG_BIT         = 0

# WHO_AM_I
MPU6050_WHO_AM_I_BIT    = 6
MPU6050_WHO_AM_I_LENGTH = 6


class MPUException(OSError):
    """MPUExeption."""

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
        self.reset_flag = False

    # SMPLRT_DIV
    def set_sample_rate(self, rate):
        """Set gyroscope sample rate divider."""
        return self.write_byte(MPU6050_RA_SMPLRT_DIV,
                               rate)

    def get_sample_rate(self):
        """
        Get gyroscope output rate divider.

        8-bit  unsigned   value.The  Sample  Rate  is  determined  by  dividing
        the gyroscope output rate by this value.
        """
        return self.read_byte(MPU6050_RA_SMPLRT_DIV)[0]

    # CONFIG
    def set_external_frame_sync(self, sync):
        """
        Set external FSYNC configuration.

        MPU6050_EXT_SYNC_DISABLED       = 0x0
        MPU6050_EXT_SYNC_TEMP_OUT_L     = 0x1
        MPU6050_EXT_SYNC_GYRO_XOUT_L    = 0x2
        MPU6050_EXT_SYNC_GYRO_YOUT_L    = 0x3
        MPU6050_EXT_SYNC_GYRO_ZOUT_L    = 0x4
        MPU6050_EXT_SYNC_ACCEL_XOUT_L   = 0x5
        MPU6050_EXT_SYNC_ACCEL_YOUT_L   = 0x6
        MPU6050_EXT_SYNC_ACCEL_ZOUT_L   = 0x7
        """
        return self.read_bits(MPU6050_RA_CONFIG,
                              MPU6050_CFG_EXT_SYNC_SET_BIT,
                              MPU6050_CFG_EXT_SYNC_SET_LENGTH,
                              sync)

    def get_external_frame_sync(self):
        """
        Get FSYNC configuration value.

        ---------------------------------
        EXT_SYNC_SET | FSYNC Bit Location
        -------------+-------------------
        0            | Input disabled
        1            | TEMP_OUT_L[0]
        2            | GYRO_XOUT_L[0]
        3            | GYRO_YOUT_L[0]
        4            | GYRO_ZOUT_L[0]
        5            | ACCEL_XOUT_L[0]
        6            | ACCEL_YOUT_L[0]
        7            | ACCEL_ZOUT_L[0]
        """
        return self.write_bits(MPU6050_RA_CONFIG,
                               MPU6050_CFG_EXT_SYNC_SET_BIT,
                               MPU6050_CFG_EXT_SYNC_SET_LENGTH)

    def set_dlpf_mode(self, mode):
        """
        Set digital low-pass filter configuration.

        MPU6050_DLPF_BW_256         = 0x00
        MPU6050_DLPF_BW_188         = 0x01
        MPU6050_DLPF_BW_98          = 0x02
        MPU6050_DLPF_BW_42          = 0x03
        MPU6050_DLPF_BW_20          = 0x04
        MPU6050_DLPF_BW_10          = 0x05
        MPU6050_DLPF_BW_5           = 0x06
        """
        return self.write_bits(MPU6050_RA_CONFIG,
                               MPU6050_CFG_DLPF_CFG_BIT,
                               MPU6050_CFG_DLPF_CFG_LENGTH,
                               mode)

    def get_dlpf_mode(self):
        """
        Get Digital Low_pass filter configuration.

        -----------------------------------------------------------------
                 |   ACCELEROMETER    |           GYROSCOPE
        DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
        ---------+-----------+--------+-----------+--------+-------------
        0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
        1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
        2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
        3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
        4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
        5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
        6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
        7        |   -- Reserved --   |   -- Reserved --   | 8Khz
        """
        return self.read_bits(MPU6050_RA_CONFIG,
                              MPU6050_CFG_DLPF_CFG_BIT,
                              MPU6050_CFG_DLPF_CFG_LENGTH)

    # GYRO_CONFIG
    def set_full_scale_gyro_range(self, fscale):
        """
        Set full-scale gyroscope range.

        MPU6050_GYRO_FS_250  = 0x00
        MPU6050_GYRO_FS_500  = 0x01
        MPU6050_GYRO_FS_1000 = 0x02
        MPU6050_GYRO_FS_2000 = 0x03
        """
        return self.write_bits(MPU6050_RA_GYRO_CONFIG,
                               MPU6050_GCONFIG_FS_SEL_BIT,
                               MPU6050_GCONFIG_FS_SEL_LENGTH,
                               fscale)

    def get_full_scale_gyro_range(self):
        """
        Get current full-scale gyroscope range setting.

        -------------------------
        FS_SEL | Full scale range
        -------------------------
        0      | +/- 250 deg/sec
        1      | +/- 500 deg/sec
        2      | +/- 1000 deg/sec
        3      | +/- 2000 deg/sec
        """
        return self.read_bits(MPU6050_RA_GYRO_CONFIG,
                              MPU6050_GCONFIG_FS_SEL_BIT,
                              MPU6050_GCONFIG_FS_SEL_LENGTH)

    # ACCEL_CONFIG
    def set_accel_x_self_test(self, enabled):
        """Set self-test enabled setting for accelerometer X axis."""
        return self.write_bit(MPU6050_RA_ACCEL_CONFIG,
                              MPU6050_ACONFIG_XA_ST_BIT,
                              enabled)

    def get_accel_x_self_test(self):
        """When set to 1, the X-Axis accelerometer performs self test."""
        return self.read_bit(MPU6050_RA_ACCEL_CONFIG,
                             MPU6050_ACONFIG_XA_ST_BIT)

    def set_accel_y_self_test(self, enabled):
        """Set self-test enabled setting for accelerometer Y axis."""
        return self.write_bit(MPU6050_RA_ACCEL_CONFIG,
                              MPU6050_ACONFIG_YA_ST_BIT,
                              enabled)

    def get_accel_y_self_test(self):
        """When set to 1, the Y-Axis accelerometer performs self test."""
        return self.read_bit(MPU6050_RA_ACCEL_CONFIG,
                             MPU6050_ACONFIG_YA_ST_BIT)

    def set_accel_z_self_test(self, enabled):
        """Set self-test enabled setting for accelerometer Z axis."""
        return self.write_bit(MPU6050_RA_ACCEL_CONFIG,
                              MPU6050_ACONFIG_ZA_ST_BIT,
                              enabled)

    def get_accel_z_self_test(self):
        """When set to 1, the Z-Axis accelerometer performs self test."""
        return self.read_bit(MPU6050_RA_ACCEL_CONFIG,
                             MPU6050_ACONFIG_ZA_ST_BIT)

    def set_full_scale_accel_range(self, afs_sel):
        """
        Set full-scale accelerometer range.

        MPU6050_ACCEL_FS_2  = 0x00
        MPU6050_ACCEL_FS_4  = 0x01
        MPU6050_ACCEL_FS_8  = 0x02
        MPU6050_ACCEL_FS_16 = 0x03
        """
        return self.write_bits(MPU6050_RA_ACCEL_CONFIG,
                               MPU6050_ACONFIG_AFS_SEL_BIT,
                               MPU6050_ACONFIG_AFS_SEL_LENGTH,
                               afs_sel)

    def get_full_scale_accel_range(self):
        """
        Get current full-scale accelerometer range setting.

        --------------------------------
        SFS_SEL | Full scale accel range
        --------------------------------
        0       | +/- 2g
        1       | +/- 4g
        2       | +/- 8g
        3       | +/- 16g
        """
        return self.read_bits(MPU6050_RA_ACCEL_CONFIG,
                              MPU6050_ACONFIG_AFS_SEL_BIT,
                              MPU6050_ACONFIG_AFS_SEL_LENGTH)

    # FIFO_EN
    def set_temp_fifo_enabled(self, enabled):
        """
        Set temperature FIFO enabled value.

        When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L
        (Registers 65 and 66) to be written into the FIFO buffer.
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_TEMP_FIFO_EN_BIT,
                              enabled)

    def get_temp_fifo_enabled(self):
        """Get temperature FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_TEMP_FIFO_EN_BIT)

    def set_x_gyro_fifo_enabled(self, enabled):
        """
        Set gyroscope X-axis FIFO enabled value.

        When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L
        (Registers 67 and 68) to be written into the FIFO buffer.
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_XG_FIFO_EN_BIT,
                              enabled)

    def get_x_gyro_fifo_enabled(self):
        """Get gyroscope X-axis FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_XG_FIFO_EN_BIT)

    def set_y_gyro_fifo_enabled(self, enabled):
        """
        Set gyroscope Y-axis FIFO enabled value.

        When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L
        (Registers 69 and 70) to be written into the FIFO buffer.
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_YG_FIFO_EN_BIT,
                              enabled)

    def get_y_gyro_fifo_enabled(self):
        """Get gyroscope Y-axis FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_YG_FIFO_EN_BIT)

    def set_z_gyro_fifo_enabled(self, enabled):
        """
        Set gyroscope Z-axis FIFO enabled value.

        When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L
        (Registers 71 and 72) to be written into the FIFO buffer.
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_ZG_FIFO_EN_BIT,
                              enabled)

    def get_z_gyro_fifo_enabled(self):
        """Get gyroscope Z-axis FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_ZG_FIFO_EN_BIT)

    def set_accel_fifo_enabled(self, enabled):
        """
        Set accelerometer FIFO enabled value.

        When set to 1, this bit enables:
        ACCEL_XOUT_H, ACCEL_XOUT_L
        ACCEL_YOUT_H, ACCEL_YOUT_L
        ACCEL_ZOUT_H, ACCEL_ZOUT_L
        (Registers 59 to 64) to be written into the FIFO buffer
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_ZG_FIFO_EN_BIT,
                              enabled)

    def get_accel_fifo_enabled(self):
        """Get accelerometer FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_ZG_FIFO_EN_BIT)

    def set_slv2_fifo_enabled(self, enabled):
        """
        Set Slave 2 FIFO enabled value.

        When set to 1, this bit enables:
        EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 2
        to be written intothe FIFO buffer.
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_SLV2_FIFO_EN_BIT)

    def get_slv2_fifo_enabled(self):
        """Get Slave 2 FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_SLV2_FIFO_EN_BIT)

    def set_slv1_fifo_enabled(self, enabled):
        """
        Set Slave 1 FIFO enabled value.

        When set to 1, this bit enables:
        EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 1
        to be written intothe FIFO buffer.
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_SLV1_FIFO_EN_BIT)

    def get_slv1_fifo_enabled(self):
        """Get Slave 1 FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_SLV1_FIFO_EN_BIT)

    def set_slv0_fifo_enabled(self, enabled):
        """
        Set Slave 0 FIFO enabled value.

        When set to 1, this bit enables:
        EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 0
        to be written intothe FIFO buffer.
        """
        return self.write_bit(MPU6050_RA_FIFO_EN,
                              MPU6050_SLV0_FIFO_EN_BIT)

    def get_slv0_fifo_enabled(self):
        """Get Slave 0 FIFO enabled value."""
        return self.read_bit(MPU6050_RA_FIFO_EN,
                             MPU6050_SLV0_FIFO_EN_BIT)

    # I2C_MST_CTRL
    def set_multi_master_enabled(self, enabled):
        """Set multi-master capability."""
        return self.write_bit(MPU6050_RA_I2C_MST_CTRL,
                              MPU6050_MULT_MST_EN_BIT,
                              enabled)

    def get_multi_master_enabled(self):
        """Get multi-master enabled value."""
        return self.read_bit(MPU6050_RA_I2C_MST_CTRL,
                             MPU6050_MULT_MST_EN_BIT)

    def set_wait_for_external_sensor_enabled(self, enabled):
        """
        Set wait for external sensor enable.

        When setto 1,this bit delays the Data Ready interrupt until
        External Sensor data from  the  Slave  devices  have  been
        loaded into  the  EXT_SENS_DATA registers.
        """
        return self.write_bit(MPU6050_RA_I2C_MST_CTRL,
                              MPU6050_WAIT_FOR_ES_BIT,
                              enabled)

    def get_wait_for_external_sensor_enabled(self):
        """Get wait for external sensor value."""
        return self.read_bit(MPU6050_RA_I2C_MST_CTRL,
                             MPU6050_WAIT_FOR_ES_BIT)

    def set_slv3_fifo_enabled(self, enabled):
        """
        Set Slave 3 FIFO enabled value.

        When set to 1, this bit enables:
        EXT_SENS_DATA registers associated with Slave 3 to be written into
        the FIFO. The corresponding bits for Slaves 0-2 can be found in
        Register 35.
        """
        return self.write_bit(MPU6050_RA_I2C_MST_CTRL,
                              MPU6050_SLV3_FIFO_EN_BIT)

    def get_slv3_fifo_enabled(self):
        """Get Slave 3 FIFO enabled value."""
        return self.read_bit(MPU6050_RA_I2C_MST_CTRL,
                             MPU6050_SLV3_FIFO_EN_BIT)

    def set_master_transition(self, enabled):
        """
        Set slave read/write transition enabled value.

        When this bit equals 0, there is a restart between reads.
        When this bit equals 1, there is a stop and start marking the beginning
        of the next read.

        When a write follows a read, a stop and start is always enforced.
        """
        return self.write_bit(MPU6050_RA_I2C_MST_CTRL,
                              MPU6050_I2C_MST_P_NSR_BIT,
                              enabled)

    def get_master_transition(self):
        """Get slave read/write transition enabled value."""
        return self.read_bit(MPU6050_RA_I2C_MST_CTRL,
                             MPU6050_I2C_MST_P_NSR_BIT)

    def set_master_clock_speed(self, speed):
        """
        Set the I2C master clock speed divider.

        MPU6050_CLOCK_DIV_348 = 0x0
        MPU6050_CLOCK_DIV_333 = 0x1
        MPU6050_CLOCK_DIV_320 = 0x2
        MPU6050_CLOCK_DIV_308 = 0x3
        MPU6050_CLOCK_DIV_296 = 0x4
        MPU6050_CLOCK_DIV_286 = 0x5
        MPU6050_CLOCK_DIV_276 = 0x6
        MPU6050_CLOCK_DIV_267 = 0x7
        MPU6050_CLOCK_DIV_258 = 0x8
        MPU6050_CLOCK_DIV_500 = 0x9
        MPU6050_CLOCK_DIV_471 = 0xA
        MPU6050_CLOCK_DIV_444 = 0xB
        MPU6050_CLOCK_DIV_421 = 0xC
        MPU6050_CLOCK_DIV_400 = 0xD
        MPU6050_CLOCK_DIV_381 = 0xE
        MPU6050_CLOCK_DIV_364 = 0xF
        """
        return self.write_bits(MPU6050_RA_I2C_MST_CTRL,
                               MPU6050_I2C_MST_CLK_BIT,
                               MPU6050_I2C_MST_CLK_LENGTH,
                               speed)

    def get_master_clock_speed(self):
        """
        Get the I2C master clock speed divider.

        ---------------------------------------------------------
        I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
        ------------+------------------------+-------------------
        0           | 348kHz                 | 23
        1           | 333kHz                 | 24
        2           | 320kHz                 | 25
        3           | 308kHz                 | 26
        4           | 296kHz                 | 27
        5           | 286kHz                 | 28
        6           | 276kHz                 | 29
        7           | 267kHz                 | 30
        8           | 258kHz                 | 31
        9           | 500kHz                 | 16
        10          | 471kHz                 | 17
        11          | 444kHz                 | 18
        12          | 421kHz                 | 19
        13          | 400kHz                 | 20
        14          | 381kHz                 | 21
        15          | 364kHz                 | 22
        """
        return self.read_bits(MPU6050_RA_I2C_MST_CTRL,
                              MPU6050_I2C_MST_CLK_BIT,
                              MPU6050_I2C_MST_CLK_LENGTH)

    # I2C_SLV0_ADDR
    # I2C_SLV0_REG
    # I2C_SLV0_CTRL
    # I2C_SLV1_ADDR
    # I2C_SLV1_REG
    # I2C_SLV1_CTRL
    # I2C_SLV2_ADDR
    # I2C_SLV2_REG
    # I2C_SLV2_CTRL
    # I2C_SLV3_ADDR
    # I2C_SLV3_REG
    # I2C_SLV3_CTRL
    # I2C_SLV4_ADDR
    # I2C_SLV4_REG
    # I2C_SLV4_CTRL
    # I2C_SLV4_DI

    # I2C_MST_STATUS
    def get_passthrough_status(self):
        """
        Get FSYNC interrupt status.

        This bit reflects the status of the FSYNC interrupt from an external
        device into the MPU-60X0. This is used as a way to pass an external
        interrupt through the MPU-60X0 to the host application processor.
        When set to 1, this bit will cause an interrupt if FSYNC_INT_EN is
        asserted in INT_PIN_CFG (Register 55).
        """
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_PASS_THROUGH_BIT)

    def get_slv4_done(self):
        """"Get slave 4 transaction completed."""
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_I2C_SLV4_DONE_BIT)

    def get_lost_arbitration(self):
        """
        Get I2C Master lost arbitration.

        This bit automatically sets to 1 when the I2C Master has lost
        arbitration of the auxiliary I2C bus(an error condition).
        This triggers an interrupt if the I2C_MST_INT_ENbit in the INT_ENABLE
        register (Register 56) is asserted.
        """
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_I2C_LOST_ARB_BIT)

    def get_slv4_nack(self):
        """
        Get Slave 4 nack status.

        This bit automatically sets to 1 when the I2C Master receives a NACK
        in a transaction with Slave 4.
        This triggers an interrupt if the I2C_MST_INT_ENbit in the INT_ENABLE
        register (Register 56) is asserted.
        """
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_I2C_SLV4_NACK_BIT)

    def get_slv3_nack(self):
        """
        Get Slave 3 nack status.

        This bit automatically sets to 1 when the I2C Master receives a NACK
        in a transaction with Slave 3.
        This triggers an interrupt if the I2C_MST_INT_ENbit in the INT_ENABLE
        register (Register 56) is asserted.
        """
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_I2C_SLV3_NACK_BIT)

    def get_slv2_nack(self):
        """
        Get Slave 2 nack status.

        This bit automatically sets to 1 when the I2C Master receives a NACK
        in a transaction with Slave 2.
        This triggers an interrupt if the I2C_MST_INT_ENbit in the INT_ENABLE
        register (Register 56) is asserted.
        """
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_I2C_SLV2_NACK_BIT)

    def get_slv1_nack(self):
        """
        Get Slave 1 nack status.

        This bit automatically sets to 1 when the I2C Master receives a NACK
        in a transaction with Slave 1.
        This triggers an interrupt if the I2C_MST_INT_ENbit in the INT_ENABLE
        register (Register 56) is asserted.
        """
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_I2C_SLV1_NACK_BIT)

    def get_slv0_nack(self):
        """
        Get Slave 0 nack status.

        This bit automatically sets to 1 when the I2C Master receives a NACK
        in a transaction with Slave 0.
        This triggers an interrupt if the I2C_MST_INT_ENbit in the INT_ENABLE
        register (Register 56) is asserted.
        """
        return self.read_bit(MPU6050_RA_I2C_MST_STATUS,
                             MPU6050_MST_I2C_SLV0_NACK_BIT)

    # INT_PIN_CFG
    def set_interrupt_mode(self, mode):
        """
        Set interrupt logic level.

        0 | HIGH
        1 | LOW
        """
        return self.write_bit(MPU6050_RA_INT_PIN_CFG,
                              MPU6050_INTCFG_INT_LEVEL_BIT,
                              mode)

    def get_interrupt_mode(self):
        """
        Set interrupt logic level.

        0, the logic level for the INT pin is active high.
        1, the logic level for the INT pin is active low.
        """
        return self.read_bit(MPU6050_RA_INT_PIN_CFG,
                             MPU6050_INTCFG_INT_LEVEL_BIT)

    def set_interrupt_drive(self, drive):
        """
        Set interrup drive level.

        0 | PUSH-PULL
        1 | OPEN DRAIN
        """
        return self.write_bit(MPU6050_RA_INT_PIN_CFG,
                              MPU6050_INTCFG_INT_OPEN_BIT,
                              drive)

    def get_interrupt_drive(self):
        """
        Get interrup drive level.

        0, the INT pin is configured as push-pull.
        1, the INT pin is configured as open drain.
        """
        return self.read_bit(MPU6050_RA_INT_PIN_CFG,
                             MPU6050_INTCFG_INT_OPEN_BIT)

    def set_latch_interrupt(self, latch):
        """
        Set interrupt latch mode.

        0 | 50us pulse
        1 | Latch until INT pin cleared
        """
        return self.write_bit(MPU6050_RA_INT_PIN_CFG,
                              MPU6050_INTCFG_LATCH_INT_EN_BIT,
                              latch)

    def get_latch_interrupt(self):
        """
        Get interrupt latch mode.

        0, the INT pin emits a 50us long pulse.
        1, the INT pin is held high until the interrupt is cleared.
        """
        return self.read_bit(MPU6050_RA_INT_PIN_CFG,
                             MPU6050_INTCFG_LATCH_INT_EN_BIT)

    def set_interrupt_rd_clear(self, mode):
        """
        Set clear interrupts status bits mode.

        0 | Only reading INT_STATUS
        1 | Any read operation
        """
        return self.write_bit(MPU6050_RA_INT_PIN_CFG,
                              MPU6050_INTCFG_INT_RD_CLEAR_BIT,
                              mode)

    def get_interrupt_rd_clear(self):
        """
        Get clear interrupts status bits mode.

        0, interrupt status bits are cleared only by reading INT_STATUS
        (Register 58)
        1,interrupt status bits are cleared on any read operation
        """
        return self.read_bit(MPU6050_RA_INT_PIN_CFG,
                             MPU6050_INTCFG_INT_RD_CLEAR_BIT)

    def set_fsync_interrupt_level(self, mode):
        """
        Set FSYNC logic level.

        0 | FSYNC active HIGH
        1 | FSYNC active LOW
        """
        return self.write_bit(MPU6050_RA_INT_PIN_CFG,
                              MPU6050_INTCFG_FSYNC_INT_EN_BIT,
                              mode)

    def get_fsync_interrupt_level(self):
        """
        Get FSYNC logic level.

        0, the logic level for the FSYNC pin (when used as an interrupt to the
        host processor) is active high.
        1, the logic level for the FSYNC pin (when used as an interrupt to the
        host processor) is active low.
        """
        return self.read_bit(MPU6050_RA_INT_PIN_CFG,
                             MPU6050_INTCFG_FSYNC_INT_EN_BIT)

    def set_fsync_interrupt_enabled(self, enabled):
        """
        Set FSYNC pin interrupt enabled setting.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_INT_PIN_CFG,
                              MPU6050_INTCFG_FSYNC_INT_EN_BIT,
                              enabled)

    def get_fsync_interrupt_enabled(self):
        """
        Get FSYNC pin interrupt enabled setting.

        0, this bit disables the FSYNC pin from causing an interrupt to the
        host processor.
        1, this bit enables the FSYNC pin to be used as an interrupt to the
        host processor.
        """
        return self.read_bit(MPU6050_RA_INT_PIN_CFG,
                             MPU6050_INTCFG_FSYNC_INT_EN_BIT)

    def set_i2c_bypass_enabled(self, enabled):
        """
        Set I2C bypass enabled status.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_INT_PIN_CFG,
                              MPU6050_INTCFG_I2C_BYPASS_EN_BIT,
                              enabled)

    def get_i2c_bypass_enabled(self):
        """
        Set I2C bypass enabled status.

        When this bit is equal to 0, the host application processor will not be
        able to directly access the auxiliary I2C bus of the MPU-60X0
        regardless of the state of I2C_MST_EN(Register 106 bit[5]).

        When this bit is equal to 1 and I2C_MST_EN(Register 106 bit[5]) is
        equal to 0, the host application processor will be able to directly
        access the auxiliary I2C bus of the MPU-60X0.
        """
        return self.read_bit(MPU6050_RA_INT_PIN_CFG,
                             MPU6050_INTCFG_I2C_BYPASS_EN_BIT)

    # INT_ENABLE
    def set_fifo_buffer_overflow_interrupt_enabled(self, enabled):
        """
        Set FIFO buffer overflow interrupt enabled.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_INT_ENABLE,
                              MPU6050_INTERRUPT_FIFO_OFLOW_BIT,
                              enabled)

    def get_fifo_buffer_overflow_interrupt_enabled(self, enabled):
        """
        Get FIFO buffer overflow interrupt enabled.

        When set to 1, this bit enables a FIFO buffer overflow to generate  an
        interrupt
        """
        return self.write_bit(MPU6050_RA_INT_ENABLE,
                              MPU6050_INTERRUPT_FIFO_OFLOW_BIT,
                              enabled)

    def set_i2c_master_interrupt_enabled(self, enabled):
        """
        Set I2C master interrupt enabled.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_INT_ENABLE,
                              MPU6050_INTERRUPT_I2C_MST_INT_BIT,
                              enabled)

    def get_i2c_master_interrupt_enabled(self, enabled):
        """
        Get I2C master interrupt enabled.

        When set to 1, this bit enables any of the I2C Master interrupt sources
        to generate an interrupt.
        """
        return self.write_bit(MPU6050_RA_INT_ENABLE,
                              MPU6050_INTERRUPT_I2C_MST_INT_BIT,
                              enabled)

    def set_data_ready_interrupt_enabled(self, enabled):
        """
        Set Data Ready interrupt enabled.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_INT_ENABLE,
                              MPU6050_INTERRUPT_DATA_RDY_BIT,
                              enabled)

    def get_data_ready_interrupt_enabled(self):
        """
        Get Data Ready interrupt enabled.

        When set to 1, this bit enables the Data Ready interrupt, which occurs
        each time a write operation to all of the sensor registers has been
        completed.
        """
        return self.read_bit(MPU6050_RA_INT_ENABLE,
                             MPU6050_INTERRUPT_DATA_RDY_BIT)

    # INT_STATUS
    def get_fifo_overflow_interrupt(self):
        """
        Get FIFO overflow interrupt status.

        This bit automatically sets to 1 when a FIFO buffer overflow interrupt
        has been generated.

        The bit clears to 0 after the register has been read.
        """
        return self.read_bit(MPU6050_RA_INT_STATUS,
                             MPU6050_INTERRUPT_FIFO_OFLOW_BIT)

    def get_i2c_master_interrupt(self):
        """
        Get I2C Master interrupt status.

        This bit automatically sets to 1 when an I2C Master interrupt has been
        enerated.For a list of I2C Master interrupts, please refer to
        Register 54.
        The bit clears to 0 after the register has been read.
        """
        return self.read_bit(MPU6050_RA_INT_STATUS,
                             MPU6050_INTERRUPT_I2C_MST_INT_BIT)

    def get_data_ready_interrupt(self):
        """
        Get Data Ready interrupt status.

        This bit automatically sets to 1 when a Data Ready interrupt is
        generated.
        The bit clears to 0 after the register has been read.
        """
        return self.read_bit(MPU6050_RA_INT_STATUS,
                             MPU6050_INTERRUPT_DATA_RDY_BIT)

    # Accelerometer Measurements
    def accel(self):
        """
        Get 3-axis accelerometer readings.

        --------------------------------------------
        AFS_SEL | Full Scale Range | LSB Sensitivity
        --------+------------------+----------------
        0       | +/- 2g           | 16384 LSB/g
        1       | +/- 4g           | 8192 LSB/g
        2       | +/- 8g           | 4096 LSB/g
        3       | +/- 16g          | 2048 LSB/g
        """
        buff = self.read_bytes(MPU6050_RA_ACCEL_XOUT_H, 6)
        ax = self.bytes_toint(buff[0], buff[1])
        ay = self.bytes_toint(buff[2], buff[3])
        az = self.bytes_toint(buff[4], buff[5])
        return (ax, ay, az)

    def accel_x(self):
        """Get X-axis accelerometer reading."""
        buff = self.read_bytes(MPU6050_RA_ACCEL_XOUT_H, 2)
        return self.bytes_toint(buff[0], buff[1])

    def accel_y(self):
        """Get Y-axis accelerometer reading."""
        buff = self.read_bytes(MPU6050_RA_ACCEL_YOUT_H, 2)
        return self.bytes_toint(buff[2], buff[3])

    def accel_z(self):
        """Get Z-axis accelerometer reading."""
        buff = self.read_bytes(MPU6050_RA_ACCEL_ZOUT_H, 2)
        return self.bytes_toint(buff[4], buff[5])

    # Temperature Measurement
    def temperature(self):
        """Get current internal temperature."""
        buff = self.read_bytes(MPU6050_RA_TEMP_OUT_H, 2)
        return self.bytes_toint(buff[0], buff[1])

    def temp_in_celsius(self):
        """Get current internal temperature in celsius."""
        return (self.temperature() / 340 + 36.53)

    # Gyroscope Measurements
    def gyro(self):
        """
        Get 3-axis gyroscope readings.

        ---------------------------------------------
        FS_SEL | Full Scale Range   | LSB Sensitivity
        -------+--------------------+----------------
        0      | +/- 250 degrees/s  | 131 LSB/deg/s
        1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
        2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
        3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
        """
        buff = self.read_bytes(MPU6050_RA_GYRO_XOUT_H, 6)
        gx = self.bytes_toint(buff[0], buff[1])
        gy = self.bytes_toint(buff[2], buff[3])
        gz = self.bytes_toint(buff[4], buff[5])
        return (gx, gy, gz)

    def gyro_x(self):
        """Get X-axis gyroscope reading."""
        buff = self.read_bytes(MPU6050_RA_GYRO_XOUT_H, 2)
        return self.bytes_toint(buff[0], buff[1])

    def gyro_y(self):
        """Get Y-axis gyroscope reading."""
        buff = self.read_bytes(MPU6050_RA_GYRO_YOUT_H, 2)
        return self.bytes_toint(buff[2], buff[3])

    def gyro_z(self):
        """Get Z-axis gyroscope reading."""
        buff = self.read_bytes(MPU6050_RA_GYRO_ZOUT_H, 2)
        return self.bytes_toint(buff[4], buff[5])

    # EXT_SENS_DATA_00 through EXT_SENS_DATA_23
    # I2C_SLV0_DO
    # I2C_SLV1_DO
    # I2C_SLV2_DO
    # I2C_SLV3_DO
    # I2C_MST_DELAY_CTRL
    # SIGNAL_PATH_RESET
    def gyro_path_reset(self):
        """1 resets the gyroscope analog and digital signal paths."""
        self.reset_flag = True
        return self.write_bit(MPU6050_RA_SIGNAL_PATH_RESET,
                              MPU6050_PATHRESET_GYRO_RESET_BIT,
                              True)

    def accel_path_reset(self):
        """1 resets the accelerometer analog and digital signal paths."""
        self.reset_flag = True
        return self.write_bit(MPU6050_RA_SIGNAL_PATH_RESET,
                              MPU6050_PATHRESET_ACCEL_RESET_BIT,
                              True)

    def temperature_path_reset(self):
        """1 resets the temperature sensor analog and digital signal paths."""
        self.reset_flag = True
        return self.write_bit(MPU6050_RA_SIGNAL_PATH_RESET,
                              MPU6050_PATHRESET_TEMP_RESET_BIT,
                              True)

    # USER_CTRL
    def set_fifo_enabled(self, enabled):
        """
        Set FIFO enabled.

        0 | FIFO buffer disabled
        1 | FIFO operations enabled
        """
        return self.write_bit(MPU6050_RA_USER_CTRL,
                              MPU6050_USERCTRL_FIFO_EN_BIT,
                              enabled)

    def get_fifo_enabled(self):
        """
        Get FIFO disabled.

        1, this bit enables FIFO operations.
        0, the FIFO buffer is disabled.The FIFO buffer cannot be written to or
        read from while disabled.

        The FIFO buffer's state does not change unless the MPU-60X0 is power
        cycled.
        """
        return self.read_bit(MPU6050_RA_USER_CTRL,
                             MPU6050_USERCTRL_FIFO_EN_BIT)

    def set_master_mode_enabled(self, enabled):
        """
        Set Master mode enabled.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_USER_CTRL,
                              MPU6050_USERCTRL_I2C_MST_EN_BIT,
                              enabled)

    def get_master_mode_enabled(self):
        """
        Get Master mode enabled.

        1, this bit enables I2C Master Mode.
        0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically driven
        by the primary I2C bus (SDA and SCL).
        """
        return self.read_bit(MPU6050_RA_USER_CTRL,
                             MPU6050_USERCTRL_I2C_MST_EN_BIT)

    def fifo_reset(self):
        """
        Set FIFO reset.

        This bit resetsthe FIFO buffewhen set to 1 while FIFO_EN equals 0.
        This bit automatically clears to 0 after the reset has been triggered.
        """
        self.reset_flag = True
        return self.write_bit(MPU6050_RA_USER_CTRL,
                              MPU6050_USERCTRL_FIFO_RESET_BIT,
                              True)

    def master_mode_reset(self):
        """
        Set Master reset.

        This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
        This bit automatically clears to 0 after the reset has been triggered.
        """
        self.reset_flag = True
        return self.write_bit(MPU6050_RA_USER_CTRL,
                              MPU6050_USERCTRL_I2C_MST_RESET_BIT,
                              True)

    def sensors_reset(self):
        """
        Set all sensors reset.

        When set to 1, this bit resets the signal paths for all sensors
        (gyroscopes, accelerometers,  and  temperature  sensor).
        This operation will also clear the sensor registers.
        This bit automatically clears to 0 after the reset has been triggered.

        When resetting only the signal path (and not the sensor registers),
        please use Register 104, SIGNAL_PATH_RESET.
        """
        self.reset_flag = True
        return self.write_bit(MPU6050_RA_USER_CTRL,
                              MPU6050_USERCTRL_SIG_COND_RESET_BIT,
                              True)

    # PWR_MGMT_1
    def device_reset(self):
        """
        Reset all internal registers to their default values.

        The bit automatically clears to 0 once the reset is done.

        Note:
        When using SPI interface, user should use DEVICE_RESET (register 107)
        as well as SIGNAL_PATH_RESET (register 104) to ensure the reset is
        performed properly.
        The sequence used should be:
          1. Set DEVICE_RESET = 1 (PWR_MGMT_1)
          2. Wait 100ms
          3. Set GYRO_RESET = ACCEL_RESET = TEMP_RESET = 1 (SIGNAL_PATH_RESET)
          4. Wait 100ms
        """
        self.reset_flag = True
        return self.write_bit(MPU6050_RA_PWR_MGMT_1,
                              MPU6050_PWR1_DEVICE_RESET_BIT,
                              True)

    def set_sleep_enabled(self, enabled):
        """
        Set sleep mode status.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_1,
                              MPU6050_PWR1_SLEEP_BIT,
                              enabled)

    def get_sleep_enabled(self):
        """
        Get sleep mode.

        1, this bit puts the MPU-60X0 into sleep mode.
        """
        return self.read_bit(MPU6050_RA_PWR_MGMT_1,
                             MPU6050_PWR1_SLEEP_BIT)

    def set_cycle_enabled(self, enabled):
        """Set cycle eneabled."""
        return self.write_bit(MPU6050_RA_PWR_MGMT_1,
                              MPU6050_PWR1_CYCLE_BIT,
                              enabled)

    def get_cycle_enabled(self):
        """
        Get cycle enabled.

        When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will
        cycle between sleep mode and waking up to take a single  sample of
        data from active sensors at a rate determined by LP_WAKE_CTRL
        (register 108).
        """
        return self.read_bit(MPU6050_RA_PWR_MGMT_1,
                             MPU6050_PWR1_CYCLE_BIT)

    def set_temperature_sensor_disabled(self, disabled):
        """
        Set temperature sensor disabled.

        0 | Enabled
        1 | Disabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_1,
                              MPU6050_PWR1_TEMP_DIS_BIT,
                              disabled)

    def get_temperature_sensor_disabled(self):
        """
        Get temperature sensor disabled.

        1, this bit disables the temperature sensor.
        """
        return self.read_bit(MPU6050_RA_PWR_MGMT_1,
                             MPU6050_PWR1_TEMP_DIS_BIT)

    def set_clock_source(self, clksel):
        """
        Set clock source setting.

        MPU6050_CLOCK_INTERNAL   = 0x00
        MPU6050_CLOCK_PLL_XGYRO  = 0x01
        MPU6050_CLOCK_PLL_YGYRO  = 0x02
        MPU6050_CLOCK_PLL_ZGYRO  = 0x03
        MPU6050_CLOCK_PLL_EXT32K = 0x04
        MPU6050_CLOCK_PLL_EXT19M = 0x05
        MPU6050_CLOCK_KEEP_RESET = 0x07
        """
        self.write_bits(MPU6050_RA_PWR_MGMT_1,
                        MPU6050_PWR1_CLKSEL_BIT,
                        MPU6050_PWR1_CLKSEL_LENGTH,
                        clksel)

    def get_clock_source(self):
        """
        Get clock source setting.

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
        return self.read_bits(MPU6050_RA_PWR_MGMT_1,
                              MPU6050_PWR1_CLKSEL_BIT,
                              MPU6050_PWR1_CLKSEL_LENGTH)

    # PWR_MGMT_2
    def set_low_power_wake_control(self, frec):
        """
        Set frequency of wake-ups during Accel Only Low Power Mode.

        0 | 1.25 Hz
        1 | 5    Hz
        2 | 20   Hz
        3 | 40   Hz
        """
        return self.write_bits(MPU6050_RA_PWR_MGMT_2,
                               MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                               MPU6050_PWR2_LP_WAKE_CTRL_LENGTH,
                               frec)

    def get_low_power_wake_control(self):
        """
        Get frequency of wake-ups during Accel Only Low Power Mode.

        --------------------------------
        LP_WAKE_CTRL | Wake_up Frecuency
        -------------+------------------
        0            |      1.25 Hz
        1            |        5  Hz
        2            |       20  Hz
        3            |       40  Hz
        """
        return self.read_bits(MPU6050_RA_PWR_MGMT_2,
                              MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                              MPU6050_PWR2_LP_WAKE_CTRL_LENGTH)

    def set_accel_x_standby_enabled(self, enabled):
        """
        Set X axis accelerometer standby mode.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_2,
                              MPU6050_PWR2_STBY_XA_BIT,
                              enabled)

    def get_accel_x_standby_enabled(self):
        """Get X axis accelerometer standby mode."""
        return self.read_bit(MPU6050_RA_PWR_MGMT_2,
                             MPU6050_PWR2_STBY_XA_BIT)

    def set_accel_y_standby_enabled(self, enabled):
        """
        Set Y axis accelerometer standby mode.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_2,
                              MPU6050_PWR2_STBY_YA_BIT,
                              enabled)

    def get_accel_y_standby_enabled(self):
        """Get Y axis accelerometer standby mode."""
        return self.read_bit(MPU6050_RA_PWR_MGMT_2,
                             MPU6050_PWR2_STBY_YA_BIT)

    def set_accel_z_standby_enabled(self, enabled):
        """
        Set Z axis accelerometer standby mode.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_2,
                              MPU6050_PWR2_STBY_ZA_BIT,
                              enabled)

    def get_accel_z_standby_enabled(self):
        """Get Z axis accelerometer standby mode."""
        return self.read_bit(MPU6050_RA_PWR_MGMT_2,
                             MPU6050_PWR2_STBY_ZA_BIT)

    def set_gyro_x_standby_enabled(self, enabled):
        """
        Set X axis gyroscope standby mode.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_2,
                              MPU6050_PWR2_STBY_XG_BIT,
                              enabled)

    def get_gyro_x_standby_enabled(self):
        """Get X axis gyroscope standby mode."""
        return self.read_bit(MPU6050_RA_PWR_MGMT_2,
                             MPU6050_PWR2_STBY_XG_BIT)

    def set_gyro_y_standby_enabled(self, enabled):
        """
        Set Y axis gyroscope standby mode.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_2,
                              MPU6050_PWR2_STBY_YG_BIT,
                              enabled)

    def get_gyro_y_standby_enabled(self):
        """Get Y axis gyroscope standby mode."""
        return self.read_bit(MPU6050_RA_PWR_MGMT_2,
                             MPU6050_PWR2_STBY_YG_BIT)

    def set_gyro_z_standby_enabled(self, enabled):
        """
        Set Z axis gyroscope standby mode.

        0 | Disabled
        1 | Enabled
        """
        return self.write_bit(MPU6050_RA_PWR_MGMT_2,
                              MPU6050_PWR2_STBY_ZG_BIT,
                              enabled)

    def get_gyro_z_standby_enabled(self):
        """Get Z axis gyroscope standby mode."""
        return self.read_bit(MPU6050_RA_PWR_MGMT_2,
                             MPU6050_PWR2_STBY_ZG_BIT)

    # FIFO_COUNT
    def get_fifo_count(self):
        """Get FIFO count value."""
        return self.read_bytes(MPU6050_RA_FIFO_COUNTH, 2)

    # FIFO_R_W
    def get_fifo_data(self, data):
        """Get FIFO data."""
        return self.read_bytes(MPU6050_RA_FIFO_R_W, data)

    # WHO_AM_I
    def who_am_i(self):
        """Get Device ID."""
        return self.read_bits(MPU6050_RA_WHO_AM_I,
                              MPU6050_WHO_AM_I_BIT,
                              MPU6050_WHO_AM_I_LENGTH)

    # I2C
    def scan(self):
        """Scan all the I2C adresses."""
        adresses = self.i2c.scan()
        if adresses:
            for adress in adresses:
                print(adress)
        else:
            "No device found."

    def read_bit(self, register, bit_num):
        """Read a single bit from an 8-bit device register."""
        self.i2c.readfrom_mem_into(self.address, register, self.buf)
        b = self.buf[0]
        b & (1 << bit_num)
        b >>= bit_num
        return b

    def write_bit(self, register, bit_num, data):
        """Write a single bit in an 8-bit device register."""
        self.buf = self.read_byte(register)
        b = self.buf[0]
        b = (b | (1 << bit_num)) if (data != 0) else (b & ~(1 << bit_num))
        return self.write_byte(register, b)

    def read_bits(self, register, bit_start, length):
        """Read multiple bits from an 8-bit device register."""
        self.i2c.readfrom_mem_into(self.address, register, self.buf)
        mask = ((1 << length) - 1) << (bit_start - length + 1)
        b = self.buf[0]
        b &= mask
        b >>= (bit_start - length + 1)
        return b

    def write_bits(self, register, bit_start, length, data):
        """Write multiples bits in an 8-bit device register."""
        self.buf = self.read_byte(register)
        b = self.buf[0]
        mask = ((1 << length) - 1) << (bit_start - length + 1)
        data <<= (bit_start - length + 1)
        data &= mask
        b &= ~(mask)
        b |= data
        return self.write_byte(register, b)

    def read_byte(self, register):
        """Read single byte from an 8-bit device register."""
        self.i2c.readfrom_mem_into(self.address, register, self.buf)
        return self.buf

    def write_byte(self, register, data):
        """Write a single byte in an 8-bit device register."""
        data = bytearray([data])
        self.i2c.writeto_mem(self.address, register, data)
        if (data == self.read_byte(register)) or (self.reset_flag):
            self.reset_flag = False
            return True
        raise MPUException()

    def read_bytes(self, register, length):
        """Read single byte from an 8-bit device register."""
        return self.i2c.readfrom_mem(self.address, register, length)

    # Helper
    def bytes_toint(self, msb, lsb):
        """Convert two bytes to signed integer."""
        if not msb & 0x80:
            return msb << 8 | lsb
        return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)

    # Methods
    def test_connection(self):
        """Verify the I2C connection."""
        return self.who_am_i() == 0x34

    def disable_all_interrupts(self):
        """Disable all interrupts."""
        self.write_byte(MPU6050_RA_INT_ENABLE, 0x00)

    def disable_fifo(self):
        """Disable FIFO."""
        self.write_byte(MPU6050_RA_FIFO_EN, 0x00)

    def disable_sleep(self):
        """Disable Sleep and Cycle."""
        self.set_sleep_enabled(False)
        self.set_cycle_enabled(False)

    def fifo_count(self):
        """Gyro and Accel data for averaging."""
        count = self.get_fifo_count()
        return self.bytes_toint(count[0], count[1])

    def initialize(self,
                   clk_sel=MPU6050_CLOCK_PLL_XGYRO,
                   dlpf_cfg=MPU6050_DLPF_BW_42,
                   fs_gyro=MPU6050_GYRO_FS_250,
                   fs_accel=MPU6050_ACCEL_FS_2):
        """Prepare for general usage."""
        self.device_reset()

        self.set_sleep_enabled(False)
        self.set_clock_source(clk_sel)
        self.set_dlpf_mode(dlpf_cfg)
        # Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        self.set_sample_rate(0x04)
        self.set_full_scale_gyro_range(fs_gyro)
        self.set_full_scale_accel_range(fs_accel)
        # Configure Interrupts and Bypass Enable
        self.set_latch_interrupt(True)
        self.set_i2c_bypass_enabled(True)
        self.set_data_ready_interrupt_enabled(True)
