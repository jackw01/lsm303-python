# Copyright 2020 jackw01. Released under the MIT license.
__version__ = '1.0.0'

import smbus
import struct
import time

LSM303_ADDRESS_ACCEL = 0x19 # 0011001x
LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20
LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21
LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22
LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23
LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24
LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25
LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26
LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27
LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28
LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29
LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A
LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B
LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C
LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D
LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E
LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F
LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30
LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31
LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32
LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33
LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34
LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35
LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36
LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37
LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38
LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39
LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A
LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B
LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C
LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D

LSM303_ADDRESS_MAG = 0x1E # 0011110x
LSM303_REGISTER_MAG_CRA_REG_M             = 0x00
LSM303_REGISTER_MAG_CRB_REG_M             = 0x01
LSM303_REGISTER_MAG_MR_REG_M              = 0x02
LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03
LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04
LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05
LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06
LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07
LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08
LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09
LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A
LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B
LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C
LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31
LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32

MAG_GAIN_1_3                              = 0x20 # +/- 1.3
MAG_GAIN_1_9                              = 0x40 # +/- 1.9
MAG_GAIN_2_5                              = 0x60 # +/- 2.5
MAG_GAIN_4_0                              = 0x80 # +/- 4.0
MAG_GAIN_4_7                              = 0xA0 # +/- 4.7
MAG_GAIN_5_6                              = 0xC0 # +/- 5.6
MAG_GAIN_8_1                              = 0xE0 # +/- 8.1

MAG_RATE_0_7                              = 0x00 # 0.75 H
MAG_RATE_1_5                              = 0x01 # 1.5 Hz
MAG_RATE_3_0                              = 0x62 # 3.0 Hz
MAG_RATE_7_5                              = 0x03 # 7.5 Hz
MAG_RATE_15                               = 0x04 # 15 Hz
MAG_RATE_30                               = 0x05 # 30 Hz
MAG_RATE_75                               = 0x06 # 75 Hz
MAG_RATE_220                              = 0x07 # 210 Hz

ACCEL_MS2_PER_LSB = 0.00980665 # meters/second^2 per least significant bit

GAUSS_TO_MICROTESLA = 100.0

class LSM303(object):
    'LSM303 3-axis accelerometer/magnetometer'

    def __init__(self, i2c, hires=True):
        'Initialize the sensor'
        self._bus = i2c

        # Enable the accelerometer - all 3 channels
        self._bus.write_i2c_block_data(LSM303_ADDRESS_ACCEL,
                                       LSM303_REGISTER_ACCEL_CTRL_REG1_A,
                                       [0b01000111])

        # Select hi-res (12-bit) or low-res (10-bit) output mode.
        # Low-res mode uses less power and sustains a higher update rate,
        # output is padded to compatible 12-bit units.
        if hires:
            self._bus.write_i2c_block_data(LSM303_ADDRESS_ACCEL,
                                           LSM303_REGISTER_ACCEL_CTRL_REG4_A,
                                           [0b00001000])
        else:
            self._bus.write_i2c_block_data(LSM303_ADDRESS_ACCEL,
                                           LSM303_REGISTER_ACCEL_CTRL_REG4_A,
                                           [0b00000000])

        # Enable the magnetometer
        self._bus.write_i2c_block_data(LSM303_ADDRESS_MAG,
                                       LSM303_REGISTER_MAG_MR_REG_M,
                                       [0b00000000])

        self.set_mag_gain(MAG_GAIN_1_3)

    def read_accel(self):
        'Read raw acceleration in meters/second squared'
        # Read as signed 12-bit little endian values
        accel_bytes = self._bus.read_i2c_block_data(LSM303_ADDRESS_ACCEL,
                                                    LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80,
                                                    6)
        accel_raw = struct.unpack('<hhh', bytearray(accel_bytes))

        return (
            (accel_raw[0] >> 4) * ACCEL_MS2_PER_LSB,
            (accel_raw[1] >> 4) * ACCEL_MS2_PER_LSB,
            (accel_raw[2] >> 4) * ACCEL_MS2_PER_LSB,
        )

    def set_mag_gain(self, gain):
        'Set magnetometer gain'
        self._gain = gain
        if gain == MAG_GAIN_1_3:
            self._lsb_per_gauss_xy = 1100
            self._lsb_per_gauss_z = 980
        elif gain == MAG_GAIN_1_9:
            self._lsb_per_gauss_xy = 855
            self._lsb_per_gauss_z = 760
        elif gain == MAG_GAIN_2_5:
            self._lsb_per_gauss_xy = 670
            self._lsb_per_gauss_z = 600
        elif gain == MAG_GAIN_4_0:
            self._lsb_per_gauss_xy = 450
            self._lsb_per_gauss_z = 400
        elif gain == MAG_GAIN_4_7:
            self._lsb_per_gauss_xy = 400
            self._lsb_per_gauss_z = 355
        elif gain == MAG_GAIN_5_6:
            self._lsb_per_gauss_xy = 330
            self._lsb_per_gauss_z = 295
        elif gain == MAG_GAIN_8_1:
            self._lsb_per_gauss_xy = 230
            self._lsb_per_gauss_z = 205

        self._bus.write_i2c_block_data(LSM303_ADDRESS_MAG,
                                       LSM303_REGISTER_MAG_CRB_REG_M,
                                       [self._gain])

    def set_mag_rate(self, rate):
        'Set magnetometer rate'
        self._bus.write_i2c_block_data(LSM303_ADDRESS_MAG,
                                       LSM303_REGISTER_MAG_CRA_REG_M,
                                       [(rate & 0x07) << 2])

    def read_mag(self):
        'Read raw magnetic field in microtesla'
        # Read as signed 16-bit big endian values
        mag_bytes = self._bus.read_i2c_block_data(LSM303_ADDRESS_MAG,
                                                  LSM303_REGISTER_MAG_OUT_X_H_M,
                                                  6)
        mag_raw = struct.unpack('>hhh', bytearray(mag_bytes))

        return (
            mag_raw[0] / self._lsb_per_gauss_xy * GAUSS_TO_MICROTESLA,
            mag_raw[2] / self._lsb_per_gauss_xy * GAUSS_TO_MICROTESLA,
            mag_raw[1] / self._lsb_per_gauss_z * GAUSS_TO_MICROTESLA,
        )

def _test():
    i2c_channel = 1
    bus = smbus.SMBus(i2c_channel)
    device = LSM303(bus)
    while True:
        accel_data = device.read_accel()
        mag_data = device.read_mag()
        print(
            [round(v, 2) for v in accel_data],
            [round(v, 2) for v in mag_data]
        )
        time.sleep(0.1)
