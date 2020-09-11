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

ACCEL_MS2_PER_LSB = 0.00980665 # meters/second^2 per least significant bit

class LSM303(object):
    'LSM303 3-axis accelerometer/magnetometer'

    def __init__(self, i2c, hires=True):
        'Initialize the sensor'
        self._bus = i2c
        self._orientation = None

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

    def read_mag(self):
        'Read raw magnetic field in ???'
        return (0, 0, 0)

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
