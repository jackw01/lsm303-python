# Copyright 2020 jackw01. Released under the MIT license.
__version__ = "1.0.0"

import smbus
import struct
import time

def _test():
    i2c_channel = 1
    bus = smbus.SMBus(i2c_channel)
    #device = L3GD20(bus)
    #while True:
    #    data = device.read()
    #    print([round(v, 2) for v in data])
    #    time.sleep(0.1)
