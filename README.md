# lsm303d-python
Python library for the LSM303D I2C accelerometer/magnetometer

## Usage
1. Install the Python `smbus` module for your platform (`sudo apt install python-smbus` for Debian or Ubuntu Linux)
2. `pip install lsm303-python`
3. `sudo lsm303_test`

## Example Code
```python
import time
import smbus
import lsm303

i2c_channel = 1
bus = smbus.SMBus(i2c_channel)

# Will raise OSError if device is not connected
device = lsm303.LSM303(bus)

while True:
    # Returns x,y,z tuple with values in degrees/second
    accel_data = device.read_accel()
    # Returns x,y,z tuple with values in microtesla
    mag_data = device.read_mag()
    print(
        [round(v, 2) for v in accel_data],
        [round(v, 2) for v in mag_data]
    )
    time.sleep(0.1)
```
