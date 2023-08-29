# SK8 BLE sensor pack drivers 

The SK8 is a small Bluetooth Low Energy IMU sensor pack based on the Invensense MPU9250. It has the ability to attach a string of 4 other mini-IMUs using a micro-USB connection and stream data from all 5 IMUs simultaneously (TODO: expand description).

This repo is a reimplementation of the original [SK8 Python driver](https://github.com/andrewramsay/sk8-drivers/tree/master/pysk8) using the cross-platform [Bleak](https://github.com/hbldh/bleak) library. The original implementation only supported a specific BLE dongle with its own Bluetooth stack and serial interface to the host device. The older version should still work if you happen to have the same dongle, but the version in this repo will be the only one receiving updates and bugfixes in the future as it makes much more sense to have an implementation that should work with any suitable BLE dongle. 

## Installing 

TODO - update pypi

## Usage

Instances of the `pysk8.core.SK8` class represent a single SK8 device, and act as [asyncio](https://docs.python.org/3/library/asyncio.html) context managers:

The code snippet below shows how you can connect to an SK8 by name:
```python
import asyncio
from pysk8.core import SK8

async def main(device_name: str) -> None:
  async with SK8() as sk8:
    # connect to the named device with a default scan timeout of 3s
    if not await sk8.connect(device_name):
      print(f'Failed to connect to device {device_name}')
      sys.exit(-1)

    print(await sk8.get_battery_level())

  # (connection is closed when leaving the scope)

if __name__ == "__main__":
  asyncio.run(main(sys.argv[1]))
```

There are some simple examples scripts [here](https://github.com/idi-group/pysk8/tree/main/examples) that demonstrate various features of the API. 
