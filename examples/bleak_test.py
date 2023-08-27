import asyncio 
import sys
import os
from typing import Sequence, Any

sys.path.append(os.path.split(os.path.dirname(__file__))[0])
from pysk8.core import SK8

def test_imu(acc: Sequence[float], gyro: Sequence[float], mag: Sequence[float], imu_index: int, seq: int, timestamp: float, _: Any):
    print(acc, gyro, mag, imu_index, seq, timestamp)

async def main(device_name: str) -> None:
    
    async with SK8() as sk8:
        await sk8.connect(device_name)

        print('Device info')
        print('Battery', await sk8.get_battery_level())
        print('Name', await sk8.get_device_name())
        print('FW', await sk8.get_firmware_version())

        # await sk8.pp_services()

        sk8.set_imu_callback(test_imu)
        await sk8.enable_imu_streaming([0])

        await asyncio.sleep(10)

        await sk8.disable_imu_streaming()
    

if __name__ == "__main__":
    asyncio.run(main(sys.argv[1]))

