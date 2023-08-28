import sys 
import asyncio
import time
import logging
import argparse

from pysk8 import setCoreLogLevel
from pysk8.core import SK8

async def hardware(device_name: str) -> None:
    async with SK8() as sk8: # this will disconnect any active connections when it goes out of scope

        # attempt to connect to the named device, with a scan timeout of 3s
        if not await sk8.connect(device_name):
            print(f'Failed to connect to device {device_name}')
            sys.exit(-1)

        while True:
            try:
                # show if the SK8 has external IMUs or ExtAna board connected
                print('Has IMUs: {} | Has ExtAna: {}'.format(await sk8.has_imus(False), await sk8.has_extana(True)))
                await asyncio.sleep(0.5)
            except asyncio.exceptions.CancelledError:
                print('Disconnecting...')
                break

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('device_name', help='SK8 device to connect to', type=str)
    parser.add_argument('-D', '--show_logs', help='Show DEBUG level log messages', action='store_true')
    args = parser.parse_args()

    # show full logging output from pysk8
    if args.show_logs:
        setCoreLogLevel(logging.DEBUG)

    asyncio.run(hardware(args.device_name))
