import asyncio
import time
import logging
import os
import struct
from configparser import ConfigParser
from typing import Any, Callable, List, Optional, Tuple, Sequence, Union

import bleak
from bleak import BleakClient, BleakScanner, BLEDevice, BleakGATTCharacteristic

from .extana import ExtAnaData
from .imu import IMUData
from .constants import (
    INT_LED_MAX,
    LED_MAX,
    LED_MIN,
    MAX_IMUS,
    SENSOR_ALL,
    UUID_BATTERY_LEVEL,
    UUID_DEVICE_NAME,
    UUID_EXTANA_LED,
    MAX_DEVICE_NAME_LEN,
    UUID_FIRMWARE_REVISION,
    UUID_HARDWARE_STATE,
    UUID_HARDWARE_STATE_TMP,
    EXT_HW_IMUS,
    EXT_HW_EXTANA,
    UUID_POLLING_OVERRIDE,
    UUID_IMU_SELECTION,
    UUID_SENSOR_SELECTION,
    UUID_IMU_CCC,
    UUID_EXTANA_CCC,
    UUID_EXTANA_IMU_STREAMING,
    UUID_EXTANA_IMU_STREAMING_TMP,
    IMU_DATA_STRUCT,
    EXTANA_DATA_STRUCT,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# bleak seems to log at DEBUG level by default which is very verbose
bleak_logger = logging.getLogger('bleak').setLevel(logging.WARN)

# signature expected for IMU callbacks
ImuCallback = Callable[[Sequence[float], Sequence[float], Sequence[float], int, int, float, Any], None]

# signature expected for ExtAna callbacks
ExtAnaCallback = Callable[[int, int, int, int, float, Any], None]

class SK8:

    def __init__(self, load_calibration: bool = True) -> None:
        self._firmware_version = 'unknown'
        self._imus = [IMUData(x) for x in range(MAX_IMUS)]
        self._extana_data = ExtAnaData()
        self._enabled_imus = []
        self._enabled_sensors = SENSOR_ALL
        self._packets = 0
        self._services = {}
        self._user_imu_callback = None
        self._user_imu_callback_data = None
        self._user_extana_callback = None
        self._user_extana_callback_data = None
        self._led_state = None
        self._hardware = -1
        self._uuid_chars = {}
        self._uuid_cccds = {}
        self._client: Optional[BleakClient] = None
        self._load_calibration = load_calibration
        self._name = None

    def _check_connected(self) -> bool:
        return self._client is not None and self._client.is_connected

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit method.
        """
        if exc_type is not None:
            logger.warning(f'__exit__ encountered an exception: {exc_type}, {exc_value}')
            logger.warning(exc_traceback)

        await self.disconnect()

    def __eq__(self, v: object) -> bool:
        """
        Devices considered equal if addresses match.
        """
        if not isinstance(v, SK8):
            return False

        if self._client is None or not self._client.is_connected or v._client is None or not v._client.is_connected:
            return False

        return self._client.address is not None and self._client.address == v._client.address

    def set_calibration(self, enabled: bool, imus: List[int]) -> None:
        """
        Set calibration state for attached IMUs.

        Args:
            enabled (bool): True to apply calibration to IMU data (if available). 
            False to output uncalibrated data.
            imus (list): indicates which IMUs the calibration state should be set on. 
            Empty list or [0, 1, 2, 3, 4] will apply to all IMUs, [0, 1] only to 
            first 2 IMUs, etc. 
        """
        if len(imus) == 0:
            imus = list(range(MAX_IMUS))

        for i in imus:
            if i < 0 or i >= MAX_IMUS:
                logger.warn(f'Invalid IMU index {i} in set_calibration')
                continue
            self._imus[i].set_calibration(enabled)

    def get_calibration(self) -> List[bool]:
        """
        Get calibration state for attached IMUs.

        Returns:
            list. A 5-element list of bools indicating if calibrated output is 
            currently enabled or disabled on the IMU with the corresponding index.
            Note that if calibration is enabled but no calibration file has been
            loaded, uncalibrated data will still be output!
        """
        return [x.get_calibration() for x in self._imus]

    def load_calibration(self, calibration_file: Optional[str] = None) -> bool:
        """
        Load calibration data for IMU(s) connected to this SK8.

        This method attempts to load a set of calibration data from a .ini file 
        produced by the sk8_calibration_gui application (TODO link!).

        By default, it will look for a file name "sk8calib.ini" in the current working
        directory. This can be overridden using the `calibration_file` parameter.

        Args:
            calibration_file (str): Path to a user-specified calibration file (ini format).

        Returns:
            True if any calibration data was loaded, False if none was. Note that True will be
            returned even if for example only 1 IMU had any calibration data available.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No connected device')
            return False

        logger.debug(f'Loading calibration for {self._client.address}')
        calibration_data = ConfigParser()
        path = calibration_file or os.path.join(os.getcwd(), 'sk8calib.ini')
        logger.debug(f'Attempting to load calibration from {path}')
        calibration_data.read(path)
        success = False
        for i in range(MAX_IMUS):
            s = f'{self._name}_IMU{i}'
            if s in calibration_data.sections():
                logger.debug(f'Calibration data for device {s} was detected, extracting...')
                success = success or self._imus[i].load_calibration(calibration_data[s])

        return success

    async def connect(self, name: Optional[str], address: Optional[str] = None, timeout: float = 3.0) -> bool:
        """
        Attempts to connect to an SK8 identified by name or address.

        Calling this method will run a BLE scan for a device using the given timeout period. 

        Either or both name and address parameters can be supplied. If both are given address
        will be used instead of name. 

        Args:
            name (str): an SK8 device name, or None
            address (str): an SK8 device address, or None
            timeout (float): scan timeout period

        Returns:
            True if the connection was successful, False otherwise
        """
        
        # decide how to search for the target device
        if name is None and address is None:
            logger.error('Must supply either a name or address to connect to!')
            return False

        if address is not None:
            # prefer address over name 
            logger.debug(f'Searching for device address={address}')
            ble_device: BLEDevice = await BleakScanner.find_device_by_address(address, timeout=timeout)
        else:
            logger.debug(f'Searching for device name={name}')
            ble_device: BLEDevice = await BleakScanner.find_device_by_name(name, timeout=timeout)

        if ble_device is None:
            logger.warning(f'Failed to find target device with name={name}/address={address}')
            return False

        # most of the examples in the Bleak repo simply do
        #   with BleakClient(...) as client:
        #       ...
        # which implicitly connects when the context manager object is created and disconnects
        # when it goes out of scaope. To keep the client around instead we need to 
        # create an instance of BleakClient normally and then call its connect() method
        self._client = BleakClient(ble_device.address)
        await self._client.connect()
        logger.debug('Connection succeeded')
        if self._load_calibration:
            logger.debug(f'Attempting to load calibration for addr={self._client.address}')
            self.load_calibration()

        return True

    async def disconnect(self) -> bool:
        """
        Disconnect the dongle from this SK8.

        Simply closes the active BLE connection to the device represented by the current instance.

        Returns:
            bool. True if connection was closed, False if not (e.g. if already closed).
        """
        result = False
        logger.debug(f'SK8.disconnect({self._client})')
        if self._client is not None:
            logger.debug('Calling disconnect')
            result = await self._client.disconnect()
            self._packets = 0

        return result

    def set_extana_callback(self, callback: ExtAnaCallback, data: Any = None) -> None:
        """
        Register a callback for incoming data packets from the SK8-ExtAna board.

        This method allows you to pass in a callable which will be called on 
        receipt of each packet sent from the SK8-ExtAna board. Set to `None` to
        disable it again.

        Args:
            callback: a callable with the following signature:
                (ana1, ana2, temp, seq, timestamp, data)
              where:
                ana1, ana2 = current values of the two analogue inputs
                temp = temperature sensor reading
                seq = packet sequence number (int, 0-255)
                timestamp = value of time.time() when packet received
                data = value of `data` parameter passed to this method
            data: an optional arbitrary object that will be passed as a 
                parameter to the callback
        """
        self._user_extana_callback = callback
        self._user_extana_callback_data = data

    async def enable_extana_streaming(self, include_imu: bool = False, enabled_sensors: int = SENSOR_ALL) -> bool:
        """
        Configures and enables sensor data streaming from the SK8-ExtAna device.

        By default this will cause the SK8 to only stream data from the analog 
        sensors on the SK8-ExtAna, but if `include_imu` is set to True, it will 
        also send data from the internal IMU in the SK8. 

        NOTE: only one streaming mode can be active at any time, so e.g. if you 
        want to stream IMU data normally, you must disable SK8-ExtAna streaming first. 

        Args:
            include_imu (bool): If False, only SK8-ExtAna packets will be streamed.
                If True, the device will also stream data from the SK8's internal IMU. 
            enabled_sensors (int): If `include_imu` is True, this can be used to 
                select which IMU sensors will be active. 

        Returns:
            bool. True if successful, False if an error occurred.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No connected device')
            return False

        # if we want to stream the internal IMU data too, set this up first
        await self.enable_imu_streaming([0], enabled_sensors)

        extana_imu_streaming_handle = self._get_characteristic_handle_from_uuid(UUID_EXTANA_IMU_STREAMING)
        extana_imu_streaming_tmp_handle = self._get_characteristic_handle_from_uuid(UUID_EXTANA_IMU_STREAMING_TMP)
        if extana_imu_streaming_handle is None and extana_imu_streaming_tmp_handle is None:
            logger.error('Failed to find handle for ExtAna configuration')
            return False

        # write to the characteristic that will enable sending of IMU packets while 
        # the ExtAna streaming is active
        if extana_imu_streaming_handle is not None:
            await self._write_attribute(extana_imu_streaming_handle, struct.pack('B', 1 if include_imu else 0))
        elif extana_imu_streaming_tmp_handle is not None:
            await self._write_attribute(extana_imu_streaming_tmp_handle, struct.pack('B', 1 if include_imu else 0))

        await self._client.start_notify(self._get_characteristic_from_uuid(UUID_EXTANA_CCC), self._extana_callback)

        self._enabled_sensors = enabled_sensors

        # have to add IMU #0 to enabled_imus if include_imu is True
        if include_imu:
            self._enabled_imus = [0]

        return True

    async def disable_extana_streaming(self) -> bool:
        """
        Disable SK8-ExtAna streaming for this device.

        Returns:
            True on success, False if an error occurred.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return False

        await self._client.stop_notify(self._get_characteristic_from_uuid(UUID_EXTANA_CCC))
        # reset IMU data state
        for imu in self._imus:
            imu.reset()
        self._enabled_imus = []
        return True

    async def get_extana_led(self, cached: bool = True) -> Tuple[int, int, int]:
        """
        Returns the current (R, G, B) colour of the SK8-ExtAna LED.

        Args:
            cached (bool): if True, returns the locally cached state of the LED (based
                on the last call to :meth:`set_extana_led`). Otherwise query the device
                for the current state.

        Returns:
            a 3-tuple (r, g, b) (all unsigned integers) in the range 0-255, or (-1, -1, -1) on error.
        """
        if cached:
            return self.led_state

        extana_led = self._get_characteristic_handle_from_uuid(UUID_EXTANA_LED)
        if extana_led is None:
            logger.warn('Failed to find handle for ExtAna LED')
            return (-1, -1, -1)

        rgb = await self._read_attribute(extana_led)
        if rgb is None:
            return (-1, -1, -1)

        return tuple(map(lambda x: int(x * (LED_MAX / INT_LED_MAX)), struct.unpack('<HHH', rgb)))
        
    def set_extana_led(self, r: int, g: int, b: int, check_state: bool = True) -> bool:
        """
        Update the colour of the RGB LED on the SK8-ExtAna board.

        Args:
            r (int): red channel, 0-255
            g (int): green channel, 0-255
            b (int): blue channel, 0-255
            check_state (bool): if True (default) and the locally cached LED state matches
                the given (r, g, b) triplet, pysk8 will NOT send any LED update command to
                the SK8. If you want to force the command to be sent even if the local state
                matches the new colour, set this to False. 

        Returns:
            True on success, False if an error occurred.
        """

        r, g, b = map(int, [r, g, b])

        if min([r, g, b]) < LED_MIN or max([r, g, b]) > LED_MAX:
            logger.warn(f'RGB channel values must be {LED_MIN}-{LED_MAX}')
            return False

        if check_state and (r, g, b) == self.led_state:
            return True

        # internally range is 0-3000
        ir, ig, ib = map(lambda x: int(x * (INT_LED_MAX / LED_MAX)), [r, g, b])
        val = struct.pack('<HHH', ir, ig, ib)

        extana_led = self._get_characteristic_handle_from_uuid(UUID_EXTANA_LED)
        if extana_led is None:
            logger.warn('Failed to find handle for ExtAna LED')
            return False
            
        if not self._write_attribute(extana_led, val):
            return False

        # update cached LED state if successful
        self.led_state = (r, g, b)
        return True

    def set_imu_callback(self, callback: ImuCallback, data: Any = None) -> None:
        """
        Register a callback for incoming IMU data packets.
        
        This method allows you to pass in a callbable which will be called on
        receipt of each IMU data packet sent by this SK8 device. Set to `None`
        to disable it again.

        Args:
            callback: a callable with the following signature:
                (acc, gyro, mag, imu_index, seq, timestamp, data)
              where:
                acc, gyro, mag = sensor data ([x,y,z] in each case)
                imu_index = originating IMU number (int, 0-4)
                seq = packet sequence number (int, 0-255)
                timestamp = value of time.time() when packet received
                data = value of `data` parameter passed to this method
            data: an optional arbitrary object that will be passed as a 
                parameter to the callback
        """
        self._user_imu_callback = callback
        self._user_imu_callback_data = data

    async def enable_imu_streaming(self, enabled_imus: List[int], enabled_sensors: int = SENSOR_ALL) -> bool:
        """
        Configures and enables IMU sensor data streaming.

        NOTE: only one streaming mode can be active at any time, so e.g. if you 
        want to stream IMU data, you must disable SK8-ExtAna streaming first.

        Args:
            enabled_imus (list): a list of distinct ints in the range `0`-`4`
                inclusive identifying the IMU. `0` is the SK8 itself, and 
                `1`-`4` are the subsidiary IMUs on the USB chain, starting
                from the end closest to the SK8. 
            enabled_sensors (int): to save battery, you can choose to enable some 
                or all of the sensors on each enabled IMU. By default, the
                accelerometer, magnetometer, and gyroscope are all enabled. Pass
                a bitwise OR of one or more of :const:`SENSOR_ACC`, 
                :const:`SENSOR_MAG`, and :const:`SENSOR_GYRO` to gain finer
                control over the active sensors.

        Returns:
            bool. True if successful, False if an error occurred.
        """

        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return False

        imus_enabled = 0
        for imu in enabled_imus:
            imus_enabled |= (1 << imu)

        if enabled_sensors == 0:
            logger.warn('Not enabling IMUs, no sensors enabled!')
            return False

        # need to configure the IMU state on the device first, then enable notifications
        imu_select = self._get_characteristic_handle_from_uuid(UUID_IMU_SELECTION)
        sensor_select = self._get_characteristic_handle_from_uuid(UUID_SENSOR_SELECTION)
        if imu_select is None or sensor_select is None:
            logger.error('Failed to configure IMUs for streaming!')
            return False

        logger.debug(f'setting IMU state = {imus_enabled:02X} on device {self._client.address}')
        await self._write_attribute(imu_select, struct.pack('B', imus_enabled))
        await self._write_attribute(sensor_select, struct.pack('B', enabled_sensors))

        await self._client.start_notify(self._get_characteristic_from_uuid(UUID_IMU_CCC), self._imu_callback)

        self._enabled_imus = enabled_imus
        self._enabled_sensors = enabled_sensors
        return True

    def get_enabled_imus(self) -> List[int]:
        """
        Returns the set of currently enabled IMUs.

        This method returns a copy of the list of enabled IMUs as most recently
        passed to the :meth:`enable_imu_streaming` method. 

        Returns:
            The list will contain ints from 0-4, up to a maximum of 5 entries, and may be empty if no IMUs are enabled. 
        """
        return self._enabled_imus

    def get_enabled_sensors(self) -> int:
        """
        Returns the bitmask indicating the currently active set of IMU sensors.

        This method returns a copy of the bitmask of enabled sensors as most recently
        passed to the :meth:`enable_imu_streaming` method. 

        Returns:
            A combination of the SENSOR_ constants, e.g. SENSOR_ALL, or SENSOR_ACC | SENSOR_GYRO.
        """
        return self._enabled_sensors

    async def disable_imu_streaming(self) -> bool:
        """
        Disable IMU streaming for this device.

        Returns:
            True on success, False if an error occurred.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return False

        await self._client.stop_notify(self._get_characteristic_from_uuid(UUID_IMU_CCC))
        # reset IMU data state
        for imu in self._imus:
            imu.reset()
        return True

    async def get_battery_level(self) -> int:
        """
        Reads the battery level descriptor on the device.

        Returns:
            int. If successful this will be a positive value representing the current 
            battery level as a percentage. On error, -1 is returned. 
        """

        battery_level = self._get_characteristic_handle_from_uuid(UUID_BATTERY_LEVEL)
        if battery_level is None:
            logger.warn('Failed to find handle for battery level')
            return -1

        level = await self._read_attribute(battery_level)
        if level is None:
            return -1
        return ord(level.decode('ascii'))

    async def get_device_name(self, cached: bool = True) -> Optional[str]:
        """
        Returns the SK8 device BLE name.

        Args:
            cached (bool): if True, returns the locally cached copy of the name. If this is
                set to False, or the name is not cached, it will read from the device instead.

        Returns:
            str. The current device name. May be `None` if an error occurs.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return None

        if cached and self._name is not None:
            return self._name

        device_name = self._get_characteristic_handle_from_uuid(UUID_DEVICE_NAME)
        if device_name is None:
            logger.warn('Failed to find handle for device name')
            return None

        name = await self._read_attribute(device_name)
        if name is None:
            return None

        self._name = name.decode('ascii')
        return self._name

    async def set_device_name(self, new_name: str) -> bool:
        """
        Sets a new BLE device name for this SK8.

        Args:
            new_name (str): the new device name as an ASCII string, max 20 characters.

        Returns:
            True if the name was updated successfully, False otherwise.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return False

        if len(new_name) > MAX_DEVICE_NAME_LEN:
            logger.error(f'Device name exceeds maximum length ({len(new_name)} > {MAX_DEVICE_NAME_LEN})')
            return False

        if new_name is None or len(new_name) == 0: # type: ignore
            logger.error('Device name cannot be set to None or empty string!')
            return False

        device_name = self._get_characteristic_handle_from_uuid(UUID_DEVICE_NAME)
        if device_name is None:
            logger.warn('Failed to find handle for device name')
            return False
        
        if await self._write_attribute(device_name, new_name.encode('ascii')):
            self._name = new_name
            return True

        return False

    def get_received_packets(self) -> int:
        """
        Returns number of received data packets.

        Returns:
            int. Number of sensor data packets received either since the connection
            was established, or since :meth:`reset_received_packets` was called.
        """
        return self._packets

    def reset_received_packets(self):
        """
        Reset counter tracking received data packets to zero
        """
        self._packets = 0

    async def get_firmware_version(self, cached: bool = True) -> Optional[str]:
        """
        Returns the SK8 device firmware version.

        Args:
            cached (bool): if True, returns the locally cached copy of the firmware version.
                If this is set to False, or the version is not cached, it will read from
                the device instead. 

        Returns:
            str. The current firmware version string. May be `None` if an error occurs.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return None

        if cached and self._firmware_version != 'unknown':
            return self._firmware_version

        char = self._get_characteristic_handle_from_uuid(UUID_FIRMWARE_REVISION)
        if char is None:
            logger.warn('Failed to find handle for firmware version')
            return None

        fwver = await self._read_attribute(char)
        if fwver is None:
            logger.error('Failed to retrieve firmware version')
            return None

        self._firmware_version = fwver.decode('ascii')
        return self._firmware_version

    def get_imu(self, imu_number: int) -> Optional[IMUData]:
        """
        Returns a :class:`pysk8.imu.IMUData` object representing one of the attached IMUs.

        Args:
            imu_number (int): a value from 0-4 inclusive identifying the IMU. `0` is the
                SK8 itself, and `1`-`4` are the subsidiary IMUs on the USB chain, starting
                from the end closest to the SK8. 

        Returns:
            :class:`pysk8.imu.IMUData` object, or `None` if an invalid index is supplied.
        """
        if imu_number < 0 or imu_number >= MAX_IMUS:
            return None
        return self._imus[imu_number]

    def get_extana(self) -> ExtAnaData:
        """
        Returns a :class:`pysk8.extana.ExtAnaData` object representing an attach SK8-ExtAna board.

        Returns:
            :class:`pysk8.extana.ExtAnaData` object.
        """
        return self._extana_data

    def _imu_callback(self, _: BleakGATTCharacteristic, data: bytearray) -> None:
        _data = IMU_DATA_STRUCT.unpack(data)
        #self._update_sensors(_data[:3], _data[3:6], _data[6:9], _data[9], _data[10], time.time())
        acc, gyro, mag, imu, seq = _data[:3], _data[3:6], _data[6:9], _data[9], _data[10]
        timestamp = time.time()
        self._imus[imu].update(acc, gyro, mag, seq, timestamp)
        # call the registered IMU callback if any
        if self._user_imu_callback is not None:
            self._user_imu_callback(acc, gyro, mag, imu, seq, timestamp, self._user_imu_callback_data)

    def _extana_callback(self, _: BleakGATTCharacteristic, data: bytearray) -> None:
        _data = EXTANA_DATA_STRUCT.unpack(data)
        ch1, ch2, temp, seq = _data[0], _data[1], _data[2], _data[3]
        timestamp = time.time()
        self._extana_data.update(ch1, ch2, temp, seq, timestamp)
        # call the registered ExtAna callback if any
        if self._user_extana_callback is not None:
            self._user_extana_callback(ch1, ch2, temp, seq, timestamp, self._user_extana_callback_data)

    async def _check_hardware(self):
        hardware_state = self._get_characteristic_handle_from_uuid(UUID_HARDWARE_STATE)
        hardware_state_tmp = self._get_characteristic_handle_from_uuid(UUID_HARDWARE_STATE_TMP)
        if hardware_state is None and hardware_state_tmp is None:
            logger.error('Failed to find handle for hardware state')
            return -1

        if hardware_state is not None:
            hardware = await self._read_attribute(hardware_state)
        elif hardware_state_tmp is not None:
            hardware = await self._read_attribute(hardware_state_tmp)
        else:
            hardware = None

        if hardware is None:
            return -1

        self.hardware = hardware[0]
        return self.hardware

    async def has_extana(self, cached: bool = True) -> bool:
        """
        Can be used to check if an SK8-ExtAna device is currently connected.
            
        NOTE: do not attempt to call while data streaming is active!

        Args:
            cached (bool): if True, use the cached value of the connected hardware
                state rather than querying the device. Set to False to force a query.

        Returns:
            bool. True if the SK8 currently has an SK8-ExtAna device attached, False otherwise.
        """
        if cached and self.hardware != -1:
            return True if (self.hardware & EXT_HW_EXTANA) else False

        result = await self._check_hardware()
        return True if (result & EXT_HW_EXTANA) != 0 else False

    async def has_imus(self, cached: bool = True) -> bool:
        """
        Can be used to check if an external IMU chain is currently connected.
        
        NOTE: do not attempt to call while data streaming is active!

        Args:
            cached (bool): if True, use the cached value of the connected hardware
                state rather than querying the device. Set to False to force a query.

        Returns:
            bool. True if the SK8 currently has an IMU chain attached, False otherwise.
        """
        if cached and self.hardware != -1:
            return True if (self.hardware & EXT_HW_IMUS) else False

        result = await self._check_hardware()
        return True if (result & EXT_HW_IMUS) != 0 else False

    async def dump_services(self) -> None:
        """
        Generate a dump of service, charactertistic and descriptor information for this device.

        Returns:
            Nothing
        """
        if self._client is None or not self._client.is_connected:
            logger.error('Device is not set or not connected')
            return

        for service in self._client.services:
            logger.info("[Service] %s", service)

            for char in service.characteristics:
                if "read" in char.properties:
                    try:
                        value = await self._client.read_gatt_char(char.uuid)
                        logger.info( "  [Characteristic] %s (%s), Value: %r", char, ",".join(char.properties), value)
                    except Exception as e:
                        logger.error( "  [Characteristic] %s (%s), Error: %s", char, ",".join(char.properties), e)

                else:
                    logger.info( "  [Characteristic] %s (%s)", char, ",".join(char.properties))

                for descriptor in char.descriptors:
                    try:
                        value = await self._client.read_gatt_descriptor(descriptor.handle)
                        logger.info("    [Descriptor] %s, Value: %r", descriptor, value)
                    except Exception as e:
                        logger.error("    [Descriptor] %s, Error: %s", descriptor, e)

    async def _read_attribute(self, handle: int) -> Optional[bytearray]:
        """
        Read the value of a GATT characteristic handle.

        Args:
            handle (int): the characteristic handle

        Returns:
            None on error, otherwise the content of the characteristic.
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return None

        logger.debug(f'Reading attribute with handle {handle}')
        result = await self._client.read_gatt_char(handle)
        logger.debug(f'read_attribute {handle} = {result}')
        return result

    async def _write_attribute(self, handle: int, data: Union[bytes, bytearray]) -> None:
        """
        Write a value to a GATT characteristic handle.

        Args:
            handle (int): the target characteristic handle
            data (bytes or bytearray): the data to write

        Returns:
            None
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return

        logger.debug(f'write_attribute: {data} --> {handle}')
        await self._client.write_gatt_char(handle, data)

    def _get_characteristic_handle_from_uuid(self, uuid: str) -> Optional[int]:
        """Given a characteristic UUID, return its handle.
    
        Args:
            uuid (str): a string containing the hex-encoded UUID

        Returns:
            None if an error occurs, otherwise an integer handle. 
        """
        ch = self._get_characteristic_from_uuid(bleak.uuids.normalize_uuid_str(uuid))
        return None if ch is None else ch.handle

    def _get_characteristic_from_uuid(self, uuid: str) -> Optional[BleakGATTCharacteristic]:
        """Given a characteristic UUID, return a :class:`BleakGATTCharacteristic` object
        containing information about that characteristic

        Args:
            uuid (str): a string containing the hex-encoded UUID

        Returns:
            None if an error occurs, otherwise a :class:`BleakGATTCharacteristic` object
        """
        if self._client is None or not self._client.is_connected:
            logger.error('No device configured or connected')
            return

        if uuid in self._uuid_chars:
            logger.debug(f'Returning cached info for char: {uuid}')
            return self._uuid_chars[uuid]

        char = self._client.services.get_characteristic(uuid)
        if char is None:
            logger.warning(f'Failed to find char for UUID: {uuid}')
            return None

        logger.debug(f'Found char for UUID: {uuid}')
        self._uuid_chars[uuid] = char
        return char

    async def get_polling_override(self) -> Optional[int]:
        """
        Get the current polling override value in milliseconds. 

        See :meth:`set_polling_override` for more information. 

        Returns:
            None on error, otherwise the current override period in milliseconds 
            (0 = disabled). 
        """
        polling_override = self._get_characteristic_handle_from_uuid(UUID_POLLING_OVERRIDE)
        if polling_override is None:
            logger.warn('Failed to find handle for polling override')
            return None
        override_ms = await self._read_attribute(polling_override)
        if override_ms is None:
            logger.error('Failed to get polling override value')
            return None

        return ord(override_ms)

    async def set_polling_override(self, override: int) -> bool:
        """
        Set the sensor polling timer override value in milliseconds. 

        Due to the time it takes to poll all the sensors on up to 5 IMUs, it's not 
        possible for the SK8 firmware to define a single fixed rate for reading 
        new samples without it being artificially low for most configurations. 
        
        Instead the firmware tries to define a sensible default value for each 
        combination of IMUs and sensors that can be enabled (any combination of 
        1-5 IMUs and 1-3 sensors on each IMU). In most cases this should work well,
        but for example if you have multiple SK8s connected through the same dongle
        and have multiple IMUs enabled on each, you may find packets start to be
        dropped quite frequently. 

        To mitigate this, you can adjust the period of the timer used by the firmware
        to poll for new sensor data (and send data packets to the host device). The
        value should be in integer milliseconds, and have a minimum value of 20. Values
        below 20 will be treated as a request to disable the override and return to the
        default polling period. 

        The method can be called before or after streaming is activated, and will take
        effect immediately. 

        NOTE1: the value is stored in RAM and will not persist across reboots, although
            it should persist for multiple connections.
        NOTE2: once set, the override applies to ALL sensor configurations, so for 
            example if you set it while using 5 IMUs on 2 SK8s, then switch to using 
            1 IMU on each SK8, you will probably want to disable it again as the 
            latter configuration should work fine with the default period.

        Args:
            override (int): polling timer override period in milliseconds. Values
                below 20 are treated as 0, and have the effect of disabling the
                override in favour of the default periods. 

        Returns:
            True on success, False on error.
        """
        polling_override = self._get_characteristic_handle_from_uuid(UUID_POLLING_OVERRIDE)
        if polling_override is None:
            logger.warn('Failed to find handle for device name')
            return False
        
        if await self._write_attribute(polling_override, struct.pack('B', override)):
            return True

        return False

