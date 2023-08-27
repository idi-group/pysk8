"""
pysk8 constants.

Attributes:
    DATA_STRUCT: structure for unpacking SK8 IMU data packets
    MAX_IMUS: maximum number of IMUs attached to any SK8
    DEF_TIMEOUT: default timeout (seconds) for various operations
"""

import struct

IMU_DATA_STRUCT                          = struct.Struct('<hhhhhhhhhBB')
ANA_DATA_STRUCT                          = struct.Struct('<HHHB')
MAX_IMUS                                 = 5
DEF_TIMEOUT                              = 3

LED_MIN                                  = 0
LED_MAX                                  = 255.0
INT_LED_MAX                              = 3000.0

EXT_HW_NONE                              = 0x00
EXT_HW_IMUS                              = 0x01
EXT_HW_EXTANA                            = 0x02

SENSOR_ACC                               = 0x01
SENSOR_GYRO                              = 0x02
SENSOR_MAG                               = 0x04
SENSOR_ALL                               = SENSOR_ACC | SENSOR_GYRO | SENSOR_MAG

MAX_DEVICE_NAME_LEN                      = 20

UUID_BATTERY_LEVEL                       = '2a19'
UUID_DEVICE_NAME                         = '2a00'
UUID_FIRMWARE_REVISION                   = '2a26'
UUID_SK8_SERVICE                         = 'b9e32260-1074-11e6-a7d5-0002a5d5c51b'
UUID_IMU_CCC                             = 'b9e32261-1074-11e6-a7d5-0002a5d5c51b'
UUID_EXTANA_CCC                          = 'b9e32262-1074-11e6-a7d5-0002a5d5c51b'
UUID_IMU_SELECTION                       = 'b9e32263-1074-11e6-a7d5-0002a5d5c51b'
UUID_SENSOR_SELECTION                    = 'b9e32264-1074-11e6-a7d5-0002a5d5c51b'
UUID_SOFT_RESET                          = 'b9e32265-1074-11e6-a7d5-0002a5d5c51b'
UUID_EXTANA_LED                          = 'b9e32266-1074-11e6-a7d5-0002a5d5c51b'
UUID_POLLING_OVERRIDE                    = 'b9e3226a-1074-11e6-a7d5-0002a5d5c51b'

# TODO messed up UUIDs in FW up to and including v0.0.2.0, so have to check
# for existence of both old and correct versions of these two
UUID_EXTANA_IMU_STREAMING                = 'b9e32286-1074-11e6-a7d5-0002a5d5c51b'
UUID_EXTANA_IMU_STREAMING_TMP            = 'b9e32268-1074-11e6-a7d5-0002a5d5c51b'
UUID_HARDWARE_STATE                      = 'b9e32269-1074-11e6-a7d5-0002a5d5c51b'
UUID_HARDWARE_STATE_TMP                  = 'b9e32296-1074-11e6-a7d5-0002a5d5c51b'

UUIDS_TO_DISCOVER = \
    [
        UUID_IMU_CCC,
        UUID_EXTANA_CCC,
        UUID_IMU_SELECTION,
        UUID_SENSOR_SELECTION,
        UUID_SOFT_RESET,
        UUID_EXTANA_LED,
        UUID_EXTANA_IMU_STREAMING,
        UUID_HARDWARE_STATE,
    ]

RAW_UUID_GATT_PRIMARY_SERVICE            = b'\x00\x28'
RAW_UUID_GATT_CHAR_DECL                  = b'\x03\x28'
RAW_UUID_GATT_CCC                        = b'\x02\x29'
UUID_GATT_CCC                            = '2902'
RAW_CCC_NOTIFY_ON                        = b'\x01\x00'
RAW_CCC_NOTIFY_OFF                       = b'\x00\x00'

# characteristic properties
PROP_BROADCAST                           = 0x01
PROP_READ                                = 0x02
PROP_WRITE_NO_RESP                       = 0x04
PROP_WRITE                               = 0x08
PROP_NOTIFY                              = 0x10
PROP_INDICATE                            = 0x20
PROP_AUTH_WRITE                          = 0x40
PROP_EXT_PROPS                           = 0x80

# attribute value types
ATTR_VALUE_TYPE_READ                     = 0x00
ATTR_VALUE_TYPE_NOTIFY                   = 0x01
ATTR_VALUE_TYPE_INDICATE                 = 0x02
ATTR_VALUE_TYPE_READ_BY_TYPE             = 0x03
ATTR_VALUE_TYPE_READ_BLOB                = 0x04
ATTR_VALUE_TYPE_INDICATE_RSP_REQ         = 0x05
