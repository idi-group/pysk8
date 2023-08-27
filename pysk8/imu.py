import time
from typing import Optional, Any, Tuple, Sequence

class IMUData:
    """
    Instances of this class provide access to sensor data from individual IMUs.

    Attributes:
        acc (list): latest accelerometer readings, [x, y, z]
        mag (list): latest magnetometer readings, [x, y, z]
        acc (list): latest gyroscope readings, [x, y, z]
        seq (int): sequence number from most recent packet (0-255 range)
        timestamp (float): value of `time.time()` when packet received
    """

    PACKET_PERIOD = 3

    def __init__(self, index: int, calibration_data: Optional[dict[str, Any]] = None):
        self._calibration_data = calibration_data
        self.index = index
        self.reset()

    def reset(self):
        self.acc = ( 0., 0., 0. )
        self.mag = ( 0, 0, 0 )
        self.gyro = ( 0, 0, 0 )
        self.acc_scale = (1., 1., 1.)
        self.acc_offsets = ( 0, 0, 0 )
        self.seq = 0
        self._use_calibration = False
        self._has_acc_calib, self.has_mag_calib, self.has_gyro_calib = False, False, False
        self.load_calibration(self._calibration_data)
        self._packet_metadata = []
        self._packet_start = time.time()
        self._packets_lost = 0

    def get_sample_rate(self) -> float:
        if time.time() - self._packet_start < IMUData.PACKET_PERIOD:
            return -1
        return len(self._packet_metadata) / IMUData.PACKET_PERIOD

    def get_packets_lost(self) -> int:
        sample_rate = self.get_sample_rate()
        if sample_rate == -1:
            return -1

        return sum([x[1] for x in self._packet_metadata])

    def get_total_packets_lost(self) -> int:
        return self._packets_lost 

    def set_calibration(self, state: bool) -> None:
        self._use_calibration = state

    def get_calibration(self) -> bool:
        return self._use_calibration

    def load_calibration(self, calibration_data: Optional[dict[str, Any]]) -> bool:
        axes = ['x', 'y', 'z']
        if calibration_data is None:
            return False

        if 'accx_offset' in calibration_data:
            self.acc_scale = tuple(map(float, [calibration_data[f'acc{x}_scale'] for x in axes]))
            self.acc_offsets = tuple(map(float, [calibration_data[f'acc{x}_offset'] for x in axes]))
            self.has_acc_calib = True
        else:
            self.acc_scale = ( 1., 1., 1.)
            self.acc_offsets = ( 0, 0, 0 )
            
        if 'gyrox_offset' in calibration_data:
            self.gyro_offsets = tuple(map(float, [calibration_data[f'gyro{x}_offset'] for x in axes]))
            self.has_gyro_calib = True
        else:
            self.gyro_offsets = ( 0, 0, 0 )

        if 'magx_offset' in calibration_data:
            self.mag_scale = tuple(map(float, [calibration_data[f'mag{x}_scale'] for x in axes]))
            self.mag_offsets = tuple(map(float, [calibration_data[f'mag{x}_offset'] for x in axes]))
            self.has_mag_calib = True
        else:
            self.mag_offsets = ( 1., 1., 1. )
            self.mag_scale = ( 0, 0, 0 )

        self._use_calibration = True
        return True

    def _get_cal(self, raw: Sequence[float], offset: Sequence[float], scale: Sequence[float], ignore_cal: bool = False) -> Sequence[float]:
        if ignore_cal:
            return raw

        return tuple((raw[x] * scale[x]) - offset[x] for x in range(len(raw)))

    def update(self, acc: Sequence[float], gyro: Sequence[float], mag: Sequence[float], seq: int, timestamp: float) -> None:
        if not self._use_calibration:
            self.acc = acc
            self.gyro = gyro
            self.mag = mag
        else:
            self.acc = tuple(map(int, self._get_cal(acc, self.acc_offsets, self.acc_scale)))
            self.gyro = self._get_cal(gyro, self.gyro_offsets, (1., 1., 1.))
            self.mag = tuple(map(int, self._get_cal(mag, self.mag_offsets, self.mag_scale)))
        
        dropped = 0
        if self.seq != -1:
            expected = (self.seq + 1) % 256
            if expected != seq:
                dropped = (expected - seq) % 256
                self._packets_lost += dropped
        self.seq = seq
        self.timestamp = timestamp
        self._packet_metadata.insert(0, (timestamp, dropped))
        now = time.time()
        while now - self._packet_metadata[-1][0] > IMUData.PACKET_PERIOD:
            self._packet_metadata.pop()

    def __repr__(self):
        return f'[{self.index}] acc={self.acc}, mag={self.mag}, gyro={self.gyro}, seq={self.seq}'

