class ExtAnaData(object):
    """
    An instance of this class provides access to data from an SK8-ExtAna board.

    Attributes:
        ch1 (int): analogue channel #1
        ch2 (int): analogue channel #2
        temp (int): temperature sensor output (units of 0.01 deg C)
        seq (int): sequence number of most recent packet
        timestamp (float): value of `time.time()` when packet received
    """

    def __init__(self) -> None:
        self.ch1 = 0
        self.ch2 = 0
        self.temp = 0
        self.seq = 0
        self.timestamp = 0

    def update(self, ch1: int, ch2: int, temp: int, seq: int, timestamp: float) -> None:
        self.ch1 = ch1
        self.ch2 = ch2
        self.temp = temp / 100.0
        self.seq = seq
        self.timestamp = timestamp

    def __repr__(self):
        return 'ch1={}, ch2={}, temp={:.1f}, seq={}'.format(self.ch1, self.ch2, self.temp, self.seq)

