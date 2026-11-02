# brick-sw-version: 5B.17
"""Sensor hardware abstraction for the Pybricks bricks.

Ports only (in1 and friends). The brick never learns what a sensor
means. FakeSensor is used by PC side tests.
"""

IN_NAMES = {"in1": "S1", "in2": "S2", "in3": "S3", "in4": "S4"}


class Sensor:
    def __init__(self, port):
        self.port = port

    def read(self):
        raise NotImplementedError("Sensor.read")


class FakeSensor(Sensor):
    """In-memory sensor for PC side tests."""

    def __init__(self, port, initial=0):
        Sensor.__init__(self, port)
        self.value = int(initial)

    def read(self):
        return int(self.value)

    def set(self, value):
        self.value = int(value)


class PybricksSensor(Sensor):
    def __init__(self, port, dev):
        Sensor.__init__(self, port)
        self._dev = dev

    def read(self):
        if self._dev.pressed():
            return 1
        return 0


class PybricksColorSensor(Sensor):
    """Color sensor read as binary via a reflection threshold.

    Used for homing-mark detection. Without a measured threshold
    the sensor is uncalibrated and the core refuses to use it in
    safety rules. Pass a threshold from wiring or set_threshold.
    """

    def __init__(self, port, dev, threshold=None):
        Sensor.__init__(self, port)
        self._dev = dev
        if threshold is None:
            self._threshold = 50.0
            self.calibrated = False
        else:
            self._threshold = float(threshold)
            self.calibrated = True

    def set_threshold(self, value):
        self._threshold = float(value)
        self.calibrated = True

    def read(self):
        if self._dev.reflection() >= self._threshold:
            return 1
        return 0


def open_sensor(port, sensor_type="touch", threshold=None):
    """Open a real pybricks sensor. Imported lazily for PC side tests."""
    if sensor_type != "touch" and sensor_type != "color":
        raise ValueError("unsupported sensor type: " + str(sensor_type))
    if port not in IN_NAMES:
        raise ValueError("unknown sensor port: " + str(port))
    from pybricks.parameters import Port
    if sensor_type == "color":
        from pybricks.ev3devices import ColorSensor
        dev = ColorSensor(getattr(Port, IN_NAMES[port]))
        return PybricksColorSensor(port, dev, threshold)
    from pybricks.ev3devices import TouchSensor
    dev = TouchSensor(getattr(Port, IN_NAMES[port]))
    return PybricksSensor(port, dev)
