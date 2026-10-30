# brick-sw-version: 5B.14
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

    Used for homing-mark detection. The default threshold is an
    uncalibrated placeholder until the PC side homing measures the
    mark and table readings on the real robot.
    """

    def __init__(self, port, dev, threshold=50):
        Sensor.__init__(self, port)
        self._dev = dev
        self._threshold = threshold

    def read(self):
        if self._dev.reflection() >= self._threshold:
            return 1
        return 0


def open_sensor(port, sensor_type="touch"):
    """Open a real pybricks sensor. Imported lazily for PC side tests."""
    if sensor_type != "touch" and sensor_type != "color":
        raise ValueError("unsupported sensor type: " + str(sensor_type))
    if port not in IN_NAMES:
        raise ValueError("unknown sensor port: " + str(port))
    from pybricks.parameters import Port
    if sensor_type == "color":
        from pybricks.ev3devices import ColorSensor
        dev = ColorSensor(getattr(Port, IN_NAMES[port]))
        return PybricksColorSensor(port, dev)
    from pybricks.ev3devices import TouchSensor
    dev = TouchSensor(getattr(Port, IN_NAMES[port]))
    return PybricksSensor(port, dev)
