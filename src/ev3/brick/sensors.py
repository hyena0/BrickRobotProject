"""Sensor hardware abstraction for the EV3 bricks.

Ports only (e.g. ``in1``). The brick never learns what a sensor means.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class Sensor(ABC):
    """Binary sensor interface used by the safety layer."""

    port: str

    @abstractmethod
    def read(self) -> int:
        """Return 0 or 1."""


class FakeSensor(Sensor):
    """In-memory sensor for PC-side tests."""

    def __init__(self, port: str, initial: int = 0) -> None:
        self.port = port
        self.value = int(initial)

    def read(self) -> int:
        return int(self.value)

    def set(self, value: int) -> None:
        self.value = int(value)


def open_sensor(port: str, sensor_type: str = "touch") -> Sensor:
    """Open a real ev3dev sensor. Imported lazily so PCs can use fakes."""
    from ev3dev2.sensor import Sensor as _DevSensor  # type: ignore
    from ev3dev2.sensor.lego import TouchSensor  # type: ignore

    if sensor_type == "touch":
        dev = TouchSensor(port)
        return _Ev3DevTouchSensor(port, dev)
    dev = _DevSensor(port)
    return _Ev3DevTouchSensor(port, dev)


class _Ev3DevTouchSensor(Sensor):
    def __init__(self, port: str, dev: object) -> None:
        self.port = port
        self._dev = dev

    def read(self) -> int:
        return 1 if getattr(self._dev, "is_pressed", False) else 0
