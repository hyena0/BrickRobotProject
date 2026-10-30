"""Fake ``pybricks.parameters``: port and stop sentinels."""

from __future__ import annotations


class Port:
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    S1 = "S1"
    S2 = "S2"
    S3 = "S3"
    S4 = "S4"


class Stop:
    HOLD = "hold"
    COAST = "coast"
    BRAKE = "brake"
