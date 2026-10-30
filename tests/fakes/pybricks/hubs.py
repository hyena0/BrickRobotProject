"""Fake ``pybricks.hubs``: EV3 brick with a recording screen."""

from __future__ import annotations


class FakeScreen:
    def __init__(self):
        self.lines = []
        self.clears = 0

    def clear(self):
        self.clears += 1
        self.lines = []

    def print(self, *args):
        self.lines.append(" ".join(str(a) for a in args))


class EV3Brick:
    def __init__(self):
        self.screen = FakeScreen()
