"""Fake ``pybricks.messaging`` with latest-value mailbox semantics.

Server and client boxes share one in-process registry keyed by box
name, mirroring a connected pair. ``BluetoothMailboxClient.disconnect()``
simulates link loss: further box reads/writes raise ``OSError``.
"""

from __future__ import annotations

_boxes: dict = {}


def reset():
    _boxes.clear()


class BluetoothMailboxServer:
    def __init__(self):
        self.kind = "server"
        self.waited = 0

    def wait_for_connection(self, count=1):
        self.waited += 1

    def close(self):
        pass


class BluetoothMailboxClient:
    def __init__(self):
        self.kind = "client"
        self.connected = False
        self.addr = None

    def connect(self, addr):
        self.addr = addr
        self.connected = True

    def disconnect(self):
        self.connected = False


class TextMailbox:
    def __init__(self, name, conn):
        self.name = name
        self.conn = conn

    def _check(self):
        if isinstance(self.conn, BluetoothMailboxClient) \
                and not self.conn.connected:
            raise OSError("mailbox link down")

    def send(self, value):
        self._check()
        _boxes[self.name] = value

    def read(self):
        self._check()
        return _boxes.get(self.name)
