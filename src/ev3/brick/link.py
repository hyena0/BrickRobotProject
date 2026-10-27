"""Length-prefixed JSON framing for the master/slave Bluetooth link.

The master relays ``robot/*/right`` MQTT traffic over Bluetooth without
interpreting it: bytes are framed as ``<4-byte big-endian length><JSON>``.
Pure stdlib so it is unit-testable on a PC.
"""

from __future__ import annotations

import json
import struct

HEADER = struct.Struct(">I")
MAX_FRAME = 1 << 20  # 1 MiB


def encode(frame: dict) -> bytes:
    body = json.dumps(frame).encode("utf-8")
    if len(body) > MAX_FRAME:
        raise ValueError("frame too large")
    return HEADER.pack(len(body)) + body


class Framer:
    """Incremental decoder for a byte stream."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[dict]:
        self._buf.extend(data)
        out: list[dict] = []
        while True:
            if len(self._buf) < HEADER.size:
                return out
            (size,) = HEADER.unpack(self._buf[:HEADER.size])
            if size > MAX_FRAME:
                raise ValueError("frame too large")
            if len(self._buf) < HEADER.size + size:
                return out
            body = bytes(self._buf[HEADER.size:HEADER.size + size])
            del self._buf[:HEADER.size + size]
            out.append(json.loads(body.decode("utf-8")))
