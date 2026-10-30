"""PC-side host driver for the two EV3 bricks.

Thin backwards-compatible wrapper over the Stage 5B PC Transport
adapter (:class:`src.adapters.pc.mqtt_transport.MqttTransport`). New code
should import the adapter directly; this module keeps the Stage 0
``Driver`` / ``DriverError`` / ``install_keyboard_estop`` names working.

Reads ``config/hardware.local.yaml`` (git-ignored): broker address and
brick names (``left`` = master, ``right`` = slave). The broker takes
anonymous clients; there are no accounts or passwords. Bricks know
ports only; this driver also stays joint-agnostic.
"""

from __future__ import annotations

from src.adapters.pc.mqtt_transport import (
    DEFAULT_TIMEOUT_S,
    HEARTBEAT_PERIOD_S,
    MqttTransport,
    TransportError,
    install_keyboard_estop,
)

DriverError = TransportError


class Driver(MqttTransport):
    """Stage 0 name for :class:`MqttTransport` (identical behaviour)."""

    def __init__(
        self,
        broker: str = "localhost",
        broker_port: int = 1883,
        client_factory=None,
        bricks: tuple[str, ...] = ("left", "right"),
        **kwargs,
    ) -> None:
        super().__init__(broker=broker, broker_port=broker_port,
                         client_factory=client_factory, bricks=bricks,
                         **kwargs)


__all__ = [
    "DEFAULT_TIMEOUT_S",
    "Driver",
    "DriverError",
    "HEARTBEAT_PERIOD_S",
    "install_keyboard_estop",
]
