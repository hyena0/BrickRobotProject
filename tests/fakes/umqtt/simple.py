"""Fake ``umqtt.simple`` backed by an in-memory broker.

Supports the call shapes the left brick uses: ``MQTTClient`` positional
construction, ``set_callback``, ``set_last_will``, ``connect``,
``subscribe(topic, qos)``, ``publish(topic, msg, retain, qos)`` and
``check_msg``. Topics are bytes; trailing ``/#`` subscriptions match
subtopics. ``drop()`` simulates an unclean disconnect (fires the will).
"""

from __future__ import annotations

_subs: dict = {}
_retained: dict = {}
_down = False


def reset():
    _subs.clear()
    _retained.clear()
    global _down
    _down = False


def set_broker_down(value=True):
    global _down
    _down = bool(value)


def _matches(sub, topic):
    if sub.endswith(b"/#"):
        return topic == sub[:-2] or topic.startswith(sub[:-2] + b"/")
    return sub == topic


class MQTTClient:
    def __init__(self, client_id, server, port=0, user=None, password=None,
                 keepalive=0, ssl=False, ssl_params=None):
        self.client_id = client_id
        self.server = server
        self.port = port
        self.user = user
        self.password = password
        self.keepalive = keepalive
        self.connected = False
        self._callback = None
        self._will = None
        self.inbox = []
        self.published = []
        self.subscriptions = []

    def set_callback(self, callback):
        self._callback = callback

    def set_last_will(self, topic, msg, retain=False, qos=0):
        self._will = (topic, msg, retain, qos)

    def connect(self):
        if _down:
            raise OSError("broker unreachable")
        self.connected = True

    def disconnect(self):
        self.connected = False

    def drop(self):
        """Unclean drop: fire the last will, then go offline."""
        if self._will is not None and self.connected:
            topic, msg, retain, qos = self._will
            self._deliver(topic, msg, retain)
        self.connected = False

    def subscribe(self, topic, qos=0):
        if not self.connected:
            raise OSError("not connected")
        _subs.setdefault(topic, [])
        if self not in _subs[topic]:
            _subs[topic].append(self)
        self.subscriptions.append((topic, qos))
        for rtopic, (rmsg, _qos) in _retained.items():
            if _matches(topic, rtopic):
                self.inbox.append((rtopic, rmsg))

    def publish(self, topic, msg, retain=False, qos=0):
        if not self.connected:
            raise OSError("not connected")
        self.published.append((topic, msg, retain, qos))
        self._deliver(topic, msg, retain)

    @staticmethod
    def _deliver(topic, msg, retain):
        if retain:
            _retained[topic] = (msg, 0)
        for sub, clients in _subs.items():
            if _matches(sub, topic):
                for client in list(clients):
                    if client.connected:
                        client.inbox.append((topic, msg))

    def check_msg(self):
        if not self.connected:
            raise OSError("not connected")
        if not self.inbox:
            return None
        topic, msg = self.inbox.pop(0)
        if self._callback is not None:
            self._callback(topic, msg)
        return (topic, msg)
