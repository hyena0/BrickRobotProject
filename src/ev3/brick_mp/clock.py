# brick-sw-version: 5B.14
"""Small time helpers that run on MicroPython and on CPython.

On the brick this uses ticks_ms and ticks_diff (wrap safe). On the PC
it falls back to monotonic milliseconds. All brick timing goes through
now_ms and diff_ms so wrap around never breaks a watchdog.
"""

try:
    from utime import ticks_ms as _ticks_ms
    from utime import ticks_diff as _ticks_diff
    from utime import sleep as _sleep_s
    _USE_TICKS = True
except ImportError:
    import time as _time
    _USE_TICKS = False


def now_ms():
    if _USE_TICKS:
        return _ticks_ms()
    return int(_time.monotonic() * 1000)


def diff_ms(a, b):
    """Return a minus b in milliseconds (a later than b is positive)."""
    if _USE_TICKS:
        return _ticks_diff(a, b)
    return a - b


def sleep_ms(ms):
    if _USE_TICKS:
        _sleep_s(ms / 1000.0)
    else:
        _time.sleep(ms / 1000.0)


def sleep_s(s):
    if _USE_TICKS:
        _sleep_s(s)
    else:
        _time.sleep(s)
