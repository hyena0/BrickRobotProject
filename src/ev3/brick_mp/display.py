# brick-sw-version: 5B.11
"""On-brick LCD status screen (best effort, never raises).

render_master and render_slave are pure: they format runtime state
into short text lines. show draws lines on the EV3 screen. Hardware
access stays inside show so the renderers run in PC tests.
"""

import version

_screen = None


def _letters(ports, keep):
    out = ""
    for p in sorted(ports):
        out = out + p[keep:]
    return out


def _status(rt):
    if rt.core.guards.latched:
        return "LATCH"
    return "OK"


def render_master(rt, hub_up, hub_addr):
    lines = ["LEFT " + version.BRICK_SW_VERSION + " " + _status(rt)]
    lines.append("M:" + _letters(rt.core.motors, -1)
                 + " S:" + _letters(rt.core.sensors, 2))
    if hub_up:
        hub = "UP"
    else:
        hub = "DOWN"
    if rt.slave_link_lost:
        slave = "DOWN"
    else:
        slave = "UP"
    lines.append("HUB:" + hub + " SLV:" + slave)
    lines.append(hub_addr)
    if rt.core.guards.latched:
        lines.append("LATCH:" + str(rt.core.guards.latch_reason)[:16])
    if rt.core.missing:
        lines.append("MISS:" + " ".join(rt.core.missing))
    return lines


def render_slave(rt, master_name):
    lines = ["RIGHT " + version.BRICK_SW_VERSION + " " + _status(rt)]
    lines.append("M:" + _letters(rt.core.motors, -1)
                 + " S:" + _letters(rt.core.sensors, 2))
    if rt.link_lost:
        master = "DOWN"
    else:
        master = "UP"
    lines.append("MST:" + master + " " + master_name)
    if rt.core.guards.latched:
        lines.append("LATCH:" + str(rt.core.guards.latch_reason)[:16])
    if rt.core.missing:
        lines.append("MISS:" + " ".join(rt.core.missing))
    return lines


def show(lines):
    global _screen
    try:
        if _screen is None:
            from pybricks.hubs import EV3Brick
            _screen = EV3Brick().screen
        _screen.clear()
        for line in lines:
            _screen.print(line)
    except Exception:
        pass
