# brick-sw-version: 5B.11
"""Brick-local JSON config loader.

Reads config/brick.local.json with ujson on the brick (json fallback on
the PC). The file holds only this brick name and wiring: the hub
address lives hardcoded in left_main and the master Bluetooth name in
right_main. The file is never committed.
"""

try:
    import ujson as json
except ImportError:
    import json

DEFAULT_PATH = "config/brick.local.json"
FLAT_PATH = "brick.local.json"


def _list_dir():
    try:
        import os
        names = list(os.listdir("."))
    except Exception:
        return []
    names.sort()
    return names


def load_brick_config(path=DEFAULT_PATH):
    """Load and validate the brick-local JSON config file.

    Tries path first, then brick.local.json next to the program, so a
    flat VS Code copy just works. When nothing is found the error lists
    every tried path plus the files actually present, so a missing or
    misnamed copy is visible right in the traceback. There is no wiring
    fallback: running motors from guessed ports would be unsafe.
    """
    tried = [path]
    if path != FLAT_PATH:
        tried.append(FLAT_PATH)
    for candidate in tried:
        try:
            fh = open(candidate, "r")
        except OSError:
            continue
        try:
            cfg = json.load(fh)
        finally:
            fh.close()
        return _validate(cfg, candidate)
    msg = "brick config not found, tried " + " and ".join(tried)
    here = _list_dir()
    if here:
        msg = msg + "; files here: " + ", ".join(here)
    raise OSError(msg)


def _validate(cfg, path):
    if not cfg:
        raise ValueError("brick config is empty: " + str(path))
    brick = str(cfg.get("brick", "left"))
    if brick != "left" and brick != "right":
        raise ValueError("brick must be left or right, got: " + brick)
    return {
        "brick": brick,
        "motors": list(cfg.get("motors", [])),
        "sensors": list(cfg.get("sensors", [])),
    }
