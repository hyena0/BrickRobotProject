# brick-sw-version: 5B.14
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
LEFT_PATH = "brick_left.json"
RIGHT_PATH = "brick_right.json"


def _list_dir():
    try:
        import os
        names = list(os.listdir("."))
    except Exception:
        return []
    names.sort()
    return names


def load_brick_config(path=DEFAULT_PATH, expect=None):
    """Load and validate the brick-local JSON config file.

    path is one file or a list of files tried in order; the legacy
    flat brick.local.json is always tried last, so a flat VS Code
    copy just works. Each main passes its own side file first
    (brick_left.json on the left, brick_right.json on the right),
    so nobody edits JSON per brick. When nothing is found the error
    lists every tried path plus the files actually present, so a
    missing or misnamed copy is visible right in the traceback.
    expect optionally pins the side ("left"/"right"): a file naming
    the other side fails loudly instead of joining the wrong link.
    """
    if isinstance(path, str):
        tried = [path]
    else:
        tried = list(path)
    if FLAT_PATH not in tried:
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
        return _validate(cfg, candidate, expect)
    msg = "brick config not found, tried " + " and ".join(tried)
    here = _list_dir()
    if here:
        msg = msg + "; files here: " + ", ".join(here)
    raise OSError(msg)


def _validate(cfg, path, expect=None):
    if not cfg:
        raise ValueError("brick config is empty: " + str(path))
    brick = str(cfg.get("brick", "left"))
    if brick != "left" and brick != "right":
        raise ValueError("brick must be left or right, got: " + brick)
    if expect is not None and brick != expect:
        raise ValueError("brick side mismatch: expected " + expect
                         + ", got " + brick + ": " + str(path))
    return {
        "brick": brick,
        "motors": list(cfg.get("motors", [])),
        "sensors": list(cfg.get("sensors", [])),
    }
