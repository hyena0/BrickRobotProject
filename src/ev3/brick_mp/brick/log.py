# brick-sw-version: 5B.14
"""Append-only brick log file (best effort, never raises).

log() prints to the console and appends the same line to brick.log, so
startup facts (version, wiring, missing devices) survive on the brick
even when the console scrolls past. Delete brick.log to reset it.
"""

LOG_PATH = "brick.log"


def log_line(text):
    try:
        fh = open(LOG_PATH, "a")
        try:
            fh.write(text + "\n")
        finally:
            fh.close()
    except OSError:
        pass


def log(text):
    print(text)
    log_line(text)
