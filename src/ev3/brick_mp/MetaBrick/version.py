# brick-sw-version: 5B.11
"""Brick software version (single source of truth).

Printed at startup by both mains. Bump on every brick-side change and
update the matching marker comment on every copied .py file (a PC test
enforces the match, so a stale copy on the brick is always visible).
"""

BRICK_SW_VERSION = "5B.11"
