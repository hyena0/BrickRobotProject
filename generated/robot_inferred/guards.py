from __future__ import annotations

"""Generated EV3 control package (Stage 5.1).

Produced by ``python -m src.codegen.generate`` from the
inferred kinematic model. Abstract kinematics + code only:
no part IDs, poses or appearance data, so this package is
public. Bricks see ports only; joint names live on the PC
side.

DO NOT hand-edit: regenerate after any model or calibration
change and re-run the bring-up checks.

Safety guards (Stage 5.1).

Every wired sensor stops all motors on its brick when triggered
outside a homing move. Guards are local to each brick; the master
forwards the stop to the other brick. Call ``apply_guards``
before any motion and check the returned acks (exit criteria)."""


from .robot_config import GUARD_RULES


def apply_guards(driver) -> dict:
    """Send the guard set to every brick. Returns ``{brick: ack}``."""
    acks = {}
    for brick, rules in GUARD_RULES.items():
        ack = driver.set_guards(brick, rules)
        print(f"[{brick}] set_guards ack: {ack} (rules={len(rules)})")
        acks[brick] = ack
    return acks


def clear_all(driver) -> None:
    """Clear the guard latch on every brick (needed after a guard stop)."""
    for brick in GUARD_RULES:
        driver.clear_latch(brick)
        print(f"[{brick}] latch cleared; re-homing is required next.")
