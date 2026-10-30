from __future__ import annotations

"""Generated EV3 control package (Stage 5.1).

Produced by ``python -m src.codegen.generate`` from the
inferred kinematic model. Abstract kinematics + code only:
no part IDs, poses or appearance data, so this package is
public. Bricks see ports only; joint names live on the PC
side.

DO NOT hand-edit: regenerate after any model or calibration
change and re-run the bring-up checks.

Synchronized joint-space motion (Stage 5.1).

``move_joints`` converts a joint target to motor targets with
the model coupling matrix (joint limits checked first, human
zero offsets applied) and fires one ``run_targets`` per brick so
every motor starts and finishes together: each motor speed is
scaled by its own travel over the shared duration. Targets are
absolute encoder angles:

    target = home_encoder + (m + zero_offset)

where ``home_encoder`` is the reading captured by
``home.home_all`` (0.0 when the encoders were zeroed at the
model zero instead)."""


from .robot_config import (ACTUATORS, C, COUPLED_LIMITS,
                           COUPLED_LOOKUP, FIRST_RUN_SPEED_DEG_S,
                           JOINT_IDS, MOTOR_IDS, SPEED_CAP_DEG_S,
                           SOFT_MARGIN_DEG, ZERO_OFFSETS)

try:
    from src.kin.motors import check_limits, joints_to_motors
except ImportError:  # generated package used outside the repo checkout
    check_limits = None
    joints_to_motors = None

SOFT_LIMITS = None  # filled by _soft_limits() from JOINTS on first use


def _require_kin():
    if check_limits is None or joints_to_motors is None:
        raise ImportError(
            "generated motion needs src.kin.motors on sys.path "
            "(run from the repo checkout, e.g. PYTHONPATH=<repo>)")
    return check_limits, joints_to_motors


def _soft_limits():
    global SOFT_LIMITS
    if SOFT_LIMITS is None:
        from .robot_config import JOINTS
        SOFT_LIMITS = [j["soft_limits_deg"] for j in JOINTS]
    return SOFT_LIMITS


def _lookup_ok(q_target_deg):
    pos = {jid: v for jid, v in zip(JOINT_IDS, q_target_deg)}
    for key, tab in COUPLED_LOOKUP.items():
        if key.startswith("_") or not isinstance(tab, dict):
            continue
        try:
            ja, jb = key.split("_")
            cells = tab.get("cells", [])
            radius = float(tab.get("radius_deg", 2.5))
        except (ValueError, AttributeError, TypeError):
            continue
        if ja in pos and jb in pos and cells and any(
                abs(pos[ja] - ca) <= radius + 1e-9
                and abs(pos[jb] - cb) <= radius + 1e-9
                for ca, cb in cells):
            return False
    return True


def _check_coupled(q_target_deg):
    pos = {jid: v for jid, v in zip(JOINT_IDS, q_target_deg)}
    bad = []
    for d in COUPLED_LIMITS:
        ja, jb = d["joints"]
        val = d["a"][0] * pos[ja] + d["a"][1] * pos[jb] + d["b"]
        if d["op"] == ">=" and val < -1e-9:
            bad.append(f"coupled {ja}+{jb}: {val:.2f} < 0")
        if d["op"] == "<=" and val > 1e-9:
            bad.append(f"coupled {ja}+{jb}: {val:.2f} > 0")
    if not _lookup_ok(q_target_deg):
        bad.append("coupled lookup: target lands on a colliding grid cell")
    return bad


def check_path(q_from_deg, q_to_deg, step_deg=5.0):
    """Interpolated waypoints every <= step_deg, all inside limits.

    Returns ``(ok, info)``; ``info`` names the first violating
    waypoint. Used by bring-up before commanding hardware.
    """
    import math as _math
    dist = max(abs(b - a) for a, b in zip(q_from_deg, q_to_deg))
    n = max(1, int(_math.ceil(dist / step_deg)))
    lims = _soft_limits()
    for k in range(n + 1):
        q = [a + (b - a) * k / n for a, b in zip(q_from_deg, q_to_deg)]
        bad = check_limits(q, lims) if check_limits else []
        bad += _check_coupled(q)
        if bad:
            return False, {"waypoint": k, "n": n, "q_deg": q,
                           "violations": bad}
    return True, {"n": n}


def plan_move(q_target_deg, duration_s=4.0,
              speed_cap=FIRST_RUN_SPEED_DEG_S, home=None):
    """Pure planning: joint target -> per-motor ``(target, speed)``.

    Returns ``(cmds, plan)`` where ``cmds`` maps motor uid to the
    absolute encoder target and ``plan`` carries the shared duration,
    per-motor speeds and the coupling residual. Raises ``ValueError``
    on soft-limit or coupled-constraint violation. No driver traffic
    happens here, so this is unit-testable without hardware.
    """
    check_limits, joints_to_motors = _require_kin()
    lims = _soft_limits()
    bad = check_limits(q_target_deg, lims)
    bad += _check_coupled(q_target_deg)
    if bad:
        raise ValueError("soft-limit violation "
                         f"(margin {SOFT_MARGIN_DEG} deg): "
                         + "; ".join(bad))
    out = joints_to_motors(list(q_target_deg), C, MOTOR_IDS, lims,
                           dict(ZERO_OFFSETS))
    home = home or {}
    cmds, travels = {}, {}
    for uid in MOTOR_IDS:
        key = None
        for a in ACTUATORS:
            if a["id"] == uid:
                key = f"{a['brick']}:{a['port']}"
        base = float(home.get(key, 0.0))
        target = base + float(out["motors_deg"][uid])
        cmds[uid] = target
        travels[uid] = abs(target - base)
    duration = max(float(duration_s), 0.5)
    speeds = {}
    for uid in MOTOR_IDS:
        need = travels[uid] / duration
        speeds[uid] = min(need, float(speed_cap))
    slowest = max(speeds.values()) if speeds else 0.0
    if slowest > 0.0 and slowest >= float(speed_cap) - 1e-9:
        # Longest travel hits the cap; stretch the duration so every
        # motor still finishes together instead of clipping one motor.
        duration = max(travels.values()) / float(speed_cap)
        speeds = {uid: travels[uid] / duration for uid in MOTOR_IDS}
    plan = {"duration_s": duration, "speeds_deg_s": speeds,
            "travels_deg": travels,
            "coupling_residual_deg": out["coupling_residual_deg"]}
    return cmds, plan


def move_joints(driver, q_target_deg, duration_s=4.0,
                speed_cap=FIRST_RUN_SPEED_DEG_S, home=None,
                timeout_s=30.0):
    """Move to a joint-space target with synchronized finish.

    Groups motor moves per brick into one ``run_targets`` each so all
    motors on a brick start together; bricks are commanded back to
    back from the same PC tick.
    """
    cmds, plan = plan_move(q_target_deg, duration_s, speed_cap, home)
    by_brick = {}
    for a in ACTUATORS:
        move = {"port": a["port"], "target": cmds[a["id"]],
                "speed": min(plan["speeds_deg_s"][a["id"]],
                             SPEED_CAP_DEG_S)}
        by_brick.setdefault(a["brick"], []).append(move)
    for brick, moves in by_brick.items():
        driver.request(brick, "run_targets", {"moves": moves},
                       timeout_s=timeout_s)
    residual = plan["coupling_residual_deg"]
    if abs(residual) > 0.5:
        print(f"warning: coupling residual {residual:.3f} deg "
              f"(coupled wrist commands are infeasible by physics; "
              f"see pred_C lock law)")
    print(f"moved to {list(q_target_deg)} in "
          f"{plan['duration_s']:.2f}s at cap {speed_cap} deg/s")
    return cmds, plan
