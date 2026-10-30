"""Generate the EV3 control package for one inferred model (Stage 5.1).

Reads ``models/robot_inferred.json`` (abstract kinematics), the hardware
layer ``config/ports.local.yaml`` (brick/port wiring, sensor list) and
optionally the agent sensor map plus human homing calibration, then
writes ``generated/<robot>/{robot_config,motion,home,guards}.py``.

The drive motor of each joint is derived from the model itself: the
column with the largest ``|C[row]|`` entry (column-normalized, self=+1
per PRED_SPEC). No ground truth is read. Sensor/homing semantics come
from the agent map (``src/agent/sensors.py`` output) and the human
``homing.local.yaml`` calibration file.

Usage::

    python -m src.codegen.generate --model models/robot_inferred.json \\
        --ports config/ports.local.yaml --sensors runs/stage5/sensors.json \\
        --out generated/robot_inferred
"""

from __future__ import annotations

import argparse
import json
import os
import pprint

import yaml

from src.kin.model import chain_order, coupling, load_model

SOFT_MARGIN_DEG = 5.0
FIRST_RUN_SPEED_DEG_S = 30.0
SPEED_CAP_DEG_S = 100.0
HOME_SEARCH_DEG = 120.0
HOME_SPEED_DEG_S = 20.0
# Stage 5A.3 conservative finish: the lookup-table code stays in the
# template but generated packages carry an empty table, so motion
# checks use only limits_deg and coupled_limits.
EMIT_COUPLED_LOOKUP = False


def motor_port_full(port) -> str:
    """Normalize a wiring-layer motor port (``B``) to brick form."""
    s = str(port)
    return s if s.startswith("out") else f"out{s}"


def sensor_port_full(port) -> str:
    """Normalize a wiring-layer sensor port (``1``) to brick form."""
    s = str(port)
    return s if s.startswith("in") else f"in{s}"


def drive_columns(c: list[list[float]]) -> list[int]:
    """Drive-motor column per joint row (max ``|C|`` entry)."""
    cols = []
    for i, row in enumerate(c):
        best = max(range(len(row)), key=lambda j: abs(row[j]))
        if abs(row[best]) == 0.0:
            raise ValueError(f"joint row {i} has no drive motor")
        cols.append(best)
    return cols


def load_homing(path: str | None) -> dict:
    """Human calibration: zeros, homing directions, colour thresholds."""
    if not path:
        return {}
    try:
        with open(path, encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except OSError:
        return {}


def load_sensor_map(path: str | None) -> list[dict]:
    if not path:
        return []
    try:
        with open(path, encoding="utf-8") as fh:
            doc = json.load(fh)
    except OSError:
        return []
    return doc if isinstance(doc, list) else doc.get("sensors", [])


def homing_sensor_for(joint_id: str, sensors: list[dict]) -> dict | None:
    """Prefer a touch sensor on the joint; colour sensors need thresholds."""
    cands = [s for s in sensors if s.get("joint") == joint_id]
    touch = [s for s in cands
             if str(s.get("type", "touch")).lower() == "touch"
             or s.get("uid") != "p000489"]
    # The colour brick sensor is usable only with a human threshold, so a
    # plain touch sensor on the same joint always wins when present.
    for s in cands:
        if s.get("type") == "touch" or "color" not in str(s.get("type", "")):
            return s
    _ = touch
    return cands[0] if cands else None


def load_limits(path: str | None) -> dict:
    """Stage 5A operating ranges + coupled constraints (or {})."""
    if not path:
        return {}
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh) or {}
    except OSError:
        return {}


def build_plan(model: dict, ports: dict, sensors: list[dict],
               homing: dict, limits_doc: dict | None = None) -> dict:
    """Assemble every constant the generated package needs.

    With limits_doc (Stage 5A runs/stage5a/limits.json), the model
    +-180 deg placeholders are replaced by the inferred operating
    ranges, and the coupled inequalities ride into robot_config for
    motion-time enforcement.
    """
    ordered = chain_order(model)
    limits_doc = limits_doc or {}
    boxes = limits_doc.get("limits_deg", {})
    sources = limits_doc.get("limit_source", {})
    coup = coupling(model)
    if coup is None:
        raise ValueError("model has no transmission.C block")
    c, motor_ids = coup["C"], coup["motor_ids"]
    drives = drive_columns(c)
    act_by_id = {a["id"]: a for a in model.get("actuators", [])}

    joints = []
    for j in ordered:
        lo, hi = (float(x) for x in j["limits_deg"])
        src = {"lo": "model", "hi": "model"}
        if j["id"] in boxes:
            blo, bhi = (float(x) for x in boxes[j["id"]])
            lo, hi = blo, bhi
            src = {"lo": sources.get(j["id"], {}).get("lo", "stage5a"),
                   "hi": sources.get(j["id"], {}).get("hi", "stage5a")}
        joints.append({
            "id": j["id"], "parent": j["parent"], "child": j["child"],
            "axis": [float(x) for x in j["axis"]],
            "point_mm": [float(x) for x in j["point_mm"]],
            "limits_deg": [lo, hi],
            "soft_limits_deg": [lo + SOFT_MARGIN_DEG, hi - SOFT_MARGIN_DEG],
            "limit_source": src,
        })

    actuators = []
    for uid in motor_ids:
        a = act_by_id.get(uid, {})
        actuators.append({
            "id": uid, "brick": a.get("brick", "left"),
            "port": motor_port_full(a.get("port", "outA")),
            "motor": a.get("motor", "medium"),
        })
    act_by_uid = {a["id"]: a for a in actuators}

    guard_rules: dict[str, list[dict]] = {}
    for s in ports.get("sensors", []) or []:
        brick = s.get("brick", "left")
        guard_rules.setdefault(brick, []).append({
            "sensor": sensor_port_full(s.get("port")),
            "invert": bool(s.get("invert", False)),
        })
    for brick in ("left", "right"):
        guard_rules.setdefault(brick, [])

    zeros = homing.get("zeros", {}) or {}
    zero_offsets = {uid: float(zeros.get(uid, 0.0)) for uid in motor_ids}
    directions = homing.get("homing_direction", {}) or {}
    confirmed = homing.get("homing_confirmed", {}) or {}

    home_plan = []
    for i, j in enumerate(ordered):
        mid = motor_ids[drives[i]]
        act = act_by_uid[mid]
        sens = homing_sensor_for(j["id"], sensors)
        if sens is None:
            method, sensor_ref = "stall", None
        elif sens.get("brick") == act["brick"]:
            method = "local"
            sensor_ref = {"uid": sens.get("uid"),
                          "brick": sens.get("brick"),
                          "port": sensor_port_full(sens.get("port"))}
        else:
            method = "remote"
            sensor_ref = {"uid": sens.get("uid"),
                          "brick": sens.get("brick"),
                          "port": sensor_port_full(sens.get("port"))}
        home_plan.append({
            "joint": j["id"], "motor": mid,
            "motor_brick": act["brick"], "motor_port": act["port"],
            "method": method, "sensor": sensor_ref,
            "direction": int(directions.get(j["id"], -1)),
            "direction_confirmed": bool(confirmed.get(j["id"], False)),
        })

    home_dirs = {}
    for row in (limits_doc.get("sensor_bounds", []) if limits_doc else []):
        if row.get("status") == "ok" and row.get("homing_direction") \
                in ("+", "-"):
            home_dirs[row["joint"]] = (1 if row["homing_direction"] == "+"
                                       else -1)
    for h in home_plan:
        if not h["direction_confirmed"] and h["joint"] in home_dirs:
            h["direction"] = home_dirs[h["joint"]]
    return {
        "joints": joints, "c": c, "motor_ids": motor_ids,
        "actuators": actuators, "guard_rules": guard_rules,
        "zero_offsets": zero_offsets, "home_plan": home_plan,
        "ee_link": model.get("end_effector", {}).get("link", "D6"),
        "coupled_limits": limits_doc.get("coupled_limits", []),
        "coupled_lookup": limits_doc.get("coupled_lookup", {}),
    }


HEADER_BODY = ('Generated EV3 control package (Stage 5.1).\n'
               '\n'
               'Produced by ``python -m src.codegen.generate`` from the\n'
               'inferred kinematic model. Abstract kinematics + code only:\n'
               'no part IDs, poses or appearance data, so this package is\n'
               'public. Bricks see ports only; joint names live on the PC\n'
               'side.\n'
               '\n'
               'DO NOT hand-edit: regenerate after any model or calibration\n'
               'change and re-run the bring-up checks.')


def _doc(specific: str) -> str:
    """One module docstring: shared header plus file-specific section."""
    return '"""' + HEADER_BODY + '\n\n' + specific + '"""\n'


def emit_config(plan: dict, robot: str) -> str:
    # Python repr, not JSON: the output is a .py file, so True/False/None
    # must be Python literals.
    lines = [_doc("Robot-specific constants for %s: joints, coupling, "
                  "actuators, guards, homing plan." % robot)]
    lines.append("ROBOT = %r" % robot)
    lines.append("")
    lines.append("FIRST_RUN_SPEED_DEG_S = %.1f  # ~30%% of the 100 deg/s cap"
                 % FIRST_RUN_SPEED_DEG_S)
    lines.append("SPEED_CAP_DEG_S = %.1f" % SPEED_CAP_DEG_S)
    lines.append("SOFT_MARGIN_DEG = %.1f  # soft limits narrow the model "
                 "limits" % SOFT_MARGIN_DEG)
    lines.append("")
    lines.append("JOINT_IDS = %r" % [j["id"] for j in plan["joints"]])
    lines.append("MOTOR_IDS = %r" % (plan["motor_ids"],))
    lines.append("")
    lines.append("JOINTS = %s" % pprint.pformat(plan["joints"]))
    lines.append("")
    lines.append("C = %r  # q = C m, joint deg from motor deg" % (plan["c"],))
    lines.append("")
    lines.append("ACTUATORS = %s" % pprint.pformat(plan["actuators"]))
    lines.append("")
    lines.append("ZERO_OFFSETS = %s  # human calibration (homing.local.yaml)"
                 % pprint.pformat(plan["zero_offsets"]))
    lines.append("")
    lines.append("GUARD_RULES = %s" % pprint.pformat(plan["guard_rules"]))
    lines.append("")
    lines.append("HOME_PLAN = %s" % pprint.pformat(plan["home_plan"]))
    lines.append("")
    lines.append("EE_LINK = %r" % plan["ee_link"])
    lines.append("")
    lines.append("LIMIT_SOURCE = %s  # per-side provenance: sensor/collision"
                 % pprint.pformat({j["id"]: j.get("limit_source",
                                                  {"lo": "model",
                                                   "hi": "model"})
                                   for j in plan["joints"]}))
    lines.append("")
    lines.append("COUPLED_LIMITS = %s  # a1*qJa + a2*qJb + b >=/<= 0"
                 % pprint.pformat(plan.get("coupled_limits", [])))
    lines.append("")
    lookup = (plan.get("coupled_lookup", {}) if EMIT_COUPLED_LOOKUP
              else {})
    lines.append("COUPLED_LOOKUP = %s  # 'JA_JB' -> colliding grid cells"
                 % pprint.pformat(lookup))
    lines.append("")
    return "\n".join(lines)


def emit_guards() -> str:
    return ('from __future__ import annotations\n\n' + _doc(
        'Safety guards (Stage 5.1).\n'
        '\n'
        'Every wired sensor stops all motors on its brick when triggered\n'
        'outside a homing move. Guards are local to each brick; the master\n'
        'forwards the stop to the other brick. Call ``apply_guards``\n'
        'before any motion and check the returned acks (exit criteria).') + '''

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
''')


def emit_motion() -> str:
    return ('from __future__ import annotations\n\n' + _doc(
        'Synchronized joint-space motion (Stage 5.1).\n'
        '\n'
        '``move_joints`` converts a joint target to motor targets with\n'
        'the model coupling matrix (joint limits checked first, human\n'
        'zero offsets applied) and fires one ``run_targets`` per brick so\n'
        'every motor starts and finishes together: each motor speed is\n'
        'scaled by its own travel over the shared duration. Targets are\n'
        'absolute encoder angles:\n'
        '\n'
        '    target = home_encoder + (m + zero_offset)\n'
        '\n'
        'where ``home_encoder`` is the reading captured by\n'
        '``home.home_all`` (0.0 when the encoders were zeroed at the\n'
        'model zero instead).') + '''

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
''')


def emit_home() -> str:
    return ('from __future__ import annotations\n\n' + _doc(
        'Homing routine (Stage 5.1).\n'
        '\n'
        'One entry per joint in ``HOME_PLAN`` (base to tip). Three\n'
        'methods: ``local`` (brick homing guard stops the motor on\n'
        'trigger), ``remote`` (PC polls the sensor on the other brick),\n'
        '``stall`` (PC steps until the encoder stops changing, then\n'
        'clears the stall latch). Returns home encoders for ``motion``\n'
        'plus a per-joint report. Unconfirmed directions home toward\n'
        'negative at low speed; confirm them in bring-up step 2.') + '''

from .robot_config import HOME_PLAN

HOME_SPEED_DEG_S = %.1f
HOME_STEP_DEG = 3.0
HOME_MAX_DEG = %.1f
STALL_EPS_DEG = 1.0


def _key(brick, port):
    return f"{brick}:{port}"


def _read_encoders(driver, brick):
    snap = driver.request(brick, "read")
    return {str(k): float(v)
            for k, v in (snap.get("encoders") or {}).items()}


def _read_sensors(driver, brick):
    snap = driver.request(brick, "read")
    return {str(k): v for k, v in (snap.get("sensors") or {}).items()}


def home_joint_local(driver, entry):
    """Home via the brick homing guard (same brick for motor+sensor)."""
    direction = float(entry["direction"]) * HOME_MAX_DEG
    res = driver.home_motor(entry["motor_brick"], entry["motor_port"],
                            entry["sensor"]["port"], direction,
                            HOME_SPEED_DEG_S)
    enc = _read_encoders(driver, entry["motor_brick"])
    return enc.get(entry["motor_port"], 0.0), res


def home_joint_remote(driver, entry):
    """Home by PC polling when the sensor lives on the other brick."""
    mbrick, mport = entry["motor_brick"], entry["motor_port"]
    sbrick, sport = (entry["sensor"]["brick"], entry["sensor"]["port"])
    step = float(entry["direction"]) * HOME_STEP_DEG
    enc = _read_encoders(driver, mbrick)
    pos = enc.get(mport, 0.0)
    travelled = 0.0
    while travelled < HOME_MAX_DEG:
        pos += step
        travelled += abs(step)
        driver.request(mbrick, "run_target",
                       {"port": mport, "target": pos,
                        "speed": HOME_SPEED_DEG_S})
        if int(_read_sensors(driver, sbrick).get(sport, 0)) == 1:
            break
    enc = _read_encoders(driver, mbrick)
    return enc.get(mport, pos), {"method": "remote",
                                 "travelled_deg": travelled}


def home_joint_stall(driver, entry):
    """Home against the mechanical end-stop (no sensor on the joint)."""
    mbrick, mport = entry["motor_brick"], entry["motor_port"]
    step = float(entry["direction"]) * HOME_STEP_DEG
    enc = _read_encoders(driver, mbrick)
    pos, prev = enc.get(mport, 0.0), None
    travelled = 0.0
    while travelled < HOME_MAX_DEG:
        prev = _read_encoders(driver, mbrick).get(mport, pos)
        pos += step
        travelled += abs(step)
        try:
            driver.request(mbrick, "run_target",
                           {"port": mport, "target": pos,
                            "speed": HOME_SPEED_DEG_S})
        except Exception:
            break  # latched stall guard refuses further moves: homed
        now = _read_encoders(driver, mbrick).get(mport, pos)
        if abs(now - prev) < STALL_EPS_DEG:
            pos = now
            break
    try:
        driver.clear_latch(mbrick)
    except Exception:
        pass
    enc = _read_encoders(driver, mbrick)
    return enc.get(mport, pos), {"method": "stall",
                                 "travelled_deg": travelled}


def home_all(driver, joints=None):
    """Home every joint in chain order. Returns the home report."""
    encoders, report = {}, []
    homed_motors = set()
    for entry in HOME_PLAN:
        if joints is not None and entry["joint"] not in joints:
            continue
        motor_key = _key(entry["motor_brick"], entry["motor_port"])
        if motor_key in homed_motors:
            # Coupled joints share one motor (e.g. the wrist): homing the
            # motor once homes every joint it drives.
            pos = encoders[motor_key]
            res = {"method": "shared",
                   "shared_with": [r["joint"] for r in report
                                   if _key(r.get("motor_brick", ""),
                                           r.get("motor_port", ""))
                                   == motor_key],
                   "note": "motor already homed; shared encoder reused"}
            report.append({"joint": entry["joint"], "method": "shared",
                           "home_encoder": pos, "detail": res})
            print(f"[{entry['joint']}] shares homed motor {motor_key} "
                  f"at {pos:.2f} deg")
            continue
        if not entry["direction_confirmed"]:
            print(f"[{entry['joint']}] homing direction "
                  f"{entry['direction']:+d} UNCONFIRMED "
                  f"(bring-up step 2 must confirm it)")
        if entry["method"] == "local":
            pos, res = home_joint_local(driver, entry)
        elif entry["method"] == "remote":
            pos, res = home_joint_remote(driver, entry)
        else:
            pos, res = home_joint_stall(driver, entry)
        encoders[motor_key] = pos
        homed_motors.add(motor_key)
        report.append({"joint": entry["joint"], "method": entry["method"],
                       "motor_brick": entry["motor_brick"],
                       "motor_port": entry["motor_port"],
                       "home_encoder": pos, "detail": res})
        print(f"[{entry['joint']}] homed via {entry['method']} "
              f"at {pos:.2f} deg")
    return {"encoders": encoders, "joints": report}
''') % (HOME_SPEED_DEG_S, HOME_SEARCH_DEG)


def emit_files(plan: dict, robot: str) -> dict[str, str]:
    return {
        "robot_config.py": emit_config(plan, robot),
        "motion.py": emit_motion(),
        "home.py": emit_home(),
        "guards.py": emit_guards(),
    }


def generate(model_path: str, ports_path: str, out_dir: str,
             sensors_path: str | None = None,
             homing_path: str | None = None,
             robot: str = "robot_inferred",
             limits_path: str | None = None) -> dict:
    """Run the full generation. Returns the plan plus written paths."""
    model = load_model(model_path)
    with open(ports_path, encoding="utf-8") as fh:
        ports = yaml.safe_load(fh) or {}
    plan = build_plan(model, ports, load_sensor_map(sensors_path),
                      load_homing(homing_path),
                      load_limits(limits_path))
    files = emit_files(plan, robot)
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for name, content in files.items():
        if "position_mm" in content or '"members"' in content:
            raise ValueError(f"{name} leaks assembly data (poses/members)")
        path = os.path.join(out_dir, name)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(content)
        written.append(path)
    with open(os.path.join(out_dir, "__init__.py"), "w",
              encoding="utf-8") as fh:
        fh.write('"""Generated EV3 control package (Stage 5.1)."""\n')
    written.append(os.path.join(out_dir, "__init__.py"))
    return {"plan": plan, "written": written}


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate EV3 control package")
    ap.add_argument("--model", required=True, help="model JSON")
    ap.add_argument("--ports", required=True,
                    help="config/ports.local.yaml (hardware layer)")
    ap.add_argument("--out", required=True, help="generated/<robot> dir")
    ap.add_argument("--sensors", default=None,
                    help="agent sensor map (Stage 2.7 output)")
    ap.add_argument("--homing", default=None,
                    help="config/homing.local.yaml (human calibration)")
    ap.add_argument("--robot", default="robot_inferred")
    ap.add_argument("--limits", default=None,
                    help="runs/stage5a/limits.json (Stage 5A ranges)")
    args = ap.parse_args()
    res = generate(args.model, args.ports, args.out, args.sensors,
                   args.homing, args.robot, args.limits)
    print(f"wrote {len(res['written'])} files to {args.out}")
    for entry in res["plan"]["home_plan"]:
        sens = (entry["sensor"]["port"] + "@" + entry["sensor"]["brick"]
                if entry["sensor"] else "stall")
        print(f"  {entry['joint']}: motor {entry['motor']} "
              f"({entry['motor_brick']}/{entry['motor_port']}) "
              f"via {entry['method']}:{sens} "
              f"dir={entry['direction']:+d}"
              + ("" if entry["direction_confirmed"] else " UNCONFIRMED"))


if __name__ == "__main__":
    main()
