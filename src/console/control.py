"""5C control console (fake-first, 127.0.0.1 only).

Reuses the mini console's API and transport: every brick command goes
through :mod:`src.console.mini` handlers and the same MQTT/fake
transport. This screen adds, on top of mini:

- status bar: brick connection, latch, heartbeat freshness, E-STOP
  (always visible),
- joint panel: joint angle + motor angle, jog, homing status; joints
  with unconfirmed direction are marked "방향 미확인" and their jog
  is limited to +/-5 joint deg,
- 3D view: the robot package's primitive link boxes (link_envelopes)
  posed by current joint angles (q = ratio*m + offset),
- M4 wizard: +5 deg jog per joint, compare model vs real, record
  correct/reversed; reversed flips the gear-ratio sign in the config,
- M5 range wizard: 5A sim ranges + measured endpoints -> adopted
  ranges (narrower wins, minus margin), saved per day under runs/,
- M6 pose wizard: 5A safe poses reproduced at low speed, per-joint
  error recorded, range/coupled-limit refusals with reasons.
- tunables (wizard step, jog cap, margin, pose speed) via /api/config,
  overridable in config/console.local.json.
- IK target gizmo (TransformControls): drag/rotate the orange marker,
  /api/ik solves with src.kin.ik clamped to adopted ranges; skeleton
  limbs are capsules sized from link envelopes, posed by the same FK.
- Visual source switch: primitives (default) or LDraw link meshes.
  /api/visual returns link id -> mesh url only; the page poses each
  mesh with F_i (identity link frame). LDraw .glb files live in
  git-ignored assets/private/visual_pack/ROBOT/ and are served
  (same-origin page, no-store) for the matching robot: explicit
  --private-visuals ROBOT, else the robot's pack auto-detected.

Direction confirmations live in ``config/direction.local.json``
(absent = unconfirmed). A reversed verdict flips the ratio sign via
the same override file the map editor uses
(``config/jogmap.local.json``).

Run::

    source .venv/bin/activate
    python -m src.console.control                 # fake bricks
    python -m src.console.control --config config/hardware.local.yaml
    # open http://127.0.0.1:8766 in a browser
"""

from __future__ import annotations

import argparse
import datetime
import json
import mimetypes
import pathlib

from src.console import mini

CONTROL_PORT = 8766
MAX_UNCONFIRMED_JOG_DEG = 5.0

# Console robot selection (additive; default MR JOS behavior unchanged).
ROBOT_IDS = ("mrjos", "tbot")

# Visual pack dir per robot (private, git-ignored LDraw meshes).
ROBOT_PACKS = {"mrjos": "robot_inferred", "tbot": "tbot"}


def _pack_has_meshes(root, name):
    d = (pathlib.Path(root) / "assets" / "private" / "visual_pack"
         / name)
    return d.is_dir() and bool(list(d.glob("D*.glb")))


def resolve_visual_pack(explicit, robot, root=None):
    """Which visual pack dir to serve, or None for primitives-only.

    An explicit --private-visuals keeps legacy behavior (served
    blindly, per-file 404). Otherwise the robot's pack dir is
    auto-detected when it holds D*.glb files, so LDraw just works
    without flags. Loopback-only serving, no-store, never in git:
    the privacy posture is unchanged.
    """
    if root is None:
        root = pathlib.Path(__file__).resolve().parents[2]
    if explicit and explicit == ROBOT_PACKS.get(robot):
        return explicit
    auto = ROBOT_PACKS.get(robot)
    if auto and _pack_has_meshes(root, auto):
        return auto
    return None


def visual_pack_for(handler):
    """Resolved pack for this handler's robot (flag or auto)."""
    return resolve_visual_pack(getattr(handler, "private_visuals",
                                       None),
                               getattr(handler, "robot", "mrjos"))


def robot_files(robot):
    """Model + package paths per console robot.

    mrjos: committed pipeline outputs. tbot: private stage-7
    analysis (git-ignored runs/); never uploaded anywhere.
    """
    root = pathlib.Path(__file__).resolve().parents[2]
    if robot == "tbot":
        return (root / "runs" / "stage7_tbot_t2" / "tbot_model.json",
                root / "runs" / "stage7_tbot_t2" / "tbot_package.json")
    if robot == "mrjos":
        return (root / "models" / "robot_inferred.json",
                root / "packages" / "robot_inferred"
                / "robot_package.json")
    raise ValueError("unknown robot: " + str(robot))


def tbot_jog_map():
    """Display-only joint map for virtual-only T-Bot (no brick link).

    brick/port "-" means no encoder source; ratio is mm per motor
    degree where measured (P1 shaft-direct) else None (drive unknown).
    """
    return {
        "P1": {"motor_uid": "A", "brick": "-", "port": "-",
               "ratio": 153.6 / 360.0},
        "P2": {"motor_uid": "B", "brick": "-", "port": "-",
               "ratio": None},
        "P3": {"motor_uid": "C", "brick": "-", "port": "-",
               "ratio": None},
    }


def select_robot(handler, robot):
    """Swap the live model + joint map (shared class state, in place).

    T-Bot stays virtual-only: callers must refuse brick motion via
    TBOT_BLOCKED while handler.robot == "tbot".
    """
    if robot not in ROBOT_IDS:
        raise ValueError("unknown robot: " + str(robot))
    from src.kin.model import load_model

    model_path, _ = robot_files(robot)
    cls = type(handler)
    cls.robot = robot
    cls.model_doc = load_model(str(model_path))
    fresh = (dict(tbot_jog_map()) if robot == "tbot"
             else mini.load_jog_map(
                 overrides=mini.read_jog_overrides(
                     handler.override_path)))
    handler.jog_map.clear()
    handler.jog_map.update(fresh)
    return {"ok": True, "robot": robot}


# Motion/sensor APIs refused while T-Bot is selected: T-Bot is
# virtual-only, real brick connection stays blocked.
TBOT_BLOCKED = frozenset({
    "/api/jog", "/api/jogmap/set", "/api/jogmap/reset",
    "/api/jogmap/delete", "/api/markzero", "/api/home", "/api/diag",
    "/api/goto", "/api/preview", "/api/home_joint", "/api/direction",
    "/api/pose", "/api/range",
    "/api/calib/park", "/api/calib/restore", "/api/calib/check",
    "/api/calib/confirm", "/api/calib/flow",
    "/api/twin/run", "/api/twin/rest",
})

STATIC_DIR = pathlib.Path(__file__).resolve().parent / "control_static"


def default_direction_path():
    root = pathlib.Path(__file__).resolve().parents[2]
    return root / "config" / "direction.local.json"


def read_directions(path=None):
    """{joint: "confirmed"}; missing file or joint means unconfirmed."""
    if path is None:
        path = default_direction_path()
    try:
        data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    return {str(k): str(v) for k, v in data.items()}


def write_directions(directions, path=None):
    if path is None:
        path = default_direction_path()
    pathlib.Path(path).write_text(json.dumps(directions, indent=1,
                                             sort_keys=True),
                                  encoding="utf-8")


def latest_homing_record(base=None):
    """Latest runs/stage5c/<date>/homing.json offsets+results, or {}."""
    root = pathlib.Path(__file__).resolve().parents[2]
    base = pathlib.Path(base or (root / "runs" / "stage5c"))
    best = None
    try:
        days = sorted(p for p in base.iterdir() if p.is_dir())
    except OSError:
        return {}
    for day in days:
        path = day / "homing.json"
        if path.exists():
            best = path
    if best is None:
        return {}
    try:
        return json.loads(best.read_text(encoding="utf-8"))
    except ValueError:
        return {}


def joint_overview(handler):
    """One poll for the joint panel: encoders, angles, latch, homing."""
    t = handler.transport
    encoders = {}
    latched = {}
    sensor_state = {}
    for brick in ("left", "right"):
        try:
            state = t.send(brick, "read")
        except Exception:
            latched[brick] = None
            continue
        latched[brick] = bool(state.get("latched"))
        for port, value in (state.get("encoders") or {}).items():
            encoders[(brick, port)] = float(value)
        for port, value in (state.get("sensors") or {}).items():
            sensor_state[(brick, port)] = value
    try:
        directions = read_directions(handler.direction_path)
    except AttributeError:
        directions = {}
    sensors = joint_sensors()
    record = latest_homing_record()
    offsets = record.get("offsets", {}) or {}
    homed = {res["joint"]: res.get("status") == "homed"
             for res in record.get("results", [])}
    try:
        readback = mini.coupled_joint_readback(encoders,
                                               handler.jog_map,
                                               offsets)
    except KeyError:
        # Non-6R maps (no wrist coupling): independent readback,
        # joint = ratio * motor + offset; missing encoders stay None.
        readback = {}
        for joint_id, entry in handler.jog_map.items():
            motor = encoders.get((entry["brick"], entry["port"]))
            ratio = entry.get("ratio")
            readback[joint_id] = (
                ratio * motor + float(offsets.get(joint_id, 0.0) or 0.0)
                if motor is not None and ratio is not None else None)
    joints = []
    for joint_id, entry in handler.jog_map.items():
        motor = encoders.get((entry["brick"], entry["port"]))
        ratio = entry["ratio"]
        joints.append({
            "joint": joint_id,
            "brick": entry["brick"],
            "port": entry["port"],
            "ratio": ratio,
            "motor_deg": motor,
            "joint_deg": readback.get(joint_id),
            "offset": offsets.get(joint_id),
            "homed": bool(homed.get(joint_id, False)),
            "direction": directions.get(joint_id, "unconfirmed"),
            "sensor": sensors.get(joint_id),
            "sensor_state": sensor_state.get(
                (entry["brick"], sensors.get(joint_id))),
        })
    return {"ok": True, "joints": joints, "latched": latched,
            "server_time": datetime.datetime.now().isoformat(
                timespec="seconds")}


def model_payload(handler):
    """Kinematics + primitive boxes for the 3D view."""
    from src.kin.model import chain_order, ee_point, joint_arrays

    doc = handler.model_doc
    arr = joint_arrays(doc)
    kinds = arr.get("kinds", ["revolute"] * len(arr["points"]))
    joints = [{"id": j["id"], "axis": list(j["axis"]),
               "point": list(p), "kind": k,
               "unit": ("mm" if k == "prismatic" else "deg")}
              for j, p, k in zip(chain_order(doc), arr["points"],
                                 kinds)]
    try:
        root = pathlib.Path(__file__).resolve().parents[2]
        package = json.loads((root / "packages" / "robot_inferred"
                              / "robot_package.json").read_text(
                                  encoding="utf-8"))
        order = [j["id"] for j in chain_order(doc)]
        envelopes = []
        for env in package.get("link_envelopes", []):
            envelopes.append({
                "link": env.get("link"),
                "carrier": (order[int(env["link"][1:]) - 1]
                            if str(env.get("link", "D0")) != "D0"
                            else None),
                "center_mm": list(env.get("center_mm", [0, 0, 0])),
                "size_mm": list(env.get("size_mm", [0, 0, 0])),
            })
    except (OSError, ValueError, KeyError, IndexError):
        envelopes = []
    frame = doc.get("frame", "") if isinstance(doc, dict) else ""
    if all(k == "prismatic" for k in kinds):
        tip = list(ee_point(doc))
    else:
        tip = tip_offset()
    jp = arr["points"][-1]
    import math as _m
    _d = [tip[i] - jp[i] for i in range(3)]
    _n = _m.sqrt(sum(v * v for v in _d))
    if _n < 1e-9:
        _d, _n = [0.0, 0.0, -1.0], 1.0
    return {"ok": True, "joints": joints,
            "ee_mm": list(ee_point(doc)), "envelopes": envelopes,
            "units": "mm,deg", "frame": frame,
            "tip_mm": tip,
            "gripper_dir0": [v / _n for v in _d]}


TUNABLE_DEFAULTS = {
    "wizard_step_deg": 5.0,
    "unconfirmed_jog_deg": 5.0,
    "measure_margin_deg": 5.0,
    "pose_speed_deg_s": 30.0,
}


def default_console_path():
    root = pathlib.Path(__file__).resolve().parents[2]
    return root / "config" / "console.local.json"


def read_console_config(path=None):
    """Runtime tunables; file values override TUNABLE_DEFAULTS."""
    if path is None:
        path = default_console_path()
    try:
        data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return {k: v for k, v in data.items() if k in TUNABLE_DEFAULTS} \
        if isinstance(data, dict) else {}


def tunables(handler):
    try:
        extra = read_console_config(handler.console_path)
    except AttributeError:
        extra = {}
    out = dict(TUNABLE_DEFAULTS)
    out.update({k: float(v) for k, v in extra.items()})
    return out


def package_doc(handler=None):
    root = pathlib.Path(__file__).resolve().parents[2]
    if handler is not None and getattr(handler, "robot", "mrjos") == "tbot":
        _, path = robot_files("tbot")
    else:
        path = (root / "packages" / "robot_inferred"
                / "robot_package.json")
    return json.loads(path.read_text(encoding="utf-8"))


STAGE_DEFAULT = {"table_z_mm": -220.8, "center_x_mm": 34.0,
                   "center_y_mm": 79.5, "reach_r_mm": 335.3,
                   "shoulder_model_mm": [21.8, 79.7, -53.0]}


def stage_table():
    """Single shared table/base-center definition (model frame, mm).

    Screen, IK targets, mat points and reports all use this one
    value: stage = model - (center_x, center_y, table_z).
    """
    import yaml

    root = pathlib.Path(__file__).resolve().parents[2]
    try:
        data = yaml.safe_load(
            (root / "config" / "calibration.local.yaml").read_text(
                encoding="utf-8")) or {}
        st = data.get("stage", {}) or {}
        out = dict(STAGE_DEFAULT)
        for k in ("table_z_mm", "center_x_mm", "center_y_mm",
                  "reach_r_mm"):
            if st.get(k) is not None:
                out[k] = float(st[k])
        if st.get("shoulder_model_mm") is not None:
            out["shoulder_model_mm"] = [float(v) for v in
                                        st["shoulder_model_mm"]]
        return out
    except (OSError, ValueError, ImportError, TypeError):
        return dict(STAGE_DEFAULT)


TIP_DEFAULT = [-160.0, 236.8, 134.2]


def tip_offset():
    """Gripper-tip offset in the D6 frame (== zero-pose model coords)."""
    import yaml

    root = pathlib.Path(__file__).resolve().parents[2]
    try:
        data = yaml.safe_load(
            (root / "config" / "calibration.local.yaml").read_text(
                encoding="utf-8")) or {}
        tip = (data.get("tcp", {}) or {}).get("tip_local_mm")
        return [float(v) for v in tip] if tip else list(TIP_DEFAULT)
    except (OSError, ValueError, ImportError, TypeError):
        return list(TIP_DEFAULT)


def tip_doc(doc):
    """Doc copy whose TCP is the gripper tip (solver/display only).

    Docs without any revolute joint (pure-prismatic gantries) have
    no wrist: the model end effector already is the tool point.
    """
    joints = doc.get("joints", []) if isinstance(doc, dict) else []
    if joints and all(j.get("type") == "prismatic" for j in joints):
        return dict(doc)
    tip = tip_offset()
    out = dict(doc)
    out["end_effector"] = {"link": "D6", "point_mm": tip}
    return out


def gripper_down_mat(doc=None):
    """Minimal rotation taking the D6-frame gripper dir to -z.

    d0 = normalize(tip - J6 point) at the zero pose.
    """
    import math

    tip = tip_offset()
    if doc is not None:
        from src.kin.model import joint_arrays
        jp = joint_arrays(doc)["points"][5]
    else:
        jp = [-115.29, 216.88, 162.33]
    d = [tip[i] - jp[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    d = [v / n for v in d]
    t = [0.0, 0.0, -1.0]
    ax = [d[1] * t[2] - d[2] * t[1],
          d[2] * t[0] - d[0] * t[2],
          d[0] * t[1] - d[1] * t[0]]
    an = math.sqrt(sum(v * v for v in ax))
    cos_a = max(-1.0, min(1.0, sum(d[i] * t[i] for i in range(3))))
    if an < 1e-9:
        if cos_a > 0:
            return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]
        ax, an = [1.0, 0.0, 0.0], 1.0
    ax = [v / an for v in ax]
    sn, cs = math.sqrt(max(0.0, 1.0 - cos_a * cos_a)), cos_a
    x, y, z = ax
    return [[cs + x * x * (1 - cs), x * y * (1 - cs) - z * sn,
             x * z * (1 - cs) + y * sn],
            [y * x * (1 - cs) + z * sn, cs + y * y * (1 - cs),
             y * z * (1 - cs) - x * sn],
            [z * x * (1 - cs) - y * sn, z * y * (1 - cs) + x * sn,
             cs + z * z * (1 - cs)]]


def joint_sensors(path=None):
    """{joint: sensor_port} from homing.local.yaml; {} when missing."""
    import yaml

    root = pathlib.Path(__file__).resolve().parents[2]
    try:
        data = yaml.safe_load((root / "config" / "homing.local.yaml"
                               if path is None else path).read_text(
                                   encoding="utf-8"))
    except (OSError, ValueError, ImportError):
        return {}
    out = {}
    for joint, spec in ((data or {}).get("joints") or {}).items():
        if isinstance(spec, dict) and spec.get("sensor"):
            out[str(joint)] = str(spec["sensor"])
    return out


def runs_day_dir(base=None):
    root = pathlib.Path(__file__).resolve().parents[2]
    day = datetime.datetime.now().strftime("%Y-%m-%d")
    path = pathlib.Path(base or (root / "runs" / "stage5c")) / day
    path.mkdir(parents=True, exist_ok=True)
    return path


def read_json(path, default):
    try:
        return json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def ranges_payload(handler):
    """5A sim ranges + saved measured endpoints + adopted ranges."""
    pkg = package_doc(handler)
    sim = {j["id"]: list(j.get("limits_mm", j.get("limits_deg")))
           for j in pkg.get("joints", [])}
    src = {j["id"]: j.get("limit_source", {}) for j in pkg.get("joints", [])}
    stored = read_json(runs_day_dir() / "ranges.json", {})
    measured = stored.get("measured", {}) or {}
    margin = tunables(handler)["measure_margin_deg"]
    joints = []
    for joint in handler.jog_map:
        lo_sim, hi_sim = sim.get(joint, [None, None])
        meas = measured.get(joint, {}) or {}
        lo_m, hi_m = meas.get("lo"), meas.get("hi")
        if (lo_m is not None and hi_m is not None
                and not lo_m < hi_m):
            # Degenerate point capture (lo == hi): not a range.
            lo_m, hi_m = None, None
        lo = (max(lo_sim, lo_m) + margin
              if lo_sim is not None and lo_m is not None
              else lo_sim if lo_m is None else lo_m + margin)
        hi = (min(hi_sim, hi_m) - margin
              if hi_sim is not None and hi_m is not None
              else hi_sim if hi_m is None else hi_m - margin)
        joints.append({
            "joint": joint, "sim": [lo_sim, hi_sim],
            "sim_source": src.get(joint, {}),
            "measured": [lo_m, hi_m],
            "adopted": [lo, hi],
            "adopted_source": ("measured" if lo_m is not None
                               and hi_m is not None else "sim"),
        })
    pkg_margin = pkg.get("operating_margin_deg",
                         pkg.get("operating_margin_mm"))
    return {"ok": True, "joints": joints,
            "package_margin_deg": pkg_margin,
            "margin_deg": margin}


def record_range(handler, joint, end, joint_deg):
    if joint not in handler.jog_map:
        raise KeyError("unknown joint: " + str(joint))
    if end not in ("lo", "hi"):
        raise ValueError("end must be lo|hi")
    path = runs_day_dir() / "ranges.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    stored = read_json(path, {})
    measured = stored.get("measured", {}) or {}
    entry = dict(measured.get(joint, {}) or {})
    entry[end] = float(joint_deg)
    measured[joint] = entry
    stored["measured"] = measured
    path.write_text(json.dumps(stored, indent=1), encoding="utf-8")
    return ranges_payload(handler)


def check_pose_against_ranges(q, adopted):
    """Refuse targets outside adopted ranges or coupled limits."""
    bad = []
    for joint, (lo, hi) in adopted.items():
        if joint in q and (lo is not None and q[joint] < lo
                           or hi is not None and q[joint] > hi):
            bad.append(f"{joint} {q[joint]:.1f} outside [{lo}, {hi}]")
    if bad:
        return "; ".join(bad)
    try:
        pkg = package_doc()
    except (OSError, ValueError):
        return ""
    for rule in pkg.get("coupled_limits", []) or []:
        # Same semantics as src.kin.ik._coupled_ok:
        # val = a.q + b must satisfy op against 0.
        joints = rule.get("joints", [])
        if all(j in q for j in joints):
            val = (sum(a * q[j]
                       for a, j in zip(rule.get("a", []), joints))
                   + rule.get("b", 0.0))
            op = rule.get("op")
            ok = (val >= -1e-9 if op == ">=" else val <= 1e-9 if op == "<="
                  else abs(val) < 1e-9)
            if not ok:
                bad.append(f"coupled {'+'.join(joints)} = {val:.1f} "
                           f"violates {op} 0")
    return "; ".join(bad)


def solve_ik(handler, pos, mat, q_init, position_only):
    """DLS IK from src.kin.ik, clamped to adopted ranges + coupled.

    TCP is the gripper tip (tip_doc); the wrist-center ee is kept
    only for the GT overlay.
    """
    from src.kin.ik import ik
    from src.kin.model import joint_arrays

    doc = tip_doc(handler.model_doc)
    ids = joint_arrays(doc)["ids"]
    adopted = {r["joint"]: r["adopted"]
               for r in ranges_payload(handler)["joints"]}
    pkg = package_doc(handler)
    if not isinstance(pos, (list, tuple)) or len(pos) != 3:
        raise ValueError("pos must be [x, y, z] in mm")
    if mat is not None:
        if (not isinstance(mat, (list, tuple)) or len(mat) != 3
                or any(len(row) != 3 for row in mat)):
            raise ValueError("mat must be 3x3 or null")
        mat = [[float(x) for x in row] for row in mat]
    # Multi-start: current pose (q_init) + 10 safe 5A poses +
    # previous solution; all run, min TCP error wins.
    order = [j["id"] for j in pkg.get("joints", [])]
    starts = []
    for pose in pkg.get("safe_poses", []) or []:
        qd = dict(zip(order,
                      pose.get("q_deg", []) or pose.get("q", [])))
        if all(j in qd for j in ids):
            starts.append([float(qd[j]) for j in ids])
    prev = getattr(handler, "last_ik", None)
    if prev:
        starts.append([float(x) for x in prev])
    if mat is not None and q_init:
        # Orientation needs wrist-diverse seeds: keep the arm
        # (J1-J3) from the incoming pose, borrow wrists (J4-J6)
        # from safe poses plus a few deterministic twists.
        import random as _rng
        qr = _rng.Random(99)
        wrists = [s[3:6] for s in starts] + [
            [float(qr.uniform(-115, 125)),
             float(qr.uniform(-90, 124)),
             float(qr.uniform(-60, 135))] for _ in range(6)]
        base = [float(x) for x in q_init]
        for w in wrists[:12]:
            starts.append(base[:3] + [float(v) for v in w])
    # The gizmo cannot go below the table: clamp the model-frame
    # target at table_z (same single value the screen/mat use).
    st = stage_table()
    pos = [float(x) for x in pos]
    clamped = False
    if pos[2] < st["table_z_mm"]:
        pos[2] = st["table_z_mm"]
        clamped = True
    res = ik(pos, mat, doc,
             q_init_deg=([float(x) for x in q_init]
                         if q_init else None),
             position_only=bool(position_only),
             limits_doc={
                 "limits_deg": {j: adopted.get(j) for j in ids},
                 "limits_mm": {j: adopted.get(j) for j in ids},
                 "coupled_limits":
                     pkg.get("coupled_limits", []) or []},
             extra_starts=starts)
    try:
        handler.last_ik = [float(x) for x in res.get("q_deg", [])]
    except (TypeError, ValueError):
        pass
    return {"ik": res, "joints": ids, "clamped": clamped,
            "pos_used": [round(v, 1) for v in pos],
            "table_z_mm": st["table_z_mm"]}


def reachable_target(handler, seed=None):
    """Random in-range q -> its FK end point (guaranteed reachable).

    Samples adopted ranges + coupled limits, so the gizmo test
    always has a '닿는 목표'. Same FK the console renders.
    """
    import random
    from src.kin.fk import fk_model
    from src.kin.ik import _coupled_ok
    from src.kin.model import joint_arrays

    doc = tip_doc(handler.model_doc)
    ids = joint_arrays(doc)["ids"]
    adopted = {r["joint"]: r["adopted"]
               for r in ranges_payload(handler)["joints"]}
    pkg = package_doc(handler)
    coupled = pkg.get("coupled_limits", []) or []
    rng = random.Random(seed)
    for _ in range(2000):
        q = [rng.uniform(*adopted[j]) for j in ids]
        if _coupled_ok([float(v) for v in q], ids, coupled):
            pos = fk_model([float(v) for v in q], doc)["pos_mm"]
            return {"ok": True,
                    "q_deg": [round(float(v), 2) for v in q],
                    "target": [round(float(v), 1) for v in pos],
                    "joints": ids}
    raise ValueError("no in-range sample in 2000 tries")


def move_joints(handler, q, speed=None, label="goto"):
    """Range-checked joint move via coupled motor targets.

    Refuses when any brick reports a latch (send clear_latch first).
    Homing/connection gating stays client-side so fake-brick demos
    keep working; the client hides send when disconnected.
    """
    latched = []
    for brick in handler.transport.bricks:
        try:
            if bool(handler.transport.send(brick, "read").get("latched")):
                latched.append(brick)
        except Exception:
            pass
    if latched:
        raise ValueError("refused: latched on "
                         + ",".join(latched)
                         + "; send clear_latch first")
    adopted = {r["joint"]: r["adopted"]
               for r in ranges_payload(handler)["joints"]}
    refusal = check_pose_against_ranges(q, adopted)
    if refusal:
        raise ValueError("refused: " + refusal)
    record = latest_homing_record()
    offsets = record.get("offsets", {}) or {}
    if speed is None:
        speed = tunables(handler)["pose_speed_deg_s"]
    t = handler.transport
    targets = mini.coupled_motor_targets(q, handler.jog_map, offsets)
    speeds = {joint: min(abs(speed / handler.jog_map[joint]["ratio"]),
                         250.0)
              for joint in q}
    t.move_motors(targets, speed=max(speeds.values()))
    errors = {}
    enc = {}
    for brick in t.bricks:
        enc.update({(brick, port): motor for port, motor in
                    t.send(brick, "read")["encoders"].items()})
    readback = mini.coupled_joint_readback(enc, handler.jog_map, offsets)
    for joint, target in q.items():
        errors[joint] = readback[joint] - target
    path = runs_day_dir() / "report.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    stored = read_json(path, {})
    runs = stored.get("poses", []) or []
    runs.append({"pose": label, "target": q, "error_deg": errors,
                 "at": datetime.datetime.now().isoformat(
                     timespec="seconds")})
    stored["poses"] = runs
    path.write_text(json.dumps(stored, indent=1), encoding="utf-8")
    from src.kin.fk import fk_model
    from src.kin.model import joint_arrays
    order = joint_arrays(handler.model_doc)["ids"]
    tcp = fk_model([float(q[j]) for j in order],
                   tip_doc(handler.model_doc))["pos_mm"]
    return {"ok": True, "pose": label, "error_deg": errors,
            "max_abs_err": max(abs(v) for v in errors.values()),
            "tcp": [round(float(v), 1) for v in tcp]}


def homing_spec():
    """Per-joint homing config from config/homing.local.yaml."""
    import yaml

    root = pathlib.Path(__file__).resolve().parents[2]
    try:
        data = yaml.safe_load(
            (root / "config" / "homing.local.yaml").read_text(
                encoding="utf-8")) or {}
        return data.get("joints", {}) or {}
    except (OSError, ValueError, ImportError):
        return {}


def write_homing_result(joint, status, offset, note=""):
    """Merge one joint result into today's homing.json."""
    day = runs_day_dir()
    day.mkdir(parents=True, exist_ok=True)
    path = day / "homing.json"
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        record = {}
    try:
        latest = latest_homing_record()
    except Exception:
        latest = {}
    offsets = dict(latest.get("offsets", {}) or {})
    offsets.update(record.get("offsets", {}) or {})
    results = [r for r in (record.get("results", []) or [])
               if r.get("joint") != joint]
    if offset is not None:
        offsets[joint] = offset
    results.append({"joint": joint, "status": status,
                    "offset": offset, "note": note})
    record = dict(record)
    record["offsets"] = offsets
    record["results"] = results
    path.write_text(json.dumps(record, indent=1), encoding="utf-8")
    return record


def home_joint(handler, joint, mark=False):
    """Home one joint: sensor run (real) or simulated (fake bricks).

    Manual joints (J5/J6): mark=True records the current position
    as zero (offset = -ratio * motor_pos).
    """
    spec = homing_spec().get(joint, {}) or {}
    method = spec.get("method", "manual")
    if joint not in handler.jog_map:
        raise ValueError("unknown joint: " + str(joint))
    entry = handler.jog_map[joint]
    record = latest_homing_record()
    offsets = record.get("offsets", {}) or {}
    if method == "manual" or mark:
        enc = {}
        for brick in handler.transport.bricks:
            try:
                enc.update(
                    {(brick, port): motor for port, motor in
                     handler.transport.send(brick, "read")
                     ["encoders"].items()})
            except Exception:
                pass
        raw = mini.coupled_joint_readback(
            enc, handler.jog_map, {}) or {}
        cur = float(raw.get(joint, 0.0))
        offset = -float(entry["ratio"]) * float(
            enc.get((entry["brick"], entry["port"]), 0.0) or 0.0)
        if cur == 0.0:
            # No readback (no record yet): fall back to motor raw.
            offset = -float(entry["ratio"]) * float(
                enc.get((entry["brick"], entry["port"]), 0.0) or 0.0)
        write_homing_result(joint, "homed", offset,
                            "manual mark at %.1f deg" % cur)
        return {"ok": True, "joint": joint, "method": "manual",
                "offset": offset}
    if getattr(handler, "virtual", False):
        offset = offsets.get(joint, 0.0)
        write_homing_result(joint, "homed", offset, "simulated")
        return {"ok": True, "joint": joint, "method": "sensor",
                "offset": offset, "simulated": True}
    sensors = joint_sensors()
    sensor = sensors.get(joint)
    if isinstance(sensor, dict):
        sensor = sensor.get("port", sensor.get("sensor"))
    if not sensor:
        raise ValueError("no sensor for " + str(joint))
    direction = int(spec.get("direction", -1) or -1)
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or +1")
    speed = float(spec.get("speed", 150.0) or 150.0)
    entry_brick, entry_port = entry["brick"], entry["port"]
    try:
        handler.transport.send(entry_brick, "set_guards",
                               {"rules": [{"sensor": sensor}]})
        ack = handler.transport.send(
            entry_brick, "run_target",
            {"port": entry_port, "target": direction * 720.0,
             "speed": speed, "until_sensor": sensor})
    except Exception as e:
        write_homing_result(joint, "failed", None, str(e))
        raise ValueError("homing failed: " + str(e))
    offset = offsets.get(joint)
    write_homing_result(joint, "homed", offset, "sensor trip")
    return {"ok": True, "joint": joint, "method": "sensor",
            "offset": offset,
            "homed": bool((ack or {}).get("homed", False))}


def _check_deviation(handler):
    """Encoder readback vs park pose, per-joint and max (deg)."""
    from src.console import calib_session as _cs

    enc = {}
    for brick in handler.transport.bricks:
        try:
            enc.update(
                {(brick, port): motor for port, motor in
                 handler.transport.send(brick, "read")
                 ["encoders"].items()})
        except Exception:
            pass
    record = latest_homing_record()
    offsets = record.get("offsets", {}) or {}
    real = mini.coupled_joint_readback(enc, handler.jog_map, offsets)
    dev, vals = {}, {}
    worst = None
    for jid, want in _cs.PARK_Q_DEG.items():
        got = real.get(jid)
        vals[jid] = None if got is None else round(float(got), 2)
        if got is None:
            dev[jid] = None
            continue
        d = round(float(got) - float(want), 2)
        dev[jid] = d
        if worst is None or abs(d) > abs(worst):
            worst = d
    return {"q_real": vals, "dev_deg": dev, "max_abs_deg":
            None if worst is None else round(abs(worst), 2)}


def _calibration_flow(handler):
    """One button: J1-J4 sensor home, J5/J6 restore-or-manual, check."""
    from src.console import calib_session as _cs

    steps = []
    saved = _cs.load_session()
    now = _cs.read_boot(handler)
    cont = _cs.boot_status(saved.get("boot", {}), now)
    steps.append({"step": "continuity", "result": cont,
                  "boot_now": now})
    for jid in ("J1", "J2", "J3", "J4"):
        try:
            r = home_joint(handler, jid)
            steps.append({"step": "home-" + jid, "result": "homed",
                          "method": r.get("method"),
                          "simulated": bool(r.get("simulated"))})
        except ValueError as e:
            steps.append({"step": "home-" + jid, "result": "failed",
                          "error": str(e)})
            return {"ok": True, "flow": "stopped", "steps": steps}
    if cont == "same" and saved.get("offsets", {}).get("J5") is not None \
            and saved.get("offsets", {}).get("J6") is not None:
        for jid in ("J5", "J6"):
            write_homing_result(
                jid, "homed", float(saved["offsets"][jid]),
                "restored from session "
                + str(saved.get("saved_at", "")))
        steps.append({"step": "wrist", "result": "restored",
                      "note": "J5/J6 offsets from session"})
    else:
        steps.append({"step": "wrist", "result": "need-manual",
                      "note": "J5/J6: match the reference picture, "
                      "then press each 'mark as 0' "
                      "(continuity=" + cont + ")"})
        return {"ok": True, "flow": "need-manual", "steps": steps}
    res = move_joints(handler, dict(_cs.PARK_Q_DEG),
                      speed=_cs.PARK_SPEED_DPS, label="flow-check")
    dev = _check_deviation(handler)
    steps.append({"step": "check", "result": "moved",
                  "max_abs_deg": dev["max_abs_deg"],
                  "note": "press 'looks right' to confirm"})
    return {"ok": True, "flow": "done", "steps": steps,
            "move": res, "deviation": dev}


def reproduce_pose(handler, pose_id):
    pkg = package_doc(handler)
    poses = {p["id"]: p for p in pkg.get("safe_poses", [])}
    if pose_id not in poses:
        raise KeyError("unknown pose: " + str(pose_id))
    order = [j["id"] for j in pkg.get("joints", [])]
    q = dict(zip(order, poses[pose_id].get("q_deg", [])))
    return move_joints(handler, q, label=pose_id)


class _Handler(mini._Handler):
    direction_path = None
    model_doc = None
    zeros = {}
    console_path = None
    private_visuals = None

    def _serve_private(self, path):
        # Private LDraw-derived meshes: resolved pack only (explicit
        # flag or per-robot auto-detect), only same-origin page loads,
        # never cached, never in git.
        allowed = visual_pack_for(self)
        if not allowed:
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        rel = path[len("/private/visual/"):]
        parts = rel.split("/")
        if (not rel or ".." in rel or rel.startswith("/")
                or len(parts) != 2):
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        robot, name = parts
        if robot != allowed:
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        if not (name.endswith(".glb") or name == "manifest.json"):
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        root = pathlib.Path(__file__).resolve().parents[2]
        target = root / "assets" / "private" / "visual_pack" / robot / name
        if not target.is_file():
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        body = target.read_bytes()
        mime = ("model/gltf-binary" if name.endswith(".glb")
                else "application/json")
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_render(self, path):
        # Private reference renders (wrist card); same gate as meshes.
        import re as _re

        if not self.private_visuals:
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        name = path[len("/private/renders/"):]
        if not _re.fullmatch(r"[A-Za-z0-9_.-]+\.png", name):
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        root = pathlib.Path(__file__).resolve().parents[2]
        target = root / "assets" / "private" / "renders" / name
        if not target.is_file():
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        body = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_evidence(self, path):
        # IK-check PNG evidence (derived skeletons, not private meshes).
        import re as _re

        name = path[len("/evidence/"):]
        if not _re.fullmatch(r"(failures/)?[A-Za-z0-9_.-]+\.png", name):
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        root = pathlib.Path(__file__).resolve().parents[2]
        target = (root / "runs" / "stage5c" / "ik_check_multi"
                  / name)
        if not target.is_file():
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        body = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_static(self, path):
        rel = path[len("/static/"):]
        if not rel or ".." in rel or rel.startswith("/"):
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        target = STATIC_DIR / rel
        if not target.is_file():
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        mime, _ = mimetypes.guess_type(str(target))
        body = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type",
                         mime or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _dispatch(self, path, args):
        if path == "/api/robot":
            return {"ok": True, "robot": getattr(self, "robot",
                                                 "mrjos"),
                    "robots": list(ROBOT_IDS)}
        if path == "/api/robot/select":
            return select_robot(self, args.get("robot"))
        if path in TBOT_BLOCKED and getattr(self, "robot",
                                            "mrjos") == "tbot":
            raise ValueError("tbot is virtual-only: real brick "
                             "motion stays blocked (" + path + ")")
        if path == "/api/overview":
            return joint_overview(self)
        if path == "/api/model":
            return model_payload(self)
        if path == "/api/directions":
            return {"ok": True,
                    "directions": read_directions(
                        self.direction_path)}
        if path == "/api/direction":
            joint = args["joint"]
            verdict = args.get("verdict", "correct")
            if joint not in self.jog_map:
                raise KeyError("unknown joint: " + str(joint))
            if verdict not in ("correct", "reversed"):
                raise ValueError("verdict must be correct|reversed")
            if verdict == "reversed":
                entry = self.jog_map[joint]
                patch = {"ratio": -entry["ratio"]}
                entry = mini._patch_entry(entry, patch)
                changed = dict(self.jog_overrides.get(joint, {}))
                changed.update({k: entry[k] for k in patch})
                self.jog_map[joint] = entry
                self.jog_overrides[joint] = changed
                pathlib.Path(self.override_path).write_text(
                    json.dumps(self.jog_overrides, indent=1),
                    encoding="utf-8")
            directions = read_directions(self.direction_path)
            directions[joint] = "confirmed"
            write_directions(directions, self.direction_path)
            return {"joint": joint, "verdict": verdict,
                    "ratio": self.jog_map[joint]["ratio"],
                    "jog_map": self.jog_map,
                    "directions": directions}
        if path == "/api/jog":
            try:
                directions = read_directions(self.direction_path)
            except AttributeError:
                directions = {}
            if directions.get(args.get("joint"),
                              "unconfirmed") != "confirmed":
                cap = tunables(self)["unconfirmed_jog_deg"]
                delta = float(args.get("delta", mini.JOG_STEP_DEG))
                if abs(delta) > cap:
                    args = dict(args)
                    delta = cap if delta > 0 else -cap
                    args["delta"] = delta
            return super()._dispatch(path, args)
        if path == "/api/config":
            pkg = package_doc(self)
            return {"ok": True, "tunables": tunables(self),
                    "defaults": TUNABLE_DEFAULTS,
                    "package": {
                        "operating_margin_deg":
                            pkg.get("operating_margin_deg"),
                        "speed_caps": pkg.get("speed_caps"),
                    }}
        if path == "/api/config/set":
            key, value = args["key"], float(args["value"])
            if key not in TUNABLE_DEFAULTS:
                raise KeyError("unknown tunable: " + str(key))
            try:
                cfg = read_console_config(self.console_path)
            except AttributeError:
                cfg = {}
            cfg[key] = value
            path_ = (self.console_path or default_console_path())
            pathlib.Path(path_).write_text(
                json.dumps(cfg, indent=1), encoding="utf-8")
            return {"ok": True, "tunables": tunables(self)}
        if path == "/api/ranges":
            return ranges_payload(self)
        if path == "/api/range":
            return record_range(self, args["joint"], args["end"],
                                args["joint_deg"])
        if path == "/api/poses":
            pkg = package_doc(self)
            stored = read_json(runs_day_dir() / "report.json", {})
            last = {}
            for run in stored.get("poses", []) or []:
                last[run["pose"]] = run["error_deg"]
            return {"ok": True,
                    "poses": [{"id": p["id"],
                               "q_deg": p.get("q_deg", [])
                               or p.get("q", []),
                               "last_error_deg": last.get(p["id"])}
                              for p in pkg.get("safe_poses", [])]}
        if path == "/api/pose":
            return reproduce_pose(self, args["pose"])
        if path == "/api/ik":
            return solve_ik(self, args["pos"], args.get("mat"),
                            args.get("q_init"),
                            args.get("position_only", True))
        if path == "/api/reachable_target":
            return reachable_target(self, args.get("seed"))
        if path == "/api/table":
            if getattr(self, "robot", "mrjos") == "tbot":
                return {"ok": True, "stage": package_doc(self)["stage"]}
            return {"ok": True, "stage": stage_table()}
        if path == "/api/mode":
            # virtual: fake bricks (send allowed for demo).
            # connected: both bricks answer reads.
            bricks = {}
            for brick in self.transport.bricks:
                try:
                    self.transport.send(brick, "read")
                    bricks[brick] = True
                except Exception:
                    bricks[brick] = False
            connected = bool(bricks) and all(bricks.values())
            if getattr(self, "robot", "mrjos") == "tbot":
                # Virtual-only: never report brick connection.
                return {"ok": True, "robot": "tbot", "virtual": True,
                        "connected": False, "bricks": {}}
            return {"ok": True,
                    "virtual": bool(getattr(self, "virtual", False)),
                    "connected": connected, "bricks": bricks}
        if path == "/api/home_joint":
            return home_joint(self, str(args.get("joint", "")),
                              mark=bool(args.get("mark", False)))
        if path == "/api/homing_spec":
            return {"ok": True, "joints": homing_spec()}
        if path == "/api/calib/session":
            from src.console import calib_session as _cs
            return _cs.session_status(self)
        if path == "/api/calib/park":
            from src.console import calib_session as _cs
            res = move_joints(self, dict(_cs.PARK_Q_DEG),
                              speed=_cs.PARK_SPEED_DPS,
                              label=_cs.PARK_LABEL)
            record = latest_homing_record()
            saved = _cs.save_session({
                "boot": _cs.read_boot(self),
                "offsets": record.get("offsets", {}) or {},
                "park_q": dict(_cs.PARK_Q_DEG),
                "confirmed": False, "confirmed_at": None})
            return {"ok": True, "move": res, "saved_at": saved["saved_at"]}
        if path == "/api/calib/restore":
            from src.console import calib_session as _cs
            saved = _cs.load_session()
            now = _cs.read_boot(self)
            cont = _cs.boot_status(saved.get("boot", {}), now)
            if cont != "same" or not saved.get("offsets"):
                raise ValueError(
                    "need-recal: continuity=" + cont
                    + " (rebooted or no saved offsets; "
                    + "run one-button calibration)")
            for jid, off in (saved.get("offsets", {}) or {}).items():
                if jid in self.jog_map:
                    write_homing_result(jid, "homed", float(off),
                                        "restored from session "
                                        + str(saved.get("saved_at", "")))
            res = move_joints(self, dict(_cs.PARK_Q_DEG),
                              speed=_cs.PARK_SPEED_DPS,
                              label=_cs.PARK_LABEL)
            return {"ok": True, "move": res, "continuity": cont}
        if path == "/api/calib/check":
            from src.console import calib_session as _cs
            res = move_joints(self, dict(_cs.PARK_Q_DEG),
                              speed=_cs.PARK_SPEED_DPS,
                              label="park-check")
            dev = _check_deviation(self)
            return {"ok": True, "move": res, "deviation": dev}
        if path == "/api/calib/confirm":
            from src.console import calib_session as _cs
            dev = _check_deviation(self)
            if dev["max_abs_deg"] is None or \
                    dev["max_abs_deg"] > _cs.CONFIRM_TOL_DEG:
                raise ValueError(
                    "not-aligned: max dev "
                    + str(dev["max_abs_deg"])
                    + " deg exceeds "
                    + str(_cs.CONFIRM_TOL_DEG) + " deg")
            saved = _cs.load_session()
            record = latest_homing_record()
            saved = _cs.save_session({
                "boot": _cs.read_boot(self),
                "offsets": (saved.get("offsets")
                            or record.get("offsets", {}) or {}),
                "park_q": dict(_cs.PARK_Q_DEG),
                "confirmed": True,
                "confirmed_at": saved.get("confirmed_at"),
                "check": dev})
            # Stamp the confirmation time on the saved record.
            import datetime as _dt
            saved["confirmed_at"] = _dt.datetime.now().isoformat(
                timespec="seconds")
            _cs.save_session(saved)
            return {"ok": True, "deviation": dev,
                    "confirmed_at": saved["confirmed_at"]}
        if path == "/api/calib/flow":
            from src.console import calib_session as _cs
            return _calibration_flow(self)
        if path == "/api/twin/slots":
            from src.vla import twin as _twin
            return {"ok": True, "slots": _twin.load_slots(),
                    "ids": sorted(_twin.S.SLOT_IDS.values()),
                    "colours": sorted(_twin.S.COLORS)}
        if path == "/api/twin/slots/set":
            from src.vla import twin as _twin
            saved = _twin.save_slots(dict(args.get("slots", {}) or {}))
            return {"ok": True, "slots": saved}
        if path == "/api/twin/run":
            import base64 as _b64
            from src.console import calib_session as _cs
            from src.vla import twin as _twin
            out = _twin.run_twin(dict(args.get("slots", {}) or {}),
                                 str(args.get("target", "")),
                                 args.get("ckpt"))
            adopted = {r["joint"]: r["adopted"]
                       for r in ranges_payload(self)["joints"]}
            q = {j: float(v) for j, v in
                 zip(["J1", "J2", "J3", "J4", "J5", "J6"],
                     out["hold_q"])}
            refusal = check_pose_against_ranges(q, adopted)
            if refusal:
                raise ValueError("twin hold refused: " + refusal)
            return {"ok": True, "policy": out["policy"],
                    "target": out["target"], "slots": out["slots"],
                    "hold_q": q, "rec": out["rec"],
                    "png": _b64.b64encode(out["png"]).decode()}
        if path == "/api/twin/rest":
            from src.console import calib_session as _cs
            res = move_joints(self, dict(_cs.PARK_Q_DEG),
                              speed=_cs.PARK_SPEED_DPS,
                              label="twin-rest")
            return {"ok": True, "move": res}
        if path == "/api/verify_report":
            import pathlib as _pl
            root = _pl.Path(__file__).resolve().parents[2]
            rep = root / "runs" / "stage5c" / "ik_check_multi"
            try:
                report = json.loads(
                    (rep / "report.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                return {"ok": False,
                        "error": "no report yet"}
            try:
                table = (rep / "table.md").read_text(encoding="utf-8")
            except OSError:
                table = ""
            return {"ok": True, "report": report, "table": table}
        if path == "/api/mat_points":
            if not self.private_visuals:
                return {"ok": False,
                        "error": "mat needs --private-visuals"}
            import pathlib as _pl
            root = _pl.Path(__file__).resolve().parents[2]
            mat = json.loads(
                (root / "assets" / "private" / "mat_points.json")
                .read_text(encoding="utf-8"))
            try:
                from src.vla.scene import SLOT_IDS
                slots = [{"id": sid, "x": x, "y": y, "z": 0.0}
                         for (x, y), sid in sorted(
                             SLOT_IDS.items(), key=lambda kv: kv[1])]
            except Exception:
                slots = []
            return {"ok": True, "mat": mat, "slots": slots}
        if path == "/api/goto":
            q = {str(k): float(v) for k, v in
                 dict(args.get("q_deg", {})).items()}
            missing = [j for j in self.jog_map if j not in q]
            if missing:
                raise ValueError("missing joints: "
                                 + ",".join(missing))
            return move_joints(
                self, q,
                speed=float(args.get("speed_dps", 15.0)),
                label=str(args.get("label", "goto")))
        if path == "/api/preview":
            # Same pipeline as the sim check: joint targets ->
            # coupled motor targets + adopted-range verdict, no motion.
            from src.console.mini import coupled_motor_targets
            record = latest_homing_record()
            offsets = record.get("offsets", {}) or {}
            q = {str(k): float(v) for k, v in
                 dict(args.get("q_deg", {})).items()}
            missing = [j for j in self.jog_map if j not in q]
            if missing:
                raise ValueError("missing joints: "
                                 + ",".join(missing))
            motors = coupled_motor_targets(q, self.jog_map, offsets)
            adopted = {r["joint"]: r["adopted"]
                       for r in ranges_payload(self)["joints"]}
            refusal = check_pose_against_ranges(q, adopted)
            by_motor = {}
            for j, e in self.jog_map.items():
                by_motor.setdefault(
                    (e["brick"], e["port"]), []).append(j)
            return {"ok": True,
                    "motors": [{"motor": mp[0] + "/" + mp[1],
                                "joints": js,
                                "target": round(motors[mp], 2)}
                               for mp, js in by_motor.items()],
                    "range": "ok" if not refusal else refusal}
        if path == "/api/gt_frames":
            # Eval display only: GT skeleton for the same q.
            # Served only with --private-visuals; never used for
            # control, inference, or calibration.
            if not self.private_visuals:
                return {"ok": False,
                        "error": "gt overlay needs --private-visuals"}
            import json as _json
            from src.kin.fk import se3_exp as _se3
            import math as _math
            gt = _json.loads(pathlib.Path(
                "gt/kinematics_gt_v2.json").read_text(
                    encoding="utf-8"))
            gj = {j["id"]: j for j in gt["joints"]}
            order = [j["id"] for j in
                     sorted(gt["joints"],
                            key=lambda j: int(j["id"][1:]))]
            q = {str(k): float(v) for k, v in
                 dict(args.get("q_deg", {})).items()}
            T = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]

            def _mm(a, b):
                return [[sum(a[i][k] * b[k][j] for k in range(4))
                         for j in range(4)] for i in range(4)]

            pts = [[0.0, 0.0, 0.0]]
            for jid in order:
                g = gj[jid]
                w = [float(x) for x in g["axis"]]
                n = _math.sqrt(sum(x * x for x in w)) or 1.0
                w = [x / n for x in w]
                p = [float(x) for x in g["point_mm"]]
                v = [-(w[1] * p[2] - w[2] * p[1]),
                     -(w[2] * p[0] - w[0] * p[2]),
                     -(w[0] * p[1] - w[1] * p[0])]
                M = _se3(w, v, _math.radians(q.get(jid, 0.0)))
                T = _mm(T, M)
                pts.append([T[0][3], T[1][3], T[2][3]])
            e = [float(x) for x in
                 gt["end_effector"]["point_mm"]]
            pts.append([T[0][0] * e[0] + T[0][1] * e[1]
                        + T[0][2] * e[2] + T[0][3],
                        T[1][0] * e[0] + T[1][1] * e[1]
                        + T[1][2] * e[2] + T[1][3],
                        T[2][0] * e[0] + T[2][1] * e[1]
                        + T[2][2] * e[2] + T[2][3]])
            return {"ok": True, "joints": order, "points": pts}
        if path == "/api/visual":
            # Screen gets link id -> mesh url only; transforms stay
            # implicit (identity link frame posed by F_i in-page).
            available = ["primitives"]
            links = {}
            pack = visual_pack_for(self)
            if pack:
                from src.kin.model import chain_order

                n = len(chain_order(self.model_doc)) + 1
                for i in range(n):
                    lid = "D" + str(i)
                    links[lid] = ("/private/visual/"
                                   + pack + "/"
                                   + lid + ".glb")
                available.append("ldraw")
            return {"ok": True, "available": available, "links": links}
        return super()._dispatch(path, args)

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        from urllib.parse import urlparse

        parsed = urlparse(self.path).path
        if parsed in ("/", "/calibration", "/verify"):
            page_id = {"/": "main", "/calibration": "cal",
                       "/verify": "verify"}[parsed]
            body = PAGE.replace("<body>", '<body data-page="%s">'
                                % page_id, 1).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            # Page changes often during 5C; never cache it.
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif parsed.startswith("/static/"):
            self._serve_static(parsed)
        elif parsed.startswith("/private/visual/"):
            self._serve_private(parsed)
        elif parsed.startswith("/private/renders/"):
            self._serve_render(parsed)
        elif parsed.startswith("/evidence/"):
            self._serve_evidence(parsed)
        else:
            super().do_GET()


def create_server(transport, host="127.0.0.1", port=CONTROL_PORT,
                  jog_map=None, override_path=None,
                  direction_path=None, model_doc=None,
                  console_path=None, private_visuals=None,
                  virtual=False, robot="mrjos"):
    """Create (not start) the control console server.

    Always binds a loopback host; only 127.0.0.1 is supported.
    virtual=True means fake bricks (send flow allowed for demo).
    robot selects the model + package ("mrjos" default, "tbot"
    virtual-only from private stage-7 analysis).
    """
    if host != "127.0.0.1":
        raise ValueError("control console serves 127.0.0.1 only")
    if robot not in ROBOT_IDS:
        raise ValueError("unknown robot: " + str(robot))
    if override_path is None:
        override_path = mini.default_override_path()
    if direction_path is None:
        direction_path = default_direction_path()
    if console_path is None:
        console_path = default_console_path()
    if jog_map is None and robot == "tbot":
        jog_map = tbot_jog_map()
        overrides = {}
    elif jog_map is None:
        overrides = mini.read_jog_overrides(override_path)
        jog_map = mini.load_jog_map(overrides=overrides)
    else:
        overrides = {}
    if model_doc is None:
        from src.kin.model import load_model

        model_path, _ = robot_files(robot)
        model_doc = load_model(str(model_path))
    handler = type("ControlHandler", (_Handler,),
                   {"transport": transport, "jog_map": jog_map,
                    "jog_overrides": overrides,
                    "override_path": str(override_path),
                    "direction_path": str(direction_path),
                    "model_doc": model_doc,
                    "robot": robot,
                    "zeros": {},
                    "console_path": str(console_path),
                    "private_visuals": private_visuals,
                    "virtual": bool(virtual)})
    from http.server import ThreadingHTTPServer

    return ThreadingHTTPServer((host, port), handler)


class OfflineTransport:
    """Virtual-only: no bricks answer, every command fails cleanly."""

    bricks = []

    def _fail(self):
        from src.adapters.pc.mqtt_transport import TransportError

        raise TransportError("offline: no bricks")

    def send(self, brick, cmd, *args, **kwargs):
        self._fail()

    def move_motors(self, targets, speed=None):
        self._fail()

    def estop(self):
        self._fail()

    def clear_latch(self):
        self._fail()

    def close(self):
        pass


def build_transport(config, offline=False):
    if offline:
        return OfflineTransport()
    if config:
        from src.adapters.pc.mqtt_transport import MqttTransport

        return MqttTransport.from_config(config)
    from src.ev3.fake import build_mp_transport

    transport, _, _ = build_mp_transport(full=True)
    return transport


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="5C control console")
    parser.add_argument("--port", type=int, default=CONTROL_PORT)
    parser.add_argument("--config", default=None,
                        help="hardware yaml for real bricks "
                             "(default: fake bricks)")
    parser.add_argument("--private-visuals", default=None, metavar="ROBOT",
                        help="serve git-ignored LDraw meshes from "
                             "assets/private/visual_pack/ROBOT "
                             "(default: auto-detect the robot's pack)")
    parser.add_argument("--offline", action="store_true",
                        help="virtual-only: no bricks answer "
                             "(body follows IK, send hidden)")
    parser.add_argument("--robot", default="mrjos",
                        choices=list(ROBOT_IDS),
                        help="console robot (tbot is virtual-only)")
    args = parser.parse_args(argv)
    transport = build_transport(args.config, offline=args.offline)
    server = create_server(transport, port=args.port,
                           private_visuals=args.private_visuals,
                           virtual=args.offline or args.config is None,
                           robot=args.robot)
    print("control console on http://127.0.0.1:" + str(args.port)
          + " (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()


PAGE = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>5C Control Console</title>
<style>
body { font-family: sans-serif; margin: 0; background: #111; color: #eee; }
#bar { position: sticky; top: 0; background: #222; padding: 8px 12px;
  display: flex; gap: 16px; align-items: center; z-index: 10; }
#bar .ok { color: #7f7; } #bar .bad { color: #f77; } #bar .na { color: #999; }
#estop { background: #c00; color: #fff; font-weight: bold; padding: 6px 14px; }
main { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; padding: 12px; }
section { background: #1a1a1a; padding: 10px; border-radius: 6px; }
table { border-collapse: collapse; width: 100%; font-size: 14px; }
td, th { border: 1px solid #444; padding: 4px 6px; text-align: center; }
.unconf { color: #fb0; font-weight: bold; }
button { margin: 2px; }
#err { color: #f77; white-space: pre-wrap; }
.wiz { border: 1px solid #555; padding: 8px; margin-top: 8px; }
.shell { color: #999; border: 1px dashed #555; padding: 8px; margin-top: 8px; }
/* Main page: 3D canvas fills the screen under the bar; the panels
   float above it. Other pages keep the plain grid layout. */
body[data-page="main"] { overflow: hidden; }
body[data-page="main"] main { display: block; padding: 0; }
#viewwrap { display: none; }
body[data-page="main"] #viewwrap { display: block; position: fixed;
  left: 0; right: 0; bottom: 0; background: #565d66; }
body[data-page="main"] #view3d { position: absolute; inset: 0;
  width: 100%; height: 100%; display: block; }
.fpanel { position: fixed; z-index: 5; background: rgba(20, 22, 26, 0.88);
  border: 1px solid #444; padding: 8px 10px; font-size: 13px;
  max-height: calc(100vh - 140px); overflow: auto; }
.fpanel h3 { font-size: 13px; margin: 8px 0 4px; }
.fpanel summary { cursor: pointer; font-size: 13px; }
#panel-right { right: 10px; width: 320px; }
#panel-left { left: 10px; bottom: 10px; width: 348px;
  max-height: 46vh; }
#viewmenu { position: absolute; top: 8px; left: 8px; z-index: 6;
  background: rgba(20, 22, 26, 0.88); border: 1px solid #444;
  padding: 6px 8px; font-size: 12px; max-width: 46vw; }
#viewmenu label { margin-right: 8px; white-space: nowrap; }
</style>
<script type="importmap">
{"imports": {
  "three": "/static/three/build/three.module.js",
  "three/addons/": "/static/three/examples/jsm/"}}
</script>
</head>
<body>
<div id="bar">
  <strong>5C Console</strong>
  <a href="/">메인</a> <a href="/calibration">보정</a>
  <a href="/verify">검증</a>
  <span id="mode">모드: …</span>
  <span id="st-left">좌: …</span><span id="st-right">우: …</span>
  <span id="st-latch">래치: …</span><span id="st-hb">하트비트: …</span>
  <span id="homedots">호밍: …</span>
  <label>로봇 <select id="robot"><option>로딩…</option></select></label>
  <a id="cal-link" href="/calibration"
     style="display:none">보정 페이지에서 호밍하세요</a>
  <button id="estop">E-STOP</button>
  <button id="btn-clear">래치 해제</button>
</div>
<main>
<div id="viewwrap" data-sec="main"><canvas id="view3d"></canvas>
<div id="glerr"></div>
<div id="viewmenu">표시:
<label><input type="checkbox" id="vm-mat" checked>매트</label>
<label><input type="checkbox" id="vm-tcp" checked>TCP</label>
<label><input type="checkbox" id="vm-gizmo" checked>기즈모</label>
<label><input type="checkbox" id="vm-line" checked>TCP→목표</label>
<label><input type="checkbox" id="vm-reach">도달범위</label>
<label><input type="checkbox" id="vm-gt">GT</label>
<label><input type="checkbox" id="vm-prev">이전자세</label>
<label><input type="checkbox" id="vm-real" checked>실측겹침</label>
<label><input type="checkbox" id="vm-dir">손목방향</label>
<br>모델: <select id="vsrc">
<option value="primitives">기본 도형</option>
<option value="ldraw">LDraw 메시(비공개)</option>
</select>
<button id="fitview">화면 맞춤</button>
<span id="vmsg"></span>
</div>
</div>
<div id="panel-right" class="fpanel" data-sec="main">
<details open><summary>목표 (<span id="vsrc-name">기본 도형</span>)</summary>
<div>
<button id="ik-t">이동</button><button id="ik-r">회전</button>
<button id="ik-x">X</button><button id="ik-y">Y</button>
<button id="ik-z">Z</button>
<button id="ik-space">world/local</button>
<button id="ik-reset">타겟→현재EE</button><br>
매트 <select id="mat-point"><option>로딩…</option></select>
<button id="mat-go">매트 목표</button>
<div id="mat-btns"></div>
좌표(mm) X<input id="xyz-x" size="6"> Y<input id="xyz-y" size="6">
Z<input id="xyz-z" size="6"> <button id="xyz-go">목표로</button><br>
<label><input type="checkbox" id="grip-down">
집게가 아래를 향함 (위치+집게 방향)</label>
<button id="ik-tcp">기즈모를 TCP로</button>
<button id="ik-reach">닿는 목표</button>
</div>
</details>
<details open><summary>실행</summary>
<div id="ik-stat">수렴 — / 오차 — / 제한 — / 목표(베이스) —</div>
<div id="send-row">미리보기 확인 후 <button id="send-go" disabled>로봇으로 보내기</button>
<button id="send-stop">정지</button> <span id="send-msg">대기 중</span></div>
</details>
<details open><summary>디지털 트윈 (실물 블록 → 가상 이미지 → VLA)</summary>
<div class="wiz" id="wiztwin">
<div>실물 블록을 슬롯 S1~S6에만 놓고, 색을 아래에 입력. 실물 카메라는 쓰지 않음.</div>
<div id="twin-slots">로딩…</div>
<div>목표 <select id="twin-target"></select>
<button id="twin-save">색 저장</button>
<button id="twin-run">트윈 실행</button></div>
<div><img id="twin-img" style="max-width:100%;display:none" alt="가상 이미지"></div>
<div id="twin-msg">대기 중</div>
<div><button id="twin-send" disabled>유지 자세로 보내기 (저속)</button>
<button id="twin-rest">휴식 자세로</button></div>
</div>
</details>
<details><summary>데모 순서</summary>
<div class="wiz" id="wizdemo">
<div>호밍 확인 → 안전자세 2개 → 매트 점(아래 번호) → 원위치. 느리게, 한 단계씩 확인.</div>
매트 점 <input id="demo-points" size="10" value="1,5,9">
<button id="demo-start">데모 순서 시작</button>
<div id="demo-msg"></div>
</div>
</details>
</div>
<div id="panel-left" class="fpanel" data-sec="main">
<details><summary>관절·모터·IK</summary>
<div><label><input type="checkbox" id="follow-live" checked>
실시간 관절값 반영</label></div>
<div id="joints">로딩…</div>
<div id="ik-msg">주황 구슬을 드래그하면 뼈대가 따라옵니다</div>
<div id="ik-live">TCP — / 목표 — / 거리 —</div>
<div id="preview-dist">미리보기 — / 목표 — / 거리 —</div>
</details>
<details><summary>로그</summary><div id="log"></div></details>
</div>
<section data-sec="cal"><h3>세션 보정 (호밍 반복 생략용)</h3>
<div class="wiz" id="wizsess">
<div id="sess-stat">로딩…</div>
<div><button id="sess-flow">한 번에 보정</button>
<button id="sess-park">주차 (저속 이동+저장)</button>
<button id="sess-restore">주차 자세에서 복원</button></div>
<div><button id="sess-check">확인 자세로 이동</button>
<button id="sess-ok">맞음 (보정 확정)</button></div>
<div id="sess-msg"></div>
<div id="flow-steps"></div>
</div>
</section>
<section data-sec="cal"><h3>호밍</h3>
<div class="wiz" id="wizhome">
<div>관절별 호밍. J5/J6은 표시 맞춤(현재 위치를 0으로 기록). 손목 기준 그림:</div>
<div><img id="wrist-img" style="max-width:100%;display:none" alt="손목 기준"></div>
<div id="home-rows">로딩…</div>
<div id="home-msg"></div>
<div><a href="/">메인으로 돌아가기</a></div>
</div>
</section>
<section data-sec="cal"><h3>M4 방향 마법사</h3>
<div class="wiz" id="wiz4">
관절 <select id="w4-joint"></select>
<button id="w4-try">+5&deg; 시도</button>
<button id="w4-ok">맞음</button>
<button id="w4-rev">반대</button>
<div id="w4-msg"></div>
</div>
</section>

<section data-sec="cal"><h3>M5 범위 마법사</h3>
<div class="wiz" id="wiz5">
<div id="ranges">로딩…</div>
</div>
</section>
<section data-sec="cal"><h3>M6 자세 재현</h3>
<div class="wiz" id="wiz6">
자세 <select id="w6-pose"></select>
<button id="w6-run">재현</button>
<div id="w6-msg"></div>
<div id="poses">로딩…</div>
</div>
</section>
<section data-sec="cal"><h3>포트 진단</h3>
<div class="wiz" id="wizdiag"><div id="diag">로딩…</div></div>
</section>
<section data-sec="cal"><h3>설정(경계값)</h3>
<div class="wiz" id="wcfg"><div id="cfgs">로딩…</div></div>
</section>
<section data-sec="verify"><h3>IK 시험 결과</h3>
<div class="wiz" id="wizverify">
<div id="verify-sum">로딩…</div>
<div id="verify-table"></div>
<div id="verify-shots"></div>
<div id="verify-gt">GT 뼈대 비교는 --private-visuals일 때만 표시됩니다.</div>
</div>
</section>
</main>
<div id="err"></div>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from
  "three/addons/controls/TransformControls.js";

window.addEventListener("error", (e) => {
  document.getElementById("glerr").textContent = "오류: " + e.message;
});

async function post(path, args) {
  const r = await fetch(path, {method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(args || {})});
  return r.json();
}
let MODEL = null;
// Stage frame (desk frame, mm): stage = model - C, table = z 0,
// base bottom center = origin. One shared definition from
// /api/table (config/calibration.local.yaml stage).
let STAGE = {table_z_mm: -220.8, center_x_mm: 34.0,
  center_y_mm: 79.5, reach_r_mm: 335.3,
  shoulder_model_mm: [21.8, 79.7, -53.0]};
const qdeg = {};
// Send flow: pendingQ = last IK result awaiting send; previewOk =
// preview drawn + /api/preview range ok. lastOv = latest overview.
let pendingQ = null, previewOk = false, previewRange = "";
let lastOv = null, VIRT = false, ROBOT = "mrjos";
async function initRobot() {
  const sel = document.getElementById("robot");
  if (!sel) return;
  try {
    const r = await post("/api/robot", {});
    if (!r.ok) return;
    ROBOT = r.robot;
    sel.innerHTML = "";
    for (const id of r.robots) {
      const o = document.createElement("option");
      o.value = id;
      o.textContent = id === "tbot" ? "T-Bot (가상)" : "MR JOS";
      if (id === r.robot) o.selected = true;
      sel.appendChild(o);
    }
    sel.onchange = async () => {
      const rr = await post("/api/robot/select", {robot: sel.value});
      if (rr.ok) location.reload();
      else {
        alert("로봇 전환 실패: " + (rr.error || "?"));
        sel.value = ROBOT;
      }
    };
  } catch (err) { /* keep default selection */ }
}
const PAGE_ID = (document.body.dataset &&
  document.body.dataset.page) || "main";
const SEGS = [], NODES = []; // limb capsules, joint spheres

async function initModel() {
  const m = await post("/api/model", {});
  MODEL = m;
  try {
    const t = await post("/api/table", {});
    if (t.ok && t.stage) STAGE = t.stage;
  } catch (err) {
    log("테이블 설정 로드 실패, 기본값 사용");
  }
  // 3D scene lives on the main page only; cal/verify share
  // the same template but skip the WebGL build (and its tick).
  if (PAGE_ID !== "main" || !__realById("view3d")) return;
  const canvas = document.getElementById("view3d");
  const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
  renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
  const scene = new THREE.Scene();
  // Mid-gray studio backdrop: the build is mostly black bricks
  // (LDraw colour 0), which vanish on a near-black background.
  scene.background = new THREE.Color(0x565d66);
  const camera = new THREE.PerspectiveCamera(50,
    canvas.clientWidth / canvas.clientHeight, 1, 20000);
  camera.position.set(700, 500, 800);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 150, 0);
  scene.add(new THREE.AmbientLight(0xffffff, 0.9));
  scene.add(new THREE.HemisphereLight(0xffffff, 0x3a3f45, 0.6));
  const dir = new THREE.DirectionalLight(0xffffff, 2.0);
  dir.position.set(500, 900, 600);
  scene.add(dir);
  const fill = new THREE.DirectionalLight(0xdfe8ff, 0.7);
  fill.position.set(-600, 400, -500);
  scene.add(fill);
  scene.add(new THREE.GridHelper(2000, 20, 0x222222, 0x3d3d3d));
  // Model frame is Z-up (see models/robot_inferred.json "frame");
  // three.js is Y-up, so the arm lives under a -90 deg X rotation.
  // Grounding + centering in the same place: stage = model - C,
  // world = RX(-90) * stage, hence arm.position = -RX*C.
  // Primitives and LDraw meshes both live under arm: same transform.
  const arm = new THREE.Group();
  arm.rotation.x = -Math.PI / 2;
  arm.position.set(-STAGE.center_x_mm, -STAGE.table_z_mm,
    STAGE.center_y_mm);
  scene.add(arm);
  // Limb radii from link envelopes: scaled geometric mean of the
  // two smallest box sides (tapered limbs, like the real build).
  // Capped so the huge base envelopes do not become giant balls.
  const RADII = [];
  for (const env of m.envelopes) {
    if (!env.carrier) continue;
    const s = [...env.size_mm].sort((a, b) => a - b);
    RADII.push({joint: env.carrier,
      r: Math.min(0.28 * Math.sqrt(s[0] * s[1]), 40)});
  }
  const segRadius = (i) => {
    const f = RADII.find((e) => e.joint === m.joints[i].id);
    return f ? f.r : Math.min(0.28 * 40, 40);
  };
  const limbMats = [0x9fb3c8, 0x7e97b5].map((c) =>
    new THREE.MeshStandardMaterial({color: c, roughness: 0.5,
      metalness: 0.35}));
  const jointMat = new THREE.MeshStandardMaterial({color: 0x3a7bd5,
    roughness: 0.45, metalness: 0.3});
  // Initial pose: runs/stage5/poses10.json pose01, the io render
  // reference (assembled robot, not the flat zero pose).
  const POSE01 = {J1: 6.595, J2: 138.148, J3: 92.007,
                  J4: 79.832, J5: -63.112, J6: -45.08};
  // Rigid links: link i is fixed in its own PoE frame F_i, so its
  // capsule endpoints (joint anchors at the zero pose) are constant
  // in that frame and the mesh just takes F_i each tick. The anchor
  // p_i lies on screw axis i, hence F_i^-1 F_{i-1} p_i == p_i.
  const q0 = m.joints.map((j) => qNative(j.id, POSE01[j.id] || 0));
  const F0pose = fkine(q0);
  const e0 = MODEL.ee_mm;
  for (let i = 0; i < m.joints.length; i++) {
    const a = m.joints[i].point;
    const b = (i < m.joints.length - 1) ? m.joints[i + 1].point : e0;
    const len = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    const r = segRadius(i);
    const mesh = new THREE.Mesh(
      new THREE.CapsuleGeometry(r, len, 4, 12), limbMats[i % 2]);
    mesh.matrixAutoUpdate = false;
    // Capsule axis is +Y; rotate local +Y onto (b - a).
    const dir = new THREE.Vector3(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    const mid = new THREE.Vector3((a[0] + b[0]) / 2,
      (a[1] + b[1]) / 2, (a[2] + b[2]) / 2);
    mesh.matrix.compose(mid,
      new THREE.Quaternion().setFromUnitVectors(
        new THREE.Vector3(0, 1, 0), dir.normalize()),
      new THREE.Vector3(1, 1, 1));
    arm.add(mesh);
    SEGS.push({mesh, local: mesh.matrix.clone()});
  }
  // Gizmo starts at the gripper-tip TCP, not the wrist.
  const tip0 = MODEL.tip_mm;
  const p0ee = new THREE.Vector3(tip0[0], tip0[1], tip0[2])
    .applyMatrix4(F0pose[F0pose.length - 1]);
  for (let i = 0; i < m.joints.length; i++) {
    const r = segRadius(i) + 6;
    const ball = new THREE.Mesh(
      new THREE.SphereGeometry(r, 16, 12), jointMat);
    ball.matrixAutoUpdate = false;
    arm.add(ball);
    NODES.push(ball);
  }
  // End-effector orientation triad (40 mm arms), posed by F_6.
  const eeAxes = new THREE.Group();
  const axCols = [0xd33a3a, 0x3ad57b, 0x3a7bd5];
  for (let a = 0; a < 3; a++) {
    const geo = new THREE.CylinderGeometry(3, 3, 40, 8);
    geo.translate(0, 20, 0);
    const ax = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
      color: axCols[a]}));
    if (a === 0) ax.rotation.z = -Math.PI / 2;
    if (a === 2) ax.rotation.x = Math.PI / 2;
    eeAxes.add(ax);
  }
  eeAxes.matrixAutoUpdate = false;
  arm.add(eeAxes);
  const baseEnv = m.envelopes.find((e) => !e.carrier);
  var baseMesh = null;
  if (baseEnv) {
    baseMesh = new THREE.Mesh(
      new THREE.BoxGeometry(baseEnv.size_mm[0], baseEnv.size_mm[1],
                            baseEnv.size_mm[2]),
      new THREE.MeshStandardMaterial({color: 0x3a7bd5, roughness: 0.6,
        metalness: 0.1, transparent: true, opacity: 0.9}));
    baseMesh.matrixAutoUpdate = false;
    baseMesh.matrix.makeTranslation(baseEnv.center_mm[0],
      baseEnv.center_mm[1], baseEnv.center_mm[2]);
    arm.add(baseMesh);
  }
  const jsel = document.getElementById("w4-joint");
  for (const j of m.joints) {
    const o = document.createElement("option");
    o.textContent = j.id; jsel.appendChild(o);
    qdeg[j.id] = (j.id in POSE01) ? POSE01[j.id] : 0;
  }
  // IK target gizmo (world frame): drag it, the skeleton follows.
  const tc = new TransformControls(camera, renderer.domElement);
  tc.setMode("translate");
  scene.add(tc);
  const target = new THREE.Group();
  const marker = new THREE.Mesh(new THREE.SphereGeometry(10, 16, 12),
    new THREE.MeshBasicMaterial({color: 0xff8800, depthTest: false,
      transparent: true, opacity: 0.9}));
  marker.renderOrder = 999;
  target.add(marker);
  arm.updateMatrixWorld(true);
  target.position.copy(arm.localToWorld(p0ee.clone()));
  scene.add(target);
  tc.attach(target);
  tc.addEventListener("dragging-changed", (e) => {
    controls.enabled = !e.value;
    if (!e.value) solveIK();
  });
  // Never drag below the table: clamp live in the stage frame
  // (world y == stage z, since arm carries the RX(-90) rotation).
  tc.addEventListener("change", () => {
    if (target.position.y < 0) target.position.y = 0;
  });
  // Reachable envelope: translucent sphere at the shoulder (J2),
  // radius = max |TCP - shoulder| over adopted-range FK samples.
  // Same stage/world transform as everything else (arm child).
  const sh = new THREE.Vector3(STAGE.shoulder_model_mm[0],
    STAGE.shoulder_model_mm[1], STAGE.shoulder_model_mm[2]);
  const reach = new THREE.Mesh(
    new THREE.SphereGeometry(STAGE.reach_r_mm, 24, 16),
    new THREE.MeshBasicMaterial({color: 0x3ad57b, transparent: true,
      opacity: 0.07, wireframe: true, depthWrite: false}));
  reach.matrixAutoUpdate = false;
  reach.matrix.makeTranslation(sh.x, sh.y, sh.z);
  arm.add(reach);
  // Mat points live on the same stage frame (arm children).
  const matDots = new THREE.Group();
  matDots.matrixAutoUpdate = false;
  matDots.matrix.identity();
  arm.add(matDots);
  // Verification overlay (arm frame = robot base, mm):
  // TCP dot, TCP-target line, pre-drag pose, GT skeleton.
  const tcpDot = new THREE.Mesh(new THREE.SphereGeometry(8, 12, 10),
    new THREE.MeshBasicMaterial({color: 0x22ff44, depthTest: false,
      transparent: true, opacity: 0.95}));
  tcpDot.renderOrder = 998;
  tcpDot.matrixAutoUpdate = false;
  arm.add(tcpDot);
  const tgtGeo = new THREE.BufferGeometry();
  tgtGeo.setAttribute("position",
    new THREE.BufferAttribute(new Float32Array(6), 3));
  const tgtLine = new THREE.Line(tgtGeo,
    new THREE.LineBasicMaterial({color: 0xff2222}));
  tgtLine.matrixAutoUpdate = false;
  tgtLine.matrix.identity();
  tgtLine.frustumCulled = false;
  arm.add(tgtLine);
  // Gripper direction: wrist -> tip, in arm (model) coords.
  const dirGeo = new THREE.BufferGeometry();
  dirGeo.setAttribute("position",
    new THREE.BufferAttribute(new Float32Array(6), 3));
  const dirLine = new THREE.Line(dirGeo,
    new THREE.LineBasicMaterial({color: 0x22ff44}));
  dirLine.matrixAutoUpdate = false;
  dirLine.matrix.identity();
  dirLine.frustumCulled = false;
  arm.add(dirLine);
  function skelLine(color, opacity) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position",
      new THREE.BufferAttribute(new Float32Array(8 * 3), 3));
    const l = new THREE.Line(g,
      new THREE.LineBasicMaterial({color, transparent: true,
        opacity}));
    l.matrixAutoUpdate = false;
    l.matrix.identity();
    l.frustumCulled = false;
    l.visible = false;
    arm.add(l);
    return l;
  }
  const prevLine = skelLine(0x999999, 0.4);
  const gtLine = skelLine(0xff8800, 0.9);
  // Encoder actual overlay (magenta). The IK preview is a
  // translucent ghost of the same body model (see GHOST).
  const actualLine = skelLine(0xff44ff, 0.9);
  function setSkel(line, pts) {
    const a = line.geometry.attributes.position;
    for (let i = 0; i < 8 && i < pts.length; i++)
      a.setXYZ(i, pts[i].x, pts[i].y, pts[i].z);
    a.needsUpdate = true;
    line.visible = true;
  }
  buildGhostPrims(arm, baseMesh);
  reach.visible = document.getElementById("vm-reach").checked;
  dirLine.visible = document.getElementById("vm-dir").checked;
  window._scene = {renderer, scene, camera, controls, arm, tc, target,
                   marker, eeAxes, baseMesh, tcpDot, tgtLine, dirLine,
                   reach, prevLine, gtLine, actualLine, matDots,
                   setSkel, ghostOn: () => ghostOn,
                   targetArm: p0ee.clone(),
                   prevQ: null, gtAxes: null, rangesAdopted: null,
                   grounded: false};
  requestAnimationFrame(tick);
}
// Ghost preview: the same body, translucent, posed by the IK result.
// ghostOn = connected mode with a pending solution.
let ghostOn = false;
const GHOST = {prims: [], nodes: [], base: null, ldraw: []};
function applyGhost() {
  const prim = VSRC === "primitives";
  const showP = ghostOn && prim;
  const showL = ghostOn && !prim && MESHES.ready;
  for (const g of GHOST.prims) g.mesh.visible = showP;
  for (const g of GHOST.nodes) g.mesh.visible = showP;
  if (GHOST.base) GHOST.base.visible = showP;
  for (const g of GHOST.ldraw) if (g) g.visible = showL;
}
function buildGhostPrims(host, base) {
  if (!host) return;
  if (!GHOST.mat) {
    GHOST.mat = new THREE.MeshStandardMaterial({color: 0x7fb3e8,
      roughness: 0.6, metalness: 0.1, transparent: true,
      opacity: 0.4, depthWrite: false});
  }
  if (GHOST.prims.length) { applyGhost(); return; }
  for (const s of SEGS) {
    const m = new THREE.Mesh(s.mesh.geometry, GHOST.mat);
    m.matrixAutoUpdate = false;
    m.visible = false;
    host.add(m);
    GHOST.prims.push({mesh: m, local: s.local});
  }
  for (const b of NODES) {
    const m = new THREE.Mesh(b.geometry, GHOST.mat);
    m.matrixAutoUpdate = false;
    m.visible = false;
    host.add(m);
    GHOST.nodes.push({mesh: m});
  }
  // Base envelope is static; mirror its transform once.
  if (base) {
    const m = new THREE.Mesh(base.geometry, GHOST.mat);
    m.matrixAutoUpdate = false;
    m.matrix.copy(base.matrix);
    m.visible = false;
    host.add(m);
    GHOST.base = m;
  }
  applyGhost();
}
function buildGhostLdraw() {
  if (!MESHES.ready || GHOST.ldraw.length) { applyGhost(); return; }
  const st = window._scene;
  if (!st) return;
  if (!GHOST.mat) {
    GHOST.mat = new THREE.MeshStandardMaterial({color: 0x7fb3e8,
      roughness: 0.6, metalness: 0.1, transparent: true,
      opacity: 0.4, depthWrite: false});
  }
  GHOST.ldraw = [];
  for (let i = 0; i < MESHES.groups.length; i++) {
    const g = MESHES.groups[i];
    if (!g) { GHOST.ldraw.push(null); continue; }
    const c = g.clone(true);
    c.traverse((o) => {
      if (o.isMesh) {
        o.material = GHOST.mat;
        o.matrixAutoUpdate = false;
      }
    });
    c.visible = false;
    st.arm.add(c);
    GHOST.ldraw.push(c);
  }
  applyGhost();
}
let VSRC = "primitives";
const MESHES = {ready: false, groups: [], urls: {}};
async function refreshVisual() {
  const r = await post("/api/visual", {});
  const sel = document.getElementById("vsrc");
  const ldraw = r.ok && (r.available || []).includes("ldraw");
  sel.options[1].disabled = !ldraw;
  if (ldraw) MESHES.urls = r.links;
  document.getElementById("vmsg").textContent = ldraw ?
    "" : "LDraw 메시 없음(기본 도형 유지)";
}
function applyVis() {
  const prim = VSRC === "primitives";
  for (const g of SEGS) g.mesh.visible = prim;
  for (const b of NODES) b.visible = prim;
  const st = window._scene;
  if (st.baseMesh) st.baseMesh.visible = prim;
  if (st.eeAxes) st.eeAxes.visible = prim;
  for (const g of MESHES.groups) g.visible = !prim;
  document.getElementById("vsrc-name").textContent =
    prim ? "기본 도형" : "LDraw 메시";
  applyGhost();
}
async function loadMeshes() {
  if (MESHES.ready || MESHES.loading) return;
  MESHES.loading = true;
  let GLTFLoader;
  try {
    ({GLTFLoader} = await import("three/addons/loaders/GLTFLoader.js"));
  } catch (err) {
    document.getElementById("vmsg").textContent =
      "GLTFLoader 없음 (기본 도형 유지)";
    document.getElementById("vsrc").value = "primitives";
    VSRC = "primitives";
    MESHES.loading = false;
    applyVis();
    return;
  }
  const loader = new GLTFLoader();
  // Link ids come from /api/visual (D0..D6 MR JOS, D0..D3 T-Bot):
  // never a hardcoded 7-link list.
  const ids = Object.keys(MESHES.urls);
  if (!ids.length) {
    document.getElementById("vmsg").textContent =
      "LDraw 링크 없음 (기본 도형 유지)";
    document.getElementById("vsrc").value = "primitives";
    VSRC = "primitives";
    MESHES.loading = false;
    applyVis();
    return;
  }
  let done = 0;
  for (const lid of ids) {
    loader.load(MESHES.urls[lid], (gltf) => {
      gltf.scene.matrixAutoUpdate = false;
      gltf.scene.visible = false;
      window._scene.arm.add(gltf.scene);
      MESHES.groups[parseInt(lid.slice(1), 10)] = gltf.scene;
      if (++done === ids.length) {
        MESHES.ready = true;
        window._scene.meshGroups = MESHES.groups;
        buildGhostLdraw();
        applyVis();
        log("LDraw 메시 로드 완료");
      }
    }, undefined, (err) => {
      document.getElementById("vmsg").textContent =
        "메시 로드 실패: " + lid + " (기본 도형 유지)";
      document.getElementById("vsrc").value = "primitives";
      VSRC = "primitives";
      applyVis();
    });
  }
}
document.getElementById("vsrc").onchange = (e) => {
  VSRC = e.target.value;
  if (VSRC === "ldraw") loadMeshes();
  applyVis();
};
function fitView() {
  const st = window._scene;
  if (!st || !SEGS.length) return;
  st.scene.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(st.arm);
  // Grounding is fixed by arm.position (stage frame); fitView only
  // aims the camera, never moves the arm.
  st.scene.updateMatrixWorld(true);
  const box2 = new THREE.Box3().setFromObject(st.arm);
  const c2 = box2.getCenter(new THREE.Vector3());
  const s2 = box2.getSize(new THREE.Vector3());
  st.controls.target.copy(c2);
  const dist = Math.max(s2.x, s2.y, s2.z) * 1.6 + 150;
  const dir = st.camera.position.clone().sub(st.controls.target);
  if (dir.lengthSq() < 1e-6) dir.set(1, 0.7, 1);
  dir.normalize().multiplyScalar(dist);
  st.camera.position.copy(st.controls.target).add(dir);
  st.camera.near = Math.max(1, dist / 1000);
  st.camera.far = dist * 20 + 20000;
  st.camera.updateProjectionMatrix();
}
document.getElementById("fitview").onclick = () => fitView();
// Display menu (top-right of the canvas): mat dots, TCP dot,
// target gizmo and the thin TCP-target line are on; reach envelope,
// GT skeleton, pre-drag pose and wrist direction are debug (off).
function fitLayout() {
  if (PAGE_ID !== "main") return;
  const bar = document.getElementById("bar");
  const wrap = document.getElementById("viewwrap");
  if (bar && wrap) wrap.style.top = bar.offsetHeight + "px";
  const st = window._scene;
  const canvas = document.getElementById("view3d");
  if (st && canvas && canvas.clientWidth > 0) {
    st.renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    st.camera.aspect = canvas.clientWidth / canvas.clientHeight;
    st.camera.updateProjectionMatrix();
  }
}
window.addEventListener("resize", fitLayout);
function bindVis(id, fn) {
  const el = document.getElementById(id);
  if (el) el.onchange = () => fn(el.checked);
}
bindVis("vm-mat", (v) => {
  const st = window._scene; if (st) st.matDots.visible = v;
});
bindVis("vm-tcp", (v) => {
  const st = window._scene; if (st) st.tcpDot.visible = v;
});
bindVis("vm-gizmo", (v) => {
  const st = window._scene;
  if (st) { st.target.visible = v; st.tc.visible = v; }
});
bindVis("vm-line", (v) => {
  const st = window._scene; if (st) st.tgtLine.visible = v;
});
bindVis("vm-reach", (v) => {
  const st = window._scene; if (st) st.reach.visible = v;
});
bindVis("vm-dir", (v) => {
  const st = window._scene; if (st) st.dirLine.visible = v;
});
function skelNodes(F) {
  // Skeleton nodes: base origin, joint-frame origins, EE point.
  const pts = [new THREE.Vector3(0, 0, 0)];
  for (const M of F) pts.push(new THREE.Vector3().setFromMatrixPosition(M));
  const e = MODEL.ee_mm;
  pts.push(new THREE.Vector3(e[0], e[1], e[2])
    .applyMatrix4(F[F.length - 1]));
  return pts;
}
function se3(w, v, th) {
  // Same operations as src/kin/fk.py se3_exp (Rodrigues):
  // R = I + sin K + (1-cos) K^2, t = (I-R)(w x v) + w(w.v)th.
  // K is built from the raw axis (no unit normalization), like Python.
  const c = Math.cos(th), s = Math.sin(th), C = 1 - c;
  const [x, y, z] = w;
  const K = [[0, -z, y], [z, 0, -x], [-y, x, 0]];
  const K2 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  const R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      K2[i][j] = K[i][0] * K[0][j] + K[i][1] * K[1][j] +
                 K[i][2] * K[2][j];
      R[i][j] = (i === j ? 1 : 0) + s * K[i][j] + C * K2[i][j];
    }
  }
  const wxv = [y*v[2] - z*v[1], z*v[0] - x*v[2], x*v[1] - y*v[0]];
  const wtv = x*v[0] + y*v[1] + z*v[2];
  const t = [0, 0, 0];
  for (let i = 0; i < 3; i++) {
    t[i] = ((i === 0 ? 1 : 0) - R[i][0]) * wxv[0] +
           ((i === 1 ? 1 : 0) - R[i][1]) * wxv[1] +
           ((i === 2 ? 1 : 0) - R[i][2]) * wxv[2] +
           [x, y, z][i] * wtv * th;
  }
  const Rm = new THREE.Matrix4().set(
    R[0][0], R[0][1], R[0][2], 0,
    R[1][0], R[1][1], R[1][2], 0,
    R[2][0], R[2][1], R[2][2], 0,
    0, 0, 0, 1);
  return {R: Rm, t: new THREE.Vector3(t[0], t[1], t[2])};
}
function uOf(id) {
  // Native display unit per joint (mm prismatic, deg revolute).
  const js = (typeof MODEL !== "undefined" && MODEL && MODEL.joints) || [];
  const j = js.find((k) => k.id === id);
  return (j && j.unit) || "deg";
}
function qNative(id, v) {
  // Client q is native units (mm prismatic, deg revolute);
  // PoE needs radians for revolute, mm as-is for prismatic.
  return uOf(id) === "mm" ? v : v * Math.PI / 180;
}
function qvec(q) {
  return MODEL.joints.map((j) => qNative(j.id, q[j.id] || 0));
}
function fkine(q) {
  const js = MODEL.joints;
  let T = new THREE.Matrix4();
  const frames = [];
  for (let i = 0; i < js.length; i++) {
    // Axes arrive raw; Python joint_arrays() normalizes to unit.
    const w0 = js[i].axis, p = js[i].point;
    const n = Math.hypot(w0[0], w0[1], w0[2]) || 1;
    const w = [w0[0] / n, w0[1] / n, w0[2] / n];
    let M;
    if (js[i].kind === "prismatic") {
      // Pure translation along the axis; q is already mm.
      M = new THREE.Matrix4().makeTranslation(
        w[0] * q[i], w[1] * q[i], w[2] * q[i]);
    } else {
      const v = [-(w[1]*p[2] - w[2]*p[1]),
                 -(w[2]*p[0] - w[0]*p[2]),
                 -(w[0]*p[1] - w[1]*p[0])];
      const {R, t} = se3(w, v, q[i]);
      M = new THREE.Matrix4().copy(R).setPosition(t);
    }
    T = T.clone().multiply(M);
    frames.push(T.clone());
  }
  return frames;
}
const _v = new THREE.Vector3();
function tick() {
  requestAnimationFrame(tick);
  const st = window._scene;
  const {renderer, scene, camera, controls, arm} = st;
  const F = fkine(qvec(qdeg));
  for (let i = 0; i < MODEL.joints.length; i++) {
    SEGS[i].mesh.matrix.copy(F[i]).multiply(SEGS[i].local);
    SEGS[i].mesh.matrixWorldNeedsUpdate = true;
  }
  for (let i = 0; i < MODEL.joints.length; i++) {
    const p = MODEL.joints[i].point;
    _v.set(p[0], p[1], p[2]).applyMatrix4(F[i]);
    NODES[i].matrix.makeTranslation(_v.x, _v.y, _v.z);
    NODES[i].matrixWorldNeedsUpdate = true;
  }
  window._scene.eeAxes.matrix.copy(F[F.length - 1]);
  window._scene.eeAxes.matrixWorldNeedsUpdate = true;
  if (VSRC === "ldraw" && MESHES.ready) {
    MESHES.groups[0].matrix.identity();
    MESHES.groups[0].matrixWorldNeedsUpdate = true;
    for (let i = 1; i <= MODEL.joints.length; i++) {
      MESHES.groups[i].matrix.copy(F[i - 1]);
      MESHES.groups[i].matrixWorldNeedsUpdate = true;
    }
  }
  // Ghost preview follows the pending IK solution (connected only).
  if (ghostOn && pendingQ) {
    try {
      const gq = qvec(pendingQ);
      const Fg = fkine(gq);
      for (let i = 0; i < MODEL.joints.length &&
           i < GHOST.prims.length; i++) {
        GHOST.prims[i].mesh.matrix.copy(Fg[i])
          .multiply(GHOST.prims[i].local);
        GHOST.prims[i].mesh.matrixWorldNeedsUpdate = true;
      }
      for (let i = 0; i < MODEL.joints.length &&
           i < GHOST.nodes.length; i++) {
        const p = MODEL.joints[i].point;
        _v.set(p[0], p[1], p[2]).applyMatrix4(Fg[i]);
        GHOST.nodes[i].mesh.matrix.makeTranslation(_v.x, _v.y, _v.z);
        GHOST.nodes[i].mesh.matrixWorldNeedsUpdate = true;
      }
      if (GHOST.ldraw.length) {
        if (GHOST.ldraw[0]) {
          GHOST.ldraw[0].matrix.identity();
          GHOST.ldraw[0].matrixWorldNeedsUpdate = true;
        }
        for (let i = 1; i <= MODEL.joints.length &&
             i < GHOST.ldraw.length; i++) {
          if (!GHOST.ldraw[i]) continue;
          GHOST.ldraw[i].matrix.copy(Fg[i - 1]);
          GHOST.ldraw[i].matrixWorldNeedsUpdate = true;
        }
      }
    } catch (err) { /* ghost is best-effort */ }
  }
  if (!st.grounded && SEGS.length) {
    // Rest the lowest box on the grid once, aim at its middle.
    st.grounded = true;
    fitView();
  }
  // Live verification: tip TCP dot, wrist->tip direction,
  // TCP-target line, distance (mm). Skeleton stays wrist-based
  // so the GT overlay still lines up.
  const pts = skelNodes(F);
  const wrist = pts[pts.length - 1];
  const tcp = tipOf(F);
  st.tcpDot.matrix.makeTranslation(tcp.x, tcp.y, tcp.z);
  st.tcpDot.matrixWorldNeedsUpdate = true;
  const da = st.dirLine.geometry.attributes.position;
  da.setXYZ(0, wrist.x, wrist.y, wrist.z);
  da.setXYZ(1, tcp.x, tcp.y, tcp.z);
  da.needsUpdate = true;
  const ta = st.targetArm || tcp;
  const la = st.tgtLine.geometry.attributes.position;
  la.setXYZ(0, tcp.x, tcp.y, tcp.z);
  la.setXYZ(1, ta.x, ta.y, ta.z);
  la.needsUpdate = true;
  const _ddx = tcp.x - ta.x, _ddy = tcp.y - ta.y, _ddz = tcp.z - ta.z;
  const _dist = Math.sqrt(_ddx * _ddx + _ddy * _ddy + _ddz * _ddz);
  const _tS = modelToStage(tcp), _aS = modelToStage(ta);
  document.getElementById("ik-live").textContent =
    "TCP [" + _tS.x.toFixed(1) + ", " + _tS.y.toFixed(1) + ", " +
    _tS.z.toFixed(1) + "] / 목표 [" + _aS.x.toFixed(1) + ", " +
    _aS.y.toFixed(1) + ", " + _aS.z.toFixed(1) + "] / 거리 " +
    _dist.toFixed(1) + "mm";
  // Encoder actual overlay (magenta): real pose vs command pose.
  try {
    if (lastOv && lastOv.joints && lastOv.joints.every(
        (j) => j.joint_deg !== null && j.joint_deg !== undefined)) {
      const aq = lastOv.joints.map((j) => j.joint_deg * Math.PI / 180);
      st.setSkel(st.actualLine, skelNodes(fkine(aq)));
      st.actualLine.visible = true;
    } else {
      st.actualLine.visible = false;
    }
    // Encoder-readback overlay follows the 실측겹침 checkbox.
    if (st.actualLine.visible) {
      const cb = document.getElementById("vm-real");
      st.actualLine.visible = cb ? !!cb.checked : true;
    }
  } catch (err) { /* overlay is best-effort */ }
  controls.update();
  renderer.render(scene, camera);
}
let lastOk = 0;
async function poll() {
  try {
    const o = await post("/api/overview", {});
    lastOk = Date.now();
    lastOv = o;
    const L = o.latched.left, R = o.latched.right;
    const el = (id, txt, cls) => {
      const e = document.getElementById(id);
      if (!e) return;
      e.textContent = txt; e.className = cls;
    };
    el("st-left", "좌: " + (L === null ? "끊김" : "연결"),
       L === null ? "bad" : "ok");
    el("st-right", "우: " + (R === null ? "끊김" : "연결"),
       R === null ? "bad" : "ok");
    const latched = (L === true || R === true);
    el("st-latch", "래치: " + (latched ? "걸림" : "없음"),
       latched ? "bad" : "ok");
    const allMm = ((MODEL && MODEL.joints) || []).length > 0 &&
      MODEL.joints.every((k) => k.unit === "mm");
    const uj = allMm ? "(mm)" : "(°)";
    let html = "<table><tr><th>관절</th><th>관절각" + uj + "</th>" +
      "<th>모터각</th><th>홈</th><th>방향</th><th>센서</th>" +
      "<th>조그</th></tr>";
    const fEl = document.getElementById("follow-live");
    const follow = fEl ? fEl.checked : false;
    const jtEl = document.getElementById("joints");
    for (const j of o.joints) {
      if (follow)
        qdeg[j.joint] = j.joint_deg === null ? 0 : j.joint_deg;
      const jog = j.direction === "confirmed" ? ""
        : (uOf(j.joint) === "mm" ? " (최대±5mm)" : " (최대±5°)");
      html += "<tr><td>" + j.joint + "</td><td>" +
        (j.joint_deg === null ? "?" : j.joint_deg.toFixed(1)) + "</td><td>" +
        (j.motor_deg === null ? "?" : j.motor_deg.toFixed(1)) + "</td><td>" +
        (j.homed ? "완료" : "−") + "</td><td>" +
        (j.direction === "confirmed" ? "확인됨"
         : "<span class='unconf'>방향 미확인</span>") + "</td><td>" +
        (j.sensor === null || j.sensor === undefined ? "−" :
         j.sensor + ":" + JSON.stringify(j.sensor_state)) +
        "</td><td>" +
        "<button data-j='" + j.joint + "' data-d='5'>+5</button>" +
        "<button data-j='" + j.joint + "' data-d='-5'>−5</button>" +
        jog + "</td></tr>";
    }
    if (jtEl) {
      jtEl.innerHTML = html + "</table>";
    }
    document.querySelectorAll("#joints button").forEach((b) => {
      b.onclick = async () => {
        const r = await post("/api/jog",
          {joint: b.dataset.j, delta: parseFloat(b.dataset.d)});
        if (!r.ok) alert("jog 실패: " + r.error);
      };
    });
    // Homing dots + calibration link + mode banner + diag.
    const dots = o.joints.map((j) =>
      j.joint + (j.homed ? "●" : "○")).join(" ");
    el("homedots", "호밍: " + dots, "");
    const unhomed = o.joints.some((j) => !j.homed);
    const cl = document.getElementById("cal-link");
    if (cl) cl.style.display = unhomed ? "" : "none";
    const conn = (L !== null && R !== null);
    try {
      const md = await post("/api/mode", {});
      if (md.ok) VIRT = !!md.virtual;
    } catch (err) { /* keep last VIRT */ }
    el("mode", (conn ? (VIRT ? "로봇 연결됨(가짜)" : "로봇 연결됨")
                    : "가상만(브릭 미연결)") +
      (ROBOT === "tbot" ? " T-Bot" : ""), conn ? "ok" : "bad");
    const dg = document.getElementById("diag");
    if (dg) {
      let dh = "<table><tr><th>관절</th><th>브릭/포트</th>" +
        "<th>모터각</th><th>관절각</th><th>홈</th><th>센서</th>" +
        "<th>래치</th></tr>";
      for (const j of o.joints) {
        dh += "<tr><td>" + j.joint + "</td><td>" + j.brick + "/" +
          j.port + "</td><td>" +
          (j.motor_deg === null ? "?" : j.motor_deg.toFixed(1)) +
          "</td><td>" +
          (j.joint_deg === null ? "?" : j.joint_deg.toFixed(1)) +
          "</td><td>" + (j.homed ? "완료" : "−") + "</td><td>" +
          (j.sensor === null || j.sensor === undefined ? "−" :
           j.sensor + ":" + JSON.stringify(j.sensor_state)) +
          "</td><td>" + (latched ? "걸림" : "없음") + "</td></tr>";
      }
      dg.innerHTML = dh + "</table>";
    }
    refreshSend();
  } catch (e) {
    document.getElementById("st-hb").textContent = "하트비트: 실패";
    document.getElementById("st-hb").className = "bad";
    return;
  }
  const ago = ((Date.now() - lastOk) / 1000).toFixed(0);
  const hb = document.getElementById("st-hb");
  hb.textContent = "하트비트: " + ago + "초 전 갱신";
  hb.className = "ok";
}
document.getElementById("estop").onclick = async () => {
  const r = await post("/api/estop", {});
  alert(r.ok ? "E-STOP 전송됨" : "E-STOP 실패: " + (r.error || "?"));
  log(r.ok ? "E-STOP 전송됨" : "E-STOP 실패: " + (r.error || "?"));
};
document.getElementById("btn-clear").onclick = async () => {
  const r = await post("/api/clear", {});
  log(r.ok ? "래치 해제됨" : "래치 해제 실패: " + (r.error || "?"));
};
document.getElementById("w4-try").onclick = async () => {
  const j = document.getElementById("w4-joint").value;
  const r = await post("/api/jog", {joint: j, delta: 5});
  document.getElementById("w4-msg").textContent =
    r.ok ? "모델과 실물을 비교하세요" : "실패: " + r.error;
};
document.getElementById("w4-ok").onclick = async () => {
  const j = document.getElementById("w4-joint").value;
  const r = await post("/api/direction", {joint: j, verdict: "correct"});
  document.getElementById("w4-msg").textContent =
    r.ok ? j + " 정방향 확정" : "실패: " + r.error;
};
document.getElementById("w4-rev").onclick = async () => {
  const j = document.getElementById("w4-joint").value;
  const r = await post("/api/direction", {joint: j, verdict: "reversed"});
  document.getElementById("w4-msg").textContent =
    r.ok ? j + " 반대 → 부호 뒤집어 저장됨(비율 " + r.ratio + ")" : "실패: " + r.error;
  log("M4 " + j + " reversed: " + (r.ok ? "비율 " + r.ratio : r.error));
};
function log(msg) {
  const el = document.getElementById("log");
  if (!el) return;
  const d = document.createElement("div");
  d.textContent = new Date().toLocaleTimeString() + " " + msg;
  el.prepend(d);
  while (el.children.length > 50) el.lastChild.remove();
}
async function refreshRanges() {
  const r = await post("/api/ranges", {});
  if (!r.ok) { log("범위 로드 실패: " + r.error); return; }
  const f1 = (v) => (v === null || v === undefined ? "−"
    : (+v).toFixed(1));
  const rng = (a) => "[" + f1(a[0]) + "," + f1(a[1]) + "]";
  let html = "<table><tr><th>관절</th><th>5A 시뮬</th>" +
    "<th>실측[lo,hi]</th><th>채택</th><th>기록</th></tr>";
  for (const j of r.joints) {
    const cur = (qdeg[j.joint] || 0).toFixed(1);
    const uu = uOf(j.joint) === "mm" ? "mm" : "°";
    html += "<tr><td>" + j.joint + "</td><td>" + rng(j.sim) +
      "</td><td>" + rng(j.measured) + "</td><td>" + rng(j.adopted) +
      (j.adopted_source === "measured" ? "(실측)" : "(시뮬)") +
      "</td><td>현재 " + cur + uu + " " +
      "<button data-j='" + j.joint + "' data-e='lo'>lo로</button>" +
      "<button data-j='" + j.joint + "' data-e='hi'>hi로</button>" +
      "</td></tr>";
  }
  document.getElementById("ranges").innerHTML = html + "</table>" +
    "<div>여유 " + r.margin_deg + "° (패키지 " +
    r.package_margin_deg + "°)</div>";
  document.querySelectorAll("#ranges button").forEach((b) => {
    b.onclick = async () => {
      const rr = await post("/api/range", {joint: b.dataset.j,
        end: b.dataset.e, joint_deg: qdeg[b.dataset.j] || 0});
      log("M5 " + b.dataset.j + " " + b.dataset.e + " 기록: " +
        (rr.ok ? "ok" : rr.error));
      refreshRanges();
    };
  });
}
async function refreshPoses() {
  const r = await post("/api/poses", {});
  if (!r.ok) { log("자세 로드 실패: " + r.error); return; }
  const sel = document.getElementById("w6-pose");
  sel.innerHTML = "";
  let html = "<table><tr><th>자세</th><th>목표각</th><th>최근 오차</th></tr>";
  for (const p of r.poses) {
    const o = document.createElement("option");
    o.textContent = p.id; sel.appendChild(o);
    let leTxt = "−";
    if (p.last_error_deg) {
      let m = 0;
      for (const k of Object.keys(p.last_error_deg))
        m = Math.max(m, Math.abs(p.last_error_deg[k]));
      leTxt = "±" + m.toFixed(2) + "°";
    }
    html += "<tr><td>" + p.id + "</td><td>" +
      p.q_deg.map((v) => (+v).toFixed(1)).join(",") + "</td><td>" +
      leTxt + "</td></tr>";
  }
  document.getElementById("poses").innerHTML = html + "</table>";
}
// Single conversion point: model (Z-up, mm) <-> stage (desk frame,
// mm, table = z 0, base bottom center = origin) <-> world (three).
function modelToStage(p) {
  return {x: p.x - STAGE.center_x_mm, y: p.y - STAGE.center_y_mm,
          z: p.z - STAGE.table_z_mm};
}
function stageToModel(s) {
  return new THREE.Vector3(s.x + STAGE.center_x_mm,
    s.y + STAGE.center_y_mm, s.z + STAGE.table_z_mm);
}
function stageOfWorld(w) {
  return {x: w.x, y: -w.z, z: w.y};
}
// Current TCP in model coords, from the live joint values.
function tipOf(F) {
  const t = MODEL.tip_mm;
  return new THREE.Vector3(t[0], t[1], t[2])
    .applyMatrix4(F[F.length - 1]);
}
function currentTCPModel() {
  return tipOf(fkine(qvec(qdeg)));
}
// Minimal rotation taking the D6-frame gripper dir to -z.
function gripperDownMat() {
  const d = MODEL.gripper_dir0;
  const t = [0, 0, -1];
  let ax = [d[1] * t[2] - d[2] * t[1], d[2] * t[0] - d[0] * t[2],
            d[0] * t[1] - d[1] * t[0]];
  let an = Math.hypot(ax[0], ax[1], ax[2]);
  const cosA = Math.max(-1, Math.min(1, d[0] * t[0] + d[1] * t[1] +
    d[2] * t[2]));
  if (an < 1e-9) {
    if (cosA > 0) return [[1,0,0],[0,1,0],[0,0,1]];
    ax = [1, 0, 0]; an = 1;
  }
  ax = [ax[0] / an, ax[1] / an, ax[2] / an];
  const sn = Math.sqrt(Math.max(0, 1 - cosA * cosA)), cs = cosA;
  const [x, y, z] = ax, C = 1 - cs;
  return [[cs + x*x*C, x*y*C - z*sn, x*z*C + y*sn],
          [y*x*C + z*sn, cs + y*y*C, y*z*C - x*sn],
          [z*x*C - y*sn, z*y*C + x*sn, cs + z*z*C]];
}
function setMarker(color) {
  const st = window._scene;
  if (st && st.marker) st.marker.material.color.setHex(color);
}
// Put the gizmo on the current TCP: used at open (initial pose),
// after homing/pose/goto moves, and by the dedicated button.
function snapGizmoToTCP() {
  const st = window._scene;
  if (!st) return;
  st.target.position.copy(st.arm.localToWorld(currentTCPModel()));
  setMarker(0xff8800);
}
async function solveIK(seedQ) {
  const st = window._scene;
  const pm = st.arm.worldToLocal(st.target.position.clone());
  st.targetArm = pm.clone();
  st.prevQ = {...qdeg};
  try {
    const pq = qvec(st.prevQ);
    st.setSkel(st.prevLine, skelNodes(fkine(pq)));
    st.prevLine.visible =
      document.getElementById("vm-prev").checked;
  } catch (err) {
    log("이전 자세 표시 실패");
  }
  let mat = null, po = true;
  if (st.tc.getMode() === "rotate") {
    po = false;
    const aq = st.arm.getWorldQuaternion(new THREE.Quaternion()).invert();
    const qm = st.target.quaternion.clone().premultiply(aq);
    const e = new THREE.Matrix4().makeRotationFromQuaternion(qm).elements;
    mat = [[e[0], e[4], e[8]], [e[1], e[5], e[9]], [e[2], e[6], e[10]]];
  } else if (document.getElementById("grip-down").checked) {
    // Demo option: position + gripper-down (overrides position-only).
    po = false;
    mat = gripperDownMat();
  }
  // Table clamp in the stage frame (server clamps again).
  let sg = modelToStage(pm);
  let wasClamped = false;
  if (sg.z < 0) { sg = {x: sg.x, y: sg.y, z: 0}; wasClamped = true; }
  const goal = stageToModel(sg);
  const qinit = seedQ || MODEL.joints.map((j) => qdeg[j.id] || 0);
  let r = await post("/api/ik", {pos: [goal.x, goal.y, goal.z],
    mat, q_init: qinit, position_only: po});
  // Out of reach: follow only to the nearest reachable point by
  // binary-searching the current-TCP -> goal segment.
  let walked = false;
  if (r.ok && r.ik && !r.ik.success && po) {
    const tcpM = currentTCPModel();
    let lo = 0, hi = 1, best = null;
    for (let k = 0; k < 8; k++) {
      const mid = (lo + hi) / 2;
      const p = new THREE.Vector3(
        tcpM.x + (goal.x - tcpM.x) * mid,
        tcpM.y + (goal.y - tcpM.y) * mid,
        tcpM.z + (goal.z - tcpM.z) * mid);
      const rr = await post("/api/ik",
        {pos: [p.x, p.y, p.z], mat, q_init: qinit,
         position_only: true});
      if (rr.ok && rr.ik && rr.ik.success) { lo = mid; best = rr; }
      else hi = mid;
    }
    if (best) { r = best; walked = true; }
  }
  const msg = document.getElementById("ik-msg");
  if (!r.ok || !r.ik) {
    setMarker(0xff2222);
    msg.textContent = "실패: " + (r.error || "?");
    log("IK 실패: " + (r.error || "?"));
    return;
  }
  // Display rule: virtual-only (bricks unreachable) drives the
  // body itself to the IK result in real time. When connected, the
  // body stays on the encoder actual and the result shows on a
  // translucent ghost model; the line preview is gone.
  const sol = {};
  r.joints.forEach((jid, i) => { sol[jid] = r.ik.q_deg[i]; });
  const solRad = qvec(sol);
  const conn = isConnected();
  if (!conn) {
    for (const j of MODEL.joints) qdeg[j.id] = sol[j.id];
    pendingQ = null; previewOk = false; previewRange = "";
    ghostOn = false; applyGhost();
    msg.textContent += " | 가상 추종(본체 이동)";
  } else {
    pendingQ = sol;
    previewOk = !!r.ik.success;
    previewRange = "";
    ghostOn = true; applyGhost();
  }
  try {
    // Client-side agreement check: solution TCP (same fkine the
    // scene draws) vs the gizmo target, in mm.
    const ptcp = tipOf(fkine(solRad));
    const pta = st.targetArm;
    const pdd = Math.sqrt((ptcp.x - pta.x) ** 2 +
      (ptcp.y - pta.y) ** 2 + (ptcp.z - pta.z) ** 2);
    const _pS = modelToStage(ptcp), _gS = modelToStage(pta);
    document.getElementById("preview-dist").textContent =
      (conn ? "미리보기" : "본체") +
      " [" + _pS.x.toFixed(1) + ", " + _pS.y.toFixed(1) +
      ", " + _pS.z.toFixed(1) + "] / 목표 [" + _gS.x.toFixed(1) +
      ", " + _gS.y.toFixed(1) + ", " + _gS.z.toFixed(1) +
      "] / 거리 " + pdd.toFixed(1) + "mm";
  } catch (err) {
    log("미리보기 표시 실패");
  }
  setMarker((wasClamped || walked || !r.ik.success) ? 0xff2222 :
    0xff8800);
  msg.textContent = (r.ik.success ? "수렴" : "미수렴(최근접)") +
    (wasClamped ? " (테이블 클램프)" : "") +
    (walked ? " (최근접 추종)" : "") +
    " 위치오차 " + r.ik.pos_err_mm.toFixed(1) + "mm" +
    " 자세오차 " + r.ik.rot_err_deg.toFixed(1) + "°";
  log("IK " + msg.textContent);
  try {
    const qd = {};
    for (const j of MODEL.joints) qd[j.id] = sol[j.id];
    const pv = await post("/api/preview", {q_deg: qd});
    if (pv.ok) {
      const ms = pv.motors.map((m) => m.motor + "=" + m.target).join(" ");
      previewRange = pv.range;
      msg.textContent += " | 모터 " + ms + " | 범위 " + pv.range;
      log("모터 미리보기: " + ms + " / 범위: " + pv.range);
    }
  } catch (err) {
    log("미리보기 실패: " + (err && err.message ? err.message : err));
  }
  refreshSend();
  try {
    if (!st.rangesAdopted) {
      const rr = await post("/api/ranges", {});
      if (rr.ok) {
        st.rangesAdopted = {};
        for (const rj of rr.joints) st.rangesAdopted[rj.joint] = rj.adopted;
      }
    }
    const lim = [];
    if (st.rangesAdopted) {
      for (const j of MODEL.joints) {
        const ad = st.rangesAdopted[j.id];
        const v = sol[j.id];
        if (v !== undefined && ad &&
            (Math.abs(v - ad[0]) < 0.5 || Math.abs(v - ad[1]) < 0.5))
          lim.push(j.id);
      }
    }
    const stat =
      (r.ik.success ? "수렴" : "미수렴(최근접)") +
      " / 오차 " + r.ik.pos_err_mm.toFixed(1) + "mm" +
      " / 제한 " + (lim.length ? lim.join(",") : "없음") +
      " / 목표(테이블) [" + sg.x.toFixed(1) + ", " + sg.y.toFixed(1) +
      ", " + sg.z.toFixed(1) + "]mm";
    document.getElementById("ik-stat").textContent = stat;
    log("IK 상태: " + stat);
    if (document.getElementById("vm-gt").checked) {
      const gq = {};
      for (const j of MODEL.joints) gq[j.id] = sol[j.id];
      const g = await post("/api/gt_frames", {q_deg: gq});
      if (g.ok) {
        const gp = g.points.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
        st.setSkel(st.gtLine, gp);
      } else {
        st.gtLine.visible = false;
        log("GT 표시 불가: " + (g.error || "no private visuals"));
      }
    } else {
      st.gtLine.visible = false;
    }
  } catch (err) {
    log("IK 상태 표시 실패: " + (err && err.message ? err.message : err));
  }
}
const SEND_SPEED_DPS = 15.0;
// Connected = bricks answer reads recently. Stale or null reads
// mean virtual-only: the body follows IK itself, send stays hidden.
function isConnected() {
  // T-Bot is virtual-only: never connected, body follows IK.
  if (typeof ROBOT !== "undefined" && ROBOT === "tbot") return false;
  return !!(lastOv && (Date.now() - lastOk < 6000) &&
    lastOv.latched.left !== null && lastOv.latched.right !== null);
}
function sendBlockReason() {
  if (!lastOv) return "상태 수신 중";
  if (!isConnected()) return "브릭 미연결(가상만)";
  const L = lastOv.latched.left, R = lastOv.latched.right;
  if (L === true || R === true) return "래치 걸림(해제 필요)";
  if (lastOv.joints.some((j) => !j.homed))
    return "호밍 안 됨(보정 페이지)";
  if (!pendingQ || !previewOk) return "유효한 미리보기 없음";
  if (previewRange && previewRange !== "ok")
    return "범위 위반: " + previewRange;
  return "";
}
function refreshSend() {
  const btn = document.getElementById("send-go");
  const msg = document.getElementById("send-msg");
  const row = document.getElementById("send-row");
  if (row) row.style.display = isConnected() ? "" : "none";
  const reason = sendBlockReason();
  if (btn) btn.disabled = !!reason;
  if (msg) msg.textContent =
    reason || ("보내기 가능 (저속 " + SEND_SPEED_DPS + "°/s)");
}
document.getElementById("send-go").onclick = async () => {
  const reason = sendBlockReason();
  if (reason) {
    document.getElementById("send-msg").textContent = reason;
    return;
  }
  let mx = 0;
  for (const j of MODEL.joints)
    mx = Math.max(mx, Math.abs((pendingQ[j.id] || 0) -
      (qdeg[j.id] || 0)));
  if (mx > 45 && !confirm("한 관절이 " + mx.toFixed(1) +
      "° 움직입니다. 진행할까요?")) {
    log("보내기 취소(대이동 확인 거부)");
    return;
  }
  const st = window._scene;
  const r = await post("/api/goto", {q_deg: pendingQ,
    speed_dps: SEND_SPEED_DPS, label: "main-send"});
  if (!r.ok) {
    document.getElementById("send-msg").textContent =
      "거부: " + (r.error || "?");
    log("보내기 거부: " + (r.error || "?"));
    return;
  }
  for (const j of MODEL.joints) qdeg[j.id] = pendingQ[j.id];
  document.getElementById("follow-live").checked = false;
  snapGizmoToTCP();
  pendingQ = null; previewOk = false; previewRange = "";
  ghostOn = false; applyGhost();
  log("로봇으로 보내기 완료");
  refreshSend();
};
document.getElementById("send-stop").onclick = async () => {
  const r = await post("/api/estop", {});
  log(r.ok ? "정지(E-STOP) 전송됨" : "정지 실패: " + (r.error || "?"));
};
document.getElementById("xyz-go").onclick = () => {
  const st = window._scene;
  const x = parseFloat(document.getElementById("xyz-x").value);
  const y = parseFloat(document.getElementById("xyz-y").value);
  const z = parseFloat(document.getElementById("xyz-z").value);
  if ([x, y, z].some((v) => isNaN(v))) {
    document.getElementById("ik-msg").textContent =
      "좌표 입력이 숫자가 아닙니다";
    return;
  }
  st.target.position.copy(st.arm.localToWorld(stageToModel(
    {x: x, y: y, z: z})));
  solveIK();
};
document.getElementById("ik-t").onclick = () => {
  window._scene.tc.setMode("translate");
};
document.getElementById("ik-r").onclick = () => {
  window._scene.tc.setMode("rotate");
};
for (const ax of ["x", "y", "z"]) {
  document.getElementById("ik-" + ax).onclick = (e) => {
    const tc = window._scene.tc;
    const show = "show" + ax.toUpperCase();
    tc[show] = !tc[show];
    e.target.style.opacity = tc[show] ? 1 : 0.4;
  };
}
document.getElementById("ik-space").onclick = () => {
  const tc = window._scene.tc;
  tc.setSpace(tc.getSpace() === "world" ? "local" : "world");
};
document.getElementById("ik-reset").onclick = () => {
  snapGizmoToTCP();
  solveIK();
};
document.getElementById("ik-tcp").onclick = () => {
  snapGizmoToTCP();
  log("기즈모를 현재 TCP로 이동");
};
document.getElementById("ik-reach").onclick = async () => {
  const st = window._scene;
  const r = await post("/api/reachable_target", {});
  if (!r.ok) {
    document.getElementById("ik-msg").textContent =
      "닿는 목표 실패: " + r.error;
    return;
  }
  const v = new THREE.Vector3(r.target[0], r.target[1], r.target[2]);
  st.target.position.copy(st.arm.localToWorld(v));
  document.getElementById("ik-msg").textContent =
    "닿는 목표: [" + r.target.join(", ") + "]mm";
  solveIK();
};
let MAT = null;
async function loadMat() {
  const r = await post("/api/mat_points", {});
  const sel = document.getElementById("mat-point");
  if (!r.ok || !r.mat) {
    sel.innerHTML = "<option>매트 없음(비공개)</option>";
    return;
  }
  MAT = r.mat;
  sel.innerHTML = MAT.points.map((p) => {
    const s = modelToStage({x: p.x, y: p.y, z: p.z});
    return `<option value="${p.n}">${p.n}: (${s.x.toFixed(0)}, ` +
      `${s.y.toFixed(0)}, ${s.z.toFixed(0)})</option>`;
  }).join("");
  // Show the mat points above the table (same arm transform).
  const st = window._scene;
  if (st && st.matDots) {
    while (st.matDots.children.length)
      st.matDots.remove(st.matDots.children[0]);
    const geo = new THREE.SphereGeometry(6, 12, 10);
    const mt = new THREE.MeshBasicMaterial({color: 0xffff00});
    for (const p of MAT.points) {
      const d = new THREE.Mesh(geo, mt);
      d.matrixAutoUpdate = false;
      d.matrix.makeTranslation(p.x, p.y, p.z);
      st.matDots.add(d);
    }
    // Mat point shortcut buttons 1..9 (same as select + go).
    const mb = document.getElementById("mat-btns");
    if (mb) {
      mb.innerHTML = "";
      for (const p of MAT.points) {
        const b = document.createElement("button");
        b.textContent = p.n;
        b.title = "매트 " + p.n;
        b.onclick = () => {
          document.getElementById("mat-point").value = String(p.n);
          document.getElementById("mat-go").click();
        };
        mb.appendChild(b);
      }
    }
    // VLA pointing slots S1..S6 (same arm transform, cyan labels).
    if (r.slots) {
      const mkLabel = (text) => {
        const c = document.createElement("canvas");
        c.width = 128; c.height = 64;
        const g = c.getContext("2d");
        g.fillStyle = "rgba(0,0,0,0.65)";
        g.fillRect(0, 0, 128, 64);
        g.font = "bold 40px system-ui, sans-serif";
        g.textAlign = "center"; g.textBaseline = "middle";
        g.fillStyle = "#7df9ff";
        g.fillText(text, 64, 34);
        const sp = new THREE.Sprite(new THREE.SpriteMaterial({
          map: new THREE.CanvasTexture(c), depthTest: false}));
        sp.scale.set(48, 24, 1);
        return sp;
      };
      for (const s of r.slots) {
        const sp = mkLabel(s.id);
        sp.matrixAutoUpdate = false;
        sp.matrix.makeTranslation(s.x, s.y, (s.z || 0) + 18);
        st.matDots.add(sp);
      }
    }
    // Riser box: table (model table_z) up to the mat plane.
    const xs = MAT.points.map((p) => p.x);
    const ys = MAT.points.map((p) => p.y);
    const top = Math.max(...MAT.points.map((p) => p.z));
    const bx = [Math.min(...xs) - 30, Math.max(...xs) + 30];
    const by = [Math.min(...ys) - 30, Math.max(...ys) + 30];
    const rbox = new THREE.Mesh(
      new THREE.BoxGeometry(bx[1] - bx[0], by[1] - by[0],
        top - STAGE.table_z_mm),
      new THREE.MeshStandardMaterial({color: 0x8a6f4d,
        roughness: 0.9, transparent: true, opacity: 0.25}));
    rbox.matrixAutoUpdate = false;
    // BoxGeometry is centered: place center at box middle.
    // (arm frame is model/Z-up, so translate then it just works.)
    rbox.matrix.makeTranslation((bx[0] + bx[1]) / 2,
      (by[0] + by[1]) / 2, (STAGE.table_z_mm + top) / 2);
    rbox.name = "riser";
    st.matDots.add(rbox);
  }
}
document.getElementById("mat-go").onclick = async () => {
  if (!MAT) await loadMat();
  if (!MAT) return;
  const n = parseInt(document.getElementById("mat-point").value, 10);
  const p = MAT.points.find((q) => q.n === n);
  if (!p) return;
  const st = window._scene;
  st.target.position.copy(st.arm.localToWorld(
    new THREE.Vector3(p.x, p.y, p.z)));
  document.getElementById("ik-msg").textContent =
    `매트 ${p.n}: [${p.x}, ${p.y}, ${p.z}]mm`;
  solveIK(p.seed || null);
};
document.getElementById("demo-start").onclick = async () => {
  const msg = document.getElementById("demo-msg");
  const step = async (name, fn) => {
    if (!confirm(`다음 단계: ${name}\n진행할까요?`)) {
      msg.textContent = `중단: ${name}`;
      log(`데모 중단: ${name}`);
      throw new Error("stop");
    }
    const r = await fn();
    msg.textContent = `${name}: ${r}`;
    log(`데모 ${name}: ${r}`);
  };
  try {
    await step("호밍 상태 확인", async () => {
      const o = await post("/api/overview", {});
      if (!o.ok) throw new Error("overview 실패");
      return "연결 ok";
    });
    const pr = await post("/api/poses", {});
    const poseQ = {};
    if (pr.ok) for (const p of pr.poses) poseQ[p.id] = p.q_deg;
    for (const pose of ["safe02", "safe05"]) {
      await step(`안전자세 ${pose} (저속)`, async () => {
        const r = await post("/api/pose", {pose});
        if (!r.ok) throw new Error(r.error || "거부");
        if (poseQ[pose]) MODEL.joints.forEach((j, i) => {
          qdeg[j.id] = poseQ[pose][i];
        });
        snapGizmoToTCP();
        return `최대오차 ${r.max_abs_err.toFixed(2)}°`;
      });
    }
    if (!MAT) await loadMat();
    const nums = document.getElementById("demo-points").value
      .split(",").map((s) => parseInt(s.trim(), 10))
      .filter((v) => !isNaN(v)).slice(0, 5);
    for (const n of nums) {
      await step(`매트 ${n}번으로 이동 (저속)`, async () => {
        const p = MAT.points.find((q) => q.n === n);
        if (!p) throw new Error("매트 점 없음");
        const ik = await post("/api/ik",
          {pos: [p.x, p.y, p.z], mat: null,
           q_init: p.seed ||
             MODEL.joints.map((j) => qdeg[j.id] || 0),
           position_only: true});
        if (!ik.ok || !ik.ik.success)
          throw new Error("IK 실패(도달 불가 표시)");
        const qd = {};
        ik.joints.forEach((jid, i) => { qd[jid] = ik.ik.q_deg[i]; });
        const g = await post("/api/goto",
          {q_deg: qd, speed_dps: 15, label: `mat${n}`});
        if (!g.ok) throw new Error(g.error || "이동 거부");
        ik.joints.forEach((jid, i) => { qdeg[jid] = ik.ik.q_deg[i]; });
        snapGizmoToTCP();
        return `오차 ${ik.ik.pos_err_mm.toFixed(1)}mm`;
      });
    }
    await step("원위치 (safe01)", async () => {
      const r = await post("/api/pose", {pose: "safe01"});
      if (!r.ok) throw new Error(r.error || "거부");
      if (poseQ["safe01"]) MODEL.joints.forEach((j, i) => {
        qdeg[j.id] = poseQ["safe01"][i];
      });
      snapGizmoToTCP();
      return "복귀 완료";
    });
    msg.textContent += " — 데모 끝";
  } catch (err) {
    if (err && err.message !== "stop") {
      msg.textContent = "실패: " + err.message;
      log("데모 실패: " + err.message);
    }
  }
};
document.getElementById("w6-run").onclick = async () => {
  const id = document.getElementById("w6-pose").value;
  const r = await post("/api/pose", {pose: id});
  document.getElementById("w6-msg").textContent =
    r.ok ? id + " 재현 완료, 최대오차 " + r.max_abs_err.toFixed(2) + "°"
         : "실패(거부): " + r.error;
  log("M6 " + id + ": " + (r.ok ? "ok" : r.error));
  if (r.ok) snapGizmoToTCP();
  refreshPoses();
};
async function refreshConfig() {
  const r = await post("/api/config", {});
  if (!r.ok) { log("설정 로드 실패: " + r.error); return; }
  let html = "<table><tr><th>항목</th><th>값</th><th>변경</th></tr>";
  for (const k of Object.keys(r.tunables)) {
    html += "<tr><td>" + k + "</td><td>" + r.tunables[k] +
      "</td><td><input id='cfg-" + k + "' size='6'>" +
      "<button data-k='" + k + "'>적용</button></td></tr>";
  }
  document.getElementById("cfgs").innerHTML = html + "</table>" +
    "<div>패키지 여유 " + r.package.operating_margin_deg + "°, 속도 상한 " +
    JSON.stringify(r.package.speed_caps) + "</div>";
  document.querySelectorAll("#cfgs button").forEach((b) => {
    b.onclick = async () => {
      const v = parseFloat(
        document.getElementById("cfg-" + b.dataset.k).value);
      const rr = await post("/api/config/set",
        {key: b.dataset.k, value: v});
      log("설정 " + b.dataset.k + "=" + v + ": " +
        (rr.ok ? "ok" : rr.error));
      refreshConfig();
    };
  });
}
async function renderHoming() {
  const box = document.getElementById("home-rows");
  if (!box) return;
  try {
    const spec = await post("/api/homing_spec", {});
    const o = await post("/api/overview", {});
    const homed = {};
    if (o.ok) for (const j of o.joints) homed[j.joint] = j.homed;
    let html = "";
    const rows = spec.ok && spec.joints
      ? Object.entries(spec.joints) : [];
    for (const [jid, jspec] of rows) {
      const method = (jspec && jspec.method) || "manual";
      html += "<div>" + jid + " [" + method + "] " +
        (homed[jid] ? "완료" : "미완") + " " +
        "<button data-home='" + jid + "'>호밍</button> " +
        "<button data-mark='" + jid + "'>현재를 0으로</button>" +
        "<span id='home-msg-" + jid + "'></span></div>";
    }
    box.innerHTML = html;
    box.querySelectorAll("[data-home]").forEach((b) => {
      b.onclick = async () => {
        const m = document.getElementById("home-msg-" + b.dataset.home);
        const r = await post("/api/home_joint",
          {joint: b.dataset.home, mark: false});
        if (m) m.textContent = r.ok
          ? "결과: " + JSON.stringify({offset: r.offset,
              method: r.method})
          : "실패: " + (r.error || "?");
        poll();
      };
    });
    box.querySelectorAll("[data-mark]").forEach((b) => {
      b.onclick = async () => {
        const m = document.getElementById("home-msg-" + b.dataset.mark);
        const r = await post("/api/home_joint",
          {joint: b.dataset.mark, mark: true});
        if (m) m.textContent = r.ok
          ? "0으로 표시됨(offset " + r.offset + ")"
          : "실패: " + (r.error || "?");
        poll();
      };
    });
    if (!spec.ok)
      box.innerHTML += "<div>호밍 명세 실패: " + spec.error + "</div>";
  } catch (err) {
    box.innerHTML = "호밍 로드 실패";
  }
}
async function renderVerify() {
  if (!document.getElementById("verify-table")) return;
  try {
    const r = await post("/api/verify_report", {});
    if (!r.ok) {
      document.getElementById("verify-table").textContent =
        "보고서 로드 실패: " + r.error;
      return;
    }
    const rep = r.report || null;
    if (!rep) {
      document.getElementById("verify-table").textContent =
        "아직 검증 보고서가 없습니다(시뮬레이션 산출 후 표시)";
      return;
    }
    // Report shape: runs/stage5c/ik_check_multi/report.json
    // {targets: [{id, kind, tcp_err_mm, dq, base_moved,
    // at_limit, pass, reason}], pass_reachable, pass_unreachable}.
    let html = "<table><tr><th>목표</th><th>종류</th>" +
      "<th>TCP 오차mm</th><th>밑동 관절</th><th>통과</th>" +
      "<th>사유</th></tr>";
    for (const t of (rep.targets || [])) {
      const dq = (t.dq || []).map((v) => (+v).toFixed(1)).join(",");
      html += "<tr><td>" + t.id + "</td><td>" + (t.kind || "") +
        "</td><td>" + t.tcp_err_mm + "</td><td>" +
        (t.base_moved || []).join(",") + " [" + dq + "]" +
        "</td><td>" + (t.pass ? "PASS" : "FAIL") + "</td><td>" +
        (t.reason || "") + "</td></tr>";
    }
    html += "</table><div>닿는 목표 통과 " +
      JSON.stringify(rep.pass_reachable) + " / 닿지 않는 목표 " +
      JSON.stringify(rep.pass_unreachable) + "</div>";
    if (rep.images && rep.images.length) {
      html += "<div>증거: " + rep.images.map((im) =>
        "<a href='" + im.url + "'>" + im.name + "</a>").join(" ") + "</div>";
    }
    try {
      const g = await post("/api/gt_frames",
        {q_deg: {J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0}});
      html += "<div>GT 뼈대: " + (g.ok
        ? "사용 가능 — 메인 3D의 GT 표시로 겹쳐 비교하세요"
        : "비공개(--private-visuals 필요)") + "</div>";
    } catch (err) {
      html += "<div>GT 뼈대: 확인 실패</div>";
    }
    document.getElementById("verify-table").innerHTML = html;
  } catch (err) {
    document.getElementById("verify-table").textContent = "로드 실패";
  }
}
// ---- Digital twin (main page) ----
let TWIN = {slots: {}, ids: [], colours: [], holdQ: null};
async function initTwin() {
  if (!document.getElementById("twin-slots")) return;
  const r = await post("/api/twin/slots", {});
  const box = document.getElementById("twin-slots");
  if (!r.ok) { box.textContent = "로드 실패: " + r.error; return; }
  TWIN.ids = r.ids; TWIN.colours = r.colours; TWIN.slots = r.slots || {};
  let html = "<table><tr>";
  for (const sid of TWIN.ids) {
    html += "<td>" + sid + "<br><select data-slot='" + sid + "'>" +
      "<option value=''>없음</option>" + TWIN.colours.map((c) =>
        "<option value='" + c + "'" +
        (TWIN.slots[sid] === c ? " selected" : "") + ">" + c +
        "</option>").join("") + "</select></td>";
  }
  box.innerHTML = html + "</tr></table>";
  const tgt = document.getElementById("twin-target");
  tgt.innerHTML = TWIN.colours.map((c) =>
    "<option value='" + c + "'>" + c + "</option>").join("");
  document.getElementById("twin-save").onclick = async () => {
    const rr = await post("/api/twin/slots/set",
      {slots: readTwinSlots()});
    document.getElementById("twin-msg").textContent =
      rr.ok ? "색 저장됨" : "저장 실패: " + rr.error;
  };
  document.getElementById("twin-run").onclick = runTwin;
  document.getElementById("twin-send").onclick = sendTwin;
  document.getElementById("twin-rest").onclick = async () => {
    if (!confirm("휴식 자세로 저속 이동할까요?")) return;
    const rr = await post("/api/twin/rest", {});
    document.getElementById("twin-msg").textContent =
      rr.ok ? "휴식 자세 이동 완료" : "실패: " + rr.error;
    log("트윈 휴식: " + (rr.ok ? "ok" : rr.error));
  };
}
function readTwinSlots() {
  const out = {};
  document.querySelectorAll("[data-slot]").forEach((s) => {
    if (s.value) out[s.dataset.slot] = s.value;
  });
  return out;
}
async function runTwin() {
  const msg = document.getElementById("twin-msg");
  msg.textContent = "가상 이미지 생성+VLA 추론 중…";
  const r = await post("/api/twin/run",
    {slots: readTwinSlots(),
     target: document.getElementById("twin-target").value});
  if (!r.ok) { msg.textContent = "실패: " + r.error; return; }
  TWIN.holdQ = r.hold_q;
  const img = document.getElementById("twin-img");
  img.src = "data:image/png;base64," + r.png;
  img.style.display = "";
  msg.textContent = "정책=" + r.policy + " 목표=" + r.target +
    " 유지=" + JSON.stringify(r.hold_q) + " " + JSON.stringify(r.rec);
  document.getElementById("twin-send").disabled = false;
  log("트윈 실행: 정책=" + r.policy + " 목표=" + r.target);
}
async function sendTwin() {
  if (!TWIN.holdQ) return;
  if (!confirm("유지 자세로 저속(15°/s) 이동할까요?")) return;
  const r = await post("/api/goto",
    {q_deg: TWIN.holdQ, speed_dps: 15, label: "twin-hold"});
  document.getElementById("twin-msg").textContent =
    r.ok ? "유지 자세 이동 완료, 최대오차 " +
      r.max_abs_err.toFixed(2) + "°" : "실패(거부): " + r.error;
  log("트윈 전송: " + (r.ok ? "ok" : r.error));
}
// ---- Session calibration (cal page) ----
async function refreshSess() {
  const box = document.getElementById("sess-stat");
  if (!box) return;
  const r = await post("/api/calib/session", {});
  if (!r.ok) { box.textContent = "로드 실패: " + r.error; return; }
  box.innerHTML = "연속성=" + r.continuity +
    " 확정=" + (r.confirmed ? ("됨(" + r.confirmed_at + ")") : "안됨") +
    " 저장=" + (r.saved_at || "없음") +
    "<br>현재 가동=" + JSON.stringify(r.boot_now) +
    "<br>저장 가동=" + JSON.stringify(r.boot_saved);
}
function initSess() {
  if (!document.getElementById("sess-flow")) return;
  refreshSess();
  const msg = (t) => {
    document.getElementById("sess-msg").textContent = t;
  };
  document.getElementById("sess-flow").onclick = async () => {
    if (!confirm("한 번에 보정: J1~J4 센서 호밍 → 손목 → 확인자세. 진행할까요?"))
      return;
    const r = await post("/api/calib/flow", {});
    const steps = (r.ok && r.steps ? r.steps : [])
      .map((s) => s.step + "=" + s.result +
        (s.max_abs_deg !== undefined ? "(" + s.max_abs_deg + "°)" : "") +
        (s.note ? " " + s.note : "")).join(" / ");
    document.getElementById("flow-steps").textContent = steps;
    msg(r.ok ? ("흐름=" + r.flow) : ("실패: " + r.error));
    if (r.ok && r.flow === "done")
      msg("확인 자세로 이동됨. 3D 실측겹침을 보고 '맞음'을 누르세요.");
    poll();
  };
  document.getElementById("sess-park").onclick = async () => {
    if (!confirm("주차: 휴식 자세로 저속 이동 후 세션 저장. 진행할까요?"))
      return;
    const r = await post("/api/calib/park", {});
    msg(r.ok ? "주차 저장됨(" + r.saved_at + ")" : "실패: " + r.error);
    refreshSess(); poll();
  };
  document.getElementById("sess-restore").onclick = async () => {
    const r = await post("/api/calib/restore", {});
    msg(r.ok ? "복원됨(연속성=" + r.continuity + ")" : "실패: " + r.error);
    poll();
  };
  document.getElementById("sess-check").onclick = async () => {
    const r = await post("/api/calib/check", {});
    msg(r.ok ? ("확인 자세 이동, 최대편차 " +
      r.deviation.max_abs_deg + "°") : ("실패: " + r.error));
    poll();
  };
  document.getElementById("sess-ok").onclick = async () => {
    const r = await post("/api/calib/confirm", {});
    msg(r.ok ? ("보정 확정됨(" + r.confirmed_at + ")")
             : ("확정 불가: " + r.error));
    refreshSess();
  };
}
// Page split: cal/verify pages lack main-only controls; absorb lookups
// with a detached element so shared handlers never throw.
const __realById = document.getElementById.bind(document);
document.getElementById = (id) =>
  __realById(id) || document.createElement("div");
initModel().then(() => {
  initRobot();
  // Show only this page's sections (log section has no data-sec
  // and stays visible everywhere).
  document.querySelectorAll("[data-sec]").forEach((s) => {
    if (s.dataset.sec !== PAGE_ID) s.style.display = "none";
  });
  const pg = PAGE_ID;
  if (pg === "main") {
    poll(); setInterval(poll, 2000);
    refreshRanges(); refreshPoses(); refreshConfig();
    refreshVisual(); loadMat(); initTwin();
    fitLayout();
  } else {
    poll(); setInterval(poll, 3000);
    if (pg === "cal") {
      // M5/M6/config panels live on the cal page.
      refreshRanges(); refreshPoses(); refreshConfig();
      initSess();
      renderHoming();
      const wrist = document.getElementById("wrist-img");
      if (wrist) {
        wrist.src = "/private/renders/wrist_zero.png";
        wrist.style.display = "";
        wrist.onerror = () => { wrist.style.display = "none"; };
      }
      setInterval(() => {
        if (document.getElementById("home-rows").dataset.auto !== "0")
          renderHoming();
      }, 10000);
    }
    if (pg === "verify") renderVerify();
  }
});
</script>
</body>
</html>"""


if __name__ == "__main__":
    main()
