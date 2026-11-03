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
    "/api/goto", "/api/preview", "/api/home_joint", "/api/home_all",
    "/api/direction",
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


def mark_zero(handler, joints):
    """Record current positions as zero in one encoder snapshot.

    Offsets are chosen so the coupled readback is exactly 0, which
    keeps the wrist cross terms (J4/J5/J6) exact. Joints without a
    live encoder reading are refused, not guessed.
    """
    enc = {}
    for brick in handler.transport.bricks:
        try:
            enc.update(
                {(brick, port): motor for port, motor in
                 handler.transport.send(brick, "read")
                 ["encoders"].items()})
        except Exception:
            pass
    try:
        raw = mini.coupled_joint_readback(
            enc, handler.jog_map, {}) or {}
    except KeyError:
        raw = {}
    offsets = {}
    for jid in joints:
        if jid not in handler.jog_map:
            raise ValueError("unknown joint: " + str(jid))
        r = raw.get(jid)
        if r is None:
            entry = handler.jog_map[jid]
            m = enc.get((entry["brick"], entry["port"]))
            if m is None:
                raise ValueError("no encoder reading for "
                                 + str(jid))
            r = float(entry["ratio"]) * float(m)
        offset = -float(r)
        write_homing_result(jid, "homed", offset,
                            "manual mark at %.1f deg" % float(r))
        offsets[jid] = offset
    return offsets


def home_joint(handler, joint, mark=False):
    """Home one joint: sensor run (real) or simulated (fake bricks).

    Manual joints (J5/J6): mark=True records the current position
    as zero via :func:`mark_zero`.
    """
    spec = homing_spec().get(joint, {}) or {}
    method = spec.get("method", "manual")
    if joint not in handler.jog_map:
        raise ValueError("unknown joint: " + str(joint))
    entry = handler.jog_map[joint]
    record = latest_homing_record()
    offsets = record.get("offsets", {}) or {}
    if method == "manual" or mark:
        offsets = mark_zero(handler, [joint])
        return {"ok": True, "joint": joint, "method": "manual",
                "offset": offsets[joint]}
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
        if path == "/api/home_all":
            return {"ok": True,
                    "offsets": mark_zero(self, list(self.jog_map))}
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
        if path == "/api/vla/camera":
            return vla_camera_info()
        if path == "/api/vla/infer":
            out = vla_infer(str(args.get("source", "real")),
                            str(args.get("target", "red")),
                            str(args.get("task", "") or ""),
                            str(args.get("ckpt", "ft9b")),
                            args.get("fake_index", 0))
            return out
        if path == "/api/vla/trial/save":
            return vla_trial_save(str(args.get("instruction", "")),
                                  str(args.get("ckpt", "ft9b")))
        if path == "/api/vla/trial/vote":
            return vla_trial_vote(str(args.get("id", "")),
                                  bool(args.get("correct", False)))
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


#: VLA webcam panel (server side). Capture is OpenCV-only; the frame
#: preprocessing (center-square crop -> 256) matches training.
VLA_CKPTS = {
    "ft9b": "runs/vla/ckpt-ft9b/pretrained_model",
    "6000": "runs/vla/ckpt-last",
}
_VLA_POLICIES = {}
_VLA_LAST = {}


def webcam_config():
    """Camera number from config/webcam.local.yaml (default 0)."""
    import yaml

    root = pathlib.Path(__file__).resolve().parents[2]
    try:
        data = yaml.safe_load(
            (root / "config" / "webcam.local.yaml").read_text(
                encoding="utf-8")) or {}
        return int(data.get("camera", 0))
    except (OSError, ValueError, ImportError, TypeError):
        return 0


def vla_fake_photos():
    """Saved real photos usable as a fake camera (no webcam needed)."""
    import pathlib as _pl

    root = _pl.Path(__file__).resolve().parents[2]
    return sorted(p.name for p in
                  (root / "runs" / "stage9" / "real").glob("batch_*.png")
                  if not p.name.endswith(".full.png"))


def vla_probe_device():
    """Open the configured webcam once and report its state.

    Returns dict with open/width/height/fps/backend/error. Never
    raises: a missing camera or missing permission is data, shown
    in the panel so the user can tell whether capture will work.
    """
    dev = {"open": False, "width": 0, "height": 0, "fps": 0.0,
           "backend": "", "error": ""}
    try:
        import cv2 as _cv2
    except ImportError:
        dev["error"] = "OpenCV 없음"
        return dev
    cap = None
    try:
        cap = _cv2.VideoCapture(webcam_config())
        if not cap.isOpened():
            dev["error"] = ("카메라 열기 실패: 번호 %d, macOS 카메라 권한 "
                            "(설정 > 개인 정보 보호 및 보안 > 카메라) 확인"
                            % webcam_config())
            return dev
        dev["backend"] = cap.getBackendName()
        ok, frame = cap.read()
        if not ok or frame is None:
            dev["error"] = ("카메라 프레임 없음: 번호 %d, 다른 앱 사용 중 "
                            "확인" % webcam_config())
            return dev
        h, w = frame.shape[:2]
        dev.update({"open": True, "width": int(w), "height": int(h),
                    "fps": round(float(cap.get(_cv2.CAP_PROP_FPS)), 1)})
        return dev
    except Exception as exc:  # noqa: BLE001 - state, not a crash
        dev["error"] = "카메라 확인 실패: %s" % exc
        return dev
    finally:
        if cap is not None:
            cap.release()


def vla_camera_info():
    """Sources for the panel select + live device info/state."""
    return {"ok": True, "camera": webcam_config(),
            "fake": vla_fake_photos(), "dev": vla_probe_device()}


def vla_policy(ckpt_key):
    """Load each checkpoint once per server process, then reuse."""
    import torch
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import (
        SmolVLAPolicy)
    from src.vla.local_infer import RENAME_MAP

    key = ckpt_key if ckpt_key in VLA_CKPTS else "ft9b"
    if key not in _VLA_POLICIES:
        ckpt_dir = VLA_CKPTS[key]
        policy = SmolVLAPolicy.from_pretrained(ckpt_dir)
        policy.eval()
        if torch.cuda.is_available():
            device_name = "cuda"
        else:
            device_name = "cpu"
            policy = policy.to(device_name)
        preprocessor, postprocessor = make_pre_post_processors(
            policy.config, pretrained_path=ckpt_dir,
            preprocessor_overrides={
                "rename_observations_processor": {
                    "rename_map": RENAME_MAP},
                "device_processor": {"device": device_name},
            })
        _VLA_POLICIES[key] = (policy, preprocessor, postprocessor)
    return _VLA_POLICIES[key]


def vla_capture(source, fake_index=0):
    """One 256x256 frame: real webcam (OpenCV) or saved real photo."""
    import numpy as _np
    from PIL import Image as _Image
    import pathlib as _pl

    if source == "fake":
        photos = vla_fake_photos()
        if not photos:
            raise ValueError("가짜 사진 없음: runs/stage9/real/ 비어 있음")
        name = photos[int(fake_index or 0) % len(photos)]
        img = _Image.open(_pl.Path(__file__).resolve().parents[2] /
                          "runs" / "stage9" / "real" / name).convert("RGB")
        return _np.asarray(img), "fake:" + name
    import cv2 as _cv2

    cap = _cv2.VideoCapture(webcam_config())
    if not cap.isOpened():
        raise ValueError("카메라 열기 실패: 번호 %d, macOS 카메라 권한 "
                         "(설정 > 개인 정보 보호 및 보안 > 카메라)을 "
                         "확인하세요" % webcam_config())
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise ValueError("카메라 프레임 없음: 번호 %d, 다른 앱이 "
                         "카메라를 쓰고 있는지 확인하세요"
                         % webcam_config())
    rgb = _cv2.cvtColor(frame, _cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    side = min(h, w)
    top, left = (h - side) // 2, (w - side) // 2
    crop = _Image.fromarray(rgb[top:top + side,
                                left:left + side]).resize((256, 256))
    return _np.asarray(crop), "cam:%d" % webcam_config()


def vla_annotate(img_u8, tip_px, slot, dist_mm):
    """Draw the VLA tip projection + nearest slot on the frame."""
    import io as _io
    import base64 as _b64
    import numpy as _np
    from PIL import Image as _Image
    from PIL import ImageDraw as _Draw

    from src.vla.scene import SLOT_IDS
    img = _Image.fromarray(_np.asarray(img_u8, dtype=_np.uint8))
    d = _Draw.Draw(img)
    tx = max(0, min(255, int(tip_px[0])))
    ty = max(0, min(255, int(tip_px[1])))
    r = 8
    d.ellipse([tx - r, ty - r, tx + r, ty + r], outline=(255, 0, 0),
              width=3)
    d.text((tx + r + 2, ty - r), "VLA", fill=(255, 0, 0))
    for (sx, sy), sid in SLOT_IDS.items():
        w = 3 if sid == slot else 1
        col = (0, 255, 0) if sid == slot else (255, 255, 255)
        d.rectangle([sx - 8, sy - 8, sx + 8, sy + 8], outline=col,
                    width=w)
        if sid == slot:
            d.text((sx + 10, sy - 14), "%s %.0fmm" % (sid, dist_mm),
                   fill=(0, 255, 0))
    buf = _io.BytesIO()
    img.save(buf, format="PNG")
    return _b64.b64encode(buf.getvalue()).decode()


def vla_infer(source, target, task, ckpt_key, fake_index=0):
    """Capture -> preprocess -> shared drain -> hold pose + overlay."""
    import math as _math
    import time as _time

    from src.kin.fk import fk_model
    from src.kin.model import load_model
    from src.vla.drain import N_STEPS, drain_chunk
    from src.vla.scene import SLOT_IDS

    policy, pre, post = vla_policy(ckpt_key)
    key = ckpt_key if ckpt_key in VLA_CKPTS else "ft9b"
    img, origin = vla_capture(source, fake_index)
    assert tuple(img.shape) == (256, 256, 3), img.shape
    rep = json.loads((pathlib.Path(__file__).resolve().parents[2] /
                      "runs" / "vla" / "6.0" / "report.json")
                     .read_text(encoding="utf-8"))
    rest_q = [float(v) for v in rep["rest_q_deg"]]
    text = (task.strip() or
            "Point to the %s block." % (target or "red"))
    t1 = _time.monotonic()
    stacked, info = drain_chunk(policy, pre, post, img, rest_q, text,
                                n=N_STEPS, state_source="command")
    infer_s = _time.monotonic() - t1
    if stacked.shape != (50, 6):
        raise ValueError("VLA drain broken: %s" % (stacked.shape,))
    hold = [float(v) for v in stacked[-1]]
    doc = load_model(str(pathlib.Path(__file__).resolve().parents[2] /
                           "models" / "robot_inferred.json"))
    tcp = fk_model(hold, tip_doc(doc))["pos_mm"]
    best, best_d = None, 1e18
    for (sx, sy), sid in SLOT_IDS.items():
        dd = (tcp[0] - sx) ** 2 + (tcp[1] - sy) ** 2
        if dd < best_d:
            best, best_d = sid, dd
    dist = _math.sqrt(best_d)
    png = vla_annotate(img, vla_project_tip(tcp), best, dist)
    _VLA_LAST.update({"img": img, "origin": origin, "text": text,
                      "ckpt": key, "hold_q": hold, "slot": best,
                      "dist_mm": round(dist, 1), "infer_s": infer_s,
                      "state_source": info["state_source"]})
    return {"ok": True, "origin": origin, "task": text, "ckpt": key,
            "hold_q": {j: hold[i] for i, j in
                       enumerate(["J1", "J2", "J3", "J4", "J5", "J6"])},
            "tcp_mm": [round(float(v), 1) for v in tcp],
            "slot": best, "dist_mm": round(dist, 1),
            "infer_s": round(infer_s, 1),
            "state_source": info["state_source"], "png": png}


def vla_project_tip(tcp):
    """Project the hold-pose tip into 256px camera coords (sim camera)."""
    import numpy as _np

    from src.vla.scene import cam_frame, load_camera
    cam = load_camera()
    eye, _right, _up, _fwd, f = cam_frame(cam)
    p = _np.array([tcp[0], tcp[1], tcp[2]], float) - eye
    u, v = _project(p, cam, f)
    return [float(u), float(v)]


def _project(p, cam, f):
    """Pinhole projection with the sim camera frame (overlay-aligned)."""
    import numpy as _np

    from src.vla.scene import cam_frame
    eye, right, up, fwd, _f = cam_frame(cam)
    z = float(_np.dot(p, fwd))
    if z <= 1e-9:
        return 128.0, 128.0
    cx, cy = cam["image_px"][0] / 2.0, cam["image_px"][1] / 2.0
    return cx + f * float(_np.dot(p, right)) / z, \
        cy - f * float(_np.dot(p, up)) / z


def vla_trial_save(instruction, ckpt_key):
    """Save last inference (frame + verdict pending) to console_trials/."""
    import datetime as _dt
    import pathlib as _pl

    if "img" not in _VLA_LAST:
        raise ValueError("먼저 캡처 -> 추론을 실행하세요")
    root = _pl.Path(__file__).resolve().parents[2]
    outdir = root / "runs" / "stage9" / "console_trials"
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    from PIL import Image as _Image
    import numpy as _np
    _Image.fromarray(_np.asarray(_VLA_LAST["img"],
                                dtype=_np.uint8)).save(
        outdir / ("trial-%s.png" % stamp))
    rec = {"id": "trial-" + stamp, "time": stamp,
           "instruction": instruction, "ckpt": ckpt_key,
           "origin": _VLA_LAST.get("origin"),
           "task": _VLA_LAST.get("text"),
           "hold_q": _VLA_LAST.get("hold_q"),
           "slot": _VLA_LAST.get("slot"),
           "dist_mm": _VLA_LAST.get("dist_mm"),
           "infer_s": _VLA_LAST.get("infer_s"),
           "state_source": _VLA_LAST.get("state_source"),
           "verdict": None}
    (outdir / ("trial-%s.json" % stamp)).write_text(
        json.dumps(rec, ensure_ascii=False, indent=1), encoding="utf-8")
    return {"ok": True, "id": rec["id"]}


def vla_trial_vote(trial_id, correct):
    """Record human 맞음/틀림 on a saved trial."""
    import pathlib as _pl

    root = _pl.Path(__file__).resolve().parents[2]
    path = root / "runs" / "stage9" / "console_trials" / \
        ("%s.json" % trial_id)
    if not path.exists():
        raise ValueError("trial 없음: " + trial_id)
    rec = json.loads(path.read_text(encoding="utf-8"))
    rec["verdict"] = bool(correct)
    path.write_text(json.dumps(rec, ensure_ascii=False, indent=1),
                    encoding="utf-8")
    return {"ok": True, "id": trial_id, "verdict": rec["verdict"]}


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
    try:
        vla_policy("ft9b")
        print("vla: checkpoint ft9b preloaded")
    except Exception as e:  # noqa: BLE001 - lazy load on first infer
        print("vla: preload skipped (%s), loads on first infer" % e)
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
  <button id="langbtn" title="언어">EN</button>
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
<details><summary>VLA 웹캠 (실시간 지시 → 유지 자세)</summary>
<div class="wiz" id="wizvla">
<div>카메라 <select id="vla-source"><option value="real">실제 카메라</option><option value="fake">가짜 사진</option></select>
<span id="vla-caminfo"></span></div>
<div>지시 <select id="vla-target"><option value="red">빨강</option><option value="green">초록</option><option value="yellow">노랑</option><option value="blue">파랑</option></select>
<input id="vla-task" size="28" placeholder="문장 지시 (비우면 색 기준)">
체크포인트 <select id="vla-ckpt"><option value="ft9b">미세조정 후</option><option value="6000">전 (6,000)</option></select></div>
<div><button id="vla-run">캡처 → 추론</button>
<span id="vla-msg">대기 중</span></div>
<div><img id="vla-img" style="max-width:100%;display:none" alt="VLA 결과"></div>
<div id="vla-info"></div>
<div><button id="vla-send" disabled>로봇으로 보내기 (저속)</button>
<button id="vla-rest">휴식 자세로</button>
<button id="vla-yes" disabled>맞음</button>
<button id="vla-no" disabled>틀림</button>
<span id="vla-trial"></span></div>
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
<div><button id="home-all">전부 0으로</button>
<span id="home-all-msg"></span></div>
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
  document.getElementById("glerr").textContent = T("오류: ") + e.message;
});

// UI language: Korean source strings are translated to English at
// display time. Static markup is swapped by applyLang(); dynamic
// strings go through T(). Default Korean; toggle persists.
let LANG = "ko";
try {
  if (window.localStorage.getItem("console-lang") === "en") LANG = "en";
} catch (err) { /* private mode: stay Korean */ }
const STR = {
  // Top bar + view menu.
  "메인": "Main",
  "보정": "Calibration",
  "검증": "Verify",
  "모드": "Mode",
  "좌:": "L:",
  "우:": "R:",
  "래치": "Latch",
  "하트비트": "Heartbeat",
  "호밍": "Homing",
  "호밍: ": "Homing: ",
  "로봇 연결됨(가짜)": "Robot connected (fake)",
  "로봇 연결됨": "Robot connected",
  " 저장 실패": " save failed",
  "안됨": "not yet",
  "됨(": "done (",
  "유지 자세로 보내기": "Send hold pose",
  "보정 확정됨(": "Calibration confirmed (",
  "흐름=": "flow=",
  "jog 실패: ": "jog failed: ",
  " / 거리 ": " / distance ",
  "로봇": "Robot",
  "로딩…": "Loading…",
  "보정 페이지에서 호밍하세요": "Please home on the calibration page",
  "래치 해제": "Release latch",
  "래치 해제됨": "Latch released",
  "래치 해제 실패: ": "Latch release failed: ",
  "표시:": "Show:",
  "매트": "Mat",
  "기즈모": "Gizmo",
  "TCP→목표": "TCP→target",
  "도달범위": "Reach",
  "이전자세": "Previous pose",
  "실측겹침": "Overlay",
  "손목방향": "Wrist dir",
  "모델": "Model",
  "기본 도형": "Primitives",
  "LDraw 메시(비공개)": "LDraw mesh (private)",
  "화면 맞춤": "Fit view",
  "이동": "Move",
  "회전": "Rotate",
  // Target + IK + execute row.
  "매트 목표": "Mat target",
  "좌표(mm)": "Coords (mm)",
  "집게가 아래를 향함 (위치+집게 방향)": "Gripper faces down (position+gripper)",
  "기즈모를 TCP로": "Gizmo to TCP",
  "닿는 목표": "Reachable target",
  "타겟→현재EE": "Target→current EE",
  "실행": "Run",
  "수렴": "Converged",
  "미수렴(최근접)": "Not converged (nearest)",
  "(최근접 추종)": "(nearest follow)",
  "(테이블 클램프)": "(table clamp)",
  "오차": "err",
  "제한": "limits",
  "없음": "none",
  "목표(베이스)": "Target (base)",
  "목표(테이블)": "Target (table)",
  "목표로": "Set target",
  "목표": "Target",
  "미리보기 확인 후": "After checking the preview",
  "로봇으로 보내기": "Send to robot",
  "정지": "Stop",
  "대기 중": "Idle",
  // Twin.
  "디지털 트윈 (실물 블록 → 가상 이미지 → VLA)": "Digital twin (real blocks → virtual image → VLA)",
  "실물 블록을 슬롯 S1~S6에만 놓고, 색을 아래에 입력. 실물 카메라는 쓰지 않음.": "Place real blocks only on slots S1–S6 and enter colours below. No real camera used.",
  "색 저장": "Save colours",
  "색 저장됨": "Colours saved",
  "트윈 실행": "Run twin",
  "가상 이미지": "Virtual image",
  "유지 자세로 보내기 (저속)": "Send hold pose (slow)",
  "휴식 자세로": "Rest pose",
  "휴식 자세 이동 완료": "Moved to rest pose",
  "휴식 자세로 저속 이동할까요?": "Move to the rest pose at low speed?",
  // VLA webcam.
  "VLA 웹캠 (실시간 지시 → 유지 자세)": "VLA webcam (live instruction → hold pose)",
  "카메라": "Camera",
  "실제 카메라": "Real camera",
  "가짜 사진": "Fake photos",
  "지시": "Instruction",
  "빨강": "Red",
  "초록": "Green",
  "노랑": "Yellow",
  "파랑": "Blue",
  "문장 지시 (비우면 색 기준)": "Sentence instruction (blank = by colour)",
  "체크포인트": "Checkpoint",
  "미세조정 후": "After fine-tune",
  "전 (6,000)": "Before (6,000)",
  "캡처 → 추론": "Capture → infer",
  "로봇으로 보내기 (저속)": "Send to robot (slow)",
  "맞음": "Correct",
  "틀림": "Wrong",
  "추론 중…": "Inferring…",
  "추론 완료": "Inference done",
  "추론": "Inference",
  // Demo.
  "데모 순서": "Demo sequence",
  "호밍 확인 → 안전자세 2개 → 매트 점(아래 번호) → 원위치. 느리게, 한 단계씩 확인.": "Check homing → 2 safe poses → mat points (numbers below) → back. Slowly, confirm each step.",
  "매트 점": "Mat points",
  "데모 순서 시작": "Start demo",
  // Joints / diag / log chrome.
  "관절·모터·IK": "Joints · motors · IK",
  "실시간 관절값 반영": "Live joint values",
  "주황 구슬을 드래그하면 뼈대가 따라옵니다": "Drag the orange ball and the skeleton follows",
  "관절": "Joint",
  "관절각": "Joint angle",
  "모터각": "Motor angle",
  "홈": "Homed",
  "방향": "Direction",
  "센서": "Sensor",
  "조그": "Jog",
  "브릭/포트": "Brick/port",
  "완료": "done",
  "미완": "incomplete",
  "확인됨": "confirmed",
  "방향 미확인": "Direction unconfirmed",
  "연결": "connected",
  "끊김": "disconnected",
  "걸림": "engaged",
  "읽김": "unread",
  "상태기준 연결=": "Status baseline connected=",
  "상태변경 연결=": "Status changed connected=",
  " 래치=": " latch=",
  "상태수신전": "Waiting for status",
  "하트비트: 실패": "Heartbeat: failed",
  "초 전 갱신": "s ago",
  "하트비트: ": "Heartbeat: ",
  "E-STOP 전송됨": "E-STOP sent",
  "E-STOP 실패: ": "E-STOP failed: ",
  "정지(E-STOP) 전송됨": "Stop (E-STOP) sent",
  "정지 실패: ": "Stop failed: ",
  "로그": "Log",
  // Calibration page.
  "세션 보정 (호밍 반복 생략용)": "Session calibration (skip homing)",
  "한 번에 보정": "Calibrate all",
  "주차 (저속 이동+저장)": "Park (slow move+save)",
  "주차 자세에서 복원": "Restore from park",
  "확인 자세로 이동": "Go to check pose",
  "맞음 (보정 확정)": "Correct (confirm calibration)",
  "관절별 호밍. J5/J6은 표시 맞춤(현재 위치를 0으로 기록). 손목 기준 그림:": "Home each joint. J5/J6 align by marking (record current position as 0). Wrist reference:",
  "손목 기준": "Wrist reference",
  "VLA 결과": "VLA result",
  "전부 0으로": "Zero all",
  "메인으로 돌아가기": "Back to main",
  "M4 방향 마법사": "M4 direction wizard",
  "+5° 시도": "+5° try",
  "반대": "Reverse",
  "M5 범위 마법사": "M5 range wizard",
  "M6 자세 재현": "M6 pose replay",
  "재현": "Replay",
  "포트 진단": "Port diagnostics",
  "설정(경계값)": "Settings (limits)",
  // Verify page + misc chrome.
  "IK 시험 결과": "IK test results",
  "GT 뼈대 비교는 --private-visuals일 때만 표시됩니다.": "GT skeleton comparison only with --private-visuals.",
  "로봇 전환 실패: ": "Robot switch failed: ",
  "테이블 설정 로드 실패, 기본값 사용": "Table config load failed, using defaults",
  "LDraw 메시 없음(기본 도형 유지)": "No LDraw mesh (keeping primitives)",
  "LDraw 메시 로딩 중…": "Loading LDraw mesh…",
  "LDraw 링크 없음 (기본 도형 유지)": "No LDraw link (keeping primitives)",
  "LDraw 메시 로드 완료": "LDraw mesh loaded",
  "LDraw 메시": "LDraw mesh",
  "메시 로드 실패: ": "Mesh load failed: ",
  " (기본 도형 유지)": " (keeping primitives)",
  "오류: ": "Error: ",
  "OpenCV 없음": "No OpenCV",
  // Common verbs / outcomes.
  "실패: ": "Failed: ",
  "실패(거부): ": "Failed (refused): ",
  "거부: ": "Refused: ",
  "거부": "refused",
  "취소": "cancel",
  "취소됨": "cancelled",
  "중단: ": "Aborted: ",
  "미완료": "incomplete",
  "확인": "Confirm",
  "저장": "Save",
  "저장됨": "saved",
  "기록": "Record",
  "설정": "Settings",
  "적용": "Apply",
  "로드": "Load",
  // Send / twin / VLA flow.
  "전송": "send",
  "전송됨": "sent",
  "보내기": "Send",
  "트윈 전송: ": "Twin send: ",
  "트윈 전송 대기(다시 누르면 전송)": "Twin send armed (press again to send)",
  "다시 누르면 저속(15도/s)으로 전송": "Press again to send at slow speed (15°/s)",
  "유지 자세 없음(트윈 실행 먼저)": "No hold pose (run twin first)",
  "유지=": "hold=",
  "정책=": "policy=",
  "트윈 실행: 정책=": "Twin run: policy=",
  "트윈 휴식: ": "Twin rest: ",
  "유지 자세 이동 완료, 최대오차 ": "Hold pose move done, max error ",
  "VLA 추론: ": "VLA inference: ",
  "VLA 추론 실패: ": "VLA inference failed: ",
  "VLA 보내기 실패: ": "VLA send failed: ",
  "VLA 보내기 완료": "VLA send done",
  "VLA 판정: ": "VLA verdict: ",
  "VLA 휴식: ": "VLA rest: ",
  "로봇으로 보내기 실패: ": "Send-to-robot failed: ",
  "로봇으로 보내기 완료": "Sent to robot",
  "보내기 거부: ": "Send refused: ",
  "보내기 취소(대이동 확인 거부)": "Send cancelled (large-move confirm refused)",
  "보내기 취소(호밍 미완료 확인 거부)": "Send cancelled (homing-incomplete confirm refused)",
  "대이동": "large move",
  "이동량": "travel",
  "브릭이 연결되지 않아 보낼 수 없습니다": "Bricks are not connected, cannot send",
  "호밍 안 됨(": "Homing incomplete (",
  "): 그래도 VLA 자세를 보내시겠습니까?": "): send the VLA pose anyway?",
  "이동량 ": "Travel ",
  "°: 저속으로 이동할까요?": "°: move at low speed?",
  "보내기 가능 (저속 ": "Ready to send (slow ",
  "경고: 호밍 미완료(": "Warning: homing incomplete (",
  ") — 클릭 시 확인 후 전송 (저속 ": ") — confirm at click, then send (slow ",
  "호밍 미완료 관절: ": "Homing incomplete joints: ",
  "\\n보정 없이 저속으로 보낼까요?": "\\nSend at low speed without calibration?",
  "한 관절이 ": "One joint moves ",
  "° 움직입니다. 진행할까요?": "°. Proceed?",
  "유효한 미리보기 없음": "No valid preview",
  "범위 위반: ": "Range violation: ",
  "범위 ": "Range ",
  "(저속 ": "(slow ",
  "미리보기": "Preview",
  "본체": "body",
  " | 가상 추종(본체 이동)": " | virtual follow (body move)",
  // IK / motor messages.
  "IK 실패: ": "IK failed: ",
  "IK 상태: ": "IK status: ",
  "IK 수렴": "IK converged",
  "IK 수렴 위치오차": "IK converged, position error",
  "위치오차 ": "position error ",
  "자세오차 ": "orientation error ",
  " / 오차 ": " / error ",
  " / 제한 ": " / limits ",
  " / 목표(테이블) [": " / target (table) [",
  "모터 미리보기: ": "Motor preview: ",
  " / 범위: ": " / range: ",
  "모터 ": "Motor ",
  " | 범위 ": " | range ",
  " | 모터 ": " | motors ",
  "미리보기 표시 실패": "Preview display failed",
  "미리보기 실패: ": "Preview failed: ",
  "GT 표시 불가: ": "GT display unavailable: ",
  "IK 상태 표시 실패: ": "IK status display failed: ",
  "기즈모를 현재 TCP로 이동": "Move gizmo to current TCP",
  "닿는 목표 실패: ": "Reachable-target failed: ",
  "닿는 목표: [": "Reachable target: [",
  "매트 없음(비공개)": "No mat (private)",
  "매트 ": "Mat ",
  "가상 이미지 생성+VLA 추론 중…": "Generating virtual image + VLA inference…",
  // Homing wizard.
  "결과: ": "Result: ",
  "0으로 표시됨(offset ": "marked as 0 (offset ",
  "미실행: ": "Not executed: ",
  "이동: 요청 ": "Move: requested ",
  "각도": "angle",
  "각도로 이동": "Go by angle",
  "각도 입력이 숫자가 아닙니다": "Angle input is not a number",
  "현재를 0으로": "Set current as 0",
  "전 관절 각도 필요: ": "All joint angles needed: ",
  "이동 완료, 최대오차 ": "Move done, max error ",
  "호밍 수동이동 ": "Homing manual move ",
  "호밍 명세 실패: ": "Homing spec failed: ",
  "호밍 명세": "Homing spec",
  "호밍 로드 실패": "Homing load failed",
  "현재 자세를 전 관절의 0으로 기록할까요?": "Record the current pose as 0 for all joints?",
  "전부 0으로 취소됨": "Zero-all cancelled",
  "전부 0으로 기록됨": "Zero-all recorded",
  "전부 0으로: ": "Zero-all: ",
  "상태 조회 실패": "Status query failed",
  // M4/M5/M6 + settings + sessions.
  "정방향 확정": "forward confirmed",
  "모델과 실물을 비교하세요": "Compare the model with the hardware",
  "반대 → 부호 뒤집어 저장됨(비율 ": "reversed → sign flipped and saved (ratio ",
  "비율": "ratio",
  "이동: 요청 {a}도, 실이동 {b}도": "Move: requested {a}°, actual {b}°",
  "이동 완료, 최대오차 {x}도": "Move done, max error {x}°",
  "5A 시뮬": "5A sim",
  "실측[lo,hi]": "measured [lo,hi]",
  "채택": "adopted",
  "(실측)": "(measured)",
  "(시뮬)": "(sim)",
  "여유 ": "margin ",
  " (최대±5mm)": " (max ±5mm)",
  " (최대±5°)": " (max ±5°)",
  "lo로": "as lo",
  "hi로": "as hi",
  " 기록: ": " recorded: ",
  "패키지 ": "package ",
  "자세 로드 실패: ": "Pose load failed: ",
  "자세": "Pose",
  "목표각": "Target angle",
  "최근 오차": "Recent error",
  "이전 자세 표시 실패": "Previous-pose display failed",
  "재현 완료, 최대오차 ": "replay done, max error ",
  "설정 로드 실패: ": "Settings load failed: ",
  "항목": "Item",
  "값": "Value",
  "변경": "Change",
  "패키지 여유 ": "Package margin ",
  "°, 속도 상한 ": "°, speed cap ",
  "보고서 로드 실패: ": "Report load failed: ",
  "아직 검증 보고서가 없습니다(시뮬레이션 산출 후 표시)": "No verification report yet (shown after simulation)",
  "종류": "Kind",
  "TCP 오차mm": "TCP err mm",
  "밑동 관절": "Base joints",
  "통과": "pass",
  "사유": "reason",
  "닿는 목표 통과 ": "Reachable-target pass ",
  " / 닿지 않는 목표 ": " / unreachable ",
  "증거: ": "Evidence: ",
  "GT 뼈대: ": "GT skeleton: ",
  "사용 가능 — 메인 3D의 GT 표시로 겹쳐 비교하세요": "available — overlay-compare with the main 3D GT view",
  "비공개(--private-visuals 필요)": "private (--private-visuals needed)",
  "GT 뼈대: 확인 실패": "GT skeleton: check failed",
  "로드 실패": "Load failed",
  "(비공개)": "(private)",
  "비공개": "private",
  "미연결": "disconnected",
  "가상만(브릭 미연결)": "Virtual only (bricks disconnected)",
  "브릭 미연결(가상만)": "Bricks disconnected (virtual only)",
  "가상만": "Virtual only",
  "브릭": "Brick",
  "(가짜)": "(fake)",
  "가상": "virtual",
  "가짜": "fake",
  "연속성=": "continuity=",
  "확정=": "confirmed=",
  "확정 불가: ": "Cannot confirm: ",
  "(해제 필요)": "(needs release)",
  "좌표 입력이 숫자가 아닙니다": "Coordinate input is not a number",
  "확정됨": "confirmed",
  "저장=": "saved=",
  "현재 가동=": "current uptime=",
  "저장 가동=": "saved uptime=",
  "연결 ok": "connected ok",
  "복귀 완료": "returned",
  // Camera errors (server side, translated client side).
  "카메라 열기 실패": "Cannot open camera",
  "카메라 프레임 없음": "No camera frame",
  "카메라 확인 실패: ": "Camera check failed: ",
  "카메라 정보 실패: ": "Camera info failed: ",
  "번호": "No.",
  "다른 앱이 카메라를 쓰고 있는지 확인하세요": "check whether another app uses the camera",
  "다른 앱 사용 중 확인": "check other-app use",
  "macOS 카메라 권한": "macOS camera permission",
  "설정 > 개인 정보 보호 및 보안 > 카메라": "Settings > Privacy & Security > Camera",
  "확인하세요": "please check",
  "가짜 사진 없음: runs/stage9/real/ 비어 있음": "No fake photos: runs/stage9/real/ is empty",
  "먼저 캡처 -> 추론을 실행하세요": "Run capture → infer first",
  "trial 없음: ": "No trial: ",
  "알 수 없음": "unknown",
  "연결됨": "Connected",
  "끊김: ": "Disconnected: ",
  "언어": "Language",
  // Follow-ups found while wrapping call sites.
  "완": "Y",
  "미": "N",
  "초 ": "s ",
  " 목표=": " target=",
  "확인: ": "Confirm: ",
  "가상 추종": "virtual follow",
  "(본체 이동)": "(body move)",
  "매트 점 없음": "No mat point",
  "IK 실패(도달 불가 표시)": "IK failed (unreachable shown)",
  "overview 실패": "overview failed",
  "상태 수신 중": "Receiving status",
  "범위 로드 실패: ": "Range load failed: ",
  "현재 ": "current ",
  "(가상)": "(virtual)",
  "호밍 미완료": "Homing incomplete",
  "데모 {n}: {r}": "Demo {n}: {r}",
  "데모 중단: ": "Demo aborted: ",
  "다음 단계: {name}\\n진행할까요?": "Next step: {name}\\nProceed?",
  "데모 실패: ": "Demo failed: ",
  " — 데모 끝": " — demo over",
  "안전자세 {p} (저속)": "Safe pose {p} (slow)",
  "매트 {n}번으로 이동 (저속)": "Move to mat {n} (slow)",
  "매트 {n}: [{x}, {y}, {z}]mm": "Mat {n}: [{x}, {y}, {z}]mm",
  "원위치 (safe01)": "Back home (safe01)",
  "호밍 상태 확인": "Check homing status",
  "최대오차 {x}°": "max error {x}°",
  "오차 {x}mm": "error {x}mm",
  "번호 {n} / {s} / 가짜 {m}장": "No. {n} / {s} / fake {m} photos",
  "하트비트: {n}초 전 갱신": "Heartbeat: {n}s ago",
  "한 번에 보정: J1~J4 센서 호밍 → 손목 → 확인자세. 진행할까요?": "Calibrate all: J1–J4 sensor homing → wrist → check pose. Proceed?",
  "주차: 휴식 자세로 저속 이동 후 세션 저장. 진행할까요?": "Park: slow-move to rest pose, then save session. Proceed?",
  "주차 저장됨(": "Park saved (",
  "복원됨(연속성=": "Restored (continuity=",
  "확인 자세로 이동됨. 3D 실측겹침을 보고 '맞음'을 누르세요.": "Moved to check pose. Look at the 3D overlay and press 'Correct'.",
  "확인 자세 이동, 최대편차 ": "Check-pose move, max deviation ",
  "설정 ": "Setting ",
};
const I18N_KEYS = Object.keys(STR).sort((a, b) => b.length - a.length);
function T(s, vars) {
  // Template mode: T("…{x}…", {x}) keeps unit/word order right.
  if (vars !== undefined) {
    const tpl = (LANG === "en" && STR[s] !== undefined) ? STR[s] : s;
    return String(tpl).replace(/\\{(\\w+)\\}/g, (m, k) => vars[k]);
  }
  if (LANG !== "en" || typeof s !== "string") return s;
  let out = s;
  for (const k of I18N_KEYS) {
    if (out.indexOf(k) === -1) continue;
    out = out.split(k).join(STR[k]);
  }
  return out;
}
function applyLang() {
  const en = LANG === "en";
  document.documentElement.lang = en ? "en" : "ko";
  const btn = document.getElementById("langbtn");
  if (btn) btn.textContent = en ? "한" : "EN";
  const walker = document.createTreeWalker(
    document.body, NodeFilter.SHOW_TEXT);
  const nodes = [];
  while (walker.nextNode()) nodes.push(walker.currentNode);
  for (const n of nodes) {
    const tag = (n.parentNode && n.parentNode.tagName) || "";
    if (tag === "SCRIPT" || tag === "STYLE") continue;
    if (n._ko === undefined) n._ko = n.nodeValue;
    n.nodeValue = en ? T(n._ko) : n._ko;
  }
  const attrs = document.querySelectorAll(
    "[placeholder],[title],[alt]");
  attrs.forEach((el) => {
    for (const a of ["placeholder", "title", "alt"]) {
      const v = el.getAttribute(a);
      if (v === null) continue;
      const k = "_ko_" + a;
      if (el[k] === undefined) el[k] = v;
      el.setAttribute(a, en ? T(el[k]) : el[k]);
    }
  });
  try {
    window.localStorage.setItem("console-lang",
      en ? "en" : "ko");
  } catch (err) { /* private mode: do not persist */ }
}

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
      o.textContent = id === "tbot" ? T("T-Bot (가상)") : "MR JOS";
      if (id === r.robot) o.selected = true;
      sel.appendChild(o);
    }
    sel.onchange = async () => {
      const rr = await post("/api/robot/select", {robot: sel.value});
      if (rr.ok) location.reload();
      else {
        alert(T("로봇 전환 실패: ") + (rr.error || "?"));
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
    log(T("테이블 설정 로드 실패, 기본값 사용"));
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
  // Initial pose: the park/rest pose (mirrors
  // src/console/calib_session.py PARK_Q_DEG), a verified ㄱ shape
  // with the TCP well above the table. poses10 pose01 put the
  // initial IK target ~190 mm UNDER the table (bugs/basic_bottom).
  const PARK_Q = {J1: 25.58, J2: 20.93, J3: 17.17,
                  J4: -43.58, J5: -87.46, J6: 60.0};
  // Rigid links: link i is fixed in its own PoE frame F_i, so its
  // capsule endpoints (joint anchors at the zero pose) are constant
  // in that frame and the mesh just takes F_i each tick. The anchor
  // p_i lies on screw axis i, hence F_i^-1 F_{i-1} p_i == p_i.
  const q0 = m.joints.map((j) => qNative(j.id, PARK_Q[j.id] || 0));
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
    qdeg[j.id] = (j.id in PARK_Q) ? PARK_Q[j.id] : 0;
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
  const showMesh = VSRC === "ldraw" && MESHES.ready;
  const showP = ghostOn && !showMesh;
  const showL = ghostOn && showMesh;
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
    "" : T("LDraw 메시 없음(기본 도형 유지)");
}
function applyVis() {
  // Keep the primitives until LDraw meshes are actually ready:
  // switching the visual must never leave an empty stage during
  // the pack load (bugs/MR_jos_hw_disappear).
  const showMesh = VSRC === "ldraw" && MESHES.ready;
  const prim = !showMesh;
  for (const g of SEGS) g.mesh.visible = prim;
  for (const b of NODES) b.visible = prim;
  const st = window._scene;
  if (st.baseMesh) st.baseMesh.visible = prim;
  if (st.eeAxes) st.eeAxes.visible = prim;
  for (const g of MESHES.groups) if (g) g.visible = showMesh;
  document.getElementById("vsrc-name").textContent =
    VSRC === "primitives" ? T("기본 도형") : T("LDraw 메시");
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
      T("GLTFLoader 없음 (기본 도형 유지)");
    document.getElementById("vsrc").value = "primitives";
    VSRC = "primitives";
    MESHES.loading = false;
    applyVis();
    return;
  }
  const loader = new GLTFLoader();
  document.getElementById("vmsg").textContent = T("LDraw 메시 로딩 중…");
  // Link ids come from /api/visual (D0..D6 MR JOS, D0..D3 T-Bot):
  // never a hardcoded 7-link list.
  const ids = Object.keys(MESHES.urls);
  if (!ids.length) {
    document.getElementById("vmsg").textContent =
      T("LDraw 링크 없음 (기본 도형 유지)");
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
        document.getElementById("vmsg").textContent = "";
        applyVis();
        log(T("LDraw 메시 로드 완료"));
      }
    }, undefined, (err) => {
      document.getElementById("vmsg").textContent =
        T("메시 로드 실패: ") + lid + T(" (기본 도형 유지)");
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
  if (VSRC === "ldraw" && MESHES.ready && MESHES.groups[0]) {
    MESHES.groups[0].matrix.identity();
    MESHES.groups[0].matrixWorldNeedsUpdate = true;
    for (let i = 1; i <= MODEL.joints.length; i++) {
      if (!MESHES.groups[i]) continue;
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
    _tS.z.toFixed(1) + T("] / 목표 [") + _aS.x.toFixed(1) + ", " +
    _aS.y.toFixed(1) + ", " + _aS.z.toFixed(1) + T("] / 거리 ") +
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
    el("st-left", T("좌:") + (L === null ? T("끊김") : T("연결")),
       L === null ? "bad" : "ok");
    el("st-right", T("우:") + (R === null ? T("끊김") : T("연결")),
       R === null ? "bad" : "ok");
    const latched = (L === true || R === true);
    el("st-latch", T("래치") + ": " + (latched ? T("걸림") : T("없음")),
       latched ? "bad" : "ok");
    const allMm = ((MODEL && MODEL.joints) || []).length > 0 &&
      MODEL.joints.every((k) => k.unit === "mm");
    const uj = allMm ? "(mm)" : "(°)";
    let html = "<table><tr><th>" + T("관절") + "</th><th>" + T("관절각") + uj + "</th>" +
      "<th>" + T("모터각") + "</th><th>" + T("홈") + "</th><th>" + T("방향") + "</th><th>" + T("센서") + "</th>" +
      "<th>" + T("조그") + "</th></tr>";
    const fEl = document.getElementById("follow-live");
    const follow = fEl ? fEl.checked : false;
    const jtEl = document.getElementById("joints");
    for (const j of o.joints) {
      // Null readings (no encoders, e.g. virtual-only T-Bot) must not
      // clobber the IK-follow pose back to zero on every poll.
      if (follow && j.joint_deg !== null && j.joint_deg !== undefined)
        qdeg[j.joint] = j.joint_deg;
      const jog = j.direction === "confirmed" ? ""
        : (uOf(j.joint) === "mm" ? T(" (최대±5mm)") : T(" (최대±5°)"));
      html += "<tr><td>" + j.joint + "</td><td>" +
        (j.joint_deg === null ? "?" : j.joint_deg.toFixed(1)) + "</td><td>" +
        (j.motor_deg === null ? "?" : j.motor_deg.toFixed(1)) + "</td><td>" +
        (j.homed ? T("완료") : "−") + "</td><td>" +
        (j.direction === "confirmed" ? T("확인됨")
         : T("<span class='unconf'>방향 미확인</span>")) + "</td><td>" +
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
        if (!r.ok) alert(T("jog 실패: ") + r.error);
      };
    });
    // Homing dots + calibration link + mode banner + diag.
    const dots = o.joints.map((j) =>
      j.joint + (j.homed ? "●" : "○")).join(" ");
    el("homedots", T("호밍: ") + dots, "");
    const unhomed = o.joints.some((j) => !j.homed);
    const cl = document.getElementById("cal-link");
    if (cl) cl.style.display = unhomed ? "" : "none";
    try {
      const md = await post("/api/mode", {});
      if (md.ok) VIRT = !!md.virtual;
    } catch (err) { /* keep last VIRT */ }
    // isConnected() is robot-aware (T-Bot virtual-only never connects);
    // identical to the latch check for MR JOS.
    const conn = isConnected();
    el("mode", (conn ? (VIRT ? T("로봇 연결됨(가짜)") : T("로봇 연결됨"))
                    : T("가상만(브릭 미연결)")) +
      (ROBOT === "tbot" ? " T-Bot" : ""), conn ? "ok" : "bad");
    // Status-change record: connection, latch, per-joint
    // homed/read state. Baseline once, then only on change, so
    // every transition stays in the log without a line per poll.
    const stOk = !!conn;
    const stLatch = !!(L === true || R === true);
    const stJoints = (o.joints || []).map((j) =>
      j.joint + "=" + (j.joint_deg === null || j.joint_deg === undefined
        ? T("읽김") : Number(j.joint_deg).toFixed(1)) + "/" +
      (j.homed ? T("완") : T("미"))).join(" ");
    const stSig = (stOk ? "1" : "0") + (stLatch ? "1" : "0") +
      (o.joints || []).map((j) =>
        (j.homed ? "1" : "0") +
        ((j.joint_deg === null || j.joint_deg === undefined) ? "0" : "1")
      ).join("");
    if (window._lastStSig === undefined) {
      log(T("상태기준 연결=") + (stOk ? "ok" : "bad") +
        T(" 래치=") + (stLatch ? T("걸림") : T("없음")) + " " + stJoints);
    } else if (window._lastStSig !== stSig) {
      log(T("상태변경 연결=") + (stOk ? "ok" : "bad") +
        T(" 래치=") + (stLatch ? T("걸림") : T("없음")) + " " + stJoints);
    }
    window._lastStSig = stSig;
    const dg = document.getElementById("diag");
    if (dg) {
      let dh = "<table><tr><th>" + T("관절") + "</th><th>" + T("브릭/포트") + "</th>" +
        "<th>" + T("모터각") + "</th><th>" + T("관절각") + "</th><th>" + T("홈") + "</th><th>" + T("센서") + "</th>" +
        "<th>" + T("래치") + "</th></tr>";
      for (const j of o.joints) {
        dh += "<tr><td>" + j.joint + "</td><td>" + j.brick + "/" +
          j.port + "</td><td>" +
          (j.motor_deg === null ? "?" : j.motor_deg.toFixed(1)) +
          "</td><td>" +
          (j.joint_deg === null ? "?" : j.joint_deg.toFixed(1)) +
          "</td><td>" + (j.homed ? T("완료") : "−") + "</td><td>" +
          (j.sensor === null || j.sensor === undefined ? "−" :
           j.sensor + ":" + JSON.stringify(j.sensor_state)) +
          "</td><td>" + (latched ? T("걸림") : T("없음")) + "</td></tr>";
      }
      dg.innerHTML = dh + "</table>";
    }
    refreshSend();
  } catch (e) {
    document.getElementById("st-hb").textContent = T("하트비트: 실패");
    document.getElementById("st-hb").className = "bad";
    return;
  }
  const ago = ((Date.now() - lastOk) / 1000).toFixed(0);
  const hb = document.getElementById("st-hb");
  hb.textContent = T("하트비트: {n}초 전 갱신", {n: ago});
  hb.className = "ok";
}
document.getElementById("estop").onclick = async () => {
  const r = await post("/api/estop", {});
  alert(r.ok ? T("E-STOP 전송됨") : T("E-STOP 실패: ") + (r.error || "?"));
  log(r.ok ? T("E-STOP 전송됨") : T("E-STOP 실패: ") + (r.error || "?"));
};
document.getElementById("btn-clear").onclick = async () => {
  const r = await post("/api/clear", {});
  log(r.ok ? T("래치 해제됨") : T("래치 해제 실패: ") + (r.error || "?"));
};
document.getElementById("w4-try").onclick = async () => {
  const j = document.getElementById("w4-joint").value;
  const r = await post("/api/jog", {joint: j, delta: 5});
  document.getElementById("w4-msg").textContent =
    r.ok ? T("모델과 실물을 비교하세요") : T("실패: ") + r.error;
};
document.getElementById("w4-ok").onclick = async () => {
  const j = document.getElementById("w4-joint").value;
  const r = await post("/api/direction", {joint: j, verdict: "correct"});
  document.getElementById("w4-msg").textContent =
    r.ok ? j + T(" 정방향 확정") : T("실패: ") + r.error;
};
document.getElementById("w4-rev").onclick = async () => {
  const j = document.getElementById("w4-joint").value;
  const r = await post("/api/direction", {joint: j, verdict: "reversed"});
  document.getElementById("w4-msg").textContent =
    r.ok ? j + T(" 반대 → 부호 뒤집어 저장됨(비율 ") + r.ratio + ")" : T("실패: ") + r.error;
  log("M4 " + j + " reversed: " + (r.ok ? T("비율 ") + r.ratio : r.error));
};
function statusCtx() {
  if (!lastOv) return T("상태수신전");
  const L = lastOv.latched.left, R = lastOv.latched.right;
  const lr = T("좌:") + (L === null ? T("끊김") : (L ? T("래치") : T("연결"))) +
    T(" 우:") + (R === null ? T("끊김") : (R ? T("래치") : T("연결")));
  const js = (lastOv.joints || []).map((j) =>
    j.joint + "=" + (j.joint_deg === null || j.joint_deg === undefined
      ? T("읽김") : Number(j.joint_deg).toFixed(1))).join(" ");
  return lr + " " + js;
}
function log(msg) {
  const el = document.getElementById("log");
  if (!el) return;
  const d = document.createElement("div");
  // Every line carries live left/right + joint context so a failure
  // line alone shows the cause; status lines already carry it.
  const ctx = (msg.indexOf("상태기준") === 0 ||
               msg.indexOf("상태변경") === 0 ||
               msg.indexOf("Status baseline") === 0 ||
               msg.indexOf("Status changed") === 0)
    ? "" : " [" + statusCtx() + "]";
  d.textContent = new Date().toLocaleTimeString() + " " + msg + ctx;
  el.prepend(d);
  while (el.children.length > 50) el.lastChild.remove();
}
async function refreshRanges() {
  const r = await post("/api/ranges", {});
  if (!r.ok) { log(T("범위 로드 실패: ") + r.error); return; }
  const f1 = (v) => (v === null || v === undefined ? "−"
    : (+v).toFixed(1));
  const rng = (a) => "[" + f1(a[0]) + "," + f1(a[1]) + "]";
  let html = "<table><tr><th>" + T("관절") + "</th><th>" + T("5A 시뮬") + "</th>" +
    "<th>" + T("실측[lo,hi]") + "</th><th>" + T("채택") + "</th><th>" + T("기록") + "</th></tr>";
  for (const j of r.joints) {
    const cur = (qdeg[j.joint] || 0).toFixed(1);
    const uu = uOf(j.joint) === "mm" ? "mm" : "°";
    html += "<tr><td>" + j.joint + "</td><td>" + rng(j.sim) +
      "</td><td>" + rng(j.measured) + "</td><td>" + rng(j.adopted) +
      (j.adopted_source === "measured" ? T("(실측)") : T("(시뮬)")) +
      "</td><td>" + T("현재 ") + cur + uu + " " +
      "<button data-j='" + j.joint + "' data-e='lo'>" + T("lo로") + "</button>" +
      "<button data-j='" + j.joint + "' data-e='hi'>" + T("hi로") + "</button>" +
      "</td></tr>";
  }
  document.getElementById("ranges").innerHTML = html + "</table>" +
    "<div>" + T("여유 ") + r.margin_deg + "°" + T(" (패키지 ") +
    r.package_margin_deg + "°)</div>";
  document.querySelectorAll("#ranges button").forEach((b) => {
    b.onclick = async () => {
      const rr = await post("/api/range", {joint: b.dataset.j,
        end: b.dataset.e, joint_deg: qdeg[b.dataset.j] || 0});
      log("M5 " + b.dataset.j + " " + b.dataset.e + T(" 기록: ") +
        (rr.ok ? "ok" : rr.error));
      refreshRanges();
    };
  });
}
async function refreshPoses() {
  const r = await post("/api/poses", {});
  if (!r.ok) { log(T("자세 로드 실패: ") + r.error); return; }
  const sel = document.getElementById("w6-pose");
  sel.innerHTML = "";
  let html = "<table><tr><th>" + T("자세") + "</th><th>" + T("목표각") + "</th><th>" + T("최근 오차") + "</th></tr>";
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
    log(T("이전 자세 표시 실패"));
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
    msg.textContent = T("실패: ") + (r.error || "?");
    log(T("IK 실패: ") + (r.error || "?"));
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
    msg.textContent += T(" | 가상 추종(본체 이동)");
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
      (conn ? T("미리보기") : T("본체")) +
      " [" + _pS.x.toFixed(1) + ", " + _pS.y.toFixed(1) +
      ", " + _pS.z.toFixed(1) + T("] / 목표 [") + _gS.x.toFixed(1) +
      ", " + _gS.y.toFixed(1) + ", " + _gS.z.toFixed(1) +
      T("] / 거리 ") + pdd.toFixed(1) + "mm";
  } catch (err) {
    log(T("미리보기 표시 실패"));
  }
  setMarker((wasClamped || walked || !r.ik.success) ? 0xff2222 :
    0xff8800);
  msg.textContent = (r.ik.success ? T("수렴") : T("미수렴(최근접)")) +
    (wasClamped ? T(" (테이블 클램프)") : "") +
    (walked ? T(" (최근접 추종)") : "") +
    T(" 위치오차 ") + r.ik.pos_err_mm.toFixed(1) + "mm" +
    T(" 자세오차 ") + r.ik.rot_err_deg.toFixed(1) + "°";
  log("IK " + msg.textContent);
  try {
    const qd = {};
    for (const j of MODEL.joints) qd[j.id] = sol[j.id];
    // T-Bot has no motor map and /api/preview refuses it (400, which
    // also pollutes the console error log), so skip the motor preview.
    const pv = ROBOT === "tbot" ? {ok: false}
      : await post("/api/preview", {q_deg: qd});
    if (pv.ok) {
      const ms = pv.motors.map((m) => m.motor + "=" + m.target).join(" ");
      previewRange = pv.range;
      msg.textContent += T(" | 모터 ") + ms + T(" | 범위 ") + pv.range;
      log(T("모터 미리보기: ") + ms + T(" / 범위: ") + pv.range);
    }
  } catch (err) {
    log(T("미리보기 실패: ") + (err && err.message ? err.message : err));
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
      (r.ik.success ? T("수렴") : T("미수렴(최근접)")) +
      T(" / 오차 ") + r.ik.pos_err_mm.toFixed(1) + "mm" +
      T(" / 제한 ") + (lim.length ? lim.join(",") : T("없음")) +
      T(" / 목표(테이블) [") + sg.x.toFixed(1) + ", " + sg.y.toFixed(1) +
      ", " + sg.z.toFixed(1) + "]mm";
    document.getElementById("ik-stat").textContent = stat;
    log(T("IK 상태: ") + stat);
    if (document.getElementById("vm-gt").checked) {
      const gq = {};
      for (const j of MODEL.joints) gq[j.id] = sol[j.id];
      const g = await post("/api/gt_frames", {q_deg: gq});
      if (g.ok) {
        const gp = g.points.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
        st.setSkel(st.gtLine, gp);
      } else {
        st.gtLine.visible = false;
        log(T("GT 표시 불가: ") + (g.error || "no private visuals"));
      }
    } else {
      st.gtLine.visible = false;
    }
  } catch (err) {
    log(T("IK 상태 표시 실패: ") + (err && err.message ? err.message : err));
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
function unhomedJoints() {
  // Calibration-independent testing: unhomed joints warn, never block.
  if (!lastOv || !lastOv.joints) return [];
  return lastOv.joints.filter((j) => !j.homed).map((j) => j.joint);
}
function sendBlockReason() {
  if (!lastOv) return T("상태 수신 중");
  if (!isConnected()) return T("브릭 미연결(가상만)");
  const L = lastOv.latched.left, R = lastOv.latched.right;
  if (L === true || R === true) return T("래치 걸림(해제 필요)");
  if (!pendingQ || !previewOk) return T("유효한 미리보기 없음");
  if (previewRange && previewRange !== "ok")
    return T("범위 위반: ") + previewRange;
  return "";
}
function refreshSend() {
  const btn = document.getElementById("send-go");
  const msg = document.getElementById("send-msg");
  const row = document.getElementById("send-row");
  if (row) row.style.display = isConnected() ? "" : "none";
  const reason = sendBlockReason();
  if (btn) btn.disabled = !!reason;
  const unhomed = unhomedJoints();
  if (msg) msg.textContent = reason ||
    (unhomed.length ? T("경고: 호밍 미완료(") + unhomed.join(",") +
      T(") — 클릭 시 확인 후 전송 (저속 ") + SEND_SPEED_DPS + "°/s)"
      : (T("보내기 가능 (저속 ") + SEND_SPEED_DPS + "°/s)"));
}
document.getElementById("send-go").onclick = async () => {
  const reason = sendBlockReason();
  if (reason) {
    document.getElementById("send-msg").textContent = reason;
    return;
  }
  const unhomed = unhomedJoints();
  if (unhomed.length && !confirm(T("호밍 미완료 관절: ") +
      unhomed.join(", ") + T("\\n보정 없이 저속으로 보낼까요?"))) {
    log(T("보내기 취소(호밍 미완료 확인 거부)"));
    return;
  }
  let mx = 0;
  for (const j of MODEL.joints)
    mx = Math.max(mx, Math.abs((pendingQ[j.id] || 0) -
      (qdeg[j.id] || 0)));
  if (mx > 45 && !confirm(T("한 관절이 ") + mx.toFixed(1) +
      T("° 움직입니다. 진행할까요?"))) {
    log(T("보내기 취소(대이동 확인 거부)"));
    return;
  }
  const st = window._scene;
  const r = await post("/api/goto", {q_deg: pendingQ,
    speed_dps: SEND_SPEED_DPS, label: "main-send"});
  if (!r.ok) {
    document.getElementById("send-msg").textContent =
      T("거부: ") + (r.error || "?");
    log(T("보내기 거부: ") + (r.error || "?"));
    return;
  }
  for (const j of MODEL.joints) qdeg[j.id] = pendingQ[j.id];
  document.getElementById("follow-live").checked = false;
  snapGizmoToTCP();
  pendingQ = null; previewOk = false; previewRange = "";
  ghostOn = false; applyGhost();
  log(T("로봇으로 보내기 완료"));
  refreshSend();
};
document.getElementById("send-stop").onclick = async () => {
  const r = await post("/api/estop", {});
  log(r.ok ? T("정지(E-STOP) 전송됨") : T("정지 실패: ") + (r.error || "?"));
};
document.getElementById("xyz-go").onclick = () => {
  const st = window._scene;
  const x = parseFloat(document.getElementById("xyz-x").value);
  const y = parseFloat(document.getElementById("xyz-y").value);
  const z = parseFloat(document.getElementById("xyz-z").value);
  if ([x, y, z].some((v) => isNaN(v))) {
    document.getElementById("ik-msg").textContent =
      T("좌표 입력이 숫자가 아닙니다");
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
  log(T("기즈모를 현재 TCP로 이동"));
};
document.getElementById("ik-reach").onclick = async () => {
  const st = window._scene;
  const r = await post("/api/reachable_target", {});
  if (!r.ok) {
    document.getElementById("ik-msg").textContent =
      T("닿는 목표 실패: ") + r.error;
    return;
  }
  const v = new THREE.Vector3(r.target[0], r.target[1], r.target[2]);
  st.target.position.copy(st.arm.localToWorld(v));
  document.getElementById("ik-msg").textContent =
    T("닿는 목표: [") + r.target.join(", ") + "]mm";
  solveIK();
};
let MAT = null;
async function loadMat() {
  const r = await post("/api/mat_points", {});
  const sel = document.getElementById("mat-point");
  if (!r.ok || !r.mat) {
    sel.innerHTML = "<option>" + T("매트 없음(비공개)") + "</option>";
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
        b.title = T("매트 ") + p.n;
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
    T("매트 {n}: [{x}, {y}, {z}]mm", {n: p.n, x: p.x, y: p.y, z: p.z});
  solveIK(p.seed || null);
};
document.getElementById("demo-start").onclick = async () => {
  const msg = document.getElementById("demo-msg");
  const step = async (name, fn) => {
    if (!confirm(T("다음 단계: {name}\\n진행할까요?", {name}))) {
      msg.textContent = T("중단: {name}", {name});
      log(T("데모 중단: {name}", {name}));
      throw new Error("stop");
    }
    const r = await fn();
    msg.textContent = `${name}: ${r}`;
    log(T("데모 {n}: {r}", {n: name, r: r}));
  };
  try {
    await step(T("호밍 상태 확인"), async () => {
      const o = await post("/api/overview", {});
      if (!o.ok) throw new Error(T("overview 실패"));
      return T("연결 ok");
    });
    const pr = await post("/api/poses", {});
    const poseQ = {};
    if (pr.ok) for (const p of pr.poses) poseQ[p.id] = p.q_deg;
    for (const pose of ["safe02", "safe05"]) {
      await step(T("안전자세 {p} (저속)", {p: pose}), async () => {
        const r = await post("/api/pose", {pose});
        if (!r.ok) throw new Error(r.error || T("거부"));
        if (poseQ[pose]) MODEL.joints.forEach((j, i) => {
          qdeg[j.id] = poseQ[pose][i];
        });
        snapGizmoToTCP();
        return T("최대오차 {x}°", {x: r.max_abs_err.toFixed(2)});
      });
    }
    if (!MAT) await loadMat();
    const nums = document.getElementById("demo-points").value
      .split(",").map((s) => parseInt(s.trim(), 10))
      .filter((v) => !isNaN(v)).slice(0, 5);
    for (const n of nums) {
      await step(T("매트 {n}번으로 이동 (저속)", {n: n}), async () => {
        const p = MAT.points.find((q) => q.n === n);
        if (!p) throw new Error(T("매트 점 없음"));
        const ik = await post("/api/ik",
          {pos: [p.x, p.y, p.z], mat: null,
           q_init: p.seed ||
             MODEL.joints.map((j) => qdeg[j.id] || 0),
           position_only: true});
        if (!ik.ok || !ik.ik.success)
          throw new Error(T("IK 실패(도달 불가 표시)"));
        const qd = {};
        ik.joints.forEach((jid, i) => { qd[jid] = ik.ik.q_deg[i]; });
        const g = await post("/api/goto",
          {q_deg: qd, speed_dps: 15, label: `mat${n}`});
        if (!g.ok) throw new Error(g.error || T("이동 거부"));
        ik.joints.forEach((jid, i) => { qdeg[jid] = ik.ik.q_deg[i]; });
        snapGizmoToTCP();
        return T("오차 {x}mm", {x: ik.ik.pos_err_mm.toFixed(1)});
      });
    }
    await step(T("원위치 (safe01)"), async () => {
      const r = await post("/api/pose", {pose: "safe01"});
      if (!r.ok) throw new Error(r.error || T("거부"));
      if (poseQ["safe01"]) MODEL.joints.forEach((j, i) => {
        qdeg[j.id] = poseQ["safe01"][i];
      });
      snapGizmoToTCP();
      return T("복귀 완료");
    });
    msg.textContent += T(" — 데모 끝");
  } catch (err) {
    if (err && err.message !== "stop") {
      msg.textContent = T("실패: ") + err.message;
      log(T("데모 실패: ") + err.message);
    }
  }
};
document.getElementById("w6-run").onclick = async () => {
  const id = document.getElementById("w6-pose").value;
  const r = await post("/api/pose", {pose: id});
  document.getElementById("w6-msg").textContent =
    r.ok ? id + T(" 재현 완료, 최대오차 ") + r.max_abs_err.toFixed(2) + "°"
         : T("실패(거부): ") + r.error;
  log("M6 " + id + ": " + (r.ok ? "ok" : r.error));
  if (r.ok) snapGizmoToTCP();
  refreshPoses();
};
async function refreshConfig() {
  const r = await post("/api/config", {});
  if (!r.ok) { log(T("설정 로드 실패: ") + r.error); return; }
  let html = "<table><tr><th>" + T("항목") + "</th><th>" + T("값") + "</th><th>" + T("변경") + "</th></tr>";
  for (const k of Object.keys(r.tunables)) {
    html += "<tr><td>" + k + "</td><td>" + r.tunables[k] +
      "</td><td><input id='cfg-" + k + "' size='6'>" +
      "<button data-k='" + k + "'>" + T("적용") + "</button></td></tr>";
  }
  document.getElementById("cfgs").innerHTML = html + "</table>" +
    "<div>" + T("패키지 여유 ") + r.package.operating_margin_deg + T("°, 속도 상한 ") +
    JSON.stringify(r.package.speed_caps) + "</div>";
  document.querySelectorAll("#cfgs button").forEach((b) => {
    b.onclick = async () => {
      const v = parseFloat(
        document.getElementById("cfg-" + b.dataset.k).value);
      const rr = await post("/api/config/set",
        {key: b.dataset.k, value: v});
      log(T("설정 ") + b.dataset.k + "=" + v + ": " +
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
        (homed[jid] ? T("완료") : T("미완")) + " " +
        "<button data-home='" + jid + "'>" + T("호밍") + "</button> " +
        "<button data-jogminus='" + jid + "'>-5</button>" +
        "<button data-jogplus='" + jid + "'>+5</button> " +
        "<input id='goto-" + jid + "' size='5' placeholder='" + T("각도") + "'> " +
        "<button data-goto='" + jid + "'>" + T("각도로 이동") + "</button> " +
        "<button data-mark='" + jid + "'>" + T("현재를 0으로") + "</button>" +
        "<span id='home-msg-" + jid + "'></span></div>";
    }
    box.innerHTML = html;
    // M4 dropdown lives on this page but initModel (3D viewer) only
    // runs on the main page, so fill the w4-joint options here from
    // the same homing spec rows.
    const w4 = document.getElementById("w4-joint");
    if (w4) {
      w4.innerHTML = "";
      for (const [jid] of rows) {
        const o = document.createElement("option");
        o.textContent = jid; w4.appendChild(o);
      }
    }
    box.querySelectorAll("[data-home]").forEach((b) => {
      b.onclick = async () => {
        const m = document.getElementById("home-msg-" + b.dataset.home);
        const r = await post("/api/home_joint",
          {joint: b.dataset.home, mark: false});
        if (m) m.textContent = r.ok
          ? T("결과: ") + JSON.stringify({offset: r.offset,
              method: r.method})
          : T("실패: ") + (r.error || "?");
        poll();
      };
    });
    box.querySelectorAll("[data-mark]").forEach((b) => {
      b.onclick = async () => {
        const m = document.getElementById("home-msg-" + b.dataset.mark);
        const r = await post("/api/home_joint",
          {joint: b.dataset.mark, mark: true});
        if (m) m.textContent = r.ok
          ? T("0으로 표시됨(offset ") + r.offset + ")"
          : T("실패: ") + (r.error || "?");
        poll();
      };
    });
    // Manual positioning when the homing direction is wrong and a
    // sensor run is impossible: jog step by step, or send one
    // joint to an exact angle (other joints hold their live read).
    const jogBy = async (jid, delta) => {
      const m = document.getElementById("home-msg-" + jid);
      const r = await post("/api/jog", {joint: jid, delta});
      if (m) m.textContent = r.ok
        ? (r.executed === false ? T("미실행: ") + (r.warning || "?")
          : T("이동: 요청 {a}도, 실이동 {b}도",
            {a: r.joint_delta,
             b: (r.moved !== undefined ? r.moved.toFixed(1) : "?")}))
        : T("실패: ") + (r.error || "?");
      poll();
    };
    box.querySelectorAll("[data-jogminus]").forEach((b) => {
      b.onclick = () => jogBy(b.dataset.jogminus, -5);
    });
    box.querySelectorAll("[data-jogplus]").forEach((b) => {
      b.onclick = () => jogBy(b.dataset.jogplus, 5);
    });
    box.querySelectorAll("[data-goto]").forEach((b) => {
      b.onclick = async () => {
        const jid = b.dataset.goto;
        const m = document.getElementById("home-msg-" + jid);
        const v = parseFloat(
          document.getElementById("goto-" + jid).value);
        if (isNaN(v)) {
          if (m) m.textContent = T("각도 입력이 숫자가 아닙니다");
          return;
        }
        const ov = await post("/api/overview", {});
        if (!ov.ok) {
          if (m) m.textContent = T("상태 조회 실패");
          return;
        }
        const q = {};
        for (const j of ov.joints) {
          if (j.joint_deg === null || j.joint_deg === undefined) {
            if (m) m.textContent =
              T("전 관절 각도 필요: ") + j.joint + T(" 읽김");
            return;
          }
          q[j.joint] = j.joint_deg;
        }
        q[jid] = v;
        const r = await post("/api/goto",
          {q_deg: q, speed_dps: 15, label: "homing-goto"});
        if (m) m.textContent = r.ok
          ? T("이동 완료, 최대오차 {x}도", {x: r.max_abs_err.toFixed(2)})
          : T("실패(거부): ") + (r.error || "?");
        log(T("호밍 수동이동 ") + jid + "=" + v + ": " +
          (r.ok ? "ok" : r.error));
        poll();
      };
    });
    if (!spec.ok)
      box.innerHTML += "<div>" + T("호밍 명세 실패: ") + spec.error + "</div>";
    // One-shot all-zero at the physical reference (ㄱ) pose: every
    // joint reads 0 here, and control starts from this pose.
    const allBtn = document.getElementById("home-all");
    if (allBtn) allBtn.onclick = async () => {
      if (!confirm(T("현재 자세를 전 관절의 0으로 기록할까요?"))) {
        log(T("전부 0으로 취소됨"));
        return;
      }
      const m = document.getElementById("home-all-msg");
      const r = await post("/api/home_all", {});
      if (m) m.textContent = r.ok
        ? T("전부 0으로 기록됨")
        : T("실패: ") + (r.error || "?");
      log(T("전부 0으로: ") +
        (r.ok ? JSON.stringify(r.offsets) : r.error));
      poll();
    };
  } catch (err) {
    box.innerHTML = T("호밍 로드 실패");
  }
}
async function renderVerify() {
  if (!document.getElementById("verify-table")) return;
  try {
    const r = await post("/api/verify_report", {});
    if (!r.ok) {
      document.getElementById("verify-table").textContent =
        T("보고서 로드 실패: ") + r.error;
      return;
    }
    const rep = r.report || null;
    if (!rep) {
      document.getElementById("verify-table").textContent =
        T("아직 검증 보고서가 없습니다(시뮬레이션 산출 후 표시)");
      return;
    }
    // Report shape: runs/stage5c/ik_check_multi/report.json
    // {targets: [{id, kind, tcp_err_mm, dq, base_moved,
    // at_limit, pass, reason}], pass_reachable, pass_unreachable}.
    let html = "<table><tr><th>" + T("목표") + "</th><th>" + T("종류") + "</th>" +
      "<th>" + T("TCP 오차mm") + "</th><th>" + T("밑동 관절") + "</th><th>" + T("통과") + "</th>" +
      "<th>" + T("사유") + "</th></tr>";
    for (const t of (rep.targets || [])) {
      const dq = (t.dq || []).map((v) => (+v).toFixed(1)).join(",");
      html += "<tr><td>" + t.id + "</td><td>" + (t.kind || "") +
        "</td><td>" + t.tcp_err_mm + "</td><td>" +
        (t.base_moved || []).join(",") + " [" + dq + "]" +
        "</td><td>" + (t.pass ? "PASS" : "FAIL") + "</td><td>" +
        (t.reason || "") + "</td></tr>";
    }
    html += "</table><div>" + T("닿는 목표 통과 ") +
      JSON.stringify(rep.pass_reachable) + T(" / 닿지 않는 목표 ") +
      JSON.stringify(rep.pass_unreachable) + "</div>";
    if (rep.images && rep.images.length) {
      html += "<div>" + T("증거: ") + rep.images.map((im) =>
        "<a href='" + im.url + "'>" + im.name + "</a>").join(" ") + "</div>";
    }
    try {
      const g = await post("/api/gt_frames",
        {q_deg: {J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0}});
      html += "<div>" + T("GT 뼈대: ") + (g.ok
        ? T("사용 가능 — 메인 3D의 GT 표시로 겹쳐 비교하세요")
        : T("비공개(--private-visuals 필요)")) + "</div>";
    } catch (err) {
      html += "<div>" + T("GT 뼈대: 확인 실패") + "</div>";
    }
    document.getElementById("verify-table").innerHTML = html;
  } catch (err) {
    document.getElementById("verify-table").textContent = T("로드 실패");
  }
}
// ---- Digital twin (main page) ----
let TWIN = {slots: {}, ids: [], colours: [], holdQ: null};
async function initTwin() {
  if (!document.getElementById("twin-slots")) return;
  const r = await post("/api/twin/slots", {});
  const box = document.getElementById("twin-slots");
  if (!r.ok) { box.textContent = T("로드 실패: ") + r.error; return; }
  TWIN.ids = r.ids; TWIN.colours = r.colours; TWIN.slots = r.slots || {};
  let html = "<table><tr>";
  for (const sid of TWIN.ids) {
    html += "<td>" + sid + "<br><select data-slot='" + sid + "'>" +
      "<option value=''>" + T("없음") + "</option>" + TWIN.colours.map((c) =>
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
      rr.ok ? T("색 저장됨") : T("저장 실패: ") + rr.error;
  };
  document.getElementById("twin-run").onclick = runTwin;
  document.getElementById("twin-send").onclick = sendTwin;
  document.getElementById("twin-rest").onclick = async () => {
    if (!confirm(T("휴식 자세로 저속 이동할까요?"))) return;
    const rr = await post("/api/twin/rest", {});
    document.getElementById("twin-msg").textContent =
      rr.ok ? T("휴식 자세 이동 완료") : T("실패: ") + rr.error;
    log(T("트윈 휴식: ") + (rr.ok ? "ok" : rr.error));
  };
}
const VLA = {fakeIdx: 0, trial: null, hold: null};
function vlaCamText(c) {
  const d = c.dev || {};
  const st = d.open ?
    T("연결됨 ") + d.width + "x" + d.height +
      (d.fps ? " @" + d.fps + "fps" : "") +
      (d.backend ? " [" + d.backend + "]" : "") :
    T("끊김: ") + (d.error || T("알 수 없음"));
  return T("번호 {n} / {s} / 가짜 {m}장",
    {n: c.camera, s: st, m: c.fake.length});
}
async function refreshVlaCam() {
  const info = document.getElementById("vla-caminfo");
  const c = await post("/api/vla/camera", {});
  if (!c.ok) {
    info.textContent = T("카메라 정보 실패: ") + c.error;
    return;
  }
  info.textContent = vlaCamText(c);
  VLA.fakes = c.fake;
}
async function initVla() {
  if (!document.getElementById("vla-run")) return;
  await refreshVlaCam();
  document.getElementById("vla-run").onclick = runVla;
  document.getElementById("vla-send").onclick = sendVla;
  document.getElementById("vla-rest").onclick = async () => {
    if (!confirm(T("휴식 자세로 저속 이동할까요?"))) return;
    const rr = await post("/api/twin/rest", {});
    document.getElementById("vla-msg").textContent =
      rr.ok ? T("휴식 자세 이동 완료") : T("실패: ") + rr.error;
    log(T("VLA 휴식: ") + (rr.ok ? "ok" : rr.error));
  };
  document.getElementById("vla-yes").onclick = () => voteVla(true);
  document.getElementById("vla-no").onclick = () => voteVla(false);
}
async function runVla() {
  const msg = document.getElementById("vla-msg");
  const source = document.getElementById("vla-source").value;
  const target = document.getElementById("vla-target").value;
  const task = document.getElementById("vla-task").value;
  const ckpt = document.getElementById("vla-ckpt").value;
  msg.textContent = T("추론 중…");
  const r = await post("/api/vla/infer",
    {source, target, task, ckpt,
     fake_index: source === "fake" ? (VLA.fakeIdx++) : 0});
  if (!r.ok) {
    msg.textContent = T("실패: ") + r.error;
    log(T("VLA 추론 실패: ") + r.error);
    await refreshVlaCam();
    return;
  }
  const img = document.getElementById("vla-img");
  img.src = "data:image/png;base64," + r.png;
  img.style.display = "";
  document.getElementById("vla-info").textContent =
    r.task + " → " + r.slot + " (" + r.dist_mm + "mm) " +
    r.infer_s + T("초 ") + r.origin + " [" + r.ckpt + "]";
  msg.textContent = T("추론 완료");
  log(T("VLA 추론: ") + r.slot + " " + r.infer_s + T("초 ") + r.origin);
  await refreshVlaCam();
  // 3D translucent preview via the shared ghost path.
  VLA.hold = r.hold_q;
  const sol = {};
  for (const j of MODEL.joints) sol[j.id] = r.hold_q[j.id];
  const pv = await post("/api/preview", {q_deg: sol});
  previewRange = (pv && pv.ok && pv.range) || "";
  if (isConnected()) {
    pendingQ = sol; previewOk = !!(pv && pv.ok);
    ghostOn = true; applyGhost();
  } else {
    for (const j of MODEL.joints) qdeg[j.id] = sol[j.id];
    pendingQ = null; previewOk = false; previewRange = "";
    ghostOn = false; applyGhost();
  }
  document.getElementById("vla-send").disabled = !!sendBlockReason();
  const sv = await post("/api/vla/trial/save",
    {instruction: r.task, ckpt: r.ckpt});
  if (sv.ok) {
    VLA.trial = sv.id;
    document.getElementById("vla-trial").textContent = sv.id;
    document.getElementById("vla-yes").disabled = false;
    document.getElementById("vla-no").disabled = false;
  }
}
async function sendVla() {
  if (!VLA.hold) return;
  if (!isConnected()) {
    alert(T("브릭이 연결되지 않아 보낼 수 없습니다"));
    return;
  }
  const uh = unhomedJoints();
  if (uh.length &&
      !confirm(T("호밍 안 됨(") + uh.join(",") +
               T("): 그래도 VLA 자세를 보내시겠습니까?"))) return;
  let mx = 0;
  for (const id of Object.keys(VLA.hold))
    mx = Math.max(mx, Math.abs((VLA.hold[id] || 0) -
                               (qdeg[id] || 0)));
  if (mx > 45 &&
      !confirm(T("이동량 ") + mx.toFixed(0) + T("°: 저속으로 이동할까요?")))
    return;
  pendingQ = Object.assign({}, VLA.hold);
  const reason = sendBlockReason();
  if (reason) {
    alert(T("보내기 거부: ") + reason);
    return;
  }
  const r = await post("/api/goto",
    {q_deg: pendingQ, speed_dps: 15, label: "vla-send"});
  if (!r.ok) {
    alert(T("로봇으로 보내기 실패: ") + r.error);
    log(T("VLA 보내기 실패: ") + r.error);
    return;
  }
  for (const j of MODEL.joints) qdeg[j.id] = pendingQ[j.id];
  document.getElementById("follow-live").checked = false;
  pendingQ = null; previewOk = false; previewRange = "";
  ghostOn = false; applyGhost();
  document.getElementById("vla-send").disabled = true;
  refreshSend(); snapGizmoToTCP(); refreshPoses();
  log(T("VLA 보내기 완료"));
}
async function voteVla(correct) {
  if (!VLA.trial) return;
  const r = await post("/api/vla/trial/vote",
    {id: VLA.trial, correct});
  document.getElementById("vla-trial").textContent =
    VLA.trial + (r.ok ? (correct ? T(" 맞음") : T(" 틀림")) : T(" 저장 실패"));
  log(T("VLA 판정: ") + VLA.trial + " " + (correct ? T("맞음") : T("틀림")));
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
  msg.textContent = T("가상 이미지 생성+VLA 추론 중…");
  const r = await post("/api/twin/run",
    {slots: readTwinSlots(),
     target: document.getElementById("twin-target").value});
  if (!r.ok) { msg.textContent = T("실패: ") + r.error; return; }
  TWIN.holdQ = r.hold_q;
  const img = document.getElementById("twin-img");
  img.src = "data:image/png;base64," + r.png;
  img.style.display = "";
  msg.textContent = T("정책=") + r.policy + T(" 목표=") + r.target +
    T(" 유지=") + JSON.stringify(r.hold_q) + " " + JSON.stringify(r.rec);
  document.getElementById("twin-send").disabled = false;
  TWIN.armSend = 0;
  document.getElementById("twin-send").textContent =
    T("유지 자세로 보내기 (저속)");
  log(T("트윈 실행: 정책=") + r.policy + T(" 목표=") + r.target);
}
async function sendTwin() {
  // Inline two-step confirm: native confirm() can be permanently
  // suppressed by the browser, which silently disabled this button.
  const msg = document.getElementById("twin-msg");
  const btn = document.getElementById("twin-send");
  if (!TWIN.holdQ) {
    if (msg) msg.textContent = T("유지 자세 없음(트윈 실행 먼저)");
    return;
  }
  const now = Date.now();
  if (!TWIN.armSend || now - TWIN.armSend > 10000) {
    TWIN.armSend = now;
    if (msg) msg.textContent = T("다시 누르면 저속(15도/s)으로 전송");
    if (btn) btn.textContent = T("확인: ") + T("유지 자세로 보내기");
    log(T("트윈 전송 대기(다시 누르면 전송)"));
    return;
  }
  TWIN.armSend = 0;
  if (btn) btn.textContent = T("유지 자세로 보내기 (저속)");
  const r = await post("/api/goto",
    {q_deg: TWIN.holdQ, speed_dps: 15, label: "twin-hold"});
  if (msg) msg.textContent =
    r.ok ? T("유지 자세 이동 완료, 최대오차 ") +
      r.max_abs_err.toFixed(2) + "°" : T("실패(거부): ") + r.error;
  log(T("트윈 전송: ") + (r.ok ? "ok" : r.error));
}
// ---- Session calibration (cal page) ----
async function refreshSess() {
  const box = document.getElementById("sess-stat");
  if (!box) return;
  const r = await post("/api/calib/session", {});
  if (!r.ok) { box.textContent = T("로드 실패: ") + r.error; return; }
  box.innerHTML = T("연속성=") + r.continuity +
    T(" 확정=") + (r.confirmed ? (T("됨(") + r.confirmed_at + ")") : T("안됨")) +
    T(" 저장=") + (r.saved_at || T("없음")) +
    "<br>" + T("현재 가동=") + JSON.stringify(r.boot_now) +
    "<br>" + T("저장 가동=") + JSON.stringify(r.boot_saved);
}
function initSess() {
  if (!document.getElementById("sess-flow")) return;
  refreshSess();
  const msg = (t) => {
    document.getElementById("sess-msg").textContent = t;
  };
  document.getElementById("sess-flow").onclick = async () => {
    if (!confirm(T("한 번에 보정: J1~J4 센서 호밍 → 손목 → 확인자세. 진행할까요?")))
      return;
    const r = await post("/api/calib/flow", {});
    const steps = (r.ok && r.steps ? r.steps : [])
      .map((s) => s.step + "=" + s.result +
        (s.max_abs_deg !== undefined ? "(" + s.max_abs_deg + "°)" : "") +
        (s.note ? " " + s.note : "")).join(" / ");
    document.getElementById("flow-steps").textContent = steps;
    msg(r.ok ? (T("흐름=") + r.flow) : (T("실패: ") + r.error));
    if (r.ok && r.flow === "done")
      msg(T("확인 자세로 이동됨. 3D 실측겹침을 보고 '맞음'을 누르세요."));
    poll();
  };
  document.getElementById("sess-park").onclick = async () => {
    if (!confirm(T("주차: 휴식 자세로 저속 이동 후 세션 저장. 진행할까요?")))
      return;
    const r = await post("/api/calib/park", {});
    msg(r.ok ? T("주차 저장됨(") + r.saved_at + ")" : T("실패: ") + r.error);
    refreshSess(); poll();
  };
  document.getElementById("sess-restore").onclick = async () => {
    const r = await post("/api/calib/restore", {});
    msg(r.ok ? T("복원됨(연속성=") + r.continuity + ")" : T("실패: ") + r.error);
    poll();
  };
  document.getElementById("sess-check").onclick = async () => {
    const r = await post("/api/calib/check", {});
    msg(r.ok ? (T("확인 자세 이동, 최대편차 ") +
      r.deviation.max_abs_deg + "°") : (T("실패: ") + r.error));
    poll();
  };
  document.getElementById("sess-ok").onclick = async () => {
    const r = await post("/api/calib/confirm", {});
    msg(r.ok ? (T("보정 확정됨(") + r.confirmed_at + ")")
             : (T("확정 불가: ") + r.error));
    refreshSess();
  };
}
// Page split: cal/verify pages lack main-only controls; absorb lookups
// with a detached element so shared handlers never throw.
const __realById = document.getElementById.bind(document);
document.getElementById = (id) =>
  __realById(id) || document.createElement("div");
document.getElementById("langbtn").onclick = () => {
  LANG = LANG === "en" ? "ko" : "en";
  applyLang();
};
initModel().then(() => {
  applyLang();
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
    refreshVisual(); loadMat(); initTwin(); initVla();
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
