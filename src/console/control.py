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
  git-ignored assets/private/visual_pack/ROBOT/ and are served only
  with --private-visuals ROBOT (same-origin page, no-store).

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
    readback = mini.coupled_joint_readback(encoders, handler.jog_map,
                                                  offsets)
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
    joints = [{"id": j["id"], "axis": list(j["axis"]),
               "point": list(p)}
              for j, p in zip(chain_order(doc), arr["points"])]
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
    tip = tip_offset()
    jp = arr["points"][5]
    import math as _m
    _d = [tip[i] - jp[i] for i in range(3)]
    _n = _m.sqrt(sum(v * v for v in _d)) or 1.0
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


def package_doc():
    root = pathlib.Path(__file__).resolve().parents[2]
    return json.loads((root / "packages" / "robot_inferred"
                       / "robot_package.json").read_text(
                           encoding="utf-8"))


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
    """Doc copy whose TCP is the gripper tip (solver/display only)."""
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
    pkg = package_doc()
    sim = {j["id"]: list(j["limits_deg"]) for j in pkg.get("joints", [])}
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
    pkg_margin = pkg.get("operating_margin_deg")
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
    pkg = package_doc()
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
        qd = dict(zip(order, pose.get("q_deg", [])))
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
    pkg = package_doc()
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
    """Range-checked joint move via coupled motor targets."""
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


def reproduce_pose(handler, pose_id):
    pkg = package_doc()
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
        # Private LDraw-derived meshes: only with --private-visuals,
        # only same-origin page loads, never cached, never in git.
        if not self.private_visuals:
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        rel = path[len("/private/visual/"):]
        parts = rel.split("/")
        if (not rel or ".." in rel or rel.startswith("/")
                or len(parts) != 2):
            self._send_json({"ok": False, "error": "not found"}, 404)
            return
        robot, name = parts
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
            pkg = package_doc()
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
            pkg = package_doc()
            stored = read_json(runs_day_dir() / "report.json", {})
            last = {}
            for run in stored.get("poses", []) or []:
                last[run["pose"]] = run["error_deg"]
            return {"ok": True,
                    "poses": [{"id": p["id"], "q_deg": p.get("q_deg", []),
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
            return {"ok": True, "stage": stage_table()}
        if path == "/api/mat_points":
            if not self.private_visuals:
                return {"ok": False,
                        "error": "mat needs --private-visuals"}
            import pathlib as _pl
            root = _pl.Path(__file__).resolve().parents[2]
            mat = json.loads(
                (root / "assets" / "private" / "mat_points.json")
                .read_text(encoding="utf-8"))
            return {"ok": True, "mat": mat}
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
            if self.private_visuals:
                for i in range(7):
                    lid = "D" + str(i)
                    links[lid] = ("/private/visual/"
                                   + self.private_visuals + "/"
                                   + lid + ".glb")
                available.append("ldraw")
            return {"ok": True, "available": available, "links": links}
        return super()._dispatch(path, args)

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        from urllib.parse import urlparse

        parsed = urlparse(self.path).path
        if parsed == "/":
            body = PAGE.encode()
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
        else:
            super().do_GET()


def create_server(transport, host="127.0.0.1", port=CONTROL_PORT,
                  jog_map=None, override_path=None,
                  direction_path=None, model_doc=None,
                  console_path=None, private_visuals=None):
    """Create (not start) the control console server.

    Always binds a loopback host; only 127.0.0.1 is supported.
    """
    if host != "127.0.0.1":
        raise ValueError("control console serves 127.0.0.1 only")
    if override_path is None:
        override_path = mini.default_override_path()
    if direction_path is None:
        direction_path = default_direction_path()
    if console_path is None:
        console_path = default_console_path()
    if jog_map is None:
        overrides = mini.read_jog_overrides(override_path)
        jog_map = mini.load_jog_map(overrides=overrides)
    else:
        overrides = {}
    if model_doc is None:
        from src.kin.model import load_model

        root = pathlib.Path(__file__).resolve().parents[2]
        model_doc = load_model(str(root / "models" / "robot_inferred.json"))
    handler = type("ControlHandler", (_Handler,),
                   {"transport": transport, "jog_map": jog_map,
                    "jog_overrides": overrides,
                    "override_path": str(override_path),
                    "direction_path": str(direction_path),
                    "model_doc": model_doc,
                    "zeros": {},
                    "console_path": str(console_path),
                    "private_visuals": private_visuals})
    from http.server import ThreadingHTTPServer

    return ThreadingHTTPServer((host, port), handler)


def build_transport(config):
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
                             "assets/private/visual_pack/ROBOT")
    args = parser.parse_args(argv)
    transport = build_transport(args.config)
    server = create_server(transport, port=args.port,
                           private_visuals=args.private_visuals)
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
#view3d { width: 100%; height: 420px; display: block; background: #565d66; }
.unconf { color: #fb0; font-weight: bold; }
button { margin: 2px; }
#err { color: #f77; white-space: pre-wrap; }
.wiz { border: 1px solid #555; padding: 8px; margin-top: 8px; }
.shell { color: #999; border: 1px dashed #555; padding: 8px; margin-top: 8px; }
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
  <span id="st-left">좌: …</span><span id="st-right">우: …</span>
  <span id="st-latch">래치: …</span><span id="st-hb">하트비트: …</span>
  <button id="estop">E-STOP</button>
  <button id="btn-clear">래치 해제</button>
</div>
<main>
<section><h3>관절 패널</h3><div id="joints">로딩…</div></section>
<section><h3>3D 보기 (<span id="vsrc-name">기본 도형</span>)</h3>
<canvas id="view3d"></canvas>
<div>표시: <select id="vsrc">
<option value="primitives">기본 도형</option>
<option value="ldraw">LDraw 메시(비공개)</option>
</select>
<button id="fitview">화면 맞춤</button></div>
<div id="glerr"></div>
<div id="vmsg"></div>
</section>
<section><h3>데모 순서</h3>
<div class="wiz" id="wizdemo">
<div>호밍 확인 → 안전자세 2개 → 매트 점(아래 번호) → 원위치. 느리게, 한 단계씩 확인.</div>
매트 점 <input id="demo-points" size="10" value="1,5,9">
<button id="demo-start">데모 순서 시작</button>
<div id="demo-msg"></div>
</div>
</section>
<section><h3>M4 방향 마법사</h3>
<div class="wiz" id="wiz4">
관절 <select id="w4-joint"></select>
<button id="w4-try">+5&deg; 시도</button>
<button id="w4-ok">맞음</button>
<button id="w4-rev">반대</button>
<div id="w4-msg"></div>
</div>
</section>
<section><h3>IK 타겟</h3>
<div class="wiz" id="wizik">
<label><input type="checkbox" id="follow-live" checked>
실시간 관절값 반영</label><br>
<button id="ik-t">이동</button><button id="ik-r">회전</button>
<button id="ik-x">X</button><button id="ik-y">Y</button>
<button id="ik-z">Z</button>
<button id="ik-space">world/local</button>
<button id="ik-reset">타겟→현재EE</button><br>
<label><input type="checkbox" id="grip-down">
집게가 아래를 향함 (위치+집게 방향)</label>
<button id="ik-tcp">기즈모를 TCP로</button>
<button id="ik-reach">닿는 목표</button>
<br>매트 <select id="mat-point"><option>로딩…</option></select>
<button id="mat-go">매트 목표</button>
<div id="ik-msg">주황 구슬을 드래그하면 뼈대가 따라옵니다</div>
<div id="ik-live">TCP — / 목표 — / 거리 —</div>
<div id="ik-stat">수렴 — / 오차 — / 제한 — / 목표(베이스) —</div>
<label><input type="checkbox" id="show-prev" checked>
드래그 전 자세(반투명)</label>
<label><input type="checkbox" id="show-gt" checked>
GT 뼈대(--private-visuals일 때만)</label>
</div>
</section>
<section><h3>M5 범위 마법사</h3>
<div class="wiz" id="wiz5">
<div id="ranges">로딩…</div>
</div>
</section>
<section><h3>M6 자세 재현</h3>
<div class="wiz" id="wiz6">
자세 <select id="w6-pose"></select>
<button id="w6-run">재현</button>
<div id="w6-msg"></div>
<div id="poses">로딩…</div>
</div>
</section>
<section><h3>설정(경계값)</h3>
<div class="wiz" id="wcfg"><div id="cfgs">로딩…</div></div>
</section>
<section><h3>로그</h3>
<div class="wiz"><div id="log"></div></div>
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
  for (let k = 1; k <= 6; k++) {
    const env = m.envelopes.find((e) => e.link === "D" + k);
    const s = env ? [...env.size_mm].sort((a, b) => a - b)
                  : [40, 40, 40];
    RADII.push(Math.min(0.28 * Math.sqrt(s[0] * s[1]), 40));
  }
  const segRadius = (i) => (i < 6 ? RADII[i] : RADII[5] * 0.7);
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
  const q0 = m.joints.map((j) => (POSE01[j.id] || 0) * Math.PI / 180);
  const F0pose = fkine(q0);
  const e0 = MODEL.ee_mm;
  for (let i = 0; i < 6; i++) {
    const a = m.joints[i].point;
    const b = (i < 5) ? m.joints[i + 1].point : e0;
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
    .applyMatrix4(F0pose[5]);
  for (let i = 0; i < 6; i++) {
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
  function setSkel(line, pts) {
    const a = line.geometry.attributes.position;
    for (let i = 0; i < 8; i++) a.setXYZ(i, pts[i].x, pts[i].y, pts[i].z);
    a.needsUpdate = true;
    line.visible = true;
  }
  window._scene = {renderer, scene, camera, controls, arm, tc, target,
                   marker, eeAxes, baseMesh, tcpDot, tgtLine, dirLine,
                   prevLine, gtLine, matDots, setSkel,
                   targetArm: p0ee.clone(),
                   prevQ: null, gtAxes: null, rangesAdopted: null,
                   grounded: false};
  requestAnimationFrame(tick);
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
    "" : "LDraw 메시 없음: --private-visuals ROBOT 필요";
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
  const ids = ["D0", "D1", "D2", "D3", "D4", "D5", "D6"];
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
function skelNodes(F) {
  // Skeleton nodes: base origin, 6 joint-frame origins, EE point.
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
function fkine(qrad) {
  const js = MODEL.joints;
  let T = new THREE.Matrix4();
  const frames = [];
  for (let i = 0; i < js.length; i++) {
    // Axes arrive raw; Python joint_arrays() normalizes to unit.
    const w0 = js[i].axis, p = js[i].point;
    const n = Math.hypot(w0[0], w0[1], w0[2]);
    const w = [w0[0] / n, w0[1] / n, w0[2] / n];
    const v = [-(w[1]*p[2] - w[2]*p[1]),
               -(w[2]*p[0] - w[0]*p[2]),
               -(w[0]*p[1] - w[1]*p[0])];
    const {R, t} = se3(w, v, qrad[i]);
    const M = new THREE.Matrix4().copy(R).setPosition(t);
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
  const qrad = MODEL.joints.map((j) => (qdeg[j.id] || 0) * Math.PI / 180);
  const F = fkine(qrad);
  for (let i = 0; i < 6; i++) {
    SEGS[i].mesh.matrix.copy(F[i]).multiply(SEGS[i].local);
    SEGS[i].mesh.matrixWorldNeedsUpdate = true;
  }
  for (let i = 0; i < 6; i++) {
    const p = MODEL.joints[i].point;
    _v.set(p[0], p[1], p[2]).applyMatrix4(F[i]);
    NODES[i].matrix.makeTranslation(_v.x, _v.y, _v.z);
    NODES[i].matrixWorldNeedsUpdate = true;
  }
  window._scene.eeAxes.matrix.copy(F[5]);
  window._scene.eeAxes.matrixWorldNeedsUpdate = true;
  if (VSRC === "ldraw" && MESHES.ready) {
    MESHES.groups[0].matrix.identity();
    MESHES.groups[0].matrixWorldNeedsUpdate = true;
    for (let i = 1; i <= 6; i++) {
      MESHES.groups[i].matrix.copy(F[i - 1]);
      MESHES.groups[i].matrixWorldNeedsUpdate = true;
    }
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
  controls.update();
  renderer.render(scene, camera);
}
let lastOk = 0;
async function poll() {
  try {
    const o = await post("/api/overview", {});
    lastOk = Date.now();
    const L = o.latched.left, R = o.latched.right;
    const el = (id, txt, cls) => {
      const e = document.getElementById(id);
      e.textContent = txt; e.className = cls;
    };
    el("st-left", "좌: " + (L === null ? "끊김" : "연결"),
       L === null ? "bad" : "ok");
    el("st-right", "우: " + (R === null ? "끊김" : "연결"),
       R === null ? "bad" : "ok");
    const latched = (L === true || R === true);
    el("st-latch", "래치: " + (latched ? "걸림" : "없음"),
       latched ? "bad" : "ok");
    let html = "<table><tr><th>관절</th><th>관절각</th>" +
      "<th>모터각</th><th>홈</th><th>방향</th><th>센서</th>" +
      "<th>조그</th></tr>";
    const follow = document.getElementById("follow-live").checked;
    for (const j of o.joints) {
      if (follow)
        qdeg[j.joint] = j.joint_deg === null ? 0 : j.joint_deg;
      const jog = j.direction === "confirmed" ? "" : " (최대±5°)";
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
    document.getElementById("joints").innerHTML = html + "</table>";
    document.querySelectorAll("#joints button").forEach((b) => {
      b.onclick = async () => {
        const r = await post("/api/jog",
          {joint: b.dataset.j, delta: parseFloat(b.dataset.d)});
        if (!r.ok) alert("jog 실패: " + r.error);
      };
    });
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
  await post("/api/estop", {});
  alert("E-STOP 전송됨");
};
document.getElementById("btn-clear").onclick = async () => {
  await post("/api/clear", {});
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
    html += "<tr><td>" + j.joint + "</td><td>" + rng(j.sim) +
      "</td><td>" + rng(j.measured) + "</td><td>" + rng(j.adopted) +
      (j.adopted_source === "measured" ? "(실측)" : "(시뮬)") +
      "</td><td>현재 " + cur + "° " +
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
  const qrad = MODEL.joints.map((j) => (qdeg[j.id] || 0) *
    Math.PI / 180);
  return tipOf(fkine(qrad));
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
    const pq = MODEL.joints.map((j) => (st.prevQ[j.id] || 0) *
      Math.PI / 180);
    st.setSkel(st.prevLine, skelNodes(fkine(pq)));
    st.prevLine.visible =
      document.getElementById("show-prev").checked;
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
  document.getElementById("follow-live").checked = false;
  r.joints.forEach((jid, i) => { qdeg[jid] = r.ik.q_deg[i]; });
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
    for (const j of MODEL.joints) qd[j.id] = qdeg[j.id] || 0;
    const pv = await post("/api/preview", {q_deg: qd});
    if (pv.ok) {
      const ms = pv.motors.map((m) => m.motor + "=" + m.target).join(" ");
      msg.textContent += " | 모터 " + ms + " | 범위 " + pv.range;
      log("모터 미리보기: " + ms + " / 범위: " + pv.range);
    }
  } catch (err) {
    log("미리보기 실패: " + (err && err.message ? err.message : err));
  }
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
        const v = qdeg[j.id] || 0;
        if (ad && (Math.abs(v - ad[0]) < 0.5 || Math.abs(v - ad[1]) < 0.5))
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
    if (document.getElementById("show-gt").checked) {
      const g = await post("/api/gt_frames", {q_deg: qd});
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
initModel().then(() => {
  poll(); setInterval(poll, 2000);
  refreshRanges(); refreshPoses(); refreshConfig(); refreshVisual();
  loadMat();
});
</script>
</body>
</html>"""


if __name__ == "__main__":
    main()
