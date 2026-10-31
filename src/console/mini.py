"""Minimal local test console for the brick link (stdlib only).

Serves one page with a few buttons (ping, info, jog, state, diag,
stop, e-stop) backed by the PC Transport. Fake bricks by default;
pass ``--config`` for real hardware. Binds to 127.0.0.1 only.

Jog takes joint degrees and converts to motor degrees on the PC via
the gear ratios in ``models/robot_inferred.json`` (q = C m); the
brick still only sees ports. Both values are shown in the response.

The joint map (joint -> brick/port/ratio) can be edited live from
the page without rebuilding: overrides apply to the next jog at
once and are saved to ``config/jogmap.local.json`` (git-ignored) so
they survive restarts. Reset restores the model values.

Run::

    source .venv/bin/activate
    python -m src.console.mini
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

JOG_STEP_DEG = 5.0
MAX_JOG_STEP_DEG = 30.0
JOG_SPEED_DEG_S = 250.0
DIAG_ANGLE_DEG = 360.0
DIAG_SPEED_DEG_S = 90.0
HOME_SPEED_DEG_S = 30.0
HOME_SEARCH_DEG = 360.0
MIN_MOVE_MOTOR_DEG = 10.0

# Physical joint-to-motor ratios (joint_deg = ratio * motor_deg, so
# motor_deg = joint_deg / ratio). Magnitudes from runs/stage3/pred_C.json
# (C_raw drive coefficients, model-derived). J2 has no model ratio (drive
# break) and carries the section-4 fallback magnitude.
# Signs are GT (gt/kinematics_gt_v2.json transmission C drive diagonal,
# all +1 in R17): GT and model (R17 chain) axes agree per joint, so the
# old negative inference signs were inference errors, not a convention
# difference. Joints missing from config/jogmap.local.json fall back here.
DRIVE_RATIOS = {
    "J1": 0.0857,
    "J2": 0.0238,
    "J3": 0.0343,
    "J4": 0.2,
    "J5": 0.2,
    "J6": 0.1429,
}

# Wrist mechanical coupling (GT kinematics_gt_v2 transmission C,
# normalized shape, R17 axes; control settings only, never stage 1-4
# input). With (m_a, m_b, m_c) the J4/J5/J6 drive motors from the jog
# map and (r4, r5, r6) their drive ratios:
#   q4 = r4*m_a
#   q5 = r5*(-0.2*m_a + m_b)
#   q6 = r6*(0.1591837*m_a - 0.0816327*m_b + m_c)
# J1-J3 rows are identity (independent). Inverted for commanding:
#   m_a = y4
#   m_b = y5 + 0.2*y4
#   m_c = y6 - 0.1428572*y4 + 0.0816327*y5
# where y_i = (q_i - offset_i) / r_i.
WRIST_JOINTS = ("J4", "J5", "J6")
WRIST_C = ((1.0, 0.0, 0.0),
           (-0.2, 1.0, 0.0),
           (0.1591837, -0.0816327, 1.0))
WRIST_C_INV = ((1.0, 0.0, 0.0),
               (0.2, 1.0, 0.0),
               (-0.1428572, 0.0816327, 1.0))


def _wrist_motors(jog_map):
    """(brick, port) of the J4/J5/J6 drive motors, in joint order."""
    return [(jog_map[j]["brick"], jog_map[j]["port"])
            for j in WRIST_JOINTS]


def coupled_motor_targets(q, jog_map, offsets=None):
    """Joint targets -> motor targets with wrist coupling.

    ``q`` maps all six joints to joint degrees; ``offsets`` maps joint
    to its zero offset (default 0). Returns ``{(brick, port):
    motor_deg}``. J1-J3 pass through independently.
    """
    off = offsets or {}
    y = {}
    for j in ("J1", "J2", "J3", "J4", "J5", "J6"):
        y[j] = ((float(q[j]) - float(off.get(j, 0.0) or 0.0))
                / jog_map[j]["ratio"])
    motors = {}
    for j in ("J1", "J2", "J3"):
        motors[(jog_map[j]["brick"], jog_map[j]["port"])] = y[j]
    yw = [y[j] for j in WRIST_JOINTS]
    mw = [sum(WRIST_C_INV[i][k] * yw[k] for k in range(3))
          for i in range(3)]
    for (brick, port), m in zip(_wrist_motors(jog_map), mw):
        motors[(brick, port)] = m
    return motors


def coupled_joint_readback(encoders, jog_map, offsets=None):
    """Motor readings -> joint degrees with wrist coupling.

    ``encoders`` maps ``(brick, port)`` to motor degrees. Inverse of
    ``coupled_motor_targets`` (up to offsets).
    """
    off = offsets or {}
    q = {}
    for j in ("J1", "J2", "J3"):
        m = encoders.get((jog_map[j]["brick"], jog_map[j]["port"]))
        q[j] = (jog_map[j]["ratio"] * m + float(off.get(j, 0.0) or 0.0)
                if m is not None else None)
    motors = _wrist_motors(jog_map)
    m = [encoders.get(mp) for mp in motors]
    if any(v is None for v in m):
        for j in WRIST_JOINTS:
            q[j] = None
        return q
    yw = [sum(WRIST_C[i][k] * m[k] for k in range(3))
          for i in range(3)]
    for j, y in zip(WRIST_JOINTS, yw):
        q[j] = (jog_map[j]["ratio"] * y
                + float(off.get(j, 0.0) or 0.0))
    return q


def default_override_path():
    """Repo-root ``config/jogmap.local.json`` (git-ignored)."""
    root = pathlib.Path(__file__).resolve().parents[2]
    return root / "config" / "jogmap.local.json"


def read_jog_overrides(path):
    """Read saved map overrides; missing or bad file means none."""
    try:
        text = pathlib.Path(path).read_text(encoding="utf-8")
    except OSError:
        return {}
    try:
        data = json.loads(text)
    except ValueError:
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def _patch_entry(entry, patch):
    """Validated copy of a map entry with ``patch`` applied."""
    if not isinstance(patch, dict):
        raise ValueError("patch must be an object")
    out = dict(entry)
    if "ratio" in patch:
        try:
            ratio = float(patch["ratio"])
        except (TypeError, ValueError):
            raise ValueError("ratio must be a number")
        if ratio == 0.0 or not math.isfinite(ratio):
            raise ValueError("ratio must be a nonzero finite number")
        out["ratio"] = ratio
    if "brick" in patch:
        if patch["brick"] not in ("left", "right"):
            raise ValueError("brick must be left or right")
        out["brick"] = patch["brick"]
    if "port" in patch:
        if patch["port"] not in ("outA", "outB", "outC", "outD"):
            raise ValueError("port must be outA..outD")
        out["port"] = patch["port"]
    return out


def load_jog_map(model_path=None, overrides=None):
    """Joint -> drive-motor map from the kinematic model.

    Returns ``{joint_id: {"motor_uid", "brick", "port", "ratio"}}``
    where ``motor_deg = joint_deg / ratio``. Wiring (motor, brick,
    port) comes from the drive column of C (the max-``|C|`` entry per
    joint row); the ratio itself comes from ``DRIVE_RATIOS`` (real
    gear ratios) with the model ratio as fallback. Raises
    ``ValueError`` when a joint row has no drive motor.
    ``overrides`` (``{joint: patch}``, as saved by the map editor)
    wins over both; invalid entries are skipped so a hand-edited
    file cannot break startup.
    """
    from src.kin.model import chain_order, coupling, load_model

    if model_path is None:
        root = pathlib.Path(__file__).resolve().parents[2]
        model_path = root / "models" / "robot_inferred.json"
    doc = load_model(str(model_path))
    coup = coupling(doc)
    if coup is None:
        raise ValueError(f"model {model_path} has no transmission.C")
    c = coup["C"]
    motor_ids = coup["motor_ids"]
    act_by_id = {a["id"]: a for a in doc.get("actuators", [])}
    jog_map = {}
    for i, joint in enumerate(chain_order(doc)):
        row = c[i]
        drive = max(range(len(row)), key=lambda j: abs(row[j]))
        model_ratio = float(row[drive])
        if model_ratio == 0.0:
            raise ValueError(f"joint row {i} has no drive motor")
        ratio = DRIVE_RATIOS.get(joint["id"], model_ratio)
        uid = motor_ids[drive]
        act = act_by_id.get(uid, {})
        port = str(act.get("port", "A"))
        if not port.startswith("out"):
            port = "out" + port
        jog_map[joint["id"]] = {
            "motor_uid": uid,
            "brick": act.get("brick", "left"),
            "port": port,
            "ratio": ratio,
        }
    for joint_id, patch in (overrides or {}).items():
        if joint_id not in jog_map:
            continue
        try:
            jog_map[joint_id] = _patch_entry(jog_map[joint_id], patch)
        except (ValueError, TypeError):
            continue
    return jog_map

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Brick link test</title>
<style>
body { font-family: system-ui, sans-serif; max-width: 640px; margin: 2em auto; }
button { font-size: 1em; margin: 0.2em; padding: 0.4em 0.8em; }
.estop { background: #c00; color: #fff; font-weight: bold; }
pre { background: #f0f0f0; padding: 1em; white-space: pre-wrap; }
</style>
</head>
<body>
<h1>Brick link test</h1>
<p>
Joint <select id="joint"><option>J1</option><option>J2</option><option>J3</option><option>J4</option><option>J5</option><option>J6</option></select>
(takes joint degrees, converts to motor degrees on the PC)
</p>
<p>
Brick <select id="brick"><option>left</option><option>right</option></select>
Port <select id="port"><option>outA</option><option>outB</option><option>outC</option><option>outD</option></select>
<strong>Diag only &mdash; Jog and Home always use the joint map below, never these.</strong>
</p>
<p>
Ratio <input id="ratio" type="number" step="any" value="1" style="width: 6em">
<button data-act="map-set" title="apply brick/port/ratio to the selected joint">Apply map</button>
<button data-act="map-show" title="show current joint map">Show map</button>
<button data-act="map-del" title="delete this joint's override">Delete entry</button>
<button data-act="map-reset" title="restore model ratios">Reset map</button>
</p>
<p>
Sensor <select id="sensor"><option>in1</option><option>in2</option><option>in3</option></select>
Direction <select id="direction"><option value="-1">-1</option><option value="+1">+1</option></select>
<button data-act="home" title="drive the joint toward its sensor">Home joint</button>
</p>
<p>
Step <select id="step"><option>1</option><option selected>5</option><option>10</option><option>30</option></select>&deg; (joint)
</p>
<p>
<button data-act="ping">Ping</button>
<button data-act="info">Info</button>
<button data-act="state">Read state</button>
<button data-act="jog+" title="jog +step joint deg">Jog +</button>
<button data-act="jog-" title="jog -step joint deg">Jog &minus;</button>
<button data-act="diag" title="run_angle(360, 90) on the selected port">Diag 360&deg;</button>
<button data-act="markzero" title="record current position as joint 0">Mark zero</button>
<button data-act="stop">Stop</button>
<button data-act="estop" class="estop">E-STOP</button>
<button data-act="clear" title="clear guard latch">Clear latch</button>
</p>
<pre id="out">ready</pre>
<script>
const out = document.getElementById("out");
async function call(path, body) {
  out.textContent = "...";
  const r = await fetch(path, {method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body || {})});
  out.textContent = JSON.stringify(await r.json(), null, 1);
}
document.querySelectorAll("button").forEach(b => b.onclick = () => {
  const joint = document.getElementById("joint").value;
  const step = parseFloat(document.getElementById("step").value) || 5;
  const sel = {brick: document.getElementById("brick").value,
               port: document.getElementById("port").value};
  const a = b.dataset.act;
  if (a === "ping") call("/api/ping");
  else if (a === "info") call("/api/info");
  else if (a === "state") call("/api/state");
  else if (a === "jog+") call("/api/jog", {joint, delta: step});
  else if (a === "jog-") call("/api/jog", {joint, delta: -step});
  else if (a === "map-set") call("/api/jogmap/set",
    {...sel, joint, ratio: parseFloat(document.getElementById("ratio").value)});
  else if (a === "map-show") call("/api/jogmap");
  else if (a === "map-del") call("/api/jogmap/delete", {joint});
  else if (a === "map-reset") call("/api/jogmap/reset");
  else if (a === "home") call("/api/home",
    {joint, sensor: document.getElementById("sensor").value,
     direction: parseInt(document.getElementById("direction").value)});
  else if (a === "markzero") call("/api/markzero", {joint});
  else if (a === "diag") call("/api/diag", sel);
  else if (a === "stop") call("/api/stop");
  else if (a === "estop") call("/api/estop");
  else if (a === "clear") call("/api/clear");
});
</script>
</body>
</html>
"""


class _Handler(BaseHTTPRequestHandler):
    transport = None
    jog_map = None
    jog_overrides = None
    override_path = None
    zeros = None

    def log_message(self, *args):
        pass

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self):
        if urlparse(self.path).path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            args = self._read_json()
        except ValueError:
            self._send_json({"ok": False, "error": "invalid JSON"}, 400)
            return
        status, obj = self.dispatch_api(path, args)
        self._send_json(obj, status)

    def dispatch_api(self, path, args):
        """Run one API call. Returns ``(status, obj)`` (socket-free)."""
        from src.adapters.pc.mqtt_transport import TransportError

        try:
            result = self._dispatch(path, args)
        except TransportError as exc:
            return 500, {"ok": False, "error": str(exc)}
        except (KeyError, ValueError) as exc:
            return 400, {"ok": False, "error": str(exc)}
        return 200, {"ok": True, **result}

    def _dispatch(self, path, args):
        t = self.transport
        if path == "/api/ping":
            out = {}
            for brick in t.bricks:
                t0 = time.monotonic()
                res = t.send(brick, "ping")
                out[brick] = {"result": res,
                              "ms": round((time.monotonic() - t0) * 1000, 1)}
            return {"ping": out}
        if path == "/api/info":
            return {"info": {b: t.send(b, "info") for b in t.bricks}}
        if path == "/api/state":
            return {"state": t.read_state()}
        if path == "/api/jog":
            entry = self.jog_map[args["joint"]]
            brick, port = entry["brick"], entry["port"]
            ratio = entry["ratio"]
            joint_delta = float(args.get("delta", JOG_STEP_DEG))
            joint_delta = max(-MAX_JOG_STEP_DEG,
                              min(MAX_JOG_STEP_DEG, joint_delta))
            # Coupled solve in delta space (offsets cancel): the jogged
            # joint moves by joint_delta, the rest hold. J1-J3 rows are
            # identity, so those jogs still move exactly one motor.
            dy = {j: 0.0 for j in ("J1", "J2", "J3",
                                   "J4", "J5", "J6")}
            dy[args["joint"]] = joint_delta / ratio
            dm = {}
            for j in ("J1", "J2", "J3"):
                e = self.jog_map[j]
                dm[(e["brick"], e["port"])] = dy[j]
            yw = [dy[j] for j in WRIST_JOINTS]
            mw = [sum(WRIST_C_INV[i][k] * yw[k] for k in range(3))
                  for i in range(3)]
            for mp, m in zip(_wrist_motors(self.jog_map), mw):
                dm[mp] = dm.get(mp, 0.0) + m
            motor_delta = dm[(brick, port)]
            if abs(motor_delta) < MIN_MOVE_MOTOR_DEG:
                return {"joint": args["joint"],
                        "joint_delta": joint_delta,
                        "ratio": ratio, "motor_uid": entry["motor_uid"],
                        "brick": brick, "port": port,
                        "motor_delta": motor_delta,
                        "speed": JOG_SPEED_DEG_S,
                        "executed": False,
                        "warning": "motor move "
                        + str(round(motor_delta, 2)) + " deg < "
                        + str(MIN_MOVE_MOTOR_DEG)
                        + " deg minimum; not executed"}
            enc = {}
            for b in t.bricks:
                enc.update({(b, p): v for p, v in
                            t.send(b, "read")["encoders"].items()})
            targets, coupled = {}, []
            for mp, d in dm.items():
                if abs(d) < MIN_MOVE_MOTOR_DEG:
                    continue
                targets[mp] = enc.get(mp, 0.0) + d
            before = enc.get((brick, port), 0.0)
            target = targets[(brick, port)]
            t.move_motors(targets, speed=JOG_SPEED_DEG_S)
            after = t.send(brick, "read")["encoders"].get(port, 0.0)
            for mp, tgt in targets.items():
                if mp == (brick, port):
                    continue
                coupled.append({"brick": mp[0], "port": mp[1],
                                "motor_delta": dm[mp],
                                "motor_target": tgt})
            return {"joint": args["joint"], "joint_delta": joint_delta,
                    "ratio": ratio, "motor_uid": entry["motor_uid"],
                    "brick": brick, "port": port,
                    "motor_delta": motor_delta, "motor_target": target,
                    "speed": JOG_SPEED_DEG_S,
                    "executed": True,
                    "before": before, "after": after,
                    "moved": after - before,
                    "coupled": coupled}
        if path == "/api/jogmap":
            return {"jog_map": self.jog_map,
                    "overrides": self.jog_overrides}
        if path == "/api/jogmap/set":
            joint = args["joint"]
            if joint not in self.jog_map:
                raise KeyError("unknown joint: " + str(joint))
            patch = {k: args[k] for k in ("ratio", "brick", "port")
                     if k in args}
            entry = _patch_entry(self.jog_map[joint], patch)
            changed = dict(self.jog_overrides.get(joint, {}))
            changed.update({k: entry[k] for k in patch})
            # Mutate in place: handler instances are per-request but
            # these dicts are shared, so every request sees updates.
            self.jog_map[joint] = entry
            self.jog_overrides[joint] = changed
            pathlib.Path(self.override_path).write_text(
                json.dumps(self.jog_overrides, indent=1),
                encoding="utf-8")
            return {"joint": joint, "entry": entry,
                    "jog_map": self.jog_map}
        if path == "/api/jogmap/reset":
            fresh = load_jog_map()
            self.jog_map.clear()
            self.jog_map.update(fresh)
            self.jog_overrides.clear()
            try:
                pathlib.Path(self.override_path).unlink()
            except OSError:
                pass
            return {"jog_map": self.jog_map, "overrides": {}}
        if path == "/api/jogmap/delete":
            joint = args["joint"]
            if joint not in self.jog_map:
                raise KeyError("unknown joint: " + str(joint))
            self.jog_overrides.pop(joint, None)
            fresh = load_jog_map(overrides=self.jog_overrides)
            self.jog_map.clear()
            self.jog_map.update(fresh)
            pathlib.Path(self.override_path).write_text(
                json.dumps(self.jog_overrides, indent=1),
                encoding="utf-8")
            return {"joint": joint, "jog_map": self.jog_map,
                    "overrides": self.jog_overrides}
        if path == "/api/markzero":
            entry = self.jog_map[args["joint"]]
            brick, port = entry["brick"], entry["port"]
            ratio = entry["ratio"]
            pos = float(t.send(brick, "read")["encoders"].get(port, 0.0))
            offset = -ratio * pos
            if self.zeros is None:
                self.zeros = {}
            self.zeros[args["joint"]] = {"motor_position": pos,
                                         "ratio": ratio,
                                         "offset": offset}
            return {"joint": args["joint"], "brick": brick,
                    "port": port, "motor_position": pos,
                    "ratio": ratio, "offset": offset,
                    "zeros": self.zeros}
        if path == "/api/home":
            entry = self.jog_map[args["joint"]]
            brick, port = entry["brick"], entry["port"]
            sensor = args["sensor"]
            direction = int(args.get("direction", -1))
            if direction not in (-1, 1):
                raise ValueError("direction must be -1 or +1")
            speed = float(args.get("speed", HOME_SPEED_DEG_S))
            t.send(brick, "set_guards", {"rules": [{"sensor": sensor}]})
            res = t.send(brick, "run_target",
                         {"port": port,
                          "target": direction * HOME_SEARCH_DEG,
                          "speed": speed, "until_sensor": sensor})
            after = t.send(brick, "read")["encoders"].get(port, 0.0)
            return {"joint": args["joint"], "brick": brick,
                    "port": port, "sensor": sensor,
                    "direction": direction, "speed": speed,
                    "homed": bool(res.get("homed", False)),
                    "position": after}
        if path == "/api/diag":
            brick, port = args["brick"], args["port"]
            res = t.send(brick, "run_angle",
                         {"port": port, "angle": DIAG_ANGLE_DEG,
                          "speed": DIAG_SPEED_DEG_S})
            return {"brick": brick, "port": port,
                    "angle": DIAG_ANGLE_DEG, "speed": DIAG_SPEED_DEG_S,
                    **res}
        if path == "/api/stop":
            t.stop()
            return {}
        if path == "/api/estop":
            t.estop()
            return {}
        if path == "/api/clear":
            t.clear_latch()
            return {}
        raise ValueError(f"unknown endpoint: {path}")


def create_server(transport, host="127.0.0.1", port=8765,
                  jog_map=None, override_path=None, zeros=None):
    """Create (not start) the console server for ``transport``.

    Saved overrides from ``override_path`` (default
    ``config/jogmap.local.json``) apply on top of the model map. An
    explicitly passed ``jog_map`` is used as-is with no overrides.
    ``zeros`` holds mark-zero offsets per joint (starts empty).
    """
    if override_path is None:
        override_path = default_override_path()
    if jog_map is None:
        overrides = read_jog_overrides(override_path)
        jog_map = load_jog_map(overrides=overrides)
    else:
        overrides = {}
    if zeros is None:
        zeros = {}
    handler = type("MiniHandler", (_Handler,),
                   {"transport": transport, "jog_map": jog_map,
                    "jog_overrides": overrides,
                    "override_path": str(override_path),
                    "zeros": zeros})
    return ThreadingHTTPServer((host, port), handler)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Minimal brick test console")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--config", default=None,
                        help="hardware yaml for real bricks "
                             "(default: fake bricks)")
    args = parser.parse_args(argv)

    if args.config:
        from src.adapters.pc.mqtt_transport import MqttTransport

        transport = MqttTransport.from_config(args.config)
        mode = f"real bricks ({args.config})"
    else:
        from src.ev3.fake import build_mp_transport

        transport, _, _ = build_mp_transport(full=True)
        mode = "fake bricks"
    server = create_server(transport, port=args.port)
    print(f"Serving {mode} at http://127.0.0.1:{args.port}/ "
          "(Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        server.server_close()


if __name__ == "__main__":
    main()
