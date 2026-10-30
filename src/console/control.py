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
- M5/M6 wizard shells (filled in later).

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
    for brick in ("left", "right"):
        try:
            state = t.send(brick, "read")
        except Exception:
            latched[brick] = None
            continue
        latched[brick] = bool(state.get("latched"))
        for port, value in (state.get("encoders") or {}).items():
            encoders[(brick, port)] = float(value)
    try:
        directions = read_directions(handler.direction_path)
    except AttributeError:
        directions = {}
    record = latest_homing_record()
    offsets = record.get("offsets", {}) or {}
    homed = {res["joint"]: res.get("status") == "homed"
             for res in record.get("results", [])}
    joints = []
    for joint_id, entry in handler.jog_map.items():
        motor = encoders.get((entry["brick"], entry["port"]))
        ratio = entry["ratio"]
        offset = offsets.get(joint_id)
        try:
            offset = float(offset) if offset is not None else 0.0
        except (TypeError, ValueError):
            offset = 0.0
        joints.append({
            "joint": joint_id,
            "brick": entry["brick"],
            "port": entry["port"],
            "ratio": ratio,
            "motor_deg": motor,
            "joint_deg": (ratio * motor + offset
                          if motor is not None else None),
            "offset": offsets.get(joint_id),
            "homed": bool(homed.get(joint_id, False)),
            "direction": directions.get(joint_id, "unconfirmed"),
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
    return {"ok": True, "joints": joints,
            "ee_mm": list(ee_point(doc)), "envelopes": envelopes,
            "units": "mm,deg"}


class _Handler(mini._Handler):
    direction_path = None
    model_doc = None
    zeros = {}

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
                delta = float(args.get("delta", mini.JOG_STEP_DEG))
                if abs(delta) > MAX_UNCONFIRMED_JOG_DEG:
                    args = dict(args)
                    delta = (MAX_UNCONFIRMED_JOG_DEG
                             if delta > 0 else -MAX_UNCONFIRMED_JOG_DEG)
                    args["delta"] = delta
            return super()._dispatch(path, args)
        return super()._dispatch(path, args)

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        from urllib.parse import urlparse

        parsed = urlparse(self.path).path
        if parsed == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif parsed.startswith("/static/"):
            self._serve_static(parsed)
        else:
            super().do_GET()


def create_server(transport, host="127.0.0.1", port=CONTROL_PORT,
                  jog_map=None, override_path=None,
                  direction_path=None, model_doc=None):
    """Create (not start) the control console server.

    Always binds a loopback host; only 127.0.0.1 is supported.
    """
    if host != "127.0.0.1":
        raise ValueError("control console serves 127.0.0.1 only")
    if override_path is None:
        override_path = mini.default_override_path()
    if direction_path is None:
        direction_path = default_direction_path()
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
                    "zeros": {}})
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
    args = parser.parse_args(argv)
    transport = build_transport(args.config)
    server = create_server(transport, port=args.port)
    print("control console on http://127.0.0.1:" + str(args.port)
          + " (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()


if __name__ == "__main__":
    main()


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
#view3d { width: 100%; height: 420px; display: block; background: #000; }
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
<section><h3>3D 보기 (기본 도형)</h3>
<canvas id="view3d"></canvas>
<div id="glerr"></div>
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
<section><h3>M5·M6 마법사</h3>
<div class="shell">M5 범위 마법사: 준비 중 (나중에 채움)</div>
<div class="shell">M6 패키지 마법사: 준비 중 (나중에 채움)</div>
</section>
</main>
<div id="err"></div>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

window.addEventListener("error", (e) => {
  document.getElementById("glerr").textContent = "오류: " + e.message;
});

async function post(path, args) {
  const r = await fetch(path, {method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(args || {})});
  return r.json();
}
let MODEL = null, LINKS = [];
const qdeg = {};

async function initModel() {
  const m = await post("/api/model", {});
  MODEL = m;
  const canvas = document.getElementById("view3d");
  const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
  renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);
  const camera = new THREE.PerspectiveCamera(50,
    canvas.clientWidth / canvas.clientHeight, 1, 20000);
  camera.position.set(900, -1100, 700);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 80, 0);
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const dir = new THREE.DirectionalLight(0xffffff, 1.2);
  dir.position.set(500, -600, 900);
  scene.add(dir);
  scene.add(new THREE.GridHelper(2000, 20, 0x444444, 0x333333));
  for (const env of m.envelopes) {
    const geo = new THREE.BoxGeometry(env.size_mm[0], env.size_mm[1],
                                      env.size_mm[2]);
    const mat = new THREE.MeshStandardMaterial({color: 0x3a7bd5,
      roughness: 0.6, metalness: 0.1, transparent: true, opacity: 0.85});
    const mesh = new THREE.Mesh(geo, mat);
    mesh.matrixAutoUpdate = false;
    scene.add(mesh);
    LINKS.push({env, mesh});
  }
  const jsel = document.getElementById("w4-joint");
  for (const j of m.joints) {
    const o = document.createElement("option");
    o.textContent = j.id; jsel.appendChild(o);
    qdeg[j.id] = 0;
  }
  window._scene = {renderer, scene, camera, controls};
  requestAnimationFrame(tick);
}
function se3(w, v, th) {
  const c = Math.cos(th), s = Math.sin(th), C = 1 - c;
  const [x, y, z] = w;
  const R = [[c + x*x*C, x*y*C - z*s, x*z*C + y*s],
             [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
             [z*x*C - y*s, z*y*C + x*s, c + z*z*C]];
  const p = [v[0]*th, v[1]*th, v[2]*th];
  const q = new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().set(R[0][0],R[0][1],R[0][2],0, R[1][0],R[1][1],R[1][2],0, R[2][0],R[2][1],R[2][2],0, 0,0,0,1));
  const Rp = new THREE.Vector3(...p).applyQuaternion(q);
  return {R: new THREE.Matrix4().makeRotationFromQuaternion(q), t: Rp};
}
function fkine(qrad) {
  const js = MODEL.joints;
  let T = new THREE.Matrix4();
  const frames = [];
  for (let i = 0; i < js.length; i++) {
    const w = js[i].axis, p = js[i].point;
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
const _m1 = new THREE.Matrix4(), _m2 = new THREE.Matrix4(),
      _p = new THREE.Vector3(), _q = new THREE.Quaternion(),
      _s = new THREE.Vector3();
function tick() {
  requestAnimationFrame(tick);
  const {renderer, scene, camera, controls} = window._scene;
  const qrad = MODEL.joints.map((j) => (qdeg[j.id] || 0) * Math.PI / 180);
  const F = fkine(qrad);
  const F0 = fkine(MODEL.joints.map(() => 0));
  for (const {env, mesh} of LINKS) {
    if (!env.carrier) {
      _m1.makeTranslation(env.center_mm[0], env.center_mm[1],
                          env.center_mm[2]);
      mesh.matrix.copy(_m1);
    } else {
      const idx = MODEL.joints.findIndex((j) => j.id === env.carrier);
      _m1.copy(F[idx]).multiply(_m2.copy(F0[idx]).invert());
      _p.set(env.center_mm[0], env.center_mm[1], env.center_mm[2])
        .applyMatrix4(_m1);
      _q.setFromRotationMatrix(_m1);
      _s.set(1, 1, 1);
      mesh.matrix.compose(_p, _q, _s);
    }
    mesh.matrixWorldNeedsUpdate = true;
  }
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
      "<th>모터각</th><th>홈</th><th>방향</th><th>조그</th></tr>";
    for (const j of o.joints) {
      qdeg[j.joint] = j.joint_deg === null ? 0 : j.joint_deg;
      const jog = j.direction === "confirmed" ? "" : " (최대±5°)";
      html += "<tr><td>" + j.joint + "</td><td>" +
        (j.joint_deg === null ? "?" : j.joint_deg.toFixed(1)) + "</td><td>" +
        (j.motor_deg === null ? "?" : j.motor_deg.toFixed(1)) + "</td><td>" +
        (j.homed ? "완료" : "−") + "</td><td>" +
        (j.direction === "confirmed" ? "확인됨"
         : "<span class='unconf'>방향 미확인</span>") + "</td><td>" +
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
};
initModel().then(() => setInterval(poll, 2000));
</script>
</body>
</html>"""
