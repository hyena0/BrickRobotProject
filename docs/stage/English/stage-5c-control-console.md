# Stage 5C: Robot Control Program and Web Console

**Latest:** Day 6 · **Requires:** Stage 5A (limits, safe poses, robot package v0) and Stage 5B (hardware connectivity, for physical steps).

**Goal:** build the **controller package** (PLAN v2 section 5) for the PC: a robot-agnostic control program in four layers plus a local web console to control and validate the robot (watch real sensors, capture physical boundaries, home, check directions, replay safe poses, measure accuracy). The same package runs against the real robot or the MuJoCo simulation, and is structured so it can later be ported to a mobile hub (Pad–robot) without changing the robot package, UI, protocol or bricks.

## Design principles

- **One hub, one controller:** the device running the controller package is the hub. It hosts the broker and is the only device that controls the robot in a session.
- **Robot-agnostic:** everything robot-specific comes from the **robot package** (`robot_package.json`). No joint names or numbers in code.
- **Four layers:** robot package (data) → core logic (platform-neutral) → adapters (per platform) → UI (shared HTML/JS).
- **Core has no direct I/O:** no file, network, OS, MuJoCo or web-framework calls inside the core; everything goes through adapters.
- **Safety in two places below the UI:** the core enforces limits, coupled constraints, path checks and speed caps; bricks enforce motor-angle limits, sensor guards and watchdogs. The UI can never bypass them.
- **A tool, not the product:** the console supports validation and demos; the project's main feature remains inference → motion.
- No Streamlit.

## Architecture

```
┌──────────────────────────────────────────────┐
│ ④ UI · src/ui/  (HTML/JS + three.js, vendored)│ PC: browser · Mobile (later): WebView
├──────────────────────────────────────────────┤
│   UI bridge (PC: FastAPI + WebSocket shell)   │ Mobile (later): JS ↔ native bridge
├──────────────────────────────────────────────┤
│ ② Core · src/core/  (pure Python)             │ FK/IK, limits, coupled constraints,
│                                               │ is_path_safe, synchronized moves,
│                                               │ homing sequence, wizard state machines,
│                                               │ protocol encode/decode, safety checks
├──────────────────────────────────────────────┤
│ ③ Adapters · src/adapters/pc/                 │ Transport: MQTT (5B) | Sim (MuJoCo)
│                                               │ Storage: files · Lifecycle: process
├──────────────────────────────────────────────┤
│ ① Robot package · packages/<robot>/           │ robot_package.json (from 5A, updated here)
└──────────────────────────────────────────────┘
```

### Adapter interfaces (`src/core/ports.py`, abstract)

| Interface | Methods | PC implementation (now) | Mobile (roadmap) |
|---|---|---|---|
| `Transport` | `send(brick, cmd)`, `on_state(cb)`, `on_event(cb)`, `estop()`, `heartbeat()` | `mqtt_transport.py` (5B), `sim_transport.py` (MuJoCo) | Embedded broker in the app |
| `Storage` | `load_package()`, `save_package(pkg)`, `write_record(session, item)` | Files under `packages/`, `runs/` | App sandbox storage |
| `Lifecycle` | `on_suspend()`, `on_resume()`, `on_exit()` | Process signals | App backgrounding / screen lock |
| `Clock` | `now()`, `sleep(dt)` | System time | Platform timer |

## Local-only policy (no external services)

| Rule | Detail |
|---|---|
| No CDNs or web fonts | three.js and every JS/CSS file vendored into `src/ui/vendor/` (with licence files). System fonts only. No analytics. |
| Access | The UI runs on the hub itself. PC: server binds to `127.0.0.1` only. |
| WebSocket protection | Every WebSocket connection sends a session token (generated at start); the server checks the `Origin` header (only its own address). Blocks other web pages in the browser from driving the robot. |
| No exposure | No router port forwarding. |
| Offline core | Homing, jog, limit capture, pose replay and Cartesian test need no internet. Meta API features (Spark, SAM) live only in Stage 6 modules and are optional. |
| MQTT | The Transport adapter connects to the local broker with the `hub` account (5B). |

## Tasks

### 5C.1 Robot package loader and core (≈3 h) · `src/core/`, `src/package/`

- Loader validates `robot_package.json` against `docs/robot_package.md` (`schema_version`, `package_version`).
- Builder merges the 5A package v0 with port wiring (`config/ports.local.yaml`), homing config and offsets → new `package_version`.
- Core functions: joint ↔ motor mapping from `C` and port wiring; FK/IK; `check_limits`, `check_coupled`, `is_path_safe` (limits + inequalities only; no mesh checks at runtime); synchronized move planning; homing sequence; protocol encode/decode.
- Commands handled by the core: `jog(joint, delta)`, `move_joints(q)`, `move_pose(target)`, `home(joint | all)`, `stop`, `clear_latch`. Every motion passes the safety checks; rejections return a reason.
- On connect to the real robot: send `set_guards` (sensor guards + motor-angle limits from the package) before any motion is allowed.

### 5C.2 PC adapters and UI bridge (≈2 h) · `src/adapters/pc/`, `src/console/`

- `mqtt_transport.py` (from 5B), `sim_transport.py` (MuJoCo, primitive-geometry MJCF generated from the package), `file_storage.py`, `process_lifecycle.py`.
- `src/console/server.py`: FastAPI shell that serves `src/ui/` and bridges WebSocket messages to the core. Contains no robot logic.
- State stream to the UI at 20 Hz: joint and motor angles, sensor states, brick status, latch state, package version.

### 5C.3 Web UI (≈4 h) · `src/ui/`

| Panel | Contents |
|---|---|
| **Status bar** | Backend (real/sim), brick online/offline, heartbeat, latency, latch state, package version, **E-STOP button always visible** (also Space key on PC) |
| **3D view** | Robot mirroring live joint angles. Display modes: **Primitives** (capsules/cylinders between joints) and **Envelopes** (`link_envelopes` boxes) from robot package data, plus **Private mesh** (real LEGO shapes from the private visual pack, development and video only; see below). Limits as arcs; target ghost. Low-detail mode for small screens |
| **Joints** | Per joint: angle, limits (source: sensor/collision/measured), **press-and-hold** jog (±1°, ±5°), speed slider capped at the safe maximum |
| **Sensors** | Live state of each sensor and its associated joint |
| **Wizards** | Homing, direction check, limit capture, pose replay, Cartesian test |
| **Log** | Events, rejections with reasons, results |

- Responsive and touch-friendly from the start (same code will run in a mobile WebView).
- The UI talks only to the bridge API; it never contains kinematics or safety logic.

#### Private mesh mode (development, testing, video; excluded from distribution)

Real LEGO shapes are used to verify motion before full validation and to record the demo video, but are never distributed.

| Rule | Detail |
|---|---|
| Source | `assets/private/visual_pack/<robot>/` from Stage 5A (git-ignored) |
| Enabled only when | the server starts with `--private-visuals` **and** the visual pack exists **and** its `manifest.json` matches the loaded package version |
| Serving | separate route `/private/visual/…`, same token and `Origin` checks, `Cache-Control: no-store`; never bundled into `src/ui/`, never cached for offline use |
| Default | off; the UI shows Primitives. Without the pack, the toggle is hidden |
| Overlay check | "Mesh + primitives" overlay: if the mesh and the primitive chain drift apart while moving, the package or the visual pack is wrong; the console flags the link |
| Video | Screen capture of the private mesh mode is allowed in the demo video with designer credit (PLAN v1 section 4a). The files themselves are never published |
| Mobile | Not part of the mobile app distribution. A local, manual copy on a test device is a roadmap option only |

### 5C.4a Release guard

- `python -m src.release.check` fails if any `.glb`, `.gltf`, `.obj`, `.stl`, `.io`, `.ldr`, `.mpd`, image of the robot, or `visual_pack` path is tracked by git, present in `src/ui/`, inside `packages/`, or in a release archive.
- The release build of the UI is produced without the private route (`--private-visuals` absent → route not registered).
- Runs in tests and before Stage 8 submission.

### 5C.4 Wizards (state machines in the core, screens in the UI)

**Homing wizard**
- Per joint at 10 % speed using the homing guard; J4 uses the cross-brick path. Shows trigger → stop latency and overtravel; warns if overtravel ≥ the 5° margin.
- Shows the simulated sensor angle (5A.1) next to the physical home; after confirmation writes zero offsets into the robot package (new `package_version`).

**Direction check wizard**
- +15° per joint; the user confirms the motion matches the arrow in the 3D view. A mismatch is recorded; the wizard tells you to fix the **model** and rebuild the package.

**Limit capture wizard** (physical sensor boundaries)
- Sensor side: move slowly (5 % speed) toward the sensor until it triggers; the guard stops the motor; record the angle as `measured`.
- Opposite side: jog in small steps toward the simulated collision boundary; the user stops when clearance looks small and marks it; the value cannot exceed the simulated boundary.
- Final limit per side = the more conservative of simulated and measured, minus the 5° margin. Written into the robot package with `limit_source: measured`.

**Pose replay wizard**
- Loads the package's safe poses; runs small → large at 30 % speed; records commanded vs. encoder angles; shows the matching render (PC only, from `assets/private/`) next to the live 3D view; reminds the user to take the tripod photo.

**Cartesian test wizard**
- The 6 Stage 5 targets × 3 repeats; the user types the ruler measurement; the core computes error and repeatability.

### 5C.5 Reports · `runs/console/<session>/`

- `report.json` + `report.md`: homing results and latency, directions, measured vs. simulated limits, pose replay errors, Cartesian accuracy, package version used.
- Side-by-side images (render | photo) for the video.

### 5C.6 Sim parity and safety tests · `tests/test_console.py`

- Every wizard runs end to end on the `sim` Transport.
- Safety: out-of-range jog, folding target, unsafe path rejected with reasons; E-STOP during a move stops the sim; missing heartbeat stops the sim.
- **Static check:** `src/core/` imports no `os`, `socket`, `paho`, `mujoco`, `fastapi` or file I/O modules.

### 5C.7 Conformance suite · `conformance/`

The porting contract for any future core implementation (e.g., mobile).

- The Python core exports golden cases as JSON: FK/IK inputs and outputs (with tolerances), limit/coupled/path verdicts, protocol messages for each command, wizard state transitions for scripted inputs, safety rejections.
- `python -m conformance.run` replays all cases against the Python core; it must pass 100 %.
- Cases are generated from the robot package and from at least one procedural robot (Stage 7), so they are not tied to one robot.

### 5C.8 Mobile porting notes · `docs/mobile_port.md` (design only)

- App shell with a WebView hosting `src/ui/` unchanged; JS ↔ native bridge replacing the FastAPI bridge.
- Embedded MQTT broker as the mobile Transport; same topics, accounts and ACLs as 5B.
- Core re-implementation in the platform language, accepted only when it passes the conformance suite.
- Lifecycle: on suspend, send `estop` and stop heartbeats; bricks' watchdog is the backstop.
- Storage of robot packages in the app sandbox; import via file share or local network.

## Physical run order

1. Start the hub (PC): broker, console; select `real`; confirm guards are set.
2. Homing wizard (all joints).
3. Direction check wizard.
4. Limit capture wizard.
5. Pose replay wizard (safe poses).
6. Cartesian test wizard.

## Exit criteria

- [ ] All wizards pass on the `sim` Transport (tests green); static check on `src/core/` passes.
- [ ] On the real robot: homing 3× in a row for every joint; J4 latency and overtravel recorded.
- [ ] All joint directions confirmed.
- [ ] Measured limits captured, compared with simulated limits, and written into the robot package.
- [ ] 10 safe poses reached without contact; side-by-side images saved.
- [ ] Cartesian error and repeatability recorded.
- [ ] A second robot package (procedural arm) loads and runs in `sim` without code changes.
- [ ] Conformance suite exported; Python core passes 100 %.
- [ ] `docs/mobile_port.md` written.
- [ ] **Offline test:** with the router's internet (WAN) disconnected, the console loads fully (3D view included) and all wizards run.
- [ ] No external URLs in served pages (automated scan of `src/ui/`).
- [ ] WebSocket without a valid token or with a foreign `Origin` is refused.
- [ ] Private mesh mode works with `--private-visuals` and the visual pack; the mesh + primitives overlay stays aligned through the safe poses.
- [ ] Without the flag or the pack, the private route is not registered and the toggle is hidden.
- [ ] Release guard passes: no meshes, model files or robot images in git, `src/ui/`, `packages/` or release archives.

## Fallbacks

- UI running late → run the same wizards from a CLI (`python -m src.console.cli <wizard>`) on the same core; finish the UI later.
- Layer separation slowing progress → keep the robot package format, the core/adapter boundary and the conformance export; keep only the PC adapters.
- Measured and simulated sensor boundary differ by > 10° → operate on the measured value, flag the joint in `LOG.md`, re-check the Stage 2.7 association.
- Large poses risky → finish with the 5 small poses; narrow limits; record.

## Safety

- First physical motion of every joint at 10 % speed; poses at 30 %.
- Hand on E-STOP for every physical step; clear the table.
- Never relax guards or limits to make a test pass.

## Token estimate

Development $7–14 (largest coding stage, including layer separation and conformance export).
