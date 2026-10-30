# Stage 5C Addendum: 5B Carry-over and Reduced Scope

**Status: add-on document.** `stage-5c-control-console.md` stays unchanged; this document takes precedence. The 5C session reads this document first.

## 1. Decisions that differ from the existing docs

| Item | Existing doc | Current decision |
|---|---|---|
| Brick platform | ev3dev2 (5B doc) | **Pybricks MicroPython** (`src/ev3/brick_mp/`) |
| MQTT | accounts + ACL | **no authentication**; broker bound to the hub's LAN IP only |
| Console | 4 layers + three.js UI + mobile | **mini console (`src/console/mini.py`) + step-by-step wizards first** |
| Deferred | – | three.js 3D UI, sim parity check, conformance tests, mobile notes, `src/core` split |

Deferred items are done only if time remains after M1–M7.

## 2. Current state

- **Exists:** package loader/builder (`src/package`, `src/codegen`), `robot_package.json`, joint ↔ motor conversion (`src/kin`), mini console, homing API (`/api/home`, now sets the sensor rule first). 293 tests pass.
- **Missing:** direction check, limit capture, pose replay wizards, report.

## 3. Work order

| Step | Task | Who | Exit criteria |
|---|---|---|---|
| **M1** | Finish 5B physical checks | human + agent | all items in the M1 table pass |
| **M2** | Confirm J4/J5/J6 port mapping | human (diagnostics) → agent (update model) | ports and directions of all 6 joints in `ports.local.yaml` and the package |
| **M3** | Homing | agent → human | every joint with a sensor reports `homed: true`; position error recorded |
| **M4** | Direction check | agent → human | each joint's + direction matches the model; sign saved to config |
| **M5** | Limit capture | agent → human | measured limits per joint (`limit_source: measured`), compared with 5A |
| **M6** | Safe pose replay + report | agent → human | 10 5A safe poses replayed; error per pose recorded |
| **M7** | Docs and LOG cleanup | agent | `stage5_dev.md`, `LOG.md` and 5B/5C docs match the current state |

### M1 physical checks

| Item | Pass |
|---|---|
| Jog every motor ±3° (joint degrees) | moves without being cut off by stall detection; if cut off, check stall threshold and speed |
| 5 sensors | each press shows an event for the right port |
| Cross-brick latency | right sensor → PC display within 1 s |
| E-STOP | both bricks stop; time to stop recorded |
| Heartbeat | stops within 2 s after the console is closed |
| Bluetooth loss | turning off the right brick also stops the left |
| Limit rejection | out-of-range targets are rejected |
| Offline | all of the above work with the internet cable unplugged |

## 4. Common wizard rules

- **Robot-package driven:** joint names, ports, ratios and limits all come from the robot package. No MR JOS-specific values in code. The same wizards are reused for the 5D generated robot.
- **Joint degrees:** inputs are in joint degrees; the screen shows both joint and motor degrees. Conversion happens on the PC side.
- **First motions:** within ±5° joint, low speed, arm supported by hand.
- **Results:** saved as JSON under `runs/stage5c/<date>/` (private, git-ignored), plus a short Markdown summary.
- **Physical checks from 5A:** check these during M5:
  - whether the D5–D4 contact actually blocks motion
  - whether the J1 range ([-50°, 39°]) is really that narrow

## 5. Wizard notes

- **M3 Homing:** set the sensor rule → move slowly (30°/s) toward the sensor → stop on trip → set zero. If the sensor does not trip within a timeout, stop and report failure. Joints without a sensor home by stall.
- **M4 Direction:** move each joint +5° → human answers "correct/reversed" → if reversed, save the sign and re-check.
- **M5 Limits:** step each joint toward its end; when the human selects "stop here", record it → compare with the 5A limit and keep the narrower value.
- **M6 Pose replay:** replay the 5A safe poses one by one at low speed → record motor angle error → write the report.

## 6. Session rules

- Time-box each task; if the criteria are missed, stop and report.
- **External integration rule:** for anything this environment cannot run (physical hardware), write minimal code → check with the fake brick → hand over the commands for the human to run, then stop. Do not refine it before real results come back.
- Session start: `musec` (contributor), `/effort medium`, Auto-review off.
- After each step, write a summary to `stage5_dev.md` and one line to `LOG.md`. Commit only when asked.

## 7. Start prompt (Muse)

> Read `docs/stage/English/stage-5c-addendum.md` first, and look up `stage-5c-control-console.md` only where needed. This addendum takes precedence over the existing doc. Start with M1: give me a short list of the physical checks and the commands I should run; when I paste the results, move to the next item. First find out why the ±3° jog is cut off by stall detection (stall threshold, speed, joint ↔ motor conversion).
