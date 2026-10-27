# Stage 0: Setup

**Latest:** Day 1 · **Goal:** a working repo, a reliable and safe path to drive every EV3 motor and read every sensor, and a measured token-cost baseline.

## Communication topology

```
PC ── Wi-Fi (local LAN) ── MQTT broker (Mosquitto on the PC)
                                │
                        Left brick (master) ── Bluetooth ── Right brick (slave)
```

- The **Left (master)** brick connects to the MQTT broker over Wi-Fi.
- The **Right (slave)** brick talks to the master over Bluetooth; the master relays its MQTT traffic.
- If both bricks later join Wi-Fi, the slave subscribes directly. Topics and payloads stay the same.
- Local network only. No internet exposure.

## Prerequisites

- Both bricks run the same runtime with Python (ev3dev with `ev3dev2`, or Pybricks MicroPython). Do not mix APIs.
- Master has a working Wi-Fi dongle and a fixed IP; bricks are paired over Bluetooth.
- PC: Python 3.11, Mosquitto installed, Muse Code installed, `META_API_KEY` set.
- Billing page shows the $150 free credit.

## Tasks

### 0.1 Repo scaffold (≈0.5 h)

- Folder layout from `AGENTS.md`.
- `.gitignore`: `.env`, `assets/private/`, `eval/ground_truth/`, `runs/`, `config/*.local.yaml`, `__pycache__/`, `.venv/`.
- `pyproject.toml`: `openai`, `numpy`, `scipy`, `mujoco`, `pyyaml`, `paho-mqtt`, `pytest`.
- `README.md` stub with an empty "Built / External / Pre-existing / Evaluation only / Human-provided" table.
- Commit: `Initial scaffold and planning docs`.

### 0.2 MQTT protocol · `docs/protocol.md`

| Topic | Direction | QoS | Payload |
|---|---|---|---|
| `robot/cmd/<brick>` | PC → brick | 1 | `{"id", "cmd", "args"}` |
| `robot/ack/<brick>` | brick → PC | 1 | `{"id", "ok", "result" \| "error"}` |
| `robot/state/<brick>` | brick → PC | 0 | encoders (deg) and sensor values, 10–20 Hz |
| `robot/event/<brick>` | brick → PC | 1 | guard triggers, stalls, link loss |
| `robot/estop` | PC → all | 1 | stop everything now |
| `robot/heartbeat` | PC → all | 0 | every 0.5 s |
| `robot/status/<brick>` | broker (last will) | 1 | `online` / `offline` |

Commands: `ping`, `info` (ports, motor and sensor types), `run_target`, `run_targets` (start together), `read`, `reset`, `stop`, `set_guards`, `clear_latch`.

Bricks know **ports only**: no joint names, no ratios, no sensor meanings.

### 0.3 Brick program · `src/ev3/brick/`

- `master.py`: MQTT client (paho-mqtt on ev3dev, `umqtt` on Pybricks), executes its own commands, relays `robot/*/right` traffic over Bluetooth.
- `slave.py`: Bluetooth client of the master, executes its own commands.
- Shared `motors.py` / `sensors.py` / `guards.py`.

### 0.4 Safety layer (on each brick, local, no network in the loop)

- **Sensor guards:** each brick checks its own sensors every ≈10 ms. Default rule: any configured sensor triggering **outside a homing move** stops **all** motors on that brick, latches, publishes an event; the master forwards the stop to the other brick at once.
- **Homing exception:** during a homing move, the brick receives `{sensor, motor}` in the command and stops that motor when the sensor triggers (this ends homing; it is not an error).
- **Stall guard:** a motor whose speed drops near zero under command for > 300 ms stops and reports (for joints without a sensor).
- **Heartbeat watchdog:** no heartbeat for 2 s → stop all.
- **Bluetooth watchdog:** slave loses the master link → stop all on slave.
- **Latch:** after a guard stop, moves are refused until `clear_latch`.

### 0.5 Host driver · `src/ev3/driver.py`

- Reads `config/hardware.local.yaml` (git-ignored): broker address, brick names (`left` = master, `right` = slave).
- API: `move_motors({(brick, port): angle}, speed)`, `read_state()`, `stop()`, `home_motor(brick, port, sensor_port, direction, speed)`, `on_event(callback)`.
- **Emergency stop:** a keyboard key publishes `robot/estop` from any script.
- Sends heartbeats in a background thread.

### 0.6 Connectivity check · `src/ev3/check.py`

- Ping both bricks, print `info` (all motors and sensors found).
- Jog each motor +10° / −10° at low speed; read back; report error and round-trip latency (master and slave separately).
- Print live sensor values; press each touch sensor by hand and confirm the event arrives.

### 0.7 Token baseline

- Before: note the Billing balance. Build 0.2–0.6 with Muse Code (`muse-spark-1.3-contributor`, effort `medium`).
- After ≈1 h: record turns, input/cached/output tokens and cost in `LOG.md`. Compare with the estimate.

### 0.8 Admin

- Send the permission email to the designer. Submit Modal and Baseten credit forms.

## Safety

- Speed cap 100 °/s and ±10° jogs only (joint limits unknown).
- Support the arm by hand during each motor's first jog.

## Exit criteria

- [ ] Every motor on both bricks moves and reads back within ±3°.
- [ ] Pressing any touch sensor during a jog stops all motors on that brick within 50 ms, and the other brick within 300 ms.
- [ ] `robot/estop` stops both bricks within 300 ms.
- [ ] Killing the PC script stops both bricks (heartbeat watchdog).
- [ ] Breaking Bluetooth stops the slave.
- [ ] Token baseline recorded; designer email and credit forms sent.

## Fallback

If MQTT on the brick is unstable by end of Day 1, keep the same message format over a plain TCP socket from the master.

## Muse Code prompt (example)

> Stage 0 tasks 0.2–0.5 in docs/stages/stage-0-setup.md. Bricks run [ev3dev2 / Pybricks]. Write docs/protocol.md, the master/slave brick programs with the safety layer, and the host driver. Bricks must stay robot-agnostic (ports only). Add PC-side tests with fake motor/sensor classes and a fake Bluetooth link.
