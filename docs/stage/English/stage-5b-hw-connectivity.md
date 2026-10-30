# Stage 5B: Hardware Connectivity (parallel track)

**Latest:** Day 4 · **Runs in parallel** with Stages 1–5 (no dependency on the inference pipeline). **Required before** the physical parts of Stage 5C.

**Relation to existing docs:** this stage carries out and verifies on the real bricks the hardware items first described in Stage 0 (tasks 0.2–0.6) and Stage 5 (5.3–5.4 bring-up). Those documents are kept unchanged; this stage is the operative definition from PLAN v2 on.

**Goal:** a reliable, safe, robot-agnostic link from the **hub device** to both EV3 bricks: every motor can be commanded, every sensor read, and every safety stop works. The hub is the single device running the controller package (PLAN v2 section 5): the PC for the hackathon, a mobile device later.

## Topology

```
Hub device (PC today; iPad/phone later)
  └─ MQTT broker (on the hub) ── Wi-Fi (local LAN)
                                    │
                    Left brick (master) ── Bluetooth ── Right brick (slave)
```

- Master connects to the hub's broker over Wi-Fi and relays slave traffic over Bluetooth.
- Bricks know **ports only**: no joint names, ratios or sensor meanings.
- **Broker-location independence:** bricks do not know or care what kind of device the hub is. Moving the hub from a PC to a mobile device changes only the hub address in the brick config; topics, payloads and brick code stay the same.
- Local network only.

## Local-only policy (no external services)

The whole hardware path must work with **no internet connection at all**, following the same principle as the earlier LEGO robot project (principle only; no code is reused).

| Rule | Detail |
|---|---|
| Broker on the hub | PC (hackathon): Mosquitto installed and running on the PC. Mobile (roadmap): broker embedded in the app. **No cloud or public brokers.** |
| Hub address | Bricks read the hub address from their config: fixed LAN IP or a local `.local` name (mDNS, LAN only). No cloud discovery. |
| Bind locally | Listeners on `127.0.0.1` and the hub's LAN IP only. |
| Authentication | `allow_anonymous false`; two accounts: **`hub`** and `brick_left`. Passwords live in `config/*.local.*` (git-ignored). |
| Topic ACL | `brick_left`: publish `robot/ack/#`, `robot/state/#`, `robot/event/#`, `robot/status/#`; subscribe `robot/cmd/#`, `robot/estop`, `robot/heartbeat`. `hub`: the reverse. |
| No exposure | No router port forwarding. Firewall allows port 1883 only from the LAN subnet. |
| No external calls | Brick programs, driver and check tools make no internet requests. Meta API calls belong only to Stage 6 modules. |

Config files:

- `config/mosquitto.conf.example` (committed, no secrets): listeners, `allow_anonymous false`, `password_file`, `acl_file`.
- `config/mosquitto.local.conf`, `config/mosquitto.passwd`, `config/mosquitto.acl` (git-ignored).
- Brick side: `hub_address`, port, account and password in a brick-local config file (not committed).

## Tasks

### 5B.1 Network and broker

- Mosquitto on the PC (hub) with the local-only config above; fixed IPs for the hub and the master.
- `config/hardware.local.yaml` (git-ignored): broker IP, port, account names, brick names (`left` = master, `right` = slave). Passwords read from a git-ignored file, never hard-coded.
- Checks: `mosquitto_sub` with the `hub` account sees a test publish from the master; an anonymous client is refused; a client with the wrong account cannot publish to `robot/cmd/#`.

### 5B.2 Protocol · `docs/protocol.md`

| Topic | Direction | QoS | Payload |
|---|---|---|---|
| `robot/cmd/<brick>` | hub → brick | 1 | `{"id", "cmd", "args"}` |
| `robot/ack/<brick>` | brick → hub | 1 | `{"id", "ok", "result" \| "error"}` |
| `robot/state/<brick>` | brick → hub | 0 | encoders (deg), sensor values, 10–20 Hz |
| `robot/event/<brick>` | brick → hub | 1 | guard triggers, stalls, link loss |
| `robot/estop` | hub → all | 1 | stop everything now |
| `robot/heartbeat` | hub → all | 0 | every 0.5 s |
| `robot/status/<brick>` | broker (last will) | 1 | `online` / `offline` |

Commands: `ping`, `info`, `run_target`, `run_targets`, `read`, `reset`, `stop`, `set_guards`, `clear_latch`.

The protocol document is platform-neutral: it is the contract every hub implementation (PC now, mobile later) must follow.

### 5B.3 Brick programs · `src/ev3/brick/`

- `master.py` (MQTT client to the hub + Bluetooth relay), `slave.py` (Bluetooth client), shared `motors.py`, `sensors.py`, `guards.py`.
- Hub address, account and password come from the brick-local config; never hard-coded.

### 5B.4 On-brick safety (local, no network in the loop)

Safety decisions stay on the bricks for **any** hub type.

- **Sensor guard:** any configured sensor triggering outside a homing move → stop all motors on that brick, latch, publish event; master forwards the stop to the other brick.
- **Homing guard:** a homing command carries `{sensor, motor}`; that sensor stops that motor (end of homing, not an error).
- **Stall guard:** commanded motor near zero speed for > 300 ms → stop and report.
- **Motor-angle limits:** per-port min/max received via `set_guards` (numbers only).
- **Heartbeat watchdog:** no heartbeat for 2 s → stop all. This also covers a mobile hub going to the background or locking its screen. **Bluetooth watchdog:** slave loses master → stop all on slave.
- **Latch:** moves refused until `clear_latch`.

### 5B.5 Host driver (PC Transport adapter) · `src/adapters/pc/mqtt_transport.py`

- Implements the controller package's **Transport** interface for the PC: `send(brick, cmd)`, `on_state(cb)`, `on_event(cb)`, `estop()`, heartbeat thread.
- Convenience wrapper for tools and checks: `move_motors({(brick, port): angle}, speed)`, `read_state()`, `stop()`, `home_motor(...)`, `set_guards(...)`.
- Keyboard e-stop from any PC script.
- Holds no robot knowledge; the core logic (Stage 5C) uses it only through the Transport interface.

### 5B.6 Connectivity check · `src/ev3/check.py`

- Ping both bricks; print `info` (6 motors, 5 sensors found).
- Jog each motor ±10° at low speed; read back; round-trip latency per brick (master and slave separately).
- Live sensor values; press each touch sensor and confirm the event.
- **Cross-brick latency:** a sensor on one brick triggering a stop on the other (needed for J4 homing). Record the time.

## Safety

- Speed cap 100 °/s, ±10° jogs only (no joint limits are known at this stage).
- Support the arm by hand during each motor's first jog.

## Exit criteria

- [ ] Both bricks online; `info` lists all 6 motors and 5 sensors.
- [ ] Every motor jogs and reads back within ±3°; latency recorded per brick.
- [ ] Touch sensor during a jog: same-brick stop ≤ 50 ms, other brick ≤ 300 ms (cross-brick latency recorded).
- [ ] `robot/estop` stops both bricks ≤ 300 ms.
- [ ] Killing the hub process stops both bricks (heartbeat watchdog).
- [ ] Breaking Bluetooth stops the slave.
- [ ] Motor-angle limits via `set_guards` reject an out-of-range target.
- [ ] **Offline test:** with the router's internet (WAN) disconnected, the full connectivity check passes.
- [ ] Anonymous MQTT connection refused; ACL blocks publishing to `robot/cmd/#` from a non-`hub` account.
- [ ] No router port forwarding; firewall limits 1883 to the LAN subnet.
- [ ] Changing only `hub_address` in the brick config (e.g., to another LAN machine running the same broker config) reconnects without code changes.

## Fallback

If MQTT on the brick is unstable by end of Day 4, keep the same message format over a plain TCP socket from the master. If Bluetooth between bricks is unstable, connect the slave to Wi-Fi and subscribe directly.
