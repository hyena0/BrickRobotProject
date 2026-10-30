# MQTT Protocol (Stage 5B)

Local network only. The hub device runs the MQTT broker (Mosquitto on the
PC for the hackathon, embedded broker in the app later). The left brick
(master) connects over Wi-Fi; the right brick (slave) talks to the master
over Bluetooth and the master relays its MQTT traffic. If both bricks
later join Wi-Fi, the slave subscribes directly. Topics and payloads stay
the same.

This document is platform-neutral: it is the contract every hub
implementation (PC now, mobile later) must follow.

Bricks know **ports only**: no joint names, no gear ratios, no sensor
meanings. Port wiring (motor/sensor uid to port) lives in
`config/hardware.local.yaml` (git-ignored, human-provided) on the hub and
in the brick-local config on each brick.

## Topology

```text
Hub device (PC today; mobile later)
  └─ MQTT broker (on the hub) ── Wi-Fi (local LAN)
                                    │
                    Left brick (master) ── Bluetooth ── Right brick (slave)
```

## Broker

- No authentication: `allow_anonymous true`, no accounts, no password
  file, no topic ACL. The broker listens on the hub LAN IP only (never
  `0.0.0.0`), so it is reachable from the local LAN alone. See
  `config/mosquitto.conf.example`.
- The hub address is hardcoded in the left brick program
  (`HUB_ADDRESS` in `left_main.py`, currently the PC LAN IP); the right
  brick finds the master by its Bluetooth name (`MASTER_BT_NAME` in
  `right_main.py`). Topics, payloads and the safety layer stay the same.

## Topics

`<brick>` is `left` (master) or `right` (slave).

| Topic | Direction | QoS | Payload |
|---|---|---|---|
| `robot/cmd/<brick>` | hub to brick | 1 | `{"id", "cmd", "args"}` |
| `robot/ack/<brick>` | brick to hub | 1 | `{"id", "ok", "result" \| "error"}` |
| `robot/state/<brick>` | brick to hub | 0 | encoders (deg) and sensor values, 10-20 Hz |
| `robot/event/<brick>` | brick to hub | 1 | guard triggers, stalls, link loss |
| `robot/estop` | hub to all | 1 | stop everything now |
| `robot/heartbeat` | hub to all | 0 | every 0.5 s |
| `robot/status/<brick>` | broker (last will) | 1 | `online` / `offline` |

## Commands

`cmd` is one of:

- `ping`: no args. Returns `{"brick": <name>}`.
- `info`: no args. Returns ports, motor and sensor types found on the brick.
- `run_target`: move one motor. Args: `{"port": "outA", "target": 90.0, "speed": 60.0}`.
  `target` is an absolute encoder angle in degrees, `speed` in deg/s (capped
  at 300 deg/s). Rejected when `target` is outside the port's motor-angle
  limits (see `set_guards`). A command whose target is already reached
  (within 2 deg, motor idle) is acked with `skipped: true` without
  re-triggering the motor.
- `run_targets`: start several motors together. Args:
  `{"moves": [{"port": ..., "target": ..., "speed": ...}]}`. Assigned a single
  `id`; the ack is sent after all moves finish. Rejected when any `target`
  is outside its port's limits. Moves already at their target are skipped;
  when all are skipped the ack carries `skipped: true`.
- `run_angle`: hardware diagnostic, relative move. Args:
  `{"port": "outA", "angle": 360.0, "speed": 90.0}`. Runs the motor by
  `angle` degrees and returns `{"port", "before", "after", "moved"}` encoder
  readings. Still refused while latched and still rejected when the
  projected absolute angle is outside the port's limits, but it does not
  latch on stall: a weak drive shows up as a small `moved` value instead,
  which is the point of the diagnostic.
- `read`: no args. Returns the latest encoder and sensor snapshot.
- `reset`: no args. Resets encoder offsets on that brick.
- `stop`: no args. Stops all motors on that brick immediately.
- `set_guards`: configure sensor guards and motor-angle limits. Args:
  `{"rules": [{"sensor": "in1", "invert": false}], "limits": [{"port": "outA", "min": -90.0, "max": 90.0}]}`.
  An empty rule list clears the sensor-guard rules (stall/heartbeat/Bluetooth
  watchdogs stay on). `limits` omitted leaves motor-angle limits unchanged;
  an empty `limits` list clears them. Limits are numbers only, per port.
- `clear_latch`: no args. Clears the guard latch so moves are accepted again.

Every `robot/cmd/<brick>` message must carry a unique string `id`. The brick
replies on `robot/ack/<brick>` with the same `id`, `ok: true/false`, and
either `result` or `error`.

## Events

Published on `robot/event/<brick>` with QoS 1:

```json
{"brick": "left", "event": "sensor_guard", "detail": {"sensor": "in1"}}
{"brick": "right", "event": "stall", "detail": {"port": "outA"}}
{"brick": "left", "event": "heartbeat_timeout", "detail": {}}
{"brick": "right", "event": "bluetooth_loss", "detail": {}}
```

`event` is one of `sensor_guard`, `stall`, `heartbeat_timeout`,
`bluetooth_loss`, `limit_reject`, `estop`, `motor_error`, `loop_error`.
`motor_error` carries the failing `port` (or `cmd`) and the driver error;
`loop_error` carries an unexpected main-loop error. Both latch the brick
instead of killing the program.

## State

Published on `robot/state/<brick>` at 10-20 Hz (QoS 0):

```json
{
  "brick": "left",
  "encoders": {"outA": 12.5, "outB": -3.0},
  "sensors": {"in1": 0, "in2": 1},
  "latched": false,
  "latch_reason": "",
  "missing": ["in3"]
}
```

Encoder values are degrees. Touch sensors report 0/1. `missing` lists
configured ports with nothing physically connected (reported at startup
instead of crashing). `latch_reason` names why the brick latched
(`stall:<port>`, `motor_error:<port>`, `motor_io:<cmd>`, `loop_error`,
...).

## Safety (binding)

The on-brick safety layer is local and never depends on the network, except
that the master forwards a stop to the other brick as soon as it can:

- Sensor guards run every ~10 ms. Any configured sensor triggering outside a
  homing move stops all motors on that brick, latches, and publishes an
  event. The master forwards the stop to the other brick at once.
- Homing exception: a move carrying `{sensor, motor}` stops that motor when
  the sensor triggers. This ends homing and is not an error.
- Stall guard: a motor commanded but near zero speed for > 300 ms stops and
  reports. The firmware `stalled()` signal (max actuation, no motion) trips
  the same latch; the brick pushes its 3 deg/s / 300 ms tolerances to the
  firmware at motor open when that API exists.
- Motor I/O errors (e.g. driver `EIO` while braking a hard-stalled motor)
  never kill the brick: every motor call that fails stops the rest, latches
  with `motor_error`, and reports. A failing main-loop pass latches with
  `loop_error` the same way.
- Motor-angle limits: per-port min/max from `set_guards`. A `run_target` /
  `run_targets` whose target is outside the limits is rejected before moving
  (ack `ok: false`); the rejection does not latch.
- Heartbeat watchdog: no heartbeat for 2 s stops everything on that brick.
  This also covers a mobile hub going to the background or locking.
- Bluetooth watchdog: slave losing the master link stops everything on slave.
- Latch: after a guard stop, moves are refused until `clear_latch`.

Speed cap is 300 deg/s during Stage 5B. Stall trip is below 3 deg/s for
more than 300 ms; settle is within 2 deg at below 3 deg/s. Pybricks
`control.limits` keep firmware defaults (1000 deg/s, 2000 deg/s^2,
200 mNm); only the stall tolerances above are pushed to the firmware.

## Fallback

If MQTT on the brick is unstable by end of Day 4, keep the same message
format (same JSON `id`/`cmd`/`args` and ack shape) over a plain TCP socket
from the master. Only the transport changes. If Bluetooth between bricks is
unstable, connect the slave to Wi-Fi and subscribe directly.
