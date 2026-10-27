# MQTT Protocol (Stage 0)

Local network only. The PC runs a Mosquitto broker. The left brick (master)
connects over Wi-Fi; the right brick (slave) talks to the master over
Bluetooth and the master relays its MQTT traffic. If both bricks later join
Wi-Fi, the slave subscribes directly. Topics and payloads stay the same.

Bricks know **ports only**: no joint names, no gear ratios, no sensor
meanings. Port wiring (motor/sensor uid to port) lives in
`config/hardware.local.yaml` (git-ignored, human-provided).

## Topology

```text
PC -- Wi-Fi (local LAN) -- MQTT broker (Mosquitto on the PC)
                              |
                  Left brick (master) -- Bluetooth -- Right brick (slave)
```

## Topics

`<brick>` is `left` (master) or `right` (slave).

| Topic | Direction | QoS | Payload |
|---|---|---|---|
| `robot/cmd/<brick>` | PC to brick | 1 | `{"id", "cmd", "args"}` |
| `robot/ack/<brick>` | brick to PC | 1 | `{"id", "ok", "result" \| "error"}` |
| `robot/state/<brick>` | brick to PC | 0 | encoders (deg) and sensor values, 10-20 Hz |
| `robot/event/<brick>` | brick to PC | 1 | guard triggers, stalls, link loss |
| `robot/estop` | PC to all | 1 | stop everything now |
| `robot/heartbeat` | PC to all | 0 | every 0.5 s |
| `robot/status/<brick>` | broker (last will) | 1 | `online` / `offline` |

## Commands

`cmd` is one of:

- `ping`: no args. Returns `{"brick": <name>}`.
- `info`: no args. Returns ports, motor and sensor types found on the brick.
- `run_target`: move one motor. Args: `{"port": "outA", "target": 90.0, "speed": 60.0}`.
  `target` is an absolute encoder angle in degrees, `speed` in deg/s (capped
  at 100 deg/s until joint limits are known).
- `run_targets`: start several motors together. Args:
  `{"moves": [{"port": ..., "target": ..., "speed": ...}]}`. Assigned a single
  `id`; the ack is sent after all moves finish.
- `read`: no args. Returns the latest encoder and sensor snapshot.
- `reset`: no args. Resets encoder offsets on that brick.
- `stop`: no args. Stops all motors on that brick immediately.
- `set_guards`: configure sensor guards. Args:
  `{"rules": [{"sensor": "in1", "invert": false}]}`. An empty rule list clears
  the sensor-guard rules (stall/heartbeat/Bluetooth watchdogs stay on).
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
`bluetooth_loss`, `estop`.

## State

Published on `robot/state/<brick>` at 10-20 Hz (QoS 0):

```json
{
  "brick": "left",
  "encoders": {"outA": 12.5, "outB": -3.0},
  "sensors": {"in1": 0, "in2": 1},
  "latched": false
}
```

Encoder values are degrees. Touch sensors report 0/1.

## Safety (binding)

The on-brick safety layer is local and never depends on the network, except
that the master forwards a stop to the other brick as soon as it can:

- Sensor guards run every ~10 ms. Any configured sensor triggering outside a
  homing move stops all motors on that brick, latches, and publishes an
  event. The master forwards the stop to the other brick at once.
- Homing exception: a move carrying `{sensor, motor}` stops that motor when
  the sensor triggers. This ends homing and is not an error.
- Stall guard: a motor commanded but near zero speed for > 300 ms stops and
  reports.
- Heartbeat watchdog: no heartbeat for 2 s stops everything on that brick.
- Bluetooth watchdog: slave losing the master link stops everything on slave.
- Latch: after a guard stop, moves are refused until `clear_latch`.

Speed cap is 100 deg/s during Stage 0 (joint limits unknown).

## Fallback

If MQTT on the brick is unstable by end of Day 1, keep the same message
format (same JSON `id`/`cmd`/`args` and ack shape) over a plain TCP socket
from the master. Only the transport changes.
