# Pybricks brick programs (`brick_mp`)

Stage 5B brick platform: Pybricks MicroPython on both EV3 bricks.
`src/ev3/brick/` stays as the legacy ev3dev implementation (PC-side
fakes still import it); the bricks run this package. MQTT topics and
payloads are unchanged (`docs/protocol.md`).

## Files

- `left_main.py` — master: umqtt client to the hub plus mailbox server.
- `right_main.py` — slave: mailbox client, no MQTT.
- `core.py`, `guards.py`, `motors.py`, `sensors.py` — shared logic and HAL.
- `link.py` — mailbox boxes plus `FakeLink` for PC-side tests.
- `clock.py`, `config.py` — wrap-safe timing, JSON config loader.
- `brick_left.json`, `brick_right.json` — per-side wiring. No
  editing: copy the whole folder to both bricks, run `left_main.py`
  on the left and `right_main.py` on the right.

On the brick the modules live flat in one directory, so sibling imports
use plain top-level names (`import clock`, not package paths).

## Mailboxes (left brick is the server)

| Box | Direction | Content |
|---|---|---|
| `brick-cmd` | master to slave | command envelope, stop-and-wait |
| `brick-reply` | slave to master | `{ack, events}` per command |
| `brick-state` | slave to master | latest state at ~15 Hz |
| `brick-evt` | slave to master | async event with sequence number |
| `brick-hb` | master to slave | heartbeat plus stop flag |

A move in progress keeps pumping the same service set (MQTT check on
the left, heartbeat box on the right), so stop and heartbeat traffic
stays live with one loop and no extra threads.

## Deploy (copy only, nothing to install on the brick)

On the PC with internet, fetch the official MQTT file once:

```bash
mkdir -p umqtt_deploy/umqtt
curl -sL https://raw.githubusercontent.com/micropython/micropython-lib/master/micropython/umqtt.simple/umqtt/simple.py \
  -o umqtt_deploy/umqtt/simple.py
touch umqtt_deploy/umqtt/__init__.py
```

Copy these files flat to the same folder on each brick via VS Code
(all from `src/ev3/brick_mp/`):

- LEFT brick: `clock.py`, `config.py`, `core.py`, `display.py`,
  `guards.py`, `link.py`, `log.py`, `motors.py`, `sensors.py`,
  `version.py`, **`left_main.py`** (run this),
  `brick_left.json`, `brick_right.json` (both copied, no editing),
  plus the `umqtt/` folder (`simple.py`, `__init__.py`).
- RIGHT brick: same files, run **`right_main.py`** (it reads
  `brick_right.json`; nothing to edit).

Both mains print `brick sw version: 5B.14` as their first line, and
every copied file carries the same `# brick-sw-version:` marker, so a
stale copy is visible immediately. After any brick-side change: bump
the version in `version.py` plus all markers, then re-copy all files.

Startup diagnostics (version, found ports, `NOT CONNECTED` list) go to
the console, to `brick.log` on the brick (delete to reset), and the
missing ports also ride along in every state message, so the PC side
(`check`, mini console) shows them too.

The EV3 screen mirrors live status at 1 Hz (version, OK/LATCH, found
motors/sensors, hub/slave link, hub address, latch reason, missing
ports), plus `wait slave...` / `wait hub...` / `wait master...` while
starting. Screen code never raises: a display failure cannot stop the
robot loop.

Connection targets are hardcoded, no input needed on the bricks:
left connects to PC `192.168.35.151:1883` (`HUB_ADDRESS` in
`left_main.py`), right connects to Bluetooth name `ev3dev`
(`MASTER_BT_NAME` in `right_main.py`).

No editing, no accounts or passwords anywhere: the broker takes
anonymous clients. Each main reads its own side file
(`brick_left.json` / `brick_right.json`) and refuses a file naming
the other side, so a wrong copy fails loudly instead of joining the
wrong link. Legacy `config/brick.local.json` / flat
`brick.local.json` are still tried as fallback. When no file exists
at all, each main uses its built-in wiring (`DEFAULT_CONFIG`) and
says so on the console.

Run order:

1. Pair the bricks in the brick Bluetooth menu first (pair only, do
   NOT tap connect). Connect the left brick to Wi-Fi.
2. Run `left_main.py` on the LEFT brick first. It prints the found
   ports, and any port with nothing connected is reported as
   `NOT CONNECTED` (check cables) and skipped instead of crashing.
   Then it waits for the slave
   (`left: waiting for slave mailbox connection`).
3. Run `right_main.py` on the RIGHT brick.

Then on the hub (PC):

```bash
mosquitto -c config/mosquitto.local.conf
source .venv/bin/activate
python -m src.ev3.check --config config/hardware.local.yaml
```

## MicroPython constraints

Everything under `brick_mp/*.py` must parse and run on MicroPython:
no f-strings, no type hints, no `typing`, no `dataclasses`, no
`threading`, no `yaml`. Hardware imports (`pybricks`, `umqtt`) stay
inside functions so PC-side tests can import every module.
`tests/test_brick_mp.py` enforces all of this statically.
