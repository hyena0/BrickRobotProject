"""Robot-agnostic homing planner and runner (Stage 5C M3).

Merges three sources:

- human config ``config/homing.local.yaml`` (per joint: method, sensor,
  direction, speed),
- inference results (gear ratios from the jog map, joint ranges from
  the model, trip angles from 5A.1 ``sensor_bounds.json``),
- wiring (``config/ports.local.yaml`` + ``runs/stage5/sensors.json``).

Config schema (all six joints listed explicitly)::

    joints:
      J1: {method: sensor, sensor: in1, direction: -1, speed: 30.0}
      J2: {method: sensor, sensor: in1, direction: -1, speed: 30.0}
      J3: {method: sensor, sensor: in2, direction: -1, speed: 30.0}
      J4: {method: sensor, sensor: in3, direction: -1, speed: 30.0}
      J5: {method: stall, direction: 1, speed: 20.0}
      J6: {method: manual}

methods: ``sensor`` (step toward the sensor in 2 joint-deg steps
up to 30 joint deg, stop on trip), ``stall``
(drive until the stall guard trips; trip angle unknown), ``manual``
(operator positions the joint, then confirms current as 0),
``assisted`` (operator jogs the joint inline until the sensor trips;
works cross-brick because the sensor is polled, never armed).

Conventions: joint angles in joint degrees; ``direction`` is a motor
sign (+1/-1); ``speed`` is motor deg/s (<= 300). ``sensor`` names the
sensor port (``in1``..); its brick is resolved from the sensor
association, so a sensor on the other brick than the motor is
reported and skipped, never driven blind.

Zero offsets: at the trip moment the true joint angle is the 5A.1
geometric boundary (``trip_joint``). Joint readings afterwards are
``q = ratio * m + offset`` with
``offset = trip_joint - ratio * m_trip`` (sensor) or
``offset = -ratio * m_now`` (manual). Stall gives no trip angle, so
its offset stays null. Offsets are recorded in the run report; M3
writes them into the robot package later.

Run::

    source .venv/bin/activate
    python -m src.homing.plan --homing config/homing.local.yaml
"""

from __future__ import annotations

import argparse
import datetime
import json
import pathlib

METHODS = ("sensor", "stall", "manual", "assisted")
HOME_SEARCH_DEG = 360.0
MAX_SPEED_DEG_S = 300.0
MANUAL_JOG_SPEED_DEG_S = 60.0


def _is_empty(value):
    return value is None or (isinstance(value, str)
                             and not value.strip())


def _norm_sensor_port(value):
    """Normalize 1 / "1" / "in1" to "in1"; else None."""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return "in" + str(value) if value >= 1 else None
    if isinstance(value, str):
        text = value.strip().lower()
        if text.startswith("in"):
            text = text[2:]
        if text.isdigit() and int(text) >= 1:
            return "in" + str(int(text))
    return None


def _norm_direction(value):
    if isinstance(value, bool):
        return None
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    if number in (-1, 1):
        return number
    return None


def load_yaml(path):
    import yaml

    with open(str(path), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def sensor_assoc_for(sensors, joint_id, sensor_port):
    """Find the sensor association for a joint + port (inN)."""
    num = sensor_port[2:]
    for entry in sensors or []:
        if entry.get("joint") != joint_id:
            continue
        if str(entry.get("port")) == num:
            return entry
    return None


def trip_joint_for(sensor_bounds, sensor_uid):
    """5A.1 geometric joint angle at trip; None when unverified."""
    for entry in (sensor_bounds or {}).get("bounds", []):
        if entry.get("uid") != sensor_uid:
            continue
        if entry.get("status") != "ok":
            return None
        value = entry.get("boundary_deg")
        if value is None:
            return None
        return float(value)
    return None


def build_plan(homing_cfg, jog_map, sensors, sensor_bounds, ports):
    """Validate and merge inputs into an executable homing plan.

    Returns ``(plan, errors, warnings)``. ``errors`` non-empty means
    do not run anything. Cross-brick joints are marked skipped with
    a warning. Pure function: no I/O, no motion.
    """
    errors = []
    warnings = []
    plan = {"joints": []}

    if not isinstance(homing_cfg, dict) or not isinstance(
            homing_cfg.get("joints"), dict):
        return plan, ["homing config needs a 'joints' mapping"], warnings
    cfg_joints = homing_cfg["joints"]

    motor_ports = set()
    sensor_ports = set()
    try:
        for item in (ports or {}).get("motors", []):
            port = str(item.get("port", ""))
            if port and not port.startswith("out"):
                port = "out" + port
            motor_ports.add((item.get("brick"), port))
        for item in (ports or {}).get("sensors", []):
            sensor_ports.add((item.get("brick"),
                              "in" + str(item.get("port"))))
    except (AttributeError, TypeError):
        errors.append("ports file has no motors/sensors lists")
        return plan, errors, warnings

    for joint_id in jog_map:
        if joint_id not in cfg_joints:
            errors.append(joint_id + ": missing from homing config")
            continue
        entry = cfg_joints[joint_id] or {}
        if not isinstance(entry, dict):
            errors.append(joint_id + ": entry must be a mapping")
            continue
        method = entry.get("method")
        if _is_empty(method):
            errors.append(joint_id + ": empty method")
            continue
        if method not in METHODS:
            errors.append(joint_id + ": unknown method " + str(method))
            continue
        wire = jog_map[joint_id]
        motor = (wire["brick"], wire["port"])
        if motor not in motor_ports:
            errors.append(joint_id + ": motor port "
                          + str(wire["brick"]) + "/"
                          + str(wire["port"]) + " not in ports file")
            continue
        item = {"joint": joint_id, "method": method,
                "brick": wire["brick"], "port": wire["port"],
                "ratio": wire["ratio"], "status": "ready"}
        if method == "manual":
            plan["joints"].append(item)
            continue
        direction = _norm_direction(entry.get("direction"))
        if direction is None:
            errors.append(joint_id + ": direction must be -1 or +1")
            continue
        speed = entry.get("speed")
        if _is_empty(speed):
            errors.append(joint_id + ": empty speed")
            continue
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            errors.append(joint_id + ": speed must be a number")
            continue
        if speed <= 0 or speed > MAX_SPEED_DEG_S:
            errors.append(joint_id + ": speed must be in (0, 300]")
            continue
        item["direction"] = direction
        item["speed"] = speed
        if method == "stall":
            item["trip_joint"] = None
            plan["joints"].append(item)
            continue
        sensor_port = _norm_sensor_port(entry.get("sensor"))
        if sensor_port is None:
            errors.append(joint_id + ": sensor must be in1.. (got "
                          + str(entry.get("sensor")) + ")")
            continue
        assoc = sensor_assoc_for(sensors, joint_id, sensor_port)
        if assoc is None:
            errors.append(joint_id + ": no sensor " + sensor_port
                          + " associated (use stall or manual)")
            continue
        sensor = (assoc.get("brick"), sensor_port)
        if sensor not in sensor_ports:
            errors.append(joint_id + ": sensor port "
                          + str(sensor[0]) + "/" + sensor_port
                          + " not in ports file")
            continue
        if sensor[0] != wire["brick"] and method != "assisted":
            item["status"] = "skipped"
            item["sensor"] = sensor_port
            item["sensor_brick"] = sensor[0]
            item["reason"] = ("sensor on " + str(sensor[0])
                              + ", motor on " + str(wire["brick"]))
            warnings.append(joint_id + ": cross-brick sensor/motor, "
                            "skipped (" + item["reason"] + ")")
            plan["joints"].append(item)
            continue
        item["sensor_brick"] = sensor[0]
        trip = trip_joint_for(sensor_bounds, assoc.get("uid"))
        if trip is None:
            warnings.append(joint_id + ": 5A.1 trip angle unverified, "
                            "offset stays null")
        item["sensor"] = sensor_port
        item["sensor_uid"] = assoc.get("uid")
        item["trip_joint"] = trip
        plan["joints"].append(item)

    for joint_id in cfg_joints:
        if joint_id not in jog_map:
            errors.append(str(joint_id) + ": unknown joint")
    return plan, errors, warnings


def _read_motor(transport, brick, port):
    state = transport.send(brick, "read")
    return float(state["encoders"].get(port, 0.0))


BACKOFF_JOINT_STEP_DEG = 1.0
BACKOFF_TRIES = 5
BACKOFF_SPEED_DEG_S = 30.0
SENSOR_STEP_JOINT_DEG = 2.0
SENSOR_MAX_TRAVEL_JOINT_DEG = 30.0


def _back_off(transport, brick, port, sensor, direction, ratio):
    """Clear an already-pressed homing sensor.

    Drives opposite the search direction in joint-degree steps
    (motor delta = step / |ratio|) until the sensor reads clear. A
    rule armed onto a pressed sensor latches on the next idle poll
    and the run is refused before moving, so homing must never
    start pressed. Motor steps would barely move high-ratio joints
    (J2: 50 motor deg is only ~1.2 joint deg), hence joint units.
    Returns True when clear, None when the sensor never clears.
    """
    step = BACKOFF_JOINT_STEP_DEG / abs(ratio)
    for _ in range(BACKOFF_TRIES):
        pos = _read_motor(transport, brick, port)
        transport.send(brick, "run_target",
                       {"port": port,
                        "target": pos - direction * step,
                        "speed": BACKOFF_SPEED_DEG_S})
        state = transport.send(brick, "read")
        if int(state["sensors"].get(sensor, 0)) == 0:
            return True
    return None


def _clear_all(transport):
    """Disarm sensor rules and clear latches on both bricks.

    After a homing trip the sensor stays pressed while its rule is
    still armed, so the idle guard would latch (and cross-latch the
    other brick) before the next joint starts.
    """
    bricks = getattr(transport, "bricks", ("left", "right"))
    for brick in bricks:
        try:
            transport.send(brick, "set_guards", {"rules": []})
        except Exception:
            pass
        try:
            transport.send(brick, "clear_latch")
        except Exception:
            pass


def make_jog_move(transport):
    """Build the manual-jog function for a transport.

    Clears any stale latch first: a previous stall would otherwise
    refuse every later jog with its old reason. A genuine stall
    during the jog still latches and is reported.
    """
    def jog_move(item, delta_joint):
        try:
            transport.send(item["brick"], "clear_latch")
        except Exception:
            pass
        pos = _read_motor(transport, item["brick"], item["port"])
        target = pos + delta_joint / item["ratio"]
        transport.send(item["brick"], "run_target",
                       {"port": item["port"], "target": target,
                        "speed": MANUAL_JOG_SPEED_DEG_S})
        return _read_motor(transport, item["brick"], item["port"])
    return jog_move


ASSIST_AUTO_STEP_JOINT_DEG = 2.0
ASSIST_MAX_STEPS = 15


def interactive_mark(item, jog_move, read_pos, tripped=None,
                     accept=None):
    """Manual-joint prompt loop. Returns motor pos to mark, or None.

    ``jog_move(delta_joint_deg)`` jogs and returns the new motor
    position; ``read_pos()`` reads the current motor position.
    ``tripped()`` (optional) auto-marks when the sensor trips;
    ``accept()`` (optional) must return ``(ok, message)`` for Enter
    to mark.
    """
    while True:
        if tripped is not None and tripped():
            print("  sensor tripped")
            return read_pos()
        answer = input(item["joint"] + ": joint deg to jog (+/-), "
                       "Enter to mark, q to skip: ")
        text = answer.strip().lower()
        if text == "" or text == "mark":
            if accept is not None:
                ok, msg = accept()
                if not ok:
                    print("  " + msg)
                    continue
            return read_pos()
        if text == "q" or text == "skip":
            return None
        try:
            delta = float(text)
        except ValueError:
            print("  enter a number, empty line, or q")
            continue
        try:
            pos = jog_move(delta)
        except Exception as exc:
            print("  jog failed: " + str(exc))
            if "guard stop" in str(exc) or "stall" in str(exc):
                print("  motor is stuck: jog back with the "
                      "opposite sign")
            continue
        print("  motor at " + str(round(pos, 2)))


def assisted_auto(item, transport, tripped):
    """Headless assisted homing (for --auto): step toward the sensor.

    Drives the motor search direction in joint-degree steps and
    polls the sensor after each step. Returns the trip motor
    position, or None after ASSIST_MAX_STEPS without a trip.
    """
    step = ASSIST_AUTO_STEP_JOINT_DEG / abs(item["ratio"])
    for _ in range(ASSIST_MAX_STEPS):
        if tripped():
            return _read_motor(transport, item["brick"], item["port"])
        pos = _read_motor(transport, item["brick"], item["port"])
        transport.send(item["brick"], "run_target",
                       {"port": item["port"],
                        "target": pos + item["direction"] * step,
                        "speed": item["speed"]})
    if tripped():
        return _read_motor(transport, item["brick"], item["port"])
    return None


def execute_plan(plan, transport, confirm=None, jog_move=None):
    """Run a validated plan on a transport (fake or real).

    Manual joints: with ``jog_move(delta_joint_deg)`` (returns the
    new motor position) the operator jogs inline at the prompt;
    otherwise ``confirm(item)`` must return True to record the
    current position as zero. Returns the run record (plan +
    per-joint results + offsets + warnings). Never raises on a
    joint failure: the failure is recorded and the run continues
    with the next joint.
    """
    if confirm is None:
        def confirm(item):
            return True
    results = []
    for item in plan["joints"]:
        _clear_all(transport)
        if item.get("status") == "skipped":
            results.append({"joint": item["joint"],
                            "status": "skipped",
                            "reason": item.get("reason", ""),
                            "offset": None})
            continue
        joint = item["joint"]
        brick, port = item["brick"], item["port"]
        ratio = item["ratio"]
        try:
            if item["method"] == "manual":
                if jog_move is not None:
                    pos = interactive_mark(
                        item,
                        lambda delta: jog_move(item, delta),
                        lambda: _read_motor(transport, brick, port))
                    if pos is None:
                        results.append({"joint": joint,
                                        "status": "declined",
                                        "offset": None})
                        continue
                else:
                    if not confirm(item):
                        results.append({"joint": joint,
                                        "status": "declined",
                                        "offset": None})
                        continue
                    pos = _read_motor(transport, brick, port)
                results.append({"joint": joint, "status": "homed",
                                "method": "manual",
                                "motor_position": pos,
                                "offset": -ratio * pos})
            elif item["method"] == "assisted":
                sensor_brick = item["sensor_brick"]
                sensor_port = item["sensor"]

                def tripped():
                    state = transport.send(sensor_brick, "read")
                    return int(state["sensors"].get(sensor_port,
                                                   0)) != 0

                def accept():
                    if tripped():
                        return True, ""
                    return (False, "sensor not pressed yet - jog "
                            "onto it, then Enter")

                if jog_move is not None:
                    pos = interactive_mark(
                        item,
                        lambda delta: jog_move(item, delta),
                        lambda: _read_motor(transport, brick, port),
                        tripped=tripped, accept=accept)
                    if pos is None:
                        results.append({"joint": joint,
                                        "status": "declined",
                                        "offset": None})
                        continue
                else:
                    pos = assisted_auto(item, transport, tripped)
                    if pos is None:
                        results.append({"joint": joint,
                                        "status": "failed",
                                        "method": "assisted",
                                        "error": ("no trip after "
                                                  + str(ASSIST_MAX_STEPS
                                                        * ASSIST_AUTO_STEP_JOINT_DEG)
                                                  + " joint deg; check "
                                                  "search direction"),
                                        "offset": None})
                        continue
                trip = item.get("trip_joint")
                offset = (trip - ratio * pos
                          if trip is not None else None)
                res = {"joint": joint, "status": "homed",
                       "method": "assisted",
                       "sensor": sensor_port,
                       "motor_position": pos,
                       "trip_joint": trip,
                       "offset": offset}
                if trip is None:
                    res["warning"] = ("trip angle unknown; "
                                      "offset stays null")
                results.append(res)
            elif item["method"] == "stall":
                transport.send(brick, "set_guards", {"rules": []})
                try:
                    transport.send(brick, "run_target",
                                   {"port": port,
                                    "target": (item["direction"]
                                               * HOME_SEARCH_DEG),
                                    "speed": item["speed"]})
                except Exception as exc:
                    if "guard stop" not in str(exc):
                        raise
                    try:
                        transport.send(brick, "clear_latch")
                    except Exception:
                        pass
                    pos = _read_motor(transport, brick, port)
                    results.append({"joint": joint, "status": "homed",
                                    "method": "stall",
                                    "motor_position": pos,
                                    "offset": None,
                                    "latch_reason": str(exc),
                                    "warning": "trip angle unknown; "
                                    "offset stays null"})
                    continue
                pos = _read_motor(transport, brick, port)
                results.append({"joint": joint, "status": "no_stall",
                                "method": "stall",
                                "motor_position": pos,
                                "offset": None,
                                "warning": "no stall encountered; "
                                "offset stays null"})
            else:
                state = transport.send(brick, "read")
                backed_off = False
                if int(state["sensors"].get(item["sensor"], 0)) != 0:
                    try:
                        backed_off = _back_off(
                            transport, brick, port, item["sensor"],
                            item["direction"], ratio)
                    except Exception as exc:
                        results.append({"joint": joint,
                                        "status": "failed",
                                        "method": "sensor",
                                        "error": ("backoff failed: "
                                                  + str(exc)),
                                        "offset": None})
                        continue
                    if not backed_off:
                        results.append({"joint": joint,
                                        "status": "failed",
                                        "method": "sensor",
                                        "error": ("sensor still pressed "
                                                  "after backoff "
                                                  "(moved "
                                                  + str(BACKOFF_TRIES
                                                        * BACKOFF_JOINT_STEP_DEG)
                                                  + " joint deg opposite "
                                                  "search); check search "
                                                  "direction sign or "
                                                  "sensor wiring"),
                                        "offset": None})
                        continue
                transport.send(brick, "set_guards",
                               {"rules": [{"sensor": item["sensor"]}]})
                # Bounded stepping, never one blind long run: each
                # step moves SENSOR_STEP_JOINT_DEG toward the search
                # direction with the trip guard armed, so a wrong
                # direction or an obstruction ends the run instead
                # of driving through it.
                step_motor = (SENSOR_STEP_JOINT_DEG / abs(ratio))
                max_steps = int(SENSOR_MAX_TRAVEL_JOINT_DEG
                                / SENSOR_STEP_JOINT_DEG)
                pos = _read_motor(transport, brick, port)
                homed = False
                steps_taken = 0
                step_error = None
                for _ in range(max_steps):
                    try:
                        ack = transport.send(
                            brick, "run_target",
                            {"port": port,
                             "target": (pos + item["direction"]
                                        * step_motor),
                             "speed": item["speed"],
                             "until_sensor": item["sensor"]})
                    except Exception as exc:
                        step_error = str(exc)
                        break
                    steps_taken += 1
                    pos = _read_motor(transport, brick, port)
                    if ack.get("homed"):
                        homed = True
                        break
                if not homed:
                    results.append({"joint": joint, "status": "failed",
                                    "method": "sensor",
                                    "error": (step_error or "no trip "
                                              "after "
                                              + str(SENSOR_MAX_TRAVEL_JOINT_DEG)
                                              + " joint deg; check "
                                              "search direction"),
                                    "motor_position": pos,
                                    "offset": None})
                else:
                    trip = item.get("trip_joint")
                    offset = (trip - ratio * pos
                              if trip is not None else None)
                    results.append({"joint": joint, "status": "homed",
                                    "method": "sensor",
                                    "sensor": item["sensor"],
                                    "direction": item["direction"],
                                    "motor_position": pos,
                                    "trip_joint": trip,
                                    "offset": offset,
                                    "backed_off": bool(backed_off),
                                    "steps": steps_taken})
        except Exception as exc:  # keep going; record and continue
            msg = str(exc)
            if item["method"] in ("sensor", "stall") and (
                    "guard stop" in msg or "latched" in msg
                    or "limit_reject" in msg or "timeout" in msg):
                status = "failed"
            else:
                status = "error"
            results.append({"joint": joint, "status": status,
                            "error": msg, "offset": None})
    return {"results": results,
            "offsets": {r["joint"]: r.get("offset") for r in results}}


def arm_fake_sensors(left, right):
    """Make fake sensors trip when their homing run starts.

    Fake motors finish in a poll or two, so tripping at run start
    stands in for a press after travel. Only for fake runs.
    """
    for core in (left, right):
        raw = core.handle

        def hooked(msg, pump=None, _raw=raw, _core=core):
            args = msg.get("args", {})
            if (msg.get("cmd") == "run_target"
                    and args.get("until_sensor")):
                _core.sensors[args["until_sensor"]].set(1)
            return _raw(msg, pump)

        core.handle = hooked


def save_run(record, plan, warnings, out_base, mode="fake"):
    """Write homing.json + homing.md under runs/stage5c/<date>/.

    Results merge into the day's file by joint (latest run wins),
    so per-joint --only runs accumulate instead of overwriting
    each other. Warnings merge as a union.
    """
    day = datetime.date.today().isoformat()
    out_dir = pathlib.Path(out_base) / day
    out_dir.mkdir(parents=True, exist_ok=True)
    merged = {}
    merged_offsets = {}
    merged_warnings = list(warnings)
    path = out_dir / "homing.json"
    if path.exists():
        try:
            old = json.loads(path.read_text(encoding="utf-8"))
        except ValueError:
            old = {}
        for res in old.get("results", []):
            merged[res["joint"]] = res
        merged_offsets.update(old.get("offsets", {}) or {})
        for warn in old.get("warnings", []):
            if warn not in merged_warnings:
                merged_warnings.append(warn)
    for res in record["results"]:
        merged[res["joint"]] = res
    merged_offsets.update(record.get("offsets", {}))
    record = {"date": day, "mode": mode, "plan": plan["joints"],
              "warnings": merged_warnings,
              "results": list(merged.values()),
              "offsets": merged_offsets,
              **{key: value for key, value in record.items()
                 if key not in ("results", "offsets")}}
    (out_dir / "homing.json").write_text(json.dumps(record, indent=1),
                                         encoding="utf-8")
    lines = ["# Homing " + day + " (" + mode + ")", ""]
    for warn in warnings:
        lines.append("- WARN: " + warn)
    for res in record["results"]:
        line = ("- " + res["joint"] + ": " + res["status"]
                + " (" + str(res.get("method", "-")) + ")")
        if res.get("offset") is not None:
            line += " offset=" + str(round(res["offset"], 3))
        if res.get("motor_position") is not None:
            line += " motor=" + str(round(res["motor_position"], 2))
        if res.get("warning"):
            line += " [" + res["warning"] + "]"
        if res.get("error"):
            line += " ERROR " + str(res["error"])
        lines.append(line)
    lines += ["", "Offsets (q = ratio*m + offset):"]
    for joint, offset in record["offsets"].items():
        lines.append("- " + joint + ": " + str(offset))
    (out_dir / "homing.md").write_text("\n".join(lines) + "\n",
                                       encoding="utf-8")
    return out_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description="Homing plan + runner")
    parser.add_argument("--homing", default="config/homing.local.yaml")
    parser.add_argument("--model",
                        default="models/robot_inferred.json")
    parser.add_argument("--sensors", default="runs/stage5/sensors.json")
    parser.add_argument("--bounds",
                        default="runs/stage5a/sensor_bounds.json")
    parser.add_argument("--ports", default="config/ports.local.yaml")
    parser.add_argument("--config", default=None,
                        help="real-brick yaml (default: fake bricks)")
    parser.add_argument("--auto", action="store_true",
                        help="accept manual marks without prompting")
    parser.add_argument("--only", default=None,
                        help="comma-separated joints to run "
                        "(default: all, e.g. --only J1)")
    parser.add_argument("--out", default="runs/stage5c")
    args = parser.parse_args(argv)

    from src.console.mini import load_jog_map

    homing_cfg = load_yaml(args.homing)
    sensors = json.loads(pathlib.Path(args.sensors).read_text(
        encoding="utf-8"))
    bounds = json.loads(pathlib.Path(args.bounds).read_text(
        encoding="utf-8"))
    ports = load_yaml(args.ports)
    jog_map = load_jog_map(model_path=args.model)

    plan, errors, warnings = build_plan(homing_cfg, jog_map, sensors,
                                        bounds, ports)
    for err in errors:
        print("ERROR: " + err)
    for warn in warnings:
        print("WARN: " + warn)
    if errors:
        raise SystemExit(1)

    only = None
    if args.only:
        only = [name.strip() for name in args.only.split(",")
                if name.strip()]
        unknown = [name for name in only if name not in jog_map]
        if unknown:
            print("ERROR: unknown joint(s): " + ", ".join(unknown))
            raise SystemExit(1)
        plan = {"joints": [item for item in plan["joints"]
                           if item["joint"] in only]}

    if args.config:
        from src.adapters.pc.mqtt_transport import MqttTransport

        transport = MqttTransport.from_config(args.config)
        mode = "real"
        print("homing on real bricks (" + args.config + ")")
    else:
        from src.ev3.fake import build_mp_transport

        transport, left, right = build_mp_transport(full=True)
        arm_fake_sensors(left, right)
        mode = "fake"
        print("homing on fake bricks (sensors auto-trip: simulated)")

    try:
        if args.auto:
            record = execute_plan(plan, transport)
        else:
            record = execute_plan(plan, transport,
                                  jog_move=make_jog_move(transport))
    finally:
        transport.close()
    if only:
        record["only"] = only
    out_dir = save_run(record, plan, warnings, args.out, mode)
    print("wrote " + str(out_dir / "homing.json"))
    for res in record["results"]:
        print("  " + res["joint"] + ": " + res["status"])


if __name__ == "__main__":
    main()
