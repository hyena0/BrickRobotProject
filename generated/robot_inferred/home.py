from __future__ import annotations

"""Generated EV3 control package (Stage 5.1).

Produced by ``python -m src.codegen.generate`` from the
inferred kinematic model. Abstract kinematics + code only:
no part IDs, poses or appearance data, so this package is
public. Bricks see ports only; joint names live on the PC
side.

DO NOT hand-edit: regenerate after any model or calibration
change and re-run the bring-up checks.

Homing routine (Stage 5.1).

One entry per joint in ``HOME_PLAN`` (base to tip). Three
methods: ``local`` (brick homing guard stops the motor on
trigger), ``remote`` (PC polls the sensor on the other brick),
``stall`` (PC steps until the encoder stops changing, then
clears the stall latch). Returns home encoders for ``motion``
plus a per-joint report. Unconfirmed directions home toward
negative at low speed; confirm them in bring-up step 2."""


from .robot_config import HOME_PLAN

HOME_SPEED_DEG_S = 20.0
HOME_STEP_DEG = 3.0
HOME_MAX_DEG = 120.0
STALL_EPS_DEG = 1.0


def _key(brick, port):
    return f"{brick}:{port}"


def _read_encoders(driver, brick):
    snap = driver.request(brick, "read")
    return {str(k): float(v)
            for k, v in (snap.get("encoders") or {}).items()}


def _read_sensors(driver, brick):
    snap = driver.request(brick, "read")
    return {str(k): v for k, v in (snap.get("sensors") or {}).items()}


def home_joint_local(driver, entry):
    """Home via the brick homing guard (same brick for motor+sensor)."""
    direction = float(entry["direction"]) * HOME_MAX_DEG
    res = driver.home_motor(entry["motor_brick"], entry["motor_port"],
                            entry["sensor"]["port"], direction,
                            HOME_SPEED_DEG_S)
    enc = _read_encoders(driver, entry["motor_brick"])
    return enc.get(entry["motor_port"], 0.0), res


def home_joint_remote(driver, entry):
    """Home by PC polling when the sensor lives on the other brick."""
    mbrick, mport = entry["motor_brick"], entry["motor_port"]
    sbrick, sport = (entry["sensor"]["brick"], entry["sensor"]["port"])
    step = float(entry["direction"]) * HOME_STEP_DEG
    enc = _read_encoders(driver, mbrick)
    pos = enc.get(mport, 0.0)
    travelled = 0.0
    while travelled < HOME_MAX_DEG:
        pos += step
        travelled += abs(step)
        driver.request(mbrick, "run_target",
                       {"port": mport, "target": pos,
                        "speed": HOME_SPEED_DEG_S})
        if int(_read_sensors(driver, sbrick).get(sport, 0)) == 1:
            break
    enc = _read_encoders(driver, mbrick)
    return enc.get(mport, pos), {"method": "remote",
                                 "travelled_deg": travelled}


def home_joint_stall(driver, entry):
    """Home against the mechanical end-stop (no sensor on the joint)."""
    mbrick, mport = entry["motor_brick"], entry["motor_port"]
    step = float(entry["direction"]) * HOME_STEP_DEG
    enc = _read_encoders(driver, mbrick)
    pos, prev = enc.get(mport, 0.0), None
    travelled = 0.0
    while travelled < HOME_MAX_DEG:
        prev = _read_encoders(driver, mbrick).get(mport, pos)
        pos += step
        travelled += abs(step)
        try:
            driver.request(mbrick, "run_target",
                           {"port": mport, "target": pos,
                            "speed": HOME_SPEED_DEG_S})
        except Exception:
            break  # latched stall guard refuses further moves: homed
        now = _read_encoders(driver, mbrick).get(mport, pos)
        if abs(now - prev) < STALL_EPS_DEG:
            pos = now
            break
    try:
        driver.clear_latch(mbrick)
    except Exception:
        pass
    enc = _read_encoders(driver, mbrick)
    return enc.get(mport, pos), {"method": "stall",
                                 "travelled_deg": travelled}


def home_all(driver, joints=None):
    """Home every joint in chain order. Returns the home report."""
    encoders, report = {}, []
    homed_motors = set()
    for entry in HOME_PLAN:
        if joints is not None and entry["joint"] not in joints:
            continue
        motor_key = _key(entry["motor_brick"], entry["motor_port"])
        if motor_key in homed_motors:
            # Coupled joints share one motor (e.g. the wrist): homing the
            # motor once homes every joint it drives.
            pos = encoders[motor_key]
            res = {"method": "shared",
                   "shared_with": [r["joint"] for r in report
                                   if _key(r.get("motor_brick", ""),
                                           r.get("motor_port", ""))
                                   == motor_key],
                   "note": "motor already homed; shared encoder reused"}
            report.append({"joint": entry["joint"], "method": "shared",
                           "home_encoder": pos, "detail": res})
            print(f"[{entry['joint']}] shares homed motor {motor_key} "
                  f"at {pos:.2f} deg")
            continue
        if not entry["direction_confirmed"]:
            print(f"[{entry['joint']}] homing direction "
                  f"{entry['direction']:+d} UNCONFIRMED "
                  f"(bring-up step 2 must confirm it)")
        if entry["method"] == "local":
            pos, res = home_joint_local(driver, entry)
        elif entry["method"] == "remote":
            pos, res = home_joint_remote(driver, entry)
        else:
            pos, res = home_joint_stall(driver, entry)
        encoders[motor_key] = pos
        homed_motors.add(motor_key)
        report.append({"joint": entry["joint"], "method": entry["method"],
                       "motor_brick": entry["motor_brick"],
                       "motor_port": entry["motor_port"],
                       "home_encoder": pos, "detail": res})
        print(f"[{entry['joint']}] homed via {entry['method']} "
              f"at {pos:.2f} deg")
    return {"encoders": encoders, "joints": report}
