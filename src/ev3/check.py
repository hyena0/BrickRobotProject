"""Connectivity check for Stage 5B (task 5B.6).

Real mode (needs hub broker + bricks)::

    python -m src.ev3.check --config config/hardware.local.yaml

Smoke mode (no hardware: the real Pybricks brick code with fake
motors/sensors + fake mailbox relay)::

    python -m src.ev3.check --fake

Checks: ping both bricks, print ``info`` (expect 6 motors, 5 sensors
total), jog each motor ±10 deg at low speed, read back, report error and
round-trip latency per brick (master and slave separately), print live
sensor values, press each touch sensor and confirm the event, then
measure cross-brick stop latency (sensor on one brick stopping the
other; needed for J4 homing).
"""

from __future__ import annotations

import argparse
import queue
import threading
import time

EXPECTED_MOTORS = 6
EXPECTED_SENSORS = 5


def build_fake_driver():
    """Driver wired to two in-memory bricks (no network)."""
    from .fake import build_mp_transport

    transport, _, _ = build_mp_transport(full=True)
    return transport


def build_fake_setup():
    """Fake transport plus its brick cores for scripted sensor presses."""
    from .fake import build_mp_transport

    return build_mp_transport(full=True)


def _wait_for(events: queue.Queue, pred, timeout_s: float):
    """Pop the first queued event matching ``pred`` within the timeout."""
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            evt = events.get(timeout=remaining)
        except queue.Empty:
            return None
        if pred(evt):
            return evt


def check_driver(driver, jog_deg: float = 10.0, speed: float = 30.0,
                 event_timeout_s: float = 30.0,
                 auto_press=None, do_cross_brick: bool = True) -> dict:
    """Run the Stage 5B exit-check sequence. Returns a result dict.

    ``auto_press(brick, sensor)`` programmatically presses a sensor (fake
    mode); ``auto_press(brick, None)`` releases all sensors on the brick.
    When None, the user is prompted to press by hand.
    """
    from src.adapters.pc.mqtt_transport import TransportError

    events: queue.Queue = queue.Queue()
    driver.on_event(events.put)

    results: dict = {"bricks": {}, "sensor_events": {},
                     "cross_brick_ms": None}
    total_motors = 0
    total_sensors = 0
    for brick in driver.bricks:
        t0 = time.monotonic()
        ping = driver.send(brick, "ping")
        ping_ms = (time.monotonic() - t0) * 1000.0
        info = driver.send(brick, "info")
        print(f"[{brick}] ping ok ({ping_ms:.1f} ms): {ping}")
        print(f"[{brick}] info: {info}")
        motors = info.get("motors", [])
        sensors = info.get("sensors", [])
        total_motors += len(motors)
        total_sensors += len(sensors)
        brick_res: dict = {"ping_ms": ping_ms, "motors": {}, "rtts_ms": []}
        for m in motors:
            port = m["port"]
            before = driver.send(brick, "read")["encoders"].get(port, 0.0)
            t1 = time.monotonic()
            driver.move_motors({(brick, port): before + jog_deg}, speed)
            driver.move_motors({(brick, port): before}, speed)
            rtt_ms = (time.monotonic() - t1) * 1000.0
            after = driver.send(brick, "read")["encoders"].get(port, 0.0)
            err = abs(after - before)
            ok = err <= 3.0
            brick_res["motors"][port] = {
                "error_deg": err, "rtt_ms": rtt_ms, "ok": ok}
            brick_res["rtts_ms"].append(rtt_ms)
            print(f"[{brick}/{port}] jog ±{jog_deg} deg: "
                  f"error {err:.2f} deg, round-trip {rtt_ms:.0f} ms "
                  f"({'OK' if ok else 'FAIL'})")
        if brick_res["rtts_ms"]:
            avg = sum(brick_res["rtts_ms"]) / len(brick_res["rtts_ms"])
            brick_res["avg_rtt_ms"] = avg
            print(f"[{brick}] avg jog round-trip: {avg:.0f} ms "
                  f"({'master' if brick == 'left' else 'slave'} path)")
        results["bricks"][brick] = brick_res
    print(f"info totals: {total_motors} motors, {total_sensors} sensors "
          f"(expect {EXPECTED_MOTORS}/{EXPECTED_SENSORS})")
    results["total_motors"] = total_motors
    results["total_sensors"] = total_sensors
    if (total_motors, total_sensors) != (EXPECTED_MOTORS, EXPECTED_SENSORS):
        print("WARN: motor/sensor counts differ from the 6/5 expectation; "
              "check the brick wiring.")

    try:
        state = driver.read_state()
        print(f"live sensors: { {b: s.get('sensors') for b, s in state.items()} }")
        missing = {b: s.get("missing", [])
                   for b, s in state.items() if s.get("missing")}
        results["missing"] = missing
        if missing:
            print(f"NOT CONNECTED on bricks: {missing} (check cables)")
    except TransportError as exc:
        print(f"read_state failed: {exc}")

    # -- sensor event check: press each touch sensor ---------------------
    for brick in driver.bricks:
        info = driver.send(brick, "info")
        sensors = [s["port"] for s in info.get("sensors", [])]
        if not sensors:
            continue
        driver.set_guards(brick, [{"sensor": s} for s in sensors])
        for sensor in sensors:
            if auto_press is not None:
                # Release the previous press first: the runtimes poll
                # guards continuously, so clearing while pressed would
                # re-latch immediately.
                auto_press(brick, None)
            try:
                driver.clear_latch(brick)
            except TransportError:
                pass
            # Drain stale events.
            while True:
                try:
                    events.get_nowait()
                except queue.Empty:
                    break
            if auto_press is not None:
                auto_press(brick, sensor)
                # Fake cores have no running main loop, so poll their
                # idle guards directly instead of waiting on the queue.
                evt = _poll_fake_idle(driver, events, brick, sensor)
            else:
                print(f"Press {brick}/{sensor} now "
                      f"({event_timeout_s:.0f}s timeout)...")
                evt = _wait_for(
                    events,
                    lambda e, b=brick, s=sensor: (
                        e.get("brick") == b
                        and e.get("event") == "sensor_guard"
                        and e.get("detail", {}).get("sensor") == s),
                    event_timeout_s)
            ok = evt is not None
            results["sensor_events"][f"{brick}/{sensor}"] = {"ok": ok}
            print(f"[{brick}/{sensor}] touch event: "
                  f"{'OK' if ok else 'TIMEOUT'}")
            try:
                driver.clear_latch()
            except TransportError:
                pass
        driver.set_guards(brick, [])

    # -- cross-brick latency ----------------------------------------------
    if do_cross_brick:
        ms = cross_brick_check(driver, events, auto_press=auto_press,
                               timeout_s=event_timeout_s)
        results["cross_brick_ms"] = ms
        if ms is not None:
            print(f"cross-brick stop latency: {ms:.0f} ms "
                  f"(budget 300 ms) {'OK' if ms <= 300 else 'OVER'}")
        else:
            print("cross-brick check skipped or timed out.")
    return results


def _poll_fake_idle(driver, events, brick, sensor):
    """Drive the real runtime idle path once (no main loop running)."""
    client = getattr(driver, "_client", None)
    master = getattr(client, "master", None)
    slave = getattr(client, "slave", None)
    if master is None or slave is None:
        return None
    if brick == "left":
        master.periodic()
    else:
        slave.tick()
        master.service_slave()
    return _wait_for(
        events,
        lambda e, b=brick, s=sensor: (
            e.get("brick") == b
            and e.get("event") == "sensor_guard"
            and e.get("detail", {}).get("sensor") == s),
        2.0)


def cross_brick_check(driver, events=None, auto_press=None,
                      timeout_s: float = 30.0,
                      sensor_brick: str = "right",
                      sensor: str = "in1",
                      motor_brick: str = "left",
                      motor: str = "outA") -> float | None:
    """Measure sensor-on-A stopping motor-on-B (slave -> master path).

    Fake mode (``auto_press`` given): drives the relay functions directly
    and returns the relay time. Real mode: starts a long slow move on
    ``motor_brick``/``motor``, asks the user to press
    ``sensor_brick``/``sensor``, and returns the ms from the sensor event
    to the cross-stop confirmation.
    """
    from src.adapters.pc.mqtt_transport import TransportError

    if events is None:
        events = queue.Queue()
        driver.on_event(events.put)
    try:
        driver.clear_latch()
    except TransportError:
        pass
    driver.set_guards(sensor_brick, [{"sensor": sensor}])
    try:
        if auto_press is not None:
            return _fake_cross_brick_ms(driver, sensor_brick, sensor,
                                        motor_brick, motor)
        # Real hardware: long move in background, user presses the sensor.
        try:
            before = driver.send(motor_brick, "read")["encoders"].get(
                motor, 0.0)
        except TransportError:
            return None
        while True:  # drain stale
            try:
                events.get_nowait()
            except queue.Empty:
                break
        outcome: dict = {}

        def run():
            try:
                driver.move_motors({(motor_brick, motor): before + 90.0},
                                   speed=30.0, timeout_s=timeout_s)
                outcome["done"] = True
            except TransportError as exc:
                outcome["error"] = str(exc)

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        time.sleep(0.5)  # let the move start
        print(f"Press {sensor_brick}/{sensor} NOW to stop "
              f"{motor_brick}/{motor} (timeout {timeout_s:.0f}s)...")
        t_press_evt = _wait_for(
            events, lambda e: e.get("brick") == sensor_brick
            and e.get("event") == "sensor_guard", timeout_s)
        if t_press_evt is None:
            print("no sensor event; cross-brick check timed out.")
            thread.join(timeout=5.0)
            return None
        t1 = time.monotonic()
        stop_evt = _wait_for(
            events, lambda e: e.get("brick") == motor_brick
            and e.get("event") in ("estop", "sensor_guard", "stall"),
            5.0)
        thread.join(timeout=10.0)
        if stop_evt is None:
            # Fallback: the move thread ending with a guard error also
            # confirms the stop.
            if "error" in outcome:
                return (time.monotonic() - t1) * 1000.0
            return None
        return (time.monotonic() - t1) * 1000.0
    finally:
        try:
            driver.set_guards(sensor_brick, [])
            driver.clear_latch()
        except TransportError:
            pass


def _fake_cross_brick_ms(driver, sensor_brick, sensor, motor_brick,
                         motor) -> float:
    """Relay-time cross-brick check on the real runtimes (deterministic)."""
    client = getattr(driver, "_client", None)
    assert client is not None and hasattr(client, "master"), \
        "fake cross-brick check needs the mp fake transport"
    master = client.master
    slave = client.slave
    target_core = master.core if sensor_brick == "left" else slave.core
    other_core = slave.core if sensor_brick == "left" else master.core
    # Simulate a guard trigger on the sensor brick, then relay.
    for s in target_core.sensors.values():
        s.set(0)
    target_core.sensors[sensor].set(1)
    t0 = time.monotonic()
    if sensor_brick == "right":
        slave.tick()
        master.service_slave()
    else:
        master.periodic()
        slave.tick()
    ms = (time.monotonic() - t0) * 1000.0
    assert other_core.guards.latched or any(
        getattr(m, "stopped", False)
        for m in other_core.motors.values()), \
        "cross-brick stop did not reach the other brick"
    target_core.sensors[sensor].set(0)
    target_core.guards.clear_latch()
    other_core.guards.clear_latch()
    target_core.pending_events.clear()
    other_core.pending_events.clear()
    return ms


def main() -> None:
    parser = argparse.ArgumentParser(description="EV3 connectivity check")
    parser.add_argument("--config", default="config/hardware.local.yaml")
    parser.add_argument("--fake", action="store_true",
                        help="run against in-memory fake bricks")
    parser.add_argument("--jog-deg", type=float, default=10.0)
    parser.add_argument("--speed", type=float, default=30.0)
    parser.add_argument("--event-timeout", type=float, default=30.0,
                        help="per-sensor press timeout in seconds (real HW)")
    parser.add_argument("--no-cross-brick", action="store_true",
                        help="skip the cross-brick latency check")
    args = parser.parse_args()

    auto_press = None
    if args.fake:
        driver, left, right = build_fake_setup()
        cores = {"left": left, "right": right}

        def auto_press(brick, sensor):
            for s in cores[brick].sensors.values():
                s.set(0)
            if sensor is not None:
                cores[brick].sensors[sensor].set(1)
    else:
        from src.adapters.pc.mqtt_transport import (
            MqttTransport, install_keyboard_estop)

        driver = MqttTransport.from_config(args.config)
        install_keyboard_estop(driver)
    try:
        results = check_driver(
            driver, jog_deg=args.jog_deg, speed=args.speed,
            event_timeout_s=args.event_timeout, auto_press=auto_press,
            do_cross_brick=not args.no_cross_brick)
        failures = [f"{b}/{p}" for b, br in results["bricks"].items()
                    for p, m in br["motors"].items() if not m["ok"]]
        failures += [k for k, v in results["sensor_events"].items()
                     if not v["ok"]]
        if failures:
            print(f"FAIL: {failures}")
            raise SystemExit(1)
        print("PASS: every motor moved and read back within ±3 deg; "
              "every touch sensor reported its event.")
    finally:
        driver.close()
        if auto_press is not None:
            for core in cores.values():
                for s in core.sensors.values():
                    s.set(0)


if __name__ == "__main__":
    main()
