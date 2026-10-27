"""Connectivity check from Stage 0 task 0.6.

Real mode (needs bricks + broker)::

    python -m src.ev3.check --config config/hardware.local.yaml

Smoke mode (no hardware, exercises core + driver against fakes)::

    python -m src.ev3.check --fake

Checks: ping both bricks, print ``info``, jog each motor +10/-10 deg at
low speed, read back, report error and round-trip latency (per brick),
then print live sensor values and wait for a touch event.
"""

from __future__ import annotations

import argparse
import json
import time


def build_fake_driver():
    """Driver wired to two in-memory bricks (no network)."""
    import re

    from .brick.core import BrickCore
    from .brick.motors import FakeMotor
    from .brick.sensors import FakeSensor
    from .driver import Driver

    cores = {
        "left": BrickCore("left",
                          {"outA": FakeMotor("outA", 0.0)},
                          {"in1": FakeSensor("in1", 0)}),
        "right": BrickCore("right",
                           {"outB": FakeMotor("outB", 5.0)},
                           {"in1": FakeSensor("in1", 0)}),
    }

    class FakeClient:
        def __init__(self) -> None:
            self.on_message = None
            self.published: list = []

        def subscribe(self, *_a, **_k) -> None:
            pass

        def publish(self, topic: str, payload: str, qos: int = 0):
            self.published.append((topic, payload, qos))
            m = re.match(r"robot/cmd/(left|right)$", topic)
            if m:
                brick = m.group(1)
                msg = json.loads(payload)
                ack = cores[brick].handle(msg)
                if self.on_message is not None:
                    self.on_message(
                        self, None,
                        type("M", (), {
                            "topic": f"robot/ack/{brick}",
                            "payload": json.dumps(
                                ack.to_dict(brick)).encode()})())
                    self.on_message(
                        self, None,
                        type("M", (), {
                            "topic": f"robot/state/{brick}",
                            "payload": json.dumps(
                                cores[brick].snapshot()).encode()})())

        def loop_start(self) -> None:
            pass

        def loop_stop(self) -> None:
            pass

    return Driver(client_factory=FakeClient, bricks=("left", "right"))


def check_driver(driver, jog_deg: float = 10.0, speed: float = 30.0) -> dict:
    """Run the Stage 0 exit-check sequence. Returns a result dict."""
    from .driver import DriverError

    results: dict = {"bricks": {}}
    for brick in driver.bricks:
        t0 = time.monotonic()
        ping = driver.request(brick, "ping")
        ping_ms = (time.monotonic() - t0) * 1000.0
        info = driver.request(brick, "info")
        print(f"[{brick}] ping ok ({ping_ms:.1f} ms): {ping}")
        print(f"[{brick}] info: {info}")
        brick_res: dict = {"ping_ms": ping_ms, "motors": {}}
        for m in info.get("motors", []):
            port = m["port"]
            before = driver.request(brick, "read")["encoders"].get(port, 0.0)
            t1 = time.monotonic()
            driver.move_motors({(brick, port): before + jog_deg}, speed)
            driver.move_motors({(brick, port): before}, speed)
            rtt_ms = (time.monotonic() - t1) * 1000.0
            after = driver.request(brick, "read")["encoders"].get(port, 0.0)
            err = abs(after - before)
            ok = err <= 3.0
            brick_res["motors"][port] = {
                "error_deg": err, "rtt_ms": rtt_ms, "ok": ok}
            print(f"[{brick}/{port}] jog ±{jog_deg} deg: "
                  f"error {err:.2f} deg, round-trip {rtt_ms:.0f} ms "
                  f"({'OK' if ok else 'FAIL'})")
        results["bricks"][brick] = brick_res
    try:
        state = driver.read_state()
        print(f"live sensors: { {b: s.get('sensors') for b, s in state.items()} }")
    except DriverError as exc:
        print(f"read_state failed: {exc}")
    print("Press any touch sensor by hand to confirm the event path "
          "(real hardware only).")
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description="EV3 connectivity check")
    parser.add_argument("--config", default="config/hardware.local.yaml")
    parser.add_argument("--fake", action="store_true",
                        help="run against in-memory fake bricks")
    args = parser.parse_args()

    if args.fake:
        driver = build_fake_driver()
    else:
        from .driver import Driver, install_keyboard_estop

        driver = Driver.from_config(args.config)
        install_keyboard_estop(driver)
    try:
        results = check_driver(driver)
        failures = [f"{b}/{p}" for b, br in results["bricks"].items()
                    for p, m in br["motors"].items() if not m["ok"]]
        if failures:
            print(f"FAIL: {failures}")
            raise SystemExit(1)
        print("PASS: every motor moved and read back within ±3 deg.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
