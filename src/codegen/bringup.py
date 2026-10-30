"""Bring-up sequence runner (Stage 5.3 / 5.4).

Drives the physical arm through the ordered checks using one generated
package plus the inferred model (for IK). Every command installs the
keyboard emergency stop, arms the safety guards first and defaults to
the 30% first-run speed cap. Cartesian targets are position-only IK;
measure the tip error with a ruler and record it for the report.

Needs real bricks (``--config``) or runs offline against fake bricks
(``--fake``) for rehearsal. Homing must precede every motion command.

Usage::

    python -m src.codegen.bringup --generated generated/robot_inferred \\
        --model models/robot_inferred.json --fake home
    python -m src.codegen.bringup --generated generated/robot_inferred \\
        --model models/robot_inferred.json --config \\
        config/hardware.local.yaml cartesian --targets targets.json
"""

from __future__ import annotations

import argparse
import json
import sys

FIRST_RUN_SPEED_DEG_S = 30.0


def load_generated(path: str):
    sys.path.insert(0, path.rsplit("/", 1)[0] if "/" in path else ".")
    pkg = path.rsplit("/", 1)[-1]
    import importlib
    return importlib.import_module(f"{pkg}.robot_config"), \
        importlib.import_module(f"{pkg}.motion"), \
        importlib.import_module(f"{pkg}.home"), \
        importlib.import_module(f"{pkg}.guards")


def build_full_fake(actuators, guard_rules):
    """In-memory bricks with one motor per actuator and one sensor per
    guard rule, so ``--fake`` rehearses the exact generated wiring."""
    import json
    import re

    from src.ev3.brick.core import BrickCore
    from src.ev3.brick.motors import FakeMotor
    from src.ev3.brick.sensors import FakeSensor
    from src.ev3.driver import Driver

    motors: dict[str, dict] = {}
    for act in actuators:
        motors.setdefault(act["brick"], {})[act["port"]] = \
            FakeMotor(act["port"])
    sensors: dict[str, dict] = {}
    for brick, rules in guard_rules.items():
        for rule in rules:
            sensors.setdefault(brick, {})[rule["sensor"]] = \
                FakeSensor(rule["sensor"], 0)
    cores = {brick: BrickCore(brick, motors.get(brick, {}),
                              sensors.get(brick, {}))
             for brick in set(list(motors) + list(sensors))}

    class FakeClient:
        def __init__(self) -> None:
            self.on_message = None

        def subscribe(self, *_a, **_k) -> None:
            pass

        def publish(self, topic: str, payload: str, qos: int = 0):
            m = re.match(r"robot/cmd/(left|right)$", topic)
            if m:
                brick = m.group(1)
                ack = cores[brick].handle(json.loads(payload))
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

    return Driver(client_factory=FakeClient,
                  bricks=tuple(sorted(cores)))


def make_driver(args, mods):
    cfg, _motion, _home, _guards = mods
    if args.fake:
        return build_full_fake(cfg.ACTUATORS, cfg.GUARD_RULES)
    from src.ev3.driver import Driver, install_keyboard_estop
    driver = Driver.from_config(args.config)
    install_keyboard_estop(driver)
    return driver


def cmd_home(driver, mods, args):
    _cfg, _motion, home_mod, guards_mod = mods
    guards_mod.apply_guards(driver)
    report = home_mod.home_all(driver)
    with open(args.save, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    print(f"home report -> {args.save}")


def cmd_jog(driver, mods, args):
    _cfg, motion_mod, _home, guards_mod = mods
    guards_mod.apply_guards(driver)
    home = json.load(open(args.home, encoding="utf-8"))["encoders"] \
        if args.home else None
    q = [0.0] * 6
    q[args.joint] = args.deg
    motion_mod.move_joints(driver, q, duration_s=args.duration,
                           speed_cap=args.speed, home=home)


def cmd_poses(driver, mods, args):
    _cfg, motion_mod, _home, guards_mod = mods
    guards_mod.apply_guards(driver)
    home = json.load(open(args.home, encoding="utf-8"))["encoders"] \
        if args.home else None
    poses = json.load(open(args.poses, encoding="utf-8"))
    for pose in poses:
        motion_mod.move_joints(driver, pose, duration_s=args.duration,
                               speed_cap=args.speed, home=home)
    for pose in reversed(poses):
        motion_mod.move_joints(driver, [0.0] * len(pose),
                               duration_s=args.duration,
                               speed_cap=args.speed, home=home)


def cmd_cartesian(driver, mods, args):
    cfg, motion_mod, _home, guards_mod = mods
    from src.kin.ik import ik
    from src.kin.model import load_model
    guards_mod.apply_guards(driver)
    home = json.load(open(args.home, encoding="utf-8"))["encoders"] \
        if args.home else None
    doc = load_model(args.model)
    targets = json.load(open(args.targets, encoding="utf-8"))
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for t in targets:
        sol = ik(list(t["pos_mm"]), identity, doc, position_only=True)
        print(f"target {t['id']}: success={sol['success']} "
              f"pos_err={sol['pos_err_mm']:.2f}mm q={sol['q_deg']}")
        if not sol["success"]:
            print(f"  SKIP {t['id']}: IK failed, arm stays put")
            continue
        for rep in range(args.repeat):
            motion_mod.move_joints(driver, sol["q_deg"],
                                   duration_s=args.duration,
                                   speed_cap=args.speed, home=home)
            print(f"  {t['id']} rep {rep + 1}/{args.repeat}: "
                  f"measure tip error with a ruler now")


def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 5 bring-up runner")
    ap.add_argument("--generated", required=True,
                    help="generated/<robot> package dir")
    ap.add_argument("--model", required=True, help="model JSON (for IK)")
    ap.add_argument("--config", default="config/hardware.local.yaml")
    ap.add_argument("--fake", action="store_true",
                    help="rehearse against in-memory fake bricks")
    ap.add_argument("--speed", type=float, default=FIRST_RUN_SPEED_DEG_S)
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--home", default=None,
                    help="home report JSON from the home command")
    ap.add_argument("--save", default="runs/stage5/home_report.json")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("home", help="home all joints (guards first)")
    p_jog = sub.add_parser("jog", help="move one joint by +deg")
    p_jog.add_argument("--joint", type=int, required=True,
                       help="joint index 0-5")
    p_jog.add_argument("--deg", type=float, default=15.0)
    p_poses = sub.add_parser("poses", help="joint-space poses and back")
    p_poses.add_argument("--poses", required=True,
                         help="JSON list of 6-angle poses")
    p_cart = sub.add_parser("cartesian", help="position-only IK targets")
    p_cart.add_argument("--targets", required=True,
                        help='JSON list [{"id", "pos_mm"}]')
    p_cart.add_argument("--repeat", type=int, default=1)
    args = ap.parse_args()

    mods = load_generated(args.generated)
    driver = make_driver(args, mods)
    try:
        if args.cmd == "home":
            cmd_home(driver, mods, args)
        elif args.cmd == "jog":
            cmd_jog(driver, mods, args)
        elif args.cmd == "poses":
            cmd_poses(driver, mods, args)
        elif args.cmd == "cartesian":
            cmd_cartesian(driver, mods, args)
    finally:
        driver.close()


if __name__ == "__main__":
    main()
