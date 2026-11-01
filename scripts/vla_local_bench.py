#!/usr/bin/env python3
"""Local VLA inference timing on the Mac (no robot motion).

Loads the downloaded checkpoint once, renders one twin frame for the
given slots, runs a single chunk prediction and prints load/infer
seconds. Missing deps or checkpoint -> clear message, exit 2.

Usage:
  .venv/bin/python scripts/vla_local_bench.py \
    --ckpt runs/vla/ckpt-last \
    --slots '{"S1": "red", "S3": "blue"}' --target red
"""

from __future__ import annotations

import argparse
import json
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ckpt", default="runs/vla/ckpt-last")
    ap.add_argument("--slots", default='{"S4": "red", "S2": "blue"}')
    ap.add_argument("--target", default="red")
    args = ap.parse_args(argv)

    from src.kin.model import load_model
    from src.vla import local_infer as LI
    from src.vla import twin as T
    from src.vla.scene import load_camera, render_frame

    deps = LI.deps_available()
    print("deps: " + json.dumps(deps))
    if not all(deps.values()):
        print("missing inference deps; install: "
              ".venv/bin/pip install torch transformers "
              "'lerobot[smolvla]==0.6.1'")
        return 2
    scene = T.scene_from_slots(json.loads(args.slots), args.target)
    img = render_frame(scene, list(T.REST_Q),
                       load_model("models/robot_inferred.json"),
                       load_camera("config/sim_camera.yaml"), None)
    try:
        out = LI.predict_chunk(args.ckpt, img, list(T.REST_Q),
                               f"Point to the {args.target} block.")
    except RuntimeError as e:
        print("cannot run: " + str(e))
        return 2
    n = len(out["actions"])
    d = len(out["actions"][0]) if n else 0
    print(f"ckpt: {args.ckpt}")
    print(f"chunk: {n} steps x {d} dims")
    print(f"load_s: {out['load_s']}")
    print(f"infer_s (one chunk): {out['infer_s']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
