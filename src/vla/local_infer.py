"""Local SmolVLA inference for the digital twin (Mac side).

Heavy imports (torch, lerobot) stay inside functions so the console
and the test suite import this module without them. Returns plain
floats only.
"""

from __future__ import annotations

import os
import time

RENAME_MAP = {"observation.images.top": "observation.images.camera1"}
EMPTY_CAMERAS = 2


def deps_available() -> dict:
    """Check the optional inference stack without importing torch."""
    import importlib.util

    return {"torch": importlib.util.find_spec("torch") is not None,
            "lerobot": importlib.util.find_spec("lerobot") is not None,
            "transformers":
                importlib.util.find_spec("transformers") is not None}


def predict_chunk(ckpt_dir: str, image_u8, q_deg: list,
                  task: str) -> dict:
    """Load ckpt (once per call) and predict one 50-step action chunk.

    ``image_u8`` is an HxWx3 uint8 array (top camera). Returns
    ``{"actions": [[..] x 50], "load_s": s, "infer_s": s}``.
    Raises RuntimeError with a plain message when deps/ckpt miss.
    """
    missing = [k for k, v in deps_available().items() if not v]
    if missing:
        raise RuntimeError("missing inference deps: " + ",".join(missing)
                           + " (pip install torch transformers "
                           + "lerobot[smolvla])")
    if not os.path.isdir(ckpt_dir):
        raise RuntimeError("checkpoint not found: " + str(ckpt_dir))
    import torch
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    t0 = time.monotonic()
    policy = SmolVLAPolicy.from_pretrained(ckpt_dir)
    policy.eval()
    # The checkpoint pins device=cuda; fall back to CPU where CUDA
    # is absent (e.g. Mac local runs) so the same call works.
    if torch.cuda.is_available():
        device_name = "cuda"
    else:
        device_name = "cpu"
        policy = policy.to(device_name)
    preprocessor, postprocessor = make_pre_post_processors(
        policy.config, pretrained_path=ckpt_dir,
        preprocessor_overrides={
            "rename_observations_processor": {
                "rename_map": RENAME_MAP},
            "device_processor": {"device": device_name},
        })
    load_s = time.monotonic() - t0
    from src.vla.drain import N_STEPS, drain_chunk
    t1 = time.monotonic()
    stacked, info = drain_chunk(
        policy, preprocessor, postprocessor, image_u8, q_deg, task,
        n=N_STEPS, state_source="command")
    infer_s = time.monotonic() - t1
    actions = [[float(v) for v in row] for row in stacked]
    return {"actions": actions, "load_s": round(load_s, 2),
            "infer_s": round(infer_s, 3),
            "state_source": info["state_source"], "n_steps": info["n"]}
