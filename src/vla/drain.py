"""Shared first-frame drain: fixed image + state feedback -> 50 actions.

Every VLA consumer (eval_policies, eval_ft, local_infer.predict_chunk,
stage9_eval_real, the webcam trial, the console digital twin) drains
through :func:`drain_chunk` so a single policy call can never again be
mistaken for a 50-step chunk (2026-11-02 single-call bug).

Protocol, per scene/episode:
- ``policy.reset()`` once (or every step when ``reset_each_step``).
- image fixed (first-frame mode) or re-rendered per step when
  ``render_fn`` is given (closed-loop mode).
- ``n`` select_action calls. SmolVLA queues the sampled chunk: the
  first call runs the model once (flow-matching sample) and calls
  2..n pop the queue, so in first-frame mode the fed-back states
  never re-enter the model -- the ``state_source`` choice is
  recorded but only stirs closed-loop mode, where every step
  re-runs the model. ``"command"`` = previous command value (used
  for real/webcam/twin); ``"mujoco"`` = MuJoCo-validated joint
  state (used for sim scenes; the repo MJCF is kinematic-only).
- missing mujoco: first-frame mode falls back to ``"command"`` and
  records ``state_fallback: mujoco-unavailable``; closed-loop mode
  raises instead of silently degrading.
- the stacked result must be exactly (50, 6); anything else raises.
"""

N_STEPS = 50

_MJ = {"doc": None, "model": None, "data": None}


def _mj_state(action, doc):
    """MuJoCo-validated joint state (deg) for a commanded action.

    The repo MJCF is kinematic-only (no actuators/dynamics), so this
    sets qpos, runs mj_forward and reads back the model's canonical
    joint state. It equals the command up to float error, and catches
    malformed commands (NaN/garbage) at the physics-model boundary.
    """
    import math
    import mujoco
    from src.kin.mjcf import build_mjcf
    if _MJ["doc"] is not doc:
        _MJ["model"] = mujoco.MjModel.from_xml_string(build_mjcf(doc))
        _MJ["data"] = mujoco.MjData(_MJ["model"])
        _MJ["doc"] = doc
    m, d = _MJ["model"], _MJ["data"]
    if len(action) != m.nq:
        raise ValueError(
            f"drain: action dim {len(action)} != model nq {m.nq}")
    d.qpos[:] = [math.radians(float(v)) for v in action]
    mujoco.mj_forward(m, d)
    out = [math.degrees(float(v)) for v in d.qpos[:]]
    if any(not math.isfinite(v) for v in out):
        raise ValueError("drain: non-finite MuJoCo state")
    return out


def _to_list(out):
    import torch
    if torch.is_tensor(out):
        return out.detach().cpu().float().tolist()
    return [list(map(float, row)) for row in out]


def drain_chunk(policy, preprocessor, postprocessor, img_u8, q0, task,
                n=N_STEPS, state_source="command", doc=None,
                render_fn=None, reset_each_step=False, frames_out=None):
    """Drain n actions; return (np.ndarray (n, 6), info dict)."""
    import numpy as np
    import torch  # local import: same convention as other VLA modules
    if state_source not in ("command", "mujoco"):
        raise ValueError(f"drain: bad state_source {state_source!r}")
    # Safety fallback: first-frame mode (fixed image) may fall back to
    # the previous command value when mujoco is not installed, and says
    # so in info. Closed-loop mode re-runs the model every step, so a
    # missing mujoco there is a hard error, never a silent fallback.
    eff_source, fallback = state_source, None
    if state_source == "mujoco":
        if doc is None:
            raise ValueError("drain: mujoco state needs doc")
        try:
            import mujoco as _mj_available
        except ImportError:
            _mj_available = None
        if _mj_available is None:
            if render_fn is not None:
                raise ModuleNotFoundError("drain: mujoco missing "
                                          "(closed-loop needs it)")
            eff_source, fallback = "command", "mujoco-unavailable"
    q = [float(v) for v in q0]
    if len(q) != 6:
        raise ValueError(f"drain: q0 must be 6 joints, got {len(q)}")
    img0 = np.asarray(img_u8, dtype=np.uint8)
    if img0.shape != (256, 256, 3):
        raise ValueError(f"drain: image must be 256x256x3, got "
                         f"{img0.shape}")
    policy.reset()
    acts = []
    with torch.no_grad():
        for _ in range(n):
            if reset_each_step:
                policy.reset()
            img = render_fn(q) if render_fn is not None else img0
            if frames_out is not None:
                frames_out.append(np.asarray(img, dtype=np.uint8))
            obs = {"observation.images.top":
                   torch.from_numpy(np.array(img, dtype=np.uint8,
                                             copy=True))
                   .permute(2, 0, 1).float() / 255.0,
                   "observation.state":
                   torch.tensor(q, dtype=torch.float32),
                   "task": task}
            out = postprocessor(policy.select_action(preprocessor(obs)))
            arr = np.asarray(_to_list(out), dtype=float)
            if arr.size != 6:
                raise ValueError(
                    f"drain: single action must have 6 values, got "
                    f"shape {arr.shape}")
            a = [float(v) for v in arr.reshape(-1)]
            acts.append(a)
            q = _mj_state(a, doc) if eff_source == "mujoco" else a
    stacked = np.asarray(acts, dtype=float)
    if stacked.shape != (n, 6):
        raise ValueError(f"drain: result must be ({n}, 6), got "
                         f"{stacked.shape}")
    info = {"state_source": eff_source, "n": n}
    if fallback is not None:
        info["state_fallback"] = fallback
    return stacked, info
