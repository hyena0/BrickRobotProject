"""6.2 trial run on Modal (the user runs this in their own terminal).

Ephemeral only: use ``modal run`` (never ``modal deploy``).
Needs the trial dataset on the Volume first (see run table) and a
Hugging Face secret named ``huggingface`` (token only; the user
registers it, it never appears in code).

All flags below are verified against lerobot 0.6.1 source
(TrainPipelineConfig/DatasetConfig/PreTrainedConfig).
"""

from __future__ import annotations

import modal

def to_jsonable(obj):
    """Recursively convert a payload to JSON primitives only.

    Remote return values travel through pickle, which forces the *local*
    env to import whatever classes they contain (e.g. torch). Plain
    JSON strings need nothing. torch.Size is a tuple, Tensors expose
    tolist(), dtypes fall back to str.
    """
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    if isinstance(obj, float):
        return obj
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    tolist = getattr(obj, "tolist", None)
    if callable(tolist):
        try:
            return to_jsonable(tolist())
        except Exception:  # noqa: BLE001
            pass
    return str(obj)


def dumps_result(payload) -> str:
    """Serialize a remote result; always returns str (never pickled)."""
    import json
    return json.dumps(to_jsonable(payload), indent=1, default=str)


app = modal.App("brickrobot-vla-trial")

VOLUME_NAME = "brickrobot-vla"
DATA_MOUNT = "/data"
DATA_DIR = "/data/lerobot-trial"
OUT_DIR = "/data/trial-out"
#: 6.3 full datasets (360 train / 40 val). Uploaded by the user.
#: Local mirrors: runs/vla/lerobot-train, runs/vla/lerobot-val.
TRAIN_DATA_DIR = "/data/lerobot-train"
VAL_DATA_DIR = "/data/lerobot-val"
EXPECTED_TRAIN_EPS = 360
EXPECTED_VAL_EPS = 40

#: Trial config (6.2, 20 episodes). Kept separate so trial values can
#: never leak into full training.
TRIAL_CFG = {"steps": 200, "batch_size": 8, "save_freq": 100,
             "job_name": "vla-trial-200", "proc_timeout_s": 1700,
             "dataset_root": DATA_DIR}
#: Full training config (6.3).
FULL_CFG = {"steps": 6000, "batch_size": 32, "save_freq": 1000,
            "job_name": "vla-full-6000", "proc_timeout_s": 13800,
            "dataset_root": TRAIN_DATA_DIR}
#: Bench config (speed probe only).
BENCH_CFG = {"steps": 50, "batch_size": 32, "save_freq": 100,
             "job_name": "vla-bench-50", "proc_timeout_s": 1700,
             "dataset_root": DATA_DIR}
REPO_ID = "local/brickrobot-point"
# (see LEROBOT_PIN comment below)
# lerobot 0.6.1 extras, verified from the wheel METADATA: "smolvla"
# pulls transformers/accelerate/num2words, "dataset" pulls
# datasets/pandas/pyarrow/av/torchcodec/jsonlines.
LEROBOT_PIN = "lerobot[smolvla,dataset]==0.6.1"

#: Modules the training step imports. check_env imports every one, so a
#: passing env can no longer fail train at the import stage.
TRAIN_MODULES = [
    "lerobot.datasets",
    "lerobot.datasets.lerobot_dataset",
    "lerobot.policies.smolvla.modeling_smolvla",
    "lerobot.scripts.lerobot_train",
]

#: Dataset key -> policy key. smolvla_base expects camera1..3; we have one
#: top camera. The rename processor applies this at runtime, and with a
#: rename_map the validator is skipped by design (factory.py).
RENAME_MAP = {"observation.images.top": "observation.images.camera1"}
#: Absent streams (camera2/3) become masked zero-images in forward
#: (modeling_smolvla.prepare_images); base default is 0.
EMPTY_CAMERAS = 2

image = (
    modal.Image.debian_slim(python_version="3.12")
    .apt_install("ffmpeg")
    .pip_install(LEROBOT_PIN, "pyyaml", "scipy")
    # Eval runs the repo's sim + IK on the remote side. Only public
    # files are baked: slot coords live in src/vla/slot_seeds.json,
    # so assets/private and *.local.* never leave the Mac.
    .add_local_dir("src", "/root/src")
    .add_local_file("config/sim_camera.yaml",
                    "/root/config/sim_camera.yaml")
    .add_local_dir("models", "/root/models")
    .add_local_dir("packages", "/root/packages")
    .add_local_dir("runs/vla/6.0", "/root/runs/vla/6.0")
)

# Local paths baked into the image above. The upload guard scans
# exactly this set and blocks execution on any private pattern.
BAKED_LOCAL_PATHS = ("src", "config/sim_camera.yaml", "models",
                     "packages", "runs/vla/6.0")
UPLOAD_DENY = ("assets/private", ".local.", "visual_pack/", ".ldr",
               ".io", "/renders", "inventory.local")


def _check_upload_list(paths=None) -> None:
    """Refuse to run when a baked path carries private content.

    Pure (takes explicit paths) so tests can feed tmp fixtures.
    """
    import pathlib

    roots = [pathlib.Path(p) for p in
             (BAKED_LOCAL_PATHS if paths is None else paths)]
    bad = []
    for root in roots:
        if root.is_file():
            cands = [root]
        elif root.is_dir():
            cands = [p for p in root.rglob("*") if p.is_file()]
        else:
            bad.append(str(root) + " (missing)")
            continue
        for p in cands:
            s = p.as_posix()
            if any(d in s for d in UPLOAD_DENY):
                bad.append(s)
    if bad:
        raise RuntimeError(
            "private upload blocked: "
            + ", ".join(sorted(set(bad))[:10]))
vol = modal.Volume.from_name(VOLUME_NAME, create_if_missing=True)
hf_secret = modal.Secret.from_name("huggingface")


@app.function(image=image, gpu="A100-40GB", timeout=600)
def check_a100() -> str:
    """Short A100 availability + access check (also warms quota)."""
    import shutil
    import subprocess
    import sys

    out = {"python": sys.version.split()[0]}
    if shutil.which("nvidia-smi") is None:
        return dumps_result({**out, "ok": False,
                             "error": "no nvidia-smi"})
    proc = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,memory.total,driver_version",
         "--format=csv,noheader"],
        capture_output=True, text=True, timeout=60)
    out.update({"ok": proc.returncode == 0,
                "returncode": proc.returncode,
                "gpu": proc.stdout.strip(),
                "stderr": proc.stderr.strip()[-300:]})
    return dumps_result(out)


@app.function(image=image, gpu="T4", timeout=1200,
              volumes={DATA_MOUNT: vol}, secrets=[hf_secret])
def check_env() -> str:
    """Toolchain + data check on the cheap GPU (no training)."""
    import json
    import os
    import sys

    rep: dict = {"python": sys.version.split()[0]}
    try:
        import torch
        rep["torch"] = str(torch.__version__)
        rep["cuda"] = str(torch.cuda.get_device_name(0))
    except Exception as exc:  # noqa: BLE001
        return dumps_result({**rep, "ok": False,
                             "error": f"torch: {exc}"[:200]})
    try:
        import lerobot
        rep["lerobot"] = str(lerobot.__version__)
        from lerobot.policies.smolvla.configuration_smolvla import (
            SmolVLAConfig)
        rep["smolvla_chunk"] = int(SmolVLAConfig.chunk_size)
    except Exception as exc:  # noqa: BLE001
        return dumps_result({**rep, "ok": False,
                             "error": f"lerobot: {exc}"[:200]})
    import importlib
    missing = []
    for mod in TRAIN_MODULES:
        try:
            importlib.import_module(mod)
        except Exception:  # noqa: BLE001
            missing.append(mod)
    rep["train_imports_ok"] = not missing
    if missing:
        return dumps_result({**rep, "ok": False,
                             "error": "missing: " + ",".join(missing)})
    import shutil
    import subprocess
    if shutil.which("lerobot-train") is None:
        return dumps_result({**rep, "ok": False,
                             "error": "no lerobot-train binary"})
    proc = subprocess.run(["lerobot-train", "--help"],
                          capture_output=True, text=True, timeout=120)
    rep["train_help_ok"] = proc.returncode == 0
    if proc.returncode != 0:
        return dumps_result({**rep, "ok": False,
                             "error": "lerobot-train --help failed: " +
                             proc.stderr.strip()[-300:]})
    info_path = os.path.join(DATA_DIR, "meta", "info.json")
    try:
        info = json.load(open(info_path, encoding="utf-8"))
        rep["dataset_episodes"] = int(info["total_episodes"])
        rep["dataset_fps"] = int(info["fps"])
    except Exception as exc:  # noqa: BLE001
        return dumps_result({**rep, "ok": False,
                             "error": f"dataset missing at {info_path}: "
                             f"{exc}"[:200]})
    extra = {}
    for name, root in (("train", TRAIN_DATA_DIR), ("val", VAL_DATA_DIR)):
        try:
            meta = json.load(open(os.path.join(root, "meta", "info.json"),
                                  encoding="utf-8"))
            extra[name] = int(meta["total_episodes"])
        except Exception:  # noqa: BLE001
            extra[name] = 0
    rep["extra_datasets"] = extra
    try:
        from lerobot.configs import FeatureType
        from lerobot.datasets.dataset_metadata import (
            LeRobotDatasetMetadata)
        from lerobot.policies.smolvla.configuration_smolvla import (
            SmolVLAConfig)
        from lerobot.policies.utils import (
            validate_visual_features_consistency)
        from lerobot.utils.feature_utils import (
            dataset_to_policy_features)
        base = SmolVLAConfig.from_pretrained("lerobot/smolvla_base")
        base.empty_cameras = EMPTY_CAMERAS
        meta = LeRobotDatasetMetadata(REPO_ID, root=DATA_DIR)
        feats = dataset_to_policy_features(meta.features)
        provided = {k for k, v in feats.items()
                    if v.type == FeatureType.VISUAL}
        expected = {k for k, v in base.input_features.items()
                    if v.type == FeatureType.VISUAL}
        if RENAME_MAP:
            renamed = {RENAME_MAP.get(k, k) for k in provided}
            unmapped_src = sorted(set(RENAME_MAP) - provided)
            missing = sorted(expected - renamed)
            rep["policy_visuals"] = {
                "renamed": sorted(renamed),
                "expected": sorted(expected),
                "pad_missing": missing}
            if unmapped_src or len(missing) > EMPTY_CAMERAS:
                return dumps_result({**rep, "ok": False, "error": (
                    "rename endpoints invalid: unmapped-src="
                    f"{unmapped_src} missing-beyond-pad={missing}")[:300]})
        else:
            validate_visual_features_consistency(base, feats)
        st = feats.get("observation.state")
        at = feats.get("action")
        exp_st = base.input_features.get("observation.state")
        exp_at = base.output_features.get("action")
        rep["state_shape"] = [int(v) for v in st.shape] if st else None
        rep["action_shape_expected"] = (
            [int(v) for v in at.shape] if at else None)
        if (st is None or at is None
                or [int(v) for v in st.shape] != [int(v) for v in
                                                  exp_st.shape]
                or [int(v) for v in at.shape] != [int(v) for v in
                                                  exp_at.shape]):
            return dumps_result({**rep, "ok": False, "error": (
                "state/action dim mismatch: state="
                f"{rep['state_shape']} action="
                f"{rep['action_shape_expected']}")[:200]})
        rep["policy_check_ok"] = True
    except Exception as exc:  # noqa: BLE001
        return dumps_result({**rep, "ok": False,
                             "error": f"policy gate: {exc}"[:300]})
    rep["ok"] = True
    return dumps_result(rep)


def _latest_ckpt(out_dir=OUT_DIR):
    """Newest checkpoints/*/pretrained_model on the Volume (or None)."""
    import glob
    import os
    ckpts = sorted(glob.glob(os.path.join(
        out_dir, "checkpoints", "*", "pretrained_model")))
    return ckpts[-1] if ckpts else None


def _latest_ckpt_any():
    """Newest checkpoint across trial-out and train-* runs (or None).

    Compares by mtime so a newer run wins even at the same step count.
    /data/trial-out (6.2) is never deleted or overwritten.
    """
    import glob
    import os
    cands = []
    for root in [OUT_DIR] + sorted(glob.glob("/data/train-*")):
        cands += glob.glob(os.path.join(
            root, "checkpoints", "*", "pretrained_model"))
    if not cands:
        return None
    return max(cands, key=os.path.getmtime)


def _run_dir(prefix, root="/data"):
    """Timestamped run dir name (never pre-created here)."""
    import datetime
    stamp = datetime.datetime.now(
        datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")
    return f"{root}/{prefix}-{stamp}"


def _unique_dir(base):
    """First non-existing path: base, base-2, base-3, ..."""
    import os
    out, k = base, 2
    while os.path.exists(out):
        out = f"{base}-{k}"
        k += 1
    return out


def _prepare_run_dirs(prefix, root="/data"):
    """Return (out_dir, log_path) for a fresh run.

    out_dir is NEVER created here: lerobot-train creates it and
    refuses existing ones, so pre-creating (even for train.log) breaks
    the run. Only the logs/ parent is created; the log file lives
    next to the output tree, not inside it.
    """
    import os
    out_dir = _unique_dir(_run_dir(prefix, root))
    log_dir = os.path.join(root, "logs")
    os.makedirs(log_dir, exist_ok=True)
    return out_dir, os.path.join(
        log_dir, os.path.basename(out_dir) + ".log")


def _check_full_data():
    """Verify the 400-episode dataset (360 train + 40 val) on the Volume.

    Returns (ok, detail). Training must not start unless both splits
    match exactly (catches a wrong --dataset.root like the 20-ep trial).
    reason is one of: ok, missing-path, no-meta, bad-meta, or
    count-mismatch.
    """
    import json
    import os

    detail = {}
    for name, root, want in (
            ("train", TRAIN_DATA_DIR, EXPECTED_TRAIN_EPS),
            ("val", VAL_DATA_DIR, EXPECTED_VAL_EPS)):
        if not os.path.isdir(root):
            detail[name] = {"episodes": None, "want": want,
                            "reason": "missing-path"}
            continue
        info_path = os.path.join(root, "meta", "info.json")
        if not os.path.isfile(info_path):
            detail[name] = {"episodes": None, "want": want,
                            "reason": "no-meta"}
            continue
        try:
            got = int(json.load(open(info_path,
                                      encoding="utf-8"))["total_episodes"])
        except Exception as exc:  # noqa: BLE001
            detail[name] = {"episodes": None, "want": want,
                            "reason": f"bad-meta: {exc}"[:120]}
            continue
        detail[name] = {"episodes": got, "want": want,
                        "reason": "ok" if got == want
                        else "count-mismatch"}
    ok = all(v["episodes"] == v["want"] for v in detail.values())
    return ok, detail


def _run_train(cfg, prefix, root="/data"):
    """Run lerobot-train; returns (ckpt|None, train_s, tail, error|None).

    Streams child output line by line: every line goes to the Modal
    logs in real time (visible via `modal app logs`) and is appended
    to <root>/logs/<run>.log on the Volume. proc timeout comes from
    cfg and must stay below the Modal function timeout.

    The output dir is picked fresh here and NEVER pre-created:
    lerobot-train creates it and refuses existing ones. No resume
    path: reruns always start a new dir.
    """
    import json
    import subprocess
    import time

    out_dir, log_path = _prepare_run_dirs(prefix, root)
    t0 = time.time()
    cmd = ["lerobot-train",
           f"--dataset.repo_id={REPO_ID}",
           f"--dataset.root={cfg['dataset_root']}",
           "--policy.path=lerobot/smolvla_base",
           "--policy.device=cuda",
           "--policy.push_to_hub=false",
           f"--policy.empty_cameras={EMPTY_CAMERAS}",
           f"--rename_map={json.dumps(RENAME_MAP)}",
           f"--steps={cfg['steps']}", f"--batch_size={cfg['batch_size']}",
           f"--save_freq={cfg['save_freq']}", "--log_freq=10",
           f"--output_dir={out_dir}",
           f"--job_name={cfg['job_name']}",
           "--seed=62",
           "--wandb.enable=false"]
    tail: list[str] = []
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True,
                                bufsize=1)
        assert proc.stdout is not None
        with open(log_path, "w", encoding="utf-8") as fh:
            for line in proc.stdout:
                print(line, end="", flush=True)
                fh.write(line)
                fh.flush()
                tail.append(line)
                del tail[:-50]
        proc.wait(timeout=cfg["proc_timeout_s"])
    except subprocess.TimeoutExpired:
        proc.kill()
        return None, round(time.time() - t0, 1), "".join(tail)[-1500:], \
            f"subprocess timeout after {cfg['proc_timeout_s']}s"
    train_s = round(time.time() - t0, 1)
    tail_s = "".join(tail)[-1500:]
    if proc.returncode != 0:
        return None, train_s, tail_s, f"exit {proc.returncode}"
    ckpt = _latest_ckpt(out_dir)
    if ckpt is None:
        return None, train_s, tail_s, "no checkpoint written"
    return ckpt, train_s, tail_s, None


def _infer_action(ckpt):
    """Load policy + saved pre/post processors, run one action."""
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    policy = SmolVLAPolicy.from_pretrained(ckpt)
    policy.to("cuda")
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy.config,
        pretrained_path=ckpt,
        preprocessor_overrides={
            "device_processor": {"device": "cuda"},
            "rename_observations_processor": {
                "rename_map": RENAME_MAP},
        })
    ds = LeRobotDataset(REPO_ID, root=DATA_DIR)
    raw = ds[0]
    task = raw["task"]
    if not isinstance(task, str):
        task = str(task)
    obs = {"observation.images.top": raw["observation.images.top"],
           "observation.state": raw["observation.state"].float(),
           "task": task}
    with torch.no_grad():
        action = policy.select_action(preprocessor(obs))
        out = postprocessor(action)
    return {"action": to_jsonable(out),
            "action_type": type(out).__name__}


@app.function(image=image, gpu="T4", timeout=1800,
              volumes={DATA_MOUNT: vol}, secrets=[hf_secret])
def train_trial() -> str:
    """200 training steps, then save -> load -> one inference.

    Fresh timestamped dir per run (/data/train-*); the 6.2 dir
    /data/trial-out is never touched.
    """
    import time

    ckpt, train_s, tail, error = _run_train(TRIAL_CFG, "train")
    if error is not None:
        return dumps_result({"ok": False, "stage": "train",
                             "error": error,
                             "train_s": train_s, "tail": tail})
    t1 = time.time()
    try:
        result = _infer_action(ckpt)
    except Exception as exc:  # noqa: BLE001
        return dumps_result({"ok": False, "stage": "load-infer",
                             "train_s": train_s, "checkpoint": ckpt,
                             "infer_error": f"{type(exc).__name__}: {exc}"
                             [:300], "tail": tail})
    infer_s = round(time.time() - t1, 1)
    return dumps_result({"ok": True, "train_s": train_s,
                         "infer_s": infer_s, "checkpoint": ckpt,
                         **result})


@app.function(image=image, gpu="T4", timeout=1200,
              volumes={DATA_MOUNT: vol}, secrets=[hf_secret])
def infer_trial() -> str:
    """Load the saved checkpoint from the Volume, run one inference.

    No training. Uses the newest checkpoint across /data/trial-out
    and /data/train-*. Fails clearly when none exists yet.
    """
    import time

    ckpt = _latest_ckpt_any()
    if ckpt is None:
        return dumps_result({"ok": False, "stage": "find-checkpoint",
                             "error": "no checkpoint on Volume yet; "
                             "run train first"})
    t1 = time.time()
    try:
        result = _infer_action(ckpt)
    except Exception as exc:  # noqa: BLE001
        return dumps_result({"ok": False, "stage": "load-infer",
                             "checkpoint": ckpt,
                             "infer_error": f"{type(exc).__name__}: {exc}"
                             [:300]})
    infer_s = round(time.time() - t1, 1)
    return dumps_result({"ok": True, "infer_s": infer_s,
                         "checkpoint": ckpt, **result})


@app.function(image=image, gpu="A100-40GB", timeout=1800,
              volumes={DATA_MOUNT: vol}, secrets=[hf_secret])
def bench() -> str:
    """A100 speed probe: 50 steps, batch 32. Reports sec/step.

    Output goes to a fresh container-/tmp dir (discarded with the
    container); nothing is written to the Volume.
    """
    ckpt, train_s, tail, error = _run_train(BENCH_CFG, "bench",
                                            root="/tmp")
    if error is not None:
        return dumps_result({"ok": False, "stage": "train",
                             "error": error,
                             "train_s": train_s, "tail": tail})
    return dumps_result({"ok": True, "steps": 50, "batch_size": 32,
                         "train_s": train_s,
                         "sec_per_step": round(train_s / 50, 2),
                         "checkpoint": ckpt,
                         "note": "includes one-time model download; "
                         "steady state is faster"})


@app.function(image=image, gpu="A100-40GB", timeout=14400,
              volumes={DATA_MOUNT: vol}, secrets=[hf_secret])
def train_full() -> str:
    """6.3 full training: A100, batch 32, 6000 steps, ckpt every 1000.

    Fresh timestamped dir per run (/data/train-*); 6.2 /data/trial-out
    and the failed 20261031-194324 dir are never touched. Stops before
    starting unless the Volume holds exactly 360 train + 40 val
    episodes. Subprocess streams to Modal logs and
    /data/logs/<run>.log; its timeout (3h50m) sits below the 4h
    function timeout. No resume: reruns always start a new dir.
    """
    ok, detail = _check_full_data()
    if not ok:
        return dumps_result({"ok": False, "stage": "data-check",
                             "detail": detail,
                             "error": "need 360 train + 40 val episodes"})
    ckpt, train_s, tail, error = _run_train(FULL_CFG, "train")
    import os as _os
    out_dir = _os.path.dirname(
        _os.path.dirname(_os.path.dirname(ckpt))) if ckpt else None
    if error is not None:
        return dumps_result({"ok": False, "stage": "train",
                             "error": error, "out_dir": out_dir,
                             "train_s": train_s, "tail": tail})
    return dumps_result({"ok": True, "steps": 6000, "batch_size": 32,
                         "out_dir": out_dir, "train_s": train_s,
                         "sec_per_step": round(train_s / 6000, 2),
                         "checkpoint": ckpt, "tail": tail[-500:]})


EVAL_N = 100
EVAL_SEED = 64


def _resolve_eval_ckpt(want_step, root="/data"):
    """Newest <root>/train-* run, checkpoint at/nearest below want_step."""
    import glob
    import os
    import re

    runs = sorted(glob.glob(os.path.join(root, "train-*")))
    if not runs:
        return None, None
    run = max(runs, key=os.path.getmtime)
    cands = []
    for d in glob.glob(os.path.join(
            run, "checkpoints", "*", "pretrained_model")):
        m = re.search(r"(\d+)$", os.path.basename(os.path.dirname(d)))
        if m:
            cands.append((int(m.group(1)), d))
    if not cands:
        return None, run
    below = [c for c in cands if c[0] <= want_step]
    pick = max(below) if below else min(cands)
    return pick[1], run


@app.function(image=image, gpu="T4", timeout=10800,
              volumes={DATA_MOUNT: vol}, secrets=[hf_secret])
def eval_policies() -> str:
    """6.4 eval: 100 unseen scenes x (closed-loop, first-frame-only).

    Plus expert (6.1 pipeline) and random baselines, for the 3000-step
    and 6000-step checkpoints. Closed-loop replans every step
    (policy.reset + fresh obs); first-frame-only drains the first
    50-chunk open-loop. Videos: first 6 closed-loop episodes of the
    6000 ckpt, tiled 2x3.
    """
    import sys
    sys.path.insert(0, "/root")
    import json
    import os
    import random
    import subprocess

    import torch

    import src.vla.demo as D
    import src.vla.evaluate as E
    import src.vla.scene as S
    from src.kin.model import load_model

    os.chdir("/root")
    rng = random.Random(EVAL_SEED)
    cam = S.load_camera("/root/config/sim_camera.yaml")
    doc = load_model("/root/models/robot_inferred.json")
    tdoc = S.tip_doc(doc)
    rep60 = json.load(open("/root/runs/vla/6.0/report.json",
                           encoding="utf-8"))
    rest_q = [float(v) for v in rep60["rest_q_deg"]]
    lims = S.adopted_ranges()
    coupled = S.load_package()["coupled_limits"]
    slots = S.load_slot_seeds()
    mat = slots
    mat_seeds = [p["seed"] for p in slots]
    from src.vla.scene import nearest_mat_seed

    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    out_dir, _ = _prepare_run_dirs("eval")
    os.makedirs(out_dir, exist_ok=True)

    scenes = [E.sample_scene_for_eval(rng) for _ in range(EVAL_N)]

    def final_tip(traj):
        tip, _ = S.tip_world(traj[-1], tdoc)
        return [float(v) for v in tip]

    expert_rows = []
    for i, sc in enumerate(scenes):
        tb = next(b for b in sc["blocks"]
                  if b["colour"] == sc["target"])
        seed_q = nearest_mat_seed(tb["x"], tb["y"], mat_seeds, mat)
        rec, traj = D.run_demo(i, sc, rest_q, tdoc, seed_q)
        if traj is None:
            expert_rows.append({"success": False, "wrong_block": False,
                                "hold_herr_mm": 1e9, "hold_z_mm": -1})
        else:
            expert_rows.append(E.score_episode(final_tip(traj),
                                              sc["blocks"],
                                              sc["target"]))
    rng_r = random.Random(6464)
    random_rows = []
    for sc in scenes:
        q = list(rest_q)
        for _ in range(50):
            q = E.random_step(rng_r, lims, coupled)
        tip, _ = S.tip_world(q, tdoc)
        random_rows.append(E.score_episode([float(v) for v in tip],
                                          sc["blocks"], sc["target"]))

    def to_obs(img_np, q, task):
        return {"observation.images.top":
                torch.from_numpy(img_np).permute(2, 0, 1).float()
                / 255.0,
                "observation.state":
                torch.tensor(q, dtype=torch.float32),
                "task": task}

    results = {"expert": E.summarize_eval(expert_rows),
               "random": E.summarize_eval(random_rows),
               "checkpoints": {}}
    for want in (3000, 6000):
        ckpt, run = _resolve_eval_ckpt(want)
        if ckpt is None:
            results["checkpoints"][str(want)] = {
                "ok": False, "error": "no checkpoint found",
                "run": run}
            continue
        policy = SmolVLAPolicy.from_pretrained(ckpt)
        policy.to("cuda")
        policy.eval()
        preprocessor, postprocessor = make_pre_post_processors(
            policy.config, pretrained_path=ckpt,
            preprocessor_overrides={
                "device_processor": {"device": "cuda"},
                "rename_observations_processor": {
                    "rename_map": RENAME_MAP},
            })
        modes = {}
        for mode in ("closed", "first"):
            rows = []
            ep_frames = []
            for i, sc in enumerate(scenes):
                task = f"Point to the {sc['target']} block."
                policy.reset()
                img0 = None
                q = list(rest_q)
                frames = []
                with torch.no_grad():
                    for t in range(50):
                        if mode == "closed" or img0 is None:
                            img = S.render_frame(sc, q, doc, cam, None)
                            if mode == "closed" and i < 6:
                                frames.append(img)
                            if mode == "first":
                                img0 = img
                        else:
                            img = img0
                        if mode == "closed":
                            policy.reset()
                        batch = preprocessor(to_obs(img, q, task))
                        action = policy.select_action(batch)
                        out = postprocessor(action)
                        a = (out.detach().flatten().tolist()
                             if torch.is_tensor(out) else list(out))
                        q = [float(v) for v in a]
                tip, _ = S.tip_world(q, tdoc)
                rows.append(E.score_episode([float(v) for v in tip],
                                            sc["blocks"], sc["target"]))
                if mode == "closed" and i < 6:
                    ep_frames.append(frames)
            modes[mode] = E.summarize_eval(rows)
            if mode == "closed":
                vdir = os.path.join(out_dir, f"ckpt_{want}_frames")
                os.makedirs(vdir, exist_ok=True)
                mp4s = []
                for j, frames in enumerate(ep_frames):
                    fdir = os.path.join(vdir, f"ep{j:02d}")
                    os.makedirs(fdir, exist_ok=True)
                    for k, fr in enumerate(frames):
                        S.write_png(os.path.join(fdir, f"f{k:02d}.png"),
                                    fr)
                    mp4 = os.path.join(vdir, f"ep{j:02d}.mp4")
                    subprocess.run(
                        ["ffmpeg", "-y", "-v", "error", "-framerate",
                         "10", "-i", os.path.join(fdir, "f%02d.png"),
                         "-pix_fmt", "yuv420p", mp4], check=True)
                    mp4s.append(mp4)
                grid = os.path.join(out_dir, f"grid_{want}.mp4")
                filt = ("[0:v][1:v][2:v][3:v][4:v][5:v]xstack=inputs=6:"
                        "layout=0_0|256_0|512_0|0_256|256_256|512_256")
                subprocess.run(
                    ["ffmpeg", "-y", "-v", "error",
                     "-i", mp4s[0], "-i", mp4s[1], "-i", mp4s[2],
                     "-i", mp4s[3], "-i", mp4s[4], "-i", mp4s[5],
                     "-filter_complex", filt, "-pix_fmt", "yuv420p",
                     grid], check=True)
                modes[mode]["grid"] = grid
                modes[mode]["episodes"] = mp4s
        results["checkpoints"][str(want)] = {"ok": True,
                                             "checkpoint": ckpt,
                                             "run": run, **modes}
        del policy
    with open(os.path.join(out_dir, "report.json"), "w",
              encoding="utf-8") as fh:
        json.dump(results, fh, indent=1)
    results["out_dir"] = out_dir
    return dumps_result(results)


SENS_N = 20
SENS_SEED = 6464
SENS_COLOURS = ("red", "blue", "green", "yellow")


@app.function(image=image, gpu="T4", timeout=1800,
              volumes={DATA_MOUNT: vol}, secrets=[hf_secret])
def sens_instructions() -> str:
    """Same 20 scenes x 4 colour instructions (first-frame chunk).

    Decides whether the instruction changes behaviour at all: one
    rendered rest frame per scene, one 50-step chunk per colour,
    drained open-loop through FK only (no more rendering). Returns
    JSON only; no Volume writes.
    """
    import os
    import sys
    sys.path.insert(0, "/root")
    import random

    import numpy as np
    import torch

    import src.vla.evaluate as E
    import src.vla.scene as S
    from src.kin.model import load_model

    os.chdir("/root")
    rng = random.Random(SENS_SEED)
    cam = S.load_camera("/root/config/sim_camera.yaml")
    doc = load_model("/root/models/robot_inferred.json")
    tdoc = S.tip_doc(doc)
    rep60 = __import__("json").load(
        open("/root/runs/vla/6.0/report.json", encoding="utf-8"))
    rest_q = [float(v) for v in rep60["rest_q_deg"]]

    ckpt, run = _resolve_eval_ckpt(6000)
    if ckpt is None:
        return dumps_result({"ok": False,
                             "error": "no checkpoint found"})
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    policy = SmolVLAPolicy.from_pretrained(ckpt)
    policy.to("cuda")
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy.config, pretrained_path=ckpt,
        preprocessor_overrides={
            "device_processor": {"device": "cuda"},
            "rename_observations_processor": {
                "rename_map": RENAME_MAP},
        })

    def to_obs(img_np, q, task):
        return {"observation.images.top":
                torch.from_numpy(img_np).permute(2, 0, 1).float()
                / 255.0,
                "observation.state":
                torch.tensor(q, dtype=torch.float32),
                "task": task}

    scenes = [E.sample_scene_for_eval(rng) for _ in range(SENS_N)]
    out_scenes, holds = [], {}
    with torch.no_grad():
        for i, sc in enumerate(scenes):
            img = S.render_frame(sc, rest_q, doc, cam, None)
            out_scenes.append({"blocks": sc["blocks"],
                               "eval_target": sc["target"]})
            per_colour = {}
            for colour in SENS_COLOURS:
                task = f"Point to the {colour} block."
                policy.reset()
                batch = preprocessor(to_obs(img, rest_q, task))
                out = postprocessor(policy.select_action(batch))
                acts = (out.detach().cpu().float().tolist()
                        if torch.is_tensor(out)
                        else [list(map(float, row)) for row in out])
                first = [float(v) for v in np.asarray(acts).reshape(
                    -1, 6)[0]]
                tip, _ = S.tip_world(
                    [float(v) for v in np.asarray(acts).reshape(
                        -1, 6)[-1]], tdoc)
                per_colour[colour] = {
                    "first_action": [round(v, 3) for v in first],
                    "hold_tip": [round(float(v), 1) for v in tip]}
            holds[i] = per_colour
    return dumps_result({"ok": True, "checkpoint": ckpt, "run": run,
                         "n": SENS_N, "colours": list(SENS_COLOURS),
                         "scenes": out_scenes, "holds": holds})


@app.local_entrypoint()
def main(which: str = "train") -> None:
    import json
    import pathlib
    _check_upload_list()
    if which == "a100":
        raw = check_a100.remote()
    elif which == "env":
        raw = check_env.remote()
    elif which == "infer":
        raw = infer_trial.remote()
    elif which == "bench":
        raw = bench.remote()
    elif which == "eval":
        raw = eval_policies.remote()
    elif which == "sens":
        raw = sens_instructions.remote()
    elif which == "train-full":
        raw = train_full.remote()
    else:
        raw = train_trial.remote()
    obj = json.loads(raw)  # remote always returns a JSON str
    if which == "sens" and isinstance(obj, dict) and obj.get("ok"):
        import datetime as _dt

        from src.vla.evaluate import summarize_sens
        verdict = summarize_sens(obj)
        path = (pathlib.Path("runs") / "vla" /
                ("sens-" + _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
                 + ".json"))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(obj, indent=1), encoding="utf-8")
        print("saved: " + str(path))
        print(json.dumps(verdict, indent=1))
    else:
        print(json.dumps(obj, indent=1)[:3000])
