"""6.x sim-VLA demo gates (fast, no IK solves, no rendering)."""

from __future__ import annotations

import pathlib

import pytest

from src.console.control import check_pose_against_ranges
from src.vla.demo import ik_accepted, split_legs
from src.vla.scene import JOINTS, adopted_ranges


def test_ik_acceptance_rejects_coupled_violation():
    """IK returns q_deg even when success is False (coupled).

    run_demo must not use such solutions (6.1 bug: 19 demos held the
    tip exactly on target while violating J3-J5 on the way down).
    """
    good = {"q_deg": [0.0] * 6, "success": True, "pos_err_mm": 0.5}
    bad_coupled = {"q_deg": [8.8, 30.6, -45.8, -27.6, 1.9, 60.0],
                   "success": False, "pos_err_mm": 1e-06}
    bad_none = {"q_deg": None, "success": False, "pos_err_mm": 1e9}
    assert ik_accepted(good) is True
    assert ik_accepted(bad_coupled) is False
    assert ik_accepted(bad_none) is False


def test_split_legs_sums_to_30_with_mins():
    assert split_legs(100.0, 10.0) == (27, 3)
    assert split_legs(10.0, 100.0) == (3, 27)
    assert split_legs(0.0, 0.0) == (27, 3)
    n1, n2 = split_legs(50.0, 50.0)
    assert (n1, n2) == (15, 15)
    # infeasible travel still sums to 30 (demo then fails the 5deg gate)
    n1, n2 = split_legs(140.0, 10.0)
    assert n1 + n2 == 30 and n1 >= 3 and n2 >= 3


def test_trial_results_are_json_safe():
    """Remote returns must not need torch (or anything) locally.

    Regression: Modal unpickles remote dicts locally, so a torch.Size /
    dtype / Tensor inside failed with DeserializationError where torch
    is missing. All remote functions return a JSON str now.
    """
    import json

    from src.vla.trial_modal import dumps_result

    class FakeTensor:
        """torch.Tensor stand-in: tolist, nothing else."""

        def __init__(self, vals):
            self._v = list(vals)

        def tolist(self):
            return list(self._v)

    class FakeSize(tuple):
        """torch.Size is a tuple subclass."""

    class FakeDtype:
        def __str__(self):
            return "torch.float32"

    payload = {"ok": True,
               "python": "3.12.1",
               "torch": "2.7.0",
               "action_shape": FakeSize((1, 50, 6)),
               "action_head": [FakeTensor([0.5]), 0.25, 1],
               "dtype": FakeDtype(),
               "nested": [{"s": {1, 2}, "b": b"xy"}],
               "missing": None}
    s = dumps_result(payload)
    assert isinstance(s, str)
    back = json.loads(s)

    def check(x):
        assert isinstance(x, (dict, list, str, int, float, bool)) \
            or x is None
        if isinstance(x, dict):
            for k, v in x.items():
                assert isinstance(k, str)
                check(v)
        elif isinstance(x, list):
            for v in x:
                check(v)

    check(back)
    assert back["action_shape"] == [1, 50, 6]
    assert back["dtype"] == "torch.float32"


def test_trial_camera_fix_matches_base():
    """Single-camera fix must match smolvla_base's three streams.

    Base expects observation.images.camera{1,2,3}; the dataset has one
    top camera. rename_map moves it to camera1, empty_cameras=2 pads the
    other two with masked zeros (prepare_images design).
    """
    from src.vla.trial_modal import EMPTY_CAMERAS, RENAME_MAP
    assert RENAME_MAP == {
        "observation.images.top": "observation.images.camera1"}
    assert EMPTY_CAMERAS == 2
    assert set(RENAME_MAP.values()) <= {
        "observation.images.camera1", "observation.images.camera2",
        "observation.images.camera3"}


def test_latest_ckpt_picks_newest_on_volume():
    import os

    from src.vla.trial_modal import _latest_ckpt
    import tempfile

    with tempfile.TemporaryDirectory() as d:
        assert _latest_ckpt(d) is None
        for step in ("000100", "000200"):
            p = os.path.join(d, "checkpoints", step, "pretrained_model")
            os.makedirs(p)
        got = _latest_ckpt(d)
        assert got is not None and got.endswith(
            os.path.join("checkpoints", "000200", "pretrained_model"))


def test_summarize_counts_gates():
    from src.vla.to_lerobot import summarize

    recs = [{"status": "pass", "target": "red",
             "range_violations": 0, "max_step_deg": 3.0,
             "hold_max_h_err_mm": 0.5},
            {"status": "pass", "target": "blue",
             "range_violations": 0, "max_step_deg": 4.0,
             "hold_max_h_err_mm": 1.0},
            {"status": "task-failed", "target": "red",
             "range_violations": 5, "max_step_deg": 9.0,
             "hold_max_h_err_mm": 99.0}]
    rep = summarize(recs)
    assert rep["pass"] == "2/3"
    assert rep["range_violations"] == 5
    assert rep["colour_share"] == {"blue": 50.0, "red": 50.0}
    assert rep["max_step_max"] == 4.0
    assert rep["hold_herr_max_mm"] == 1.0


def test_score_episode_success_wrong_miss():
    from src.vla.evaluate import score_episode

    blocks = [{"colour": "red", "x": -120.0, "y": 230.0},
              {"colour": "blue", "x": -200.0, "y": 230.0}]
    assert score_episode([-120.0, 230.0, 60.0], blocks,
                         "red")["success"] is True
    r = score_episode([-200.0, 230.0, 60.0], blocks, "red")
    assert r["success"] is False and r["wrong_block"] is True
    r = score_episode([0.0, 0.0, 60.0], blocks, "red")
    assert r["success"] is False and r["wrong_block"] is False


def test_rollout_static_predictor_reaches_hold():
    import random

    from src.vla import evaluate as E
    from src.vla.scene import load_camera, load_model, tip_doc

    doc = load_model("models/robot_inferred.json")
    rng = random.Random(0)
    scene = E.sample_scene_for_eval(rng)
    rest = [0.0] * 6
    out = E.rollout(scene, rest, doc, load_camera(),
                    lambda t, q, img: list(rest),
                    render_each=False)
    assert len(out["traj"]) == 50 and len(out["tips"]) == 50
    assert len(out["frames"]) == 1


def test_resolve_eval_ckpt_picks_below_want():
    import os
    import tempfile

    from src.vla.trial_modal import _resolve_eval_ckpt

    with tempfile.TemporaryDirectory() as d:
        run = os.path.join(d, "train-20260101-000000")
        for step in ("000100", "003000", "006000"):
            os.makedirs(os.path.join(run, "checkpoints", step,
                                      "pretrained_model"))
        ckpt, got_run = _resolve_eval_ckpt(3000, root=d)
        assert ckpt is not None and ckpt.endswith(
            os.path.join("003000", "pretrained_model"))
        assert got_run == run
        ckpt6, _ = _resolve_eval_ckpt(6000, root=d)
        assert ckpt6 is not None and ckpt6.endswith(
            os.path.join("006000", "pretrained_model"))
        assert _resolve_eval_ckpt(100, root=d + "-absent") == (None,
                                                              None)


def test_run_dir_is_fresh_timestamp():
    import re

    from src.vla.trial_modal import _run_dir
    d = _run_dir("train")
    assert re.fullmatch(r"/data/train-\d{8}-\d{6}", d)


def test_prepare_run_dirs_never_precreates_output():
    """The FileExistsError regression: out_dir must not exist when
    lerobot-train starts; only logs/ is created, elsewhere."""
    import os
    import tempfile

    from src.vla.trial_modal import _prepare_run_dirs, _unique_dir

    with tempfile.TemporaryDirectory() as d:
        base = os.path.join(d, "train-20260101-000000")
        assert _unique_dir(base) == base
        os.makedirs(base)
        assert _unique_dir(base) == base + "-2"
        out, log = _prepare_run_dirs("train", root=d)
        assert not os.path.exists(out)
        assert os.path.isdir(os.path.join(d, "logs"))
        assert log == os.path.join(
            d, "logs", os.path.basename(out) + ".log")
        assert os.path.dirname(log) != out


def test_full_data_gate_counts_episodes():
    import json
    import os
    import tempfile

    from src.vla import trial_modal as tm

    with tempfile.TemporaryDirectory() as d:
        tr = os.path.join(d, "t")
        va = os.path.join(d, "v")
        for root, n in ((tr, 360), (va, 39)):
            os.makedirs(os.path.join(root, "meta"))
            json.dump({"total_episodes": n},
                      open(os.path.join(root, "meta", "info.json"),
                           "w"))
        old_tr, old_va = tm.TRAIN_DATA_DIR, tm.VAL_DATA_DIR
        tm.TRAIN_DATA_DIR, tm.VAL_DATA_DIR = tr, va
        try:
            ok, detail = tm._check_full_data()
            assert ok is False
            assert detail["val"] == {"episodes": 39, "want": 40,
                                     "reason": "count-mismatch"}
            assert detail["train"]["reason"] == "ok"
            tm.VAL_DATA_DIR = os.path.join(d, "absent")
            ok2, detail2 = tm._check_full_data()
            assert ok2 is False
            assert detail2["val"]["reason"] == "missing-path"
        finally:
            tm.TRAIN_DATA_DIR, tm.VAL_DATA_DIR = old_tr, old_va


def test_trial_and_full_configs_are_separate():
    from src.vla.trial_modal import (BENCH_CFG, FULL_CFG, TRIAL_CFG,
                                     TRAIN_DATA_DIR)

    assert TRIAL_CFG["dataset_root"] != FULL_CFG["dataset_root"]
    assert FULL_CFG["dataset_root"] == TRAIN_DATA_DIR
    assert FULL_CFG["steps"] == 6000
    assert FULL_CFG["batch_size"] == 32
    assert FULL_CFG["proc_timeout_s"] == 13800  # 3h50m < 4h fn timeout
    assert BENCH_CFG["steps"] == 50


def test_trial_env_covers_train_imports():
    """check_env must import everything train needs.

    Regression: train died on `import datasets` after a passing env.
    The module list is the contract; check_env iterates it.
    """
    from src.vla.trial_modal import TRAIN_MODULES
    for mod in ("lerobot.datasets",
                "lerobot.policies.smolvla.modeling_smolvla",
                "lerobot.scripts.lerobot_train"):
        assert mod in TRAIN_MODULES


def test_slot_ids_cover_lattice():
    from src.vla.scene import LATTICE_SLOTS, SLOT_IDS
    assert sorted(SLOT_IDS.values()) == ["S1", "S2", "S3",
                                        "S4", "S5", "S6"]
    assert set(SLOT_IDS) == set(LATTICE_SLOTS)
    # front row (near robot) first: S1 left, S2 right
    assert SLOT_IDS[(-200.0, 150.0)] == "S1"
    assert SLOT_IDS[(-120.0, 310.0)] == "S6"


def test_known_violating_pose_is_flagged():
    """The 6.1 J3-J5 dip is caught by the shared range function."""
    adopted = adopted_ranges()
    q = {j: v for j, v in zip(JOINTS, [8.8, 30.6, -45.8, -27.6, 1.9, 60.0])}
    assert check_pose_against_ranges(q, adopted) != ""


def test_slot_seeds_are_public_and_cover_slots():
    import json

    from src.vla.scene import (SLOT_IDS, SLOT_SEEDS, load_slot_seeds,
                               nearest_mat_seed)
    assert not SLOT_SEEDS.startswith("assets/private")
    rows = load_slot_seeds()
    assert sorted(r["slot"] for r in rows) == sorted(SLOT_IDS.values())
    seeds = [r["seed"] for r in rows]
    for (x, y), sid in SLOT_IDS.items():
        got = nearest_mat_seed(x, y, seeds, rows)
        assert len(got) == 6


def test_upload_guard_blocks_private_marker(tmp_path):
    import pytest

    from src.vla.trial_modal import _check_upload_list
    good = tmp_path / "pub"
    good.mkdir()
    (good / "a.json").write_text("{}", encoding="utf-8")
    _check_upload_list([str(good)])
    bad = tmp_path / "mix"
    bad.mkdir()
    (bad / "x.local.yaml").write_text("{}", encoding="utf-8")
    with pytest.raises(RuntimeError, match="private upload blocked"):
        _check_upload_list([str(good), str(bad)])
    with pytest.raises(RuntimeError, match="missing"):
        _check_upload_list([str(tmp_path / "absent")])


def test_vla_sources_pass_pyflakes():
    """F821/F401 gate: Modal functions define their own imports, so
    an undefined name fails only on the remote side. Run pyflakes
    locally before handing over any run command."""
    import subprocess
    import sys

    try:
        import pyflakes  # noqa: F401 - presence check only
    except ImportError:
        pytest.skip("pyflakes not installed")
    root = pathlib.Path(__file__).resolve().parents[1]
    files = [str(root / "src" / "vla" / name) for name in
             ("trial_modal.py", "evaluate.py", "scene.py", "demo.py",
              "to_lerobot.py", "twin.py", "local_infer.py")]
    proc = subprocess.run(
        [sys.executable, "-m", "pyflakes", *files],
        capture_output=True, text=True, cwd=str(root))
    assert proc.returncode == 0, proc.stdout


def test_summarize_sens_verdicts():
    from src.vla.evaluate import summarize_sens

    blocks = [{"colour": "red", "x": 0.0, "y": 0.0},
              {"colour": "blue", "x": 200.0, "y": 0.0}]
    base = {"colours": ["red", "blue"],
            "scenes": [{"blocks": blocks, "eval_target": "red"}]}
    same = dict(base, holds={"0": {
        "red": {"hold_tip": [1.0, 0.0, 60.0]},
        "blue": {"hold_tip": [1.0, 0.0, 60.0]}}})
    assert summarize_sens(same)["verdict"] == "ignored"
    moved = dict(base, holds={"0": {
        "red": {"hold_tip": [100.0, 0.0, 60.0]},
        "blue": {"hold_tip": [150.0, 0.0, 60.0]}}})
    assert summarize_sens(moved)["verdict"] == "weak"
    follows = dict(base, holds={"0": {
        "red": {"hold_tip": [1.0, 0.0, 60.0]},
        "blue": {"hold_tip": [200.0, 0.0, 60.0]}}})
    assert summarize_sens(follows)["verdict"] == "follows"


def test_vla_camera_info_reports_device_state():
    """Panel device line: number + open/resolution/fps/backend or error.

    Must never raise headless (no camera, no permission, no OpenCV):
    the closed state is data the panel shows, not a crash.
    """
    from src.console.control import vla_camera_info

    info = vla_camera_info()
    assert info["ok"] is True
    assert isinstance(info["camera"], int)
    assert isinstance(info["fake"], list)
    dev = info["dev"]
    for key in ("open", "width", "height", "fps", "backend", "error"):
        assert key in dev, key
    assert isinstance(dev["open"], bool)
    if dev["open"]:
        assert dev["width"] > 0 and dev["height"] > 0
    else:
        assert dev["error"]
