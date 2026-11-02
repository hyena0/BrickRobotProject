"""Focused 9B tests: split rule, render9b determinism/defaults, recipes."""

import json
import os
import sys

sys.path.insert(0, ".")

import numpy as np


def test_assign_split_spreads_test():
    sys.path.insert(0, "scripts")
    import stage9_collect as C
    got = [C.assign_split(n) for n in range(1, 21)]
    assert got.count("test") == 5, got
    assert got.count("train") == 15, got
    assert [n for n in range(1, 21) if C.assign_split(n) == "test"] == \
        [4, 8, 12, 16, 20]


def test_render_frame_default_unchanged():
    import src.vla.scene as S
    assert S.PALETTE9B is None and S.PAPER9B is None and S.DR9B is False
    assert S.COLORS["red"] == (0.85, 0.15, 0.12)
    from src.kin.model import load_model
    doc = load_model("models/robot_inferred.json")
    cam = S.load_camera()
    scene = {"blocks": [{"colour": "red", "x": -120.0, "y": 150.0,
                         "yaw_deg": 0.0, "slot": "S2"}]}
    rep = json.load(open("runs/vla/6.0/report.json", encoding="utf-8"))
    rest = [float(v) for v in rep["rest_q_deg"]]
    a = S.render_frame(scene, rest, doc, cam, None)
    S.PALETTE9B = dict(S.PALETTE9B_MEASURED)
    b = S.render_frame(scene, rest, doc, cam, None)
    S.PALETTE9B = None
    assert a.shape == (256, 256, 3)
    assert not np.array_equal(a, b)  # override path is actually used
    assert np.array_equal(a, S.render_frame(scene, rest, doc, cam, None))


def test_render9b_deterministic_and_shaped():
    import src.vla.scene as S
    from src.kin.model import load_model
    doc = load_model("models/robot_inferred.json")
    cam = S.load_camera()
    scene = {"blocks": [{"colour": "blue", "x": -120.0, "y": 230.0,
                         "yaw_deg": 0.0, "slot": "S4"}]}
    rep = json.load(open("runs/vla/6.0/report.json", encoding="utf-8"))
    rest = [float(v) for v in rep["rest_q_deg"]]
    a = S.render9b(scene, rest, doc, cam, 7)
    b = S.render9b(scene, rest, doc, cam, 7)
    c = S.render9b(scene, rest, doc, cam, 8)
    assert a.shape == (256, 256, 3) and a.dtype == np.uint8
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


class _StubPolicy:
    def __init__(self, action):
        self.action = action
        self.resets = 0
        self.calls = 0

    def reset(self):
        self.resets += 1

    def select_action(self, batch):
        self.calls += 1
        import torch
        assert batch["observation.state"].shape == (6,)
        assert batch["observation.images.top"].shape == (3, 256, 256)
        return torch.tensor([self.action])


def _stub_pair():
    return (lambda obs: obs), (lambda out: out)


def _img():
    import numpy as np
    # Read-only like real pipeline arrays (PIL/raster outputs); drain
    # must copy before torch.from_numpy instead of warning.
    arr = np.zeros((256, 256, 3), np.uint8)
    arr.setflags(write=False)
    return arr


def test_drain_returns_50x6_and_resets_once_per_scene():
    import src.vla.drain as DR
    pre, post = _stub_pair()
    pol = _StubPolicy([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    stacked, info = DR.drain_chunk(pol, pre, post, _img(),
                                   [0.0] * 6, "Point to the red block.",
                                   state_source="command")
    assert stacked.shape == (50, 6)
    assert (stacked[0] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).all()
    assert pol.resets == 1 and pol.calls == 50
    assert info == {"state_source": "command", "n": 50}


def test_drain_rejects_non_50x6_result():
    import pytest
    import src.vla.drain as DR
    pre, post = _stub_pair()
    pol = _StubPolicy([[1.0] * 6] * 6)  # 6x6, not a single action
    with pytest.raises(ValueError, match="must have 6 values"):
        DR.drain_chunk(pol, pre, post, _img(), [0.0] * 6, "t",
                       state_source="command", n=3)
    pol2 = _StubPolicy([1.0] * 6)
    with pytest.raises(ValueError, match="q0 must be 6 joints"):
        DR.drain_chunk(pol2, pre, post, _img(), [0.0] * 5, "t",
                       state_source="command", n=3)
    with pytest.raises(ValueError, match="mujoco state needs doc"):
        DR.drain_chunk(pol2, pre, post, _img(), [0.0] * 6, "t",
                       state_source="mujoco", n=3)


def test_drain_falls_back_to_command_when_mujoco_missing(monkeypatch):
    import sys
    import pytest
    import src.vla.drain as DR
    monkeypatch.setitem(sys.modules, "mujoco", None)
    pre, post = _stub_pair()
    stacked, info = DR.drain_chunk(_StubPolicy([1.0] * 6), pre, post,
                                   _img(), [0.0] * 6, "t",
                                   state_source="mujoco", n=5,
                                   doc={"ids": ["J1"]})
    assert stacked.shape == (5, 6)
    assert info["state_source"] == "command"
    assert info["state_fallback"] == "mujoco-unavailable"
    with pytest.raises(ImportError):
        DR.drain_chunk(_StubPolicy([1.0] * 6), pre, post, _img(),
                       [0.0] * 6, "t", state_source="mujoco", n=5,
                       doc={"ids": ["J1"]},
                       render_fn=lambda q: _img())


def test_drain_mujoco_matches_command_state():
    import numpy as np
    import src.vla.drain as DR
    from src.kin.model import load_model
    doc = load_model("models/robot_inferred.json")
    pre, post = _stub_pair()
    q0 = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    s_c, _ = DR.drain_chunk(_StubPolicy(q0), pre, post, _img(), q0,
                            "t", state_source="command", n=5, doc=doc)
    s_m, info = DR.drain_chunk(_StubPolicy(q0), pre, post, _img(), q0,
                               "t", state_source="mujoco", n=5, doc=doc)
    assert info["state_source"] == "mujoco"
    assert s_c.shape == s_m.shape == (5, 6)
    assert bool((np.abs(s_c - s_m) < 1e-9).all())


def _make_real_fixture(root):
    import json as _json
    import os as _os
    import numpy as _np
    from PIL import Image as _Image
    specs = {1: ("train", "red", "S2"),
             4: ("test", "blue", "S1"),
             21: ("holdout", "green", "S5")}
    _os.makedirs(root, exist_ok=True)
    man = []
    for no, (split, colour, slot) in specs.items():
        photo = "batch_%02d.png" % no
        _Image.fromarray(_np.zeros((8, 8, 3), _np.uint8) + no).save(
            _os.path.join(root, photo))
        tj_rel = ("batch_%02d_traj.json" % no if split == "test"
                  else _os.path.join(split, "batch_%02d_traj.json" % no))
        if split != "test":
            _os.makedirs(_os.path.join(root, split), exist_ok=True)
        tj = {"samples": [{"colour": colour,
                           "task": "Point to the %s block." % colour,
                           "slot": slot,
                           "traj": [[float(no)] * 6] * 50}]}
        _json.dump(tj, open(_os.path.join(root, tj_rel), "w"))
        man.append({"batch": no, "split": split,
                    "slots": {slot: colour}, "photo": photo,
                    "traj_file": tj_rel})
    with open(_os.path.join(root, "manifest.jsonl"), "w") as fh:
        for rec in man:
            fh.write(_json.dumps(rec) + "\n")


def test_holdout_never_enters_training_but_eval_selects_it():
    import shutil
    import sys
    sys.path.insert(0, "scripts")
    import stage9_collect as C
    import src.vla.build9b as B9
    import stage9_eval_real as EV
    root = "/tmp/real_fixture"
    shutil.rmtree(root, ignore_errors=True)
    _make_real_fixture(root)
    assert set(C.NEVER_TRAIN) == {"test", "holdout"}
    eps, batches = B9.load_real_train(root)
    assert batches == [1] and len(eps) == 1
    assert [b["batch"] for b in EV.load_test_samples(root)] == [4]
    got = EV.load_test_samples(root, ("test", "holdout"))
    assert sorted(b["batch"] for b in got) == [4, 21]


def test_write_report_round_trips_stdout_bytes():
    import json as _json
    import sys as _sys
    _sys.path.insert(0, "scripts")
    import stage9_eval_real as _EV
    res = {"batches": [4], "splits": ["test"], "n_samples": 3,
           "before": {"n": 3}, "after": {"n": 3},
           "random": {"n": 3}}
    path = "/tmp/eval_real_probe.json"
    assert _EV.write_report(res, path) == path
    back = _json.load(open(path, encoding="utf-8"))
    assert back == res


def test_stamp_outdir_is_fresh_and_timestamped():
    import re
    import src.vla.build9b as B9
    d = B9.stamp_outdir()
    assert re.fullmatch(r"runs/stage9/lerobot9b-\d{8}-\d{6}", d), d


def test_fast_encoder_is_h264():
    import src.vla.build9b as B9
    enc = B9.fast_encoder()
    assert enc.vcodec == "h264" and enc.crf == "23"


def test_local_convert_two_episodes():
    import shutil
    import src.vla.build9b as B9
    out = "/tmp/ep9b_convert"
    shutil.rmtree(out, ignore_errors=True)
    recs, _ = B9.plan_recipes(n=2, seed=99)
    B9.convert9b(out, recs, "runs/stage9/real", repeat=0,
                 progress_every=0)
    rep = json.load(open(os.path.join(out, "report9b.json"),
                         encoding="utf-8"))
    assert rep["virtual_episodes"] == 2 and rep["real_episodes"] == 0
    assert rep["total"] == 2
    assert os.path.exists(os.path.join(
        out, "videos", "observation.images.top",
        "chunk-000", "file-000.mp4"))


def test_recipes_schema_and_gates():
    import src.vla.build9b as B9
    out = "/tmp/ep9b_test"
    B9.gen_recipes(out, n=8, seed=1234, render_n=2)
    rep = json.load(open(os.path.join(out, "report.json"),
                         encoding="utf-8"))
    assert rep["pass"] == "8/8" and rep["range_violations"] == 0
    assert rep["colour_share"] == {"blue": 25.0, "green": 25.0,
                                   "red": 25.0, "yellow": 25.0}
    recs = json.load(open(os.path.join(out, "recipes.json"),
                          encoding="utf-8"))
    ok = [r for r in recs if r.get("status") == "pass"]
    assert len(ok) == 8
    for r in ok:
        assert len(r["traj"]) == 50 and len(r["traj"][0]) == 6
        assert r["seed_ep"] and r["task"] and r["cam"]
    assert os.path.exists(os.path.join(out, "grid_3x3.png"))
