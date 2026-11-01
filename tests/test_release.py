"""A few tests for the minimal release guard (rough cut)."""

from src.release.check import key_hits, private_hits, url_hits


def test_private_paths_flagged():
    dirs = ("assets/private/", "runs/", "gt/", "eval/ground_truth/")
    hits = private_hits([
        "assets/private/visual_pack/x.glb",
        "config/hw_observed.local.yaml",
        "runs/stage5c/report.json",
        "gt/kinematics_gt_v2.json",
        "eval/ground_truth/x.json",
        "models/robot_inferred.json",
        "model.io",
        "src/console/control.py",
    ], dirs=dirs)
    assert "models/robot_inferred.json" not in hits
    assert "src/console/control.py" not in hits
    assert len(hits) == 6


def test_tbot_public_renders_pass():
    from src.release.check import private_hits
    hits = private_hits([
        "assets/public_models/tbot/PERMISSION.md",
        "assets/public_models/tbot/renders/pack_home.png",
        "assets/public_models/tbot/renders/pack_posed.png",
    ])
    assert hits == []


def test_tbot_originals_and_meshes_blocked_anywhere():
    from src.release.check import private_hits
    hits = private_hits([
        "assets/public_models/tbot/tbot.io",
        "assets/public_models/tbot/tbot.ldr",
        "assets/public_models/tbot/tbot.mpd",
        "assets/public_models/tbot/D0.glb",
        "assets/private/models_ext/tbot/x.ldr",
    ])
    assert len(hits) == 5


def test_allowlist_excuses_listed_paths(tmp_path):
    from src.release import check as rc

    paths = ["tests/fixtures/gears.ldr", "src/release/check.py",
             "assets/private/x.glb"]
    assert rc.private_hits(paths, allow=frozenset(paths[:1])) == \
        ["assets/private/x.glb"]
    (tmp_path / "a.py").write_text('x = "https://cdn.x/y"\n',
                                    encoding="utf-8")
    assert rc.url_hits(tmp_path, ["a.py"],
                       allow=frozenset(["a.py"])) == []
    (tmp_path / "b.py").write_text('api_key = "[REDACTED]"\n',
                                    encoding="utf-8")
    assert rc.key_hits(tmp_path, ["b.py"],
                       allow=frozenset(["b.py"])) == []


def test_load_allowlist_covers_known_exceptions():
    import pathlib

    from src.release import check as rc

    root = pathlib.Path(__file__).resolve().parents[1]
    allow = rc.load_allowlist(root)
    for p in ("tests/fixtures/gears.ldr",
              "assets/modules/M_base_yaw/module.ldr",
              "src/release/check.py", "tests/test_release.py"):
        assert p in allow


def test_cdn_url_flagged(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    f = src / "a.py"
    f.write_text('URL = "https://cdn.jsdelivr.net/npm/x"\n',
                 encoding="utf-8")
    ok = src / "b.py"
    ok.write_text('URL = "http://127.0.0.1:8766/x"\n'
                  '# see https://github.com/example/repo\n',
                  encoding="utf-8")
    hits = url_hits(tmp_path, ["src/a.py", "src/b.py"])
    assert hits == ["src/a.py:1"]


def test_api_key_pattern_flagged(tmp_path):
    f = tmp_path / "a.py"
    f.write_text('api_key = "sk-abcdefghij123"\n', encoding="utf-8")
    ok = tmp_path / "b.py"
    ok.write_text('key = "local"\n', encoding="utf-8")
    hits = key_hits(tmp_path, ["a.py", "b.py"])
    assert hits == ["a.py:1"]
