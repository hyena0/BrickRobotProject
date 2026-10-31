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
