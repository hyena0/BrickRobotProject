"""Tests for src.gen.inventory (synthetic fixtures only, no private data)."""

import yaml

from src.gen import inventory as inv


def _catalog():
    return {
        "95658.dat": {"category": "motor", "motor": "large",
                      "evidence": "title: Electric, Motor EV3, Large; x"},
        "99455.dat": {"category": "motor", "motor": "medium",
                      "evidence": "title: Electric, Motor EV3, Medium; x"},
        "95648.dat": {"category": "sensor",
                      "evidence": "title: Electric Sensor, Touch - EV3; x"},
        "10928.dat": {"category": "gear", "teeth": 8,
                      "evidence": "title: Technic, Gear 8 Tooth; x"},
        "10929.dat": {"category": "gear", "teeth": 40,
                      "evidence": "title: Technic, Gear 40 Tooth; x"},
        "18938.dat": {"category": "turntable", "teeth": 60,
                      "evidence": "title: Technic Turntable 60 Tooth Top; x"},
        "11478.dat": {"category": "beam",
                      "evidence": "title: Technic, Liftarm Thin 1 x 5; x"},
        "11214.dat": {"category": "axle", "axle_length": 3,
                      "evidence": "title: Technic, Axle 3; x"},
        "15100.dat": {"category": "pin",
                      "evidence": "title: Technic, Pin; x"},
    }


def test_merge_and_totals():
    counts = {"95658.dat": 2, "11478.dat": 10}
    out = inv.build_inventory(counts, _catalog(), surplus={"11478.dat": 3})
    assert out["parts"]["95658.dat"]["total"] == 2
    assert out["parts"]["11478.dat"]["total"] == 13
    assert out["n_robot_parts"] == 12


def test_precious_flags():
    counts = {"95658.dat": 1, "95648.dat": 1, "10928.dat": 5,
              "10929.dat": 1, "18938.dat": 1, "11478.dat": 4}
    out = inv.build_inventory(counts, _catalog())
    precious = set(out["precious"])
    assert {"95658.dat", "95648.dat", "10929.dat", "18938.dat"} <= precious
    assert "10928.dat" not in precious  # small gear is not precious
    assert "11478.dat" not in precious


def test_summary_rollup():
    counts = {"95658.dat": 2, "99455.dat": 4, "95648.dat": 4,
              "10928.dat": 6, "11214.dat": 7}
    out = inv.build_inventory(counts, _catalog())
    assert out["summary"]["motors"] == {"large": 2, "medium": 4}
    assert out["summary"]["sensors"] == {"touch": 4}
    assert out["summary"]["gears_by_teeth"] == {8: 6}
    assert out["summary"]["axles_by_length"] == {3: 7}


def test_surplus_validation_rejects_unknown_id():
    try:
        inv.build_inventory({"11478.dat": 1}, _catalog(),
                            surplus={"nope.dat": 2})
    except ValueError as exc:
        assert "nope.dat" in str(exc)
    else:
        raise AssertionError("expected ValueError for unknown part id")


def test_overlay_merge(tmp_path):
    base = tmp_path / "parts.yaml"
    base.write_text(yaml.safe_dump(
        {"a.dat": {"category": "beam"}}), encoding="utf-8")
    merged = inv.load_catalog_merged(
        parts_yaml=base, parts_5d_yaml=tmp_path / "missing.yaml")
    assert merged == {"a.dat": {"category": "beam"}}
    overlay = tmp_path / "parts_5d.yaml"
    overlay.write_text(yaml.safe_dump(
        {"b.dat": {"category": "motor"}}), encoding="utf-8")
    merged = inv.load_catalog_merged(parts_yaml=base, parts_5d_yaml=overlay)
    assert set(merged) == {"a.dat", "b.dat"}
