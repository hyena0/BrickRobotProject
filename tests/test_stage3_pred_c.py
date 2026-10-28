"""Stage 3 pred-C tests: pure helpers on synthetic fixtures.

Synthetic only. The real model (runs/, assets/private/) and ground
truth (eval/ground_truth/) are never read here.
"""

from __future__ import annotations

import pytest

from src.tools.pred_c import (WRIST_F_OUT, WRIST_STAGES, chain_product,
                              normalize_columns, rolling_contact_sign,
                              sign_flip_check)


def test_rolling_contact_external_is_minus_one():
    # Parallel spur pair: contact velocities oppose -> w_b = -w_a.
    s = rolling_contact_sign([0, 0, 1], [6.0, 0, 0],
                             [0, 0, 1], [-6.0, 0, 0])
    assert s == -1


def test_rolling_contact_internal_is_plus_one():
    # Internal ring pair: contact velocities align -> w_b = +w_a.
    s = rolling_contact_sign([0, 0, 1], [6.0, 0, 0],
                             [0, 0, 1], [6.0, 0, 0])
    assert s == 1


def test_rolling_contact_mitre_bevel_is_minus_one():
    # Equal mitre bevel (perpendicular shafts): rolling contact
    # reverses the sense; contact-point velocities oppose.
    s = rolling_contact_sign([0.7071, -0.7071, 0.0],
                             [10.34, -10.34, 6.0],
                             [0, 0, 1],
                             [4.24, -4.24, 13.2])
    assert s == -1


def test_normalize_columns_self_plus_one():
    c_raw = [[-0.0857143, 0.0],
             [0.0, -0.0342857]]
    c, notes = normalize_columns(c_raw, {0: 0, 1: 1})
    assert c[0][0] == pytest.approx(1.0)
    assert c[1][1] == pytest.approx(1.0)
    assert c[1][0] == pytest.approx(0.0)
    assert all(n["scaled"] for n in notes)


def test_normalize_columns_raw_passthrough():
    c_raw = [[0.24], [-0.048]]
    c, notes = normalize_columns(c_raw, {0: None})
    assert c == c_raw
    assert notes[0]["scaled"] is False


def test_sign_flip_invariant_passes():
    c = [[1.0, 0.0, 0.5],
         [0.0, 1.0, -0.3],
         [0.0, 0.0, 1.0]]
    records = sign_flip_check(c, {0: 0, 1: 1, 2: 2})
    assert len(records) == 3
    assert all(r["pass"] for r in records)


def test_sign_flip_self_cell_preserved():
    # (-1)(+1)(-1) = +1: the self cell survives its own flip.
    c = [[1.0, 0.2], [-0.4, 1.0]]
    records = sign_flip_check(c, {0: 0, 1: 1})
    assert all(r["pass"] for r in records)


def test_sign_flip_fails_without_unit_self():
    c = [[0.5, 0.2], [-0.4, 1.0]]
    records = sign_flip_check(c, {0: 0, 1: 1})
    assert records[0]["pass"] is False
    assert records[1]["pass"] is True


def test_sign_flip_row_only_without_drive_column():
    c = [[1.0, 0.24], [0.0, -0.048]]
    records = sign_flip_check(c, {0: 0, 1: None})
    assert records[1]["mode"].startswith("row-only")
    assert records[1]["pass"] is True


def test_chain_product_multiplies_signed_stages():
    # Raw k4 stage product is -0.2; the R17 +0.2 needs the J4
    # frame flip (WRIST_F_OUT), applied by the builder.
    assert chain_product([+16.0 / 16.0, -16.0 / 16.0, -1.0,
                          -12.0 / 60.0]) == pytest.approx(-0.2)
    assert chain_product([]) == pytest.approx(1.0)


def test_wrist_coefficients_from_evidenced_stages():
    k4 = WRIST_F_OUT["J4"] * chain_product(
        s for _, s in WRIST_STAGES["k4_737_J4"])
    k5 = WRIST_F_OUT["J5"] * chain_product(
        s for _, s in WRIST_STAGES["k5_736_J5"])
    k6 = WRIST_F_OUT["J6"] * chain_product(
        s for _, s in WRIST_STAGES["k6_736_J6"])
    assert k4 == pytest.approx(0.2)
    assert k5 == pytest.approx(-0.2)
    assert k6 == pytest.approx(-1.0 / 7.0)
    # 736 column normalizes by J5: J6/J5 co-motion is +5/7.
    assert (k6 / k5) == pytest.approx(5.0 / 7.0)
