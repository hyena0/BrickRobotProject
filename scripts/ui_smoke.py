"""Headless-Chromium smoke test for the 5C control console.

Opens the fake-brick console page, fails on any browser console
error, checks the scene graph via window._scene, drives one mat
point (plain + gripper-down) through IK, and saves 3 screenshots.
Prints a PASS/FAIL table; exit code 0 only when all rows pass.
"""

import argparse
import json
import re
import sys

from playwright.sync_api import sync_playwright


def check_scene(page):
    """Robot meshes, gizmo, TCP dot, 9 mat dots + riser via _scene."""
    info = page.evaluate(
        """() => {
        const st = window._scene;
        if (!st) return {ready: false};
        const armKids = st.arm ? st.arm.children.length : 0;
        const dots = st.matDots ? st.matDots.children.length : 0;
        let riser = 0;
        if (st.matDots) for (const c of st.matDots.children)
            if (c.name === "riser") riser++;
        return {ready: true, armKids, dots, riser,
                gizmo: !!(st.tc && st.target),
                marker: !!st.marker, tcpDot: !!st.tcpDot};
        }""")
    opts = page.evaluate(
        "() => document.querySelectorAll('#mat-point option').length")
    return info, opts


def wait_solve(page, timeout_ms=180000):
    """Wait until this solve finishes, then return the live distance.

    #ik-live refreshes every animation frame, so a numeric distance
    is always present (possibly stale). #ik-stat is only rewritten
    when a solve completes, so wait for it to change first.
    Returns (dist_or_None, diag_text).
    """
    old = page.text_content("#ik-stat") or ""
    page.wait_for_function(
        """(prev) => {
        const s = document.getElementById('ik-stat').textContent;
        return s !== prev && s.includes('목표(테이블)');
        }""",
        arg=old, timeout=timeout_ms)
    txt = page.text_content("#ik-live") or ""
    m = re.search(r"거리\s*([\d.]+)mm", txt)
    dist = float(m.group(1)) if m else None
    return dist, ik_diag(page) if dist is None or dist > 5.0 else ""


def ik_diag(page):
    """Server-side verdict + client-side status lines for failures."""
    msg = page.text_content("#ik-msg") or ""
    stat = page.text_content("#ik-stat") or ""
    return f"msg={msg[:120]} stat={stat[:160]}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--out", default="runs/ui_smoke")
    args = ap.parse_args()

    rows = []
    errors = []
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, args=[
                "--enable-unsafe-swiftshader",
                "--disable-dev-shm-usage",
            ])
            page = browser.new_page(viewport={"width": 1400,
                                              "height": 1000})
            page.on("console", lambda m: errors.append(m.text)
                    if m.type == "error" else None)
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(f"http://127.0.0.1:{args.port}/")
            page.wait_for_function("() => !!window._scene",
                                   timeout=60000)
            page.wait_for_function(
                "() => document.querySelectorAll("
                "'#mat-point option').length > 1",
                timeout=60000)

            info, opts = check_scene(page)
            rows.append(("scene: robot meshes (arm kids>=10)",
                         info.get("armKids", 0) >= 10,
                         f"armKids={info.get('armKids')}"))
            rows.append(("scene: gizmo + marker + TCP dot",
                         bool(info.get("gizmo") and info.get("marker")
                              and info.get("tcpDot")),
                         json.dumps({k: info.get(k) for k in
                                     ("gizmo", "marker", "tcpDot")})))
            rows.append(("scene: mat 9 dots + riser box",
                         info.get("dots") == 10 and
                         info.get("riser") == 1 and opts == 9,
                         f"dots={info.get('dots')} "
                         f"riser={info.get('riser')} opts={opts}"))
            page.screenshot(path=f"{args.out}/shot-open.png")

            page.select_option("#mat-point", "1")
            page.click("#mat-go")
            d1, det1 = wait_solve(page)
            det1 = f"dist={d1}" + (f" {det1}" if det1 else "")
            rows.append(("mat 1: TCP-target<=5mm",
                         d1 is not None and d1 <= 5.0, det1))
            page.screenshot(path=f"{args.out}/shot-mat1.png")

            page.check("#grip-down")
            page.select_option("#mat-point", "2")
            page.click("#mat-go")
            d2, det2 = wait_solve(page)
            det2 = f"dist={d2}" + (f" {det2}" if det2 else "")
            rows.append(("mat 2 + grip-down: TCP-target<=5mm",
                         d2 is not None and d2 <= 5.0, det2))
            page.screenshot(path=f"{args.out}/shot-gripdown.png")
            browser.close()
    except Exception as e:  # noqa: BLE001 - table must print
        rows.append(("run to completion", False, f"{type(e).__name__}"))
        errors.append(str(e))

    rows.append(("browser console: zero errors", not errors,
                 f"errors={len(errors)}"))
    if errors:
        print("console errors (first 5):")
        for e in errors[:5]:
            print("  -", e[:300])

    width = max(len(r[0]) for r in rows)
    ok_all = True
    print(f"{'check':<{width}}  result  detail")
    for name, ok, detail in rows:
        ok_all = ok_all and bool(ok)
        print(f"{name:<{width}}  "
              f"{'PASS' if ok else 'FAIL'}  {detail}")
    print("OVERALL:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
