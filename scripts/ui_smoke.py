"""Headless-Chromium smoke test for the 5C control console.

Covers all 3 pages (main / calibration / verify) on fake bricks:
fails on any browser console error, checks the scene graph via
window._scene, drives one mat point through IK, checks the
preview -> blocked -> home -> send -> actual-refresh flow on the
main page, and saves screenshots. Prints a PASS/FAIL table;
exit code 0 only when all rows pass.
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


def _dist_from(txt):
    m = re.search(r"거리\s*([\d.]+)mm", txt or "")
    return float(m.group(1)) if m else None


def wait_solve(page, timeout_ms=180000):
    """Wait until this solve finishes.

    #ik-live refreshes every animation frame, so a numeric distance
    is always present (possibly stale). #ik-stat is only rewritten
    when a solve completes, so wait for it to change first.
    Since the preview flow, the arm (and #ik-live) only follows the
    solution after send; right after a solve, #preview-dist carries
    the solution-vs-target agreement check.
    Returns (live_dist, preview_dist, diag_text).
    """
    old = page.text_content("#ik-stat") or ""
    page.wait_for_function(
        """(prev) => {
        const s = document.getElementById('ik-stat').textContent;
        return s !== prev && s.includes('목표(테이블)');
        }""",
        arg=old, timeout=timeout_ms)
    live = _dist_from(page.text_content("#ik-live"))
    prev = _dist_from(page.text_content("#preview-dist"))
    bad = ((prev is None or prev > 5.0) and
           (live is None or live > 5.0))
    return live, prev, ik_diag(page) if bad else ""


def ik_diag(page):
    """Server-side verdict + client-side status lines for failures."""
    msg = page.text_content("#ik-msg") or ""
    stat = page.text_content("#ik-stat") or ""
    return f"msg={msg[:120]} stat={stat[:160]}"


def open_main(page, base):
    page.goto(base + "/")
    page.wait_for_function("() => !!window._scene", timeout=60000)
    page.wait_for_function(
        "() => document.querySelectorAll("
        "'#mat-point option').length > 1",
        timeout=60000)


def debug_lines_off(page):
    return page.evaluate(
        """() => { const st = window._scene; if (!st) return false;
        return ["reach", "prevLine", "gtLine", "dirLine"]
          .every((k) => st[k] && !st[k].visible); }""")


def canvas_fraction(page):
    return page.evaluate(
        """() => { const c = document.getElementById("view3d");
        const r = c.getBoundingClientRect();
        return [r.width / window.innerWidth,
                r.height / window.innerHeight]; }""")


def run_connected(page, base, args, rows):
    """Fake-brick connected server: ghost preview, homing, send."""
    open_main(page, base)
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
    mode = page.text_content("#mode") or ""
    rows.append(("mode banner shows fake-connected",
                 "가짜" in mode, mode.strip()[:60]))
    fw, fh = canvas_fraction(page)
    rows.append(("canvas fills >=80% of screen",
                 fw >= 0.8 and fh >= 0.8,
                 f"w={fw:.2f} h={fh:.2f}"))
    rows.append(("debug lines off by default",
                 debug_lines_off(page), ""))
    ov = page.evaluate(
        """() => { const a = document.getElementById("viewmenu")
        .getBoundingClientRect(); const b =
        document.getElementById("panel-right").getBoundingClientRect();
        return !(a.right < b.left || b.right < a.left ||
          a.bottom < b.top || b.bottom < a.top); }""")
    rows.append(("viewmenu: model select visible, no panel overlap",
                 page.is_visible("#vsrc") and not ov,
                 f"vsrc={page.is_visible('#vsrc')} overlap={ov}"))

    page.select_option("#mat-point", "1")
    page.click("#mat-go")
    live1, prev1, det1 = wait_solve(page)
    ghost = page.evaluate(
        "() => !!(window._scene && window._scene.ghostOn())")
    det1 = (f"preview={prev1} live={live1} ghost={ghost}" +
            (f" {det1}" if det1 else ""))
    rows.append(("mat 1: ghost preview-target<=5mm",
                 prev1 is not None and prev1 <= 5.0 and ghost,
                 det1))
    page.screenshot(path=f"{args.out}/shot-preview.png")

    ghost_shown = page.evaluate(
        """() => { const st = window._scene;
        const g = (st && st.arm) ? st.arm.children : [];
        return g.some((o) => o.visible && o.material &&
          o.material.transparent && o.material.opacity === 0.4); }""")
    send_msg = page.text_content("#send-msg") or ""
    rows.append(("ghost model shown, send blocked (unhomed)",
                 ghost_shown and "호밍" in send_msg,
                 f"ghost={ghost_shown} "
                 f"msg={send_msg.strip()[:60]}"))

    # Calibration page: home every joint through the UI.
    page.goto(base + "/calibration")
    page.wait_for_function(
        "() => document.querySelectorAll("
        "'[data-home]').length === 6",
        timeout=60000)
    for j in ["J1", "J2", "J3", "J4", "J5", "J6"]:
        page.click(f"[data-home='{j}']")
        page.wait_for_function(
            f"() => (document.getElementById("
            f"'home-msg-{j}').textContent || '')."
            f"startsWith('결과:')",
            timeout=60000)
    page.wait_for_function(
        "() => (document.getElementById('homedots') "
        ".textContent || '').includes('J6●')",
        timeout=60000)
    dots = page.text_content("#homedots") or ""
    rows.append(("cal: 6 joints homed via UI",
                 all(f"{j}●" in dots
                     for j in ["J1", "J2", "J3", "J4", "J5", "J6"]),
                 dots.strip()[:80]))

    # Back on main: solve again, send at low speed, ghost hides
    # and the body takes the preview pose.
    open_main(page, base)
    page.click("#mat-btns button >> nth=0")
    live3, prev3, det3 = wait_solve(page)
    send_msg2 = page.text_content("#send-msg") or ""
    can_send = "보내기 가능" in send_msg2
    rows.append(("send enabled after homing",
                 prev3 is not None and prev3 <= 5.0
                 and can_send,
                 f"preview={prev3} "
                 f"msg={send_msg2.strip()[:60]}"))
    joints_before = page.text_content("#joints") or ""
    page.click("#send-go")
    page.wait_for_function(
        "() => (document.getElementById('log') "
        ".textContent || '').includes('로봇으로 보내기 완료')",
        timeout=60000)
    page.wait_for_function(
        """(prev) => {
        const t = document.getElementById('joints')
            .textContent;
        return t !== prev;
        }""",
        arg=joints_before, timeout=60000)
    actual = page.evaluate(
        "() => !!(window._scene && "
        "window._scene.actualLine.visible)")
    ghost_after = page.evaluate(
        "() => !!(window._scene && window._scene.ghostOn())")
    live_after = _dist_from(page.text_content("#ik-live"))
    rows.append(("send: ghost off, arm=preview, live<=5mm",
                 actual and not ghost_after and
                 live_after is not None and live_after <= 5.0,
                 f"actual={actual} ghost={ghost_after} "
                 f"live={live_after}"))
    page.screenshot(path=f"{args.out}/shot-sent.png")

    # Verify page renders; E-STOP present on all 3 pages.
    page.goto(base + "/verify")
    page.wait_for_function(
        "() => (document.getElementById('verify-table') "
        ".textContent || '').length > 10",
        timeout=60000)
    vt = page.text_content("#verify-table") or ""
    rows.append(("verify page renders, no undefined",
                 len(vt) > 10 and "undefined" not in vt,
                 vt.strip()[:80]))
    estops = []
    for url in [base + "/", base + "/calibration",
                base + "/verify"]:
        page.goto(url)
        estops.append(page.is_visible("#estop"))
    rows.append(("E-STOP on all 3 pages", all(estops),
                 f"{estops}"))


def run_virtual(page, base, args, rows):
    """Offline (virtual-only) server: body follows IK, no send."""
    open_main(page, base)
    mode = page.text_content("#mode") or ""
    rows.append(("mode banner shows virtual-only",
                 "가상만" in mode, mode.strip()[:60]))
    rows.append(("debug lines off by default",
                 debug_lines_off(page), ""))
    send_visible = page.is_visible("#send-row")
    rows.append(("send row hidden when virtual-only",
                 not send_visible, f"visible={send_visible}"))
    joints0 = page.text_content("#joints") or ""
    page.click("#mat-btns button >> nth=0")
    live, prev, det = wait_solve(page)
    ghost = page.evaluate(
        "() => !!(window._scene && window._scene.ghostOn())")
    det = (f"preview={prev} live={live} ghost={ghost}" +
           (f" {det}" if det else ""))
    rows.append(("virtual: body follows IK, live<=5mm",
                 live is not None and live <= 5.0 and not ghost,
                 det))
    rows.append(("joint panel shows unknown encoders",
                 "?" in joints0, joints0.strip()[:60]))
    page.screenshot(path=f"{args.out}/shot-follow.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--out", default="runs/ui_smoke")
    ap.add_argument("--phase", choices=["connected", "virtual"],
                    default="connected")
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
            page.on("dialog", lambda d: d.accept())
            base = f"http://127.0.0.1:{args.port}"
            if args.phase == "connected":
                run_connected(page, base, args, rows)
            else:
                run_virtual(page, base, args, rows)
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
