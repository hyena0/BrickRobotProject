"""Stage 5D.6: visual materials (local convert + Modal batch renders).

Pipeline: robot.ldr -> local Blender model.blend + model.glb (link
colours, joint axes, articulation empties) -> .glb + render_job.json
uploaded to a Modal Volume -> one Modal call per (pose, view) on GPU
Cycles -> PNGs downloaded to generated_robots/<name>/renders/.

Only generated-robot assets ever leave the Mac (denied paths fail
loudly). Local Blender runs behind scripts/heavy_guard.sh, the
/tmp/metahack-heavy.lock flock and low priority (nice, 2 threads).

Dual entry points: this file is imported as a module (CLI, Modal app
definition) and executed by headless Blender (-P ... -- --blender-inner
...). The Modal render script is built by build_inner_script() from the
tested pure functions below, so shipped math has a single source.
"""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import inspect
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

try:
    import modal

    _MODAL_OK = True
except ImportError:  # Blender-bundled Python has no modal; inner is modal-free
    modal = None  # type: ignore[assignment]
    _MODAL_OK = False

BLENDER_VERSION = "5.2.2"  # pinned to the local Blender (verified 2026-10-30)
BLENDER_URL = ("https://download.blender.org/release/Blender5.2/"
               f"blender-{BLENDER_VERSION}-linux-x64.tar.xz")
BLENDER_CONTAINER_BIN = "/opt/blender/blender"
BLENDER_MAC_BIN = ("/Applications/Blender.app/Contents/MacOS/Blender")
MODAL_VOLUME = "metahack-5d-renders"
MODAL_GPU = "L4"
MODAL_TIMEOUT_SECS = 20 * 60
HEAVY_LOCK = "/tmp/metahack-heavy.lock"
STUDIO_LDRAW = "/Applications/Studio 2.0/ldraw"
SKILL_SCRIPTS = ("/Users/kunwoo.chang/.local/share/muse/skills/bundled/"
                 "muse-core/skills/blender/scripts")
GUARD_SCRIPT = "scripts/heavy_guard.sh"
DEFAULT_LOG = "docs/stage/English/LOG.md"

# NOTE: "eval/" covers the ground-truth tree (GT isolation test forbids
# naming GT locations in code outside src/verify/; docstrings may note it).
DENY_SUBSTRINGS = ("assets/private", "eval/", "runs/", "3D models",
                   "gt/")
DENY_PATTERNS = ("config/*.local.yaml", "*/config/*.local.yaml")
RIGID_LINK_TYPES = frozenset({"axle_in_axle_hole", "pin_friction", "stud"})
LINK_COLOURS = ["#8c8c8c", "#e41a1c", "#377eb8", "#ffcc00", "#4daf4a",
                "#984ea3", "#ff7f00", "#17becf"]
AXIS_COLOUR = "#ff2222"
MM = 0.001


# ------------------------------------------------- pure math (shipped)


def dir_from_az_el(azimuth_deg: float, elevation_deg: float):
    """Unit vector from target toward the camera (plain math)."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    return [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
            math.sin(el)]


def quat_from_axis_angle(axis, angle_rad: float):
    """Unit quaternion (w, x, y, z) for an axis-angle rotation."""
    x, y, z = (float(v) for v in axis)
    norm = math.sqrt(x * x + y * y + z * z) or 1.0
    s = math.sin(angle_rad / 2.0)
    return [math.cos(angle_rad / 2.0), x / norm * s, y / norm * s,
            z / norm * s]


def quat_apply(quat, vec):
    """Rotate vec by unit quaternion (w, x, y, z)."""
    w, x, y, z = (float(v) for v in quat)
    vx, vy, vz = (float(v) for v in vec)
    tx, ty, tz = 2.0 * (y * vz - z * vy), 2.0 * (z * vx - x * vz), \
        2.0 * (x * vy - y * vx)
    return [vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx)]


def look_quat(forward):
    """Camera quaternion looking along unit `forward` (-Z convention)."""
    fx, fy, fz = (float(v) for v in forward)
    norm = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    fx, fy, fz = fx / norm, fy / norm, fz / norm
    zx, zy, zz = -fx, -fy, -fz
    ux, uy, uz = (0.0, 0.0, 1.0)
    if abs(zx * ux + zy * uy + zz * uz) > 0.99:
        ux, uy, uz = (0.0, 1.0, 0.0)
    xx = uy * zz - uz * zy
    xy = uz * zx - ux * zz
    xz = ux * zy - uy * zx
    xn = math.sqrt(xx * xx + xy * xy + xz * xz) or 1.0
    xx, xy, xz = xx / xn, xy / xn, xz / xn
    yx, yy, yz = zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx
    tr = xx + yy + zz
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return [s / 4.0, (yz - zy) / s, (zx - xz) / s, (xy - yx) / s]
    if xx > yy and xx > zz:
        s = math.sqrt(1.0 + xx - yy - zz) * 2.0
        return [(yz - zy) / s, s / 4.0, (xy + yx) / s, (xz + zx) / s]
    if yy > zz:
        s = math.sqrt(1.0 + yy - xx - zz) * 2.0
        return [(zx - xz) / s, (xy + yx) / s, s / 4.0, (yz + zy) / s]
    s = math.sqrt(1.0 + zz - xx - yy) * 2.0
    return [(xy - yx) / s, (xz + zx) / s, (yz + zy) / s, s / 4.0]


# -------------------------------------------------- link split (local)


def _coaxial_gears(parts: list[dict], catalog: dict, scope: set[str],
                   joint: dict) -> list[str]:
    """Gear uids in ``scope`` coaxial with a world-frame joint."""
    from src.tools.geometry import axis_world

    axis = [float(v) for v in joint["axis"]]
    norm = math.sqrt(sum(v * v for v in axis)) or 1.0
    axis = [v / norm for v in axis]
    point = [float(v) for v in joint["point_mm"]]
    out = []
    for part in parts:
        if part["uid"] not in scope:
            continue
        if not (catalog.get(part["part_id"]) or {}).get("teeth"):
            continue
        axes = axis_world(part, catalog)
        if axes is None:
            continue
        unit = [float(v) for v in axes]
        unorm = math.sqrt(sum(v * v for v in unit)) or 1.0
        unit = [v / unorm for v in unit]
        if abs(sum(a * b for a, b in zip(unit, axis))) < 0.99:
            continue
        rel = [part["position_mm"][i] - point[i] for i in range(3)]
        along = sum(r * u for r, u in zip(rel, unit))
        lateral = math.sqrt(max(0.0, sum(r * r for r in rel)
                                - along * along))
        if lateral <= 2.0:
            out.append(part["uid"])
    return out


def _fold_trailing_beam_segment(segments: dict, beams: list[dict]
                                ) -> tuple[dict, list[dict]]:
    """Fold a beam-only top segment into the previous one (pure).

    The 5D assembler tags the end beam one segment past the last
    joint; link grouping folds it back so the flood always leaves a
    free joint. Folding applies only when the top segment holds
    nothing but gap beams; module-bearing top segments keep their
    articulation split. Graph nodes are unaffected (rigid groups
    already span segments; link_of only splits them).
    """
    top = max(int(v) for v in segments.values())
    beam_uids = {b["uid"] for b in beams}
    top_uids = [u for u, s in segments.items() if int(s) == top]
    if not top_uids or any(u not in beam_uids for u in top_uids):
        return dict(segments), [dict(b) for b in beams]
    remapped = {u: (int(s) if int(s) < top else top - 1)
                for u, s in segments.items()}
    folded = [dict(b, segment=(b["segment"] if b["segment"] < top
                               else top - 1)) for b in beams]
    return remapped, folded


def split_links(parts: list[dict], catalog: dict, segments: dict,
                beams: list[dict],
                joints: list[dict] | None = None) -> dict:
    """Group part uids into articulation links L0..Ln.

    Floods rigid connections from each beam to find the rotor side of
    every module (rotor + beam + next statics form a link). When
    ``joints`` (world-frame, joint i in segment i) is given, gears
    coaxial with the joint also seed the flood: the output train is
    the rotor even where the beam meets the module in a butt joint.
    Returns {"links": {link_id: [uids]}, "warnings": [...]}. Raises
    ValueError when a module shows no free joint (flood leaks
    everywhere).
    """
    from src.tools.geometry import find_connections

    by_uid = {p["uid"]: p for p in parts}
    beam_of_seg = {}
    for beam in beams:
        beam_of_seg.setdefault(beam["segment"], []).append(beam["uid"])
    seg_uids: dict[int, list[str]] = {}
    for uid, seg in segments.items():
        if uid in by_uid:
            seg_uids.setdefault(int(seg), []).append(uid)
    n = len(seg_uids)
    rotor: dict[int, set[str]] = {}
    static: dict[int, set[str]] = {}
    for seg in sorted(seg_uids):
        scope = set(seg_uids[seg])
        seeds = [u for u in beam_of_seg.get(seg, []) if u in scope]
        if joints is not None and seg < len(joints):
            seeds += [u for u in _coaxial_gears(parts, catalog, scope,
                                                joints[seg])
                      if u in scope and u not in seeds]
        if not seeds:
            raise ValueError(f"segment {seg}: no beam to flood from")
        seen, queue = set(seeds), list(seeds)
        while queue:
            uid = queue.pop()
            seen.add(uid)
            for conn in find_connections(uid, parts, catalog):
                other = conn["uid"]
                if other in scope and other not in seen \
                        and conn["connection"] in RIGID_LINK_TYPES:
                    queue.append(other)
        rotor[seg] = seen
        static[seg] = scope - seen
        if not static[seg]:
            raise ValueError(
                f"segment {seg}: flood covers the module, no free joint; "
                "check turntable/hinge connection types")
    warnings = []
    for seg, uids in rotor.items():
        for uid in sorted(uids):
            if (catalog.get(by_uid[uid]["part_id"]) or {}).get("motor"):
                warnings.append(f"segment {seg}: motor {uid} on rotor side")
    links: dict[str, list[str]] = {"L0": sorted(static[0])}
    for i in range(1, n):
        uids = set(rotor.get(i - 1, set())) | set(static.get(i, set()))
        uids |= {u for b in beams if b["segment"] == i - 1
                 for u in [b["uid"]] if u in by_uid}
        links[f"L{i}"] = sorted(uids)
    links[f"L{n}"] = sorted(set(rotor.get(n - 1, set())) | {
        u for b in beams if b["segment"] == n - 1
        for u in [b["uid"]] if u in by_uid})
    return {"links": links, "warnings": warnings}


# ------------------------------------------------------- render job


def default_views() -> list[dict]:
    return [{"name": "front", "azimuth": 0.0, "elevation": 8.0},
            {"name": "side", "azimuth": 90.0, "elevation": 8.0},
            {"name": "top", "azimuth": 35.0, "elevation": 89.0}]


def default_poses(exp_doc: dict) -> list[dict]:
    """Zero pose plus per-joint +-25% range samples (deterministic)."""
    from src.kin.model import chain_order

    joints = chain_order(exp_doc)
    n = len(joints)
    poses = [{"id": "zero", "q_deg": [0.0] * n}]
    for i, joint in enumerate(joints):
        lo, hi = (float(v) for v in joint["limits_deg"])
        for tag, frac in (("p", 0.25), ("m", -0.25)):
            q = [0.0] * n
            q[i] = round((hi - lo) * frac, 2)
            poses.append({"id": f"{joint['id']}{tag}", "q_deg": q})
    return poses


def build_render_job(robot: str, poses: list[dict],
                     views: list[dict] | None = None,
                     resolution: tuple[int, int] = (1280, 960),
                     samples: int = 64) -> dict:
    return {"robot": robot, "glb": "model.glb", "engine": "CYCLES",
            "device": "GPU", "resolution": list(resolution),
            "samples": samples, "poses": poses,
            "views": views if views is not None else default_views()}


def validate_job(job: dict) -> list[str]:
    errors: list[str] = []
    for key in ("robot", "glb", "poses", "views", "resolution", "samples"):
        if key not in job:
            errors.append(f"missing key: {key}")
    if errors:
        return errors
    n = None
    for pose in job["poses"]:
        q = pose.get("q_deg")
        if not isinstance(q, list) or not q \
                or not all(isinstance(v, (int, float)) for v in q):
            errors.append(f"pose {pose.get('id')}: bad q_deg")
            continue
        if n is None:
            n = len(q)
        elif len(q) != n:
            errors.append(f"pose {pose.get('id')}: joint count differs")
    for view in job["views"]:
        for key in ("name", "azimuth", "elevation"):
            if key not in view:
                errors.append(f"view missing key: {key}")
    res = job["resolution"]
    if not isinstance(res, list) or len(res) != 2 \
            or not all(isinstance(v, int) and v > 0 for v in res):
        errors.append("resolution must be [w, h] positive ints")
    if not isinstance(job["samples"], int) or job["samples"] <= 0:
        errors.append("samples must be a positive int")
    if not job["poses"] or not job["views"]:
        errors.append("poses and views must be non-empty")
    return errors


def iter_tasks(job: dict) -> list[dict]:
    """One task per (pose, view); ids are filesystem-safe."""
    tasks = []
    for pose in job["poses"]:
        for view in job["views"]:
            tasks.append({
                "id": f"{pose['id']}_{view['name']}",
                "pose": pose, "view": view,
                "resolution": job["resolution"],
                "samples": job["samples"],
            })
    return tasks


# ------------------------------------------------- local guard rails


def check_denylist(*paths: str | Path) -> None:
    """Refuse private robot paths (they must never reach Modal)."""
    import fnmatch

    for path in paths:
        text = str(path)
        for denied in DENY_SUBSTRINGS:
            if denied in text:
                raise ValueError(f"refusing private path: {text}")
        for pattern in DENY_PATTERNS:
            if fnmatch.fnmatch(text, pattern):
                raise ValueError(f"refusing private path: {text}")


def resolve_blender_bin(override: str | None = None) -> str:
    if override:
        return override
    found = shutil.which("blender")
    if found:
        return found
    if os.path.exists(BLENDER_MAC_BIN):
        return BLENDER_MAC_BIN
    raise ValueError("no Blender binary (pass --blender-bin)")


@contextlib.contextmanager
def heavy_lock(path: str = HEAVY_LOCK):
    """Inter-process lock for heavy local work (macOS fcntl flock)."""
    print(f"acquiring {path} ...", flush=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(f"{os.getpid()}\n")
        fh.flush()
        fcntl.flock(fh.fileno(), fcntl.LOCK_EX)
        try:
            print("lock acquired", flush=True)
            yield
        finally:
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)


def run_guard(guard: str = GUARD_SCRIPT, port: int | None = None,
              log: str | None = DEFAULT_LOG) -> None:
    """Run heavy_guard.sh (wait mode); raises on busy-timeout skip."""
    cmd = [guard]
    if port is not None:
        cmd += ["--port", str(port)]
    if log is not None:
        cmd += ["--log", log]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    print(proc.stdout.strip())
    if proc.returncode == 20:
        raise RuntimeError("guard timed out; work skipped (see log)")
    if proc.returncode != 0:
        raise RuntimeError(f"guard failed: {proc.stderr.strip()[:300]}")


def blender_command(script: str, blender_bin: str,
                    extra: list[str]) -> list[str]:
    return ["nice", "-n", "15", blender_bin, "-b", "--factory-startup",
            "--python-exit-code", "1", "--threads", "2", "-P", script,
            "--"] + extra


def run_blender(script: str, extra: list[str], blender_bin: str,
                dry_run: bool = False) -> None:
    cmd = blender_command(script, blender_bin, extra)
    env = dict(os.environ, OMP_NUM_THREADS="2")
    print("+" + " ".join(cmd))
    if dry_run:
        return
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    print(proc.stdout[-3000:])
    if proc.returncode != 0:
        raise RuntimeError(f"blender failed: {proc.stderr[-2000:]}")


# ------------------------------------------- local convert (orchestration)


def local_convert(robot_dir: str | Path, blender_bin: str,
                  studio_ldraw: str = STUDIO_LDRAW,
                  guard_log: str | None = DEFAULT_LOG,
                  guard_port: int | None = None,
                  dry_run: bool = False) -> dict:
    """Convert robot.ldr to model.blend + model.glb via headless Blender."""
    from src.gen.inventory import load_catalog_merged
    from src.parse import parse_file

    check_denylist(robot_dir)
    robot_dir = Path(robot_dir)
    parts = parse_file(str(robot_dir / "robot.ldr"))
    report = json.loads((robot_dir / "report.json").read_text())
    catalog = load_catalog_merged()
    exp_path = robot_dir / "expected_kinematics.json"
    joints = None
    if exp_path.is_file():
        joints = json.loads(exp_path.read_text()).get("joints")
    segments, beams = _fold_trailing_beam_segment(
        report["segments"], report["beams"])
    split = split_links(parts, catalog, segments, beams,
                        joints=joints)
    for warning in split["warnings"]:
        print(f"warning: {warning}")
    (robot_dir / "links.json").write_text(
        json.dumps({"links": [
            {"id": lid, "members": uids}
            for lid, uids in split["links"].items()]}, indent=1),
        encoding="utf-8")
    run_guard(log=guard_log, port=guard_port)
    with heavy_lock():
        run_blender(str(Path(__file__).resolve()),
                    ["--blender-inner", "--mode", "convert",
                     "--robot-dir", str(robot_dir),
                     "--studio-ldraw", studio_ldraw],
                    blender_bin, dry_run)
    if dry_run:
        return {"dry_run": True}
    return json.loads((robot_dir / "convert_report.json").read_text())


def local_preview(robot_dir: str | Path, blender_bin: str,
                  view: str = "front", out: str | Path | None = None,
                  guard_log: str | None = DEFAULT_LOG,
                  guard_port: int | None = None,
                  dry_run: bool = False) -> Path:
    """Quick local Cycles preview of the converted scene (check render)."""
    check_denylist(robot_dir)
    robot_dir = Path(robot_dir)
    out_path = Path(out) if out else robot_dir / "preview.png"
    run_guard(log=guard_log, port=guard_port)
    with heavy_lock():
        run_blender(str(Path(__file__).resolve()),
                    ["--blender-inner", "--mode", "preview",
                     "--robot-dir", str(robot_dir), "--view", view,
                     "--out", str(out_path)],
                    blender_bin, dry_run)
    return out_path


# ------------------------------------------------- Blender inner (local)


def _inner_argv() -> dict:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out: dict = {}
    i = 0
    while i < len(argv):
        if argv[i].startswith("--") and i + 1 < len(argv) \
                and not argv[i + 1].startswith("--"):
            out[argv[i][2:].replace("-", "_")] = argv[i + 1]
            i += 2
        else:
            out[argv[i].lstrip("-").replace("-", "_")] = True
            i += 1
    return out


def _ldu_to_m(x, y, z):
    return (x * 0.0004, z * 0.0004, -y * 0.0004)


def blender_inner_main() -> None:
    """Headless-Blender entry: build the scene, convert or preview."""
    import bpy  # noqa: E402
    from mathutils import Matrix, Vector  # noqa: E402

    args = _inner_argv()
    repo = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repo))
    sys.path.insert(0, SKILL_SCRIPTS)
    import bpy_helpers as H  # noqa: E402

    from src.catalog.mesh import collect_faces  # noqa: E402
    from src.catalog.review import Library  # noqa: E402
    from src.parse import parse_file  # noqa: E402

    robot_dir = Path(args.get("robot_dir", "."))
    parts = parse_file(str(robot_dir / "robot.ldr"))
    links_doc = json.loads((robot_dir / "links.json").read_text())
    exp_doc = json.loads(
        (robot_dir / "expected_kinematics.json").read_text())
    joints = exp_doc["joints"]

    lib = Library.from_dir(args.get("studio_ldraw", STUDIO_LDRAW),
                           sorted({p["part_id"] for p in parts}))
    H.reset_scene()
    col = H.collection("robot")

    mesh_of, missing = {}, []
    for pid in sorted({p["part_id"] for p in parts}):
        if pid.lower() not in lib.blocks:
            missing.append(pid)
            continue
        verts: dict = {}
        faces: list = []
        ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        collect_faces(lib, pid, ident, [0, 0, 0], verts, faces,
                      xform=_ldu_to_m)
        if not verts:
            missing.append(pid)
            continue
        mesh = bpy.data.meshes.new(f"m_{pid}")
        mesh.from_pydata(list(verts.keys()), [], faces)
        mesh.materials.append(None)
        mesh.update()
        mesh_of[pid] = mesh
    print(f"meshes: {len(mesh_of)}, missing geometry: {missing}")

    link_of = {}
    for link in links_doc["links"]:
        for uid in link["members"]:
            link_of[uid] = link["id"]
    mat_of_link = {}
    for i, link in enumerate(links_doc["links"]):
        mat_of_link[link["id"]] = H.material(
            f"link_{link['id']}",
            base_color=LINK_COLOURS[i % len(LINK_COLOURS)], roughness=0.4)
    mat_missing = H.material("missing", base_color="#FF00FF", roughness=0.6)

    by_link: dict[str, list] = {}
    for part in parts:
        lid = link_of.get(part["uid"])
        mesh = mesh_of.get(part["part_id"])
        if mesh is None or lid is None:
            obj = H.add_box(part["uid"], size=(0.02, 0.02, 0.02),
                            collection=col)
            H.assign(obj, mat_missing)
        else:
            obj = bpy.data.objects.new(part["uid"], mesh)
            col.objects.link(obj)
            obj.material_slots[0].link = "OBJECT"
            obj.material_slots[0].material = mat_of_link[lid]
        loc = [v * MM for v in part["position_mm"]]
        obj.matrix_world = (Matrix.Translation(Vector(loc))
                            @ Matrix(part["rotation"]).to_4x4())
        by_link.setdefault(lid or "L?", []).append(obj)

    mat_axis = H.material("joint_axis", base_color=AXIS_COLOUR,
                          roughness=0.35)
    for joint in joints:
        axis = Vector(joint["axis"]).normalized()
        obj = H.add_cylinder(f"AX_{joint['id']}", radius=0.0012,
                             depth=0.03, collection=col)
        H.assign(obj, mat_axis)
        obj.location = Vector([v * MM for v in joint["point_mm"]])
        obj.rotation_mode = "QUATERNION"
        obj.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(axis)

    empties = {}
    prev_pt = [0.0, 0.0, 0.0]
    parent = None
    for joint in joints:
        empty = bpy.data.objects.new("EJ_" + joint["id"], None)
        col.objects.link(empty)
        if parent is not None:
            empty.parent = parent
        empty.location = Vector([(a - b) * MM for a, b in
                                 zip(joint["point_mm"], prev_pt)])
        empties[joint["id"]] = empty
        parent = empty
        prev_pt = joint["point_mm"]
    bpy.context.view_layer.update()
    link_ids = [link["id"] for link in links_doc["links"]]
    for n, joint in enumerate(joints):
        parent_inv = empties[joint["id"]].matrix_world.inverted()
        for obj in by_link.get(link_ids[n + 1], []):
            # glTF-safe parenting: relative basis, identity inverse
            # (matrix_parent_inverse does not survive glTF export).
            basis = parent_inv @ obj.matrix_world
            obj.parent = empties[joint["id"]]
            obj.matrix_parent_inverse.identity()
            obj.matrix_basis = basis
    bpy.context.view_layer.update()
    ee = bpy.data.objects.new("EE", None)
    col.objects.link(ee)
    ee.parent = empties[joints[-1]["id"]]
    tip = exp_doc["end_effector"]["point_mm"]
    ee.location = Vector([(a - b) * MM for a, b in
                          zip(tip, joints[-1]["point_mm"])])
    bpy.context.view_layer.update()

    mode = args.get("mode", "convert")
    if mode == "preview":
        views = {v["name"]: v for v in default_views()}
        spec = views.get(args.get("view", "front"), views["front"])
        meshes = [o for o in bpy.data.objects if o.type == "MESH"]
        H.studio_lighting(target=meshes, contrast=0.6, softness=0.5)
        cam = H.add_camera("cam_preview", location=(2, -2, 1.5),
                           target=(0, 0, 0.2), lens=50.0)
        H.frame_objects(cam, meshes, azimuth=spec["azimuth"],
                        elevation=spec["elevation"], margin=1.15)
        H.set_render(engine="CYCLES", resolution=(640, 480), samples=16,
                     percentage=100, view_transform="Standard")
        out = args.get("out", str(robot_dir / "preview.png"))
        H.render_still(out, background="#3A3F44", camera=cam)
        print(f"preview -> {out}")
        return

    blend_path = str(robot_dir / "model.blend")
    glb_path = str(robot_dir / "model.glb")
    H.save_blend(blend_path)
    bpy.ops.export_scene.gltf(filepath=glb_path, export_format="GLB",
                              export_apply=False)
    report = {"n_parts": len(parts), "n_meshes": len(mesh_of),
              "missing_geometry": missing,
              "n_links": len(link_ids),
              "warnings": [f"missing geometry for {m}" for m in missing]}
    (robot_dir / "convert_report.json").write_text(json.dumps(report),
                                                   encoding="utf-8")
    print(f"convert -> {blend_path}, {glb_path}")


# ------------------------------------------------ Modal batch render


INNER_TEMPLATE = '''"""Modal-side single-task renderer (generated by 5D, self-contained)."""
import bpy  # noqa: E402
import json
import math
import os
import sys
from mathutils import Vector


@@MATH@@


def parse_task():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    opts = {}
    i = 0
    while i < len(argv):
        if argv[i].startswith("--") and i + 1 < len(argv):
            opts[argv[i][2:]] = argv[i + 1]
            i += 2
        else:
            i += 1
    with open(opts["task"], encoding="utf-8") as fh:
        return json.load(fh), opts


def clear_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)


def bbox_sphere(objs):
    pts = []
    for obj in objs:
        pts.extend(obj.matrix_world @ Vector(c) for c in obj.bound_box)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    center = [(min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0,
              (min(zs) + max(zs)) / 2.0]
    radius = max((x - center[0]) ** 2 + (y - center[1]) ** 2
                 + (z - center[2]) ** 2 for x, y, z in pts) ** 0.5
    return center, radius


def main():
    task, opts = parse_task()
    clear_scene()
    bpy.ops.import_scene.gltf(filepath=task["glb"])
    for obj in bpy.data.objects:
        if obj.name.startswith("AX_"):
            obj.hide_render = True
    joints = {j["id"]: j for j in task["joints"]}
    for jid, qd in zip(task["joint_ids"], task["pose"]["q_deg"]):
        empty = bpy.data.objects.get("EJ_" + jid)
        assert empty is not None, "missing empty EJ_" + jid
        empty.rotation_mode = "QUATERNION"
        empty.rotation_quaternion = quat_from_axis_angle(
            joints[jid]["axis"], math.radians(qd))
    bpy.context.view_layer.update()

    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    center, radius = bbox_sphere(meshes)
    direction = dir_from_az_el(task["view"]["azimuth"],
                               task["view"]["elevation"])
    width, height = task["resolution"]
    half_sensor = min(18.0, 18.0 * height / width)
    dist = radius / math.sin(math.atan(half_sensor / 50.0)) * 1.15
    loc = [c + d * dist for c, d in zip(center, direction)]
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = 50.0
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    cam.location = loc
    cam.rotation_mode = "QUATERNION"
    cam.rotation_quaternion = look_quat([-d for d in direction])
    bpy.context.scene.camera = cam

    world = bpy.data.worlds["World"]
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (
        0.23, 0.25, 0.27, 1.0)
    sun_data = bpy.data.lights.new("sun", type="SUN")
    sun_data.energy = 3.0
    sun = bpy.data.objects.new("sun", sun_data)
    bpy.context.scene.collection.objects.link(sun)
    sun.location = (5.0, -5.0, 8.0)

    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.render.resolution_x, scene.render.resolution_y = task[
        "resolution"]
    scene.render.resolution_percentage = 100
    scene.cycles.samples = task["samples"]
    scene.view_settings.view_transform = "Standard"
    try:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        prefs.compute_device_type = "OPTIX"
        scene.cycles.device = "GPU"
    except (KeyError, TypeError, AttributeError):
        try:
            prefs.compute_device_type = "CUDA"
            scene.cycles.device = "GPU"
        except (KeyError, TypeError, AttributeError, NameError):
            scene.cycles.device = "CPU"
    scene.render.film_transparent = False
    scene.render.filepath = opts["out"]
    bpy.ops.render.render(write_still=True)
    print("rendered " + opts["out"])


main()
'''


def build_inner_script() -> str:
    """Modal-side render script with tested math inlined (single source)."""
    sources = [inspect.getsource(fn) for fn in
               (dir_from_az_el, quat_from_axis_angle, look_quat)]
    return INNER_TEMPLATE.replace("@@MATH@@", "".join(sources))


def blender_modal_image():
    """Modal image with Blender pinned to the local version."""
    return (modal.Image.debian_slim(python_version="3.11")
            .apt_install("curl", "xz-utils", "libx11-6", "libxi6",
                         "libxrender1", "libgl1", "libsm6", "libxext6",
                         "libxfixes3", "libglib2.0-0", "libgomp1",
                         "libxxf86vm1")
            .run_commands(
                f"curl -L {BLENDER_URL} -o /tmp/blender.tar.xz",
                "mkdir -p /opt/blender && tar -xf /tmp/blender.tar.xz "
                "-C /opt/blender --strip-components=1",
                "rm /tmp/blender.tar.xz",
                f"{BLENDER_CONTAINER_BIN} --version"))


if _MODAL_OK:
    app = modal.App("metahack-5d-render")
    _VOLUME = modal.Volume.from_name(MODAL_VOLUME, create_if_missing=True)

    @app.function(image=blender_modal_image(), gpu=MODAL_GPU,
                  timeout=MODAL_TIMEOUT_SECS,
                  volumes={"/vol": _VOLUME})
    def render_task(task: dict) -> dict:
        """Render one (pose, view) task; never raises (batch-robust)."""
        import subprocess
        import tempfile

        try:
            with tempfile.TemporaryDirectory() as tmp:
                inner = os.path.join(tmp, "inner.py")
                spec = os.path.join(tmp, "task.json")
                out = os.path.join(tmp, "out.png")
                with open(inner, "w", encoding="utf-8") as fh:
                    fh.write(task["inner_script"])
                slim = {k: v for k, v in task.items()
                        if k != "inner_script"}
                with open(spec, "w", encoding="utf-8") as fh:
                    json.dump(slim, fh)
                cmd = [BLENDER_CONTAINER_BIN, "-b", "--factory-startup",
                       "--python-exit-code", "1", "-P", inner, "--",
                       "--task", spec, "--out", out]
                proc = subprocess.run(cmd, capture_output=True, text=True,
                                      timeout=MODAL_TIMEOUT_SECS - 60)
                if proc.returncode != 0:
                    return {"ok": False, "id": task["id"],
                            "error": proc.stderr[-2000:]
                            or proc.stdout[-2000:]}
                with open(out, "rb") as fh:
                    png = fh.read()
                return {"ok": True, "id": task["id"], "png": png}
        except Exception as exc:  # noqa: BLE001 - reported per task
            return {"ok": False, "id": task.get("id", "?"),
                    "error": str(exc)[:1000]}


def remote_render(robot_dir: str | Path, job: dict,
                  exp_doc: dict) -> dict:
    """Remote render: tasks via volume .glb, PNGs returned (L4, 20 min)."""
    if not _MODAL_OK:
        raise RuntimeError("modal is not installed in this environment")
    robot_dir = Path(robot_dir)
    robot = job["robot"]
    inner_script = build_inner_script()
    joint_ids = [j["id"] for j in exp_doc["joints"]]
    joints = [{"id": j["id"], "axis": j["axis"]} for j in exp_doc["joints"]]
    tasks = []
    for task in iter_tasks(job):
        tasks.append({**task, "glb": f"/vol/{robot}/model.glb",
                      "joint_ids": joint_ids, "joints": joints,
                      "inner_script": inner_script})
    print(f"rendering {len(tasks)} tasks on {MODAL_GPU} "
          f"(timeout {MODAL_TIMEOUT_SECS}s each)")
    ok, failed = 0, []
    renders = robot_dir / "renders"
    renders.mkdir(parents=True, exist_ok=True)
    with app.run():
        for result in render_task.map(tasks, return_exceptions=True):
            if isinstance(result, Exception):
                failed.append(f"exception: {result}"[:200])
                continue
            if not result.get("ok"):
                failed.append(f"{result.get('id')}: "
                              f"{result.get('error', '?')}"[:200])
                continue
            (renders / f"{result['id']}.png").write_bytes(result["png"])
            ok += 1
    print(f"rendered {ok}/{len(tasks)} to {renders}")
    return {"ok": ok, "failed": failed, "n": len(tasks)}


def modal_preflight() -> None:
    """Fail fast with a hint when Modal auth is missing."""
    if shutil.which("modal") is None:
        raise RuntimeError("modal CLI not found (pip install modal)")
    proc = subprocess.run(["modal", "volume", "list"],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError("modal auth failed; run `modal setup` "
                           f"({proc.stderr.strip()[:200]})")


def modal_upload(robot_dir: str | Path, job_name: str = "render_job.json",
                 force: bool = True) -> None:
    """Upload model.glb + render_job.json to the Modal Volume."""
    check_denylist(robot_dir)
    robot_dir = Path(robot_dir)
    if shutil.which("modal") is None:
        raise RuntimeError("modal CLI not found (pip install modal)")
    for name in ("model.glb", job_name):
        src = robot_dir / name
        if not src.is_file():
            raise ValueError(f"missing upload input: {src}")
        cmd = ["modal", "volume", "put", MODAL_VOLUME, str(src),
               f"{robot_dir.name}/{name}"]
        if force:
            cmd.append("--force")
        print("+" + " ".join(cmd))
        proc = subprocess.run(cmd, capture_output=True, text=True)
        print(proc.stdout.strip() or proc.stderr.strip()[:500])
        if proc.returncode != 0:
            raise RuntimeError(f"upload failed for {name}")


# ------------------------------------------------------------------- CLI


def _parse_views(text: str) -> list[dict]:
    views = []
    for chunk in text.split(";"):
        name, _, rest = chunk.partition(":")
        az, _, el = rest.partition(",")
        views.append({"name": name.strip(), "azimuth": float(az),
                      "elevation": float(el)})
    return views


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="5D visual materials.")
    parser.add_argument("--robot-dir", required=True)
    parser.add_argument("--build-job", action="store_true")
    parser.add_argument("--convert", action="store_true")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--upload", action="store_true")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--poses", default=None)
    parser.add_argument("--views", default=None,
                        help="'name:az,el;...' (default front/side/top)")
    parser.add_argument("--resolution", default="1280x960")
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--view", default="front")
    parser.add_argument("--preview-out", default=None)
    parser.add_argument("--blender-bin", default=None)
    parser.add_argument("--studio-ldraw", default=STUDIO_LDRAW)
    parser.add_argument("--guard-port", type=int, default=None)
    parser.add_argument("--no-guard-log", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    if not any([args.build_job, args.convert, args.preview, args.upload,
                args.render]):
        print("nothing to do (pass --build-job/--convert/--preview/"
              "--upload/--render)")
        return 2
    try:
        check_denylist(args.robot_dir)
    except ValueError as exc:
        print(exc)
        return 2
    robot_dir = Path(args.robot_dir)
    guard_log = None if args.no_guard_log else DEFAULT_LOG

    try:
        if args.build_job:
            exp_doc = json.loads(
                (robot_dir / "expected_kinematics.json").read_text())
            if args.poses:
                poses = json.loads(Path(args.poses).read_text())
            else:
                poses = default_poses(exp_doc)
            width, height = (int(v) for v in
                             args.resolution.lower().split("x"))
            job = build_render_job(
                robot_dir.name, poses,
                _parse_views(args.views) if args.views else None,
                (width, height), args.samples)
            errors = validate_job(job)
            if errors:
                print(f"job invalid: {errors[0]}")
                return 1
            (robot_dir / "render_job.json").write_text(
                json.dumps(job, indent=1), encoding="utf-8")
            print(f"wrote render_job.json: {len(poses)} poses, "
                  f"{len(job['views'])} views, "
                  f"{len(poses) * len(job['views'])} tasks")
        if args.convert:
            report = local_convert(
                robot_dir, resolve_blender_bin(args.blender_bin),
                args.studio_ldraw, guard_log, args.guard_port,
                args.dry_run)
            print(f"convert: {report}")
        if args.preview:
            out = local_preview(
                robot_dir, resolve_blender_bin(args.blender_bin),
                args.view, args.preview_out, guard_log,
                args.guard_port, args.dry_run)
            print(f"preview: {out}")
        if args.upload:
            if args.dry_run:
                print("dry-run: would upload model.glb + render_job.json")
            else:
                modal_preflight()
                modal_upload(robot_dir)
        if args.render:
            job = json.loads(
                (robot_dir / "render_job.json").read_text())
            errors = validate_job(job)
            if errors:
                print(f"job invalid: {errors[0]}")
                return 1
            if args.dry_run:
                tasks = iter_tasks(job)
                print(f"dry-run: {len(tasks)} tasks "
                      f"({tasks[0]['id']} .. {tasks[-1]['id']})")
            else:
                modal_preflight()
                exp_doc = json.loads(
                    (robot_dir / "expected_kinematics.json").read_text())
                summary = remote_render(robot_dir, job, exp_doc)
                if summary["failed"]:
                    print(f"failures: {summary['failed'][:3]}")
                    return 2
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"failed: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    if "--blender-inner" in sys.argv:
        blender_inner_main()
    else:
        raise SystemExit(main())
