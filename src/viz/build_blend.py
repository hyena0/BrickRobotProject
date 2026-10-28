"""Build a Blender verification scene from the Studio .io export.

Stage 2.8: colours parts by predicted link, draws each joint axis as
an arrow. Run headless::

    Blender -b --factory-startup --python-exit-code 1 \\
        -P src/viz/build_blend.py -- --mode links --final

Modes: `links` (link colours + joint arrows) and `true` (LDraw colours,
no arrows). Outputs (all git-ignored): .blend to runs/stage2-blend/,
renders to assets/private/renders/stage2-blend/.
"""

from __future__ import annotations

import colorsys
import json
import math
import os
import sys
import zipfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, REPO)
SKILL_SCRIPTS = ("/Users/kunwoo.chang/.local/share/muse/skills/bundled/"
                 "muse-core/skills/blender/scripts")
sys.path.insert(0, SKILL_SCRIPTS)

import bpy  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402


import bpy_helpers as H  # noqa: E402
from src.catalog.mesh import collect_faces  # noqa: E402
from src.catalog.review import Library  # noqa: E402
from src.viz.rotutil import orthonormalize_rotation  # noqa: E402

P = {
    "io": "3D models/Robot_6axis_integration_Comb.io",
    "studio_ldraw": "/Applications/Studio 2.0/ldraw",
    "parts": "runs/stage1/parts.json",
    "links": "runs/stage2-v4/links_joints.json",
    "out_blend": "runs/stage2-blend/model_links.blend",
    "out_dir": "assets/private/renders/stage2-blend",
    "ldraw_scale": 0.0004,  # LDU -> metres
    "arrow_len": 0.05,
    "arrow_rad": 0.0025,
}

# Approximate sRGB for the LDraw colour codes present in the model.
COLORS = {
    "0": "#1B2A34", "15": "#F4F4F4", "7": "#9BA19D", "23": "#0055BF",
    "72": "#6C6E68", "191": "#FE8A18", "216": "#AA7F2E",
    "70": "#5F3109", "100241": "#3F3F3F",
}

# Part types missing from the embedded library snapshot.
SUBSTITUTE = {
    "18938.dat": "18938u.dat",  # same 60T top family
}
PLACEHOLDER = {
    # magenta flags non-exact stand-ins (documented in the report)
    "59443.dat": ("box", (0.016, 0.016, 0.04)),
    "32123a.dat": ("cyl", (0.006, 0.010)),
    "3648b.dat": ("cyl", (0.013, 0.008)),  # 24T spur ~26mm dia
    # (was r=0.024: 48mm drum that swallowed neighbours; user
    # flagged the unphysical block in j2_void render)
    "95646.dat": ("box", (0.110, 0.070, 0.045)),
}


def parse_argv():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {"mode": "links", "final": "--final" in argv}
    for i, a in enumerate(argv):
        if not a.startswith("--") and i == 0:
            continue
        if a == "--mode" and i + 1 < len(argv):
            out["mode"] = argv[i + 1]
        if a == "--links" and i + 1 < len(argv):
            out["links"] = argv[i + 1]
        if a == "--out-blend" and i + 1 < len(argv):
            out["out_blend"] = argv[i + 1]
        if a == "--out-dir" and i + 1 < len(argv):
            out["out_dir"] = argv[i + 1]
    return out


def ldu_to_m(x, y, z):
    s = P["ldraw_scale"]
    return (x * s, z * s, -y * s)


def type_mesh(lib, pid):
    """Triangle/quad soup of one part type in metres (type-local)."""
    verts: dict = {}
    faces: list = []
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    collect_faces(lib, pid, ident, [0, 0, 0], verts, faces,
                  xform=ldu_to_m)
    return list(verts.keys()), faces


def link_palette(n):
    mats = []
    for i in range(n):
        h = (i * 0.61803398875) % 1.0
        r, g, b = colorsys.hsv_to_rgb(h, 0.72, 0.88)
        mats.append((r, g, b, 1.0))
    return mats


def build(args):
    os.chdir(REPO)
    # Primary geometry comes from the Studio ldraw/ tree; the .io
    # embedded snapshot only fills custom types Studio lacks
    # (never the reverse), so official parts render exact.
    lib = Library.from_dir(P["studio_ldraw"],
                           sorted({p["part_id"] for p in
                                   json.load(open(P["parts"],
                                                  encoding="utf-8"))}))
    lib.merge_missing(Library.from_io(P["io"]))
    parts = json.load(open(P["parts"], encoding="utf-8"))
    doc = json.load(open(args.get("links", P["links"]), encoding="utf-8"))

    H.reset_scene()
    need = sorted({p["part_id"] for p in parts})
    print(f"part types: {len(need)}, parts: {len(parts)}")

    mesh_of = {}
    missing = []
    for pid in need:
        actual = SUBSTITUTE.get(pid, pid)
        if actual.lower() not in lib.blocks and pid not in PLACEHOLDER:
            missing.append(pid)
            continue
        verts, faces = type_mesh(lib, actual)
        if not verts:
            missing.append(pid)
            continue
        mesh = bpy.data.meshes.new(f"m_{pid}")
        mesh.from_pydata(verts, [], faces)
        mesh.update()
        mesh_of[pid] = mesh
    print(f"meshes: {len(mesh_of)}, missing geometry: {missing}")

    link_of = {}
    for link in doc.get("links", []):
        for uid in link.get("members", []):
            link_of[uid] = link["id"]
    link_ids = [link["id"] for link in doc.get("links", [])]
    palette = link_palette(len(link_ids))
    mat_of_link = {}
    for i, lid in enumerate(link_ids):
        mat = H.material(f"link_{lid}", base_color=palette[i],
                         roughness=0.55)
        mat_of_link[lid] = mat
    mat_of_color = {}
    for code, hexcol in COLORS.items():
        mat_of_color[code] = H.material(f"c_{code}", base_color=hexcol,
                                        roughness=0.4)
    mat_missing = H.material("missing", base_color="#FF00FF", roughness=0.6)
    mat_joint = H.material("joint", base_color="#FF2222", roughness=0.4)

    col = H.collection("robot")
    noblend = []
    used_meshes = set()
    for p in parts:
        mesh = mesh_of.get(p["part_id"])
        if mesh is None:
            kind = PLACEHOLDER.get(p["part_id"], ("box", (0.02,) * 3))
            if kind[0] == "box":
                obj = H.add_box(p["uid"], size=kind[1], collection=col)
            else:
                obj = H.add_cylinder(p["uid"], radius=kind[1][0],
                                     depth=kind[1][1], collection=col)
            H.assign(obj, mat_missing)
            noblend.append(p["part_id"])
        else:
            # One mesh per object: link colours vary per instance even
            # for shared part types (slots live on the mesh).
            if mesh.name in used_meshes:
                mesh = mesh.copy()
            used_meshes.add(mesh.name)
            obj = bpy.data.objects.new(p["uid"], mesh)
            H.link(obj, col)
        loc = [v / 1000.0 for v in p["position_mm"]]
        rot = orthonormalize_rotation(p["rotation"])
        obj.matrix_world = Matrix((
            (rot[0][0], rot[0][1], rot[0][2], loc[0]),
            (rot[1][0], rot[1][1], rot[1][2], loc[1]),
            (rot[2][0], rot[2][1], rot[2][2], loc[2]),
            (0, 0, 0, 1)))
        if args["mode"] == "links":
            lid = link_of.get(p["uid"])
            if lid is not None:
                obj.data.materials.clear()
                H.assign(obj, mat_of_link[lid])
        else:
            obj.data.materials.clear()
            H.assign(obj, mat_of_color.get(str(p.get("colour", "72")),
                                           mat_missing))
    print(f"placeholders used for: {sorted(set(noblend))}")

    mat_dropped = H.material("joint_dropped", base_color="#8A8A8A",
                               roughness=0.6)
    arrows = []
    if args["mode"] == "links":
        for j in doc.get("joints", []):
            # Selected joints: thick red arrows. Dropped top candidates:
            # thin gray arrows. Anything else is not drawn.
            status = j.get("status", "selected")
            if status not in ("selected", "dropped"):
                continue
            thick = status == "selected"
            rad = P["arrow_rad"] * (1.6 if thick else 0.5)
            length = P["arrow_len"] * (1.8 if thick else 1.0)
            tip = Vector([v / 1000.0 for v in j["point_mm"]])
            direction = Vector(j["axis"]).normalized()
            base = tip - direction * length / 2
            shaft = H.add_cylinder(
                f"ax_{j['id']}", radius=rad,
                depth=length * 0.7,
                location=base + direction * length * 0.35,
                collection=col)
            shaft.rotation_mode = "QUATERNION"
            shaft.rotation_quaternion = direction.to_track_quat("Z", "Y")
            head = H.add_cone(
                f"ah_{j['id']}", radius=rad * 2.4,
                depth=length * 0.3,
                location=tip - direction * length * 0.15,
                collection=col)
            head.rotation_mode = "QUATERNION"
            head.rotation_quaternion = direction.to_track_quat("Z", "Y")
            H.assign(shaft, mat_joint if thick else mat_dropped)
            H.assign(head, mat_joint if thick else mat_dropped)
            arrows += [shaft, head]
    print(f"joint arrows drawn: {len(arrows)}")

    objs = [o for o in col.objects if o.type == "MESH"]
    H.report()
    return col, objs


def main():
    args = parse_argv()
    final = args["final"]
    out_blend = args.get("out_blend", P["out_blend"])
    out_dir = args.get("out_dir", P["out_dir"])
    col, objs = build(args)
    target = [o for o in objs if not o.name.startswith(("ax_", "ah_"))]

    H.studio_lighting(target=target, contrast=0.6, softness=0.5)
    res = (1600, 1200) if final else (800, 600)
    samples = 64 if final else 16
    H.set_render(engine="CYCLES", resolution=res, samples=samples,
                 percentage=100, view_transform="Standard")
    os.makedirs(out_dir, exist_ok=True)

    views = [("threequarter", 35.0, 20.0), ("front", 0.0, 8.0),
             ("side", 90.0, 8.0), ("top", 35.0, 89.0)]
    mode = args["mode"]
    for name, az, el in views:
        cam = H.add_camera(f"cam_{name}", location=(2, -2, 1.5),
                           target=(0, 0, 0.2), lens=50.0)
        H.frame_objects(cam, target, azimuth=az, elevation=el, margin=1.15)
        out = os.path.join(out_dir, f"{mode}_{name}.png")
        H.render_still(out, background="#3A3F44", camera=cam)
        print(f"rendered {out}")
    H.save_blend(out_blend)
    print(f"saved {out_blend}")


if __name__ == "__main__":
    main()
