"""Articulate the robot in Blender for pose_check (runs headless).

Reads the pose batch (joints, members, poses), builds one joint-Empty
hierarchy so Blender itself evaluates the chain, then per pose sets
AXIS_ANGLE rotations, records world AABBs plus the EE position, and
renders front/side/top. Prints only counts and paths.

Run via src/viz/pose_check.py (which launches Blender)::

    Blender -b --factory-startup --python-exit-code 1 \\
        -P src/viz/pose_render_inner.py -- --batch ... --out ... \\
        --renders ... [--quick]
"""

import json
import math
import os
import sys

REPO = None
SKILL_SCRIPTS = ("/Users/kunwoo.chang/.local/share/muse/skills/bundled/"
                 "muse-core/skills/blender/scripts")

MM = 0.001


def parse_argv():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {"quick": "--quick" in argv}
    for i, a in enumerate(argv):
        if a == "--batch" and i + 1 < len(argv):
            out["batch"] = argv[i + 1]
        if a == "--out" and i + 1 < len(argv):
            out["out"] = argv[i + 1]
        if a == "--renders" and i + 1 < len(argv):
            out["renders"] = argv[i + 1]
    return out


def main():
    global REPO
    args = parse_argv()
    import bpy  # noqa: E402
    from mathutils import Vector  # noqa: E402
    sys.path.insert(0, SKILL_SCRIPTS)
    import bpy_helpers as H  # noqa: E402

    batch = json.load(open(args["batch"], encoding="utf-8"))
    REPO = batch["repo"]
    os.chdir(REPO)
    bpy.ops.wm.open_mainfile(
        filepath=os.path.join(REPO, batch["base_blend"]))

    link_ids = batch["link_ids"]
    joints = batch["joints"]
    members = batch["members"]
    by_link = {}
    for lid in link_ids:
        by_link[lid] = [bpy.data.objects[u] for u in members.get(lid, [])
                        if u in bpy.data.objects]
    n_member = sum(len(v) for v in by_link.values())
    print(f"member objects parented: {n_member}")
    for o in bpy.data.objects:
        if o.name.startswith(("ax_", "ah_")):
            o.hide_render = True
            o.hide_viewport = True

    # One rotor empty per joint. The rotor's local frame IS the parent
    # link frame, so an axis-angle rotation about the catalog axis is
    # exactly the joint motion: child world = W_{k-1} T(Jk) R_k.
    # Children carry locations relative to their parent's frame, and
    # only member objects use matrix_parent_inverse (their link frame
    # at the zero pose); hierarchy links keep the identity inverse.
    col = bpy.data.collections.get("robot")
    empties = {}
    prev_pt = [0.0, 0.0, 0.0]
    parent = None
    for j in joints:
        e = bpy.data.objects.new("EJ_" + j["id"], None)
        if col is not None:
            col.objects.link(e)
        else:
            bpy.context.scene.collection.objects.link(e)
        if parent is not None:
            e.parent = parent
        e.location = Vector([(a - b) * MM for a, b in
                             zip(j["point_mm"], prev_pt)])
        e.rotation_mode = "AXIS_ANGLE"
        empties[j["id"]] = e
        parent = e
        prev_pt = j["point_mm"]
    bpy.context.view_layer.update()
    for n, j in enumerate(joints):
        child_link = link_ids[n + 1]
        inv = empties[j["id"]].matrix_world.inverted()
        for o in by_link.get(child_link, []):
            o.parent = empties[j["id"]]
            o.matrix_parent_inverse = inv
    ee = bpy.data.objects.new("EE_pose", None)
    if col is not None:
        col.objects.link(ee)
    else:
        bpy.context.scene.collection.objects.link(ee)
    last = joints[-1]
    ee.parent = empties[last["id"]]
    ee.location = Vector([(a - b) * MM for a, b in
                          zip(batch["ee_mm"], last["point_mm"])])
    bpy.context.view_layer.update()
    others = [o.name for o in bpy.data.objects if o.type == "MESH"
              and not o.name.startswith(("ax_", "ah_"))
              and not any(o.name in members.get(lid, []) for lid in
                          link_ids)]
    print(f"unparented non-member meshes: {len(others)} {others[:5]}")

    member_set = set()
    for lid in link_ids:
        member_set.update(members.get(lid, []))
    meshes = [o for o in bpy.data.objects
              if o.type == "MESH" and o.name in member_set]

    def snapshot(tag):
        bpy.context.view_layer.update()
        aabbs = {}
        for o in meshes:
            ws = [o.matrix_world @ Vector(c) for c in o.bound_box]
            xs = [v[0] for v in ws]
            ys = [v[1] for v in ws]
            zs = [v[2] for v in ws]
            aabbs[o.name] = [[round(min(xs) / MM, 3),
                              round(min(ys) / MM, 3),
                              round(min(zs) / MM, 3)],
                             [round(max(xs) / MM, 3),
                              round(max(ys) / MM, 3),
                              round(max(zs) / MM, 3)]]
        we = ee.matrix_world.translation
        return {"aabbs_mm": aabbs,
                "ee_mm": [round(we[0] / MM, 3), round(we[1] / MM, 3),
                          round(we[2] / MM, 3)]}

    zero = snapshot("zero")
    print(f"zero-pose AABBs: {len(zero['aabbs_mm'])}")

    if args["quick"]:
        res, samples = (800, 600), 16
    else:
        res, samples = (1280, 960), 32
    H.set_render(engine="CYCLES", resolution=res, samples=samples,
                 percentage=100, view_transform="Standard")
    os.makedirs(args["renders"], exist_ok=True)
    views = {"front": (0.0, 8.0), "side": (90.0, 8.0), "top": (35.0, 89.0)}
    cams = {}
    for name in views:
        cam = H.add_camera(f"cam_pose_{name}", location=(2, -2, 1.5),
                           target=(0, 0, 0.2), lens=50.0)
        cams[name] = cam

    out_poses = {}
    for pose in batch["poses"]:
        for j, qd in zip(joints, pose["q_deg"]):
            ax = Vector(j["axis"]).normalized()
            e = empties[j["id"]]
            e.rotation_axis_angle = [math.radians(qd), ax[0], ax[1],
                                     ax[2]]
        snap = snapshot(pose["id"])
        out_poses[pose["id"]] = snap
        for name, (az, el) in views.items():
            H.frame_objects(cams[name], meshes, azimuth=az, elevation=el,
                            margin=1.15)
            out = os.path.join(args["renders"],
                               f"{pose['id']}_{name}.png")
            H.render_still(out, background="#3A3F44", camera=cams[name])
        print(f"rendered {pose['id']}: ee={snap['ee_mm']}")

    json.dump({"poses": out_poses, "zero_aabbs_mm": zero["aabbs_mm"]},
              open(args["out"], "w"), indent=1)
    print(f"wrote {args['out']}")


if __name__ == "__main__":
    main()
