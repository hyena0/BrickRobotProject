"""Link-segmentation agreement (Stage 2.5).

Matches predicted links to ground-truth links greedily by shared part
count, then reports the fraction of parts on the correct link, weighted
by part count. Loud failure when ground truth is still pending.
"""

from __future__ import annotations

import argparse
import json


def agreement(pred_members: dict[str, list[str]],
              gt_membership: dict[str, str]) -> dict:
    """Score predicted link membership against ground truth."""
    if not gt_membership:
        raise ValueError("ground-truth membership is empty or pending")
    # pred link -> gt link -> shared count.
    overlap: dict[str, dict[str, int]] = {}
    for link_id, uids in pred_members.items():
        for uid in uids:
            gt = gt_membership.get(uid)
            if gt is None:
                continue
            overlap.setdefault(link_id, {}).setdefault(gt, 0)
            overlap[link_id][gt] += 1
    mapping = {link: max(counts, key=counts.get)
               for link, counts in overlap.items() if counts}
    scored = sum(1 for uid in gt_membership if uid)
    correct = sum(1 for uid, gt in gt_membership.items()
                  if any(uid in uids and mapping.get(link) == gt
                         for link, uids in pred_members.items()))
    return {"accuracy": correct / scored if scored else 0.0,
            "correct": correct, "scored": scored,
            "link_mapping": mapping}


def main() -> None:
    ap = argparse.ArgumentParser(description="Score link segmentation")
    ap.add_argument("--pred", required=True,
                    help="runs/<id>/links_joints.json")
    ap.add_argument("--gt", required=True,
                    help="eval/ground_truth/segmentation.json")
    args = ap.parse_args()
    pred = json.load(open(args.pred, encoding="utf-8"))
    gt = json.load(open(args.gt, encoding="utf-8"))
    members = {link["id"]: link.get("members", [])
               for link in pred.get("links", [])}
    try:
        result = agreement(members, gt.get("membership", {}))
    except ValueError:
        print("ground-truth membership is empty or pending; cannot score")
        raise SystemExit(2)
    print(f"accuracy={result['accuracy']:.3f} "
          f"({result['correct']}/{result['scored']})")
    print(f"gate>=0.90: {'PASS' if result['accuracy'] >= 0.90 else 'FAIL'}")


if __name__ == "__main__":
    main()
