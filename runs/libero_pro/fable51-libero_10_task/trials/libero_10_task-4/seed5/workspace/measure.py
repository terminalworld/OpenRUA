#!/usr/bin/env python3
"""Hover 0.25 m above a guessed mug position and measure its rim centre/radius.

Usage: python3 measure.py <x> <y> [tag]
"""
import sys

import numpy as np

from eyescan import segments, snap
from robot import Robot, down_quat


def measure(r, gx, gy, tag="mug", hover_z=0.80):
    q = r.ik([gx, gy, hover_z], down_quat(0))
    r.move_q(q, 3.0)
    print("tcp", r.tcp()[0].round(4))
    color, depth, info, P = snap(r, tag)
    segs = segments(P, depth, color, 0.46, 0.75)
    for s in segs:
        print(f"  seg px=({s['px'][0]:.0f},{s['px'][1]:.0f}) area={s['area']} c={s['c'].round(3)} "
              f"z=({s['zmin']:.3f},{s['zmax']:.3f}) xr=({s['xr'][0]:.3f},{s['xr'][1]:.3f}) "
              f"yr=({s['yr'][0]:.3f},{s['yr'][1]:.3f}) rgb={s['rgb']}")
    s = min(segs, key=lambda s: np.hypot(s["c"][0] - gx, s["c"][1] - gy))
    pts = P[s["mask"]]
    zmax = s["zmax"]
    top = pts[pts[:, 2] > zmax - 0.008]
    cx, cy = top[:, 0].mean(), top[:, 1].mean()
    rr = np.hypot(top[:, 0] - cx, top[:, 1] - cy)
    # handle direction: points beyond r=0.055
    far = pts[np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) > 0.055]
    hdir = (far[:, :2].mean(0) - [cx, cy]) if len(far) else np.zeros(2)
    print(f"rim centre ({cx:.4f},{cy:.4f}) z_top={zmax:.3f} r_out~{np.percentile(rr, 95):.3f} "
          f"r_in~{np.percentile(rr, 5):.3f} handle_dir={hdir.round(3)} n_far={len(far)}")
    return np.array([cx, cy]), zmax, hdir


if __name__ == "__main__":
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    tag = sys.argv[3] if len(sys.argv) > 3 else "mug"
    measure(Robot("measure"), gx, gy, tag)
