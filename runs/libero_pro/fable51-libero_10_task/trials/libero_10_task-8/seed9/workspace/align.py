#!/usr/bin/env python3
"""Measure pot rim vs fingers from the wrist camera (world frame via TF).

Returns dict with rim centre (x,y), rim top z, flat-normal angle (deg,
world, in [0,45)), finger gap centre and finger axis angle, plus FK tcp.
"""
import numpy as np

import cloud
from rob import quat_to_R


def extents(q, ang_deg):
    a = np.deg2rad(ang_deg)
    u = np.array([np.cos(a), np.sin(a)])
    pr = q @ u
    return pr.min(), pr.max()


def measure(r, pot_guess, verbose=True):
    P, rgb, depth = cloud.grab("robot0_eye_in_hand", node=None)
    tcp, quat = r.tcp()
    R = quat_to_R(quat)
    out = {"tcp": tcp, "hand_y": R[:, 1]}
    # ---- fingers: points within 12 cm of tcp and above tcp-1cm, at |hand-y| ~ 3.5..5 cm
    rel = P - tcp
    hy = rel @ R[:, 1]
    hx = rel @ R[:, 0]
    hz = rel @ R[:, 2]
    fm = (np.abs(hx) < 0.02) & (hz > -0.06) & (hz < 0.015) & (np.abs(hy) > 0.03) & (np.abs(hy) < 0.06)
    fm &= np.isfinite(P).all(-1)
    fp = P[fm]
    if len(fp) > 50:
        hyf = (fp - tcp) @ R[:, 1]
        left = hyf[hyf < 0]
        right = hyf[hyf > 0]
        # inner faces = max of left blob, min of right blob (robust percentiles)
        li = np.percentile(left, 99) if len(left) else np.nan
        ri = np.percentile(right, 1) if len(right) else np.nan
        out["finger_inner_hy"] = (li, ri)
        out["finger_gap"] = ri - li
        out["finger_centre_hy"] = (li + ri) / 2
    # ---- rim
    m = (np.abs(P[..., 0] - pot_guess[0]) < 0.08) & (np.abs(P[..., 1] - pot_guess[1]) < 0.08)
    m &= np.isfinite(P).all(-1)
    pts = P[m]
    ztop = np.percentile(pts[:, 2], 99.5)
    out["z_top"] = ztop
    rim = pts[(pts[:, 2] > ztop - 0.035) & (pts[:, 2] < ztop - 0.012)]  # rim + top wall, below knob
    c = rim[:, :2].mean(0)
    for _ in range(8):
        keep = np.linalg.norm(rim[:, :2] - c, axis=1) < 0.0415
        c = rim[keep][:, :2].mean(0)
    q = rim[keep][:, :2] - c
    angs = np.arange(0, 45, 0.5)
    ext = np.array([np.subtract(*extents(q, a)[::-1]) for a in angs])
    amin = angs[np.argmin(ext)]
    # refine centre as midpoint along flat normals amin and amin+90 (and 45, 135)
    cs = []
    for a in (amin, amin + 45, amin + 90, amin + 135):
        lo, hi = extents(q, a)
        u = np.array([np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a))])
        cs.append(((lo + hi) / 2) * u)
    c_ref = c + np.sum(cs, 0) / 2  # each pair of perpendicular normals gives full 2D shift
    out["rim_centre"] = c_ref
    out["flat_normal_deg"] = amin
    out["ftf"] = ext.min()
    out["vtv"] = ext.max()
    out["ext_profile"] = list(zip(angs[::10], ext[::10].round(4)))
    if verbose:
        for k, v in out.items():
            if k != "ext_profile":
                print(f"  {k}: {np.round(v, 4) if not isinstance(v, tuple) else tuple(np.round(v, 4))}")
    return out
