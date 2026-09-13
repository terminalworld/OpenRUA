#!/usr/bin/env python3
"""Pan: push clear of the moka pot, grasp handle, place on burner.

Usage: python3 pan.py <stage>   stage in {push, grasp, place, scan}
"""
import math
import sys
import numpy as np
from rob import *

TCP = M["hand"]["tcp_offset_m"]
TABLE = 0.900
BURNER = np.array([-0.049, 0.200])   # burner ring centre (world)
BURNER_TOP = 0.930
SAFE_Z = 1.20                        # hand z for traverses

r = Robot("pan")


def pan_geometry(pw):
    """Locate the pan body (disc) and handle from a birdview cloud."""
    z = pw[..., 2]
    # pan body: points above table, y < -0.13 region (away from moka), x<0.15
    m = np.isfinite(z) & (z > 0.907) & (pw[..., 1] < -0.12) & (pw[..., 0] > -0.35) & (pw[..., 0] < 0.15) & (pw[..., 1] > -0.5)
    P = pw[m]
    # rim = highest ring; fit circle centre as bbox centre of points z>0.925
    rim = P[P[:, 2] > 0.925]
    cx = (rim[:, 0].min() + rim[:, 0].max()) / 2
    rad = (rim[:, 0].max() - rim[:, 0].min()) / 2
    cy = rim[:, 1].min() + rad                    # y-max side is contaminated by the handle bracket
    print(f"pan body centre=({cx:.3f},{cy:.3f}) r={rad:.3f} rim z max={rim[:,2].max():.3f}")
    # handle: narrow strip beyond the rim on +y side
    out = {"c": np.array([cx, cy]), "r": rad}
    for dy in (0.05, 0.06, 0.08, 0.10, 0.12):
        y0 = cy + rad + dy
        mh = np.isfinite(z) & (z > 0.907) & (z < 0.98) & (pw[..., 1] >= y0 - 0.005) & (pw[..., 1] < y0 + 0.005) & (np.abs(pw[..., 0] - cx) < 0.05)
        H = pw[mh]
        if len(H):
            print(f"  handle @y={y0:.3f}: x[{H[:,0].min():.3f},{H[:,0].max():.3f}] xc={H[:,0].mean():.3f} ztop={H[:,2].max():.3f}")
            out[f"h{dy}"] = (H[:, 0].mean(), y0, H[:, 2].max())
    return out


def scan():
    pw = r.cloud("birdview")
    g = pan_geometry(pw)
    # moka pot
    z = pw[..., 2]
    m = np.isfinite(z) & (z > 0.99) & (pw[..., 0] > -0.1) & (pw[..., 0] < 0.15) & (pw[..., 1] > -0.15) & (pw[..., 1] < 0.1)
    P = pw[m]
    if len(P):
        print(f"moka top pts: x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] zmax={P[:,2].max():.3f}")
    return g, pw


stage = sys.argv[1]
print("q", np.round(r.arm_q(), 3), "fingers", np.round(r.fingers(), 4), flush=True)


def go_home():
    q = np.array(r.arm_q())
    if np.abs(q - np.array(HOME)).max() > 0.05:
        r.move_q(HOME, 4.0)


def hover(xy, yaw, z=SAFE_Z):
    """From HOME: IK seeded with HOME so the branch stays elbow-up; move there."""
    go_home()
    sol = r.ik([xy[0], xy[1], z], topdown_quat(yaw), seed=HOME)
    if sol is None:
        sol = r.ik([xy[0], xy[1], z], topdown_quat(-yaw), seed=HOME)
        if sol is None:
            raise SystemExit("hover IK failed")
        yaw = -yaw
    print("hover sol", np.round(sol, 3), "diff from HOME", np.round(np.array(sol) - HOME, 2), flush=True)
    r.move_q(sol, 4.0)
    print("  hand", r.fk()[0].round(4), flush=True)
    return sol, yaw

if stage == "scan":
    scan()

elif stage == "push":
    g, pw = scan()
    c, rad = g["c"], g["r"]
    r.gripper(0.0)
    yaw = 0.0                                     # fingers along y; closed gripper ~2cm in y
    start = np.array([c[0] + rad + 0.02, c[1]])
    zt = 0.925 + TCP
    q1, yaw = hover(start, yaw)
    q2 = r.move_pose([start[0], start[1], zt], topdown_quat(yaw), 2.5, seed=q1)
    print("wrench before push", r.wrench(), flush=True)
    end = np.array([start[0] - 0.09, start[1]])
    q3 = r.move_pose([end[0], end[1], zt], topdown_quat(yaw), 3.0, seed=q2)
    print("wrench after push", r.wrench(), flush=True)
    r.move_pose([end[0], end[1], SAFE_Z], topdown_quat(yaw), 2.5, seed=q3)
    go_home()
    scan()

elif stage == "grasp":
    g, pw = scan()
    c, rad = g["c"], g["r"]
    hx, hy, htop = g["h0.05"]
    yaw = math.pi / 2                             # fingers close along world x
    r.gripper(0.04)
    ft = htop - 0.014
    print(f"grasp at ({hx:.3f},{hy:.3f}) fingertip z {ft:.3f}", flush=True)
    q1, yaw = hover((hx, hy), yaw, z=1.15)
    q3 = r.move_line([hx, hy, ft + TCP], topdown_quat(yaw), step=0.03, speed=0.08, seed=q1)
    if q3 is None:
        raise SystemExit("descent failed")
    p, qu = r.fk()
    print("hand", p.round(4), "fingertip z", round(p[2] - TCP, 4), flush=True)
    f = r.gripper(0.0)
    if abs(f[0]) < 0.004:
        print("!! closed on air", flush=True)
    w0 = r.wrench()[0]
    q4 = r.move_line([hx, hy, ft + TCP + 0.10], topdown_quat(yaw), step=0.03, speed=0.05, seed=q3)
    print("fingers after lift", np.round(r.fingers(), 4), "dF", (r.wrench()[0] - w0).round(2), flush=True)
    np.save("grasp_info.npy", np.array([hx, hy, htop, c[0], c[1], yaw]))
    # verify: pan rim should now be ~10cm higher (scan from birdview; arm is over the pan though)
    pw2 = r.cloud("birdview")
    z = pw2[..., 2]
    m = np.isfinite(z) & (pw2[..., 1] < c[1] + 0.02) & (pw2[..., 1] > c[1] - 0.12) & (np.abs(pw2[..., 0] - c[0]) < 0.12) & (z > 0.905) & (z < 1.3)
    P = pw2[m]
    print(f"points in pan region: n={len(P)} z[{P[:,2].min():.3f},{P[:,2].max():.3f}] median z {np.median(P[:,2]):.3f}", flush=True)

elif stage == "place":
    hx, hy, htop, cx, cy, yaw = np.load("grasp_info.npy")
    off = np.array([hx - cx, hy - cy])            # grasp point relative to pan centre
    tgt = BURNER + off + np.array([0.015, 0.0])   # pan centre 1.5cm +x of burner centre: keeps rim clear of the knob
    print("place: hand xy target", tgt.round(4), "offset", off.round(4), flush=True)
    q0 = r.arm_q()
    p, qu = r.fk(q0)
    ft_at_grasp = htop - 0.014
    # traverse: straight line at current height (pan bottom ~10cm above table)
    q_up = r.move_line([p[0], p[1], 1.22], topdown_quat(yaw), step=0.03, speed=0.08, seed=q0)
    q1 = r.move_line([tgt[0], tgt[1], 1.22], topdown_quat(yaw), step=0.03, speed=0.08, seed=q_up)
    if q1 is None:
        raise SystemExit("traverse failed")
    ft_target = ft_at_grasp + (BURNER_TOP - TABLE) + 0.010   # pan bottom 1cm above burner (if hanging level)
    q2 = r.move_line([tgt[0], tgt[1], ft_target + TCP], topdown_quat(yaw), step=0.03, speed=0.05, seed=q1)
    w0 = r.wrench()[0]
    print("wrench above burner", w0.round(3), flush=True)
    zt = ft_target
    q = q2
    for i in range(4):
        zt -= 0.005
        q = r.move_line([tgt[0], tgt[1], zt + TCP], topdown_quat(yaw), step=0.01, speed=0.02, seed=q)
        w = r.wrench()[0]
        print(f"  step {i}: fingertip z {zt:.3f} dF {(w - w0).round(3)}", flush=True)
        if w[2] - w0[2] > 1.0:
            print("  contact", flush=True); break
    r.gripper(0.04)
    r.move_line([tgt[0], tgt[1], zt + TCP + 0.12], topdown_quat(yaw), step=0.03, speed=0.08, seed=q)
    go_home()
    scan()

r.close()
