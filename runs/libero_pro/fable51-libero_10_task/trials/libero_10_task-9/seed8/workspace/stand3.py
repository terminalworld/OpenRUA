"""Phases (python3 stand3.py <phase> [args] [--go]):
  pull  bx by bz        : lying mug in cavity, bar on top along y. Rigid bar pinch with hand pointing +y (fingers
                          close along x), drag it out along -y, carry to LAY, lower, release.
  stand bx by           : lying mug on table (axis along y, rim +y, bar on top). Pinch bar from above (close along
                          x), lift, rotate about x -> hand points +y, mug hangs upright (handle -y). Then hold.
  measure               : snap sideview/agentview clouds, report hanging mug extents and tilt.
  pitch  deg            : rotate the hand about its y_hand (x world) by deg (compensate droop), keep tips fixed.
  insert base_dz top_dz cy: carry into cavity (tips y = cy - OFF_Y), set down, release, retreat.
"""
import sys, subprocess
from common import *
c = Ctl("stand3"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
phase = sys.argv[1]; args = [float(a) for a in sys.argv[2:] if not a.startswith("--")]
Rf = R_from_axes(np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))    # z=+y, y=+x, x=+z
Rb = R_from_axes(np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), np.array([0, 0, -1.0]))   # vertical, close along x
def Rx(th):
    th = np.radians(th); return np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]])
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
LAY = np.array([-0.05, -0.17])
def tips_now():
    p, R = r.fk(); return p + 0.1034 * R[:, 2], R
def plan(seq):
    q = r.arm_q(); ok = True
    for name, t, R in seq:
        qq = c.ik_valid(origin(t, R), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q; ok &= qq is not None
    return ok
def go(name, t, R, T=5.0):
    qq = c.ik_valid(origin(t, R), R, seed=r.arm_q(), tries=8)
    assert qq is not None and c.goto_q(qq, t=T), name + " failed"; r.report(name)
def line(name, t, R, n=4, T=4.0, tolerant=False):
    try:
        ok = c.line(origin(t, R), R, n=n, t=T)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        if not tolerant: raise
        print(name, e)
    r.report(name)

if phase == "pull":
    bx, by, bz = args[:3]
    seq = [("pre", np.r_[bx, 0.10, bz], Rf), ("in", np.r_[bx, by, bz], Rf), ("drag", np.r_[bx, 0.0, bz], Rf),
           ("carry", np.r_[LAY, bz], Rf), ("lower", np.r_[LAY, 1.018], Rf), ("up", np.r_[LAY, 1.15], Rf)]
    ok = plan(seq)
    if dry or not ok: sys.exit()
    if r.finger_gap() < 0.07: r.gripper(0.08)
    go(*seq[0]); line(*seq[1], n=5, T=5.0, tolerant=True)
    gap = r.gripper(0.0); print("gap", gap)
    if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
    line(*seq[2], n=6, T=7.0, tolerant=True); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0, tolerant=True)
    r.gripper(0.08); line(*seq[5], n=3, T=3.0)
    r.snap("agentview")
elif phase == "stand":
    bx, by = args[:2]
    R45 = Rx(45) @ Rb
    seq = [("above", np.r_[bx, by, 1.10], Rb), ("down", np.r_[bx, by, 1.013], Rb), ("lift", np.r_[bx, by, 1.15], Rb),
           ("rot45", np.r_[bx, by, 1.15], R45), ("rot90", np.r_[bx, by, 1.15], Rf)]
    ok = plan(seq)
    if dry or not ok: sys.exit()
    if r.finger_gap() < 0.07: r.gripper(0.08)
    go(*seq[0]); line(*seq[1], tolerant=True)
    gap = r.gripper(0.0); print("gap", gap)
    if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
    line(*seq[2], T=5.0); go(*seq[3], T=7.0); go(*seq[4], T=7.0)
    print("gap now", r.finger_gap())
    r.snap("agentview", "/workspace/hang_a.png"); r.snap("sideview", "/workspace/hang_s.png")
elif phase == "measure":
    tips, R = tips_now(); print("tips", np.round(tips, 4), "z_hand", np.round(R[:, 2], 3), "x_hand", np.round(R[:, 0], 3))
    for cam in ("sideview", "agentview"):
        r.snap(cam); subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (np.abs(pw[:, 0] - tips[0]) < 0.08) & (pw[:, 1] > tips[1] - 0.02) & (pw[:, 1] < tips[1] + 0.16) & (pw[:, 2] > tips[2] - 0.12) & (pw[:, 2] < tips[2] + 0.12)
        P = pw[m]; print(cam, "pts", len(P))
        if len(P) < 50: continue
        print("  z rel tips: min %.4f max %.4f" % (P[:, 2].min() - tips[2], P[:, 2].max() - tips[2]))
        print("  y rel tips: min %.4f max %.4f" % (P[:, 1].min() - tips[1], P[:, 1].max() - tips[1]))
        zs = np.arange(P[:, 2].min(), P[:, 2].max(), 0.01)
        for z0 in zs:
            s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.01)]
            if len(s) > 5: print(f"    z[{z0-tips[2]:+.3f}] n={len(s)} y[{s[:,1].min()-tips[1]:+.3f},{s[:,1].max()-tips[1]:+.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
elif phase == "pitch":
    deg = args[0]
    tips, R = tips_now(); Rn = Rx(deg) @ R
    ok = plan([("pitch", tips, Rn)])
    if dry or not ok: sys.exit()
    line("pitch", tips, Rn, n=3, T=3.0); print("gap now", r.finger_gap())
elif phase == "insert":
    base_dz, top_dz, cy = args[:3]          # mug bottom / top z relative to tips (from measure)
    OFF_Y = 0.0755
    zt = 1.0145 - (base_dz + top_dz) / 2    # centre the mug vertically in the cavity (0.942..1.087)
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now(); CX = -0.03
    seq = [("carry", np.r_[CX, 0.05, zt + 0.05], R), ("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - OFF_Y, zt], R),
           ("set", np.r_[CX, cy - OFF_Y, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    ok = plan(seq)
    if dry or not ok: sys.exit()
    go(*seq[0], T=6.0); line(*seq[1], n=4, T=5.0); line(*seq[2], n=6, T=7.0, tolerant=True); line(*seq[3], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[4], n=4, T=4.0); line(*seq[5], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
elif phase == "insert2":
    cy = args[0] if args else 0.36
    def top_ring():
        tips, R = tips_now()
        r.snap("sideview"); subprocess.run(["python3", "cloud.py", "sideview"], capture_output=True)
        d = np.load("sideview_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (np.abs(pw[:, 0] - tips[0]) < 0.08) & (pw[:, 1] > tips[1] - 0.02) & (pw[:, 1] < tips[1] + 0.16) & (pw[:, 2] > tips[2] - 0.12) & (pw[:, 2] < tips[2] + 0.12)
        P = pw[m]
        if len(P) < 30: print("top_ring: few pts", len(P)); return None
        zt = P[:, 2].max(); top = P[P[:, 2] > zt - 0.006]
        yc = (top[:, 1].min() + top[:, 1].max()) / 2 - tips[1]
        print(f"top ring: n={len(top)} top_dz={zt - tips[2]:+.4f} yc_rel={yc:+.4f} y[{top[:,1].min()-tips[1]:+.3f},{top[:,1].max()-tips[1]:+.3f}]")
        return zt - tips[2], yc
    tips, R = tips_now(); CX = -0.03
    line("carry", np.r_[CX, 0.05, tips[2]], R, n=8, T=10.0)
    m = top_ring()
    if m is None: sys.exit("cannot see mug")
    top_dz, yc = m
    alpha = np.degrees(np.arcsin(np.clip((yc - 0.0755) / 0.057, -1, 1))); print("droop est deg", round(alpha, 1))
    if abs(alpha) > 5:
        tips, R = tips_now(); Rn = Rx(alpha) @ R
        line("pitchfix", tips, Rn, n=3, T=4.0); print("gap now", r.finger_gap())
        m = top_ring(); top_dz, yc = m if m else (top_dz, yc)
    base_dz = top_dz - 0.107
    zt = 1.0145 - (base_dz + top_dz) / 2
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now()
    seq = [("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - 0.0755, zt], R),
           ("set", np.r_[CX, cy - 0.0755, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    if not plan(seq): sys.exit("plan failed")
    line(*seq[0], n=4, T=5.0); line(*seq[1], n=6, T=8.0, tolerant=True); r.report("in"); line(*seq[2], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
elif phase == "insert3":
    cy = args[0] if args else 0.36
    def top_ring(cam):
        tips, R = tips_now()
        r.snap(cam); subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (np.abs(pw[:, 0] - tips[0]) < 0.08) & (pw[:, 1] > tips[1] - 0.02) & (pw[:, 1] < tips[1] + 0.16) & (pw[:, 2] > tips[2] - 0.12) & (pw[:, 2] < tips[2] + 0.12)
        P = pw[m]
        if len(P) < 30: print(cam, "top_ring: few pts", len(P)); return None
        zt = P[:, 2].max(); top = P[P[:, 2] > zt - 0.006]
        w = top[:, 1].max() - top[:, 1].min(); yc = (top[:, 1].min() + top[:, 1].max()) / 2 - tips[1]
        print(f"{cam} top ring: n={len(top)} top_dz={zt - tips[2]:+.4f} yc_rel={yc:+.4f} width={w:.3f} zmin_rel={P[:,2].min()-tips[2]:+.4f}")
        if w < 0.08: print("  (partial ring, unreliable)"); return None
        return zt - tips[2], yc
    m0 = top_ring("sideview")
    tips, R = tips_now(); CX = -0.03
    line("carry", np.r_[CX, 0.05, tips[2]], R, n=8, T=10.0)
    ms = [top_ring(cam) for cam in ("sideview", "agentview")]
    ms = [m for m in ms if m] or ([m0] if m0 else [])
    if not ms: sys.exit("no reliable measurement")
    top_dz = np.mean([m[0] for m in ms]); yc = np.mean([m[1] for m in ms])
    alpha = np.degrees(np.arcsin(np.clip((yc - 0.0755) / 0.057, -1, 1))); print("droop est deg", round(alpha, 1), "top_dz", round(top_dz, 4))
    if abs(alpha) > 8: sys.exit("droop too large; aborting insertion")
    base_dz = top_dz - 0.107
    zt = 1.0145 - (base_dz + top_dz) / 2
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now()
    seq = [("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - 0.0755, zt], R),
           ("set", np.r_[CX, cy - 0.0755, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    if not plan(seq): sys.exit("plan failed")
    line(*seq[0], n=4, T=6.0); line(*seq[1], n=6, T=10.0, tolerant=True); line(*seq[2], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
elif phase == "insert4":
    base_dz, top_dz, cy = args[:3]
    zt = 1.0145 - (base_dz + top_dz) / 2
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now(); CX = -0.03
    seq = [("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - 0.0755, zt], R),
           ("set", np.r_[CX, cy - 0.0755, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    if not plan(seq): sys.exit("plan failed")
    if dry: sys.exit()
    line(*seq[0], n=5, T=8.0); line(*seq[1], n=6, T=10.0, tolerant=True); line(*seq[2], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
