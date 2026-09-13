"""Careful insertion of the hanging upright mug (after stand3.py stand). python3 insert5.py [cy] [--go]
1. carry level to y 0.05, descend to the 'front' pose, measure the mug (frontview/agentview clouds): base/top z, tilt.
2. re-centre the mug vertically in the opening (0.945 .. 1.089) if needed, re-measure.
3. advance in 3 cm steps, checking the hand tracks the commanded pose; stop and back off on deviation.
4. set down, release, retreat."""
import sys, subprocess
from common import *
c = Ctl("ins5"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
args = [float(a) for a in sys.argv[1:] if not a.startswith("--")]
CY = args[0] if args else 0.36
CX, OFF_Y = -0.03, 0.0755
FLOOR, TOP = 0.945, 1.089
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
def tips_now():
    p, R = r.fk(); return p + 0.1034 * R[:, 2], R
def line(name, t, R, n=4, T=4.0, tolerant=False):
    try:
        ok = c.line(origin(t, R), R, n=n, t=T)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        if not tolerant: raise
        print(name, e)
    r.report(name)
def measure():
    tips, R = tips_now(); best = None
    for cam in ("frontview", "agentview", "sideview"):
        r.snap(cam); subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 0] > CX - 0.08) & (pw[:, 0] < CX + 0.08) & (pw[:, 1] > tips[1] - 0.01) & (pw[:, 1] < tips[1] + 0.17) & (pw[:, 2] > 0.90) & (pw[:, 2] < 1.13)
        P = pw[m]
        if len(P) < 50: print(cam, "few pts", len(P)); continue
        zmin, zmax = P[:, 2].min(), P[:, 2].max()
        print(f"{cam}: n={len(P)} z[{zmin:.4f},{zmax:.4f}] (rel tips {zmin-tips[2]:+.4f},{zmax-tips[2]:+.4f}) height {zmax-zmin:.4f}")
        for z0 in np.arange(zmin, zmax, 0.02):
            s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.02)]
            if len(s) > 5: print(f"   z[{z0:.3f}] n={len(s)} y[{s[:,1].min()-tips[1]:+.3f},{s[:,1].max()-tips[1]:+.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
        if best is None or (zmax - zmin) > best[2] - best[1]: best = (cam, zmin, zmax)
    return best
tips, R = tips_now(); print("tips", np.round(tips, 4), "z_hand", np.round(R[:, 2], 3))
if dry: sys.exit()
if r.finger_gap() < 0.008: sys.exit("nothing held")
ZT = 1.014
line("carry", np.r_[CX, 0.05, tips[2]], R, n=6, T=8.0)
line("front", np.r_[CX, 0.15, ZT], R, n=6, T=8.0)
for it in range(3):
    b = measure()
    if b is None: sys.exit("cannot see the mug")
    cam, zmin, zmax = b
    if zmax - zmin < 0.095: print("mug not fully seen (height %.3f); trusting nominal base = top - 0.107" % (zmax - zmin)); zmin = zmax - 0.107
    shift = (FLOOR + TOP) / 2 - (zmin + zmax) / 2
    print(f"base {zmin:.4f} top {zmax:.4f} -> shift {shift:+.4f}")
    if abs(shift) < 0.004: break
    tips, R = tips_now(); ZT = tips[2] + shift
    line("recentre", np.r_[CX, 0.15, ZT], R, n=2, T=3.0)
base_dz = zmin - tips_now()[0][2]
print("gap", r.finger_gap(), "base_dz", round(base_dz, 4))
# advance in steps
y_goal = CY - OFF_Y
tips, R = tips_now(); y = tips[1]; ok = True
while y < y_goal - 0.002:
    y = min(y + 0.03, y_goal)
    tgt = np.r_[CX, y, ZT]
    try:
        c.line(origin(tgt, R), R, n=2, t=3.0)
    except RuntimeError as e:
        print("step", round(y, 3), e)
    tips2, R2 = tips_now(); dev = tips2 - tgt
    print(f"step y={y:.3f}: tips {np.round(tips2,4)} dev {np.round(dev,4)} gap {r.finger_gap():.4f}")
    if abs(dev[2]) > 0.006 or abs(dev[0]) > 0.006 or abs(dev[1]) > 0.006:
        print("DEVIATION - stopping"); ok = False
        r.snap("frontview", "/workspace/dev_f.png"); r.snap("agentview", "/workspace/dev_a.png")
        break
if ok:
    tips, R = tips_now()
    line("set", np.r_[CX, y_goal, FLOOR + 0.002 - base_dz], R, n=2, T=2.5, tolerant=True)
    r.gripper(0.08)
    line("back", np.r_[CX, 0.10, ZT], R, n=4, T=4.0); line("up", np.r_[CX, 0.05, 1.25], R, n=3, T=3.0)
else:
    tips, R = tips_now()
    line("backoff", np.r_[CX, tips[1] - 0.04, ZT], R, n=2, T=3.0, tolerant=True)
r.snap("agentview"); r.snap("frontview")
