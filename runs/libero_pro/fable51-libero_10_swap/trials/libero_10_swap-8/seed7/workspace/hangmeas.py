import sys
from rob import *
from cloud import cloud
R_HOOK = np.array([[1.0,0,0],[0,0,1.0],[0,-1.0,0]])
r = Robot()
if len(sys.argv) > 1 and sys.argv[1] == "lift":
    pos, quat, t = r.fk_hand()
    q = r.ik_tcp((t[0], t[1], 1.22), R_HOOK, seed=r.arm_q())
    print("lift q", np.round(q,2), "jump", np.round(np.max(np.abs(np.array(q)-np.array(r.arm_q()))),2))
    code, err = r.move_q(q, 3.0)
    if err > 0.02: code, err = r.move_q(q, 3.0)
pos, quat, t = r.fk_hand(); print("tcp", np.round(t,4), "fingers", r.fingers(), flush=True)
finger_top = t[2] - 0.04
print("lower finger top z", round(finger_top,4), "upper finger bottom z", round(t[2]+0.04,4))
for cam in ("sideview", "frontview"):
    X, Y, Z, _ = cloud(cam)
    m = (X > -0.15) & (X < 0.25) & (Y > 0.05) & (Y < 0.45) & (Z > 0.905) & (Z < t[2] + 0.02)
    # exclude hand: hand body is at y > t[1]+0.03 roughly; pot hangs below/around fingers
    print(cam, "pts", m.sum())
    P = np.stack([X[m], Y[m], Z[m]], 1)
    lo = P[np.argmin(P[:,2])]; print("  lowest point", np.round(lo,3))
    for z0 in np.arange(0.90, t[2]+0.02, 0.02):
        s = m & (Z >= z0) & (Z < z0+0.02)
        if s.sum() > 3:
            print(f"  z {z0:.2f}-{z0+0.02:.2f}: n{s.sum():4d} x {X[s].min():.3f}..{X[s].max():.3f} (mean {X[s].mean():.3f})  y {Y[s].min():.3f}..{Y[s].max():.3f} (mean {Y[s].mean():.3f})")
