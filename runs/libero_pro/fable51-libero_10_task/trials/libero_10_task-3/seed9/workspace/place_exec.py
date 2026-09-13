import numpy as np, sys
from rob import *
from goto import ik_checked, margin
from scene import check, R_for

r = Robot("pe")
q0 = r.arm_q(); t0, R0 = r.tcp()
print("start tcp", np.round(t0, 3), "gap", round(r.finger_gap(), 4))
u = np.array([-1.0, 0, 0])
Rp = R_for(R0, u, 0.0)
np.save("snaps/R_place.npy", Rp)
G_REL = np.array([-0.0825, 0.14, 0.972])
G_HIGH = G_REL + [0, 0, 0.12]


def safe_path(tcp0, Ra, tcp1, Rb, n, seed):
    path = cart_path(r, tcp0, Ra, tcp1, Rb, n, seed, max_step=0.6)
    if path is None:
        return None
    for i, q in enumerate(path):
        c = check(r, q)
        print(f"   via {i}: margin={margin(q):.2f} clear={c[0]:.3f} {c[1]}")
        if c[0] < -0.01:
            print("   !! unsafe via point"); return None
    return path


def run(path, seconds):
    r.move_q(path[-1], seconds, via=path[:-1])
    t, _ = r.tcp(); print("   tcp", np.round(t, 4), flush=True)
    return t


# A. rotate in place
print("A rotate")
pA = safe_path(t0, R0, t0, Rp, 8, q0)
if pA is None: sys.exit("A failed")
run(pA, 8.0)
r.snap("sideview", "/workspace/snaps/sv_rot.png"); r.snap("agentview", "/workspace/snaps/av_rot.png")
print("gap", round(r.finger_gap(), 4))
# B. move to above release
print("B transit")
pB = safe_path(t0, Rp, G_HIGH, Rp, 3, r.arm_q())
if pB is None: sys.exit("B failed")
run(pB, 5.0)
r.snap("agentview", "/workspace/snaps/av_high.png")
# C. descend
print("C descend")
pC = safe_path(G_HIGH, Rp, G_REL, Rp, 3, r.arm_q())
if pC is None: sys.exit("C failed")
run(pC, 4.0)
print("gap", round(r.finger_gap(), 4), "force", np.round(r.force(), 2))
r.snap("agentview", "/workspace/snaps/av_rel.png"); r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_rel.png")
