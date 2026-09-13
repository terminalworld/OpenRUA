"""Collect start configs giving full-fraction door arc: doorplan3.py lean rho th0 th1 nseeds"""
import sys, numpy as np, scene
from rob import *; from plan import Planner
lean, rho, th0, th1 = map(float, sys.argv[1:5]); nseeds = int(sys.argv[5]); Z = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0
H = np.array([-0.165, 0.27]); OFF = -0.032
lim = np.array(ARM["limits_rad"])
def pose(th_deg):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d + OFF*n1; tcp[2] = Z
    b = np.radians(lean); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
r = Planner("doorplan3")
scene.apply(r.node, [scene.remove("mw_door")])
ths = np.arange(th0, th1 + 0.1, 4.0)
targets = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
tcp0, R0 = pose(ths[0]); good = []
rng = np.random.default_rng(1)
for k in range(nseeds):
    seed = rng.uniform(lim[:,0]+0.1, lim[:,1]-0.1)
    q0 = r.solve_ik(hand_pose_from_tcp(tcp0, R0), R0, seed=list(seed), timeout=1.0)
    if q0 is None: continue
    q0 = np.array(q0); margin = np.min(np.minimum(q0-lim[:,0], lim[:,1]-q0))
    if margin < 0.15: continue
    traj, frac = r.cartesian(targets, avoid=True, step=0.01, start_q=q0)
    if frac >= 0.99:
        qs = np.array([p.positions for p in traj.points]); m2 = np.min(np.minimum(qs-lim[:,0], lim[:,1]-qs))
        good.append((m2, q0)); print("GOOD start", np.round(q0,2), "min limit margin along path", round(m2,3))
print("found", len(good))
if good:
    best = max(good, key=lambda g: g[0]); print("BEST", np.round(best[1],3), "margin", round(best[0],3))
