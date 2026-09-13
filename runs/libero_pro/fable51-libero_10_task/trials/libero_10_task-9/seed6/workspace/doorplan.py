"""Test-plan door closing arcs: doorplan.py lean_deg rho [execute]"""
import sys, numpy as np, scene
from rob import *; from plan import Planner; from ikbest import best_ik
lean = float(sys.argv[1]); rho = float(sys.argv[2]); execute = len(sys.argv) > 3
H = np.array([-0.165, 0.27]); Z = 1.0
def pose(th_deg):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d - 0.01*n1; tcp[2] = Z
    b = np.radians(lean); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
r = Planner("doorplan")
scene.apply(r.node, [scene.remove("mw_door")])
ths = np.arange(-114, 2.1, 4.0)
targets = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
best = None
for k in range(12):
    q0 = best_ik(r, *pose(ths[0]), tries=1 if k == 0 else 3, seeds=[np.random.default_rng(k).uniform(-1.5, 1.5, 7)]) if k else best_ik(r, *pose(ths[0]), tries=10)
    if q0 is None: continue
    traj, frac = r.cartesian(targets, avoid=True, step=0.01, start_q=q0)
    print("start", np.round(q0,2), "fraction", round(frac,3))
    if best is None or frac > best[1]: best = (q0, frac, traj)
    if frac >= 0.99: break
print("BEST fraction", best[1] if best else None)
