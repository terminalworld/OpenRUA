"""Test-plan a door arc segment: doorplan2.py lean rho th0 th1"""
import sys, numpy as np, scene
from rob import *; from plan import Planner; from ikbest import best_ik
lean, rho, th0, th1 = map(float, sys.argv[1:5])
H = np.array([-0.165, 0.27]); Z = 1.0
def pose(th_deg):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d - 0.01*n1; tcp[2] = Z
    b = np.radians(lean); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
r = Planner("doorplan2")
ths = np.arange(th0, th1 + 0.1, 4.0)
targets = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
best = None
for k in range(10):
    rng = np.random.default_rng(k)
    q0 = best_ik(r, *pose(ths[0]), tries=4, seeds=[rng.uniform(-1.5, 1.5, 7)])
    if q0 is None: continue
    traj, frac = r.cartesian(targets, avoid=True, step=0.01, start_q=q0)
    print("start", np.round(q0,2), "fraction", round(frac,3))
    if best is None or frac > best[1]: best = (q0, frac)
    if frac >= 0.99: break
print("BEST", best[1] if best else None, np.round(best[0],3) if best else None)
