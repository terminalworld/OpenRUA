import numpy as np, json, sys
from ctl import *; from mp import *
from wr import Wrench
r=Robot("s14"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
def go(tip, degs=(0,3,6,-3,-6,10,-10), maxd=0.6, sec=None):
    seed=r.arm_q(); best=None
    for deg in degs:
        pos,q=pose(tip,deg); s=r.ik_solve(pos,q,seed,collide=True)
        if s is None: continue
        d=np.abs(np.array(s)-np.array(seed)).max()
        if d<maxd and (best is None or d<best[0]): best=(d,deg,s)
    if best is None: print("NO IK for",tip); return False
    d,deg,s=best; ok=r.move_joints(s, seconds=sec or max(1.5,d/0.3))
    p,_=r.hand_pose(); print(f"tip {np.round(tip,3)} tilt {deg} delta {d:.2f} ok={ok} hand={p.round(4)} fing={np.round(r.finger(),4)} wr={w.get()}",flush=True)
    return ok
wps=json.loads(sys.argv[1])
for tip in wps:
    if not go(tip): break
