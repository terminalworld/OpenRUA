import numpy as np, json, sys
from ctl import *; from mp import *
from wr import Wrench
r=Robot("s18"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
TH=float(sys.argv[1]); XB=float(sys.argv[2]); YR=-0.143
def tip_for_base(zb):
    th=np.radians(TH); a=np.array([-np.sin(th),0,np.cos(th)]); B=np.array([XB,YR,zb])
    rim=B+0.11*a; pinch=rim+np.array([0,0.0485,0]); return pinch-0.02*a
def go(tip, deg, maxd=0.6):
    seed=r.arm_q(); pos,q=pose(tip,deg); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("NO IK",np.round(tip,3)); return False
    d=np.abs(np.array(s)-np.array(seed)).max()
    if d>maxd: print("too far",d); return False
    ok=r.move_joints(s,seconds=max(1.5,d/0.3)); p,_=r.hand_pose()
    print(f"tip {np.round(tip,3)} tilt {deg} d {d:.2f} ok={ok} hand={p.round(4)} fing={np.round(r.finger(),4)} wr={w.get()}",flush=True); return ok
zs=[float(v) for v in sys.argv[3].split(",")]
for zb in zs:
    if not go(tip_for_base(zb), -TH): break
    wz=w.get()[2]
    if wz>-4.6: print("CONTACT? fz",wz); break
