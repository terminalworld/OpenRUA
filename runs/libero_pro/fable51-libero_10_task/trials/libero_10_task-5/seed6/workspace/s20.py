import numpy as np, sys
from ctl import *; from wr import Wrench
r=Robot("s20"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
def go(tip,deg=0,maxd=0.7,sec=None):
    seed=r.arm_q(); pos,q=pose(tip,deg); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("NO IK",np.round(tip,3)); return False
    d=np.abs(np.array(s)-np.array(seed)).max()
    if d>maxd: print("too far",d, np.round(s,2)); return False
    ok=r.move_joints(s,seconds=sec or max(1.5,d/0.3)); p,_=r.hand_pose()
    print(f"tip {np.round(tip,3)} d {d:.2f} ok={ok} hand={p.round(4)} fing={np.round(r.finger(),4)} wr={w.get()}",flush=True); return ok
RX,RY,RZTOP=-0.407,-0.129,1.097
print("fingers", r.finger()); r.gripper(0.08)
pin=np.array([RX,RY+0.0485,RZTOP-0.02])
for z in [1.16,1.12,1.095,pin[2]]:
    if not go([pin[0],pin[1],z]): sys.exit()
r.snap("robot0_eye_in_hand","/workspace/s20_eih.png")
