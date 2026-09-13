import numpy as np
from ctl import *; from wr import Wrench
r=Robot("s19"); w=Wrench(r)
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
p,_=r.hand_pose(); tip=p+0.1034*np.array([-np.sin(np.radians(-10)),0,-np.cos(np.radians(-10))])
for dz,deg in [(0.03,-10),(0.07,-5),(0.12,0)]:
    t=tip+[0,0,dz]; pos,q=pose(t,deg); s=r.ik_solve(pos,q,r.arm_q(),collide=True)
    if s is None: print("noik"); break
    r.move_joints(s,seconds=2); print("hand", r.hand_pose()[0].round(4), "wr", w.get())
# let time pass: repeat current position twice (sim advances during commands)
q=r.arm_q()
for i in range(3): r.move_joints(q,seconds=2,retries=0)
print("settled wr", w.get())
