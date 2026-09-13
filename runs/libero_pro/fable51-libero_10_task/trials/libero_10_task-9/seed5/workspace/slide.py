import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('slide')
prev=np.array(r.arm_q()); print('start tcp',np.round(r.tcp()[0],4))
qh=hand_quat([0,0,-1],[0,1,0])
r.gripper(0.0)
def go(p,secs=2.5):
    global prev
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',p); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
y=-0.28; z=0.973
go((0.0,y,1.08)); go((0.0,y,z))
for x in np.arange(-0.03,-0.19,-0.03):
    go((x,y,z),2.0)
go((-0.18,y,1.1)); go((-0.10,-0.05,1.30),3.0)
