import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('park')
prev=np.array(r.arm_q()); print('start tcp',np.round(r.tcp()[0],4),'q',np.round(prev,3))
qh=hand_quat([0,0,-1],[0,1,0])
# retreat to a pose far from mug region so birdview sees the mug: over -x side, high
for p in [(-0.10,-0.25,1.25),(-0.35,-0.05,1.30)]:
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',p); break
    r.move_joints(s,3.0); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4))
