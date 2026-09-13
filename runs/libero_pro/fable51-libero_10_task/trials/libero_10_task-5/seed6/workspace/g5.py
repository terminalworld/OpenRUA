import numpy as np, sys
from ctl import *
r=Robot("g5")
execute = "--go" in sys.argv
q90=topdown_quat(np.pi/2); q0=topdown_quat(0.0)
wps=[([-0.0738,0.045,1.32],q90),
     ([-0.0738,0.045,1.32],q0),
     ([-0.25,-0.05,1.32],q0),
     ([-0.444,-0.1366,1.32],q0)]
seed=r.arm_q()
sols=[]
for pos,q in wps:
    s=r.ik_solve(pos,q,seed)
    if s is None: print("IK FAIL",pos); sys.exit(1)
    d=np.abs(np.array(s)-np.array(seed)); print(pos, "max joint delta %.2f"%d.max(), np.round(s,2))
    sols.append(s); seed=s
if execute:
    for (pos,q),s in zip(wps,sols):
        r.move_joints(s,seconds=4)
        p,qq=r.hand_pose(); print("  hand",p.round(4),qq.round(3),"fingers",np.round(r.finger(),4))
    r.snap("frontview","/workspace/g5_front.png"); r.snap("agentview","/workspace/g5_agent.png")
