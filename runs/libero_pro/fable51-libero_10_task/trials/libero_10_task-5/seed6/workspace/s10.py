import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s10"); pl=Planner(r); go="--go" in sys.argv
chain=json.load(open("chain2.json")); top=chain[0][2]
tr=None
for i in range(4):
    tr=pl.plan_joints(top,t=30.0,attempts=4)
    if tr: break
if tr is None: sys.exit(1)
P=np.array([p.positions for p in tr.points]); print("min",P.min(0).round(2),"max",P.max(0).round(2))
if go:
    pl.execute_steps(tr); p,q=r.hand_pose(); print("hand",p.round(4),q.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s10_eih.png"); r.snap("sideview","/workspace/s10_side.png")
