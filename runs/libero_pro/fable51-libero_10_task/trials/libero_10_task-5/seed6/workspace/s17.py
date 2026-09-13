import numpy as np, json
from ctl import *; from mp import *
from wr import Wrench
r=Robot("s17"); pl=Planner(r); w=Wrench(r)
goal=[-0.72,-0.68,0.41,-2.69,0.28,2.04,0.26]
tr=pl.plan_joints(goal, t=20.0, attempts=20)
if tr:
    idx=[tr.joint_names.index(j) for j in ARM]
    pts=np.array([[p.positions[i] for i in idx] for p in tr.points]); print("path len", len(pts), "total joint travel", np.abs(np.diff(pts,axis=0)).sum(0).round(2))
    pl.execute_steps(tr, stride=3, seconds=1.5)
    p,qq=r.hand_pose(); print("hand", p.round(4), qq.round(3), "fing", np.round(r.finger(),4), "wr", w.get())
    r.snap("agentview","/workspace/s17_agent.png"); r.snap("sideview","/workspace/s17_side.png")
