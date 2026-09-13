import numpy as np, sys, json, time, pickle
from ctl import *; from mp import *
r=Robot("s5"); pl=Planner(r); go="--go" in sys.argv
pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]); print("scene",pl.scene(scene_objects()))
chain=json.load(open("chain.json"))
goal=chain[1][2]  # (1.15,-30)
tr=None
for i in range(4):
    t0=time.time(); tr=pl.plan_joints(goal,t=40.0,attempts=4); print("took %.1f"%(time.time()-t0))
    if tr: break
if tr is None: sys.exit(1)
P=np.array([p.positions for p in tr.points]); print("min",P.min(0).round(2),"max",P.max(0).round(2))
pickle.dump(tr,open("tr_to_top.pkl","wb"))
if go:
    pl.execute(tr); p,q=r.hand_pose(); print("hand",p.round(4),q.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s5_eih.png"); r.snap("sideview","/workspace/s5_side.png")
