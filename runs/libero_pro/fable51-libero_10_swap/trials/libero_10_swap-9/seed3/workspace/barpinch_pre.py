import numpy as np, rob, sys
P=np.load("barpinch_pose.npy"); H=P[:3]; quat=list(P[3:7]); zh=P[7:10]
r=rob.Robot(); q_grasp=[-0.567,1.23,-0.11,-0.742,-0.001,1.389,1.376]
cands={"back5":H-0.05*zh,"back3":H-0.03*zh,"up6":H+np.array([0,0,0.06]),"up4back3":H-0.03*zh+np.array([0,0,0.04]),"up8":H+np.array([0,0,0.08])}
for k,p in cands.items():
    try: q=r.ik(list(p),quat,seed=q_grasp,timeout=3,avoid=True)
    except Exception as e: q=None
    print(k,np.round(p,3),None if q is None else np.round(q,3))
