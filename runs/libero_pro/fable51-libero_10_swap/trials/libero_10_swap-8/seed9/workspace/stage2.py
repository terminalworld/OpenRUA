import numpy as np, sys
from rob import *
r = Robot("stage2")
POT = np.array([-0.196, -0.200])
Q_G = (0.7071, -0.7071, 0, 0)   # fingers along world x, hand z down, hand x = world -y
q0 = r.arm_q()
pre = r.ik_tcp_world((POT[0], POT[1], 1.15), Q_G, seed=q0, timeout=5)
print("pre q", np.round(pre,3)); r.move_q(pre, 3.0)
tcp, fq = r.tcp_world(); print("tcp", np.round(tcp,4), "quat", np.round(fq,3))
# eye-in-hand depth -> refine pot center
d = r.depth("robot0_eye_in_hand"); r.snap("robot0_eye_in_hand")
res = tf_lookup(r.node, "robot0_eye_in_hand_optical_frame")
print("cam pose", None if res is None else (np.round(res[0],4), np.round(res[1],3)))
K = [-312.77408948188935,0,320,0,-312.77408948188935,240,0,0,1]
for sign in [1,-1]:
    Kt = list(K); Kt[0]*=sign; Kt[4]*=sign
    P = cloud_from_depth(d, Kt, *res)
    tab = P[(P[:,2]>0.85)&(P[:,2]<0.95)]
    top = P[(P[:,2]>1.0)&(P[:,2]<1.045)&(np.abs(P[:,0]-POT[0])<0.08)&(np.abs(P[:,1]-POT[1])<0.08)]
    print(f"sign {sign}: table pts {len(tab)} z med {np.median(tab[:,2]) if len(tab) else None}; top pts {len(top)}",
          "center", np.round(top[:,:2].mean(0),4) if len(top) else None,
          "x-range", (np.round(top[:,0].min(),3), np.round(top[:,0].max(),3)) if len(top) else None,
          "y-range", (np.round(top[:,1].min(),3), np.round(top[:,1].max(),3)) if len(top) else None)
    # body-only slice (exclude handle/spout): z 0.99..1.02
    body = P[(P[:,2]>0.985)&(P[:,2]<1.025)&(np.abs(P[:,0]-POT[0])<0.08)&(np.abs(P[:,1]-POT[1])<0.08)]
    if len(body): print("   body slice center", np.round(body[:,:2].mean(0),4), "x-range", np.round([body[:,0].min(),body[:,0].max()],3), "y-range", np.round([body[:,1].min(),body[:,1].max()],3))
