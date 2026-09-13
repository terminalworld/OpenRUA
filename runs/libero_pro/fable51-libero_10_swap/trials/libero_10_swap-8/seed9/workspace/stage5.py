import numpy as np, sys
from rob import *
r = Robot("stage5")
C = np.array([-0.035, 0.2385])
Q_G = (0.7071, -0.7071, 0, 0)
def go(tcp, quat, seconds=3.0, via=None):
    seed = r.arm_q()
    q = r.ik_tcp_world(tcp, quat, seed=seed, timeout=5)
    if q is None: raise SystemExit(f"IK fail {tcp}")
    vias = []
    if via:
        s = seed
        for p in via:
            s = r.ik_tcp_world(p, quat, seed=s, timeout=5)
            if s is None: raise SystemExit(f"IK fail via {p}")
            vias.append(s)
    code, err = r.move_q(q, seconds, via=vias or None)
    if err > 0.01:
        print("  resending"); code, err = r.move_q(q, seconds)
    tcp_now, fq = r.tcp_world(); R = quat_to_R(*fq)
    print("  tcp", np.round(tcp_now,4), "hz", np.round(R[:,2],3), "fingers", np.round(r.fingers(),4), flush=True)
    return q
cur,_ = r.tcp_world()
mid = (cur + np.array([C[0],C[1],1.25]))/2; mid[2]=1.30
go((C[0], C[1], 1.25), Q_G, 5.0, via=[mid])
# birdview check of pot A on the stove
d = r.depth("birdview"); r.snap("birdview", "/workspace/birdview_s5.png")
fx=579.4112549695428; T=np.array([-0.2,0,3.0])
v,u = np.mgrid[0:480,0:640]
pc = np.stack([(u-320)*d/fx,(v-240)*d/fx,d],-1).reshape(-1,3)
P = np.stack([pc[:,1], pc[:,0], -pc[:,2]],-1) + T
stove = P[(P[:,0]>0.10)&(P[:,0]<0.31)&(P[:,1]>-0.06)&(P[:,1]<0.14)]
top = stove[(stove[:,2]>1.05)&(stove[:,2]<1.08)]
knob = stove[stove[:,2]>1.08]
print("pot A on stove: lid pts", len(top), "lid center", np.round(top[:,:2].mean(0),4) if len(top) else None, "lid z", np.round(np.median(top[:,2]),4) if len(top) else None,
      "knob center", np.round(knob[:,:2].mean(0),4) if len(knob) else None, "zmax", np.round(stove[:,2].max(),4))
# eye-in-hand refine pot B knob
dh = r.depth("robot0_eye_in_hand"); r.snap("robot0_eye_in_hand", "/workspace/eih_B.png")
res = tf_lookup(r.node, "robot0_eye_in_hand_optical_frame")
K = [312.77408948188935,0,320,0,312.77408948188935,240,0,0,1]
Ph = cloud_from_depth(dh, K, *res)
Q = Ph[(np.abs(Ph[:,0]-C[0])<0.09)&(np.abs(Ph[:,1]-C[1])<0.09)]
for z0 in [1.025,1.030,1.035,1.040,1.045,1.050,1.055]:
    s = Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)]
    if len(s)<5: continue
    print(f"z {z0:.3f} n={len(s):5d} x[{s[:,0].min():.4f},{s[:,0].max():.4f}] y[{s[:,1].min():.4f},{s[:,1].max():.4f}] cx={s[:,0].mean():.4f} cy={s[:,1].mean():.4f}")
knobB = Q[Q[:,2]>1.045]
print("knob B center", np.round(knobB[:,:2].mean(0),4), "zmax", np.round(Q[:,2].max(),4))
np.save("/workspace/knobB.npy", knobB[:,:2].mean(0))
