import numpy as np
from rob import *
from plan2 import *
r=Rob("lifttest"); sols=np.load("sols.npy",allow_pickle=True).item()
def blob(c):
    P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    d=np.linalg.norm(P[:,:2]-c,axis=1); m=(d<0.06)&(P[:,2]>0.95)&(P[:,2]<1.2)
    return (P[m,:2].mean(0).round(3), P[m,2].min().round(3), P[m,2].max().round(3), int(m.sum())) if m.sum()>5 else None
print("fingers",r.fingers(),"tcp",r.tcp()[0].round(3))
# lift just 3 cm first
q=r.ik(P3(A,ZG+0.03),hand_R(-45),seed=r.arm_q(),at_tcp=True)
r.move_q(q,2.5); print("after +3cm: fingers",np.abs(r.fingers()).round(4),"blob",blob(A),"F",r.wrench()[:3].round(2))
r.snap("agentview","/workspace/lift3.png")
