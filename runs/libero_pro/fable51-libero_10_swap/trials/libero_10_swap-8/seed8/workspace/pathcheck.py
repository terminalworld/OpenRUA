import numpy as np
from rob import *
from plan2 import *
r=Rob("pathcheck")
sols=np.load("sols.npy",allow_pickle=True).item()
names=[n for n,_,_ in SEQ]
prev_n="ready"; prev_q=r.arm_q()
for n in names:
    q=sols[n]; worst=(0,0,9); 
    for s in np.linspace(0,1,9)[1:-1]:
        qi=prev_q+(q-prev_q)*s
        p,R=r.tcp(qi)
        tilt=np.degrees(np.arcsin(abs(R[2,2])))        # approach axis off horizontal
        roll=np.degrees(np.arccos(np.clip(-R[2,0],-1,1)))  # hand x vs straight down
        worst=(max(worst[0],tilt),max(worst[1],roll),min(worst[2],p[2]))
    print(f"{prev_n:10s}->{n:10s} max_tilt={worst[0]:5.1f} max_roll={worst[1]:5.1f} min_z={worst[2]:.3f}")
    prev_n,prev_q=n,q
