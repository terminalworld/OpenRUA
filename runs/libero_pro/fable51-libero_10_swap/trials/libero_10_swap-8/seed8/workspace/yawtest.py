import numpy as np
from rob import *
from plan import *
r=Rob("yaw")
for name,xy in (("A",A),("B",B)):
    q=r.ik(P3(xy,ZL),hand_R(-45),seed=r.arm_q(),at_tcp=True)
    print(name,"start",q.round(2))
    prev=q
    for az in range(-35,50,10):
        q=r.ik(P3(xy,ZL),hand_R(az),seed=prev,at_tcp=True)
        if q is None: print(f"  az {az}: FAIL"); break
        print(f"  az {az}: q={q.round(2)} dq_max={np.abs(q-prev).max():.2f}")
        prev=q
