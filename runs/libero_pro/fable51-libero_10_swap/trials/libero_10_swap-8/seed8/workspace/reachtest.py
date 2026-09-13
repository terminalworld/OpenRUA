import numpy as np
from rob import *
from plan import hand_R, P3, u
r=Rob("reach")
seed0=np.array([-0.24,0.71,0.07,-2.51,-2.54,1.48,0.79])  # A_grasp-like (q5<0 branch)
for xy in ((0.225,0.05),(0.23,0.05),(0.235,0.05),(0.225,0.07)):
    for az in (-45,-35,-25,-15):
        for z in (1.014,1.10):
            q=r.ik(P3(xy,z),hand_R(az),seed=seed0,at_tcp=True)
            print(xy,az,z, None if q is None else q.round(2))
