import sys, numpy as np
from arm import Arm, down_quat, world_to_base
a = Arm("iktest")
x,y,z = map(float, sys.argv[1:4])
seeds = {"cur": a.arm_q(), "ready": np.array([0,-0.785,0,-2.356,0,1.571,0.785]),
         "ready2": np.array([-0.4,-0.3,0,-2.2,0,1.9,0.4]), "ready3": np.array([0.0,0.2,0,-2.0,0,2.2,0.785])}
for n,s in seeds.items():
    q = a.ik(world_to_base([x,y,z]), down_quat(0), seed=s)
    print(n, None if q is None else q.round(3), flush=True)
    if q is not None:
        print("   check tcp:", a.tcp_world(q)[0].round(4))
