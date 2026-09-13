import numpy as np
from rob import *
r = Robot("oritest")
q0 = r.arm_q()
pos = (-0.196, -0.200, 1.15)
for quat in [(1,0,0,0),(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0),(0,1,0,0),(0.9239,-0.3827,0,0),(0.3827,-0.9239,0,0)]:
    sol = r.ik_tcp_world(pos, quat, seed=q0, timeout=5)
    if sol is None: print(quat, "fail"); continue
    tcp, fq = r.tcp_world(sol)
    R = quat_to_R(*fq)
    print("req", quat, "-> fk quat", np.round(fq,3), "hand y (finger axis) in world", np.round(R[:,1],3), "j7", round(sol[6],3))
