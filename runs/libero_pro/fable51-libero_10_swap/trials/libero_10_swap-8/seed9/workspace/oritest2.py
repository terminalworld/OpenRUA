import numpy as np
from rob import *
r = Robot("oritest2")
q0 = r.arm_q()
pos = (-0.196, -0.200, 1.15)
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
tests = [(1,0,0,0),(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0), tuple(R_to_quat(rot_y(np.radians(-10))@quat_to_R(1,0,0,0)))]
for quat in tests:
    sol = r.ik_tcp_world(pos, quat, seed=q0, timeout=5)
    if sol is None: print(quat, "fail"); continue
    tcp, fq = r.tcp_world(sol)
    R = quat_to_R(*fq)
    print("req", np.round(quat,3), "-> fk quat", np.round(fq,3), "tcp", np.round(tcp,3), "finger axis", np.round(R[:,1],3), "hand z", np.round(R[:,2],3), "j7", round(sol[6],3))
