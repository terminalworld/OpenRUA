import numpy as np
from robot import Robot
r = Robot("cur")
q = r.arm_q(); t,quat = r.tcp(q); f = r.fingers(); w = r.wrench()
print("q", np.round(q,4).tolist()); print("tcp", np.round(t,4), "quat", np.round(quat,4)); print("fingers", f); print("wrench", np.round(w[0],2), np.round(w[1],2))
