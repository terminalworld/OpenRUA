import numpy as np
from rob import Robot
r=Robot("st")
q=r.arm_q(); print("q", np.round(q,3))
p,_,R=r.tcp(); print("tcp", np.round(p,4), "z", np.round(R[:,2],3), "y", np.round(R[:,1],3))
print("fingers", np.round(r.finger(),4)); print("wrench", np.round(r.wrench()[0],2))
