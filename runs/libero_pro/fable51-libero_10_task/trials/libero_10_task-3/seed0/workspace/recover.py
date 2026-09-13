import numpy as np
from rob import Robot
r=Robot("rec")
p,_,R=r.tcp(); print("tcp", np.round(p,4))
q=r.move_tcp(p+[0,0,0.12],R,max_jump=0.6)
print("wrench", np.round(r.wrench()[0],2))
