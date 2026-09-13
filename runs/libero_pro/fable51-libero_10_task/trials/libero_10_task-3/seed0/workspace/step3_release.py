import numpy as np
from rob import Robot
r=Robot("rel")
r.gripper(0.04)
print("wrench", np.round(r.wrench()[0],2))
q=r.arm_q()
p,_,R=r.tcp(); print("tcp", np.round(p,4))
# retreat straight back along -hand z (away from bottle) then up
q2=r.move_tcp(p-0.08*R[:,2]+[0,0,0.05],R,seed=q)
q3=r.move_tcp([-0.15,0.05,1.30],R,seed=q2)
