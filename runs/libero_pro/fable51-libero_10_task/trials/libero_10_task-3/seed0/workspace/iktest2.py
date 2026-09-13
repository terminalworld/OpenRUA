import numpy as np
from rob import Robot, rot_from_axes, R_to_quat
r=Robot("ikt2")
Rg=rot_from_axes([0,0,-1],[1,0,0])
s=r.ik_tcp([-0.14,0.049,1.10],Rg)
p,quat,R=r.tcp(s)
print("sol",np.round(s,3)); print("tcp",np.round(p,4),"quat",np.round(quat,4)); print("R\n",np.round(R,3))
