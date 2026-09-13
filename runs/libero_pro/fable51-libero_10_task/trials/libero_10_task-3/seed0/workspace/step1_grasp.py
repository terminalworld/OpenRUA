import numpy as np
from rob import Robot, rot_from_axes
r=Robot("grasp")
BX,BY=-0.137,0.049
Rg=rot_from_axes([0,0,-1],[0,1,0])   # z down, fingers close along world x
r.gripper(0.04)
q=r.move_tcp([BX,BY,1.22],Rg,4.0)
q=r.move_tcp([BX,BY,1.00],Rg,3.0,seed=q)
print("wrench before close", np.round(r.wrench()[0],2))
f=r.gripper(0.0)
print("wrench after close", np.round(r.wrench()[0],2))
q=r.move_tcp([BX,BY,1.18],Rg,3.0,seed=q)
print("fingers after lift", np.round(r.finger(),4))
print("wrench after lift", np.round(r.wrench()[0],2))
