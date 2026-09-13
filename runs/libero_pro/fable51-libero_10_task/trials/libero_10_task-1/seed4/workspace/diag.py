import numpy as np, sys
from ctl import Ctl, ARM
import run
c=Ctl()
cur=np.array(c.arm_q()); print('current', np.round(cur,3))
sol=c.solve_ik([-0.12,-0.156,0.452], run.TOPDOWN); print('ik sol ', np.round(sol,3))
print('diff   ', np.round(np.array(sol)-cur,3))
lim=[[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]]
print('limits ', lim)
c.close()
