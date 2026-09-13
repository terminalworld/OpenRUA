from robot import *
import sys
r=Robot(); q=r.arm_q().copy(); q[0]+=0.3
print('7s move:'); r.move_q(q,7.0)
q[0]-=0.3
print('2s move back:'); r.move_q(q,2.0)
print(np.round(r.arm_q(),3))
