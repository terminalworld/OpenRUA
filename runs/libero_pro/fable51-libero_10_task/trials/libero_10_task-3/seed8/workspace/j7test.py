from robot import *
r=Robot(); q=r.arm_q().copy(); q[6]=-2.081
r.move_q(q,3.0); print(np.round(r.arm_q(),3)); print(r.wrench())
