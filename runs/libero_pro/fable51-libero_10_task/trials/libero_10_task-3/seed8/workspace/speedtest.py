from robot import *
r=Robot(); q=r.arm_q().copy(); q1=q.copy(); q1[0]+=0.8
r.move_q(q1,2.0); print(np.round(r.arm_q(),3))
r.move_q(q,2.0); print(np.round(r.arm_q(),3))
