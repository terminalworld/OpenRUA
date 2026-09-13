from robot import *
r=Robot(); q=r.arm_q().copy()
w1=q.copy(); w1[0]+=0.2; w2=q.copy(); w2[0]+=0.4
r.move_q(q,6.0,waypoints=[(w1,2.0),(w2,4.0)])
print(np.round(r.arm_q(),3))
