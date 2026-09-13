import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)
Ta = make_T([0.08, 0.10, 1.20], R)
Tb = make_T([0.08, 0.10, 0.975], R)
Tc = make_T([0.08, 0.07, 0.975], R)   # gentle 3 cm pull
Td = make_T([0.08, 0.07, 1.20], R)
qa = ik(Ta, q0); qb = ik(Tb, qa); qc = ik(Tc, qb); qd = ik(Td, qc)
print('paths', check_path([q0,qa], ignore=('bowl',)), check_path([qa,qb], ignore=('drawer','bottle','bowl')))
print(r.move_q(qa)); print(r.move_tcp(Tb, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
print(r.move_tcp(Tc, n_wp=3, vmax=0.05)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
print(r.move_tcp(Td, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
