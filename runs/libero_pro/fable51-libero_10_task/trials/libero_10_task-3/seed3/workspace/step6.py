import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
R = topdown(math.pi/2)
Ta = make_T([-0.085, -0.03, 1.10], R)   # above start
Tb = make_T([-0.085, -0.03, 0.93], R)   # pusher down behind bowl (-x side)
Tc = make_T([ 0.13, -0.03, 0.93], R)    # push +x
Td = make_T([ 0.13, -0.03, 1.15], R)    # lift
qa = ik(Ta, q0); qb = ik(Tb, qa); qc = ik(Tc, qb); qd = ik(Td, qc)
for n,q in (('qa',qa),('qb',qb),('qc',qc),('qd',qd)):
    print(n, q.round(3), check(q, ignore=('bowl',), verbose=(n in('qb','qc'))))
print('paths', check_path([q0,qa]), check_path([qa,qb], ignore=('bowl',)), check_path([qb,qc], ignore=('bowl',)), check_path([qc,qd], ignore=('bowl',)))
if '--go' in sys.argv:
    r.gripper(0.0)
    print(r.move_q(qa)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tb, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tc, n_wp=5)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
    print(r.move_tcp(Td, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
