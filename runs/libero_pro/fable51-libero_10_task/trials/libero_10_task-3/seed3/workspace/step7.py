import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)
Ta = make_T([0.08, 0.12, 1.20], R)
Tb = make_T([0.08, 0.12, 0.975], R)   # hook inside, behind front panel
Tc = make_T([0.08, 0.00, 0.975], R)   # pull -y 12 cm
Td = make_T([0.08, 0.00, 1.20], R)
qa = ik(Ta, q0); qb = ik(Tb, qa); qc = ik(Tc, qb); qd = ik(Td, qc)
for n,q in (('qa',qa),('qb',qb),('qc',qc),('qd',qd)):
    print(n, q.round(3), check(q, ignore=('drawer','bottle','bowl'), verbose=(n in('qb','qc'))))
print('paths', check_path([q0,qa], ignore=('bowl',)), check_path([qa,qb], ignore=('drawer','bottle','bowl')), check_path([qb,qc], ignore=('drawer','bottle','bowl')))
if '--go' in sys.argv:
    print(r.move_q(qa)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tb, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tc, n_wp=6)); print('tcp', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
    print(r.move_tcp(Td, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
