import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)
T1 = make_T([-0.132, 0.058, 1.20], R)     # lift
T2 = make_T([-0.04, 0.14, 1.20], R)       # over drawer open area
T3 = make_T([-0.04, 0.14, 1.052], R)      # bottle base ~4 mm above drawer floor (0.923)
q1 = ik(T1, q0); q2 = ik(T2, q1); q3 = ik(T3, q2)
for n,q in (('q1',q1),('q2',q2),('q3',q3)):
    print(n, q.round(3), check(q, ignore=('bottle','drawer'), verbose=(n=='q3')))
print('path', check_path([q0,q1,q2], ignore=('bottle',)), check_path([q2,q3], ignore=('bottle','drawer')))
if '--go' in sys.argv:
    print(r.move_tcp(T1, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers())
    print(r.move_q(q2)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers())
    print(r.move_tcp(T3, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers(), 'wrench', r.wrench().round(2))
    r.gripper(0.04)
    print(r.move_tcp(T2, n_wp=2)); print('tcp', r.tcp()[:3,3].round(4))
