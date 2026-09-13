import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
# lift straight up
T1 = make_T([0.050, -0.029, 1.30], topdown(math.pi/2))
T2 = make_T([-0.35, 0.30, 1.30], topdown(math.pi/2))
T3 = make_T([-0.35, 0.30, 1.00], topdown(math.pi/2))
q1 = ik(T1, q0); q2 = ik(T2, q1); q3 = ik(T3, q2)
for n,q in (('q1',q1),('q2',q2),('q3',q3)):
    print(n, q.round(3), check(q, ignore=('bowl',)))
print('path', check_path([q0,q1,q2,q3], ignore=('bowl',)))
if '--go' in sys.argv:
    print(r.move_tcp(T1, n_wp=3)); print('fingers', r.fingers())
    print(r.move_q(q2)); print('tcp', r.tcp()[:3,3].round(3), 'fingers', r.fingers())
    print(r.move_tcp(T3, n_wp=3)); print('tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(2))
    r.gripper(0.04)
    T4 = make_T([-0.35, 0.30, 1.20], topdown(math.pi/2))
    print(r.move_tcp(T4, n_wp=2)); print('tcp', r.tcp()[:3,3].round(3))
