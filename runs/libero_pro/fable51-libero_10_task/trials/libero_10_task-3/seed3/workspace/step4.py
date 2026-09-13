import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
R = topdown(math.pi/2)   # finger axis along world x
Ta = make_T([-0.132, 0.058, 1.20], R)
Tg = make_T([-0.132, 0.058, 1.025], R)
qa = ik(Ta, q0); qg = ik(Tg, qa)
print('qa', qa.round(3), check(qa, verbose=False))
print('qg', qg.round(3), check(qg, ignore=('bottle',), verbose=True))
print('path0', check_path([q0, qa]), 'path1', check_path([qa, qg], ignore=('bottle',)))
if '--go' in sys.argv:
    print(r.move_q(qa)); print('tcp', r.tcp()[:3,3].round(4))
    print(r.move_tcp(Tg, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
    r.gripper(0.0)
    print('fingers', r.fingers(), 'wrench', r.wrench().round(2))
