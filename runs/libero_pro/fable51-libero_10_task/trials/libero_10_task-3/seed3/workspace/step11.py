import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'fingers', r.fingers())
R = topdown(math.pi/2)   # finger axis along world x -> wide face pushes +y
P = [([0.04, 0.045, 1.15], 1), ([0.04, 0.045, 0.9766], 3), ([0.04, 0.06, 0.9766], 2), ([0.04, 0.06, 1.0015], 1),
     ([0.04, 0.10, 1.0015], 3), ([0.04, 0.10, 1.20], 2)]
ign = ('bottle','drawer')
qs = []; qp = q0
for p, n in P:
    q = ik(make_T(p, R), qp); assert q is not None, p
    print(p, q.round(3), check(q, ignore=ign)); qs.append(q); qp = q
print('path', check_path([q0]+qs, ignore=ign))
if '--go' in sys.argv:
    r.gripper(0.0)
    for (p, n), q in zip(P, qs):
        print(p, r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))
