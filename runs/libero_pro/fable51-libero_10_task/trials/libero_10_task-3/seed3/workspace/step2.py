import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3))
Tt = make_T([0.050, -0.029, 0.99], topdown(math.pi/2))
qt = ik(Tt, q0); print('target q', qt.round(3), 'clear', check(qt, ignore=('bowl',), verbose=True))
print('path clear', check_path([q0, qt], ignore=('bowl',)))
if '--go' in sys.argv:
    print(r.move_tcp(Tt, n_wp=4))
    print('tcp now', r.tcp()[:3,3].round(4), 'wrench', r.wrench().round(2))
    r.gripper(0.0)
    print('fingers', r.fingers())
