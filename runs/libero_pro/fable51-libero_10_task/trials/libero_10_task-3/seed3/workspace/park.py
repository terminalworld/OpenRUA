import numpy as np, math, sys
from panda import *
from collide import check, check_path
r = Robot()
q0 = r.q()
p = [float(v) for v in sys.argv[1].split(',')]
T = make_T(p, topdown(math.pi/2))
qt = ik(T, q0); print('qt', qt.round(3), check(qt), check_path([q0,qt]))
print(r.move_q(qt)); print('tcp', r.tcp()[:3,3].round(4))
