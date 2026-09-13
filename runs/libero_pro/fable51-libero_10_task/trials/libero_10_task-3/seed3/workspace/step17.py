import numpy as np, math, sys, subprocess, shutil
from panda import *
from collide import check
np.set_printoptions(precision=4, suppress=True)
r = Robot()
R = topdown(math.pi/2)
def go(p, n=1):
    q = ik(make_T(p, R), r.q()); c = check(q, ignore=('bottle','drawer'))
    print('->', p, 'clear', c); assert c[0] > -0.005
    print(r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(1), flush=True)
print('start tcp', r.tcp()[:3,3].round(3), 'fingers', r.fingers())
r.gripper(0.08)
go([0.028, 0.098, 1.15]); go([0.028, 0.098, 0.985], 3)
r.gripper(0.0)
go([0.045, 0.11, 0.985], 2); go([0.06, 0.125, 0.985], 2); go([0.075, 0.14, 0.985], 2)
r.gripper(0.08)
go([0.075, 0.14, 1.2], 2)
