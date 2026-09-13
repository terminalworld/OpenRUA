import numpy as np, math, sys
from panda import *
from collide import check, check_path
np.set_printoptions(precision=4, suppress=True)
r = Robot()
th = math.radians(52)
R = R_from_axes([0, math.cos(th), -math.sin(th)], y=[1, 0, 0])
z = 0.959; ign = ('drawer','bottle')
seeds = np.load('seed18_52.npy')
def go(p, n=1, seed=None):
    q = ik(make_T(p, R), r.q() if seed is None else seed); c = check(q, ignore=ign)
    print('->', p, 'clear', c, flush=True); assert c[0] > -0.003
    res = r.move_q(q) if n <= 1 else r.move_tcp(make_T(p, R), n_wp=n, q0=r.q())
    print('  ', res, 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1), flush=True)
    return res
print('start', r.tcp()[:3,3].round(3), r.fingers())
r.gripper(0.0)
# go to first seed configuration via joint move (check path from current)
print('path to seed0', check_path([r.q(), seeds[0]], ignore=ign))
go([0.0, 0.025, 1.15], seed=seeds[0]); go([0.0, 0.025, z], 3)
y = 0.025
while y < 0.185:
    y = min(y + 0.03, 0.185)
    res = go([0.0, y, z], 2)
    w = r.wrench()
    if res is None or res[0] != 0 or abs(w[1]) > 15:
        print('stopped at y', y, 'code', res, 'wrench', w.round(1)); break
print('final tcp', r.tcp()[:3,3].round(3))
yc = r.tcp()[1,3]
go([0.0, yc - 0.04, z], 2); go([0.0, yc - 0.04, 1.2], 2)
