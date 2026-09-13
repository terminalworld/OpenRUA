import numpy as np, math, sys
import collide
from panda import *
from collide import check, check_path, OBST
from pushmodel import *
np.set_printoptions(precision=4, suppress=True)
r = Robot()
deg, zc, x = 80, 0.945, -0.08
th = math.radians(deg)
a = np.array([0, math.sin(th), -math.cos(th)]); xh = np.array([0, math.cos(th), math.sin(th)])
R = R_from_axes(a, y=[1, 0, 0]); off = 0.0116*a - 0.012*xh
seeds = np.load(f'plan19_{deg}_{zc}.npy')
def tcp_for(yc, z): return np.array([x, yc, z]) - off
def go(p, n=1, yp=0.175, seed=None):
    q = ik(make_T(p, R), r.q() if seed is None else seed); assert q is not None
    c = check(q, ignore=ign, extra=extras(yp)); print('->', p.round(3), 'clear', c, flush=True); assert c[0] > 0.002
    if seed is not None:
        pc = check_path([r.q(), q], ignore=ign, extra=extras(yp)); print('   path', pc); assert pc[0] > -0.01
        res = r.move_q(q)
    else:
        res = r.move_tcp(make_T(p, R), n_wp=n)
    print('  ', res, 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1), flush=True)
    return res
print('start', r.tcp()[:3,3].round(3), r.fingers())
r.gripper(0.0)
via = np.load('via19.npy')
print('via path', check_path([r.q(), via[0]], ignore=ign, extra=extras(0.175)), check_path([via[0], via[1]], ignore=ign, extra=extras(0.175)))
print(r.move_q(via[0]), 'tcp', r.tcp()[:3,3].round(3))
print(r.move_q(via[1]), 'tcp', r.tcp()[:3,3].round(3), 'wrench', r.wrench().round(1))
go(tcp_for(0.15, zc), n=3, yp=0.175)
yc = 0.15; w0 = r.wrench()
while yc < 0.236:
    yc = min(yc + 0.015, 0.236)
    res = go(tcp_for(yc, zc), n=2, yp=max(0.175, yc))
    w = r.wrench()
    if res is None or res[0] != 0 or abs(w[1]-w0[1]) > 8:
        print('STOP at contact-edge y', yc, 'code', res, 'wrench', w.round(1)); break
print('final tcp', r.tcp()[:3,3].round(3))
go(tcp_for(yc - 0.03, zc), n=2, yp=0.228); go(tcp_for(yc - 0.03, 1.10), n=2, yp=0.228)
