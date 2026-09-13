from rb import *
from clear import hand_clearance
import sys
r = Robot('ins4')
BETA = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
b = np.radians(BETA)
zh = np.array([0, np.sin(b), -np.cos(b)]); yh = np.array([0, np.cos(b), np.sin(b)]); R = R_from(zh, yh)
def flange(t): return t - 0.1034 * zh
xm, ym, rim = -0.106, 0.191, 1.012
D = float(sys.argv[2]) if len(sys.argv) > 2 else 0.014
g = np.array([xm, ym - 0.046 + 0.004, rim - D])
XT, YT = -0.075, 0.287
ZH = 1.056
print('open', r.grip(0.08))
w0 = None
def go(t, sec, tag='', fmax=8, seed=None):
    global w0
    q = r.ik(flange(t), R, seed=seed)
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    f = r.fingers()
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), 'fing', np.round(f, 4), 'clr', round(hand_clearance(p + 0.1034 * zh, R, 0.006), 3))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
    return q, f
go([g[0], g[1], 1.09], 5, 'above')
go(g, 3, 'grasp pose')
print('close', r.grip(0.0)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
_, f = go([g[0], g[1], ZH], 1.2, 'lift', fmax=12)
if f[0] < 0.0015:
    if f[0] > 0.0003:
        print('SLIPPED to lip -> set down'); go([g[0], g[1], 1.010], 1.2, 'setdown')
    else:
        print('mug lost'); 
    r.grip(0.08); go([g[0], g[1], 1.09], 3, 'up'); raise SystemExit
r.snap('frontview', 'front_hang.png'); r.snap('agentview', 'agent_hang.png')
_, f = go([XT, YT - 0.042 - 0.07, ZH], 1.6, 'approach', fmax=12)
r.snap('frontview', 'front_appr.png')
if f[0] < 0.0012:
    print('SLIPPED during approach -> set down here'); go([XT, YT - 0.042 - 0.07, 1.012], 1.2, 'setdown2'); r.grip(0.08); go([XT, YT - 0.042 - 0.07, 1.09], 3, 'up'); raise SystemExit
_, f = go([XT, YT - 0.042, ZH], 1.0, 'in', fmax=12)
print('open', r.grip(0.08))
go([XT, YT - 0.042 - 0.06, ZH + 0.01], 2, 'back')
go([XT, 0.12, 1.15], 3, 'up')
r.snap('agentview', 'agent_ins4.png'); r.snap('frontview', 'front_ins4.png')
