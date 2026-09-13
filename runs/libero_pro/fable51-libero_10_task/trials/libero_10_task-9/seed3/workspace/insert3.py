from rb import *
from clear import hand_clearance
r = Robot('ins3')
b = np.radians(45)
zh = np.array([0, np.sin(b), -np.cos(b)]); yh = np.array([0, np.cos(b), np.sin(b)]); R = R_from(zh, yh)
def flange(t): return t - 0.1034 * zh
xm, ym, rim = -0.104, 0.150, 1.012
yo = ym - 0.046
g = np.array([xm, yo + 0.0093, rim - 0.02])
XT, YT = -0.07, 0.30                      # target mug centre in cavity
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
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), 'fing', np.round(r.fingers(), 4), 'clr', round(hand_clearance(p + 0.1034 * zh, R, 0.013), 3))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
    return q
go([g[0], g[1], 1.07], 5, 'above')
go(g, 3, 'grasp pose')
print('close', r.grip(0.0)); r.spin(0.5); print('fingers', np.round(r.fingers(), 4))
wg = r.wrench(); print('dW after grasp', (wg - w0).round(1))
z1 = 1.055
go([g[0], g[1], z1], 1.2, 'lift', fmax=12)
go([XT, YT - 0.0367 - 0.08, z1], 1.5, 'approach', fmax=12)      # mug far edge just before ledge
go([XT, YT - 0.0367, z1], 1.2, 'in', fmax=12)
go([XT, YT - 0.0367, 1.034], 0.8, 'down', fmax=15)
go([XT, YT - 0.0367 - 0.01, 1.034], 0.5, 'drag', fmax=15)
print('open', r.grip(0.08))
go([XT, YT - 0.0367 - 0.06, 1.05], 2, 'back')
go([XT, 0.12, 1.15], 3, 'up')
r.snap('agentview', 'agent_ins3.png'); r.snap('frontview', 'front_ins3.png')
