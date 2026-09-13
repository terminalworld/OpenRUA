from rb import *
from clear import hand_clearance
import sys
r = Robot('ins5')
BETA = 25.0; b = np.radians(BETA)
zh = np.array([0, np.sin(b), -np.cos(b)]); yh = np.array([0, np.cos(b), np.sin(b)]); RF = R_from(zh, yh)       # final frame
Rz90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
flip = np.diag([-1, -1, 1.0])
RF = RF @ flip
R0 = Rz90 @ RF                                                                                                 # grasp frame (fingers down & -x)
w0 = None
def go(t, sec, R, tag='', fmax=8, seed=None, check=True):
    global w0
    zh_ = R[:, 2]
    q = r.ik(np.asarray(t) - 0.1034 * zh_, R, seed=seed)
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    if jump > 1.6: raise SystemExit(f'jump too big {jump:.2f} ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p, Rc = r.fk(); w = r.wrench()
    if w0 is None: w0 = w
    r.spin(0.2); f = r.fingers()
    print(tag, 'tcp', (p + 0.1034 * zh_).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), 'fing', np.round(f, 4), 'clr', round(hand_clearance(p + 0.1034 * zh_, R, 0.006), 3), flush=True)
    if check and np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force! ' + tag)
    return q, f, w - w0

stage = sys.argv[1] if len(sys.argv) > 1 else 'all'
if stage in ('setdown', 'all'):
    # currently hanging mug at beta=20 frame; set down at (-0.09,0.172)
    b2 = np.radians(20); z2 = np.array([0, np.sin(b2), -np.cos(b2)]); y2 = np.array([0, np.cos(b2), np.sin(b2)]); R2 = R_from(z2, y2)
    go([-0.09, 0.130, 1.06], 1.5, R2, 'move back', fmax=12)
    go([-0.09, 0.130, 1.012], 1.5, R2, 'lower', fmax=12, check=False)
    print('open', r.grip(0.08))
    go([-0.09, 0.130, 1.10], 2, R2, 'up')
    w0 = None
    r.snap('agentview', 'agent_sd5.png')
if stage in ('grasp', 'all'):
    cx, cy, rim = -0.0945, 0.1875, 1.012
    D = 0.014
    g = np.array([cx + 0.046 - 0.004, cy, rim - D])
    print('open', r.grip(0.08))
    go([g[0], g[1], 1.10], 4, R0, 'above')
    go(g, 3, R0, 'grasp pose')
    print('close', r.grip(0.0)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
    _, f, _ = go([g[0], g[1], 1.06], 1.2, R0, 'lift', fmax=12)
    if f[0] < 0.0015:
        print('slipped'); 
        if f[0] > 0.0003: go([g[0], g[1], 1.010], 1.2, R0, 'setdown')
        r.grip(0.08); go([g[0], g[1], 1.10], 3, R0, 'up'); raise SystemExit
    # yaw -90 while moving: mug centre -> (-0.075, 0.17); TCP = mug - 0.042 y
    q, f, _ = go([-0.075, 0.170 - 0.042, 1.06], 2.5, RF, 'yaw+move', fmax=12)
    r.snap('frontview', 'front_yaw.png'); r.snap('agentview', 'agent_yaw.png')
    if f[0] < 0.0012: print('slipped after yaw'); go([-0.075, 0.128, 1.010], 1.5, RF, 'setdown'); r.grip(0.08); go([-0.075, 0.128, 1.10], 3, RF, 'up'); raise SystemExit
    go([-0.075, 0.128, 1.012], 1.5, RF, 'lower to table', fmax=15, check=False)
    _, f, dw = go([-0.075, 0.175, 1.012], 1.5, RF, 'slide y1', fmax=15, check=False)
    _, f, dw = go([-0.075, 0.186, 1.012], 0.8, RF, 'slide y2', fmax=15, check=False)
    r.snap('frontview', 'front_slide.png'); r.snap('agentview', 'agent_slide.png')
    _, f, dw = go([-0.075, 0.186, 1.058], 1.5, RF, 'lift along face', fmax=15, check=False)
    print('fingers', np.round(f, 4))
    if f[0] < 0.0010: print('slipped on lift'); go([-0.075, 0.186, 1.012], 1.2, RF, 'setdown'); r.grip(0.08); go([-0.075, 0.186, 1.10], 3, RF, 'up'); raise SystemExit
    _, f, dw = go([-0.075, 0.216, 1.058], 1.0, RF, 'over ledge', fmax=15, check=False)
    r.snap('frontview', 'front_ledge.png'); r.snap('agentview', 'agent_ledge.png')
    _, f, dw = go([-0.075, 0.250, 1.056], 1.2, RF, 'in', fmax=20, check=False)
    _, f, dw = go([-0.075, 0.250, 1.048], 0.6, RF, 'down', fmax=20, check=False)
    print('open', r.grip(0.08))
    go([-0.075, 0.190, 1.06], 1.5, RF, 'back', check=False)
    go([-0.075, 0.12, 1.15], 3, RF, 'up', check=False)
    r.snap('agentview', 'agent_ins5.png'); r.snap('frontview', 'front_ins5.png')
if stage == 'final':
    cx, cy, rim = -0.075, 0.213, 1.012
    D = 0.014
    g = np.array([cx, cy - 0.046 + 0.004, rim - D])
    print('open', r.grip(0.08))
    go([g[0], g[1], 1.10], 3, RF, 'above')
    go(g, 3, RF, 'grasp pose')
    print('close', r.grip(0.0)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
    _, f, dw = go([g[0], 0.184, g[2]], 1.0, RF, 'slide to face', fmax=15, check=False)
    _, f, dw = go([g[0], 0.186, 1.072], 1.5, RF, 'lift along face', fmax=15, check=False)
    if f[0] < 0.0010: print('slipped on lift'); go([g[0], 0.186, 1.010], 1.2, RF, 'setdown'); r.grip(0.08); go([g[0], 0.186, 1.10], 3, RF, 'up'); raise SystemExit
    _, f, dw = go([g[0], 0.216, 1.072], 0.8, RF, 'over ledge', fmax=15, check=False)
    r.snap('frontview', 'front_ledge.png')
    _, f, dw = go([g[0], 0.250, 1.062], 1.0, RF, 'in', fmax=20, check=False)
    r.snap('frontview', 'front_in.png')
    _, f, dw = go([g[0], 0.250, 1.056], 0.5, RF, 'down', fmax=20, check=False)
    print('open partial', r.grip(0.02)); r.spin(0.3); print('fingers', np.round(r.fingers(), 4))
    go([g[0], 0.190, 1.062], 1.5, RF, 'back', check=False)
    print('open', r.grip(0.08))
    go([g[0], 0.12, 1.15], 3, RF, 'up', check=False)
    r.snap('agentview', 'agent_ins5.png'); r.snap('frontview', 'front_ins5.png')
