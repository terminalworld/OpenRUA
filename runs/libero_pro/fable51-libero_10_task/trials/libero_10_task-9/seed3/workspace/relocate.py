from rb import *
r = Robot('reloc')
zh = np.array([0, 0, -1.0]); yh = np.array([1.0, 0, 0]); R = R_from(zh, yh)   # vertical, closing along x
def flange(t): return t - 0.1034 * zh
xm, ym, rim = -0.061, 0.088, 1.012
dst = np.array([-0.105, 0.150])
print('open', r.grip(0.08))
w0 = None
def go(t, sec, tag='', fmax=8):
    global w0
    q = r.ik(flange(t), R)
    if q is None: raise SystemExit('ik fail ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 3)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'dW', (w - w0).round(1), 'fing', np.round(r.fingers(), 4))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
    return q
g = np.array([xm + 0.042, ym, rim - 0.014])
go([g[0], g[1], 1.09], 5, 'above')
go(g, 3, 'grasp pose')
print('close', r.grip(0.0))
t0 = time.time()
q1 = r.ik(flange([g[0], g[1], rim + 0.02]), R)
q2 = r.ik(flange([dst[0] + 0.042, dst[1], rim + 0.02]), R, seed=q1)
q3 = r.ik(flange([dst[0] + 0.042, dst[1], rim - 0.014]), R, seed=q2)
for a, b in ((r.arm_q(), q1), (q1, q2), (q2, q3)):
    print('jump', np.abs(np.array(a) - np.array(b)).max().round(3))
code, err = r.move(q3, 3.5, extra_points=[(q1, 1.0), (q2, 2.5)])
print('traj', code, round(err, 3), 'wall', round(time.time() - t0, 1), 'fing', np.round(r.fingers(), 4))
print('open', r.grip(0.08))
go([dst[0] + 0.042, dst[1], 1.09], 3, 'up')
r.snap('agentview', 'agent_reloc.png')
