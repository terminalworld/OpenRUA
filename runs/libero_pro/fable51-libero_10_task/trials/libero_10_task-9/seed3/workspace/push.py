from rb import *
r = Robot('push')
# fingers pitched 30 deg: pointing down and -x; body leans up/+x. closing axis in x-z plane
b = np.radians(30)
zh = np.array([-np.sin(b), 0, -np.cos(b)]); yh = np.array([np.cos(b), 0, -np.sin(b)]); R = R_from(zh, yh)
def flange(t): return t - 0.1034 * zh
xm, ym = -0.061, 0.088
print('fingers', np.round(r.fingers(), 4))
w0 = None
def go(t, sec, tag='', fmax=8):
    global w0
    q = r.ik(flange(t), R)
    if q is None: raise SystemExit('ik fail ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 3)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'dW', (w - w0).round(1))
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force!')
x0 = xm + 0.046 + 0.02
go([x0, ym, 1.05], 5, 'above')
go([x0, ym, 0.93], 3, 'down')
go([x0 - 0.04, ym, 0.93], 4, 'push1')
go([-0.040, ym, 0.93], 4, "push2")
go([-0.040, ym, 1.05], 3, "up")
r.snap('agentview', 'agent_push.png')
