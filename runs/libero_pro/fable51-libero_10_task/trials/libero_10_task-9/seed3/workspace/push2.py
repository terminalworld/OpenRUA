from rb import *
r = Robot('push2')
zh = np.array([0, 1.0, 0]); yh = np.array([-1.0, 0, 0]); R = R_from(zh, yh)
print('close', r.grip(0.0))
w0 = None
def go(t, sec, tag, fmax=15):
    global w0
    q = r.ik(np.asarray(t) - 0.1034 * zh, R, seed=r.arm_q())
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    if jump > 1.7: raise SystemExit(f'jump {jump:.2f} ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), flush=True)
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force! ' + tag)
go([-0.06, 0.20, 0.985], 1.0, 'approach')
r.snap('agentview', 'agent_push_a.png')
go([-0.062, 0.31, 0.985], 2.5, 'push', fmax=25)
go([-0.06, 0.15, 0.985], 1.5, 'back', fmax=25)
go([-0.06, 0.15, 1.10], 2, 'up', fmax=25)
r.snap('agentview', 'agent_push_b.png')
