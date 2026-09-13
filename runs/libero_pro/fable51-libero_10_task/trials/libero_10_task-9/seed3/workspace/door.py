from rb import *
import sys
r = Robot('door')
zh = np.array([0, 0, -1.0]); yh = np.array([0, -1.0, 0]); R = R_from(zh, yh)
H = np.array([-0.19, 0.27]); RHO = 0.16; OFF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.033
Z = 1.075
def tcp_at(th, off):
    d = np.array([np.sin(th), -np.cos(th)]); n = np.array([-np.cos(th), -np.sin(th)])
    p = H + RHO * d + off * n
    return np.array([p[0], p[1], Z])
w0 = None
def go(t, sec, tag, fmax=20):
    global w0
    q = r.ik(np.asarray(t) - 0.1034 * zh, R, seed=r.arm_q())
    if q is None: raise SystemExit('ik fail ' + tag)
    jump = np.abs(q - r.arm_q()).max()
    if jump > 1.2: raise SystemExit(f'jump {jump:.2f} ' + tag)
    code, err = r.move(q, sec)
    if err > 0.01: code, err = r.move(q, 2)
    p = r.fk()[0]; w = r.wrench()
    if w0 is None: w0 = w
    print(tag, 'tcp', (p + 0.1034 * zh).round(3), 'code', code, 'err', round(err, 3), 'jump', round(jump, 2), 'dW', (w - w0).round(1), flush=True)
    if np.abs(w - w0)[:3].max() > fmax: raise SystemExit('force! ' + tag)
print('close', r.grip(0.0))
TH0 = np.radians(-35)
p0 = tcp_at(TH0, 0.045)
go([p0[0], p0[1], 1.20], 4, 'above')
go(p0, 3, 'down')
for th in np.radians(np.arange(-30, 91, 5)):
    go(tcp_at(th, OFF), 0.7, f'th {np.degrees(th):.0f}')
r.snap('agentview', 'agent_door.png'); r.snap('birdview', 'bird_door.png')
go(tcp_at(np.radians(90), 0.08), 1.5, 'release')
go([-0.05, 0.15, 1.25], 3, 'up')
r.snap('agentview', 'agent_door2.png'); r.snap('frontview', 'front_door2.png')
