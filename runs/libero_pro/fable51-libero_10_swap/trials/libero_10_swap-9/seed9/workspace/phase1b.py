import sys
from rob import *
r = Robot('p1b')
q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0,3), 'hand', np.round(p0,3))
MUG = np.array([-0.1409, 0.1993]); R_MUG = 0.047
q_down_a = quat_from_axes([0,-1,0],[-1,0,0],[0,0,-1])   # hand z down, fingers along x
q_down_b = quat_from_axes([0,1,0],[1,0,0],[0,0,-1])
gx, gy = MUG[0] + R_MUG, MUG[1]            # straddle wall at +x point
DROP = np.array([-0.22, 0.32])             # where to set the mug down (open table)
Z_GRASP = 1.072                            # fingertips ~3.7cm below rim (rim 1.006)
qy0,_ = Q_yaw(0)
plan = [('up', None, np.array([-0.546,0.468,0.239,-2.084,1.237,1.469,-1.754])),
        ('up2', [-0.138, 0.0, 1.32], qy0),
        ('over_mug', [gx, gy, 1.25], None),
        ('descend',  [gx, gy, Z_GRASP], None),
        ('CLOSE',),
        ('lift',     [gx, gy, 1.22], None),
        ('over_drop',[DROP[0]+R_MUG, DROP[1], 1.22], None),
        ('setdown',  [DROP[0]+R_MUG, DROP[1], Z_GRASP+0.004], None),
        ('OPEN',),
        ('lift2',    [DROP[0]+R_MUG, DROP[1], 1.20], None)]
sols = []; seed = q0; qd = None
for step in plan:
    if len(step) == 1: sols.append(step); continue
    name, p, q = step
    if p is None:
        s = q
    elif q is None:
        if qd is None:
            ca = ik_near(r, p, q_down_a, seed); cb = ik_near(r, p, q_down_b, seed)
            cands = [(np.abs(c-seed).max(), c, qq) for c, qq in ((ca,q_down_a),(cb,q_down_b)) if c is not None]
            if not cands: sys.exit(f'IK fail {name}')
            cands.sort(key=lambda t: t[0]); s, qd = cands[0][1], cands[0][2]
            print('  chose orientation', 'a' if qd is q_down_a else 'b')
        else:
            s = ik_near(r, p, qd, seed)
    else:
        s = ik_near(r, p, q, seed)
    if s is None: sys.exit(f'IK fail {name}')
    print(f'{name:9s} {p if p is None else np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-seed).max():.2f}', flush=True)
    print('  bad steps:', path_check(r, seed, s, verbose=False), flush=True)
    sols.append((name, s)); seed = s
if '--go' not in sys.argv: sys.exit(0)
for step in sols:
    if step[0] == 'CLOSE':
        r.gripper(0.0); f = r.fingers(); print('   fingers after close', f, flush=True); continue
    if step[0] == 'OPEN':
        r.gripper(0.04); continue
    name, s = step
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0)
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0],4), flush=True)
    if not ok: sys.exit('move failed')
    if name in ('lift',):
        f = r.fingers(); print('   fingers while lifted', f, flush=True)
        if f[0] < 0.002: sys.exit('grasp lost')
