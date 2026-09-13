import sys
from rob import *
r = Robot('p2')
q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0,3), 'hand', np.round(p0,3))
BAR = np.array([-0.208, 0.242]); PZ = 0.958
qy0, _ = Q_yaw(0)
plan = [('pregrasp', [BAR[0], BAR[1]+0.01-TCP-0.08, PZ], qy0),
        ('grasp',    [BAR[0], BAR[1]+0.01-TCP, PZ], qy0),
        ('CLOSE',),
        ('lift',     [BAR[0], BAR[1]+0.01-TCP, 1.03], qy0)]
SEED = np.array([-0.627,1.387,0.165,-0.94,1.037,1.247,-1.578])  # phase-1 grasp branch
plan.insert(0, ('high', [BAR[0], BAR[1]+0.01-TCP-0.08, 1.15], qy0))
sols=[]; seed=SEED; prev=q0
for step in plan:
    if len(step)==1: sols.append(step); continue
    name,p,q = step
    s = ik_near(r, p, q, seed, tries=8)
    if s is None: sys.exit(f'IK fail {name}')
    print(f'{name:9s} {np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-prev).max():.2f}', flush=True)
    print('  bad steps:', path_check(r, prev, s, verbose=False), flush=True)
    sols.append((name,s)); seed=s; prev=s
if '--go' not in sys.argv: sys.exit(0)
for step in sols:
    if step[0]=='CLOSE':
        r.gripper(0.0); print('   fingers after close', r.fingers(), flush=True); continue
    name,s = step
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0)
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0],4), flush=True)
    if not ok: sys.exit('move failed')
print('fingers after lift', r.fingers())
