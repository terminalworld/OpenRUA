import sys
from rob import *
r = Robot('door')
HINGE = np.array([-0.29, -0.36]); TIP_Z = 1.0


def door_pose(a_deg, R, pitch_deg=30.0):
    """Hand pose whose closed fingertips touch the OUTER face of the door
    (door angle a, hinge distance R), hand pointing at the door and down."""
    a = np.radians(a_deg); c, s = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    contact = np.array([HINGE[0] + R * np.cos(a), HINGE[1] + R * np.sin(a), TIP_Z])
    n_out = np.array([np.sin(a), -np.cos(a), 0.0])
    d = -n_out * c + np.array([0, 0, -s])
    hy = np.array([np.cos(a), np.sin(a), 0.0])
    hx = np.cross(hy, d)
    if hx[2] < 0:           # keep the camera side up
        hy = -hy; hx = -hx
    q = quat_from_axes(hx, hy, d)
    return list(contact - TCP * d), q, contact


q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))
ARC = [(236, 0.16), (250, 0.16), (265, 0.16), (280, 0.16), (295, 0.16), (310, 0.15), (325, 0.14),
       (338, 0.12), (348, 0.10), (355, 0.10)]
plan = []
h, q, c = door_pose(236, 0.16); plan.append(('over_a', [h[0], h[1], 1.25], q))
h, q, c = door_pose(230, 0.16); plan.append(('pre', h, q))          # 6 deg behind the door
for a, R in ARC:
    h, q, c = door_pose(a, R); plan.append((f'a{a}', h, q))
h, q, c = door_pose(360, 0.10, 45); plan.append(('a360', h, q))
h, q, c = door_pose(355, 0.10); plan.append(('back', [h[0], h[1] - 0.05, h[2]], q))
h, q, c = door_pose(355, 0.10); plan.append(('up', [h[0], h[1] - 0.05, 1.25], q))

sols = []; seed = q0; prev = q0
for name, p, qq in plan:
    s = ik_near(r, p, qq, seed, tries=8)
    if s is None:
        print(f'IK fail {name} {np.round(p,3)}'); sols.append((name, None)); continue
    bad = path_check(r, prev, s, verbose=False)
    print(f'{name:7s} hand {np.round(p,3)} -> {np.round(s,3)} dist {np.abs(s-prev).max():.2f} bad {bad}', flush=True)
    sols.append((name, s)); seed = s; prev = s
if '--go' not in sys.argv: sys.exit(0)
r.gripper(0.0)
for name, s in sols:
    if s is None: print('skip', name); continue
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=3, tol=0.012)
    F = r.wrench()[:3]
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'F', np.round(F, 1), '|F| %.1f' % np.linalg.norm(F), flush=True)
    if name.startswith('a') and np.linalg.norm(F) > 40:
        print('   large force -> abort sweep', flush=True); break
