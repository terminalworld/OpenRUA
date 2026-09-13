import sys
from rob import *
from moveit_msgs.srv import GetPositionFK

r = Robot('p3')
q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))

L = 0.172                      # mug centre is this far in front of the hand (along d)
CX = -0.15                     # mug track x inside the cavity
Z_IN = 1.038                   # hand z while inserting (rim ~1.054, bottom ~0.955)
Z_LOW = 1.003                  # set-down (bottom ~0.946 vs floor 0.944)
HINGE = np.array([-0.29, -0.36]); ANG = np.radians(236)
U = np.array([np.cos(ANG), np.sin(ANG)]); N = np.array([-np.sin(ANG), np.cos(ANG)])   # N -> (+x,-y) side
if N[0] < 0: N = -N


def hand_for(cy, psi, z):
    q, d = Q_yaw(psi)
    c = np.array([CX, cy, 0.0])
    return list(c - L * d + np.array([0, 0, z])), q


def fk_links(q, links=('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand')):
    req = GetPositionFK.Request(); req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = list(map(float, q))
    f = r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, f, timeout_sec=60)
    out = {}
    for l, p in zip(links, f.result().pose_stamped):
        pp = p.pose
        out[l] = (np.array([pp.position.x, pp.position.y, pp.position.z]),
                  np.array([pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w]))
    return out


def door_clearance(q):
    """min signed distance (m) of hand ends / wrist links to the open door plane,
    only for points within the door's radial extent (<0.30 from hinge) and below z 1.12."""
    lk = fk_links(q)
    hp, hq = lk['panda_hand']; R = quat_to_R(hq)
    pts = [('hand+', hp + 0.10 * R[:, 1], 0.0), ('hand-', hp - 0.10 * R[:, 1], 0.0),
           ('tip+', hp + 0.10 * R[:, 1] + TCP * R[:, 2], 0.0), ('tip-', hp - 0.10 * R[:, 1] + TCP * R[:, 2], 0.0),
           ('l7', lk['panda_link7'][0], 0.05), ('l6', lk['panda_link6'][0], 0.06), ('l5', lk['panda_link5'][0], 0.06)]
    worst = (9, '')
    for name, p, rad in pts:
        rel = p[:2] - HINGE
        along = rel @ U; dist = rel @ N - rad
        if p[2] < 1.12 and -0.05 < along < 0.30 and dist < worst[0]:
            worst = (dist, name)
    return worst


# mug-centre waypoints (cy, psi)
WPS = [(-0.40, 30), (-0.36, 25), (-0.33, 20), (-0.29, 10), (-0.27, 5), (-0.25, 0)]
plan = [('lift_hi', [p0[0], p0[1], 1.25], qcur)]
h, q = hand_for(*WPS[0], 1.25); plan.append(('over_pre', h, q))
h, q = hand_for(*WPS[0], Z_IN); plan.append(('pre_ins', h, q))
for i, (cy, psi) in enumerate(WPS[1:], 1):
    h, q = hand_for(cy, psi, Z_IN); plan.append((f'ins{i}', h, q))
for j, z in enumerate((1.028, 1.020, 1.012, 1.004)):
    h, q = hand_for(*WPS[-1], z); plan.append((f'lower{j}', h, q))
plan.append(('OPEN',))
for i in (4, 3, 2, 1, 0):
    h, q = hand_for(*WPS[i], Z_IN); plan.append((f'ret{i}', h, q))
h, q = hand_for(*WPS[0], 1.25); plan.append(('ret_hi', h, q))

sols = []; seed = q0; prev = q0
for step in plan:
    if len(step) == 1: sols.append(step); continue
    name, p, qq = step
    s = ik_near(r, p, qq, seed, tries=8)
    if s is None: sys.exit(f'IK fail {name} {np.round(p,3)}')
    bad = path_check(r, prev, s, verbose=False)
    dc = door_clearance(s)
    print(f'{name:9s} {np.round(p,3)} -> {np.round(s,3)}  dist {np.abs(s-prev).max():.2f} bad {bad} door {dc[0]:.3f} {dc[1]}', flush=True)
    sols.append((name, s)); seed = s; prev = s
np.save('phase3_sols.npy', np.array([s[1] for s in sols if len(s) == 2]))
if '--go' not in sys.argv: sys.exit(0)
if '--from' in sys.argv:
    k = [i for i, s in enumerate(sols) if s[0] == sys.argv[sys.argv.index('--from') + 1]][0]; sols = sols[k:]
skip_lower = False
for step in sols:
    if step[0].startswith('lower') and skip_lower: continue
    if step[0] == 'OPEN':
        r.gripper(0.04); continue
    name, s = step
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=5, tol=0.006)
    f = r.fingers()
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'fingers %.4f' % f[0], 'wrench', np.round(r.wrench()[:3], 1), flush=True)
    if name.startswith('lower'):
        F = np.linalg.norm(r.wrench()[:3])
        print('   |F| = %.1f' % F, flush=True)
        if F > 9.0 or not ok:
            print('   contact with floor -> stop lowering', flush=True); skip_lower = True
        continue
    if not ok: sys.exit('move failed')
    if name.startswith('ins') or name in ('pre_ins', 'lower', 'over_pre'):
        if f[0] < 0.004: sys.exit('grasp lost')
