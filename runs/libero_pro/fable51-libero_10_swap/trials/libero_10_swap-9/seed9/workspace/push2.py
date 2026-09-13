import sys
from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot('push')
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




TIP_Z = 0.98


def hand_for_tips(tx, ty, psi, z=TIP_Z):
    q, d = Q_yaw(psi)
    return list(np.array([tx, ty, z]) - TCP * d), q


q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))
# (tip x, tip y, yaw)
TIPS = [('behind', -0.17, -0.41, 30),
        ('push1', -0.17, -0.39, 25), ('push2', -0.17, -0.375, 20), ('push3', -0.168, -0.365, 15), ('push4', -0.165, -0.355, 10),
        ('back1', -0.17, -0.39, 15), ('back2', -0.18, -0.43, 30)]
plan = [('hi', [p0[0], p0[1], 1.25], qcur)]
h, q = hand_for_tips(-0.17, -0.41, 30, 1.20); plan.append(('over', h, q))
for name, tx, ty, psi in TIPS:
    h, q = hand_for_tips(tx, ty, psi); plan.append((name, h, q))
h, q = hand_for_tips(-0.18, -0.43, 30, 1.20); plan.append(('up', h, q))

sols = []; seed = q0; prev = q0
for name, p, qq in plan:
    s = ik_near(r, p, qq, seed, tries=8)
    if s is None: sys.exit(f'IK fail {name} {np.round(p,3)}')
    bad = path_check(r, prev, s, verbose=False)
    dc = door_clearance(s)
    print(f'{name:8s} hand {np.round(p,3)} -> {np.round(s,3)} dist {np.abs(s-prev).max():.2f} bad {bad} door {dc[0]:.3f} {dc[1]}', flush=True)
    sols.append((name, s)); seed = s; prev = s
if '--go' not in sys.argv: sys.exit(0)
r.gripper(0.0)
abort = False
for name, s in sols:
    if abort and not name.startswith(('back', 'up')): continue
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=4, tol=0.006)
    F = r.wrench()[:3]
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'F', np.round(F, 1), '|F| %.1f' % np.linalg.norm(F), flush=True)
    if name.startswith('push') and (np.linalg.norm(F) > 25 or not ok):
        print('   large force / blocked -> stop pushing', flush=True); abort = True
