import sys
from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot('door2')
Q_DOWN = {'a': quat_from_axes([0,-1,0],[-1,0,0],[0,0,-1]), 'b': quat_from_axes([0,1,0],[1,0,0],[0,0,-1])}
YAW = sys.argv[sys.argv.index('--yaw')+1] if '--yaw' in sys.argv else 'b'
HINGE = np.array([-0.29, -0.36]); TIP_Z = 1.08      # tips 2.8cm below door top (1.108); hand body bottom ~1.125
OUT = 0.022                                        # tip centre offset to the outer side of the door model line (1cm penetration)


def fk_links(q, links=('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand')):
    req = GetPositionFK.Request(); req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = list(map(float, q))
    f = r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, f, timeout_sec=60)
    return {l: np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for l, p in zip(links, f.result().pose_stamped)}


def door_pose(a_deg, R, z=TIP_Z):
    """Vertical hand, closed fingertips just outside the door's OUTER face at hinge distance R."""
    a = np.radians(a_deg)
    u = np.array([np.cos(a), np.sin(a), 0.0]); n_out = np.array([np.sin(a), -np.cos(a), 0.0])
    tip = np.array([HINGE[0], HINGE[1], 0.0]) + R * u + OUT * n_out + np.array([0, 0, z])
    d = np.array([0, 0, -1.0])
    return list(tip - TCP * d), (Q_DOWN[YAW],)


q0 = r.arm_q(); p0, qcur = r.fk(); print('start q', np.round(q0, 3), 'hand', np.round(p0, 3))
ARC = [(228, 0.16), (240, 0.16), (252, 0.16), (264, 0.16), (276, 0.16), (288, 0.16), (300, 0.16), (312, 0.15),
       (324, 0.14), (336, 0.13), (346, 0.12), (354, 0.11), (362, 0.11)]
plan = [('hi', [p0[0], p0[1], 1.25], (qcur,))]
h, qq = door_pose(228, 0.16, 1.25); plan.append(('over', h, qq))
for a, R in ARC:
    h, qq = door_pose(a, R); plan.append((f'a{a}', h, qq))
h, qq = door_pose(354, 0.11); plan.append(('back', [h[0], h[1] - 0.04, h[2]], qq))
h, qq = door_pose(354, 0.11, 1.25); plan.append(('up', [h[0], h[1] - 0.04, h[2]], qq))

sols = []; seed = q0; prev = q0; chosen = None
for name, p, qs in plan:
    if len(qs) == 2 and chosen is not None: qs = (qs[chosen],)
    cands = []
    for i, qq in enumerate(qs):
        s = ik_near(r, p, qq, seed, tries=8)
        if s is not None: cands.append((np.abs(s - seed).max(), i, s))
    if not cands: print(f'IK fail {name} {np.round(p,3)}'); sols.append((name, None)); continue
    cands.sort(key=lambda t: t[0]); _, i, s = cands[0]
    if len(qs) == 2 and chosen is None: chosen = i; print('  yaw branch', i)
    bad = path_check(r, prev, s, verbose=False)
    lk = fk_links(s)
    print(f'{name:5s} hand {np.round(p,3)} -> {np.round(s,3)} dist {np.abs(s-prev).max():.2f} bad {bad} l7 {np.round(lk["panda_link7"],3)} l6 {np.round(lk["panda_link6"],3)}', flush=True)
    # link sweep check along the joint-space interpolation
    worst = (9, ''); inbox = []
    for t in np.linspace(0, 1, 11):
        lk = fk_links(prev + t * (s - prev), ('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand'))
        for l, p in lk.items():
            rad = 0.06 if l != 'panda_hand' else 0.10
            if p[2] - rad < worst[0]: worst = (p[2] - rad, l)
            if -0.29 - rad < p[0] < 0.06 + rad and -0.36 - rad < p[1] < -0.14 + rad and p[2] - rad < 1.108: inbox.append(l)
    print(f'      sweep: lowest link-bottom z {worst[0]:.3f} ({worst[1]})  over-microwave hits: {sorted(set(inbox))}', flush=True)
    sols.append((name, s)); seed = s; prev = s
np.save('door2_sols.npy', np.array([x[1] for x in sols if x[1] is not None]))
if '--go' not in sys.argv: sys.exit(0)
r.gripper(0.0)
for name, s in sols:
    if s is None: print('skip', name); continue
    print('>>', name, flush=True)
    ok = move_q_retry(r, s, 3.0, tries=3, tol=0.012)
    F = r.wrench()[:3]
    print('   reached' if ok else '   NOT reached', 'hand', np.round(r.fk()[0], 4), 'F', np.round(F, 1), '|F| %.1f' % np.linalg.norm(F), flush=True)
    if name.startswith('a') and np.linalg.norm(F) > 30:
        print('   large force -> stop sweep here', flush=True)
        rest = [x for x in sols if x[0] in ('back', 'up')]
        for n2, s2 in rest:
            print('>>', n2, flush=True); move_q_retry(r, s2, 3.0, tries=3, tol=0.012)
            print('   hand', np.round(r.fk()[0], 4), flush=True)
        break
