from refine import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr = {}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m', m), 1)
def wrench():
    wr.pop('m', None)
    for _ in range(50):
        rclpy.spin_once(r.node, timeout_sec=0.1)
        if 'm' in wr: break
    f = wr['m'].wrench.force; return np.round([f.x, f.y, f.z], 2)
CX, CY, GZ, ang = 0.268, 0.353, 0.455, math.radians(31.0)
def frames(ang):
    c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
    t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    return c, f, a, quat_from_axes(a, -f)
c, f, a, q = frames(ang)
r.gripper(0.04)
pre = np.array([CX, CY, GZ]) - 0.14 * a
PRE = r.solve_ik(*pre, q, seed=[0.366, 1.26, 0.093, -1.01, -0.011, 2.786, 0.624]); log("PRE", np.round(PRE, 3))
ok = r.move_joints(PRE, secs=6); log("at PRE", ok, "wrench", wrench())
# wrist-camera refinement of the can pose from here
try:
    cen, pts = refine(r, CX, CY, 0.44, 0.50, rad=0.13, cam="robot0_eye_in_hand")
    pts = np.asarray(pts); ridge = pts[pts[:, 2] > 0.475]
    c0 = ridge[:, :2].mean(0); u, s, vt = np.linalg.svd(ridge[:, :2] - c0); ax = vt[0]
    ang2 = math.atan2(ax[1], ax[0])
    if math.cos(ang2 - ang) < 0: ang2 += math.pi
    log("refined center", c0.round(4), "axis deg", round(math.degrees(ang2), 1), "n", len(ridge), "ztop", round(pts[:, 2].max(), 4))
    if len(ridge) > 50 and np.linalg.norm(c0 - [CX, CY]) < 0.03 and abs(math.degrees(ang2) - 31) < 15:
        CX, CY = float(c0[0]), float(c0[1]); ang = ang2; c, f, a, q = frames(ang); log("using refined pose")
except Exception as e:
    log("refine failed", e)
mid = np.array([CX, CY, GZ]) - 0.05 * a
ok = move_tcp_iter(r, *mid, q, secs=3, tol=0.004); log("at mid", ok, "wrench", wrench())
if not ok: log("mid not reached; stopping"); raise SystemExit
ok = move_tcp_iter(r, CX, CY, GZ, q, secs=3, tries=3, tol=0.004); log("at grasp", ok, "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp before close", tcp.round(4))
if not ok:
    log("grasp pose not reached; stopping for inspection"); raise SystemExit
r.gripper(0.0); r.spin(0.5); fg = r.fingers(); gap = fg[0] - fg[1]
log(f"finger gap after close = {gap:.4f}")
if gap < 0.05:
    log("missed the can; reopening"); r.gripper(0.04); raise SystemExit
back = np.array([CX, CY, GZ]) - 0.12 * a
ok2 = move_tcp_iter(r, *back, q, secs=4, tries=2, tol=0.01); log("retreat", ok2)
up = np.array([CX - 0.05, CY, 0.70]) - 0.10 * a
ok3 = move_tcp_iter(r, *up, q, secs=4, tries=2, tol=0.01); log("lift", ok3)
r.spin(0.5); fg = r.fingers(); log("fingers after lift", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
log("PHASE10 DONE")
