from rob import *
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
PX, PY, PZ = 0.070, 0.607, 0.565
t = math.radians(30)
cands = []
for ang2 in (math.pi / 2, -math.pi / 2):
    c2 = np.array([math.cos(ang2), math.sin(ang2), 0.0]); f2 = np.array([-math.sin(ang2), math.cos(ang2), 0.0])
    a2 = math.sin(t) * c2 + np.array([0, 0, -math.cos(t)]); q2 = quat_from_axes(a2, -f2)
    s_lo = r.solve_ik(PX, PY, PZ, q2, seed=r.arm_q())
    s_hi = r.solve_ik(PX, PY, 0.76, q2, seed=s_lo if s_lo else r.arm_q())
    log(f"ang2 {math.degrees(ang2):.0f}: low {None if s_lo is None else np.round(s_lo,3)} high {None if s_hi is None else np.round(s_hi,3)}")
    if s_lo is not None and s_hi is not None: cands.append((ang2, q2, a2, s_hi, s_lo))
if not cands: log("no place IK; holding"); raise SystemExit
ang2, q2, a2, s_hi, s_lo = cands[0]; log("using ang2", math.degrees(ang2))
fg = r.fingers(); log("gap before transport", round(fg[0] - fg[1], 4))
ok = r.move_joints(s_hi, secs=6); log("above basket", ok)
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4), "wrench", wrench())
fg = r.fingers(); log("gap", round(fg[0] - fg[1], 4))
if round(fg[0] - fg[1], 4) < 0.04: log("lost the can!"); raise SystemExit
ok = move_tcp_iter(r, PX, PY, PZ, q2, secs=4, tries=3, tol=0.006); log("lowered", ok, "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
if tcp[2] > PZ + 0.03: log("descent blocked; stopping for inspection"); raise SystemExit
r.gripper(0.04); r.spin(1.0); log("released, fingers", r.fingers(), "wrench", wrench())
ok = move_tcp_iter(r, PX, PY, 0.80, q2, secs=4, tries=2, tol=0.01); log("lifted", ok)
ok = r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=6); log("parked", ok)
if not ok: r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=6)
log("PHASE11 DONE")
