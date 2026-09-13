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
CX, CY, GZ, ang = 0.230, 0.342, 0.45, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
q = quat_from_axes(a, -f)
# place-pose feasibility (can horizontal, axis along 60 deg, over basket)
ang2 = math.radians(60); c2 = np.array([math.cos(ang2), math.sin(ang2), 0]); f2 = np.array([-math.sin(ang2), math.cos(ang2), 0])
a2 = math.sin(t) * c2 + np.array([0, 0, -math.cos(t)]); q2 = quat_from_axes(a2, -f2)
for z in (0.72, 0.68, 0.66):
    s = r.solve_ik(0.027, 0.447, z, q2, seed=[0.369, 1.114, 0.087, -1.314, -0.009, 2.945, 0.64])
    log("place IK z", z, None if s is None else np.round(s, 3))
PRE = [0.312, 0.714, 0.121, -1.749, -0.0, 2.978, 0.627]
r.gripper(0.04)
ok = r.move_joints(PRE, secs=5); log("at PRE", ok, "wrench", wrench())
mid = np.array([CX, CY, GZ]) - 0.05 * a
ok = move_tcp_iter(r, *mid, q, secs=3, tol=0.004); log("at mid", ok, "wrench", wrench())
ok = move_tcp_iter(r, CX, CY, GZ, q, secs=3, tol=0.004); log("at grasp", ok, "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp before close", tcp.round(4))
if not ok:
    log("grasp pose not reached; stopping for inspection"); raise SystemExit
r.gripper(0.0); r.spin(0.5); fg = r.fingers(); gap = fg[0] - fg[1]
log(f"finger gap after close = {gap:.4f}")
if gap < 0.05:
    log("missed the can; reopening"); r.gripper(0.04); raise SystemExit
back = np.array([CX, CY, GZ]) - 0.12 * a
ok2 = move_tcp_iter(r, *back, q, secs=4, tries=2, tol=0.01); log("retreat", ok2)
up = np.array([CX, CY, 0.70]) - 0.10 * a
ok3 = move_tcp_iter(r, *up, q, secs=4, tries=2, tol=0.01); log("lift", ok3)
r.spin(0.5); fg = r.fingers(); log("fingers after lift", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
log("PHASE8 DONE")
