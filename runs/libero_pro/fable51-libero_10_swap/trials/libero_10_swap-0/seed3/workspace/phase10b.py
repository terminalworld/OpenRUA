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
CX, CY, GZ, ang = 0.2691, 0.3535, 0.455, math.radians(31.1)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)]); q = quat_from_axes(a, -f)
log("wrench before", wrench())
r.gripper(0.0); r.spin(0.5); fg = r.fingers(); gap = fg[0] - fg[1]
log(f"finger gap after close = {gap:.4f}", "wrench", wrench())
if gap < 0.04:
    log("missed the can; reopening"); r.gripper(0.04); raise SystemExit
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
back = tcp - 0.12 * a
ok2 = move_tcp_iter(r, *back, q, secs=4, tries=2, tol=0.01); log("retreat", ok2)
r.spin(0.5); fg = r.fingers(); log("fingers after retreat", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
up = np.array([CX - 0.05, CY, 0.70]) - 0.10 * a
ok3 = move_tcp_iter(r, *up, q, secs=4, tries=2, tol=0.01); log("lift", ok3)
r.spin(0.5); fg = r.fingers(); log("fingers after lift", fg, "gap", round(fg[0] - fg[1], 4), "wrench", wrench())
_, _, tcp = r.hand_pose(); log("tcp", tcp.round(4))
log("PHASE10B DONE")
