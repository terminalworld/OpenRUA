from rob import *
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    f=wr['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
CX, CY, ang = 0.230, 0.342, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
q = quat_from_axes(a, -f)
GZ = 0.45
r.gripper(0.04)
mid = np.array([CX, CY, GZ]) - 0.06 * a
ok = move_tcp_iter(r, *mid, q, secs=3); log("wrench", wrench(), ok)
ok = move_tcp_iter(r, CX, CY, GZ, q, secs=3); log("wrench", wrench(), ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
r.gripper(0.0); r.spin(0.5); fg=r.fingers()
log(f"finger gap after close = {fg[0]-fg[1]:.4f}")
back = np.array([CX, CY, GZ]) - 0.15 * a
ok2 = r.move_tcp_line(*back, quat=q, secs=4, n=2)
if not ok2: log("retry"); r.move_tcp_line(*back, quat=q, secs=4, n=2)
r.spin(0.5); fg=r.fingers(); log("fingers after lift", fg, "gap", fg[0]-fg[1], "wrench", wrench())
log("PHASE6B DONE")
