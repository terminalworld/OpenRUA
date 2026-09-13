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
CX, CY, ang = 0.228, 0.346, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
t = math.radians(30); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
q = quat_from_axes(a, -f)
PRE = [0.324, 0.724, 0.115, -1.747, -0.001, 2.986, 0.635]
GZ = 0.45
r.gripper(0.04)
ok = r.move_joints(PRE, secs=5)
_,_,tcp = r.hand_pose(); log("pre tcp", tcp.round(4), "expected", (np.array([CX,CY,0.445]) - 0.12*a).round(4))
log("wrench", wrench())
mid = np.array([CX, CY, GZ]) - 0.05 * a
ok = r.move_tcp_line(*mid, quat=q, secs=3, n=2); log("wrench", wrench(), ok)
ok = r.move_tcp_line(CX, CY, GZ, quat=q, secs=3, n=2); log("wrench", wrench(), ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
r.gripper(0.0); r.spin(0.5); fg=r.fingers()
log(f"finger gap after close = {fg[0]-fg[1]:.4f}")
# retreat back along the approach axis, then up
back = np.array([CX, CY, GZ]) - 0.15 * a
ok2 = r.move_tcp_line(*back, quat=q, secs=4, n=2)
if not ok2: log("retry"); r.move_tcp_line(*back, quat=q, secs=4, n=2)
r.spin(0.5); fg=r.fingers(); log("fingers after lift", fg, "gap", fg[0]-fg[1], "wrench", wrench())
log("PHASE6 DONE")
