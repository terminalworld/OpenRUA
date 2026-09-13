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
SX, SY = -0.238, -0.1765
r.spin(0.5); log("fingers", r.fingers())
assert r.move_tcp_line(SX, SY, 0.56, yaw=0.0, secs=3, n=2)
log("wrench", wrench())
assert r.move_tcp_line(SX, SY, 0.50, yaw=0.0, secs=3, n=2)
log("wrench", wrench())
ok = r.move_tcp_line(SX, SY, 0.45, yaw=0.0, secs=3, n=2)
log("wrench", wrench(), "ok", ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
f = r.gripper(0.0)
r.spin(0.5); f=r.fingers()
log(f"finger gap after close = {f[0]-f[1]:.4f}")
ok2 = r.move_tcp_line(SX, SY, 0.75, yaw=0.0, secs=4, n=3)
if not ok2:
    log("retry lift"); r.move_tcp_line(SX, SY, 0.75, yaw=0.0, secs=4, n=2)
r.spin(0.5); f=r.fingers(); log("fingers after lift", f, "gap", f[0]-f[1], "wrench", wrench())
log("PHASE1D DONE")
