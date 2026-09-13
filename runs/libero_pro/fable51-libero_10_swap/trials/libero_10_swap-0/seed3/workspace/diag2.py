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
    if 'm' not in wr: return None
    f=wr['m'].wrench.force; t=wr['m'].wrench.torque
    return np.round([f.x,f.y,f.z,t.x,t.y,t.z],2)
log("wrench at start", wrench())
q0=r.arm_q(); log("q0",np.round(q0,3))
_,_,tcp=r.hand_pose(); log("tcp",tcp.round(4))
for z in (0.60, 0.55, 0.52, 0.50):
    q = r.solve_ik(-0.244,-0.173,z, topdown_quat(0.0))
    log("target z",z,"q",np.round(q,3))
    ok = r.move_joints(q, secs=3, retries=0)
    cur=r.arm_q()
    log("per-joint err", np.round(np.array(cur)-np.array(q),3))
    _,_,tcp=r.hand_pose(); log("tcp",tcp.round(4), "wrench", wrench())
    if not ok: break
log("DIAG2 DONE")
