from lib import *
import lib
r = Robot("t2")
req = GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link0","panda_link1"]
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=r.arm_q()
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for ps in fut.result().pose_stamped:
    print(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4))
# try IK without base offset
lib.BASE_IN_WORLD = np.zeros(3)
for name, p in [("cur_hand", (-0.2035,0,1.2701))]:
    q = r.ik_world(p, (0.9996,0,-0.0284,0), at_tcp=False)
    print(name, None if q is None else np.round(q,3))
for name, p in [("aboveA", (-0.206,-0.195,1.20)), ("graspA", (-0.206,-0.195,1.00)),("stoveC", (0.188,0.03,1.00))]:
    q = r.ik_world(p, down_quat(90))
    print(name, None if q is None else np.round(q,3))
