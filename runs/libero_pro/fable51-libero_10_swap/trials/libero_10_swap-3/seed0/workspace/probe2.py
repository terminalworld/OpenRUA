from robot import *
import robot
r = Robot("probe2")
q0 = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = q0
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
ps = fut.result().pose_stamped[0]
print("FK frame:", repr(ps.header.frame_id), "pos", ps.pose.position)
# IK with world coords (no offset)
robot.BASE = np.zeros(3)
q = r.ik_hand([-0.203, 0.0, 1.2696], [1,0,0,0])
print("IK world-coords:", None if q is None else np.round(q,3))
robot.BASE = np.array([-0.66,0,0.912])
q = r.ik_hand([-0.203, 0.0, 1.2696], [1,0,0,0])
print("IK base-offset:", None if q is None else np.round(q,3))
