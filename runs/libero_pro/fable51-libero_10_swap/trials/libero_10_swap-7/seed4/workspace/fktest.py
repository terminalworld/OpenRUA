from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import rclpy
cli = A.node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request()
req.header.frame_id = ""
req.fk_link_names = ["panda_link8", "panda_hand"]
s = JointState(); s.name = list(A.arm_q() and __import__('arm').JOINTS); s.position = [float(v) for v in A.arm_q()]
req.robot_state.joint_state = s
fut = cli.call_async(req)
rclpy.spin_until_future_complete(A.node, fut, timeout_sec=30)
r = fut.result()
A.log("FK code", r.error_code.val)
for n, ps in zip(r.fk_link_names, r.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    A.log(n, ps.header.frame_id, (round(p.x,4), round(p.y,4), round(p.z,4)), (round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4)))
    # feed back into IK, with frame_id empty, ik_link = n
    A.ik_link = n
    pw = np.array([p.x, p.y, p.z]) + A.base_off
    A.log(" IK feedback:", A.solve_ik(pw, (q.x,q.y,q.z,q.w), at_tcp=False))
