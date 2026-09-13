from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot()
cli = r.node.create_client(GetPositionFK, '/compute_fk')
cli.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request()
req.header.frame_id = ''
req.fk_link_names = ['panda_link8', 'panda_hand']
s = JointState(); s.name = list(ARM); s.position = r.arm_q()
req.robot_state.joint_state = s
fut = cli.call_async(req)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
res = fut.result()
print('code', res.error_code.val)
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4), '|', round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4))
print('TF hand world', r.hand_world())
