import rclpy, yaml, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open('/workspace/machine.yaml')); ARM = M['actuators'][0]['joints']
rclpy.init(); node = rclpy.create_node('iktest')
js = {}
node.create_subscription(JointState, '/joint_states', lambda m: js.setdefault('m', m), 1)
while 'm' not in js: rclpy.spin_once(node, timeout_sec=0.2)
d = dict(zip(js['m'].name, js['m'].position))
cli = node.create_client(GetPositionIK, '/compute_ik'); cli.wait_for_service()
for label, pos in [('world', (-0.2030, 0.0, 1.2696)), ('base', (0.457, 0.0, 0.358))]:
    req = GetPositionIK.Request(); req.ik_request.group_name = 'panda_arm'
    req.ik_request.pose_stamped.header.frame_id = ''
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = pos
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = 0.9996, 0.0, -0.0284, 0.0
    req.ik_request.robot_state.joint_state.name = ARM
    req.ik_request.robot_state.joint_state.position = [d[j] for j in ARM]
    req.ik_request.timeout.sec = 5
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    r = fut.result()
    print(label, 'err', r.error_code.val, [round(v,3) for n,v in zip(r.solution.joint_state.name, r.solution.joint_state.position) if n in ARM])
print('current', [round(d[j],3) for j in ARM])
rclpy.shutdown()
