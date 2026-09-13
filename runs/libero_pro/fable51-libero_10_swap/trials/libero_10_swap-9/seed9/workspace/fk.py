#!/usr/bin/env python3
"""Print the current hand pose (panda_hand in panda_link0 and world) via /compute_fk."""
import rclpy, yaml, numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open('/workspace/machine.yaml'))
ARM = M['actuators'][0]['joints']
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (from TF)

def main():
    rclpy.init(); node = rclpy.create_node('fk')
    js = {}
    node.create_subscription(JointState, '/joint_states', lambda m: js.setdefault('m', m), 1)
    while 'm' not in js: rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionFK, '/compute_fk'); cli.wait_for_service()
    req = GetPositionFK.Request(); req.fk_link_names = ['panda_hand']
    d = dict(zip(js['m'].name, js['m'].position))
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [d[j] for j in ARM]
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    p = r.pose_stamped[0].pose
    print('joints', [round(d[j],4) for j in ARM], 'fingers', round(d['panda_finger_joint1'],4), round(d['panda_finger_joint2'],4))
    print('hand in base: %.4f %.4f %.4f  q %.4f %.4f %.4f %.4f' % (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
    w = BASE + np.array([p.position.x,p.position.y,p.position.z])
    print('hand in world: %.4f %.4f %.4f' % tuple(w))
    rclpy.shutdown()
main()
