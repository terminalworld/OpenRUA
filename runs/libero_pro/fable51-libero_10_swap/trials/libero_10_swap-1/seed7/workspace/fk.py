#!/usr/bin/env python3
"""Print current hand pose (panda_hand) in base and world frames via /compute_fk."""
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

BASE_W = np.array([-0.51, 0.0, 0.42])
M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]

rclpy.init()
node = rclpy.create_node("fk")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
cur = dict(zip(js["m"].name, js["m"].position))
import sys
if len(sys.argv) > 1:
    cur.update(zip(ARM, map(float, sys.argv[1].split(","))))
print("joints:", {k: round(v, 4) for k, v in cur.items()})
cli = node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(10)
req = GetPositionFK.Request()
req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = ARM
req.robot_state.joint_state.position = [cur[j] for j in ARM]
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
r = fut.result()
p = r.pose_stamped[0].pose
print("err", r.error_code.val, "frame", r.pose_stamped[0].header.frame_id)
pos = np.array([p.position.x, p.position.y, p.position.z])
q = p.orientation
print("hand base:", pos.round(4), "quat xyzw:", np.array([q.x, q.y, q.z, q.w]).round(4))
print("hand world:", (pos + BASE_W).round(4))
x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])
print("R (cols = hand x,y,z in base):\n", R.round(3))
print("tcp world:", (pos + BASE_W + 0.1034 * R[:, 2]).round(4))
rclpy.shutdown()
