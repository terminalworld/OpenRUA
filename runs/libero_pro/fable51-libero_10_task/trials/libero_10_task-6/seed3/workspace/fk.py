#!/usr/bin/env python3
"""Print the panda_hand pose (base frame and world frame) via /compute_fk."""
import rclpy, yaml
import numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")["joints"]
BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (tf2_echo)


def main():
    rclpy.init()
    node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    seed = JointState()
    for n, p in zip(js["m"].name, js["m"].position):
        if n in ARM:
            seed.name.append(n); seed.position.append(p)
    req.robot_state.joint_state = seed
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    print("joints", dict(zip(seed.name, [round(x, 4) for x in seed.position])))
    print("hand in base :", pos.round(4), "quat", [round(v, 4) for v in (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)])
    print("hand in world:", (pos + BASE).round(4), " tcp in world:", end=" ")
    q = p.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    zaxis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
    print((pos + BASE + 0.1034 * zaxis).round(4), "hand z-axis", zaxis.round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
