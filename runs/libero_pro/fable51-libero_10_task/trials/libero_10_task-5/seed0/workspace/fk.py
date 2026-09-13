#!/usr/bin/env python3
"""Print current hand pose (panda_hand in panda_link0 and world) via /compute_fk."""
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE = np.array([-0.75, 0.0, 0.912])  # world -> panda_link0 (tf2_echo)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("fk_probe")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [cur[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    res = fut.result()
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    q = p.orientation
    R = quat_R(q.x, q.y, q.z, q.w)
    tcp = pos + 0.1034 * R[:, 2]
    print("joints:", [round(cur[j], 4) for j in ARM])
    print("fingers:", cur.get("panda_finger_joint1"), cur.get("panda_finger_joint2"))
    print("hand in world:", pos.round(4), "quat xyzw:", [round(v, 4) for v in (q.x, q.y, q.z, q.w)])
    print("(FK is already in world frame)")
    print("tcp  in world:", tcp.round(4))
    print("hand z-axis (approach) in world:", R[:, 2].round(3), " x-axis:", R[:, 0].round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
