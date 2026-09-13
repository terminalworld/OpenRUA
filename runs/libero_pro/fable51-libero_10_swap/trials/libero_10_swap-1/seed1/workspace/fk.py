#!/usr/bin/env python3
"""Print the current panda_hand pose (base frame and world frame) via /compute_fk."""
import rclpy
import numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_W = np.array([-0.51, 0.0, 0.42])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    print("joints:", [round(d[j], 4) for j in ARM])
    print("fingers:", round(d["panda_finger_joint1"], 4), round(d["panda_finger_joint2"], 4))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [d[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        raise SystemExit(f"FK failed: {res}")
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    R = quat_R(*q)
    tcp = pos + 0.1034 * R[:, 2]
    print("frame:", res.pose_stamped[0].header.frame_id)
    print("hand base:", pos.round(4), "quat xyzw:", np.round(q, 4))
    print("hand world:", (pos + BASE_W).round(4))
    print("tcp world:", (tcp + BASE_W).round(4))
    print("hand axes (world): x=", R[:, 0].round(3), "y=", R[:, 1].round(3), "z=", R[:, 2].round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
