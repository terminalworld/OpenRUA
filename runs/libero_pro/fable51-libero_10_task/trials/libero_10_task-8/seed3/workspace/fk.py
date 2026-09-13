#!/usr/bin/env python3
"""Print current hand + TCP pose (world frame) via MoveIt FK, and finger gap."""
import time
import numpy as np, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from px import q2R

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation (TF, R=I)
TCP = 0.1034


def main():
    rclpy.init(); node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    end = time.time() + 10
    while "m" not in js and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    m = js["m"]; pos = dict(zip(m.name, m.position))
    print("arm:", ", ".join(f"{pos[j]:.4f}" for j in ARM))
    print("fingers:", pos["panda_finger_joint1"], pos["panda_finger_joint2"],
          "gap=", round(pos["panda_finger_joint1"] - pos["panda_finger_joint2"], 4))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [pos[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        print("FK failed", r); return
    p = r.pose_stamped[0].pose
    hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE
    R = q2R(p.orientation)
    tcp = hand + TCP * R[:, 2]
    print("hand world:", hand.round(4), "quat xyzw:", [round(v, 4) for v in (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)])
    print("tcp  world:", tcp.round(4))
    print("hand axes (world): x", R[:, 0].round(3), "y", R[:, 1].round(3), "z", R[:, 2].round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
