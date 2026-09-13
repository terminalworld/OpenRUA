#!/usr/bin/env python3
"""Print the current hand pose (panda_hand, and TCP) in panda_link0 and world frames via /compute_fk."""
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]
BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (identity rotation)
TCP = M["hand"]["tcp_offset_m"]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def main():
    rclpy.init(); node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    print("joints:", ",".join(f"{d[j]:.4f}" for j in ARM))
    print("fingers:", d.get("panda_finger_joint1"), d.get("panda_finger_joint2"))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [d[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    res = fut.result()
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    q = p.orientation
    R = quat_R(q.x, q.y, q.z, q.w)
    tcp = pos + TCP * R[:, 2]
    print(f"hand(base): {pos.round(4)} quat xyzw=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    print(f"hand(world): {(pos+BASE).round(4)}   tcp(world): {(tcp+BASE).round(4)}")
    print("hand z-axis (approach) in world:", R[:, 2].round(3), " x-axis (finger-open dir):", R[:, 0].round(3))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
