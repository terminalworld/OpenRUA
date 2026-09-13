#!/usr/bin/env python3
"""Print world->panda_link0, hand FK (via /compute_fk), finger gap."""
import rclpy, yaml, sys
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from tf2_msgs.msg import TFMessage
import numpy as np

def main():
    rclpy.init(); node = rclpy.create_node("state_probe")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    tfs = {}
    def tfcb(m):
        for t in m.transforms:
            tfs[t.child_frame_id] = t
    node.create_subscription(TFMessage, "/tf", tfcb, 10)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.2)
        if "m" in js and "panda_link0" in tfs: break
    m = js["m"]
    d = dict(zip(m.name, m.position))
    print("joints:", {k: round(v,4) for k,v in d.items()})
    if "panda_link0" in tfs:
        t = tfs["panda_link0"].transform
        print("world->panda_link0: t=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)" % (t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = ["panda_hand"]
    arm = [f"panda_joint{i}" for i in range(1,8)]
    req.robot_state.joint_state.name = arm
    req.robot_state.joint_state.position = [d[a] for a in arm]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r is None: print("FK no answer"); return
    p = r.pose_stamped[0].pose
    print("hand (base frame): p=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f) frame=%s err=%d" % (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w, r.pose_stamped[0].header.frame_id, r.error_code.val))
    rclpy.shutdown()
main()
