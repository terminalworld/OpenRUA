#!/usr/bin/env python3
"""Scene helper: TF tree dump + pixel->world for several cameras/pixels.

Usage: python3 scene.py tf
       python3 scene.py px <camera> u,v [u,v ...]
       python3 scene.py fk            # hand pose in base frame via /compute_fk
"""
import struct
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy


def grab(node, topic, msg_type, timeout=15.0, qos=1):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), qos)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic} within {timeout}s")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_to_T(t):
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    return T


def main():
    rclpy.init()
    node = rclpy.create_node("scene")
    cmd = sys.argv[1]
    if cmd == "tf":
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, q in (("/tf_static", qos), ("/tf", 10)):
            try:
                m = grab(node, topic, TFMessage, timeout=5, qos=q)
            except SystemExit as e:
                print(e); continue
            for t in m.transforms:
                tr, r = t.transform.translation, t.transform.rotation
                print(f"{topic}: {t.header.frame_id} -> {t.child_frame_id}: "
                      f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) "
                      f"q=({r.x:.4f},{r.y:.4f},{r.z:.4f},{r.w:.4f})")
    elif cmd == "px":
        cam = sys.argv[2]
        tfbuf = Buffer(); TransformListener(tfbuf, node)
        depth = grab(node, f"/{cam}/depth/image_raw", Image)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        import time
        end = time.time() + 10
        while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        T = tf_to_T(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        for uv in sys.argv[3:]:
            u, v = map(int, uv.split(","))
            z = struct.unpack_from("<f", depth.data, (v * depth.width + u) * 4)[0]
            if not np.isfinite(z) or z <= 0:
                print(f"({u},{v}): no depth ({z})"); continue
            p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
            print(f"({u},{v}): depth={z:.4f} world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    elif cmd == "fk":
        js = grab(node, "/joint_states", JointState)
        cli = node.create_client(GetPositionFK, "/compute_fk")
        cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand", "panda_link8"]
        arm = [f"panda_joint{i}" for i in range(1, 8)]
        for n, p in zip(js.name, js.position):
            if n in arm:
                req.robot_state.joint_state.name.append(n)
                req.robot_state.joint_state.position.append(p)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
        res = fut.result()
        print("error", res.error_code.val)
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            print(f"{name} [{ps.header.frame_id}]: p=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
                  f"q=({o.x:.4f},{o.y:.4f},{o.z:.4f},{o.w:.4f})")
        print("joints:", dict(zip(js.name, [round(x, 4) for x in js.position])))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
