import struct, sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds/1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub); return got["m"]
def cloud(cam):
    rclpy.init(); node = rclpy.create_node("cloud"); tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image); info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width).astype(float)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(depth.width), np.arange(depth.height))
    pc = np.stack([(u-cx)*D/fx, (v-cy)*D/fy, D], -1).reshape(-1,3)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds/1e9 + 10
    while node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()): break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time()); q = t.transform.rotation
    x,y,z,w = q.x,q.y,q.z,q.w
    R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    T = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    P = (R@pc.T).T + T; ok = np.isfinite(P).all(1)
    rclpy.shutdown(); return P[ok], (u.reshape(-1)[ok], v.reshape(-1)[ok])
if __name__ == "__main__":
    P,_ = cloud(sys.argv[1]); np.save(sys.argv[2], P); print(P.shape)
