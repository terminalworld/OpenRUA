"""cloud(cam) -> (X,Y,Z) world-frame arrays from a camera's current depth frame (TF-based)."""
import struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds/1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")

def cloud(cam):
    if not rclpy.ok(): rclpy.init()
    node = rclpy.create_node("cloud_"+cam)
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    d = CvBridge().imgmsg_to_cv2(depth, desired_encoding="passthrough").astype(np.float64)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds/1e9 + 10
    while node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()): break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    P = np.stack([(u-cx)*d/fx, (v-cy)*d/fy, d], -1).reshape(-1, 3)
    W = P @ R.T + T
    ok = np.isfinite(d.reshape(-1)) & (d.reshape(-1) > 0)
    node.destroy_node()
    return W[ok, 0], W[ok, 1], W[ok, 2], (u.reshape(-1)[ok], v.reshape(-1)[ok])

if __name__ == "__main__":
    import sys
    X, Y, Z, _ = cloud(sys.argv[1])
    print("points", X.size, "x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]" % (X.min(), X.max(), Y.min(), Y.max(), Z.min(), Z.max()))
    # table plane sanity
    m = (Z > 0.85) & (Z < 0.95) & (abs(X) < 0.4) & (abs(Y) < 0.4)
    print("table-ish z median", np.median(Z[m]) if m.any() else None)
