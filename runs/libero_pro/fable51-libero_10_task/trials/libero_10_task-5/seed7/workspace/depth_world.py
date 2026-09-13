"""Project a camera's full depth frame into world coords; save npz."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T, timeout=15.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds/1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("dw")
buf = Buffer(); TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
end = node.get_clock().now().nanoseconds/1e9 + 10
while node.get_clock().now().nanoseconds/1e9 < end:
    rclpy.spin_once(node, timeout_sec=0.2)
    if buf.can_transform("world", frame, rclpy.time.Time()): break
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; R = quat_R(q.x,q.y,q.z,q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
h,w = depth.shape
u,v = np.meshgrid(np.arange(w), np.arange(h))
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
pw = pc @ R.T + tr
np.savez(f"{cam}_world.npz", xyz=pw, depth=depth)
print("saved", cam, "cam pos", tr, "shape", pw.shape)
rclpy.shutdown()
