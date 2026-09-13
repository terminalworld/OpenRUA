"""Centroid/extent in world of pixels whose world z is within [zlo, zhi] in an ROI."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rob import quat_to_R
cam = sys.argv[1]; zlo, zhi = float(sys.argv[2]), float(sys.argv[3])
u0,v0,u1,v1 = map(int, sys.argv[4:8]) if len(sys.argv) > 4 else (0,0,640,480)
rclpy.init(); node = rclpy.create_node('blob')
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(Image, f'/{cam}/depth/image_raw', lambda m: got.setdefault('d', m), 1)
node.create_subscription(CameraInfo, f'/{cam}/color/camera_info', lambda m: got.setdefault('i', m), 1)
while 'd' not in got or 'i' not in got: rclpy.spin_once(node, timeout_sec=0.2)
d, info = got['d'], got['i']
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
frame = f'{cam}_optical_frame'
while not tfbuf.can_transform('world', frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform('world', frame, rclpy.time.Time())
q = t.transform.rotation
T = np.eye(4); T[:3,:3] = quat_to_R(q.x,q.y,q.z,q.w); T[:3,3] = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
vv, uu = np.mgrid[0:d.height, 0:d.width]
Z = depth
P = np.stack([(uu-cx)*Z/fx, (vv-cy)*Z/fy, Z, np.ones_like(Z)], -1) @ T.T
W = P[..., :3]
m = (W[...,2] >= zlo) & (W[...,2] <= zhi) & np.isfinite(Z)
roi = np.zeros_like(m); roi[v0:v1, u0:u1] = True; m &= roi
pts = W[m]
print('n', len(pts))
if len(pts):
    print('centroid', pts.mean(0).round(4))
    print('min', pts.min(0).round(4), 'max', pts.max(0).round(4))
    ys, xs = np.nonzero(m); print('pixel bbox u', xs.min(), xs.max(), 'v', ys.min(), ys.max())
rclpy.shutdown()
