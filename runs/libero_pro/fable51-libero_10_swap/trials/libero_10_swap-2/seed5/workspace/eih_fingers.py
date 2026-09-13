"""Measure fingertip lateral extents relative to the hand line from the wrist depth camera."""
import numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cloud import quat_R
from ctl import Ctl

c = Ctl('fing')
buf = Buffer(); TransformListener(buf, c.node)
got = {}
c.node.create_subscription(Image, '/robot0_eye_in_hand/depth/image_raw', lambda m: got.setdefault('d', m), 1)
c.node.create_subscription(CameraInfo, '/robot0_eye_in_hand/color/camera_info', lambda m: got.setdefault('i', m), 1)
while 'd' not in got or 'i' not in got: c.spin()
while not buf.can_transform('world', 'robot0_eye_in_hand_optical_frame', rclpy.time.Time()): c.spin()
t = buf.lookup_transform('world', 'robot0_eye_in_hand_optical_frame', rclpy.time.Time())
q = t.transform.rotation; tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
d = got['d']; D = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
k = got['i'].k; fx, fy_, cx, cy = k[0], k[4], k[2], k[5]
H, W = D.shape; v, u = np.mgrid[0:H, 0:W]
pc = np.stack([(u-cx)*D/fx, (v-cy)*D/fy_, D], -1); R = quat_R(q.x, q.y, q.z, q.w); pw = pc@R.T+tr
tcp, quat = c.tcp(); Rh = quat_R(*quat); a = Rh[:, 2]; fy = Rh[:, 1]; hx = Rh[:, 0]
rel = pw - tcp
along = rel@a; lat = rel@fy; up = rel@hx
print('tcp', np.round(tcp, 4), 'fingers', np.round(c.fingers(), 4), 'cam rel tcp: along %.3f lat %.3f hx %.3f' % ((tr-tcp)@a, (tr-tcp)@fy, (tr-tcp)@hx))
m = (along > -0.06) & (along < 0.02) & (np.abs(lat) < 0.07) & (np.abs(up) < 0.03) & (D < 0.2)
print('near pts', m.sum())
for lo, hi in [(-0.06, -0.05), (-0.05, -0.04), (-0.04, -0.03), (-0.03, -0.02), (-0.02, -0.01), (-0.01, 0.0), (0.0, 0.01)]:
    mm = m & (along >= lo) & (along < hi)
    if mm.sum() < 5: continue
    L = lat[mm]; U = up[mm]
    neg = L[L < 0]; pos = L[L > 0]
    print(f'along[{lo:+.2f},{hi:+.2f}) n={mm.sum()} lat- [{neg.min() if neg.size else np.nan:+.4f},{neg.max() if neg.size else np.nan:+.4f}]  lat+ [{pos.min() if pos.size else np.nan:+.4f},{pos.max() if pos.size else np.nan:+.4f}]  up[{U.min():+.4f},{U.max():+.4f}]')
rclpy.shutdown()
