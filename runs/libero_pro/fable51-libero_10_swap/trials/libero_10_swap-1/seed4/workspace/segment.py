#!/usr/bin/env python3
"""Segment objects above the table in a camera's depth frame; print world centroid, principal axis yaw, extents, top height."""
import sys, struct
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

cam = sys.argv[1] if len(sys.argv) > 1 else "robot0_eye_in_hand"
TABLE_Z = 0.425
rclpy.init(); node = rclpy.create_node("seg")
tfbuf = Buffer(); TransformListener(tfbuf, node)
def grab(topic, T):
    got = {}
    s = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
depth = CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(grab(f"/{cam}/color/image_raw", Image), "bgr8")
info = grab(f"/{cam}/color/camera_info", CameraInfo)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
frame = f"{cam}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1)
pw = pc @ R.T + tr
valid = np.isfinite(depth) & (depth > 0)
mask = valid & (pw[..., 2] > TABLE_Z + 0.008) & (pw[..., 2] < TABLE_Z + 0.35)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1, n):
    m = lab == i
    if m.sum() < 60: continue
    P = pw[m]; c = P.mean(0)
    xy = P[:, :2] - c[:2]
    ev, evec = np.linalg.eigh(np.cov(xy.T))
    ax = evec[:, 1]; yaw = np.degrees(np.arctan2(ax[1], ax[0]))
    proj_l = xy @ ax; proj_w = xy @ evec[:, 0]
    us, vs = uu[m], vv[m]
    bgr = color[m].mean(0)
    if P[:,2].max() < TABLE_Z + 0.05:
        e = (ax * (proj_l.max()-proj_l.min())/2)
        # world->pixel for the endpoints: use nearest mask pixels
        for sgn in (1, -1):
            tgt = c[:2] + sgn * e
            d = np.linalg.norm(P[:, :2] - tgt, axis=1); k = d.argmin()
            cv2.circle(color, (int(us[k]), int(vs[k])), 6, (0, 255, 0), 2)
        cv2.imwrite("seg_debug.png", color)
    print(f"obj{i}: px=({us.mean():.0f},{vs.mean():.0f}) n={m.sum()} world c=({c[0]:.3f},{c[1]:.3f}) top={P[:,2].max():.3f} "
          f"len={proj_l.max()-proj_l.min():.3f} wid={proj_w.max()-proj_w.min():.3f} yaw={yaw:.0f}deg bgr={bgr.astype(int)}")
rclpy.shutdown()
