#!/usr/bin/env python3
"""Project a camera's depth image to world points; save as npy (H,W,3)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
sys.path.insert(0, '/workspace')
from px_at_z import quat_to_R
cam = sys.argv[1]
d = np.load(f'{cam}_depth.npy')
rclpy.init(); node = rclpy.create_node('cloud')
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(CameraInfo, f'/{cam}/color/camera_info', lambda m: got.setdefault('m', m), 1)
while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
info = got['m']; frame = f'{cam}_optical_frame'
while not tfbuf.can_transform('world', frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform('world', frame, rclpy.time.Time()); q = t.transform.rotation
R = quat_to_R(q.x, q.y, q.z, q.w); o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = d.shape; v, u = np.mgrid[0:H, 0:W]
pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
pw = pc @ R.T + o
np.save(f'{cam}_cloud.npy', pw); print('cam origin', o, 'saved', pw.shape)
rclpy.shutdown()
