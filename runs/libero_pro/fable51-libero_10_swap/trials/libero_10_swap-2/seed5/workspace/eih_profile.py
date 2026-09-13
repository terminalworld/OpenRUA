"""Profile the pot from the wrist depth cam: lateral offset & radius vs z relative to the hand line."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cloud import quat_R
from ctl import Ctl, TCP

def main():
    c = Ctl('prof')
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
    wx, wy, wz = pw[..., 0], pw[..., 1], pw[..., 2]
    tcp, quat = c.tcp(); Rh = quat_R(*quat); a = Rh[:, 2]; fy = Rh[:, 1]
    print('tcp', np.round(tcp, 4), 'a', np.round(a, 3), 'fy', np.round(fy, 3))
    rel = np.stack([wx-tcp[0], wy-tcp[1]], -1)
    along = rel[..., 0]*a[0]+rel[..., 1]*a[1]; lat = rel[..., 0]*fy[0]+rel[..., 1]*fy[1]
    m = (along > 0.02) & (along < 0.30) & (np.abs(lat) < 0.08) & (wz > 0.905) & (wz < 1.06)
    print('pts', m.sum())
    for z0 in np.arange(0.905, 1.05, 0.005):
        mm = m & (wz >= z0) & (wz < z0+0.005)
        if mm.sum() > 5:
            mc = mm & (np.abs(lat) < 0.008)
            near = along[mc].min() if mc.sum() else float('nan')
            print(f'z {z0:.3f}: lat[{lat[mm].min():+.4f},{lat[mm].max():+.4f}] mid={(lat[mm].min()+lat[mm].max())/2:+.4f} halfw={(lat[mm].max()-lat[mm].min())/2:.4f} nearest along={near:.4f} n={mm.sum()}')
    rclpy.shutdown()

main()
