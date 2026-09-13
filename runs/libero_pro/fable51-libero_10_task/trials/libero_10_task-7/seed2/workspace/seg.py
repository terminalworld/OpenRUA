"""Depth-based object segmentation into world-frame clusters (library form of locate.py)."""
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from rclpy.time import Time
from cv_bridge import CvBridge

TABLE_Z = 0.425


def _quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def cloud(node, buf, cam):
    """Return (P[H,W,3] world points, color[H,W,3], valid mask) for a fresh frame."""
    got = {}
    subs = [node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1),
            node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1),
            node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)]
    frame = f"{cam}_optical_frame"
    while not all(k in got for k in "dci") or not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    for s in subs: node.destroy_subscription(s)
    depth = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
    K = np.array(got["i"].k).reshape(3, 3)
    t = buf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = _quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    vv, uu = np.mgrid[0:H, 0:W]
    X = (uu - K[0, 2]) * depth / K[0, 0]; Y = (vv - K[1, 2]) * depth / K[1, 1]
    P = np.stack([X, Y, depth], -1) @ R.T + T
    valid = np.isfinite(depth) & (depth > 0.05)
    return P, color, valid


def clusters(node, buf, cam, minh=0.012, maxh=0.12, table_z=TABLE_Z, min_area=15):
    """Objects above the table (excluding tall things like the arm): list of dicts."""
    P, color, valid = cloud(node, buf, cam)
    mask = (valid & (P[..., 2] > table_z + minh) & (P[..., 2] < table_z + maxh)).astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_area: continue
        pts = P[lab == i]
        xy = pts[:, :2]; c = xy.mean(0)
        u, s, vt = np.linalg.svd(xy - c, full_matrices=False)
        axis = vt[0]
        out.append(dict(centre=c, ztop=pts[:, 2].max(), area=int(stats[i, 4]), n=len(pts),
                        xmin=pts[:, 0].min(), xmax=pts[:, 0].max(), ymin=pts[:, 1].min(), ymax=pts[:, 1].max(),
                        axis_deg=float(np.degrees(np.arctan2(axis[1], axis[0]))),
                        length=float(((xy - c) @ axis).ptp()), width=float(((xy - c) @ vt[1]).ptp()),
                        pts=pts, bgr=color[lab == i].mean(0)))
    return out


def nearest(cls, xy, maxd=0.06):
    best = None
    for c in cls:
        d = np.linalg.norm(c["centre"] - np.asarray(xy))
        if d < maxd and (best is None or d < best[0]):
            best = (d, c)
    return None if best is None else best[1]


def describe(c):
    return (f"centre=({c['centre'][0]:.4f},{c['centre'][1]:.4f}) ztop={c['ztop']:.3f} n={c['n']} "
            f"x[{c['xmin']:.3f},{c['xmax']:.3f}] y[{c['ymin']:.3f},{c['ymax']:.3f}] "
            f"axis={c['axis_deg']:.1f}deg L={c['length']:.3f} W={c['width']:.3f}")
