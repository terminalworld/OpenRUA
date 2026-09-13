"""Crude arm-vs-scene collision check using link FK and the table heightmap."""
import numpy as np
from rob import *

HM = np.load("snaps/hm.npy")
X0, Y0, RES = -0.45, -0.35, 0.005
HM = np.where(HM > 1.22, np.nan, HM)  # strip the robot's own points from the old cloud
_xs = X0 + RES * np.arange(HM.shape[0])[:, None]; _ys = Y0 + RES * np.arange(HM.shape[1])[None, :]
_robot = (HM > 1.15) & (_xs > -0.30) & (_xs < -0.14) & (_ys > -0.15) & (_ys < 0.15)
HM = np.where(_robot, np.nan, HM)
BOT = np.array([-0.166, 0.073])
LINKS = [f"panda_link{i}" for i in range(1, 9)] + ["panda_hand"]
# capsules: (link a, link b, radius)
CAPS = [("panda_link3", "panda_link4", 0.06), ("panda_link4", "panda_link5", 0.06),
        ("panda_link5", "panda_link6", 0.06), ("panda_link6", "panda_link7", 0.05),
        ("panda_link7", "panda_hand", 0.045)]


def fk_all(r, q):
    req = GetPositionFK.Request(); req.header.frame_id = ""
    req.fk_link_names = LINKS
    req.robot_state.joint_state.name = list(JOINTS)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    return {n: np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
            for n, p in zip(LINKS, res.pose_stamped)}


def height_at(x, y, rad):
    i0 = int((x - rad - X0) / RES); i1 = int((x + rad - X0) / RES) + 1
    j0 = int((y - rad - Y0) / RES); j1 = int((y + rad - Y0) / RES) + 1
    i0, j0 = max(i0, 0), max(j0, 0)
    if i1 <= i0 or j1 <= j0 or i0 >= HM.shape[0] or j0 >= HM.shape[1]:
        return 0.90
    v = HM[i0:i1, j0:j1]; v = v[np.isfinite(v)]
    return float(v.max()) if v.size else 0.90


def clearance(r, q, ignore_bottle=True):
    """Min vertical clearance (m) of arm capsules above scene heightmap. Negative = collision."""
    P = fk_all(r, q)
    worst = 9.0; where = None
    for a, b, rad in CAPS:
        pa, pb = P[a], P[b]
        for t in np.linspace(0, 1, 6):
            p = pa + t * (pb - pa)
            if ignore_bottle and np.hypot(p[0] - BOT[0], p[1] - BOT[1]) < 0.05 + rad:
                continue
            h = height_at(p[0], p[1], rad)
            c = (p[2] - rad) - h
            if c < worst:
                worst, where = c, (a, np.round(p, 3), round(h, 3))
    return worst, where
