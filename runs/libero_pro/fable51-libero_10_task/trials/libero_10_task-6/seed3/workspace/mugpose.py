#!/usr/bin/env python3
"""Measure the lying red mug from a fresh birdview capture.

Body points (z > 0.47 excludes plate rim, handle on the table, and the flat pudding)
are fit with a minimum-area rectangle -> axis direction u (toward the open end),
center, length, width. Returns a dict; run standalone to print it.
"""
import subprocess, sys
import numpy as np, cv2

BIRD = dict(cam="birdview")


def capture():
    subprocess.run([sys.executable, "/workspace/locate.py", "birdview"], check=True,
                   stdout=subprocess.DEVNULL)


def cloud(cam="birdview"):
    d = np.load(f"{cam}_depth.npy"); meta = np.load(f"{cam}_meta.npy", allow_pickle=True).item()
    K, T = meta["K"], meta["T"]; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    H, W = d.shape; us, vs = np.meshgrid(np.arange(W), np.arange(H))
    P = (T[:3, :3] @ np.vstack([((us - cx) * d / fx).ravel(), ((vs - cy) * d / fy).ravel(), d.ravel()])
         + T[:3, 3:4]).reshape(3, H, W)
    return P


def measure(P, region=(0.12, 0.40, -0.25, 0.25), zmin=0.48, zmax=0.60, rim_toward=None):
    x0, x1, y0, y1 = region
    m = (P[0] > x0) & (P[0] < x1) & (P[1] > y0) & (P[1] < y1) & (P[2] > zmin) & (P[2] < zmax)
    pts = np.c_[P[0][m], P[1][m]].astype(np.float32)
    if len(pts) < 20:
        raise RuntimeError("mug body not found")
    (cx, cy), (w, h), ang = cv2.minAreaRect(pts)
    ang = np.radians(ang)
    if w >= h:
        L, Wd, u = w, h, np.array([np.cos(ang), np.sin(ang)])
    else:
        L, Wd, u = h, w, np.array([-np.sin(ang), np.cos(ang)])
    c = np.array([cx, cy])
    # open end: default = the end farther from the robot base (+x), or nearest rim_toward point
    if rim_toward is not None:
        if np.dot(np.array(rim_toward) - c, u) < 0:
            u = -u
    elif u[0] < 0:
        u = -u
    v = np.array([-u[1], u[0]])
    zs = P[2][m]
    top = float(np.percentile(zs, 99))
    rim = c + u * L / 2
    bottom = c - u * L / 2
    return dict(center=c, u=u, v=v, length=float(L), width=float(Wd), top=top,
                rim=rim, bottom=bottom, angle_deg=float(np.degrees(np.arctan2(u[1], u[0]))), n=int(m.sum()))


def report(mp):
    print("mug body: center (%.3f,%.3f) axis %.1f deg  len %.3f width %.3f top z %.3f  n=%d"
          % (*mp["center"], mp["angle_deg"], mp["length"], mp["width"], mp["top"], mp["n"]))
    print("  rim end (%.3f,%.3f)  bottom end (%.3f,%.3f)  u=%s v=%s"
          % (*mp["rim"], *mp["bottom"], mp["u"].round(3), mp["v"].round(3)))


if __name__ == "__main__":
    if "--nocap" not in sys.argv:
        capture()
    P = cloud()
    np.save("P_bird.npy", P)
    report(measure(P))
