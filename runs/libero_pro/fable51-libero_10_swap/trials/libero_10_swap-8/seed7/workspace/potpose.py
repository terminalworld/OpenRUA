"""Estimate a pot's body centre and handle direction from a fresh birdview depth frame."""
import numpy as np, subprocess, sys
def measure(fresh=True):
    if fresh:
        subprocess.run(["timeout","60","python3","tools/perception/cam_snap.py","/birdview/depth/image_raw","snaps/bird_now.png"],check=True,capture_output=True)
    d = np.load("snaps/bird_now.npy"); fx=fy=579.4112549695428; cx=320; cy=240
    H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
    wx = -0.2 + (vs-cy)*d/fy; wy = (us-cx)*d/fx; wz = 3.0 - d
    return wx, wy, wz
def pot_pose(wx, wy, wz, cx0, cy0, support_z):
    box = (np.abs(wx-cx0)<0.09)&(np.abs(wy-cy0)<0.09)
    h = wz - support_z
    top = box & (h > 0.145)                    # knob
    if top.sum() < 3: return None
    kc = np.array([wx[top].mean(), wy[top].mean()])
    body = box & (h > 0.12) & (h < 0.145)      # lid + handle bar
    far = body & (np.hypot(wx-kc[0], wy-kc[1]) > 0.045)   # beyond the lid radius -> handle
    if far.sum() < 3: return kc, None, wz[top].max()
    hc = np.array([wx[far].mean(), wy[far].mean()])
    ang = np.degrees(np.arctan2(hc[1]-kc[0]*0-kc[1], hc[0]-kc[0]))
    return kc, ang, wz[top].max()
if __name__ == "__main__":
    wx, wy, wz = measure()
    for name,(x,y,sz) in {"A@stove":(0.116,0.003,0.93),"B":(-0.076,0.220,0.899)}.items():
        print(name, pot_pose(wx, wy, wz, x, y, sz))
