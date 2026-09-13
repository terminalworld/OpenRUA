#!/usr/bin/env python3
"""Segment objects in the birdview height map; print world centroids."""
import cv2
import numpy as np

depth = np.load("birdview_depth.npy")
color = cv2.imread("birdview.png")
fx = fy = 579.4112549695428
cx, cy = 320.0, 240.0
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
z_cam = depth
X_cam = (uu - cx) * z_cam / fx
Y_cam = (vv - cy) * z_cam / fy
wx = Y_cam - 0.2
wy = X_cam
wz = 3.0 - z_cam

# table height: mode of z in the table region (dark wood)
tbl = wz[(np.abs(wx) < 0.6) & (np.abs(wy) < 0.6)]
hist, edges = np.histogram(tbl[np.isfinite(tbl)], bins=400, range=(0, 2))
table_z = edges[np.argmax(hist)]
print(f"table_z ~ {table_z:.3f}")

mask = (wz > table_z + 0.008) & (np.abs(wx) < 0.7) & (np.abs(wy) < 0.7)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
for i in range(1, n):
    area = stats[i, cv2.CC_STAT_AREA]
    if area < 30:
        continue
    m = lab == i
    zs = wz[m]
    bgr = color[m].mean(axis=0)
    u, v = cents[i]
    print(f"blob {i}: px=({u:.0f},{v:.0f}) area={area} "
          f"world=({wx[m].mean():.3f},{wy[m].mean():.3f}) "
          f"top_z={np.percentile(zs,95):.3f} min_z={zs.min():.3f} "
          f"bgr={bgr.astype(int)} "
          f"bbox_x=({wx[m].min():.3f},{wx[m].max():.3f}) bbox_y=({wy[m].min():.3f},{wy[m].max():.3f})")
