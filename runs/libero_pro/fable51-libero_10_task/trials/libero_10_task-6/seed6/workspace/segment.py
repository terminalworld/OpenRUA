#!/usr/bin/env python3
"""Segment table objects in birdview point cloud by height above table; report clusters."""
import numpy as np
import cv2

TABLE_Z = 0.426
xyz = np.load("birdview_xyz.npy")
bgr = np.load("birdview_bgr.npy")
z = xyz[..., 2]
# table region: within table footprint (x in -0.6..0.6, y in -0.8..0.8), exclude the robot (x < -0.3 near y=0 handled by color later)
above = (z > TABLE_Z + 0.008) & (z < TABLE_Z + 0.4) & np.isfinite(z)
mask = above.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
out = bgr.copy()
for i in range(1, n):
    area = stats[i, cv2.CC_STAT_AREA]
    if area < 15:
        continue
    m = lab == i
    pts = xyz[m]
    col = bgr[m].mean(0)
    x0, y0, x1, y1 = pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max()
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    print(f"comp {i}: px_area={area} centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world center=({cx:.3f},{cy:.3f}) x[{x0:.3f},{x1:.3f}] y[{y0:.3f},{y1:.3f}] "
          f"ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} bgr={col.round(0)}")
    x, y, w, h = stats[i, 0], stats[i, 1], stats[i, 2], stats[i, 3]
    cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 1)
    cv2.putText(out, str(i), (x, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
cv2.imwrite("birdview_seg.png", out)
