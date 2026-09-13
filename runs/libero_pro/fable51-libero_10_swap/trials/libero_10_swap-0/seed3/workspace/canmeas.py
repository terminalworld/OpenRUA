from refine import *
import sys
r = Robot()
cen, pts = refine(r, 0.24, 0.34, 0.44, 0.50, rad=0.12, cam="birdview")
pts = np.asarray(pts)
ridge = pts[pts[:, 2] > 0.475]
c0 = ridge[:, :2].mean(0)
u, s, vt = np.linalg.svd(ridge[:, :2] - c0)
ax = vt[0]; ang = math.degrees(math.atan2(ax[1], ax[0]))
proj = (ridge[:, :2] - c0) @ ax
log("ridge n", len(ridge), "center", c0.round(4), "axis deg", round(ang, 1), "length", round(proj.max() - proj.min(), 3), "ztop", round(pts[:, 2].max(), 4))
c1 = pts[:, :2].mean(0); log("all pts center", c1.round(4))
