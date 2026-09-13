from refine import *
import cv2
r = Robot()
cen, pts = refine(r, 0.03, 0.63, 0.56, 0.68, rad=0.16, cam="birdview")
pts = np.asarray(pts)
rim = pts[pts[:, 2] > 0.555]
log("rim n", len(rim), "z range", rim[:, 2].min().round(3), rim[:, 2].max().round(3), "centroid", rim[:, :2].mean(0).round(4))
(cx, cy), (w, h), th = cv2.minAreaRect(rim[:, :2].astype(np.float32))
log("bbox center", round(cx, 4), round(cy, 4), "size", round(w, 3), round(h, 3), "angle", round(th, 1))
cen2, pts2 = refine(r, cx, cy, 0.48, 0.56, rad=0.09, cam="birdview")
pts2 = np.asarray(pts2); log("soup-top pts n", len(pts2), "center", pts2[:, :2].mean(0).round(4) if len(pts2) else None, "ztop", pts2[:, 2].max().round(3) if len(pts2) else None)
