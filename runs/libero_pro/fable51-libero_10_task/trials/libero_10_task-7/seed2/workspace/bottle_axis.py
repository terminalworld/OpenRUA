import sys, numpy as np, rclpy, cv2
sys.argv = ["x", "birdview", "0.012", "0.425"]
# reuse locate.py internals by exec'ing up to the mask
src = open("locate.py").read().split("n, lab, stats")[0]
exec(src)
# select region of interest: ketchup approx
roi = mask.astype(bool) & (P[...,0] > -0.25) & (P[...,0] < 0.0) & (P[...,1] > -0.26) & (P[...,1] < -0.08) & (P[...,2] < 0.50)
pts = P[roi]
print("n pts", len(pts), "ztop", pts[:,2].max())
xy = pts[:, :2]
c = xy.mean(0)
u, s, vt = np.linalg.svd(xy - c, full_matrices=False)
axis = vt[0]; ang = np.degrees(np.arctan2(axis[1], axis[0]))
proj = (xy - c) @ axis
print(f"centroid=({c[0]:.4f},{c[1]:.4f}) axis angle={ang:.1f} deg length={proj.max()-proj.min():.3f} width={((xy-c)@vt[1]).ptp():.3f}")
# thickness profile along axis
for lo in np.arange(proj.min(), proj.max(), 0.01):
    sel = (proj >= lo) & (proj < lo + 0.01)
    if sel.sum():
        print(f"  s={lo:+.3f} n={sel.sum():3d} zmax={pts[sel,2].max():.3f} width={((xy[sel]-c)@vt[1]).ptp():.3f}")
thick = pts[:,2] > 0.462
cb = xy[thick].mean(0)
print(f"body centroid (z>0.462): ({cb[0]:.4f},{cb[1]:.4f}), n={thick.sum()}")
