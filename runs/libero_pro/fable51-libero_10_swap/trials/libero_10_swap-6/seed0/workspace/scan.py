import sys, numpy as np, cv2
sys.path.insert(0, "/workspace")
from wlib import start
w = start()
cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
pw, color = w.cloud(cam)
np.save(f"{cam}_cloud.npy", pw)
z = pw[..., 2]
ok = np.isfinite(z)
print("z range", np.nanmin(z[ok]), np.nanmax(z[ok]))
# table height: mode of z in the central region
hist, edges = np.histogram(z[ok], bins=400, range=(0.3, 1.0))
table_z = edges[np.argmax(hist)]
print("table z ~", table_z)
mask = ok & (z > table_z + 0.008) & (z < table_z + 0.3)
mask8 = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask8)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    P = pw[m]
    c = color[m].mean(0)
    print(f"blob {i}: area={stats[i,4]} px centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] "
          f"z[{P[:,2].min():.3f},{P[:,2].max():.3f}] mean=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) color(bgr)={c.astype(int)}")
w.destroy_node()
