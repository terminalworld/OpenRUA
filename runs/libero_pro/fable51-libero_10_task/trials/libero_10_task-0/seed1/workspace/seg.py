import numpy as np, cv2
pw = np.load("birdview_xyz.npy"); bgr = np.load("birdview_bgr.npy")
z = pw[...,2]
# table height: mode of z in the table region
tab = z[(np.abs(pw[...,0])<0.6)&(np.abs(pw[...,1])<0.6)]
hist, edges = np.histogram(tab[np.isfinite(tab)], bins=200)
zt = edges[np.argmax(hist)]
print("table z ~", zt)
mask = (z > zt + 0.01) & (z < zt + 0.4) & (pw[...,0] > -0.3)  # exclude robot base region roughly
mask = mask.astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 15: continue
    m = lab == i
    pts = pw[m]
    col = bgr[m].mean(0)
    print(f"comp {i}: px area {stats[i,4]}, centroid px ({cent[i][0]:.0f},{cent[i][1]:.0f}), world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} mean {pts[:,0].mean():.3f},{pts[:,1].mean():.3f} color BGR {col.astype(int)}")
