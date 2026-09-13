import numpy as np, cv2, sys
cam = sys.argv[1]
pw = np.load(f"{cam}_xyz.npy"); bgr = np.load(f"{cam}_bgr.npy")
z = pw[...,2]; zt = 0.4204
mask = (z > zt + 0.008) & (z < zt + 0.35) & (pw[...,0] > -0.32) & (pw[...,0] < 0.4) & (np.abs(pw[...,1]) < 0.5)
mask = mask.astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = pw[m]; col = bgr[m].mean(0)
    print(f"comp {i}: area {stats[i,4]}, px ({cent[i][0]:.0f},{cent[i][1]:.0f}), x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) BGR {col.astype(int)}")
cv2.imwrite(f"{cam}_mask.png", (lab>0).astype(np.uint8)*255)
