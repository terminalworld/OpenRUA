import numpy as np, cv2, sys
sys.path.insert(0, "/workspace")
from scene import grab, cloud
cam = sys.argv[1]
color, depth, K, T = grab(cam)
P = cloud(depth, K, T)
Z = P[...,2]
cv2.imwrite(f"snaps/{cam}.png", color)
np.save(f"snaps/{cam}_P.npy", P)
# table: points in workspace region with z within 0.40..0.45
ws = (P[...,0] > -0.4) & (P[...,0] < 0.4) & (np.abs(P[...,1]) < 0.5) & np.isfinite(Z)
tz = Z[ws & (Z > 0.38) & (Z < 0.46)]
hist, edges = np.histogram(tz, bins=100)
table_z = edges[np.argmax(hist)]
print("table_z ~", table_z, "n", tz.size)
mask = ws & (Z > table_z + 0.012) & (Z < table_z + 0.30)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
for i in range(1, n):
    x,y,w,h,a = stats[i]
    if a < 30: continue
    m = lab == i
    pts = P[m]
    print(f"comp {i}: px bbox x={x}..{x+w} y={y}..{y+h} area={a} centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}")
    print("   mean color BGR", color[m].mean(0).round(0))
