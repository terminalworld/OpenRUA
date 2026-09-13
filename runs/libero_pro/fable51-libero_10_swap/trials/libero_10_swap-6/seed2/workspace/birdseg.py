import numpy as np, cv2, sys
sys.path.insert(0, "/workspace")
from scene import grab, cloud
color, depth, K, T = grab("birdview")
P = cloud(depth, K, T)
Z = P[...,2]
cv2.imwrite("snaps/birdview.png", color)
np.save("snaps/birdview_P.npy", P)
print("K", K.tolist())
# table height: mode of Z in central region
zc = Z[200:350, 150:500]
hist, edges = np.histogram(zc[np.isfinite(zc)], bins=200)
table_z = edges[np.argmax(hist)]
print("table_z ~", table_z)
mask = (Z > table_z + 0.01) & (Z < table_z + 0.35) & np.isfinite(Z)
# exclude robot: robot is anything with z > table+0.35 or connected... just cluster
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
for i in range(1, n):
    x,y,w,h,a = stats[i]
    if a < 15: continue
    m = lab == i
    pts = P[m]
    print(f"comp {i}: px bbox x={x}..{x+w} y={y}..{y+h} area={a} centroid_px={cents[i]} "
          f"world xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) xrange=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yrange=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}")
    bgr = color[m].mean(0)
    print("   mean color BGR", bgr)
