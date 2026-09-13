import numpy as np, cv2
wz = np.load("snaps/bird_wz.npy")
d = np.load("snaps/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
H,W = d.shape
vs,us = np.mgrid[0:H,0:W]
wx = -0.2 + (vs-cy)*d/fy
wy = 0.0 + (us-cx)*d/fx
region = wz[160:480, 160:480]
hist,edges = np.histogram(region[np.isfinite(region)], bins=400, range=(0,1.7))
i = hist.argmax(); table = (edges[i]+edges[i+1])/2
print("table z ~", table)
# stats of heights above table
mask = ((wz > table+0.008) & (wz < table+0.5)).astype(np.uint8)
mask[:150,:]=0
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 20: continue
    m = lab==i
    print(f"comp {i}: area={stats[i,4]} bbox(u,v,w,h)={stats[i,:4]} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f})")
# table extents
tm = np.abs(wz - table) < 0.005
print("table x", wx[tm].min(), wx[tm].max(), "y", wy[tm].min(), wy[tm].max())
