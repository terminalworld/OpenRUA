import numpy as np, cv2
d = np.load("snaps/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
camz = 3.0; camx=-0.2; camy=0.0
# birdview optical: q=(0.7071,0.7071,0,0): rotation about axis (1,1,0)/sqrt2 by 180deg.
# R = [[0,1,0],[1,0,0],[0,0,-1]] -> world = camx + v_cam_y... let's compute properly
def R_from_q(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
R = R_from_q(0.7071,0.7071,0,0)
print(R.round(3))
H,W = d.shape
vs,us = np.mgrid[0:H,0:W]
Z = d
X = (us-cx)*Z/fx; Y=(vs-cy)*Z/fy
P = np.stack([X,Y,Z],-1) @ R.T + np.array([camx,camy,camz])
wz = P[...,2]
print("z stats", np.nanmin(wz), np.nanmax(wz))
# table height: mode
hist,edges = np.histogram(wz[np.isfinite(wz)], bins=200)
i = hist.argmax(); table = (edges[i]+edges[i+1])/2
print("table z ~", table)
mask = (wz > table+0.01) & (wz < table+0.5)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 30: continue
    m = lab==i
    pts = P[m]
    print(f"comp {i}: px area={stats[i,4]} bbox={stats[i,:4]} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
np.save("snaps/bird_wz.npy", wz)
