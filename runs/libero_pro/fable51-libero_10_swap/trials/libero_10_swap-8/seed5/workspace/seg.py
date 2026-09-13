import numpy as np, cv2
d = np.load("birdview.npy")
fx=fy=579.4112549695428; cx=320; cy=240
t = np.array([-0.2,0,3.0])
H,W = d.shape
vv,uu = np.mgrid[0:H,0:W]
Z = d
X = (uu-cx)*Z/fx; Y=(vv-cy)*Z/fy
# R for q=(0.7071,0.7071,0,0): world = R@p + t
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
P = np.stack([X,Y,Z],-1) @ R.T + t
wz = P[...,2]
mask = (wz > 0.905) & (vv>150)&(vv<470)&(uu>150)&(uu<480)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA] < 30: continue
    m = lab==i
    pts = P[m]
    print(f"comp {i}: area={stats[i,cv2.CC_STAT_AREA]} px centroid=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
