import numpy as np, cv2
Z = np.load("birdZ.npy"); d=np.load("birdview_depth.npy")
fx=579.4112549695428; cx=320; cy=240; T=np.array([-0.2,0,3.0])
def w(u,v):
    z=d[v,u]; pc=np.array([(u-cx)*z/fx,(v-cy)*z/fy if False else (v-cy)*z/fx,z]); return np.array([pc[1],pc[0],-pc[2]])+T
print("Z hist table region:", np.percentile(Z[300:450,180:460],[5,25,50,75,95]))
mask = ((Z > 0.92) & (Z < 1.3)).astype(np.uint8)
mask[:150,:]=0
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,ww,hh,a = stats[i]
    if a < 30: continue
    ys,xs = np.where(lab==i)
    c = cent[i]
    print(f"blob {i}: px ({c[0]:.0f},{c[1]:.0f}) bbox x{x} y{y} w{ww} h{hh} area {a} zmax {Z[ys,xs].max():.3f} zmin {Z[ys,xs].min():.3f} world {w(int(c[0]),int(c[1]))}")
