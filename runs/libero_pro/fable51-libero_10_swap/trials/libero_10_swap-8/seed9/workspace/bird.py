import numpy as np, cv2
d = np.load("birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
# birdview optical: t=(-0.2,0,3.0) q=(0.7071,0.7071,0,0) -> R = rot about (1,1,0)/sqrt2 by 180deg
# R = [[0,1,0],[1,0,0],[0,0,-1]]
T = np.array([-0.2,0,3.0])
def w(u,v):
    z = d[v,u]
    pc = np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    return np.array([pc[1], pc[0], -pc[2]]) + T
print("table at (320,420):", w(320,420))
print("floor at (100,50):", w(100,50))
# heights map: world z
img = cv2.imread("birdview.png")
Z = np.zeros_like(d)
for v in range(0,480):
    for u in range(0,640):
        Z[v,u] = w(u,v)[2]
np.save("birdZ.npy", Z)
tab = np.median(Z[400:470, 200:440])
print("table z median", tab)
mask = (Z > tab + 0.02).astype(np.uint8)
mask[:200,:] = 0  # cut the robot area roughly
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,ww,hh,a = stats[i]
    if a < 30: continue
    ys,xs = np.where(lab==i)
    zmax = Z[ys,xs].max()
    c = cent[i]
    print(f"blob {i}: px center ({c[0]:.0f},{c[1]:.0f}) bbox {x},{y},{ww},{hh} area {a} zmax {zmax:.3f} world_xy {w(int(c[0]),int(c[1]))[:2]}")
