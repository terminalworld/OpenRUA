import numpy as np, cv2
H = np.load("snaps/bird_H.npy")
fx=579.4112549695428; cx=320; cy=240
def px2w(u,v):
    # birdview straight down: from previous mapping
    Z = 3.0 - H[v,u]
    return np.array([-0.2 + (v-cy)*Z/fx, (u-cx)*Z/fx*-1, H[v,u]])
# verify mapping sign using earlier sample: (200,350)->(0.1987,-0.4349)
print(px2w(200,350))
tab=0.9001
mask = ((H > tab+0.008) & (H < 1.2)).astype(np.uint8)
mask[:, :] = mask
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    a = stats[i, cv2.CC_STAT_AREA]
    if a < 30: continue
    ys,xs = np.where(lab==i)
    hs = H[ys,xs]
    u,v = cents[i]
    w = px2w(int(u),int(v))
    # world extents
    W = np.array([px2w(x,y) for x,y in zip(xs[::5],ys[::5])])
    print(f"blob{i} area={a} px_c=({u:.0f},{v:.0f}) bbox={stats[i][:4]} zmax={hs.max():.3f} zmed={np.median(hs):.3f} x[{W[:,0].min():.3f},{W[:,0].max():.3f}] y[{W[:,1].min():.3f},{W[:,1].max():.3f}]")
