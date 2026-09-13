import numpy as np, cv2
d = np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([Y-0.2, X, 3.0-z])
# table height: sample an empty table area
print("table", w(200,200), w(450,300))
# find pixels above table: height > table+0.005 within table region
H = 3.0 - d
tab = np.median(H[180:330, 200:460])
print("table z median", tab)
mask = (H > tab+0.01) & (H < tab+0.4)
mask[:, :] &= np.arange(640)[None,:] > 200
mask[:, :] &= np.arange(480)[:,None] > 200
mask[:, :] &= np.arange(480)[:,None] < 340
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4] < 10: continue
    m = lab==i
    hmax = H[m].max()
    u,v = cent[i]
    print(f"comp {i}: px=({u:.0f},{v:.0f}) area={stats[i,4]} bbox={stats[i,:4]} top_z={hmax:.3f} world_xy={w(int(u),int(v))[:2]}")
