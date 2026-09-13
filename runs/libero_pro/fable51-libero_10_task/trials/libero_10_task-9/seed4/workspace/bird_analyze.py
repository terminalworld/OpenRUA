import numpy as np, cv2
d = np.load("snaps/birdview_depth.npy")
img = cv2.imread("snaps/birdview.png")
fx=fy=579.4112549695428; cx=320; cy=240
# camera at (-0.2,0,3.0), R: cam x->world y, cam y->world x, cam z->world -z
def w(u,v):
    z = d[v,u]
    X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
print("depth range", np.nanmin(d), np.nanmax(d))
# height map
H = 3.0 - d
# table height: mode of a big region
print("table z sample", H[400,150], H[450,600], H[200,200])
# print heights along row 275 (white mug) and col
for name,(u,v) in {"white_mug":(232,275),"yellow_mug":(318,305),"mw_body_center":(425,295),"door_line":(365,240),"table":(200,400)}.items():
    print(name, (u,v), w(u,v), "h=",H[v,u])
# find objects above table: threshold
tab = np.median(H[350:470, 100:540])
print("table median z", tab)
mask = (H > tab+0.02)
mask[:200,:]=False  # ignore robot area roughly
ys,xs = np.where(mask)
print("above-table pixel bbox", xs.min(), xs.max(), ys.min(), ys.max())
# connected components
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    x,y,ww,hh,a = stats[i]
    if a<30: continue
    sub = H[y:y+hh, x:x+ww]; m = lab[y:y+hh, x:x+ww]==i
    print(f"comp {i}: bbox u[{x},{x+ww}] v[{y},{y+hh}] area={a} maxh={sub[m].max():.3f} centroid={cents[i]}")
    # world extents
    c = w(int(cents[i][0]), int(cents[i][1]))
    print("   world at centroid", c, " corners:", w(x,y)[:2], w(x+ww-1,y+hh-1)[:2])
