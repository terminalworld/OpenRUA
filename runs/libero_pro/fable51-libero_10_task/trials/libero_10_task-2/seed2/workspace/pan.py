import numpy as np
H = np.load("snaps/bird_H.npy")
np.set_printoptions(linewidth=300, precision=3, suppress=True)
fx=579.4112549695428; cx=320; cy=240
def px2w(u,v):
    Z = 3.0 - H[v,u]; return np.array([-0.2 + (v-cy)*Z/fx, (u-cx)*Z/fx, H[v,u]])
print("pan rows 250..312 step 3, cols 212..320 step 4")
print(H[250:313:3, 212:321:4].round(3))
# pan body: circular region cols 214..280, find rim center via pixels with H>0.95 within cols<285
ys,xs = np.where((H[245:320, 205:290] > 0.95))
ys+=245; xs+=205
print("rim px center", xs.mean(), ys.mean(), "px extents", xs.min(), xs.max(), ys.min(), ys.max())
print("rim world center", px2w(int(xs.mean()), int(ys.mean())).round(3))
# handle: cols 285..320
ys,xs = np.where((H[245:320, 283:325] > 0.91))
ys+=245; xs+=283
print("handle px", xs.min(), xs.max(), ys.min(), ys.max(), "heights", H[ys,xs].min(), H[ys,xs].max())
for u in range(285,322,4):
    col = H[260:300,u]; v = 260+np.argmax(col)
    print(u, v, px2w(u,v).round(3))
