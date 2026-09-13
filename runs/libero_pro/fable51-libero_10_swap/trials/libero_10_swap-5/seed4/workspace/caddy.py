import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
c = cv2.imread("birdview.png")
u0,v0,w,h = 205,150,115,70
sub = z[v0:v0+h, u0:u0+w]
# print height map coarse (every 3 px)
for v in range(0,h,3):
    print("".join(" " if sub[v,u]<0.89 else ("." if sub[v,u]<0.93 else ("o" if sub[v,u]<1.0 else "#")) for u in range(0,w,1)))
crop = cv2.resize(c[v0:v0+h, u0:u0+w], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)
cv2.imwrite("caddy_crop.png", crop)
# world coords of corners
for (u,v) in [(217,160),(309,160),(217,207),(309,207)]:
    print((u,v), P[v,u])
