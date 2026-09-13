import numpy as np, yaml
d = np.load("snaps/birdview_depth.npy")
print(d.shape, np.nanmin(d), np.nanmax(d))
# birdview: cam at (-0.2,0,3.0), optical z down. q=(0.7071,0.7071,0,0): R = rotation... compute
fx=fy=579.4112549695428; cx=320; cy=240
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t = np.array([-0.2,0,3.0])
def px2w(u,v):
    Z = d[v,u]; p = np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z]); return R@p+t
# table height: center region
for (u,v) in [(320,240),(200,350),(500,350),(240,280),(330,310),(377,283),(377,240),(300,240),(320,200)]:
    print((u,v), d[v,u], px2w(u,v).round(4))
# height map
H = np.zeros_like(d)
for v in range(0,480,1):
    for u in range(0,640,1):
        H[v,u] = px2w(u,v)[2]
np.save("snaps/bird_H.npy", H)
tab = np.median(H[300:400, 200:450])
print("table z ~", tab)
mask = H > tab+0.01
import cv2
cv2.imwrite("snaps/bird_mask.png", (mask*255).astype(np.uint8))
