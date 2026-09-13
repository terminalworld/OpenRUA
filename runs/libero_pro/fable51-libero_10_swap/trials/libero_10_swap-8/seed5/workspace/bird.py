import numpy as np, sys
d = np.load("birdview.npy")
fx=fy=579.4112549695428; cx=320; cy=240
# world->cam: t=(-0.2,0,3.0), q=(0.7071,0.7071,0,0)
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t = np.array([-0.2,0,3.0])
def w(u,v):
    Z = d[v,u]; p = np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z])
    return R@p + t
for u,v in [(262,238),(385,280),(330,348),(330,302),(200,400),(450,200),(160,160),(478,430)]:
    print((u,v), d[v,u], np.round(w(u,v),4))
print("depth min/max", np.nanmin(d), np.nanmax(d))
