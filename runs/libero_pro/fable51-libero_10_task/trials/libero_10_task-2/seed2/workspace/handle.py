import numpy as np
from rob import quat_R
d = np.load("snaps/front_depth.npy")
fx=579.4112549695428; cx=320; cy=240
# frontview optical pose
t = np.array([1.0,0.0,1.48]); R = quat_R(0.5608,0.5608,-0.4306,-0.4306)
def px2w(u,v):
    Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+t
np.set_printoptions(linewidth=250, precision=3, suppress=True)
# scan columns across the handle; for each column print world points where x is near -0.06 (handle) 
for u in range(236, 306, 6):
    pts=[]
    for v in range(330, 380):
        w = px2w(u,v)
        pts.append((v, w.round(3)))
    # handle pixels: world x between -0.12 and 0.0 and z>0.95
    hp = [(v,w) for v,w in pts if -0.13 < w[0] < 0.0 and w[2] > 0.95]
    if hp:
        print(f"u={u}: rows {hp[0][0]}..{hp[-1][0]} top {hp[0][1]} bottom {hp[-1][1]}")
    else:
        print(f"u={u}: none; sample", pts[10][1], pts[30][1])
