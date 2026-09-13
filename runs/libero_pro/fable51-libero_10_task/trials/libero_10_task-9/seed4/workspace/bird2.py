import numpy as np
d = np.load("snaps/birdview_depth.npy"); H = 3.0-d
fx=579.4112549695428
def w(u,v):
    z=d[v,u]; return np.array([-0.2+(v-240)*z/fx, (u-320)*z/fx, 3.0-z])
np.set_printoptions(precision=3, suppress=True, linewidth=200)
# microwave: row profile at v=300 across u 370..470
print("row v=300 heights u=370..470:"); print(np.round(H[300,370:470],3))
print("col u=430 heights v=220..360:"); print(np.round(H[220:360,430],3))
# door line: heights along v=236..246 at u=360
print("door col u=360 v=230..250:", np.round(H[230:250,360],3))
print("door row v=240 u=330..400:", np.round(H[240,330:400],3))
print("door row v=239 u=330..400:", np.round(H[239,330:400],3))
print("door row v=241 u=330..400:", np.round(H[241,330:400],3))
# white mug detail: heights along row 274 u 215..265
print("white mug row v=274:", np.round(H[274,215:265],3))
print("white mug col u=235:", np.round(H[255:295,235],3))
