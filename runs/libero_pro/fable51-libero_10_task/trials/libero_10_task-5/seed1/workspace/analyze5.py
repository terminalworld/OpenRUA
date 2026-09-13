import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; return np.array([-0.2+(v-cy)*z/fy, (u-cx)*z/fx, 3.0-z])
print('column u=268 (middle column), v -> world x, z')
for v in range(156,206):
    p=w(268,v); print(v, p[0].round(3), p[2].round(3))
print('row v=175 (back compartment), u -> world y, z')
for u in range(205,300):
    p=w(u,175); print(u, p[1].round(3), p[2].round(3))
