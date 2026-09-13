import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; return np.array([-0.2+(v-cy)*z/fy, (u-cx)*z/fx, 3.0-z])
# caddy region approx px x 210-330, y 150-215 ; print height map (z above table) coarse
print('cols(u):', list(range(205,335,5)))
for v in range(150,220,3):
    row=''
    for u in range(205,335,3):
        h=3.0-d[v,u]-0.88
        row+= ' ' if h<0.01 else ('.' if h<0.03 else ('o' if h<0.09 else ('O' if h<0.14 else '#')))
    print(v, row)
# Print world coords of some key points
for (u,v) in [(210,155),(330,155),(210,215),(330,215)]:
    print((u,v), w(u,v).round(3))
