import numpy as np, sys
d=np.load('snaps/bird_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
# birdview optical frame: quat (0.7071,0.7071,0,0) => R = rot about (1,1,0)/sqrt2 by 180deg
q=np.array([0.7071,0.7071,0.0,0.0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([-0.2,0,3.0])
def w(u,v):
    Z=d[v,u]; p=np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z]); return R@p+T
print('R=',R)
for name,(u,v) in {'table':(320,400),'book':(325,272),'caddy_back_right':(280,172),'caddy_front_right':(280,196),'caddy_left':(237,185),'caddy_top_wall_left':(217,185),'mug':(360,240)}.items():
    print(name,(u,v),d[v,u],w(u,v).round(4))
# table height stats
print('table z hist near center', np.round(w(300,350),3), np.round(w(500,300),3))
