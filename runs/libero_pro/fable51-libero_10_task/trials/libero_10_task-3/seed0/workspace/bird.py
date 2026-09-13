import numpy as np, sys
d = np.load("/workspace/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
# camera at (-0.2,0,3.0), q=(0.7071,0.7071,0,0): R = rot 180deg about (1,1,0)/sqrt2
# R = [[0,1,0],[1,0,0],[0,0,-1]]
def px2w(u,v):
    z=d[v,u]; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([Y-0.2, X, 3.0-z])
H,W=d.shape
uu,vv=np.meshgrid(np.arange(W),np.arange(H))
X=(uu-cx)*d/fx; Y=(vv-cy)*d/fy
wx=Y-0.2; wy=X; wz=3.0-d
np.save("/workspace/bird_world.npy", np.stack([wx,wy,wz],-1))
print("table height estimate (median z in center):", np.median(wz[350:450,150:500]))
for name,(u,v) in {"bottle_cap":(335,258),"bowl":(308,300),"drawer_handle":(345,300),"drawer_front_center":(365,300),"cabinet_top":(420,300),"shelf":(235,270),"table_free":(300,400)}.items():
    print(name, (u,v), px2w(u,v).round(4), "depth",d[v,u].round(4))
