import numpy as np
d=np.load('agentview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
q=np.array([0.638,0.638,-0.305,-0.305]); q/=np.linalg.norm(q)
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([0.459,0.0,1.610])
def px2w(u,v):
    Z=d[v,u]; p=np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z]); return R@p+t
# scan the cup column at u=340 in the agentview image
for v in range(270,360,4):
    print(v, d[v,340].round(3), px2w(340,v).round(3))
print('caddy back wall top?')
for v in range(150,260,5):
    print(v, d[v,240].round(3), px2w(240,v).round(3))
