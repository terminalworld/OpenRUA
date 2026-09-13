import numpy as np
np.set_printoptions(linewidth=300, threshold=100000)
fx=fy=312.77408948188935; cx=320; cy=240
def R_from_q(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
d=np.load("snaps/eih_depth.npy"); H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
P = np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1) @ R_from_q(0.7070,-0.7066,-0.0201,0.0201).T + np.array([-0.153,0,1.2673])
table=0.899
h = P[...,2]-table
# pot B region: world x[-0.12,-0.03], y[0.13,0.29]
m = (P[...,0]>-0.13)&(P[...,0]<-0.02)&(P[...,1]>0.12)&(P[...,1]<0.30)&(h>0.005)
print("potB px", m.sum())
for z0 in np.arange(0.0,0.17,0.01):
    s = P[m & (h>=z0)&(h<z0+0.01)]
    if len(s)<3: continue
    print(f"  h {z0*100:4.0f}-{z0*100+1:.0f}cm n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
# print height map of pot B region in eih pixels
vv,uu = np.where(m)
v0,v1,u0,u1 = vv.min(),vv.max(),uu.min(),uu.max()
print(v0,v1,u0,u1)
sub = np.round(h[v0:v1+1,u0:u1+1]*100).astype(int)
sub[~m[v0:v1+1,u0:u1+1]] = 0
for row in sub[::2]: print("".join(f"{v:3d}" for v in row[::2]))
