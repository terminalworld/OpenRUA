import numpy as np
P=np.load("snaps/birdview_P.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
def ext(mask,name):
    print(name, "n=",mask.sum(), "x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]"%(x[mask].min(),x[mask].max(),y[mask].min(),y[mask].max(),z[mask].min(),z[mask].max()))
ext((z>1.0)&(z<1.2)&(y<-0.05)&(y>-0.36)&(x<0.2)&(x>-0.4),"microwave body top")
ext((z>1.0)&(z<1.2)&(y<-0.05)&(y>-0.36)&(x<0.2)&(x>-0.4)&(z>1.09),"microwave top face z>1.09")
ext((z>0.95)&(z<1.0)&(abs(y)<0.1)&(abs(x)<0.1),"yellow mug rim")
ext((z>0.95)&(z<1.0)&(abs(y-0.35)<0.1)&(abs(x)<0.1),"grey mug rim")
# mug body pixels at z between 0.92 and 1.0
for name,yc in [("yellow",0.0),("grey",0.35)]:
    m=(z>0.91)&(z<1.0)&(abs(y-yc)<0.1)&(abs(x)<0.1)
    print(name,"centroid",x[m].mean(),y[m].mean(),"zmax",z[m].max())
# profile microwave along x at y=-0.2
m=(z>1.0)&(y<-0.05)&(y>-0.36)&(x<0.2)&(x>-0.4)
for xx in np.arange(-0.32,0.12,0.02):
    mm=m&(abs(x-xx)<0.01)
    if mm.sum(): print("x=%.2f y[%.3f %.3f] z[%.3f %.3f]"%(xx,y[mm].min(),y[mm].max(),z[mm].min(),z[mm].max()))
