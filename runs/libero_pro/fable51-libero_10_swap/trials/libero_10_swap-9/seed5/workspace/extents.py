import numpy as np
P=np.load("snaps/birdview_P.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
def ext(mask,name):
    print(name, "n=",mask.sum(), "x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]"%(x[mask].min(),x[mask].max(),y[mask].min(),y[mask].max(),z[mask].min(),z[mask].max()))
# microwave body: tall stuff at y<-0.05
ext((z>1.05)&(y<-0.05)&(x<0.3),"microwave top")
ext((z>0.93)&(z<1.05)&(y<-0.35)&(x<0.3),"door?")
ext((z>0.93)&(y<-0.35)&(x<0.3),"door all")
# yellow mug region
ext((z>0.92)&(abs(y)<0.1)&(abs(x)<0.1),"yellow mug")
ext((z>0.92)&(abs(y-0.35)<0.1)&(abs(x)<0.1),"grey mug")
# table
tb=(abs(z-0.9)<0.005)
print("table x[%.3f %.3f] y[%.3f %.3f]"%(x[tb].min(),x[tb].max(),y[tb].min(),y[tb].max()))
# door height profile along y at x~-0.3
m=(z>0.93)&(y<-0.3)&(x<-0.2)&(x>-0.4)
for yy in np.arange(-0.5,-0.28,0.02):
    mm=m&(abs(y-yy)<0.01)
    if mm.sum(): print("y=%.2f x[%.3f %.3f] zmax=%.3f"%(yy,x[mm].min(),x[mm].max(),z[mm].max()))
