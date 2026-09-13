import numpy as np, sys
from ctl import Robot
def fit_circle(xy):
    A=np.c_[2*xy,np.ones(len(xy))]; b=(xy**2).sum(1); cx,cy,c0=np.linalg.lstsq(A,b,rcond=None)[0]; return cx,cy,np.sqrt(max(c0+cx**2+cy**2,0))
def analyze(r, cams, box, tag):
    allp=[]
    for cam in cams:
        c,d,P,T=r.snap(cam,f"/workspace/{tag}_{cam}.png")
        pts=P.reshape(-1,3); col=c.reshape(-1,3).astype(int); ok=np.isfinite(pts).all(1); pts=pts[ok]; col=col[ok]
        (x0,x1),(y0,y1),(z0,z1)=box
        m=(pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>z0)&(pts[:,2]<z1)
        b,g,rr=col[m].T; gray=(np.abs(rr-g)<20)&(np.abs(g-b)<20)&(rr<170); dark=(rr+g+b<150)
        q=pts[m][~gray&~dark]; print(cam,"pts",len(q)); allp.append(q)
    q=np.vstack(allp)
    if len(q)<20: print("too few"); return
    print("z %.3f..%.3f  x %.3f..%.3f  y %.3f..%.3f"%(q[:,2].min(),q[:,2].max(),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
    zs=np.arange(q[:,2].min(),q[:,2].max(),0.01)
    for z in zs:
        s=q[(q[:,2]>=z)&(q[:,2]<z+0.01)]
        if len(s)>15:
            cx,cy,rad=fit_circle(s[:,:2]); print("  z %.3f n %4d  cxy (%.3f,%.3f) r %.3f  xrange %.3f..%.3f yrange %.3f..%.3f"%(z,len(s),cx,cy,rad,s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
    return q
if __name__=="__main__":
    r=Robot("mugpose"); box=eval(sys.argv[1]); tag=sys.argv[2]
    q=analyze(r,["agentview","sideview","birdview","frontview"],box,tag); np.save(f"/workspace/{tag}_pts.npy",q)
