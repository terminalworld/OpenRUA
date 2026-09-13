import numpy as np, sys
P=np.load(sys.argv[1]); cx0,cy0=float(sys.argv[2]),float(sys.argv[3])
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
near=np.hypot(Q[:,0]-cx0,Q[:,1]-cy0)<0.075
sel=near&(Q[:,2]>0.975)&(Q[:,2]<1.01)
R=Q[sel]; print("rim pts", len(R), "z range %.4f %.4f"%(R[:,2].min(), R[:,2].max()))
def fit(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx*cx+cy*cy); return cx,cy,rad
pts=R[:,:2]
for it in range(6):
    cx,cy,rad=fit(pts)
    d=np.abs(np.hypot(pts[:,0]-cx,pts[:,1]-cy)-rad)
    pts=pts[d<max(np.percentile(d,85),0.002)]
print("rim circle center (%.4f %.4f) radius %.4f (n=%d)"%(cx,cy,rad,len(pts)))
# outer radius = max radial distance among fitted-in points
rr=np.hypot(R[:,0]-cx,R[:,1]-cy)
print("radial pct 50/90/99:", np.percentile(rr,[50,90,99]).round(4))
# handle direction: points within z band 0.93-0.97 and radial > rad+0.005
sel2=near&(Q[:,2]>0.90)&(Q[:,2]<0.975)
H=Q[sel2]; rh=np.hypot(H[:,0]-cx,H[:,1]-cy); Hh=H[rh>rad+0.003]
if len(Hh): 
    ang=np.degrees(np.arctan2(Hh[:,1].mean()-cy,Hh[:,0].mean()-cx))
    print("handle pts",len(Hh),"mean",Hh.mean(0).round(4),"angle deg",round(ang,1),"max radial %.4f"%rh.max())
res=0.002; x0,x1,y0,y1=cx-0.075,cx+0.075,cy-0.085,cy+0.075
W=int((x1-x0)/res); H2=int((y1-y0)/res); hm=np.full((H2,W),np.nan)
sel=(Q[:,0]>x0)&(Q[:,0]<x1)&(Q[:,1]>y0)&(Q[:,1]<y1)&(Q[:,2]<1.1)
S=Q[sel]; ix=np.clip(((S[:,0]-x0)/res).astype(int),0,W-1); iy=np.clip(((S[:,1]-y0)/res).astype(int),0,H2-1)
for a,b,z in zip(ix,iy,S[:,2]):
    if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
print("x cols from %.3f step 2mm"%x0)
for j in range(0,H2,2):
    print("y=%+.3f "%(y0+j*res)+"".join(" ." if np.isnan(v) else ("%2d"%min(99,int(round((v-0.88)*100)))) for v in hm[j]))
