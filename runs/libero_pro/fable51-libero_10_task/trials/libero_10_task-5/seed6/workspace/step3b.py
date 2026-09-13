from ctl import *
r=Robot("step3b")
r.spin(20)
for f in ["panda_hand","panda_leftfinger","panda_rightfinger","robot0_eye_in_hand_optical_frame"]:
    t=r.tfbuf.lookup_transform("world",f,Time()).transform
    print(f, np.round([t.translation.x,t.translation.y,t.translation.z],4), np.round([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w],4))
P=np.load("/workspace/eih2_P.npy")
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
# rim points: z between 0.975 and 1.0 near cup
sel=(Q[:,2]>0.97)&(Q[:,2]<1.01)&(Q[:,0]>-0.25)&(Q[:,0]<0.0)&(Q[:,1]>-0.1)&(Q[:,1]<0.15)
R=Q[sel]; print("rim pts", len(R), "z range", R[:,2].min(), R[:,2].max())
# fit circle to rim points (exclude handle: use algebraic fit with robust iteration)
def fit(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx*cx+cy*cy); return cx,cy,rad
pts=R[:,:2]
for it in range(5):
    cx,cy,rad=fit(pts)
    d=np.abs(np.hypot(pts[:,0]-cx,pts[:,1]-cy)-rad)
    pts=pts[d<np.percentile(d,80)]
print("rim circle center (%.4f %.4f) radius %.4f"%(cx,cy,rad))
# top-down height map around cup at 2mm
res=0.002; x0,x1,y0,y1=cx-0.08,cx+0.08,cy-0.09,cy+0.08
W=int((x1-x0)/res); H=int((y1-y0)/res); hm=np.full((H,W),np.nan)
sel=(Q[:,0]>x0)&(Q[:,0]<x1)&(Q[:,1]>y0)&(Q[:,1]<y1)&(Q[:,2]<1.1)
S=Q[sel]; ix=((S[:,0]-x0)/res).astype(int); iy=((S[:,1]-y0)/res).astype(int)
for a,b,z in zip(ix,iy,S[:,2]):
    if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
print("x cols from %.3f step 2mm"%x0)
for j in range(0,H,2):
    print("y=%+.3f "%(y0+j*res)+"".join(" ." if np.isnan(v) else ("%2d"%min(99,int(round((v-0.88)*100)))) for v in hm[j]))
