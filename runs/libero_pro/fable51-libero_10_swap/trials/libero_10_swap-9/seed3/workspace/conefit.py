import numpy as np, sys
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
box=(P[:,0]>-0.34)&(P[:,0]<-0.04)&(P[:,1]<-0.35)&(P[:,1]>-0.60)&(P[:,2]>0.94)&(P[:,2]<1.05)
Q=P[box]
R0,R1,L=0.035,0.051,0.106   # base radius, rim radius, length
def score(cx,cy,yaw):
    a=np.array([np.cos(yaw),np.sin(yaw),0]); perp=np.array([-a[1],a[0],0]); c=np.array([cx,cy,0.95])
    d=Q-c; t=d@a; w=d@perp; h=d[:,2]
    r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
    res=np.abs(r-rt); res[np.abs(t)>L/2+0.005]=0.05   # outside ends
    res=np.minimum(res,0.02)  # robust cap (handle points)
    return res.mean(), (np.abs(res)<0.006).sum()
c0=Q.mean(0)
best=None
for yaw in np.radians(np.arange(-180,180,5)):
    for dx in np.arange(-0.04,0.041,0.01):
        for dy in np.arange(-0.04,0.041,0.01):
            s,n=score(c0[0]+dx,c0[1]+dy,yaw)
            if best is None or s<best[0]: best=(s,n,c0[0]+dx,c0[1]+dy,yaw)
s,n,cx,cy,yaw=best
# refine
for it in range(3):
    step=0.005/(it+1); ystep=np.radians(2/(it+1))
    for yy in yaw+np.arange(-3,4)*ystep:
        for dx in np.arange(-2,3)*step:
            for dy in np.arange(-2,3)*step:
                s2,n2=score(cx+dx,cy+dy,yy)
                if s2<s: s,n,bx,by,byaw=s2,n2,cx+dx,cy+dy,yy
    cx,cy,yaw=bx,by,byaw
a=np.array([np.cos(yaw),np.sin(yaw),0]); perp=np.array([-a[1],a[0],0])
print(f"cone fit: c=({cx:.3f},{cy:.3f}) yaw={np.degrees(yaw):.1f} axis(base->rim)={np.round(a[:2],3)} perp={np.round(perp[:2],3)} score={s:.4f} inliers={n}/{len(Q)}")
print(f"rim centre=({cx+a[0]*L/2:.3f},{cy+a[1]*L/2:.3f}) base centre=({cx-a[0]*L/2:.3f},{cy-a[1]*L/2:.3f})")
# handle: points far from cone surface
c=np.array([cx,cy,0.95]); d=Q-c; t=d@a; w=d@perp; h=d[:,2]; r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
out=Q[(r-rt>0.012)&(np.abs(t)<L/2+0.01)]
if len(out):
    dd=out-c; print(f"handle pts {len(out)}: t {(dd@a).min():+.3f}..{(dd@a).max():+.3f} perp {(dd@perp).min():+.3f}..{(dd@perp).max():+.3f} z {out[:,2].min():.3f}..{out[:,2].max():.3f}")
    elev=np.degrees(np.arctan2(out[:,2]-0.95,dd@perp)); print("handle elevation deg (median)",np.median(elev), "radius median", np.median(np.hypot(dd@perp,out[:,2]-0.95)))
np.save('mugpose.npy',np.array([cx,cy,0.95,*a,cx+a[0]*L/2,cy+a[1]*L/2,0.95,cx-a[0]*L/2,cy-a[1]*L/2,0.95,0.0]))
