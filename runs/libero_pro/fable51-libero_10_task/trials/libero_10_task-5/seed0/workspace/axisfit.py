import numpy as np
d=np.load("cup_pts.npz"); W=d["W"]; Y=d["Y"]
# fit axis direction in xy from ridge: for each x-slice, take the top-z points' mean y
a=np.array([-0.638,0.77]); a/=np.linalg.norm(a); n=np.array([-a[1],a[0]])
s=W[:,:2]@a; t=W[:,:2]@n
print("s range",s.min().round(3),s.max().round(3))
for lo in np.arange(s.min(),s.max(),0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<5: continue
    print(f"s={lo:.3f} n={m.sum():4d} t[{t[m].min():.3f},{t[m].max():.3f}] tmid={(t[m].min()+t[m].max())/2:.3f} zmax={W[m,2].max():.3f}")
# refine direction: ridge = top 20% z per slice
mids=[]; ss=[]
for lo in np.arange(s.min()+0.01,s.max()-0.01,0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<20: continue
    P=W[m]; k=P[:,2]>np.percentile(P[:,2],85)
    mids.append(P[k,:2].mean(0)); ss.append(lo)
mids=np.array(mids); c=mids.mean(0); u,sv,vt=np.linalg.svd(mids-c)
print("ridge dir",vt[0].round(3),"centre",c.round(3))
a=vt[0]; 
if a[0]>0: a=-a
n=np.array([-a[1],a[0]]); s=W[:,:2]@a; t=W[:,:2]@n
for lo in np.arange(s.min(),s.max(),0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<5: continue
    print(f"s={lo:.3f} n={m.sum():4d} tmid={(t[m].min()+t[m].max())/2:.3f} w={t[m].max()-t[m].min():.3f} zmax={W[m,2].max():.3f}")
sb=s.min()+0.005; sr=np.percentile(s,99)
tm=np.median(t[(s>s.min()+0.02)&(s<s.max()-0.02)])
print("axis dir",a.round(3),"bottom centre",(a*sb+n*tm).round(3),"rim centre",(a*sr+n*tm).round(3), "tm",tm.round(3))
print("ridge points (world xy, zmax):")
for lo in np.arange(s.min(),s.max(),0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<20: continue
    P=W[m]; k=P[:,2]>np.percentile(P[:,2],85)
    print(f"  s={lo:.3f} ridge={P[k,:2].mean(0).round(3)} zmax={P[:,2].max():.3f} n={m.sum()}")
