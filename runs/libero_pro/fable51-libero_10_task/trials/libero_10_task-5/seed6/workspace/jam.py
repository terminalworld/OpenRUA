import numpy as np
# slot mouth: back wall inner face x=-0.456 (top z=1.056); divider back face x=-0.402 (top z=0.988); floor 0.90
rb,rt,H=0.033,0.050,0.105
def r(s): return rb+(rt-rb)*s/H
S=np.linspace(0,H,106)
best=None
for th in np.radians(np.arange(0,71,5)):
    u=np.array([np.sin(th),np.cos(th)]); v=np.array([np.cos(th),-np.sin(th)])
    # find max depth: minimize zb over xb s.t. constraints
    res=[]
    for xb in np.arange(-0.50,-0.36,0.001):
        # constraint: for each s, the -x extreme point (xm,zm) and +x extreme (xp,zp) and base disc edge points
        lo,hi=0.85,1.20
        # binary search zb: feasible if constraints hold
        def ok(zb):
            for s in S:
                c=np.array([xb,zb])+s*u
                pm=c-r(s)*v; pp=c+r(s)*v
                if pm[1]<1.056 and pm[0]<-0.456: return False
                if pp[1]<0.988 and pp[0]>-0.402: return False
                if pm[1]<0.90 or pp[1]<0.90: return False
                # also the -x point must not be inside the back wall below top; and the +x point inside divider (x>-0.402 and z<0.988) handled
                # also points over the divider top: if x in [-0.402,-0.386] z must be >0.988
                if -0.402<=pp[0]<=-0.386 and pp[1]<0.988: return False
            return True
        if not ok(hi): continue
        for _ in range(25):
            mid=(lo+hi)/2
            if ok(mid): hi=mid
            else: lo=mid
        res.append((hi,xb))
    if res:
        zb,xb=min(res)
        com=np.array([xb,zb])+0.05*u
        print("tilt %2d deg: base center z=%.3f x=%.3f  COM x=%.3f z=%.3f"%(np.degrees(th),zb,xb,com[0],com[1]))
