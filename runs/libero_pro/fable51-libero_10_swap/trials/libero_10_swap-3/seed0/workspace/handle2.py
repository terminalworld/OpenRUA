import numpy as np
for cam in ["agentview","frontview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    m = ok&(y>-0.205)&(y<-0.19)&(z>0.98)&(z<1.06)&(x>-0.16)&(x<-0.04)
    print(cam, "middle handle bar: n", m.sum(), "z pct 1/5/50/95/99", np.percentile(z[m],[1,5,50,95,99]).round(4), "x", x[m].min().round(3), x[m].max().round(3), "y", y[m].min().round(3), y[m].max().round(3))
    # bottom drawer front panel top and inner face
    m = ok&(y>-0.11)&(y<-0.06)&(z>0.93)&(z<1.0)&(x>-0.19)&(x<-0.02)
    print(cam, "front panel: n", m.sum(), "z pct 50/95/99", np.percentile(z[m],[50,95,99]).round(4), "y pct 1/5/50/95/99", np.percentile(y[m],[1,5,50,95,99]).round(4))
    # bowl now: rim ring near TCP (-0.1595,-0.0504,1.08): bowl center approx (-0.107,-0.05)
    r = np.hypot(x+0.107, y+0.050)
    m = ok&(r<0.075)&(z>1.06)&(z<1.12)
    print(cam, "bowl rim: n", m.sum(), "z pct 50/95/99", np.percentile(z[m],[50,95,99]).round(4), "x", x[m].min().round(4), x[m].max().round(4), "y", y[m].min().round(4), y[m].max().round(4))
