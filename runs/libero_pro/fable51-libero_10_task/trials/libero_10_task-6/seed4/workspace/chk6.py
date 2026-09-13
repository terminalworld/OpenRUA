import numpy as np
from circfit import fit
d = np.load("birdview_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]
ok = np.isfinite(z)
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.56)&(z<0.62)
pts=Pw[m]; print("mug rim n", m.sum(), "z", pts[:,2].min().round(3), pts[:,2].max().round(3), "fit", fit(pts[:,:2], 0.0435).round(4), "free", fit(pts[:,:2]).round(4))
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)&(z<0.47)
pts=Pw[m]; print("plate ring n", m.sum(), "fit", fit(pts[:,:2], 0.066).round(4))
m = ok&(Pw[...,0]>-0.10)&(Pw[...,0]<0.03)&(Pw[...,1]>0.02)&(Pw[...,1]<0.16)&(z>0.455)&(z<0.48)
pts=Pw[m]; print("pudding top n", m.sum(), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4), "z", pts[:,2].mean().round(3), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3))
c = pts[:,:2]-pts[:,:2].mean(0); w,v = np.linalg.eigh(c.T@c); print("pudding long axis yaw deg", np.degrees(np.arctan2(v[1,1], v[0,1])).round(1))
