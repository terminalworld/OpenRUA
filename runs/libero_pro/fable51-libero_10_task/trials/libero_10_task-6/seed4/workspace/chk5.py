import numpy as np
from circfit import fit
d = np.load("birdview_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]
# mug rim in the air above the plate: z between 0.68 and 0.76
m = np.isfinite(z)&(Pw[...,0]>0.0)&(Pw[...,0]<0.3)&(np.abs(Pw[...,1])<0.2)&(z>0.66)&(z<0.75)
pts = Pw[m]; print("mug-rim-ish pts n", m.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "zmax", pts[:,2].max().round(3))
print("circle fit r=0.044:", fit(pts[:,:2], 0.0435).round(4))
m2 = np.isfinite(z)&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)&(z<0.47)
p2=Pw[m2]; print("plate centroid", p2[:,0].mean().round(4), p2[:,1].mean().round(4))
