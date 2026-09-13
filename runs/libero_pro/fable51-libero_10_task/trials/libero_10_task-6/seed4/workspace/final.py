import numpy as np
from circfit import fit
d = np.load("birdview_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]; ok=np.isfinite(z)
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.56)&(z<0.62)
pts=Pw[m]; mug = fit(pts[:,:2], 0.0435); print(f"red mug rim: n={m.sum()} center=({mug[0]:.3f},{mug[1]:.3f}) rim z={pts[:,2].mean():.3f}")
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)&(z<0.47)
pts=Pw[m]; plate = fit(pts[:,:2], 0.066); print(f"plate ring: n={m.sum()} center=({plate[0]:.3f},{plate[1]:.3f})")
print(f"mug offset from plate center: {np.hypot(*(mug-plate)):.3f} m (plate radius 0.066, mug radius 0.044)")
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(Pw[...,1]>0.13)&(Pw[...,1]<0.30)&(z>0.45)&(z<0.50)
pts=Pw[m]; print(f"pudding top: n={m.sum()} center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) z={pts[:,2].mean():.3f} y-range [{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
print(f"pudding is {pts[:,1].mean()-plate[1]:.3f} m to +y (agentview-right) of plate center; gap to plate edge {pts[:,1].min()-plate[1]-0.066:.3f} m")
