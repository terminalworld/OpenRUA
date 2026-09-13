import numpy as np
for cam in ["agentview","sideview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    z = pw[...,2]; x = pw[...,0]; y = pw[...,1]
    # points within bowl footprint circle radius 7cm around (-0.138,0.04), z>0.902
    r = np.hypot(x+0.138, y-0.040)
    m = np.isfinite(z)&(r<0.075)&(z>0.902)&(z<1.10)
    print(cam, "n", m.sum())
    if m.sum():
        hist, edges = np.histogram(z[m], bins=np.arange(0.90,1.06,0.01))
        for h,e in zip(hist,edges):
            if h: 
                mm = m&(z>=e)&(z<e+0.01)
                print(f"   z {e:.2f}: {h:5d}  r[{r[mm].min():.3f},{r[mm].max():.3f}]")
