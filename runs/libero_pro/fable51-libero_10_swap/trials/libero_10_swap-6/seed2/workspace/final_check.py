import numpy as np
P = np.load("snaps/birdview_P.npy"); Z = P[...,2]
def fit_circle(xy):
    x,y = xy[:,0], xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    c, *_ = np.linalg.lstsq(A, x*x+y*y, rcond=None)
    return c[0], c[1], np.sqrt(c[2] + c[0]**2 + c[1]**2)
sel = lambda box, zlo, zhi: P[(P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)]
rim = sel((0.03,0.25,-0.12,0.12), 0.545, 0.60)
cx,cy,r = fit_circle(rim[:,:2]); print(f"white mug rim on plate: c=({cx:.3f},{cy:.3f}) r={r:.3f} z=({rim[:,2].min():.3f},{rim[:,2].max():.3f}) n={len(rim)}")
plate = sel((0.03,0.25,-0.12,0.12), 0.435, 0.47)
print(f"plate footprint: x=({plate[:,0].min():.3f},{plate[:,0].max():.3f}) y=({plate[:,1].min():.3f},{plate[:,1].max():.3f}) -> center ({(plate[:,0].min()+plate[:,0].max())/2:.3f},{(plate[:,1].min()+plate[:,1].max())/2:.3f})")
box = sel((0.05,0.25,0.10,0.25), 0.44, 0.50)
print(f"pudding: center=({box[:,0].mean():.3f},{box[:,1].mean():.3f}) x=({box[:,0].min():.3f},{box[:,0].max():.3f}) y=({box[:,1].min():.3f},{box[:,1].max():.3f}) top z=({box[:,2].min():.3f},{box[:,2].max():.3f})")
