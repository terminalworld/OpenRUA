import numpy as np
W = np.load("/workspace/bird_world.npy"); wx,wy,wz = W[...,0],W[...,1],W[...,2]
for lo,hi in [(0.95,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.15)]:
    m=np.zeros(wz.shape,bool); m[235:290,315:365]=True; m&=(wz>lo)&(wz<hi)
    if m.sum(): print(f"z in ({lo},{hi}): n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f})")
# row through the bottle
v=262
print("row v=262:", [(u, round(float(wx[v,u]),3), round(float(wy[v,u]),3), round(float(wz[v,u]),3)) for u in range(322,362,2)])
