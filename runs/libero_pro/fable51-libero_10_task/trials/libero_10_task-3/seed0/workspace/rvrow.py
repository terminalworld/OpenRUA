import numpy as np
Wd=np.load("/workspace/rv_world.npy"); wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
for v in (250,300):
  print("row v=",v)
  for u in range(380,640,5):
    print(f"  u={u} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
