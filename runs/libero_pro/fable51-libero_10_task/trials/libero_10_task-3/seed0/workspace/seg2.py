import numpy as np
W = np.load("/workspace/bird_world.npy"); wx,wy,wz = W[...,0],W[...,1],W[...,2]
for u in (350, 365, 380):
    print(f"column u={u} (y~{wy[300,u]:.3f}):")
    for v in range(240,345,3):
        print(f"  v={v} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
