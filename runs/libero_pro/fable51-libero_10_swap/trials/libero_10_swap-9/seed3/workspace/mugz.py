import numpy as np
d=np.load("birdview_depth.npy"); zs=3.0-d
np.set_printoptions(linewidth=250)
for v in range(278,312,2):
    print(v, " ".join(f"{zs[v,u]:.2f}"[1:] if zs[v,u]>0.905 else " .." for u in range(302,348,1)))
