import numpy as np, cv2
from geo import cloud
f=579.4112549695428
for cam in ['agentview','birdview']:
    Pw=cloud(cam,f)
    z=Pw[...,2]
    print(cam,'z range',np.nanmin(z),np.nanmax(z))
    # histogram of z to find table height
    h,edges=np.histogram(z[np.isfinite(z)],bins=np.arange(0.3,1.2,0.01))
    top=np.argsort(h)[-5:]
    print(' most common z:',[(round(edges[i],2),h[i]) for i in top])
    np.save(f'{cam}_xyz.npy',Pw)
