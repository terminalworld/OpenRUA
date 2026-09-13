import numpy as np, subprocess, sys
from PIL import Image
cam=sys.argv[1] if len(sys.argv)>1 else "frontview"
subprocess.run(["timeout","120","./tools/perception/cam_snap.py",cam,f"{cam}.png"],stdout=subprocess.DEVNULL)
im=np.asarray(Image.open(f"{cam}.png").convert("RGB")).astype(int)
r,g,b=im[...,0],im[...,1],im[...,2]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
rows=np.where(yellow.any(axis=1))[0]
print("yellow rows",rows.min(),rows.max())
for y in range(rows.min(),rows.max()+1,4):
    xs=np.where(yellow[y])[0]
    if len(xs): print(y, xs.min(), xs.max(), "w",xs.max()-xs.min(),"c",(xs.min()+xs.max())/2)
