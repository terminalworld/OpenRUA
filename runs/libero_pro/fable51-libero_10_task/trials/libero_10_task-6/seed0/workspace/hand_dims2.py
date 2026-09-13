import numpy as np
for cam in ["sideview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; z=pc[...,2]; col=d["col"]
    m=np.isfinite(z)&(np.abs(pc[...,0]+0.19)<0.2)&(np.abs(pc[...,1]+0.05)<0.15)&(z>0.72)&(z<0.755)
    P=pc[m]
    xs=np.arange(-0.26,-0.12,0.01); ys=np.arange(-0.13,0.05,0.01)
    H,_,_=np.histogram2d(P[:,0],P[:,1],bins=[xs,ys])
    print(cam, "finger slice z 0.72-0.755  (rows x, cols y)")
    print("        y:"+"".join(f"{y*100:+4.0f}" for y in ys[:-1]))
    for i,x in enumerate(xs[:-1]): print(f"x={x:+.2f} "+"".join(f"{int(H[i,j]):4d}" if H[i,j]>0 else "   ." for j in range(len(ys)-1)))
