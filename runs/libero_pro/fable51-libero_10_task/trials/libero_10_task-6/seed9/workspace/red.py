import numpy as np, cv2, sys
for cam in ["agentview","birdview"]:
    P=np.load(f"{cam}_xyz.npy"); c=np.load(f"{cam}_bgr.npy").astype(int)
    b,g,r=c[...,0],c[...,1],c[...,2]
    red=(r>90)&(r>1.5*g)&(r>1.5*b)&(P[...,2]>0.43)&(P[...,2]<0.7)
    red=red.astype(np.uint8)
    nl,lab,stats,cent=cv2.connectedComponentsWithStats(red)
    for j in range(1,nl):
        if stats[j,4]<10: continue
        pts=P[lab==j]
        print(cam,f"red{j} area={stats[j,4]} px=({cent[j][0]:.0f},{cent[j][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    vis=np.load(f"{cam}_bgr.npy").copy(); vis[red>0]=(0,255,0); cv2.imwrite(f"{cam}_red.png",vis)
