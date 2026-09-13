import numpy as np
d=np.load("birdview_depth.npy"); zs=3.0-d
fx=fy=579.31
def world(u,v):
    Z=d[v,u]; return ((v-240)*Z/fy-0.2,(u-320)*Z/fx,3.0-Z)
for name,(u0,u1,v0,v1) in {"yellow":(290,370,270,330),"gray":(390,460,270,330)}.items():
    sub=zs[v0:v1,u0:u1]; m=sub>0.905
    vv,uu=np.where(m); uu+=u0; vv+=v0
    print(name,"px u",uu.min(),uu.max(),"v",vv.min(),vv.max(),"n",len(uu))
    W=np.array([world(u,v) for u,v in zip(uu,vv)])
    print("  x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f"%(W[:,0].min(),W[:,0].max(),W[:,1].min(),W[:,1].max(),W[:,2].min(),W[:,2].max()))
    # rim only (z>0.96)
    r=W[W[:,2]>0.95]
    print("  rim x %.3f..%.3f y %.3f..%.3f center (%.3f,%.3f)"%(r[:,0].min(),r[:,0].max(),r[:,1].min(),r[:,1].max(),r[:,0].mean(),r[:,1].mean()))
    # print z histogram
    print("  z vals",np.round(np.unique(np.round(W[:,2],2)),2))
    # ascii map
    for v in range(vv.min(),vv.max()+1):
        print("  ",v,"".join("#" if zs[v,u]>0.95 else ("+" if zs[v,u]>0.905 else ".") for u in range(uu.min(),uu.max()+1)))
