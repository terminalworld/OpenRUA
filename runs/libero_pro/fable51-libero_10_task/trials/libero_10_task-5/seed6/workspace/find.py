import numpy as np, cv2
for cam in ["sideview","frontview","birdview","agentview"]:
    P=np.load(f"g7_{cam}_P.npy"); img=cv2.imread(f"g7_{cam}.png")
    H,W,_=P.shape
    # yellow-ish pixels: B low, R,G high
    b,g,r=img[...,0].astype(int),img[...,1].astype(int),img[...,2].astype(int)
    yel=(r>150)&(g>130)&(b<110)&(r-b>80)
    Q=P[yel]; Q=Q[np.isfinite(Q).all(1)]
    # exclude table-colored? print clusters by rounding
    print(cam,"yellow px",yel.sum())
    if len(Q):
        # remove points at table level far away (wood is brownish, may match). bin xy at 2cm
        keys=np.round(Q[:,:2]/0.02).astype(int)
        u,cnt=np.unique(keys,axis=0,return_counts=True)
        idx=np.argsort(-cnt)[:8]
        for i in idx:
            m=(keys==u[i]).all(1)
            print("   xy %.2f %.2f n=%d z %.3f..%.3f"%(u[i][0]*0.02,u[i][1]*0.02,cnt[i],Q[m][:,2].min(),Q[m][:,2].max()))
