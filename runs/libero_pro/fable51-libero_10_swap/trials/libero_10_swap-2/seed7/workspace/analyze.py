import numpy as np, cv2
P = np.load("/workspace/P_birdview.npy"); Z = P[...,2]
def comps(mask, minarea=30, label=""):
    nlab, lab, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for k in range(1, nlab):
        if stats[k,4] < minarea: continue
        m = lab == k
        print(f"{label} comp {k}: area={stats[k,4]} bbox={stats[k,:4]} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] z[{Z[m].min():.3f},{Z[m].max():.3f}] centroid=({P[m][:,0].mean():.3f},{P[m][:,1].mean():.3f})")
comps((Z > 0.915) & (Z < 0.945) & (P[...,0] > -0.4) & (P[...,1] > 0.0), label="stove-ish")
comps((Z > 0.95) & (Z < 1.05) & (P[...,0] > -0.4) & (P[...,1] > -0.1), label="pan-ish")
# moka pot body: top region
comps((Z > 1.0) & (P[...,1] < -0.1), label="moka top")
comps((Z > 0.905) & (P[...,1] < -0.1) & (P[...,0] > -0.4), label="moka all")
