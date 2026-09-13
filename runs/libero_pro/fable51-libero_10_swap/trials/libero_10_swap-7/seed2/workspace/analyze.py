import numpy as np, cv2
P = np.load("birdview_world.npy")
Z = P[...,2]
valid = np.isfinite(Z) & (Z > 0.44) & (Z < 1.0)
n, lab = cv2.connectedComponents(valid.astype(np.uint8))
H, W = Z.shape
vv, uu = np.mgrid[0:H, 0:W]
for i in range(1, n):
    m = lab == i
    if m.sum() < 5: continue
    pts = P[m]
    print(f"bird blob {i}: n={m.sum()} u[{uu[m].min()},{uu[m].max()}] v[{vv[m].min()},{vv[m].max()}] x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
print("---- robotview top surfaces")
P = np.load("robot0_robotview_world.npy")
Z = P[...,2]
for name, (x0,x1,y0,y1) in {"alphabet_can":(-0.26,-0.15,-0.22,-0.10), "tomato_can":(-0.22,-0.11,0.0,0.12), "ketchup":(-0.02,0.12,-0.14,-0.02), "cheese_box":(0.05,0.2,-0.24,-0.14), "basket":(-0.12,0.1,0.14,0.36)}.items():
    m = np.isfinite(Z) & (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(Z>0.44)
    pts = P[m]
    top = pts[:,2].max()
    tm = pts[pts[:,2] > top-0.012]
    print(f"{name}: top z={top:.3f} n_top={len(tm)} top center=({tm[:,0].mean():.3f},{tm[:,1].mean():.3f}) x[{tm[:,0].min():.3f},{tm[:,0].max():.3f}] y[{tm[:,1].min():.3f},{tm[:,1].max():.3f}]")
    if name=="cheese_box":
        xy = tm[:,:2] - tm[:,:2].mean(0)
        w,v = np.linalg.eigh(xy.T@xy)
        print("  box principal axis", v[:,1], "angle deg", np.degrees(np.arctan2(v[1,1],v[0,1])), "extents", np.sqrt(w/len(xy))*np.sqrt(12))
