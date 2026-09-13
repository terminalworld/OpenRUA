import numpy as np, cv2
from scipy import ndimage
for cam in ["agentview"]:
    d = np.load(f"{cam}_cloud.npz"); Pw, color = d["pw"], d["color"]
    hsv = cv2.cvtColor(color, cv2.COLOR_RGB2HSV)
    z = Pw[...,2]; ok = np.isfinite(z) & (z > 0.428) & (z < 0.8) & (Pw[...,0] > -0.35)
    h, s, v = hsv[...,0], hsv[...,1], hsv[...,2]
    red = ok & ((h < 8) | (h > 170)) & (s > 120) & (v > 60)
    brown = ok & (v < 90) & (s > 40) & (h > 5) & (h < 30)
    for name, m in [("red", red), ("brown", brown)]:
        lab, n = ndimage.label(m)
        sizes = ndimage.sum(m, lab, range(1, n+1))
        i = np.argmax(sizes) + 1
        mm = lab == i
        pts = Pw[mm]; ys, xs = np.where(mm)
        print(f"{cam} {name}: px={mm.sum()} uv=({xs.mean():.0f},{ys.mean():.0f}) x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z=[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f})")
        vis = color.copy(); vis[mm] = (0,255,0)
        cv2.imwrite(f"{cam}_{name}.png", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
# birdview: mug body diameter estimation. Take red mug comp near (-0.2, 0.03): points z>0.5
d = np.load("birdview_cloud.npz"); Pw = d["pw"]
z = Pw[...,2]
for name, (x0,x1,y0,y1) in {"redmug":(-0.26,-0.15,-0.06,0.10), "whitemug":(-0.16,-0.04,-0.25,-0.10), "plate":(0.05,0.23,-0.08,0.10), "pudding":(-0.09,0.02,0.03,0.14)}.items():
    m = np.isfinite(z) & (Pw[...,0]>x0)&(Pw[...,0]<x1)&(Pw[...,1]>y0)&(Pw[...,1]<y1)&(z>0.428)
    pts = Pw[m]
    print(name, "n", m.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "z", pts[:,2].min().round(3), pts[:,2].max().round(3))
    # z histogram
    hs, es = np.histogram(pts[:,2], bins=np.arange(0.42, 0.80, 0.01))
    print("   z hist:", {round(e,2):int(c) for e,c in zip(es, hs) if c>0})
