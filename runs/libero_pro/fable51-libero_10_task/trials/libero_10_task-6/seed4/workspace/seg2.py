import numpy as np, cv2
from scipy import ndimage
d = np.load("birdview_cloud.npz"); Pw, color = d["pw"], d["color"]
z = Pw[...,2]
table = 0.42
mask = np.isfinite(z) & (z > table + 0.008) & (z < table + 0.35)
# exclude robot: robot in birdview is above y? robot base at world x=-0.51. Objects at x > -0.35 roughly
mask &= Pw[...,0] > -0.40
lab, n = ndimage.label(mask)
print("components", n)
vis = color.copy()
for i in range(1, n+1):
    m = lab == i
    if m.sum() < 30: continue
    pts = Pw[m]; c = color[m].mean(0)
    ys, xs = np.where(m)
    print(f"comp {i}: px={m.sum()} center_uv=({xs.mean():.0f},{ys.mean():.0f}) world x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) rgb={c.astype(int)}")
    cv2.rectangle(vis, (xs.min(), ys.min()), (xs.max(), ys.max()), (0,255,0), 1)
    cv2.putText(vis, str(i), (xs.min(), ys.min()-2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)
cv2.imwrite("birdview_seg.png", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
