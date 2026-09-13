from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
for lab, box in [("B", (-0.30, -0.25, 0.0, 0.05)), ("A", (0.10, -0.10, 0.30, 0.10))]:
    m = (wx > box[0]) & (wy > box[1]) & (wx < box[2]) & (wy < box[3]) & (h > 0.02) & ~((wx > 0.086) & (wy < 0.14) & (h < 0.035))
    top = m & (h > h[m].max() - 0.03)
    print(lab, "n", m.sum(), "hmax", h[m].max().round(3), "top(>hmax-3cm) centre", wx[top].mean().round(4), wy[top].mean().round(4), "top extent x", wx[top].min().round(3), wx[top].max().round(3), "y", wy[top].min().round(3), wy[top].max().round(3))
    body = m & (h > 0.10)
    print("   body(h>0.10) centre", wx[body].mean().round(4), wy[body].mean().round(4))
    low = m & (h > 0.06) & (h < 0.13)
    cx, cy = wx[body].mean(), wy[body].mean()
    d = np.hypot(wx[low]-cx, wy[low]-cy); far = d > 0.05
    if far.sum(): print("   handle pts (r>5cm, 6<h<13):", far.sum(), "dir", np.round(np.arctan2((wy[low][far]-cy).mean(), (wx[low][far]-cx).mean())*180/np.pi,1), "deg, r", d[far].max().round(3))
