import numpy as np, cv2, sys
P = np.load(sys.argv[1] if len(sys.argv) > 1 else "robot0_eye_in_hand_P.npy")
x0, x1, y0, y1 = -0.16, 0.06, 0.05, 0.36   # region
res = 0.002
W, H = int((y1-y0)/res), int((x1-x0)/res)
hm = np.full((H, W), np.nan)
sel = np.isfinite(P[...,2]) & (P[...,2] < 0.65) & (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
pts = P[sel]
ix = ((pts[:,0]-x0)/res).astype(int); iy = ((pts[:,1]-y0)/res).astype(int)
for a, b, z in zip(ix, iy, pts[:,2]):
    if np.isnan(hm[a,b]) or z > hm[a,b]: hm[a,b] = z
img = np.zeros((H, W, 3), np.uint8)
v = np.clip((hm - 0.43)/0.15, 0, 1); v[np.isnan(v)] = 0
img = cv2.applyColorMap((v*255).astype(np.uint8), cv2.COLORMAP_JET)
img[np.isnan(hm)] = 0
# grid lines every 5 cm
for xx in np.arange(-0.15, 0.06, 0.05):
    r = int((xx-x0)/res); cv2.line(img, (0, r), (W-1, r), (255,255,255), 1); cv2.putText(img, f"x={xx:.2f}", (2, r-2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)
for yy in np.arange(0.05, 0.36, 0.05):
    c = int((yy-y0)/res); cv2.line(img, (c, 0), (c, H-1), (255,255,255), 1); cv2.putText(img, f"y={yy:.2f}", (c+2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)
cv2.imwrite("hmap.png", cv2.resize(img, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
print("saved hmap.png; rows = x (down = +x), cols = y (right = +y)")
