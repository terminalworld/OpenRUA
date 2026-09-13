import numpy as np, cv2
img = cv2.imread("agentview.png"); pts = np.load("agentview_pts.npy")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
m = ((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 60)
P = pts[m]; P = P[np.isfinite(P).all(1)]
P = P[(P[:, 2] > 0.43) & (P[:, 0] < 0.1)]   # exclude the plate stripes region roughly? keep x<0.1
print("n", len(P))
print("x range", P[:, 0].min().round(3), P[:, 0].max().round(3))
print("y range", P[:, 1].min().round(3), P[:, 1].max().round(3))
print("z range", P[:, 2].min().round(3), P[:, 2].max().round(3))
# body points (no handle): z<0.53; fit circle in xz for points with y in body band
B = P[(P[:, 2] < 0.53)]
from numpy.linalg import lstsq
A = np.c_[2 * B[:, 0], 2 * B[:, 2], np.ones(len(B))]; b = B[:, 0] ** 2 + B[:, 2] ** 2
s = lstsq(A, b, rcond=None)[0]; r = np.sqrt(s[2] + s[0] ** 2 + s[1] ** 2)
print("circle fit xz: cx %.4f cz %.4f r %.4f" % (s[0], s[1], r))
for ylo in np.arange(-0.08, 0.14, 0.02):
    S = P[(P[:, 1] >= ylo) & (P[:, 1] < ylo + 0.02)]
    if len(S): print("y %.2f..%.2f n %4d zmax %.3f xmin %.3f xmax %.3f" % (ylo, ylo + 0.02, len(S), S[:, 2].max(), S[:, 0].min(), S[:, 0].max()))
H = P[(P[:, 1] > -0.02) & (P[:, 1] < 0.04) & (P[:, 0] > 0.03)]
print("handle-ish: n", len(H), "x", H[:, 0].min().round(3), H[:, 0].max().round(3), "z", H[:, 2].min().round(3), H[:, 2].max().round(3))
for xlo in np.arange(-0.04, 0.10, 0.02):
    S = P[(P[:, 0] >= xlo) & (P[:, 0] < xlo + 0.02) & (P[:, 1] > -0.02) & (P[:, 1] < 0.04)]
    if len(S): print("x %.2f..%.2f n %4d z %.3f..%.3f" % (xlo, xlo + 0.02, len(S), S[:, 2].min(), S[:, 2].max()))
# mouth end: points y<-0.02
Mo = P[P[:, 1] < -0.025]
print("mouth end n", len(Mo), "x", Mo[:, 0].min().round(3), Mo[:, 0].max().round(3), "z", Mo[:, 2].min().round(3), Mo[:, 2].max().round(3), "ymin", Mo[:, 1].min().round(3))
