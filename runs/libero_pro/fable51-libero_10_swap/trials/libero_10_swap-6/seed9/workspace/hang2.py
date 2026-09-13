import numpy as np, cv2
img = cv2.imread("agentview.png"); pts = np.load("agentview_pts.npy")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
m = ((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 60)
P = pts[m]; P = P[np.isfinite(P).all(1)]
P = P[(P[:, 2] > 0.58) & (np.abs(P[:, 0] + 0.02) < 0.12) & (np.abs(P[:, 1] + 0.03) < 0.12)]
for zlo in np.arange(0.60, 0.76, 0.01):
    S = P[(P[:, 2] >= zlo) & (P[:, 2] < zlo + 0.01)]
    if len(S) > 20:
        thr = np.quantile(S[:, 0], 0.9)
        F = S[S[:, 0] >= thr]
        print("z %.2f n %4d front x %.3f  y_med %.3f (y span %.3f..%.3f)" % (zlo, len(S), F[:, 0].mean(), np.median(F[:, 1]), S[:, 1].min(), S[:, 1].max()))
