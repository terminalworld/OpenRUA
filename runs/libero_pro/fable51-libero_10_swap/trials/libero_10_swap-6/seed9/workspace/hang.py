import numpy as np, cv2
from numpy.linalg import lstsq
for cam in ["birdview", "agentview"]:
    img = cv2.imread(f"{cam}.png"); pts = np.load(f"{cam}_pts.npy")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 60)
    P = pts[m]; P = P[np.isfinite(P).all(1)]
    P = P[(P[:, 2] > 0.58) & (np.abs(P[:, 0] + 0.02) < 0.12) & (np.abs(P[:, 1] + 0.03) < 0.12)]
    print(cam, "n", len(P), "x", P[:, 0].min().round(3), P[:, 0].max().round(3), "y", P[:, 1].min().round(3), P[:, 1].max().round(3), "z", P[:, 2].min().round(3), P[:, 2].max().round(3))
    for zlo in np.arange(0.60, 0.76, 0.02):
        S = P[(P[:, 2] >= zlo) & (P[:, 2] < zlo + 0.02)]
        if len(S) > 20:
            A = np.c_[2 * S[:, 0], 2 * S[:, 1], np.ones(len(S))]; b = S[:, 0] ** 2 + S[:, 1] ** 2
            s = lstsq(A, b, rcond=None)[0]; r = np.sqrt(max(s[2] + s[0] ** 2 + s[1] ** 2, 0))
            print("  z %.2f n %4d x %.3f..%.3f y %.3f..%.3f  fit c(%.3f,%.3f) r %.3f" % (zlo, len(S), S[:, 0].min(), S[:, 0].max(), S[:, 1].min(), S[:, 1].max(), s[0], s[1], r))
