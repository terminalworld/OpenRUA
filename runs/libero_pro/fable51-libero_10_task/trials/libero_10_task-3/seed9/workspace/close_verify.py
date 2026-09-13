import numpy as np
from rob import *
from goto import margin
from close_plan2 import check2, R_tilt

r = Robot("cv")
t0, R0 = r.tcp(); q = r.arm_q()
# retreat: back along -y, then up
for tcp, tilt, n, secs in [(np.array([0.0, 0.10, 0.98]), 45, 2, 3), (np.array([0.0, -0.02, 1.15]), 45, 3, 4)]:
    path = cart_path(r, t0, R0, tcp, R_tilt(tilt), n, q, max_step=0.6)
    for qq in path:
        c = check2(r, qq, 0.147); print("   via clear", round(c[0], 3), c[1])
    r.move_q(path[-1], secs, via=path[:-1])
    t0, R0 = r.tcp(); q = r.arm_q(); print("tcp", np.round(t0, 3))

r.snap("agentview", "/workspace/snaps/av_closed.png"); r.snap("frontview", "/workspace/snaps/fv_closed.png")
r.snap("sideview", "/workspace/snaps/sv_closed.png")
pts = np.vstack([r.cloud(c)[1].reshape(-1, 3) for c in ["agentview", "frontview", "sideview", "birdview"]])
pts = pts[np.isfinite(pts).all(1)]
# anything in front of the cabinet face (y<0.215) within the cabinet's x span, above the table?
box = (pts[:, 0] > -0.17) & (pts[:, 0] < 0.13) & (pts[:, 1] > 0.0) & (pts[:, 1] < 0.215) & (pts[:, 2] > 0.905) & (pts[:, 2] < 1.13)
p = pts[box]
print("points in front of cabinet face:", len(p))
for zlo in np.arange(0.90, 1.13, 0.02):
    s = (p[:, 2] >= zlo) & (p[:, 2] < zlo + 0.02)
    if s.sum(): print(f"  z {zlo:.2f}: n={s.sum():5d} y min {p[s,1].min():.3f} x [{p[s,0].min():.3f},{p[s,0].max():.3f}]")
# bottom-drawer panel face: points at z 0.92-0.975 with |x|<0.1 -> min y should be ~0.222 minus handle depth
face = pts[(np.abs(pts[:, 0]) < 0.1) & (pts[:, 2] > 0.92) & (pts[:, 2] < 0.975) & (pts[:, 1] > 0.1) & (pts[:, 1] < 0.30)]
print("lower-front region y percentiles [1,5,50]:", np.round(np.percentile(face[:, 1], [1, 5, 50]), 3))
uh = pts[(np.abs(pts[:, 0]) < 0.1) & (pts[:, 2] > 1.0) & (pts[:, 2] < 1.03) & (pts[:, 1] > 0.1) & (pts[:, 1] < 0.30)]
print("upper-handle region y percentiles [1,5,50]:", np.round(np.percentile(uh[:, 1], [1, 5, 50]), 3))
