import sys
from cart import *
r = Robot()
H = np.array([-0.19, 0.25]); lim = np.array(FJT["limits_rad"])
def pose(deg, rho, press, zt):
    t = np.radians(deg); d = np.array([np.cos(t), -np.sin(t)]); n = np.array([-np.sin(t), -np.cos(t)])
    p = H + rho*d + (0.01 - press)*n
    tcp = np.array([p[0], p[1], zt]); yh = np.array([d[0], d[1], 0]); zh = np.array([0,0,-1.0]); xh = np.cross(yh, zh)
    return tcp, R_from_axes(xh, yh, zh)
def margin(q): return min(min(qi - lo, hi - qi) for qi, (lo, hi) in zip(q, lim))
if __name__ == "__main__":
    rho, press, zt = 0.19, 0.0, 1.085
    rng = np.random.default_rng(3)
    tcp, R = pose(97, rho, press, zt); hand = hand_pose_from_tcp(tcp, R)
    best = None
    for i in range(30):
        seed = rng.uniform(lim[:,0], lim[:,1])
        q = r.ik_world(hand, quat_from_R(R), seed=seed, attempts=1, timeout=0.5)
        if q is not None:
            m = margin(q)
            if best is None or m > best[0]: best = (m, q)
    print("best start margin", round(best[0],2), np.round(best[1],2))
    # follow arc
    seed = best[1]
    for dg in list(np.arange(97, 0, -8)) + [0]:
        tcp, R = pose(dg, rho, press, zt); q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=seed, attempts=3)
        if q is None: print("fail", dg); break
        print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={max(abs(a-b) for a,b in zip(q,seed)):.3f} margin={margin(q):.2f} q={np.round(q,2)}"); seed = q
