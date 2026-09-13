import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
mugc = np.array([float(v) for v in sys.argv[1:4]])   # desired mug centre (world)
lim = np.array(FJT["limits_rad"]); rng = np.random.default_rng(1)
for deg in range(-90, 91, 15):
    Ry = Rotation.from_euler('y', deg, degrees=True).as_matrix()
    R = Ry @ Rf
    tcp = mugc + 0.044 * R[:,1]          # mug centre = tcp - 0.044*y_h
    hand = hand_pose_from_tcp(tcp, R); quat = quat_from_R(R)
    best = None
    seeds = [r.joints()] + [rng.uniform(lim[:,0], lim[:,1]) for _ in range(6)]
    for seed in seeds:
        q = r.ik_world(hand, quat, seed=seed, attempts=1, timeout=0.5)
        if q is not None:
            m = min(min(qi - lo, hi - qi) for qi, (lo, hi) in zip(q, lim))
            if best is None or m > best[0]: best = (m, np.round(q,2))
    print(f"roll {deg:4d}: tcp={np.round(tcp,3)} y_h={np.round(R[:,1],2)} ", "none" if best is None else f"margin={best[0]:.2f} q={best[1]}")
