import sys, numpy as np, kin, motion
from ctl import Ctl
# scene update after the pull
kin.OBST.pop("bottle_fallen", None)
kin.OBST["drawer"] = (-0.11, 0.11, 0.07, 0.24, 0.9, 0.99)
kin.OBST["handle_low"] = (-0.05, 0.06, 0.035, 0.08, 0.9, 0.965)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
kin.OBST["bowl"] = (-0.06, 0.10, -0.12, 0.035, 0.9, 0.97)
# richer hand envelope + carried bottle (axis along hand X, center at hand Z=0.096, neck toward -X)
PTS = list(kin.HAND_PTS) + [("palmcorner+", (0.025, 0.1, 0.058)), ("palmcorner-", (0.025, -0.1, 0.058)),
        ("palmcorner2+", (-0.025, 0.1, 0.058)), ("palmcorner2-", (-0.025, -0.1, 0.058)), ("camlow", (0.05, 0, 0.058))]
BOT = [(f"bottle{t:+.2f}", (t, 0, 0.096 + dz)) for t in np.arange(-0.115, 0.045, 0.02) for dz in (-0.02, 0.02)]
c = Ctl()
q = np.array(c.arm_q())
th = np.radians(50)
Z = np.array([0, np.sin(th), -np.cos(th)]); f = np.array([0, np.cos(th), np.sin(th)])
plans = []
for sgn in (1, -1):
    R = motion.R_from_axes(Z, sgn*f)
    xc = -0.035 if R[0,0] < 0 else 0.035   # keep the 16 cm bottle centred in the 18 cm interior
    via = np.array([-0.12, 0.02, 1.16]); tgt = np.array([xc, 0.15, 1.02])
    try:
        qs1 = motion.cart_path(q, via, R); qs2 = motion.cart_path(qs1[-1], tgt, R)
    except RuntimeError as e:
        print("ik fail", sgn, e); continue
    h1 = motion.check(qs1, q, hand_pts=PTS+BOT)
    h2 = motion.check(qs2, qs1[-1], hand_pts=PTS+BOT, ignore=("drawer",), skip_pts=("tcp","finger+y","finger-y")+tuple(n for n,_ in BOT))
    # bottle points vs drawer: only allow inside the interior footprint
    print(f"sgn {sgn} X={R[:,0].round(2)} xc={xc}: dq1 {np.abs(qs1[-1]-q).max():.2f} hits1 {len(h1)} {h1[:2]}  dq2 {np.abs(qs2[-1]-qs1[-1]).max():.2f} hits2 {len(h2)} {h2[:2]}")
    print("   q_end", qs2[-1].round(2))
    if not h1 and not h2: plans.append((np.abs(qs1[-1]-q).max(), sgn, R, qs1, qs2, xc))
plans.sort(key=lambda p: p[0])
if plans and "--go" in sys.argv:
    _, sgn, R, qs1, qs2, xc = plans[0]
    print("using sgn", sgn)
    motion.execute(c, qs1, hand_pts=PTS+BOT, label="via")
    motion.execute(c, qs2, hand_pts=PTS+BOT, ignore=("drawer",), skip_pts=("tcp","finger+y","finger-y")+tuple(n for n,_ in BOT), label="place")
    ph, Rh = kin.hand(np.array(c.arm_q()))
    for n, o in PTS: print("  ", n, (ph + Rh @ np.array(o)).round(3))
c.node.destroy_node()
