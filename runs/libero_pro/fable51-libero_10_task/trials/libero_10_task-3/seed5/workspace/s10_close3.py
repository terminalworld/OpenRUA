import numpy as np, rob, sys
r = rob.Robot("s10")
al = np.deg2rad(45)
Q45 = rob.frame_quat([0, np.sin(al), -np.cos(al)], [-1, 0, 0])
tcp0 = rob.tcp_now(r)
print("start q", np.round(r.arm_q(),3), "tcp", tcp0.round(4), flush=True)

# dry-run IK for the key poses
seed = r.arm_q()
poses = [([-0.08, 0.155, 1.10], "rot"), ([-0.08, 0.145, 0.935], "low"), ([-0.08, 0.207, 0.935], "end")]
for p, nm in poses:
    sol = rob.ik_near(r, rob.hand_from_tcp(p, Q45), Q45, seed=seed, max_jump=1.5)
    print(nm, None if sol is None else np.round(sol, 3), flush=True)
    if sol is None:
        sys.exit("IK dry run failed")
    seed = sol
if "--dry" in sys.argv:
    sys.exit(0)

print("== rotate in place", flush=True)
if rob.go_tcp(r, [-0.08, 0.155, 1.10], Q45, steps=6, seconds=8.0, max_jump=0.9) is None: sys.exit("rot fail")
print("== lower", flush=True)
if rob.go_tcp(r, [-0.08, 0.145, 0.935], Q45, steps=4, seconds=5.0, max_jump=0.8) is None: sys.exit("low fail")
print("wrench", np.round(r.wrench(),2), flush=True)
for y in (0.185, 0.207):
    print(f"== push to {y}", flush=True)
    rob.go_tcp(r, [-0.08, y, 0.935], Q45, steps=3, seconds=4.0, max_jump=0.8)
    print("wrench", np.round(r.wrench(),2), flush=True)
print("== retreat/lift", flush=True)
rob.go_tcp(r, [-0.08, 0.17, 0.96], Q45, steps=2, seconds=3.0, max_jump=0.8)
rob.go_tcp(r, [-0.08, 0.15, 1.10], Q45, steps=3, seconds=5.0, max_jump=0.8)
print("wrench", np.round(r.wrench(),2), flush=True)
for _ in range(3):
    try:
        r.snap("birdview", "bv_p3.png"); break
    except Exception as e:
        print("snap retry", e, flush=True)
W = np.load("bv_p3_xyz.npy"); X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
m = (X > -0.09) & (X < 0.09) & (Z > 0.975) & (Z < 0.99) & (Y > 0.10) & (Y < 0.22)
print("panel top y", Y[m].min().round(3) if m.sum() else None, Y[m].max().round(3) if m.sum() else None, flush=True)
print("DONE", flush=True)
