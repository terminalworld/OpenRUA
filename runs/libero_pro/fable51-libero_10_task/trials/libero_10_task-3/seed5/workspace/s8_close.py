import numpy as np, rob, sys, time

def panel_y(r, tag):
    img, W = r.snap("birdview", f"bv_{tag}.png")
    X, Y, Z = W[:,:,0], W[:,:,1], W[:,:,2]
    m = (Z > 0.975) & (Z < 0.99) & (X > 0.0) & (X < 0.08) & (Y > 0.0) & (Y < 0.25)
    if m.sum():
        print(f"  [{tag}] panel top y[{Y[m].min():.3f},{Y[m].max():.3f}] n{m.sum()}", flush=True)
    else:
        print(f"  [{tag}] no panel-top points", flush=True)

def push(r, Q, x, y0, y1, zp, zsafe=1.05, tag=""):
    print(f"== push {tag}: y {y0}->{y1} at z {zp}", flush=True)
    q = r.move_pose(rob.hand_from_tcp([x, y0, zsafe], Q), Q, seconds=6.0, max_jump=2.0)
    if q is None: sys.exit("ik fail")
    rob.settle(r, q)
    if rob.go_tcp(r, [x, y0, zp], Q, steps=3, seconds=4.0, max_jump=0.8) is None: sys.exit("fail")
    w0 = r.wrench()
    if rob.go_tcp(r, [x, y1, zp], Q, steps=4, seconds=6.0, max_jump=0.8) is None: sys.exit("fail")
    print("  wrench before", w0.round(2), "after", r.wrench().round(2), flush=True)
    if rob.go_tcp(r, [x, y1, zsafe], Q, steps=2, seconds=3.0, max_jump=0.8) is None: sys.exit("fail")
    panel_y(r, tag)

r = rob.Robot()
r.gripper(0.0)
panel_y(r, "start")
Qv = rob.frame_quat([0, 0, -1], [0, -1, 0])
push(r, Qv, -0.09, 0.04, 0.10, 0.95, tag="p1")
Qx = rob.frame_quat([0, 0, -1], [-1, 0, 0])
push(r, Qx, -0.08, 0.09, 0.17, 0.95, tag="p2")
al = np.radians(45)
Q45 = rob.frame_quat([0, np.sin(al), -np.cos(al)], [-1, 0, 0])
push(r, Q45, -0.08, 0.16, 0.205, 0.935, zsafe=1.0, tag="p3")
print("DONE", flush=True)
