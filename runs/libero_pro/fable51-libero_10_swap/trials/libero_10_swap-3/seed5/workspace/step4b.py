from rlib import *
r = Robot("s4b")
th = np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc = -0.11
r.report()
w0 = r.wrench(); print("wrench@pre", np.round(w0, 2))
z = 1.10
while z > 0.955:
    z -= 0.015
    code, err = r.move_cart([([xc, 0.0, z], Rpush)])
    q, p, R = r.report()
    print(f"target z={z:.3f} tcp={np.round(p + TCP*R[:,2],4)} err={err:.3f} wrench={np.round(r.wrench(),2)}")
    if err > 0.02:
        print("STALL"); break
