from rlib import *
r = Robot("s1a")
r.report()
R = top_down_R(np.pi / 2)
x_c, y_h, z_hi = -0.11, -0.071, 1.10
q_pre = r.ik([x_c, y_h, z_hi], R)
print("q_pre", np.round(q_pre, 3))
p, Rk = r.tcp(q_pre); print("pre tcp", np.round(p, 4), "Z", np.round(Rk[:, 2], 3), "Y", np.round(Rk[:, 1], 3))
r.move_q(q_pre)
r.report()
