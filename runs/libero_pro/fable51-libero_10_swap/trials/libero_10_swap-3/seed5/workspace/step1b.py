from rlib import *
r = Robot("s1b")
r.report()
R = top_down_R(np.pi / 2)
x_c, y_h, z_hi, z_lo = -0.11, -0.071, 1.10, 0.965
r.move_cart([([x_c, y_h, z_lo], R)])
r.report(); print("wrench", np.round(r.wrench(), 2))
r.move_cart([([x_c, y_h + 0.07, z_lo], R)])
r.report(); print("wrench", np.round(r.wrench(), 2))
r.move_cart([([x_c, y_h + 0.07, z_hi], R)])
r.report()
