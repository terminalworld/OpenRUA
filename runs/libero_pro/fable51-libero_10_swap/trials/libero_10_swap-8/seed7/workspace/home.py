from rob import *
r = Robot()
HOME = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
code, err = r.move_q(HOME, 4.0)
if err > 0.02: r.move_q(HOME, 4.0)
print("fk", np.round(r.fk_hand()[2],3))
