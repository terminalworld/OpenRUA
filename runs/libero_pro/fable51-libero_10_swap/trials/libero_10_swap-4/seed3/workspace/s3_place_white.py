import numpy as np
from rob import Robot, topdown_quat
r = Robot("s3")
Q = topdown_quat(0.0)
PLATE = np.array([-0.004, -0.315]); R_WALL = 0.042; PLATE_TOP = 0.445; MUG_H = 0.123
tcp_xy = [PLATE[0], PLATE[1] - R_WALL]
print("traverse to above left plate")
if r.move_tcp([tcp_xy[0], tcp_xy[1], 0.70], Q, seconds=4.0) is None: raise SystemExit("fail")
print("lower to 0.62")
if r.move_tcp([tcp_xy[0], tcp_xy[1], 0.62], Q, seconds=2.5) is None: raise SystemExit("fail")
print("wrench", r.read_wrench())
print("lower to placement height")
z_place = PLATE_TOP + MUG_H - 0.025 + 0.004
if r.move_tcp([tcp_xy[0], tcp_xy[1], z_place], Q, seconds=3.0) is None: raise SystemExit("fail")
print("wrench", r.read_wrench(), "fingers", r.fingers())
