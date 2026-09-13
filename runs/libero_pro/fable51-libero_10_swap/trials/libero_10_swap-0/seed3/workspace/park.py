from rob import *
r = Robot()
ok = r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=5)
if not ok: r.move_tcp(-0.30, -0.15, 0.85, yaw=0.0, secs=5)
