from rob import *
r = Robot()
r.spin(0.5); f=r.fingers(); log("fingers", f, "gap", f[0]-f[1])
BX, BY = 0.02, 0.29
ok = r.move_tcp(BX, BY, 0.84, yaw=0.0, secs=5)
if not ok: log("retry"); ok = r.move_tcp(BX, BY, 0.84, yaw=0.0, secs=5)
f=r.fingers(); log("fingers over basket", f, "gap", f[0]-f[1])
ok = r.move_tcp_line(BX, BY, 0.67, yaw=0.0, secs=4, n=3)
if not ok: log("retry"); r.move_tcp_line(BX, BY, 0.67, yaw=0.0, secs=4, n=2)
r.gripper(0.04)
r.spin(0.5); log("fingers after open", r.fingers())
ok = r.move_tcp_line(BX, BY, 0.88, yaw=0.0, secs=4, n=2)
if not ok: log("retry"); r.move_tcp_line(BX, BY, 0.88, yaw=0.0, secs=4, n=2)
# retreat to a neutral pose away from the basket so cameras see it
r.move_tcp(-0.15, 0.0, 0.85, yaw=0.0, secs=5)
log("PHASE4 DONE")
