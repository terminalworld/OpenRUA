from rob import *
r = Robot()
SX, SY = -0.244, -0.173
log("fingers", r.fingers())
ok = r.move_tcp_line(SX, SY, 0.455, yaw=0.0, secs=4, n=3)
if not ok:
    log("retry descent")
    ok = r.move_tcp_line(SX, SY, 0.455, yaw=0.0, secs=4, n=2)
_,_,tcp = r.hand_pose(); log("tcp before close", tcp.round(4))
f = r.gripper(0.0)
log(f"finger gap after close = {f[0]-f[1]:.4f}")
ok2 = r.move_tcp_line(SX, SY, 0.72, yaw=0.0, secs=4, n=3)
if not ok2:
    log("retry lift"); r.move_tcp_line(SX, SY, 0.72, yaw=0.0, secs=4, n=2)
log("fingers after lift", r.fingers())
log("PHASE1B DONE")
