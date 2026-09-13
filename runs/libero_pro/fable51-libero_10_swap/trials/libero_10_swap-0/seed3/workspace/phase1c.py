from rob import *
from refine import refine
r = Robot()
SX, SY = -0.244, -0.173
r.move_tcp_line(SX, SY, 0.62, yaw=0.0, secs=3, n=2)
r.gripper(0.04)
res = refine(r, SX, SY, 0.47, 0.53)
if res is not None:
    c, pts = res
    # use the extent midpoint (robust to partial views) 
    SX = (pts[:,0].min()+pts[:,0].max())/2; SY = (pts[:,1].min()+pts[:,1].max())/2
    log(f"refined center {SX:.4f},{SY:.4f}")
log("PHASE1C DONE")
