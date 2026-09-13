import sys, numpy as np
from rob import *; from plan import Planner; from attach import detach_mug
r = Planner("place"); R = R_from_axes([0,1,0],[1,0,0])
tcp,_ = r.tcp(); print("tcp", tcp.round(4))
ok = r.move_line_tcp([[tcp[0], tcp[1], 1.009]], R, avoid=False); print("lower ok", ok, "fingers", np.round(r.fingers(),4))
r.gripper(0.04)
tcp,_ = r.tcp(); detach_mug(r, [tcp[0], tcp[1]+0.0725, 0.9017+0.0423+0.056])
ok = r.move_line_tcp([[tcp[0], 0.12, tcp[2]]], R, avoid=False); print("retreat ok", ok, r.tcp()[0].round(3))
