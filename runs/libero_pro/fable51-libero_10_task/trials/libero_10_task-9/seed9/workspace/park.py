#!/usr/bin/env python3
import sys, numpy as np
from rob import *
x,y,z = map(float, sys.argv[1:4])
r = Robot("park")
q0 = r.arm_q(); t, R = r.tcp(); log("tcp", np.round(t,4))
r.move_to_pose(np.array([x,y,z]), R_down(0.0), seed=q0, speed=0.3)
