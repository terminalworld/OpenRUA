import sys, numpy as np
from kin import Robot
r = Robot("look")
x, y, z = map(float, sys.argv[1:4])
q = r.ik([x, y, z], [1, 0, 0, 0], seed=r.arm_q()) or r.ik([x, y, z], [1, 0, 0, 0], seed=[0,-0.785,0,-2.356,0,1.571,0.785])
assert q; r.move(q, 4.0); r.report("look")
