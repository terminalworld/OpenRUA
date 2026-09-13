from arm import *
import sys
a = Arm()
x,y,z = map(float, sys.argv[1:4]); yaw = float(sys.argv[4]) if len(sys.argv)>4 else 0.0
move_tcp(a, [x,y,z], yaw, float(sys.argv[5]) if len(sys.argv)>5 else 3.0)
