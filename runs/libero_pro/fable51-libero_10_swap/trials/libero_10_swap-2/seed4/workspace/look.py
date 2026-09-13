from rk import *
import sys
m = Mover("look")
x,y,z = map(float, sys.argv[1:4]); secs = float(sys.argv[4]) if len(sys.argv)>4 else 4.0
m.goto_pose([x,y,z], TOPDOWN, secs)
