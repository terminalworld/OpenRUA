from arm import *
import sys
a = Arm('hover')
x, y, z = map(float, sys.argv[1:4])
assert a.move_tcp((x, y, z), Q_DOWN_Y, secs=4) is not None
print('tcp', a.tcp_world()[0].round(4), np.round(a.tcp_world()[1],3), flush=True)
