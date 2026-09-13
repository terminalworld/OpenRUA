from arm import *
a = Arm('gohome')
p,_ = a.tcp_world()
assert a.move_line((p[0], p[1], 0.78), Q_DOWN_Y, secs=3) is not None
a.move_joints(HOME, secs=6)
print('home tcp', a.tcp_world()[0].round(4), flush=True)
