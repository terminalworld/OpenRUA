from arm import *
a = Arm('stage3')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
G = (0.137, -0.022)   # -y wall of the tilted mug, at fingertip height 0.52
assert a.move_tcp((G[0], G[1], 0.68), Q_DOWN_Y, secs=4) is not None
assert a.move_line((G[0], G[1], 0.52), Q_DOWN_Y, secs=3) is not None
a.close()
assert a.move_line((G[0], G[1], 0.72), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
# level the mug: hand = R_x(165 deg)
Q_LEVEL = (math.sin(math.radians(82.5)), 0.0, 0.0, math.cos(math.radians(82.5)))
assert a.move_tcp((G[0], G[1], 0.72), Q_LEVEL, secs=3) is not None
print('leveled; fingers', a.finger_gap(), 'tcp', a.tcp_world()[0].round(4), flush=True)
