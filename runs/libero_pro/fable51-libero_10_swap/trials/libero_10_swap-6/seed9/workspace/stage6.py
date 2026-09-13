from arm import *
a = Arm('stage6')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
PUD = (-0.176, -0.037)        # box centre; short axis along y
DST = (0.147, 0.18)           # right (+y) of the plate (plate centre y=0.02, r=0.068)
assert a.move_tcp((PUD[0], PUD[1], 0.60), Q_DOWN_Y, secs=4) is not None
assert a.move_line((PUD[0], PUD[1], 0.44), Q_DOWN_Y, secs=3) is not None
a.close()
assert a.move_line((PUD[0], PUD[1], 0.60), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
assert a.move_tcp((DST[0], DST[1], 0.60), Q_DOWN_Y, secs=5) is not None
assert a.move_line((DST[0], DST[1], 0.447), Q_DOWN_Y, secs=3) is not None
a.open()
assert a.move_line((DST[0], DST[1], 0.62), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
