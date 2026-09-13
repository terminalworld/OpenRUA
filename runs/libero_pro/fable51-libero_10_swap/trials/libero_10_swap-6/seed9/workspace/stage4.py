from arm import *
a = Arm('stage4')
p, q = a.tcp_world()
print('start tcp', p.round(4), np.round(q,3), 'fingers', a.finger_gap(), flush=True)
# set the mug down on the free table spot near its original location
T = (-0.096, -0.196)
assert a.move_tcp((T[0], T[1], 0.72), q, secs=5) is not None
assert a.move_line((T[0], T[1], 0.55), q, secs=3) is not None
a.open()
assert a.move_line((T[0], T[1], 0.72), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
