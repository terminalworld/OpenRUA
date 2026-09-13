from arm import *
a = Arm('stage5')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
G = (-0.129, -0.261)          # -y wall of the upright mug
PLATE = (0.138, 0.020)
OFF = 0.047                   # mug centre is +y of the grasped wall
assert a.move_tcp((G[0], G[1], 0.68), Q_DOWN_Y, secs=4) is not None
p, q = a.tcp_world(); print('pre-grasp hand quat', np.round(q, 3), flush=True)
assert a.move_line((G[0], G[1], 0.52), Q_DOWN_Y, secs=3) is not None
a.close()
assert a.move_line((G[0], G[1], 0.70), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
assert a.move_tcp((PLATE[0], PLATE[1] - OFF, 0.70), Q_DOWN_Y, secs=5) is not None
assert a.move_line((PLATE[0], PLATE[1] - OFF, 0.545), Q_DOWN_Y, secs=3) is not None
a.open()
assert a.move_line((PLATE[0], PLATE[1] - OFF, 0.72), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
