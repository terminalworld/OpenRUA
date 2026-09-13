from arm import *
a = Arm('stage1')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
MUG = (-0.110, -0.199)          # TCP over the mug's -y wall (opposite the handle)
PLATE = (0.145, 0.002)
# 1. above the mug wall
assert a.move_tcp((MUG[0], MUG[1], 0.66), Q_DOWN_Y, secs=4) is not None
# 2. descend straight to grasp height (rim at 0.55)
assert a.move_line((MUG[0], MUG[1], 0.52), Q_DOWN_Y, secs=3) is not None
# 3. close
g = a.close()
# 4. lift
assert a.move_line((MUG[0], MUG[1], 0.68), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
