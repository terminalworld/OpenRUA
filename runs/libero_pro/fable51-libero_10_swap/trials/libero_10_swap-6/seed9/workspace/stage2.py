from arm import *
a = Arm('stage2')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
PLATE = (0.145, 0.002)
# 1. carry above the plate (mug bottom ~0.095 below TCP; keep it high)
assert a.move_tcp((PLATE[0], PLATE[1], 0.68), Q_DOWN_Y, secs=5) is not None
print('fingers', a.finger_gap(), flush=True)
# 2. lower straight until the mug bottom is just above the plate (~0.44)
assert a.move_line((PLATE[0], PLATE[1], 0.545), Q_DOWN_Y, secs=3) is not None
# 3. release
a.open()
# 4. retreat straight up
assert a.move_line((PLATE[0], PLATE[1], 0.70), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
