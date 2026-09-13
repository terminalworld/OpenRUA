from arm import *
a = Arm('stage7')
print('start tcp', a.tcp_world()[0].round(4), 'q', np.round(a.arm_q(),3), flush=True)
# lift straight up first, then return to the nominal home configuration
assert a.move_line((0.147, 0.18, 0.78), Q_DOWN_Y, secs=3) is not None
a.move_joints(HOME, secs=6)
print('home tcp', a.tcp_world()[0].round(4), 'q', np.round(a.arm_q(),3), flush=True)
