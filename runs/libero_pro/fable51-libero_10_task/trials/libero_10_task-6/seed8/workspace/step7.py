from robot import *
r = Robot()
q = down_quat(0)
PUD = np.array([-0.061, 0.0985])
r.gripper(True); r.settle(0.5)
print("pre-grasp"); r.move_tcp([*PUD, 0.55], q, 3.0)
print("descend");   r.move_tcp([*PUD, 0.442], q, 3.0)
print("tcp", r.tcp_pose()[0].round(4))
r.gripper(False)
f = r.settle(1.0); print("gap after settle", round(f[0]-f[1],4))
f = r.settle(0.5); print("gap after settle2", round(f[0]-f[1],4))
if f[0]-f[1] > 0.02:
    print("lift"); r.move_tcp([*PUD, 0.65], q, 3.0)
    print("fingers after lift", r.fingers())
else:
    print("GRASP FAILED, not lifting")
