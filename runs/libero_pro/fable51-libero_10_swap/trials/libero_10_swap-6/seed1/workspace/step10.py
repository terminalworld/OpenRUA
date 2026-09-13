from arm import *
a = Arm()
a.gripper(0.04)
pos,_ = a.fk()
q = a.ik((pos[0], pos[1], pos[2] + 0.20), Q_DOWN)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
