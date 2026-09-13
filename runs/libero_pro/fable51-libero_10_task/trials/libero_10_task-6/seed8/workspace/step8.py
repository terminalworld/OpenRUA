from robot import *
r = Robot()
q = down_quat(0)
TGT = np.array([0.156, 0.155])
print("transit"); r.move_tcp([*TGT, 0.65], q, 4.0)
print("fingers", r.fingers())
print("lower"); r.move_tcp([*TGT, 0.50], q, 3.0); r.move_tcp([*TGT, 0.450], q, 2.0)
print("tcp", r.tcp_pose()[0].round(4))
r.gripper(True); r.settle(1.0)
print("retreat"); r.move_tcp([*TGT, 0.65], q, 3.0)
# park away for a clear view
r.move_hand([-0.25, -0.30, 0.85], down_quat(0), 4.0)
r.settle(1.0)
