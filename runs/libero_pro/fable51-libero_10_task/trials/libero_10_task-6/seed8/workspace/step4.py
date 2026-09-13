from robot import *
r = Robot()
q = down_quat(90)
PLATE = np.array([0.158, 0.020])
hand_xy = PLATE + [0.037, 0.015]
print("transit above plate"); r.move_tcp([*hand_xy, 0.80], q, 4.0)
print("fingers", r.fingers())
print("lower"); r.move_tcp([*hand_xy, 0.60], q, 3.0)
r.move_tcp([*hand_xy, 0.583], q, 2.0)
print("tcp", r.tcp_pose()[0].round(4))
r.gripper(True)
print("retreat"); r.move_tcp([*hand_xy, 0.75], q, 3.0)
