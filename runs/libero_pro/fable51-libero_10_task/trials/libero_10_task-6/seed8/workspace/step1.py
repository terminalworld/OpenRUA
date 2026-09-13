from robot import *
r = Robot()
ok = r.move_hand([-0.185, 0.0, 0.90], down_quat(90), 4.0)
print("ok", ok, "joints", np.round(r.joints(),3))
print("hand", r.hand_pose())
