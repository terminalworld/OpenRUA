from rob import *
r = Robot()
print("joints", r.joints())
xyz, R = r.fk_hand()
print("hand", xyz.round(4)); print(R.round(3))
t, _ = r.tcp(); print("tcp", t.round(4))
print("open gripper"); r.gripper(GRIP["open_m"])
print(r.joints())
