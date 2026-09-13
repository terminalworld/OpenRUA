from rlib import *
r = Robot()
print("joints", np.round(r.joints(),3))
print("FK hand world", r.fk_world())
print("TF  hand world", r.tf("world","panda_hand"))
