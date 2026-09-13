from robot import *
r = Robot()
q = down_quat(90)
r.gripper(False)
f = r.fingers(); print("fingers after close", f, "gap", round(f[0]-f[1],4))
p = r.tcp_pose()[0]
print("lift"); r.move_tcp([p[0], p[1], 0.80], q, 3.0)
print("fingers after lift", r.fingers())
