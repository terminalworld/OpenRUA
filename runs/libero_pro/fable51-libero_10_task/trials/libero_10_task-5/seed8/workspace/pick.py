from rob import *
import sys
r = Robot()
cx, cy = -0.122, -0.001          # cup center (world)
rim_r, rim_z = 0.050, 0.985
R = hand_R(0.0)                  # fingers along world X, approach down
wall_x = cx + rim_r - 0.001      # pinch point on +x side of the rim
pre = np.array([wall_x, cy, 1.12])
grasp = np.array([wall_x, cy, rim_z - 0.015])

print("pre-grasp", pre)
ok, info = r.move_tcp(pre, R, 4.0); print(" ->", ok, info)
if not ok: print("WARN: not converged")
print("tcp now", r.tcp_pose()[0])
print("descend", grasp)
ok, info = r.move_tcp(grasp, R, 3.0); print(" ->", ok, info)
print("tcp now", r.tcp_pose()[0])
w0 = r.wrench(); print("wrench before close", w0)
print("close ->", r.gripper(0.0))
print("gap after close", r.finger_gap())
print("wrench after close", r.wrench())
