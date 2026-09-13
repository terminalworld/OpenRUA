from common import *
r = Robot()
DEST = np.array([PLATE[0], 0.21])
print("lower"); r.go_tcp_line([*DEST, 0.60], [*DEST, 0.452], PUD_Q, 3.0, n=2)
print("open"); r.gripper(0.04)
print("retreat"); r.go_tcp_line([*DEST, 0.452], [*DEST, 0.65], PUD_Q, 2.5, n=1)
# move the arm clear of the scene for an unoccluded view
print("park"); r.go_tcp([-0.15, 0.0, 0.75], TOP, 4.0)
print("fingers", r.fingers())
