from common import *
r = Robot()
top = [*(PLATE + [0, R_MUG]), 0.72]; dn = [*(PLATE + [0, R_MUG]), 0.58]
print("lower onto plate"); r.go_tcp_line(top, dn, TOP, 3.0, n=2)
print("open"); r.gripper(0.04)
print("retreat up"); r.go_tcp_line(dn, top, TOP, 2.0, n=1)
print("fingers", r.fingers())
