import numpy as np, sys
from rob import Robot, topdown_quat
r = Robot("s3b")
Q = topdown_quat(0.0)
tcp, _ = r.tcp_pose_world()
xy = tcp[:2]
z = tcp[2]
base = r.read_wrench()
print("start", np.round(tcp, 4), "wrench", np.round(base, 3))
zmin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.505
while z > zmin + 1e-4:
    z = max(zmin, z - 0.01)
    if r.move_tcp([xy[0], xy[1], z], Q, seconds=1.5) is None: raise SystemExit("fail")
    w = r.read_wrench()
    print(f"z={z:.3f} wrench={np.round(w,3)} dFz={w[2]-base[2]:+.3f} fingers={np.round(r.fingers(),4)}")
    if abs(w[2] - base[2]) > 1.5:
        print("contact detected"); break
