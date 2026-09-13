import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm
from scipy.spatial.transform import Rotation as Rot
a=Arm()
for p in [(0.0,0,0.7),(-0.258,-0.135,0.62),(-0.258,-0.135,0.57),(0.017,-0.236,0.60),(0.0,0.255,0.80)]:
    print(p, "->", None if a.ik(p,[1,0,0,0]) is None else np.round(a.ik(p,[1,0,0,0]),3))
# soup pregrasp with tcp
print("soup tcp 0.62 yaw variants")
for yaw in [0,45,90,-45,-90]:
    q=(Rot.from_euler('z',yaw,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
    s=a.ik((-0.258,-0.135,0.62),q,at_tcp=True)
    print(yaw, None if s is None else np.round(s,3))
rclpy.shutdown()
