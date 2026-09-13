import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, JOINTS
from scipy.spatial.transform import Rotation as Rot
a=Arm()
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(s): s=np.array(s); return np.min(np.minimum(s-lim[:,0], lim[:,1]-s))
for z in [0.465,0.60]:
  for tilt in [0,15,30,45]:
    for yaw in [0,90,-90]:
        q=(Rot.from_euler('z',yaw,degrees=True)*Rot.from_euler('y',tilt,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
        s=a.ik((-0.258,-0.135,z),q,at_tcp=True)
        print(f"z={z} tilt={tilt} yaw={yaw}: ", "fail" if s is None else f"{np.round(s,2)} limit-margin={margin(s):.2f}", flush=True)
rclpy.shutdown()
