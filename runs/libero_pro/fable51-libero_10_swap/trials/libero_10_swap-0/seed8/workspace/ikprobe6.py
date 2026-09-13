import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm
from scipy.spatial.transform import Rotation as Rot
a=Arm()
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(s): s=np.array(s); return np.min(np.minimum(s-lim[:,0], lim[:,1]-s))
seed=[-0.1,0.2,-0.28,-2.73,-0.21,2.42,-0.2]
for tilt in [20,25,30]:
    q=(Rot.from_euler('y',tilt,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
    print("tilt",tilt,"q",np.round(q,4))
    for z in [0.60,0.53,0.47]:
        s=a.ik((-0.258,-0.135,z),q,seed=seed,at_tcp=True)
        print(f"  z={z}: ", "fail" if s is None else f"{np.round(s,3)} margin={margin(s):.2f}", flush=True)
rclpy.shutdown()
