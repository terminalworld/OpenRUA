import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, quat_to_R
from scipy.spatial.transform import Rotation as Rot
a=Arm()
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(s): s=np.array(s); return np.min(np.minimum(s-lim[:,0], lim[:,1]-s))
seed=a.arm_q()
for yaw in [90,-90]:
    q=(Rot.from_euler('z',yaw,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
    s=seed
    for z in [0.65,0.57,0.49]:
        s=a.ik((0.017,-0.236,z),q,seed=s,at_tcp=True)
        if s is None: print(yaw,z,"fail"); break
        pos,fq=a.fk(s)
        print(f"yaw={yaw} z={z}: {np.round(s,3)} margin={margin(s):.2f} fingeraxis={np.round(quat_to_R(fq)[:,1],2)}")
rclpy.shutdown()
