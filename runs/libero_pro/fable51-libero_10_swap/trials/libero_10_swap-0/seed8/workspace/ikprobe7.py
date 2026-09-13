import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, quat_to_R
from scipy.spatial.transform import Rotation as Rot
a=Arm()
q=(Rot.from_euler('y',25,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
s=a.ik((-0.258,-0.135,0.60),q,seed=a.arm_q(),at_tcp=True)
print("sol",np.round(s,3))
pos,fq=a.fk(s)
print("requested hand q",np.round(q,4),"\nFK hand q       ",np.round(fq,4))
print("R\n",np.round(quat_to_R(fq),3))
print("tcp", np.round(pos+0.1034*quat_to_R(fq)[:,2],4))
rclpy.shutdown()
