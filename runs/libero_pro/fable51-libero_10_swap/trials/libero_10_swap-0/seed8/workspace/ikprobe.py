import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, quat_to_R
from scipy.spatial.transform import Rotation as Rot
a=Arm()
tx,ty,tz=0.252,-0.134,0.20
for tilt in [0,10,20]:
    for yaw in [0,45,90,135,180,-45,-90,-135]:
        # hand down, tilted toward -x by tilt degrees, yawed
        R=Rot.from_euler('z',yaw,degrees=True)*Rot.from_euler('y',tilt,degrees=True)*Rot.from_quat([1,0,0,0])
        q=R.as_quat()
        sol=a.ik([tx,ty,tz],q,at_tcp=True)
        print(f"tilt={tilt} yaw={yaw} -> {'OK '+str(np.round(sol,3)) if sol else 'fail'}", flush=True)
rclpy.shutdown()
