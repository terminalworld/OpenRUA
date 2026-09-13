import numpy as np, rclpy, sys
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("probe"); buf=Buffer(); TransformListener(buf,r.node)
sols=np.load("sols.npy",allow_pickle=True).item()
cam="robot0_eye_in_hand"; fr=f"{cam}_optical_frame"
def cloud(tag):
    for _ in range(50):
        r.spin(0.1)
        if buf.can_transform("world",fr,rclpy.time.Time()): break
    t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
    tt=[t.translation.x,t.translation.y,t.translation.z]; q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
    P=r.depth_world(cam,tt,q); np.save(f"eye_{tag}.npy",P); return P
r.move_q(sols["A_grasp"],3.0); print("tcp",r.tcp()[0].round(4),"fingers",r.fingers(),flush=True)
cloud("open")
r.gripper(0.0); print("closed fingers",r.fingers(),flush=True)
cloud("closed")
r.snap(cam,"/workspace/eye_closed.png")
r.gripper(0.04); r.move_q(sols["A_pre"],3.0); print("back at pre",r.fingers(),flush=True)
