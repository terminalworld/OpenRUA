"""Segment objects above the table from the birdview depth; print world boxes."""
import numpy as np, cv2, rclpy, sys
from sensor_msgs.msg import Image
from rob import quat_to_R
def grab():
    rclpy.init(); node=rclpy.create_node("seg")
    got={}
    node.create_subscription(Image,"/birdview/depth/image_raw",lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    m=got["m"]; d=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width)
    rclpy.shutdown()
    fx=fy=579.4112549695428; cx=320; cy=240
    Rm=quat_to_R(0.7071,0.7071,0,0); t=np.array([-0.2,0,3.0])
    v,u=np.mgrid[0:480,0:640]
    return (np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@Rm.T+t).reshape(480,640,3)
if __name__=="__main__":
    P=grab(); np.save("/workspace/bird_world.npy",P)
    zs=P[...,2]
    zmax=float(sys.argv[1]) if len(sys.argv)>1 else 0.7
    mask=((zs>0.433)&(zs<zmax)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    for k in range(1,n):
        if stats[k,4]<5: continue
        pts=P[lab==k]
        print(f"blob {k}: area={stats[k,4]} px=({cent[k][0]:.0f},{cent[k][1]:.0f}) x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} c=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
