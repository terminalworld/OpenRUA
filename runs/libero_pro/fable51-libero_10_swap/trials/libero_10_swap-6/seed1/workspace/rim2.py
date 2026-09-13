import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
exec(open("pxw.py").read().split("def main")[0])
cam="robot0_eye_in_hand"
rclpy.init(); node=rclpy.create_node("rim2")
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
T=cam_pose(node,f"{cam}_optical_frame")
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape; vv,uu=np.mgrid[0:H,0:W]
P=(np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D,np.ones_like(D)],-1)@T.T)[...,:3]
sel=(P[...,0]>0.05)&(P[...,0]<0.25)&(P[...,1]>-0.15)&(P[...,1]<0.03)&(P[...,2]>0.47)&(P[...,2]<0.60)
pts=P[sel]
print("mug pts:",len(pts),"z range",pts[:,2].min(),pts[:,2].max())
# top-most points (rim highest)
for zlo in (0.56,0.54,0.52,0.50):
    s=pts[pts[:,2]>zlo]
    print(f"z>{zlo}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] mean=({s[:,0].mean():.3f},{s[:,1].mean():.3f})")
# rim extreme points in x among top z>0.52
s=pts[pts[:,2]>0.50]
i=s[:,0].argmax(); j=s[:,0].argmin()
print("x-max rim point:",np.round(s[i],3)," x-min rim point:",np.round(s[j],3))
k=s[:,1].argmax(); l=s[:,1].argmin()
print("y-max point:",np.round(s[k],3)," y-min point:",np.round(s[l],3))
# for x-max side: points with x > xmax-0.01
e=s[s[:,0]>s[:,0].max()-0.012]; print("x-max edge: mean",np.round(e.mean(0),3),"zmax",e[:,2].max())
e=s[s[:,0]<s[:,0].min()+0.012]; print("x-min edge: mean",np.round(e.mean(0),3),"zmax",e[:,2].max())
rclpy.shutdown()
