import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
exec(open("pxw.py").read().split("def main")[0])
cam=sys.argv[1]; zlo=float(sys.argv[2]); zhi=float(sys.argv[3])
rclpy.init(); node=rclpy.create_node("rim")
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
T=cam_pose(node,f"{cam}_optical_frame")
print("cam at", np.round(T[:3,3],4))
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
Z=P[...,2]
mask=((Z>zlo)&(Z<zhi)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    pts=P[lab==i][:,:3]
    print(f"blob{i}: area={stats[i,4]} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
    # circle fit (algebraic) on xy
    x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
    (cx_,cy_,c_),*_=np.linalg.lstsq(A,b,rcond=None); r=np.sqrt(c_+cx_**2+cy_**2)
    print(f"   circle fit: center=({cx_:.4f},{cy_:.4f}) r={r:.4f}")
cv2.imwrite("rim_mask.png",mask*255)
rclpy.shutdown()
