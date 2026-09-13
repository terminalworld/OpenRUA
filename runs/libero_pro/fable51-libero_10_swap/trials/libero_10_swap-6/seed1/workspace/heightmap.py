import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
sys.argv=[sys.argv[0],"birdview"]
exec(open("pxw.py").read().split("def main")[0])
rclpy.init(); node=rclpy.create_node("hm")
cam="birdview"
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
T=cam_pose(node,f"{cam}_optical_frame")
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape
vv,uu=np.mgrid[0:H,0:W]
X=(uu-cx)*D/fx; Y=(vv-cy)*D/fy
P=np.stack([X,Y,D,np.ones_like(D)],-1)@T.T
Z=P[...,2]
np.save("bird_world.npy",P[...,:3])
print("table z mode:", np.median(Z[(Z>0.3)&(Z<0.5)]))
mask=((Z>0.44)&(Z<0.75)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    pts=P[m][:,:3]
    print(f"blob{i}: px area={stats[i,4]} centroid px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
cv2.imwrite("bird_mask.png", mask*255)
rclpy.shutdown()
