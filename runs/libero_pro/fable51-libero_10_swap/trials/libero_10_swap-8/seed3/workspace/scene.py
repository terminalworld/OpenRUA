"""Segment birdview depth into objects above table; print world centroids + heights."""
import numpy as np, rclpy, struct
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2
from cv_bridge import CvBridge

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

rclpy.init(); node=rclpy.create_node("scene")
buf=Buffer(); TransformListener(buf,node)
cam="birdview"
depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float64)
color=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
while not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=depth.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
pw=pc@R.T+o
Z=pw[...,2]
np.save("birdview_world.npy",pw)
# table height: mode of Z in the center region
zc=Z[150:450,150:490]
hist,edges=np.histogram(zc[np.isfinite(zc)],bins=200)
table_z=edges[np.argmax(hist)]
print("table_z ~",table_z)
mask=(Z>table_z+0.01)&np.isfinite(Z)
# restrict to table region (x range) to exclude robot
mask&=(pw[...,0]>-0.45)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    P=pw[m]
    print(f"blob {i}: px centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) area={stats[i,4]} world x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] ztop={P[:,2].max():.3f} centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f})")
cv2.imwrite("birdview_mask.png",(mask*255).astype(np.uint8))
