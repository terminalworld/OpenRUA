import numpy as np, rclpy, struct
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
rclpy.init(); node=rclpy.create_node("scene")
got={}
def grab(topic, T):
    got.pop('m',None)
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
d=CvBridge().imgmsg_to_cv2(grab("/birdview/depth/image_raw", Image), "passthrough").astype(float)
info=grab("/birdview/color/camera_info", CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
# birdview optical frame: world (-0.2,0,3.0), quat (0.7071,0.7071,0,0) -> R = rotx(180)? compute
q=np.array([0.7071,0.7071,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([-0.2,0,3.0])
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
Z=d
X=(us-cx)*Z/fx; Y=(vs-cy)*Z/fy
P=np.stack([X,Y,Z],-1)@R.T+t
np.save("bird_world.npy",P)
zw=P[...,2]
print("z range", np.nanmin(zw), np.nanmax(zw))
# histogram of heights
h,e=np.histogram(zw[np.isfinite(zw)], bins=60)
for hh,ee in zip(h,e): 
    if hh>50: print(f"{ee:.3f} {hh}")
