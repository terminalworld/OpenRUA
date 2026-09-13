import rclpy, numpy as np, struct, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from cv_bridge import CvBridge
import cv2

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class Cam:
    def __init__(self, node, buf, name):
        self.name=name; self.node=node; self.buf=buf
        self.depth=None; self.color=None; self.info=None
        node.create_subscription(Image, f"/{name}/depth/image_raw", self._d, 1)
        node.create_subscription(Image, f"/{name}/color/image_raw", self._c, 1)
        node.create_subscription(CameraInfo, f"/{name}/color/camera_info", self._i, 1)
    def _d(self,m): self.depth=CvBridge().imgmsg_to_cv2(m,"passthrough")
    def _c(self,m): self.color=CvBridge().imgmsg_to_cv2(m,"bgr8")
    def _i(self,m): self.info=m
    def ready(self): return self.depth is not None and self.color is not None and self.info is not None
    def T(self):
        t=self.buf.lookup_transform("world", f"{self.name}_optical_frame", Time())
        q=t.transform.rotation; T=np.eye(4); T[:3,:3]=quat_R(q.x,q.y,q.z,q.w)
        T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]; return T
    def cloud(self):
        k=self.info.k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
        h,w=self.depth.shape; u,v=np.meshgrid(np.arange(w),np.arange(h))
        z=self.depth.astype(float)
        pc=np.stack([(u-cx)*z/fx,(v-cy)*z/fy,z,np.ones_like(z)],-1)
        P=(self.T()@pc.reshape(-1,4).T).T[:,:3].reshape(h,w,3)
        return P
    def px2world(self,u,v):
        return self.cloud()[v,u]

def grab(names):
    rclpy.init(); node=rclpy.create_node("scene"); buf=Buffer(); TransformListener(buf,node)
    cams=[Cam(node,buf,n) for n in names]
    while not all(c.ready() for c in cams): rclpy.spin_once(node,timeout_sec=0.2)
    for _ in range(10): rclpy.spin_once(node,timeout_sec=0.1)
    for c in cams:
        while not buf.can_transform("world", f"{c.name}_optical_frame", Time()): rclpy.spin_once(node,timeout_sec=0.2)
    return node,cams

if __name__=="__main__":
    node,cams=grab([sys.argv[1]])
    c=cams[0]; P=c.cloud()
    np.save(f"{c.name}_cloud.npy",P); cv2.imwrite(f"{c.name}.png",c.color)
    for a in sys.argv[2:]:
        u,v=map(int,a.split(","))
        print(a, P[v,u], "depth", c.depth[v,u])
    rclpy.shutdown()
