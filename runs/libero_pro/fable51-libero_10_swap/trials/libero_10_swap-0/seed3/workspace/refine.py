"""refine.py x y zlo zhi -> centroid of eye-in-hand depth points near (x,y) with world z in [zlo,zhi]."""
import sys
from rob import *
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
def refine(r, x0, y0, zlo, zhi, rad=0.06, cam="robot0_eye_in_hand"):
    got={}
    s1=r.node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault('d',m),1)
    s2=r.node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault('i',m),1)
    while not('d' in got and 'i' in got): rclpy.spin_once(r.node,timeout_sec=0.2)
    r.node.destroy_subscription(s1); r.node.destroy_subscription(s2)
    d=CvBridge().imgmsg_to_cv2(got['d'],"passthrough").astype(float)
    k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    frame=f"{cam}_optical_frame"
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if r.tfbuf.can_transform("world",frame,rclpy.time.Time()): break
    t=r.tfbuf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
    R=quat_to_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    log("cam at",T.round(4))
    vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
    Z=d; X=(us-cx)*Z/fx; Y=(vs-cy)*Z/fy
    P=np.stack([X,Y,Z],-1).reshape(-1,3)@R.T+T
    ok=np.isfinite(P).all(1)&(P[:,2]>zlo)&(P[:,2]<zhi)&(np.hypot(P[:,0]-x0,P[:,1]-y0)<rad)
    pts=P[ok]
    if len(pts)==0: log("no points"); return None
    c=pts.mean(0); 
    log(f"n={len(pts)} centroid {c.round(4)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.4f}")
    return c, pts
if __name__=="__main__":
    r=Robot(); x0,y0,zlo,zhi=map(float,sys.argv[1:5]); refine(r,x0,y0,zlo,zhi)
