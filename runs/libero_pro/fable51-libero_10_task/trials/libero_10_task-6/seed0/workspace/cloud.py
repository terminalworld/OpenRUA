"""Grab color+depth+info from a camera, produce world-frame point cloud; save npz."""
import sys, time, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T, timeout=20):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    end=time.time()+timeout
    while "m" not in got and time.time()<end: rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got["m"]

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    buf=Buffer(); TransformListener(buf,node)
    br=CvBridge()
    col=br.imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
    dep=br.imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float32)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    end=time.time()+10
    while time.time()<end and not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.1)
    t=buf.lookup_transform("world",frame,rclpy.time.Time())
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w)
    p=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    h,w=dep.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    X=(u-cx)*dep/fx; Y=(v-cy)*dep/fy; Z=dep
    pc=np.stack([X,Y,Z],-1).reshape(-1,3)@R.T+p
    pc=pc.reshape(h,w,3)
    np.savez(f"{cam}_cloud.npz",pc=pc,col=col,dep=dep,K=np.array(info.k).reshape(3,3),R=R,p=p)
    print("saved", f"{cam}_cloud.npz", "depth range", np.nanmin(dep), np.nanmax(dep))
    print("optical axes in world: x(right)=",R[:,0].round(3)," y(down)=",R[:,1].round(3)," z(fwd)=",R[:,2].round(3))
    rclpy.shutdown()
main()
