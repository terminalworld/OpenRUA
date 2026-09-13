"""Project a camera's depth frame to world points. Usage: cloud.py <cam> -> saves <cam>_cloud.npy (H,W,3) world xyz + <cam>_rgb from png"""
import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub); return got["m"]

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
    depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float64)
    color=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    t0=time.time()
    while not buf.can_transform("world",frame,rclpy.time.Time()) and time.time()-t0<15: rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation; tr=t.transform.translation
    R=quat_R(q.x,q.y,q.z,q.w); p=np.array([tr.x,tr.y,tr.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=depth.shape; v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
    pw=pc@R.T+p
    np.save(f"{cam}_cloud.npy",pw); np.save(f"{cam}_rgb.npy",color)
    print(cam, "cam pos",p, "depth range",np.nanmin(depth),np.nanmax(depth))
    rclpy.shutdown()
main()
