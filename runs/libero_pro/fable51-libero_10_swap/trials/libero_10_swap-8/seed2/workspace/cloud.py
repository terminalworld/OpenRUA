"""World-frame point cloud from a camera: python3 cloud.py <cam> -> <cam>_cloud.npy (H,W,3) + <cam>_rgb.npy"""
import rclpy, numpy as np, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    tfs={}
    node.create_subscription(TFMessage,"/tf",lambda m:[tfs.setdefault(t.child_frame_id,t.transform) for t in m.transforms],50)
    d = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(float)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    col = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    fr=f"{cam}_optical_frame"
    while fr not in tfs: rclpy.spin_once(node, timeout_sec=0.2)
    t=tfs[fr]; q=t.rotation; R=quat_R(q.x,q.y,q.z,q.w); o=np.array([t.translation.x,t.translation.y,t.translation.z])
    fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=d.shape; v,u=np.mgrid[0:H,0:W]
    P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)@R.T+o
    np.save(f"{cam}_cloud.npy",P); np.save(f"{cam}_rgb.npy",col)
    print(cam, P.shape, "cam at", o)
main()
