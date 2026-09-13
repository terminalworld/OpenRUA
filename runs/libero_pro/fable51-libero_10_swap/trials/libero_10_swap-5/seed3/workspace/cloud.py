import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import struct

def grab(node, topic, T, timeout=20):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get('m')

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def cloud(node, tfbuf, cam):
    depth=grab(node, f'/{cam}/depth/image_raw', Image)
    info=grab(node, f'/{cam}/color/camera_info', CameraInfo)
    d=np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    frame=f'{cam}_optical_frame'
    import time; t0=time.time()
    while time.time()-t0<10:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform('world', frame, rclpy.time.Time()): break
    t=tfbuf.lookup_transform('world', frame, rclpy.time.Time())
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w)
    tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    v,u=np.mgrid[0:depth.height,0:depth.width]
    ok=np.isfinite(d)&(d>0)
    P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)[ok]
    W=P@R.T+tr
    return W, tr

if __name__=='__main__':
    rclpy.init(); node=rclpy.create_node('cloud'); tfbuf=Buffer(); TransformListener(tfbuf,node)
    for cam in sys.argv[1:]:
        W,tr=cloud(node,tfbuf,cam)
        np.save(f'img/{cam}_cloud.npy', W.astype(np.float32))
        print(cam, 'cam pos', tr, 'pts', len(W))
    rclpy.shutdown()
