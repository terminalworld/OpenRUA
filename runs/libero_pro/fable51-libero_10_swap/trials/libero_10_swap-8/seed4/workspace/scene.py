import numpy as np, struct, rclpy, sys
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
import cv2

def grab(node, topic, T, timeout=20):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m),1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def cloud(cam):
    rclpy.init(); node=rclpy.create_node('scene'); buf=Buffer(); TransformListener(buf,node)
    d=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    col=grab(node,f'/{cam}/color/image_raw',Image)
    import time; t0=time.time()
    while not buf.can_transform('world',f'{cam}_optical_frame',rclpy.time.Time()) and time.time()-t0<10: rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform('world',f'{cam}_optical_frame',rclpy.time.Time())
    depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:d.height,0:d.width]
    X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
    P=np.stack([X,Y,depth],-1)
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w); tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    W=P@R.T+tr
    rgb=np.frombuffer(col.data,dtype=np.uint8).reshape(col.height,col.width,-1)
    rclpy.shutdown()
    return W,depth,rgb

if __name__=='__main__':
    cam=sys.argv[1] if len(sys.argv)>1 else 'birdview'
    W,depth,rgb=cloud(cam)
    np.save(f'{cam}_world.npy',W)
    z=W[...,2]
    print('z range',np.nanmin(z),np.nanmax(z))
    # table height: mode of z in the middle region
    hist,edges=np.histogram(z[np.isfinite(z)],bins=200)
    i=np.argmax(hist); print('table z ~',edges[i],edges[i+1])
