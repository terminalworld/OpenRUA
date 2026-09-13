import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node('pts'); buf=Buffer(); TransformListener(buf,node)
cam=sys.argv[1]; pts=[tuple(map(int,a.split(','))) for a in sys.argv[2:]]
d=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fr=f'{cam}_optical_frame'
while not buf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,rclpy.time.Time()); q=t.transform.rotation
R=qR(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
for u,v in pts:
    z=D[v,u]; p=R@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])+o
    print(f'({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}')
rclpy.shutdown()
