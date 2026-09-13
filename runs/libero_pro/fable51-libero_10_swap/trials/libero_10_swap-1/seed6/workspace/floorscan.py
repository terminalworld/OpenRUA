import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node('scan'); buf=Buffer(); TransformListener(buf,node)
for cam in sys.argv[1:]:
    d=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    fr=f'{cam}_optical_frame'
    while not buf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform('world',fr,rclpy.time.Time()); q=t.transform.rotation
    R=qR(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:d.height,0:d.width]
    P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1).reshape(-1,3)@R.T+o
    P=P.reshape(d.height,d.width,3)
    np.save(f'{cam}_xyz.npy',P)
    z=P[...,2]; ok=np.isfinite(z)
    print(cam,'z range',np.nanmin(z[ok]),np.nanmax(z[ok]), 'cam at',o.round(2))
    # floor-level small objects: 0.015<z<0.2, and not the table pedestal (which is a cylinder around table center)
    m=(z>0.015)&(z<0.25)&ok
    r=np.hypot(P[...,0]+0.245,P[...,1])
    m&= r>0.5   # outside pedestal radius (~0.45)
    import cv2
    n,lab,st,cen=cv2.connectedComponentsWithStats(m.astype(np.uint8))
    for i in range(1,n):
        if st[i,4]>=3:
            pts=P[lab==i]; print('  blob px',cen[i].astype(int),'n',st[i,4],'world',pts.mean(0).round(3))
rclpy.shutdown()
