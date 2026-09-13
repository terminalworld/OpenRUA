import numpy as np, cv2
from rob import *
from look import q_from_dir
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
r=Rob()
def snap(name):
    got=[]
    s=r.node.create_subscription(Image,'/robot0_eye_in_hand/color/image_raw',got.append,1)
    while not got: r.spin(0.2)
    r.node.destroy_subscription(s)
    cv2.imwrite(name, CvBridge().imgmsg_to_cv2(got[0],'bgr8')); print('saved',name)
dirs={'py':(0,1,-0.45),'px':(1,0,-0.45),'mxpy':(-1,1,-0.45),'pxmy':(1,-1,-0.45),'my':(0,-1,-0.45),'mxmy':(-1,-1,-0.45),'mx':(-1,0,-0.3)}
for k,d in dirs.items():
    # stand the hand up high, offset opposite the look direction
    p=np.array([-0.35,0.0,0.95])
    q=q_from_dir(d)
    ok=r.move_tcp(*p,q=q,sec=4); print(k,'ok',ok)
    snap(f'look_{k}.png')
r.shutdown()
