import numpy as np, rclpy, sys
from sensor_msgs.msg import Image
from ctl import Ctl
from cloud import world_points
c = Ctl('lid')
got=[]
sub=c.node.create_subscription(Image,'/birdview/depth/image_raw',got.append,1)
while not got: c.spin()
m=got[0]; D=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width); np.save('birdview_depth.npy',D)
x,y,z,_=world_points('birdview')
# pot region: near tcp, excluding the gripper: highest points in a 12cm box around the pot's old center
m2=(x>-0.13)&(x<0.03)&(y>-0.36)&(y<-0.17)
zz=z[m2]; xx=x[m2]; yy=y[m2]
order=np.argsort(-zz)[:20]
print('top pts in pot box:'); 
for i in order[:8]: print(round(xx[i],4),round(yy[i],4),round(zz[i],4))
# table-level check: any points at z in [0.9,0.92] inside pot base footprint?
base=(np.hypot(x+0.04,y+0.262)<0.03)
print('z hist under old pot center:', np.histogram(z[base],bins=[0.89,0.91,0.93,0.95,0.97,1.0,1.03,1.06,1.1,1.2])[0])
rclpy.shutdown()
