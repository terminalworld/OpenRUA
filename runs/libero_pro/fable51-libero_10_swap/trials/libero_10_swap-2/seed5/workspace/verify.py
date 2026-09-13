import numpy as np, rclpy
from sensor_msgs.msg import Image
from ctl import Ctl
from cloud import world_points
c = Ctl('verify')
got=[]; sub=c.node.create_subscription(Image,'/birdview/depth/image_raw',got.append,1)
while not got: c.spin()
m=got[0]; D=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width); np.save('birdview_depth.npy',D)
x,y,z,_=world_points('birdview')
# stove plate box
box=(x>-0.143)&(x<0.044)&(y>0.111)&(y<0.297)
zz=z[box]; print('stove box z hist', np.histogram(zz,bins=[0.9,0.92,0.935,0.96,1.0,1.03,1.045,1.06,1.2])[0])
top=box&(z>1.045)
print('lid knob top pts:', top.sum(), 'center', np.round([x[top].mean(),y[top].mean(),z[top].max()],4))
body=box&(z>0.99)&(z<1.045)
print('upper chamber center', np.round([x[body].mean(),y[body].mean()],4), 'x range', np.round([x[body].min(),x[body].max()],3), 'y range', np.round([y[body].min(),y[body].max()],3))
# knob region
kb=(np.hypot(x+0.203,y-0.203)<0.05)&(z>0.94)
print('knob bar pts', kb.sum(), 'x range', np.round([x[kb].min(),x[kb].max()],3), 'y range', np.round([y[kb].min(),y[kb].max()],3))
rclpy.shutdown()
