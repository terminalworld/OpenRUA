#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ----; ros2 node list

# openrua op 4
cd /workspace; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ----; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 6
cd /workspace; echo "bowl center agentview:"; timeout 60 python3 tools/perception/px2world.py agentview 352 210; echo "bowl rim left:"; timeout 60 python3 tools/perception/px2world.py agentview 325 208; echo "bowl rim right:"; timeout 60 python3 tools/perception/px2world.py agentview 380 208; echo "table near bowl:"; timeout 60 python3 tools/perception/px2world.py agentview 352 260; echo "bottle top:"; timeout 60 python3 tools/perception/px2world.py agentview 287 215; echo "drawer front top edge:"; timeout 60 python3 tools/perception/px2world.py agentview 230 300; echo "drawer interior:"; timeout 60 python3 tools/perception/px2world.py agentview 225 260

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud: <cam>_cloud.npy (H,W,3)."""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pts = np.stack([X, Y, depth], -1) @ R.T + p0
    np.save(f"{cam}_cloud.npy", pts)
    print(f"{cam}_cloud.npy", pts.shape, "cam at", p0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
cd /workspace; timeout 120 python3 cloud.py birdview; timeout 120 python3 cloud.py agentview; timeout 120 python3 cloud.py robot0_eye_in_hand

# openrua op 9
cd /workspace; python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy')
# top-down height map: print z stats in regions
def show(name,u0,u1,v0,v1):
    r=P[v0:v1,u0:u1]; z=r[...,2]
    print(name,'x[%.3f,%.3f] y[%.3f,%.3f] zmin %.3f zmax %.3f'%(r[...,0].min(),r[...,0].max(),r[...,1].min(),r[...,1].max(),z.min(),z.max()))
show('table',150,500,150,400)
# objects above table: z>0.93
m=P[...,2]>0.93
import scipy.ndimage as nd
lab,n=nd.label(m)
for i in range(1,n+1):
    sel=lab==i
    if sel.sum()<30: continue
    pts=P[sel]
    vs,us=np.nonzero(sel)
    print('blob',i,'n',sel.sum(),'px u[%d,%d] v[%d,%d]'%(us.min(),us.max(),vs.min(),vs.max()),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
"


# openrua op 10
cd /workspace; python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy')
# coarse heightmap: 2cm cells over x[-0.45,0.2], y[-0.45,0.2]
xs=np.arange(-0.45,0.20,0.02); ys=np.arange(-0.45,0.20,0.02)
X=P[...,0].ravel();Y=P[...,1].ravel();Z=P[...,2].ravel()
H=np.full((len(xs),len(ys)),np.nan)
for i,x in enumerate(xs):
  for j,y in enumerate(ys):
    s=(X>=x)&(X<x+0.02)&(Y>=y)&(Y<y+0.02)
    if s.any(): H[i,j]=Z[s].max()
print('      '+' '.join('%5.2f'%y for y in ys))
for i,x in enumerate(xs):
  print('%5.2f '%x+' '.join(('%5.3f'%H[i,j])[1:] if not np.isnan(H[i,j]) else '  .  ' for j in range(len(ys))))
"


# openrua op 11
cd /workspace; python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy')
xs=np.arange(-0.27,0.05,0.02); ys=np.arange(-0.47,0.17,0.02)
X=P[...,0].ravel();Y=P[...,1].ravel();Z=P[...,2].ravel()
print('        '+' '.join('%5.2f'%y for y in ys))
for x in xs:
  row=[]
  for y in ys:
    s=(X>=x)&(X<x+0.02)&(Y>=y)&(Y<y+0.02)
    row.append('%5.3f'%Z[s].max() if s.any() else '  .  ')
  print('%5.2f '%x+' '.join(row))
"


# openrua op 12
cd /workspace; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy')
X=P[...,0];Y=P[...,1];Z=P[...,2]
# bowl region: x in [-0.3,-0.05], y in [-0.03,0.16], z>0.905
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.905)&(Z<1.2)
pts=P[s]
print('n',len(pts))
print('x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
# rim points: z>1.0
r=pts[pts[:,2]>1.0]
print('rim n',len(r),'x[%.3f,%.3f] y[%.3f,%.3f] mean',r[:,:2].mean(0), 'zmean %.3f'%r[:,2].mean())
# histogram z
h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.06,0.01))
for c,a in zip(h,e): print('%.2f %d'%(a,c))
vs,us=np.nonzero(s); print('px u[%d,%d] v[%d,%d]'%(us.min(),us.max(),vs.min(),vs.max()))
"


# openrua op 13
cd /workspace; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy')
X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.905)&(Z<1.03)
pts=P[s]
print('x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
r=pts[pts[:,2]>1.012]
print('rim n',len(r),'x[%.3f,%.3f] y[%.3f,%.3f]'%(r[:,0].min(),r[:,0].max(),r[:,1].min(),r[:,1].max()),'center',r[:,:2].mean(0))
# fit circle to rim in xy
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('circle center (%.3f,%.3f) radius %.3f'%(c[0],c[1],R))
lo=pts[pts[:,2]<0.985]
print('low pts x[%.3f,%.3f] y[%.3f,%.3f]'%(lo[:,0].min(),lo[:,0].max(),lo[:,1].min(),lo[:,1].max()))
"


# openrua op 14
cd /workspace; python3 -c "
import numpy as np
for cam in ['robot0_eye_in_hand']:
    P=np.load(cam+'_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
    s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.905)&(Z<1.1)
    pts=P[s]
    print(cam,'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
    h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.06,0.01))
    for c,a in zip(h,e): print('%.2f %d'%(a,c))
    r=pts[pts[:,2]>1.012]
    A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print('circle center (%.3f,%.3f) radius %.3f'%(c[0],c[1],R))
    # table right around the bowl
    s2=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z<0.905)
    print('table pts near bowl', s2.sum())
"


# openrua op 15
cd /workspace; timeout 120 python3 cloud.py robot0_robotview; timeout 120 python3 cloud.py frontview; python3 -c "
import numpy as np
for cam in ['robot0_robotview','frontview']:
    P=np.load(cam+'_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
    s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.903)&(Z<1.1)
    pts=P[s]
    print(cam,'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
    h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.06,0.01))
    print(' '.join('%.2f:%d'%(a,c) for c,a in zip(h,e)))
    for zlo in np.arange(0.90,1.03,0.01):
        q=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
        if len(q): print('  z %.2f x[%.3f,%.3f] y[%.3f,%.3f]'%(zlo,q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
"


# openrua op 16
cd /workspace; python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[270:350,290:400]; cv2.imwrite('crop_front.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('robot0_robotview.png'); c=im[40:150,300:460]; cv2.imwrite('crop_robot.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 17
cd /workspace; timeout 120 python3 cloud.py sideview; python3 -c "
import numpy as np
P=np.load('sideview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.903)&(Z<1.1)
pts=P[s]
print('x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
for zlo in np.arange(0.90,1.03,0.01):
    q=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(q): print('  z %.2f n %d x[%.3f,%.3f] y[%.3f,%.3f]'%(zlo,len(q),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
"


# openrua op 18
cd /workspace; timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 19
cd /workspace; timeout 120 python3 cloud.py birdview; python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.903)&(Z<1.1)
pts=P[s]
print('bowl x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
s=(X>-0.25)&(X<0.0)&(Y>-0.22)&(Y<-0.04)&(Z>0.903)&(Z<1.1)
pts=P[s]; print('drawer z[%.3f,%.3f]'%(pts[:,2].min(),pts[:,2].max()))
"

# openrua op 20
cd /workspace; timeout 120 python3 cloud.py agentview; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.903)&(Z<1.1)
pts=P[s]
print('bowl x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
r=pts[pts[:,2]>pts[:,2].max()-0.012]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center (%.3f,%.3f) radius %.3f zrim %.3f'%(c[0],c[1],R,r[:,2].mean()))
P=np.load('birdview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
xs=np.arange(-0.27,0.05,0.02); ys=np.arange(-0.47,0.17,0.02)
Xf=X.ravel();Yf=Y.ravel();Zf=Z.ravel()
print('        '+' '.join('%5.2f'%y for y in ys))
for x in xs:
  row=[]
  for y in ys:
    s=(Xf>=x)&(Xf<x+0.02)&(Yf>=y)&(Yf<y+0.02)
    row.append('%5.3f'%Zf[s].max() if s.any() else '  .  ')
  print('%5.2f '%x+' '.join(row))
"

# openrua op 21
cd /workspace; python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy'); X=P[...,0].ravel();Y=P[...,1].ravel();Z=P[...,2].ravel()
# x profile of drawer at y in [-0.19,-0.11]
print('x profile (y -0.19..-0.11):')
for x in np.arange(-0.25,0.03,0.005):
    s=(X>=x)&(X<x+0.005)&(Y>-0.19)&(Y<-0.11)
    if s.any(): print(' x %.3f zmax %.3f'%(x,Z[s].max()))
print('y profile (x -0.19..-0.05):')
for y in np.arange(-0.25,0.0,0.005):
    s=(Y>=y)&(Y<y+0.005)&(X>-0.19)&(X<-0.05)
    if s.any(): print(' y %.3f zmax %.3f'%(y,Z[s].max()))
"

# openrua op 22
cd /workspace; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0].ravel();Y=P[...,1].ravel();Z=P[...,2].ravel()
print('y profile (x -0.19..-0.05) from agentview:')
for y in np.arange(-0.25,0.0,0.005):
    s=(Y>=y)&(Y<y+0.005)&(X>-0.19)&(X<-0.05)&(Z>0.905)
    if s.any(): print(' y %.3f zmax %.3f zmin %.3f n %d'%(y,Z[s].max(),Z[s].min(),s.sum()))
print('handle region x profile at y in [-0.06,-0.04], z in [0.94,0.97]')
s=(Y>-0.065)&(Y<-0.04)&(Z>0.93)&(Z<0.975)&(X>-0.25)&(X<0.0)
print(' x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] n %d'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max(),s.sum()))
"

# openrua op 23
cd /workspace; python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
EOF
grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A14 'link name="panda_leftfinger"' robot.urdf | head -50; grep -n -A6 "joint name=\"panda_finger_joint1\"" robot.urdf

# openrua op 24
cd /workspace; ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/; python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        import re
        v=np.array([[float(x) for x in m] for m in re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)',d)],float)
    else:
        n=struct.unpack('<I',d[80:84])[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
base='/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/'
for f in ['hand.stl','finger.stl','link7.stl','link6.stl']:
    lo,hi=stl_bounds(base+f); print(f,'min',np.round(lo,3),'max',np.round(hi,3))
EOF

# openrua op 25
cd /workspace; find / -name "hand.stl" -o -name "finger.stl" -o -name "hand.dae" 2>/dev/null | head; grep -o '<joint name="panda_finger_joint1"[^/]*/>' robot.urdf | head -3; grep -o 'panda_finger_joint1.*\?</joint>' robot.urdf | head -c 600; echo; grep -o '<joint name="panda_hand_joint".\{0,400\}' robot.urdf | head -c 500; echo; grep -o '<joint name="panda_joint8".\{0,400\}' robot.urdf | head -c 500

# openrua op 26
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper layer over MoveIt IK/FK + FollowJointTrajectory + gripper.

All poses are the panda_hand frame in the world frame (world = panda_link0
shifted by the base offset read from TF once).
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_axis(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.array([*(axis * math.sin(ang / 2)), math.cos(ang / 2)])


# hand pointing straight down, finger axis along world X
Q_DOWN_FX = np.array([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0])
# hand pointing straight down, finger axis along world Y (home-like)
Q_DOWN_FY = np.array([1.0, 0.0, 0.0, 0.0])


def tilted(q_base, axis, ang):
    """Rotate orientation q_base by ang about a WORLD axis."""
    return quat_mul(quat_axis(axis, ang), q_base)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.wrench = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self.wrench.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        # world -> panda_link0 offset (static)
        buf = Buffer(); TransformListener(buf, self.node)
        t0 = time.time()
        while time.time() - t0 < 10 and not buf.can_transform("world", "panda_link0", rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = buf.lookup_transform("world", "panda_link0", rclpy.time.Time()).transform.translation
        self.base = np.array([t.x, t.y, t.z])
        self.spin(0.3)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
            while "m" not in self.js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]), abs(j["panda_finger_joint2"])

    def force(self):
        self.wrench.pop("m", None)
        t0 = time.time()
        while "m" not in self.wrench and time.time() - t0 < 3:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self.wrench:
            return None
        f = self.wrench["m"].wrench.force
        return np.array([f.x, f.y, f.z])

    def hand_pose(self, q=None):
        """FK: world-frame position + quaternion of panda_hand."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    # ---------------- planning ----------------
    def solve_ik(self, pos, quat, seed=None, attempts=3):
        """pos: world-frame panda_hand position. Returns joint list or None."""
        pos = np.asarray(pos, float) - self.base
        seed = list(seed if seed is not None else self.arm_q())
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = seed
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[n] for n in JOINTS]
        return None

    # ---------------- acting ----------------
    def move_joints(self, waypoints, seconds, verify=True):
        """waypoints: list of joint vectors (or one). seconds: total time."""
        if not isinstance(waypoints[0], (list, tuple, np.ndarray)):
            waypoints = [waypoints]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = None
        if verify:
            q = np.array(self.arm_q())
            err = float(np.max(np.abs(q - np.array(waypoints[-1], float))))
        return code, err

    def move_to(self, pos, quat, seconds=3.0, seed=None):
        q = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(pos,3)}")
        code, err = self.move_joints(q, seconds)
        return q, code, err

    def move_tcp(self, tcp_pos, quat, seconds=3.0, seed=None):
        """Place the fingertip point (TCP) at tcp_pos."""
        R = quat_R(quat)
        hand = np.asarray(tcp_pos, float) - TCP * R[:, 2]
        return self.move_to(hand, quat, seconds, seed)

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 27
cd /workspace; timeout 300 python3 - <<'EOF'
import numpy as np
from arm import *
a=Arm()
print('base',a.base)
print('q',np.round(a.arm_q(),3))
pos,q=a.hand_pose(); print('hand',np.round(pos,3),np.round(q,3))
print('fingers',a.fingers(),'force',a.force())
# IK for pre-grasp above the bowl rim (+x side), fingers along x
bowl=np.array([-0.161,0.057]); grasp=np.array([bowl[0]+0.052,bowl[1]])
tcp=np.array([grasp[0],grasp[1],1.02])
R=quat_R(Q_DOWN_FX); hand=tcp-TCP*R[:,2]
print('hand target',np.round(hand,3),'R z',R[:,2],'R y (finger axis)',R[:,1])
sol=a.solve_ik(hand,Q_DOWN_FX); print('ik',None if sol is None else np.round(sol,3))
if sol is not None:
    p,qq=a.hand_pose(sol); print('fk check',np.round(p,3),np.round(qq,3))
qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(15)); R=quat_R(qt); print('tilted z axis',np.round(R[:,2],3),'finger axis',np.round(R[:,1],3))
a.close()
EOF

# openrua op 28
cd /workspace; grep -o '<link name="world"' robot.urdf; grep -o '<joint name="[^"]*" type="fixed"><parent link="world"/>.\{0,200\}' robot.urdf; grep -o '<joint name="virtual_joint".\{0,300\}' robot.urdf; grep -o 'parent link="world".\{0,200\}' robot.urdf | head

# openrua op 29
cd /workspace; grep -o '<link name="[^"]*"' robot.urdf | head -20; grep -o '<joint name="[^"]*" type="[^"]*"><[^>]*><[^>]*><[^>]*>' robot.urdf | grep -i -E "world|link0" | head; timeout 30 ros2 param get /move_group robot_description_planning.joint_limits 2>/dev/null | head -2; timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1}}" 2>/dev/null | grep -E "frame_id|position|orientation" | head -20

# openrua op 30
cd /workspace; timeout 300 python3 - <<'EOF'
import numpy as np
from arm import *
a=Arm(); a.base=np.zeros(3)
p,q=a.hand_pose([0,-0.785,0,-2.356,0,1.571,0.785]); print('ready pose FK raw',np.round(p,3),np.round(q,3))
p,q=a.hand_pose(); print('current FK raw',np.round(p,3))
# IK test in world coords: target the current hand pose exactly
sol=a.solve_ik(p,q); print('ik at current pose',None if sol is None else np.round(sol,3))
# IK pre-grasp with world coords
tcp=np.array([-0.109,0.057,1.02]); R=quat_R(Q_DOWN_FX); hand=tcp-TCP*R[:,2]
sol=a.solve_ik(hand,Q_DOWN_FX); print('ik pregrasp world',None if sol is None else np.round(sol,3))
if sol is not None: print('fk',np.round(a.hand_pose(sol)[0],3))
a.close()
EOF

# openrua op 31
cd /workspace; python3 - <<'EOF'
s=open('arm.py').read()
s=s.replace('''        # world -> panda_link0 offset (static)
        buf = Buffer(); TransformListener(buf, self.node)
        t0 = time.time()
        while time.time() - t0 < 10 and not buf.can_transform("world", "panda_link0", rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = buf.lookup_transform("world", "panda_link0", rclpy.time.Time()).transform.translation
        self.base = np.array([t.x, t.y, t.z])
''','''        # machine fact (verified): /compute_fk and /compute_ik already work in
        # WORLD coordinates (FK of the current pose == TF world->panda_hand)
        self.base = np.zeros(3)
''')
open('arm.py','w').write(s)
EOF
echo ok

# openrua op 32
cd /workspace; cat >> arm.py <<'EOF'


def line_ik(arm, tcp_from, tcp_to, quat, steps=4, seed=None, max_jump=0.6):
    """IK along a straight TCP line; returns list of joint waypoints (seeded
    consecutively) or raises if a solution is missing / jumps too far."""
    R = quat_R(quat)
    seed = list(seed if seed is not None else arm.arm_q())
    wps = []
    for i in range(1, steps + 1):
        tcp = np.asarray(tcp_from, float) + (np.asarray(tcp_to, float) - np.asarray(tcp_from, float)) * i / steps
        hand = tcp - TCP * R[:, 2]
        best = None
        for _ in range(6):
            q = arm.solve_ik(hand, quat, seed=seed, attempts=1)
            if q is None:
                continue
            jump = float(np.max(np.abs(np.array(q) - np.array(seed))))
            if best is None or jump < best[0]:
                best = (jump, q)
            if jump < max_jump:
                break
        if best is None:
            raise RuntimeError(f"IK failed at {np.round(tcp,3)}")
        if best[0] >= max_jump:
            raise RuntimeError(f"IK jump {best[0]:.2f} rad at {np.round(tcp,3)}")
        wps.append(best[1]); seed = best[1]
    return wps
EOF
timeout 600 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
q0=a.arm_q(); p0,_=a.hand_pose(); print('start hand',np.round(p0,3))
R=quat_R(Q_DOWN_FX)
tcp0=p0+TCP*R[:,2]  # current TCP approx (orientation differs but z same)
pre=np.array([-0.109,0.057,1.02])
wps=line_ik(a,tcp0,pre,Q_DOWN_FX,steps=3)
for w in wps: print(np.round(w,3))
code,err=a.move_joints(wps,4.0); print('code',code,'err',err)
p,q=a.hand_pose(); print('hand now',np.round(p,3),np.round(q,3),'tcp',np.round(p+quat_R(q)[:,2]*TCP,3))
a.close()
EOF

# openrua op 33
cd /workspace; timeout 600 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
R=quat_R(Q_DOWN_FX)
pre=np.array([-0.109,0.057,1.02])
hand=pre-TCP*R[:,2]
sol=a.solve_ik(hand,Q_DOWN_FX); print('sol',np.round(sol,3))
code,err=a.move_joints(sol,4.0); print('code',code,'err',err)
p,q=a.hand_pose(); print('hand now',np.round(p,3),np.round(q,3),'tcp',np.round(p+quat_R(q)[:,2]*TCP,3))
a.close()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 34
cd /workspace; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 35
cd /workspace; timeout 600 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
sol=[0.143,0.367,-0.039,-2.209,0.026,2.576,-1.486]
code,err=a.move_joints(sol,3.0); print('code',code,'err',err)
print(np.round(a.arm_q(),3))
p,q=a.hand_pose(); print('hand now',np.round(p,3),np.round(q,3))
a.close()
EOF

# openrua op 36
cd /workspace; timeout 600 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
sol=[0.143,0.367,-0.039,-2.209,0.026,2.576,-1.486]
code,err=a.move_joints(sol,8.0); print('code',code,'err',err)
print(np.round(a.arm_q(),3))
p,q=a.hand_pose(); print('hand now',np.round(p,3),np.round(q,3))
a.close()
EOF

# openrua op 37
cd /workspace; timeout 600 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
sol=[0.143,0.367,-0.039,-2.209,0.026,2.576,-1.486]
p,q=a.hand_pose(sol); print('fk(sol)',np.round(p,3),np.round(q,3))
p,q=a.hand_pose(); print('fk(current)',np.round(p,3),np.round(q,3), np.round(a.arm_q(),3))
for j7 in [-1.486, -0.7, 0.0, 0.785, 1.5]:
    s=list(sol); s[6]=j7; p,q=a.hand_pose(s); print('j7',j7,'->',np.round(q,3), 'finger axis', np.round(quat_R(q)[:,1],3))
a.close()
EOF

# openrua op 38
cd /workspace; python3 - <<'EOF'
s=open('arm.py').read()
s=s.replace('''            req.ik_request.group_name = M["planning"]["group"]
''','''            req.ik_request.group_name = M["planning"]["group"]
            # the group's default tip is panda_link8 (45 deg off panda_hand)
            req.ik_request.ik_link_name = "panda_hand"
''')
open('arm.py','w').write(s)
EOF
timeout 600 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
R=quat_R(Q_DOWN_FX)
pre=np.array([-0.109,0.057,1.02]); hand=pre-TCP*R[:,2]
sol=a.solve_ik(hand,Q_DOWN_FX); print('sol',np.round(sol,3))
p,q=a.hand_pose(sol); print('fk(sol)',np.round(p,3),np.round(q,3),'finger axis',np.round(quat_R(q)[:,1],3))
a.close()
EOF

# openrua op 39
cd /workspace; timeout 600 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
sol=[0.28,0.373,-0.172,-2.207,0.116,2.572,-0.763]
code,err=a.move_joints(sol,5.0); print('code',code,'err',err)
p,q=a.hand_pose(); print('hand',np.round(p,3),np.round(q,3),'finger axis',np.round(quat_R(q)[:,1],3),'fingers',a.fingers())
a.close()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 40
cd /workspace; timeout 120 python3 cloud.py robot0_eye_in_hand; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.903)&(Z<0.97)
pts=P[s]
print('bowl x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
r=pts[pts[:,2]>0.94]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center (%.3f,%.3f) radius %.3f'%(c[0],c[1],R))
# fingers: points with z>1.0 near the hand
s=(Z>0.97)&(Z<1.1)&(X>-0.25)&(X<0.05)&(Y>-0.05)&(Y<0.15)
if s.any():
  f=P[s]; print('finger pts x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(f[:,0].min(),f[:,0].max(),f[:,1].min(),f[:,1].max(),f[:,2].min(),f[:,2].max()))
"

# openrua op 41
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp',np.round(tcp,3),'force',np.round(a.force(),2))
target=np.array([-0.110,0.058,0.93])
wps=line_ik(a,tcp,target,Q_DOWN_FX,steps=3)
code,err=a.move_joints(wps,4.0); print('descend code',code,'err',err)
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp',np.round(tcp,3),'force',np.round(a.force(),2))
print('closing'); print(a.gripper(0.0))
print('fingers after',a.fingers(),'force',np.round(a.force(),2))
a.close()
EOF

# openrua op 42
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
a=Arm()
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP
target=np.array([-0.110,0.058,1.04])
wps=line_ik(a,tcp,target,Q_DOWN_FX,steps=2)
code,err=a.move_joints(wps,3.0); print('lift code',code,'err',err)
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp',np.round(tcp,3),'fingers',a.fingers(),'force',np.round(a.force(),2))
a.close()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 120 python3 cloud.py agentview; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.903)&(Z<1.2)
pts=P[s]
print('objects over bowl area: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.2,0.02)); print(' '.join('%.2f:%d'%(a,c) for c,a in zip(h,e)))
"

# openrua op 43
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py frontview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 44
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
"""Publish the measured scene (cabinet, drawer, table) as MoveIt collision
boxes and check joint configurations for collisions."""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive

from arm import JOINTS


def box(name, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(hi - lo))
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, (lo + hi) / 2)
    p.orientation.w = 1.0
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co


# measured world-frame boxes (see depth analysis)
BOXES = {
    "table": ([-0.7, -0.9, 0.80], [0.7, 0.9, 0.90]),
    "cabinet": ([-0.24, -0.46, 0.90], [0.03, -0.22, 1.127]),
    "top_handle": ([-0.15, -0.22, 1.08], [-0.06, -0.19, 1.11]),
    "drawer_floor": ([-0.215, -0.22, 0.90], [0.01, -0.065, 0.924]),
    "drawer_wall_xneg": ([-0.215, -0.22, 0.924], [-0.205, -0.105, 0.983]),
    "drawer_wall_xpos": ([0.0, -0.22, 0.924], [0.01, -0.105, 0.983]),
    "drawer_front": ([-0.215, -0.105, 0.924], [0.01, -0.065, 0.984]),
    "drawer_handle": ([-0.14, -0.065, 0.945], [-0.10, -0.04, 0.96]),
}


class Scene:
    def __init__(self, arm):
        self.arm = arm
        self.node = arm.node
        self.apply = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.apply.wait_for_service(10); self.valid.wait_for_service(10)

    def publish(self, boxes=BOXES):
        ps = PlanningScene(); ps.is_diff = True
        for name, (lo, hi) in boxes.items():
            ps.world.collision_objects.append(box(name, lo, hi))
        fut = self.apply.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success

    def check(self, q, fingers=None):
        req = GetStateValidity.Request()
        req.group_name = "panda_arm"
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if fingers is not None:
            req.robot_state.joint_state.name += ["panda_finger_joint1", "panda_finger_joint2"]
            req.robot_state.joint_state.position += [float(fingers), float(fingers)]
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None:
            return None, ["timeout"]
        contacts = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in res.contacts]
        return res.valid, contacts
OPENRUA_EOF

# openrua op 45
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene
a=Arm(); s=Scene(a)
print('publish',s.publish())
q=a.arm_q(); print('current valid',s.check(q), s.check(q,fingers=0.004))
# test: a config with hand deep inside the cabinet should be invalid
R=quat_R(Q_DOWN_FX); hand=np.array([-0.1,-0.3,1.0])-TCP*R[:,2]
qq=a.solve_ik(hand,Q_DOWN_FX)
print('inside-cabinet valid?', None if qq is None else s.check(qq))
# plan waypoints for placement
for tilt_deg in [0,10,15]:
    qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(tilt_deg)); Rt=quat_R(qt)
    for tcp in [[-0.063,-0.165,1.03],[-0.063,-0.165,0.965]]:
        hand=np.array(tcp)-TCP*Rt[:,2]
        sol=a.solve_ik(hand,qt)
        if sol is None: print(tilt_deg,tcp,'IK none'); continue
        v,c=s.check(sol); print('tilt',tilt_deg,'tcp',tcp,'valid',v,c,'hand',np.round(hand,3))
a.close()
EOF

# openrua op 46
cd /workspace; python3 - <<'EOF'
s=open('scene.py').read()
s=s.replace('''        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if fingers is not None:
            req.robot_state.joint_state.name += ["panda_finger_joint1", "panda_finger_joint2"]
            req.robot_state.joint_state.position += [float(fingers), float(fingers)]''','''        names = list(JOINTS); pos = [float(v) for v in q]
        if fingers is not None:
            names += ["panda_finger_joint1", "panda_finger_joint2"]
            pos += [float(fingers), float(fingers)]
        req.robot_state.joint_state.name = names
        req.robot_state.joint_state.position = pos''')
open('scene.py','w').write(s)
EOF
timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene
a=Arm(); s=Scene(a)
q=a.arm_q(); print('current valid',s.check(q), s.check(q,fingers=0.004))
R=quat_R(Q_DOWN_FX); hand=np.array([-0.1,-0.3,1.0])-TCP*R[:,2]
qq=a.solve_ik(hand,Q_DOWN_FX)
print('inside-cabinet valid?', None if qq is None else s.check(qq))
for tilt_deg in [0,10,15]:
    qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(tilt_deg)); Rt=quat_R(qt)
    for tcp in [[-0.063,-0.165,1.03],[-0.063,-0.165,0.965]]:
        hand=np.array(tcp)-TCP*Rt[:,2]
        sol=a.solve_ik(hand,qt)
        if sol is None: print(tilt_deg,tcp,'IK none'); continue
        v,c=s.check(sol,fingers=0.004); print('tilt',tilt_deg,'tcp',tcp,'valid',v,c,'hand',np.round(hand,3))
a.close()
EOF

# openrua op 47
cd /workspace; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.3)&(X<-0.05)&(Y>-0.03)&(Y<0.16)&(Z>0.95)&(Z<1.07)
pts=P[s]
for zlo in np.arange(0.99,1.07,0.01):
    q=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(q): print('z %.2f n %d x[%.3f,%.3f] y[%.3f,%.3f]'%(zlo,len(q),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
r=pts[pts[:,2]>1.045]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center (%.3f,%.3f) radius %.3f'%(c[0],c[1],R))
"

# openrua op 48
cd /workspace; timeout 120 python3 cloud.py birdview; python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    P=np.load(cam+'_cloud.npy'); X=P[...,0].ravel();Y=P[...,1].ravel();Z=P[...,2].ravel()
    print(cam,'y profile (x -0.19..-0.03), z<1.0:')
    for y in np.arange(-0.24,-0.03,0.005):
        s=(Y>=y)&(Y<y+0.005)&(X>-0.19)&(X<-0.03)&(Z>0.905)&(Z<1.0)
        if s.any(): print(' y %.3f zmax %.3f zmin %.3f n %d'%(y,Z[s].max(),Z[s].min(),s.sum()))
"

# openrua op 49
cd /workspace; python3 - <<'EOF'
s=open('scene.py').read()
s=s.replace('"drawer_wall_xneg": ([-0.215, -0.22, 0.924], [-0.205, -0.105, 0.983]),','"drawer_wall_xneg": ([-0.215, -0.22, 0.924], [-0.205, -0.09, 0.983]),')
s=s.replace('"drawer_wall_xpos": ([0.0, -0.22, 0.924], [0.01, -0.105, 0.983]),','"drawer_wall_xpos": ([0.0, -0.22, 0.924], [0.01, -0.09, 0.983]),')
s=s.replace('"drawer_front": ([-0.215, -0.105, 0.924], [0.01, -0.065, 0.984]),','"drawer_front": ([-0.215, -0.09, 0.924], [0.01, -0.065, 0.984]),')
open('scene.py','w').write(s)
EOF
timeout 1500 python3 -u - <<'EOF'
import numpy as np, time
from arm import *
from scene import Scene
a=Arm(); s=Scene(a); print('publish',s.publish())
qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(15)); Rt=quat_R(qt)
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp now',np.round(tcp,3))
def go(tcp_to, steps, secs, fingers, quat=qt, tcp_from=None):
    global tcp
    if tcp_from is None: tcp_from=tcp
    wps=line_ik(a,tcp_from,tcp_to,quat,steps=steps)
    for w in wps:
        v,c=s.check(w,fingers=fingers)
        if not v: raise SystemExit(f'collision predicted at {np.round(w,3)}: {c}')
    code,err=a.move_joints(wps,secs)
    p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP
    print('  -> code',code,'err %.4f'%err,'tcp',np.round(tcp,3),'fingers',np.round(a.fingers(),4))
    return code
# 1. tilt in place, slightly higher
print('tilt in place'); go([-0.109,0.058,1.05],2,4.0,0.004)
# 2. transit
print('transit'); go([-0.062,-0.152,1.05],4,6.0,0.004)
a.close()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 50
cd /workspace; timeout 120 python3 cloud.py agentview; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.25)&(X<0.02)&(Y>-0.25)&(Y<-0.03)&(Z>0.99)&(Z<1.075)
pts=P[s]
for zlo in np.arange(0.99,1.08,0.01):
    q=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(q): print('z %.2f n %d x[%.3f,%.3f] y[%.3f,%.3f]'%(zlo,len(q),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
"

# openrua op 51
cd /workspace; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.25)&(X<-0.0)&(Y>-0.218)&(Y<-0.03)&(Z>0.99)&(Z<1.075)
pts=P[s]
for zlo in np.arange(0.99,1.08,0.01):
    q=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(q): print('z %.2f n %d x[%.3f,%.3f] y[%.3f,%.3f]'%(zlo,len(q),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
r=pts[(pts[:,2]>1.03)&(pts[:,2]<1.065)&(pts[:,0]<-0.07)]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim-ish circle center (%.3f,%.3f) radius %.3f'%(c[0],c[1],R))
"

# openrua op 52
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 53
cd /workspace; timeout 1500 python3 -u - <<'EOF'
import numpy as np, time
from arm import *
from scene import Scene
a=Arm(); s=Scene(a)
qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(15))
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp now',np.round(tcp,3),'force',np.round(a.force(),2))
def go(tcp_to, steps, secs, fingers, quat=qt):
    global tcp
    wps=line_ik(a,tcp,tcp_to,quat,steps=steps)
    for w in wps:
        v,c=s.check(w,fingers=fingers)
        if not v: raise SystemExit(f'collision predicted at {np.round(w,3)}: {c}')
    code,err=a.move_joints(wps,secs)
    p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP
    print('  -> code',code,'err %.4f'%err,'tcp',np.round(tcp,3),'fingers',np.round(a.fingers(),4),'force',np.round(a.force(),2))
    return code
print('descend 1'); go([-0.062,-0.152,1.015],1,2.5,0.004)
print('descend 2'); go([-0.062,-0.152,0.985],1,2.5,0.004)
print('open'); print(a.gripper(0.04))
print('lift'); go([-0.062,-0.152,1.06],2,3.0,0.04)
a.close()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 54
cd /workspace; timeout 120 python3 cloud.py agentview; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.21)&(X<-0.0)&(Y>-0.218)&(Y<-0.09)&(Z>0.93)&(Z<1.0)
pts=P[s]
for zlo in np.arange(0.93,1.0,0.01):
    q=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(q): print('z %.2f n %d x[%.3f,%.3f] y[%.3f,%.3f]'%(zlo,len(q),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
r=pts[(pts[:,2]>0.965)]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center (%.3f,%.3f) radius %.3f zmean %.3f'%(c[0],c[1],R,r[:,2].mean()))
"

# openrua op 55
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene
a=Arm(); s=Scene(a)
qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(15))
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp now',np.round(tcp,3))
wps=line_ik(a,tcp,[-0.06,0.02,1.15],Q_DOWN_FX,steps=2,max_jump=1.0)
for w in wps: print(s.check(w,fingers=0.04))
print(a.move_joints(wps,4.0))
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp now',np.round(tcp,3))
a.close()
EOF
timeout 120 python3 cloud.py birdview; timeout 60 python3 tools/perception/cam_snap.py birdview; python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy'); X=P[...,0];Y=P[...,1];Z=P[...,2]
s=(X>-0.20)&(X<-0.005)&(Y>-0.215)&(Y<-0.095)&(Z>0.93)&(Z<1.0)
pts=P[s]
print('bowl pts x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
r=pts[(pts[:,2]>0.965)]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,0]**2+r[:,1]**2)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center (%.3f,%.3f) radius %.3f zmean %.3f n %d'%(c[0],c[1],R,r[:,2].mean(),len(r)))
"

# openrua op 56
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene, BOXES
a=Arm(); s=Scene(a)
static={k:v for k,v in BOXES.items() if not k.startswith('drawer')}
for tilt in [25,30,35,40]:
    qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(tilt)); Rt=quat_R(qt)
    res=[]
    for y in [-0.02,-0.10,-0.16,-0.20,-0.215]:
        hand=np.array([-0.16,y,0.955])-TCP*Rt[:,2]
        sol=a.solve_ik(hand,qt)
        if sol is None: res.append((y,'IK none')); continue
        # check against static scene only (drawer moves with the push)
        s.publish(static)
        # remove drawer objects by publishing... simpler: check full scene too
        v,c=s.check(sol,fingers=0.0)
        res.append((y,v,c))
    print('tilt',tilt,res)
a.close()
EOF

# openrua op 57
cd /workspace; cat >> scene.py <<'EOF'


def remove(scene, names):
    ps = PlanningScene(); ps.is_diff = True
    for n in names:
        co = CollisionObject(); co.header.frame_id = "world"; co.id = n
        co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)
    fut = scene.apply.call_async(ApplyPlanningScene.Request(scene=ps))
    rclpy.spin_until_future_complete(scene.node, fut, timeout_sec=30)
    return fut.result().success
EOF
timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene, BOXES, remove
a=Arm(); s=Scene(a)
print('remove drawer', remove(s,[k for k in BOXES if k.startswith('drawer')]))
for tilt in [20,25,30]:
    qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(tilt)); Rt=quat_R(qt)
    res=[]
    for y in [-0.02,-0.16,-0.19,-0.20,-0.21]:
        hand=np.array([-0.16,y,0.965])-TCP*Rt[:,2]
        sol=a.solve_ik(hand,qt)
        if sol is None: res.append((y,'IK none')); continue
        v,c=s.check(sol,fingers=0.0)
        res.append((y,v,c))
    print('tilt',tilt,res)
a.close()
EOF

# openrua op 58
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene, BOXES, remove
a=Arm(); s=Scene(a)
walls={k:v for k,v in BOXES.items() if k in ('drawer_wall_xneg','drawer_wall_xpos','drawer_floor')}
print('publish walls',s.publish(walls))
for tilt in [25,30,35]:
  for z in [0.965,0.975]:
    qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(tilt)); Rt=quat_R(qt)
    res=[]
    for y in [-0.02,-0.12,-0.16,-0.19,-0.20]:
        hand=np.array([-0.16,y,z])-TCP*Rt[:,2]
        sol=a.solve_ik(hand,qt)
        if sol is None: res.append((y,'IK none')); continue
        v,c=s.check(sol,fingers=0.0)
        res.append((y,v,c))
    print('tilt',tilt,'z',z,res)
a.close()
EOF

# openrua op 59
cd /workspace; timeout 1700 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene
a=Arm(); s=Scene(a)
qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(35)); Rt=quat_R(qt)
print('close gripper',a.gripper(0.0))
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp now',np.round(tcp,3))
def go(tcp_to, steps, secs, fingers=0.0, quat=qt, max_jump=0.6):
    global tcp
    wps=line_ik(a,tcp,tcp_to,quat,steps=steps,max_jump=max_jump)
    for w in wps:
        v,c=s.check(w,fingers=fingers)
        if not v: raise SystemExit(f'collision predicted at {np.round(w,3)}: {c}')
    code,err=a.move_joints(wps,secs)
    p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP
    f=a.force()
    print('  -> code',code,'err %.4f'%err,'tcp',np.round(tcp,3),'force',np.round(f,2))
    return code
print('pre-push'); go([-0.16,-0.02,1.03],2,5.0,max_jump=1.5)
print('descend'); go([-0.16,-0.02,0.97],1,2.5)
f0=a.force()
for y in [-0.07,-0.10,-0.13,-0.16,-0.18,-0.195,-0.205]:
    print('push to',y)
    go([-0.16,y,0.97],1,2.0)
    lag=tcp[1]-y
    print('   lag %.4f'%lag)
    if lag>0.012:
        print('   stalled -> drawer stop reached'); break
a.close()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 60
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np
from arm import *
from scene import Scene
a=Arm(); s=Scene(a)
qt=tilted(Q_DOWN_FX,[1,0,0],-np.radians(35))
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp now',np.round(tcp,3))
# back off +y then up
wps=line_ik(a,tcp,[-0.16,-0.10,1.0],qt,steps=2)
print(a.move_joints(wps,3.0))
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP
wps=line_ik(a,tcp,[-0.10,0.10,1.20],Q_DOWN_FX,steps=2,max_jump=1.5)
print(a.move_joints(wps,5.0))
p,q=a.hand_pose(); tcp=p+quat_R(q)[:,2]*TCP; print('tcp now',np.round(tcp,3),'force',np.round(a.force(),2))
a.close()
EOF
for c in agentview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 120 python3 cloud.py agentview; timeout 120 python3 cloud.py birdview

# openrua op 61
cd /workspace; python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy'); X=P[...,0].ravel();Y=P[...,1].ravel();Z=P[...,2].ravel()
# cabinet +y face profile: for z bands, the max y of cabinet points (x in [-0.19,-0.05])
print('cabinet front face (max y where surface exists) by height band:')
for zlo in np.arange(0.92,1.13,0.02):
    s=(X>-0.19)&(X<-0.05)&(Y<-0.10)&(Y>-0.30)&(Z>=zlo)&(Z<zlo+0.02)
    if s.any(): print('  z %.2f..: y max %.3f (n %d)'%(zlo,Y[s].max(),s.sum()))
# handles: anything with y > -0.20 in the cabinet x range, z<1.13
s=(X>-0.19)&(X<-0.05)&(Y>-0.20)&(Y<-0.12)&(Z>0.905)&(Z<1.13)
print('protrusions beyond y=-0.20: n',s.sum(), 'y[%.3f,%.3f] z[%.3f,%.3f]'%(Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max()) if s.any() else '')
# any bowl-like object on the table outside the cabinet?
s=(Y>-0.10)&(Z>0.93)&(Z<1.0)&(X>-0.4)&(X<0.4)&(Y<0.5)
print('objects on table 0.93<z<1.0, y>-0.10: n',s.sum(), 'x[%.3f,%.3f] y[%.3f,%.3f]'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max()) if s.any() else '')
P=np.load('birdview_cloud.npy'); X=P[...,0].ravel();Y=P[...,1].ravel();Z=P[...,2].ravel()
print('birdview y profile at x in [-0.19,-0.05]:')
for y in np.arange(-0.25,-0.10,0.005):
    s=(Y>=y)&(Y<y+0.005)&(X>-0.19)&(X<-0.05)
    if s.any(): print('  y %.3f zmax %.3f'%(y,Z[s].max()))
"

# openrua op 62
cd /workspace; grep -o 'tilted(Q_DOWN_F[XY][^)]*)' /claude-config/projects/-workspace/d6dbe117-3fe4-4003-93e3-4daecb3f52b1.jsonl | sort | uniq -c | tail; grep -o 'push to[^"]{0,200}' /claude-config/projects/-workspace/d6dbe117-3fe4-4003-93e3-4daecb3f52b1.jsonl | tail -5

# openrua op 63
cd /workspace; grep -o '"text":"[^"]*push[^"]{0,300}' /claude-config/projects/-workspace/d6dbe117-3fe4-4003-93e3-4daecb3f52b1.jsonl | tail -6

# openrua op 64
cd /workspace; grep -o 'push to -0[^\\]{0,200}' /claude-config/projects/-workspace/d6dbe117-3fe4-4003-93e3-4daecb3f52b1.jsonl | tail -8; ls *.py

# openrua op 65
cd /workspace; grep -o 'tcp \[-0.16 -0.[0-9]* 0.97\] force[^\\]{0,120}' /claude-config/projects/-workspace/d6dbe117-3fe4-4003-93e3-4daecb3f52b1.jsonl | tail -8

# openrua op 66
cd /workspace; cat > push2.py <<'EOF'
import numpy as np, math
from arm import Arm, Q_DOWN_FX, tilted, TCP, quat_R, line_ik
a = Arm()
q = tilted(Q_DOWN_FX, [1,0,0], -np.radians(35))
R = quat_R(q)
def tcp():
    p, _ = a.hand_pose(); return p + TCP*R[:,2]
print("fingers", a.fingers(), "tcp", np.round(tcp(),3))
# approach above then in front of the drawer
for pos, sec in [((-0.16, -0.12, 1.10), 5), ((-0.16, -0.15, 0.97), 4)]:
    _, code, err = a.move_tcp(pos, q, seconds=sec)
    print("to", pos, "code", code, "err", round(err,4), "tcp", np.round(tcp(),3), "F", np.round(a.force(),2))
# push in small steps, watch progress
for y in [-0.19, -0.205, -0.215, -0.225]:
    wps = line_ik(a, tcp(), (-0.16, y, 0.97), q, steps=2, max_jump=0.5)
    code, err = a.move_joints(wps, 3)
    t = tcp(); F = a.force()
    print("push to", y, "code", code, "lag", round(err,4), "tcp", np.round(t,3), "F", np.round(F,2))
    if t[1] > y + 0.004:  # not following -> at the stop
        print("stopped short: drawer at its limit"); break
# retract
_, code, err = a.move_tcp((-0.16, -0.12, 1.10), q, seconds=4)
_, code, err = a.move_tcp((-0.10, 0.10, 1.20), q, seconds=5)
print("parked tcp", np.round(tcp(),3), "code", code, "err", round(err,4))
a.close()
EOF
python3 push2.py 2>&1 | grep -v '^\[' 

# openrua op 67
cd /workspace; python3 tools/perception/cam_snap.py frontview frontview.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py agentview agentview.png 2>&1 | tail -1
