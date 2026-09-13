#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | head -50 && echo ---NODES--- && ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
ros2 topic echo /tf_static --once --qos-durability transient_local 2>&1 | grep -E "frame_id|translation|rotation|x:|y:|z:|w:" | head -80; echo ---; ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1023}}" 2>&1 | grep -E "id:|frame_id|position|x:|y:|z:" | head -60

# openrua op 6
cat /tmp/claude-1000/-workspace/d913cfab-b7fc-4388-9670-1d9222516a94/tasks/b8ketlxct.output | head -50

# openrua op 7
mkdir -p "$(dirname /workspace/tf_dump.py)"
cat > /workspace/tf_dump.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump TF frames and selected transforms relative to world."""
import sys
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("tf_dump")
buf = Buffer()
TransformListener(buf, node)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = buf.all_frames_as_yaml()
print(frames)
targets = sys.argv[1:] or ["panda_link0", "panda_hand", "agentview_optical_frame",
                           "frontview_optical_frame", "birdview_optical_frame",
                           "sideview_optical_frame", "robot0_robotview_optical_frame",
                           "robot0_eye_in_hand_optical_frame"]
for f in targets:
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"world->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"world->{f}: FAIL {type(e).__name__}: {e}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 tf_dump.py 2>&1 | tail -60

# openrua op 9
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+color+intrinsics+TF for a camera, save a world-frame point cloud
as <cam>_cloud.npz (xyz world, rgb, pixel uv). Usage: cloud.py <cam>"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def grab(node, topic, mt):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
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
    buf = Buffer()
    TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "rgb8")
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q, tr = t.transform.rotation, t.transform.translation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([tr.x, tr.y, tr.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    z = depth
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    pc = np.stack([X, Y, z], -1).reshape(-1, 3)
    pw = pc @ R.T + T
    valid = np.isfinite(z).reshape(-1) & (z.reshape(-1) > 0)
    np.savez(f"{cam}_cloud.npz", xyz=pw.reshape(H, W, 3), rgb=color, valid=valid.reshape(H, W))
    print(f"saved {cam}_cloud.npz  H={H} W={W} fx={fx:.1f} cx={cx:.1f} cy={cy:.1f}")
    print(f"depth range {np.nanmin(z):.3f}..{np.nanmax(z):.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py sideview

# openrua op 11
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']; valid=d['valid']
z=xyz[...,2]
# table height: mode of z in the table region
import collections
zs=np.round(z[valid],3)
vals,counts=np.unique(zs,return_counts=True)
print('top z modes:', [(vals[i],counts[i]) for i in np.argsort(-counts)[:8]])
# heights histogram above table
table=vals[np.argmax(counts)]
print('table z', table)
mask=(z>table+0.02)&valid
print('above-table px', mask.sum())
# cluster by connected components
import cv2
m=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if stats[i,4]<30: continue
    sel=lab==i
    p=xyz[sel]; c=rgb[sel].mean(0)
    print(f'comp {i}: px={stats[i,4]} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] rgb={c.astype(int)} centroid_uv={cent[i]}')
"


# openrua op 12
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']; valid=d['valid']
z=xyz[...,2]
table=0.9
mask=(z>table+0.015)&(z<1.5)&valid
m=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if stats[i,4]<30: continue
    sel=lab==i
    p=xyz[sel]; c=rgb[sel].mean(0)
    print(f'comp {i}: px={stats[i,4]} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] rgb={c.astype(int)} uv={cent[i].round(0)}')
# table extent
tm=(np.abs(z-0.9)<0.005)&valid
p=xyz[tm]; print('table x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max())
"


# openrua op 13
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']; valid=d['valid']
z=xyz[...,2]
for lo,hi in [(0.92,1.0),(1.0,1.1),(1.1,1.2),(1.2,1.3),(1.3,1.4),(1.4,1.55)]:
    m=(z>lo)&(z<=hi)&valid&(xyz[...,1]>-0.15)&(xyz[...,0]>-0.45)&(xyz[...,0]<0.3)
    p=xyz[m]
    if len(p)==0: print(lo,hi,'none'); continue
    print(f'z({lo},{hi}] n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}]')
"


# openrua op 14
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']; valid=d['valid']
z=xyz[...,2]
# print z along image row 300 (through microwave body) and row 245 (door)
for row in [240,245,250,260,300,340]:
    print('row',row, 'x_world=%.3f'%xyz[row,320,0])
    print('  ', ' '.join(f'{z[row,c]:.2f}' for c in range(300,480,6)))
print('cols:', ' '.join(f'{c}' for c in range(300,480,6)))
print('y_world at cols:', ' '.join(f'{xyz[300,c,1]:.2f}' for c in range(300,480,6)))
"


# openrua op 15
timeout 60 python3 tools/perception/cam_snap.py galleryview; timeout 60 python3 tools/perception/cam_snap.py paperview; timeout 60 python3 tf_dump.py galleryview_optical_frame paperview_optical_frame 2>&1 | tail -2

# openrua op 16
python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; valid=d['valid']
    m=valid&(xyz[...,0]>-0.25)&(xyz[...,0]<0.2)&(xyz[...,2]>0.905)&(xyz[...,2]<1.105)&(xyz[...,1]>0.0)&(xyz[...,1]<0.6)
    p=xyz[m]
    print(cam,'n',len(p))
    h,e=np.histogram(p[:,1],bins=np.arange(0.0,0.62,0.02))
    for c,lo in zip(h,e[:-1]):
        if c: print(f'  y[{lo:.2f},{lo+0.02:.2f}) n={c}  x[{p[(p[:,1]>=lo)&(p[:,1]<lo+0.02),0].min():.3f},{p[(p[:,1]>=lo)&(p[:,1]<lo+0.02),0].max():.3f}]')
"


# openrua op 17
python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; valid=d['valid']; rgb=d['rgb']
m=valid&(xyz[...,0]>-0.25)&(xyz[...,0]<0.15)&(xyz[...,2]>0.905)&(xyz[...,2]<1.105)&(xyz[...,1]>0.2)&(xyz[...,1]<0.6)
p=xyz[m]
print('n',len(p))
h,ex,ey=np.histogram2d(p[:,1],p[:,0],bins=[np.arange(0.2,0.5,0.02),np.arange(-0.25,0.16,0.05)])
print('rows y, cols x:',ex.round(2),ey.round(2))
print(h.astype(int))
# z histogram of those with y in 0.26-0.34
q=p[(p[:,1]>0.26)&(p[:,1]<0.34)]
print(np.histogram(q[:,2],bins=np.arange(0.9,1.12,0.02)))
# pixel locations
vs,us=np.nonzero(m&(xyz[...,1]>0.26)&(xyz[...,1]<0.34))
print('u range',us.min(),us.max(),'v range',vs.min(),vs.max())
"


# openrua op 18
python3 -c "
import numpy as np
for cam in ['agentview','sideview','birdview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; valid=d['valid']; rgb=d['rgb']
    # white mug region
    m=valid&(xyz[...,0]>-0.22)&(xyz[...,0]<-0.03)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.15)&(xyz[...,2]>0.905)
    p=xyz[m]
    print(cam,'white mug n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
    h,e=np.histogram(p[:,2],bins=np.arange(0.9,1.12,0.01)); print('   z hist',list(zip(e[:-1].round(2),h)))
    # yellow mug
    m=valid&(xyz[...,0]>-0.08)&(xyz[...,0]<0.07)&(xyz[...,1]>-0.12)&(xyz[...,1]<0.07)&(xyz[...,2]>0.905)&(xyz[...,2]<1.2)
    p=xyz[m]
    print(cam,'yellow mug n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
"


# openrua op 19
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 20
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; valid=d['valid']
    m=valid&(xyz[...,0]>-0.22)&(xyz[...,0]<-0.03)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.15)&(xyz[...,2]>0.905)
    p=xyz[m]
    print(cam,'white mug x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
    m=valid&(xyz[...,0]>-0.08)&(xyz[...,0]<0.07)&(xyz[...,1]>-0.12)&(xyz[...,1]<0.07)&(xyz[...,2]>0.905)&(xyz[...,2]<1.2)
    p=xyz[m]
    print(cam,'yellow mug x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
"

# openrua op 21
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; valid=d['valid']; rgb=d['rgb']
m=valid&(xyz[...,0]>-0.22)&(xyz[...,0]<-0.03)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.15)&(xyz[...,2]>0.905)
p=xyz[m]
# rim: top points
rim=p[p[:,2]>1.0]
print('rim x[%.3f,%.3f] y[%.3f,%.3f] n=%d'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max(),len(rim)))
cx,cy=(rim[:,0].min()+rim[:,0].max())/2,(rim[:,1].min()+rim[:,1].max())/2
print('rim center guess',cx,cy)
# handle: points at lower z (handle top lower than rim?) 
for lo in np.arange(0.90,1.02,0.01):
    q=p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(q): print(f'z[{lo:.2f}] n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,0]>-0.22)&(xyz[...,0]<-0.03)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.15)&(xyz[...,2]>0.905)
p=xyz[m]
print('agentview profile (sees +x side):')
for lo in np.arange(0.90,1.02,0.01):
    q=p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(q): print(f'z[{lo:.2f}] n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
"


# openrua op 22
python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
# front frame region: y in [0.22,0.27], x in [-0.14,0.04]
m=valid&(xyz[...,0]>-0.14)&(xyz[...,0]<0.04)&(xyz[...,1]>0.22)&(xyz[...,1]<0.27)
p=xyz[m]
print('front-plane points z hist:')
h,e=np.histogram(p[:,2],bins=np.arange(0.9,1.13,0.01))
for c,lo in zip(h,e[:-1]): print(f'  z[{lo:.2f}] {c}')
# interior: y in [0.27,0.45], x in [-0.14,0.04]
m=valid&(xyz[...,0]>-0.14)&(xyz[...,0]<0.04)&(xyz[...,1]>0.27)&(xyz[...,1]<0.46)
p=xyz[m]
print('interior points: n',len(p))
h,e=np.histogram(p[:,2],bins=np.arange(0.9,1.13,0.01))
for c,lo in zip(h,e[:-1]): 
    if c: q=p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]; print(f'  z[{lo:.2f}] {c}  x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
"


# openrua op 23
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,0]>-0.22)&(xyz[...,0]<-0.03)&(xyz[...,1]>-0.205)&(xyz[...,1]<-0.15)&(xyz[...,2]>0.905)
p=xyz[m]
print('handle (birdview) n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
for lo in np.arange(-0.205,-0.165,0.005):
    q=p[(p[:,1]>=lo)&(p[:,1]<lo+0.005)]
    if len(q): print(f'  y[{lo:.3f}] n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]')
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,0]>-0.22)&(xyz[...,0]<-0.03)&(xyz[...,1]>-0.205)&(xyz[...,1]<-0.15)&(xyz[...,2]>0.905)
p=xyz[m]
print('handle (agentview) n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
for lo in np.arange(0.90,1.0,0.01):
    q=p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(q): print(f'  z[{lo:.2f}] n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
"


# openrua op 24
python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,1]>0.2)&(xyz[...,1]<0.28)&(xyz[...,2]>0.96)&(xyz[...,2]<1.06)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.2)
p=xyz[m]
print('front plane mid-height points by x:')
h,e=np.histogram(p[:,0],bins=np.arange(-0.3,0.2,0.01))
for c,lo in zip(h,e[:-1]):
    if c: q=p[(p[:,0]>=lo)&(p[:,0]<lo+0.01)]; print(f'  x[{lo:.2f}] n={c} y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
# top bar over opening
m=valid&(xyz[...,1]>0.2)&(xyz[...,1]<0.28)&(xyz[...,2]>1.07)&(xyz[...,2]<1.115)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.2)
p=xyz[m]; print('top bar x[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,2].min(),p[:,2].max()))
# door extents
m=valid&(xyz[...,1]>-0.05)&(xyz[...,1]<0.24)&(xyz[...,2]>0.91)&(xyz[...,0]>-0.3)&(xyz[...,0]<-0.1)
p=xyz[m]; print('door x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
for lo in np.arange(-0.05,0.25,0.05):
    q=p[(p[:,1]>=lo)&(p[:,1]<lo+0.05)]
    if len(q): print(f'  y[{lo:.2f}] x[{q[:,0].min():.3f},{q[:,0].max():.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]')
"


# openrua op 25
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,2]>1.0)&(xyz[...,2]<1.115)&(xyz[...,0]>-0.25)&(xyz[...,0]<-0.10)&(xyz[...,1]>-0.15)&(xyz[...,1]<0.26)
p=xyz[m]
print('door-top points n',len(p))
for lo in np.arange(-0.15,0.26,0.02):
    q=p[(p[:,1]>=lo)&(p[:,1]<lo+0.02)]
    if len(q): print(f'  y[{lo:.2f}] n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]')
# fit line
A=np.c_[p[:,1],np.ones(len(p))]; k,b=np.linalg.lstsq(A,p[:,0],rcond=None)[0]
print('x = %.3f*y + %.3f'%(k,b))
"


# openrua op 26
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control library for this Panda: joint read, FK/IK (MoveIt), multi-point
trajectories, gripper.  Poses are given in WORLD frame and converted to the arm
base (planner model frame) internally.  Import and use from scripts."""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])   # world->panda_link0 (measured, TF)
TCP = M["hand"]["tcp_offset_m"]


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def R_to_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_from_axes(x_hand=None, y_hand=None, z_hand=None):
    """Build hand rotation from two of its axes given in world coords."""
    if z_hand is not None and y_hand is not None:
        z = np.asarray(z_hand, float); z /= np.linalg.norm(z)
        y = np.asarray(y_hand, float); y -= z * (y @ z); y /= np.linalg.norm(y)
        x = np.cross(y, z)
    elif z_hand is not None and x_hand is not None:
        z = np.asarray(z_hand, float); z /= np.linalg.norm(z)
        x = np.asarray(x_hand, float); x -= z * (x @ z); x /= np.linalg.norm(x)
        y = np.cross(z, x)
    else:
        raise ValueError
    return np.stack([x, y, z], axis=1)


def R_down(yaw=0.0):
    """Hand pointing straight down; fingers open along world y rotated by yaw."""
    z = np.array([0, 0, -1.0])
    y = np.array([-math.sin(yaw), -math.cos(yaw), 0.0])  # yaw=0: fingers along -y (panda default-ish)
    return R_from_axes(z_hand=z, y_hand=y)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        """dict name->position (all joints)."""
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self, fresh=True):
        j = self.joints(fresh)
        return np.array([j[n] for n in ARM])

    def finger_gap(self, fresh=True):
        j = self.joints(fresh)
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand", timeout=60):
        """World pose (p, quat) of link for arm joints q (default: current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        q_ = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q_

    def tcp(self, q=None):
        """World position of the TCP (fingertip centre)."""
        p, quat = self.fk(q)
        R = quat_to_R(quat)
        return p + TCP * R[:, 2], R

    # ---------------- planning ----------------
    def ik(self, p_world, R, seed=None, at_tcp=True, timeout=60, attempts=3):
        """Joint solution for hand pose. p_world is the TCP (fingertip) point if
        at_tcp else the hand frame origin. Returns np.array or None."""
        p = np.asarray(p_world, float)
        if at_tcp:
            p = p - TCP * R[:, 2]
        pb = p - BASE_IN_WORLD
        quat = R_to_quat(R)
        if seed is None:
            seed = self.arm_q()
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            ps = req.ik_request.pose_stamped.pose
            ps.position.x, ps.position.y, ps.position.z = map(float, pb)
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            s = np.asarray(seed, float)
            if k > 0:  # perturb seed on retry
                s = s + np.random.uniform(-0.3, 0.3, size=7)
                s = np.clip(s, [l[0] + 0.05 for l in LIMITS], [l[1] - 0.05 for l in LIMITS])
            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=2)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is None:
                log("IK: no answer")
                continue
            if res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[n] for n in ARM])
            log(f"IK attempt {k}: error {res.error_code.val}")
        return None

    # ---------------- acting ----------------
    def move_joints(self, waypoints, durations, timeout=900):
        """Send one trajectory through joint waypoints; durations are cumulative
        times (s). Returns error_code."""
        if not self.traj.wait_for_server(timeout_sec=10):
            raise RuntimeError("no trajectory server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(waypoints, durations):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if rf.result() is None:
            log("trajectory: no result within timeout")
            return None
        code = rf.result().result.error_code
        q_now = self.arm_q()
        err = np.abs(q_now - np.asarray(waypoints[-1])).max()
        log(f"trajectory done code={code} max_joint_err={err:.4f}")
        return code

    def move_to_pose(self, p_world, R, seconds=4.0, seed=None, at_tcp=True):
        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {p_world}")
        code = self.move_joints([q], [seconds])
        return q, code

    def gripper(self, width, timeout=300):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        log(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("arm q:", np.round(q, 4))
    print("finger gap:", r.finger_gap())
    p, quat = r.fk(q)
    print("hand world pos:", np.round(p, 4), "quat:", np.round(quat, 4))
    t, R = r.tcp(q)
    print("tcp world:", np.round(t, 4))
    print("hand axes (cols x,y,z):\n", np.round(R, 3))
OPENRUA_EOF

# openrua op 27
timeout 120 python3 rob.py 2>&1 | tail -20

# openrua op 28
timeout 200 python3 -c "
import numpy as np, rclpy
from rob import *
r=Robot()
q=r.arm_q()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
r.fk_cli.wait_for_service(timeout_sec=10)
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK round trip: pose in world coords, no base offset
BASE0=BASE_IN_WORLD.copy()
import rob
rob.BASE_IN_WORLD[:]=0  # treat planner frame as world
p=np.array([-0.203,0.0,1.2696]); R=quat_to_R([0.9996,0,-0.0284,0])
sol=r.ik(p,R,seed=q,at_tcp=False)
print('IK (world coords) sol:', None if sol is None else np.round(sol,4))
rob.BASE_IN_WORLD[:]=BASE0
sol2=r.ik(p,R,seed=q,at_tcp=False)
print('IK (base-offset coords) sol:', None if sol2 is None else np.round(sol2,4))
" 2>&1 | tail -12

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])   # world->panda_link0 (measured, TF)", "new_string": "# Verified via /compute_fk: the planner\'s model frame IS `world` on this machine\\n# (panda_link0 sits at (-0.66, 0, 0.912) in it), so no offset is applied.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        if not self.ik_cli.wait_for_service(timeout_sec=10):\\n            raise RuntimeError(\\"no IK service\\")\\n        for k in range(attempts):", "new_string": "        if not self.ik_cli.wait_for_service(timeout_sec=10):\\n            raise RuntimeError(\\"no IK service\\")\\n        best = None\\n        for k in range(attempts):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 31
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "            if res.error_code.val == 1:\\n                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                return np.array([sol[n] for n in ARM])\\n            log(f\\"IK attempt {k}: error {res.error_code.val}\\")\\n        return None", "new_string": "            if res.error_code.val == 1:\\n                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                qs = np.array([sol[n] for n in ARM])\\n                dev = np.abs(qs - np.asarray(seed, float)).max()\\n                if best is None or dev < best[0]:\\n                    best = (dev, qs)\\n                if dev < max_dev:  # close enough to the seed: take it\\n                    return qs\\n                log(f\\"IK attempt {k}: solution deviates {dev:.2f} rad from seed\\")\\n                continue\\n            log(f\\"IK attempt {k}: error {res.error_code.val}\\")\\n        if best is not None:\\n            log(f\\"IK: returning best solution (dev {best[0]:.2f})\\")\\n            return best[1]\\n        return None", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 32
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def ik(self, p_world, R, seed=None, at_tcp=True, timeout=60, attempts=3):", "new_string": "    def ik(self, p_world, R, seed=None, at_tcp=True, timeout=60, attempts=4, max_dev=0.8):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 33
mkdir -p "$(dirname /workspace/phase1_rotate_mug.py)"
cat > /workspace/phase1_rotate_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 1: pinch white mug rim from above, lift, rotate 180 deg (joint 7),
place at staging spot with handle facing -y, release, retreat."""
import math
import sys
import numpy as np
from rob import *

M_CENTER = np.array([-0.1225, -0.2525])   # white mug centre (world xy)
RIM_Z = 1.012
S_CENTER = np.array([-0.20, -0.15])       # staging spot for mug centre
WALL_OFF = 0.044                          # TCP sits on the -y wall (radius 0.0465 - half wall)

r = Robot("phase1")
q0 = r.arm_q()
log("start q", np.round(q0, 3), "gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.07:
    r.gripper(0.04)

R = R_down(0.0)                           # fingers open along world y
tcp_xy = M_CENTER + np.array([0.0, -WALL_OFF])
above = np.array([*tcp_xy, 1.12])
grasp = np.array([*tcp_xy, RIM_Z - 0.022])

log("move above mug", above)
q1, code = r.move_to_pose(above, R, seconds=4.0, seed=q0)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))

log("descend to grasp", grasp)
q2, code = r.move_to_pose(grasp, R, seconds=3.0, seed=q1)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))

gap = r.gripper(0.0)
log("closed; gap", round(gap, 4))
if gap < 0.003:
    log("!! gripper closed on air, abort"); sys.exit(2)

lift = np.array([*tcp_xy, 1.13])
log("lift", lift)
q3, code = r.move_to_pose(lift, R, seconds=3.0, seed=q2)
t, _ = r.tcp(); log("tcp now", np.round(t, 4), "gap", round(r.finger_gap(), 4))

# rotate joint 7 by pi within limits
q4 = q3.copy()
lo, hi = LIMITS[6]
if q4[6] + math.pi <= hi - 0.05:
    q4[6] += math.pi
elif q4[6] - math.pi >= lo + 0.05:
    q4[6] -= math.pi
else:
    log("!! cannot rotate joint7 by pi from", q4[6]); sys.exit(3)
log("rotate j7 ->", round(q4[6], 3))
r.move_joints([q4], [4.0])
t, Rn = r.tcp(); log("tcp now", np.round(t, 4), "y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4))

# after rotation the mug centre is TCP + (0, -WALL_OFF); place at S
R2 = Rn.copy()
tcp2 = S_CENTER + np.array([0.0, +WALL_OFF])
over = np.array([*tcp2, 1.13])
log("move over staging", over)
q5, code = r.move_to_pose(over, R2, seconds=4.0, seed=q4)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))
down = np.array([*tcp2, RIM_Z - 0.022 + 0.004])
log("lower", down)
q6, code = r.move_to_pose(down, R2, seconds=3.0, seed=q5)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))
r.gripper(0.04)
up = np.array([*tcp2, 1.15])
q7, code = r.move_to_pose(up, R2, seconds=3.0, seed=q6)
t, _ = r.tcp(); log("retreated; tcp now", np.round(t, 4))
log("PHASE1 DONE")
OPENRUA_EOF

# openrua op 34
nohup python3 -u phase1_rotate_mug.py > phase1.log 2>&1 &
sleep 45; cat /workspace/phase1.log

# openrua op 35
sleep 60; cat /workspace/phase1.log | tail -12

# openrua op 36
cat /workspace/phase1.log | sed -n 18,40p; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 37
timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,0]>-0.30)&(xyz[...,0]<-0.10)&(xyz[...,1]>-0.20)&(xyz[...,1]<0.02)&(xyz[...,2]>0.905)&(xyz[...,2]<1.05)
p=xyz[m]
print('mug region n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
rim=p[p[:,2]>1.0]
print('rim x[%.3f,%.3f] y[%.3f,%.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()))
cx=(rim[:,0].min()+rim[:,0].max())/2; cy=(rim[:,1].min()+rim[:,1].max())/2
print('centre',round(cx,4),round(cy,4))
h=p[(p[:,2]>0.92)&(p[:,2]<0.995)]
# handle = points outside radius 0.05 from centre
rr=np.hypot(h[:,0]-cx,h[:,1]-cy); hh=h[rr>0.05]
print('handle pts n',len(hh), 'mean offset', np.round([ (hh[:,0]-cx).mean(), (hh[:,1]-cy).mean()],3) if len(hh) else None)
"

# openrua op 38
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[200:280,260:340]; crop=cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST); cv2.imwrite('crop_bird.png',crop)
im=cv2.imread('agentview.png'); crop=im[160:260,230:340]; crop=cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop_agent.png',crop)
"

# openrua op 39
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_joints(self, waypoints, durations, timeout=900):\\n        \\"\\"\\"Send one trajectory through joint waypoints; durations are cumulative\\n        times (s). Returns error_code.\\"\\"\\"\\n        if not self.traj.wait_for_server(timeout_sec=10):\\n            raise RuntimeError(\\"no trajectory server\\")\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(ARM)\\n        for q, t in zip(waypoints, durations):\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in q])\\n            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n            goal.trajectory.points.append(pt)\\n        fut = self.traj.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)\\n        gh = fut.result()\\n        if gh is None or not gh.accepted:\\n            raise RuntimeError(\\"trajectory goal rejected\\")\\n        rf = gh.get_result_async()\\n        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)\\n        if rf.result() is None:\\n            log(\\"trajectory: no result within timeout\\")\\n            return None\\n        code = rf.result().result.error_code\\n        q_now = self.arm_q()\\n        err = np.abs(q_now - np.asarray(waypoints[-1])).max()\\n        log(f\\"trajectory done code={code} max_joint_err={err:.4f}\\")\\n        return code\\n\\n    def move_to_pose(self, p_world, R, seconds=4.0, seed=None, at_tcp=True):\\n        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp)\\n        if q is None:\\n            raise RuntimeError(f\\"IK failed for {p_world}\\")\\n        code = self.move_joints([q], [seconds])\\n        return q, code", "new_string": "    def _send_traj(self, waypoints, durations, timeout=900):\\n        if not self.traj.wait_for_server(timeout_sec=10):\\n            raise RuntimeError(\\"no trajectory server\\")\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(ARM)\\n        for q, t in zip(waypoints, durations):\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in q])\\n            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n            goal.trajectory.points.append(pt)\\n        fut = self.traj.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)\\n        gh = fut.result()\\n        if gh is None or not gh.accepted:\\n            raise RuntimeError(\\"trajectory goal rejected\\")\\n        rf = gh.get_result_async()\\n        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)\\n        if rf.result() is None:\\n            log(\\"trajectory: no result within timeout\\")\\n            return None\\n        return rf.result().result.error_code\\n\\n    def settle(self, target, tol=0.01, max_iter=20):\\n        \\"\\"\\"Poll joints until they stop changing; return (err, q).\\"\\"\\"\\n        prev = self.arm_q()\\n        for _ in range(max_iter):\\n            q = self.arm_q()\\n            if np.abs(q - prev).max() < 1e-4:\\n                break\\n            prev = q\\n        err = np.abs(q - np.asarray(target)).max()\\n        return err, q\\n\\n    def move_joints(self, waypoints, durations=None, speed=0.5, min_time=2.0,\\n                    tol=0.01, retries=3):\\n        \\"\\"\\"Move through joint waypoints (durations cumulative seconds; if None,\\n        derived from `speed` rad/s per segment). Verifies arrival from\\n        /joint_states and resends the final point if off by > tol.\\"\\"\\"\\n        waypoints = [np.asarray(w, float) for w in waypoints]\\n        if durations is None:\\n            q = self.arm_q()\\n            durations, t = [], 0.0\\n            for w in waypoints:\\n                t += max(min_time, np.abs(w - q).max() / speed)\\n                durations.append(round(t, 2)); q = w\\n        code = self._send_traj(waypoints, durations)\\n        err, q = self.settle(waypoints[-1])\\n        log(f\\"trajectory code={code} err={err:.4f} (dur {durations[-1]}s)\\")\\n        n = 0\\n        while err > tol and n < retries:\\n            n += 1\\n            d = max(min_time, err / speed)\\n            log(f\\"  re-sending final point (err {err:.4f}, {d:.1f}s)\\")\\n            code = self._send_traj([waypoints[-1]], [d])\\n            err, q = self.settle(waypoints[-1])\\n            log(f\\"  code={code} err={err:.4f}\\")\\n        return err\\n\\n    def move_to_pose(self, p_world, R, seed=None, at_tcp=True, speed=0.5, **kw):\\n        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp, **kw)\\n        if q is None:\\n            raise RuntimeError(f\\"IK failed for {p_world}\\")\\n        err = self.move_joints([q], speed=speed)\\n        t, _ = self.tcp()\\n        log(f\\"  tcp now {np.round(t, 4)} (target {np.round(p_world, 4)}) joint err {err:.4f}\\")\\n        return q, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 40
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,0]>-0.32)&(xyz[...,0]<-0.12)&(xyz[...,1]>-0.20)&(xyz[...,1]<0.03)&(xyz[...,2]>0.995)&(xyz[...,2]<1.03)
p=xyz[m]
# fit circle (algebraic) to rim points
x,y=p[:,0],p[:,1]
A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
print('rim fit centre',round(cx,4),round(cy,4),'radius',round(rad,4),'n',len(p))
m=valid&(xyz[...,2]>0.915)&(xyz[...,2]<0.995)&(np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)>0.052)&(np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)<0.10)
h=xyz[m]
print('handle pts',len(h),'x[%.3f,%.3f] y[%.3f,%.3f]'%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max()))
ang=np.degrees(np.arctan2(h[:,1]-cy,h[:,0]-cx)); print('handle azimuth mean %.1f deg'%ang.mean(), 'std', ang.std().round(1))
"

# openrua op 41
mkdir -p "$(dirname /workspace/phase1b_rotate_mug.py)"
cat > /workspace/phase1b_rotate_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 1b: re-grasp mug rim from above, lift, rotate via joint 7 so the
handle faces -y, place at S, release, retreat."""
import math
import sys
import numpy as np
from rob import *

C = np.array([-0.2201, -0.0698])      # mug centre now
HANDLE_AZ = math.radians(133.5)       # handle azimuth now
TARGET_AZ = math.radians(-90.0)       # want handle pointing -y
RIM_Z = 1.012
S = np.array([-0.21, -0.09])          # staging spot for mug centre
WALL_OFF = 0.044

alpha = (TARGET_AZ - HANDLE_AZ + math.pi) % (2 * math.pi) - math.pi   # CCW rotation needed, in (-pi, pi]
log(f"need CCW rotation alpha={math.degrees(alpha):.1f} deg")

r = Robot("phase1b")
q0 = r.arm_q()
log("start q", np.round(q0, 3), "gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.07:
    r.gripper(0.04)

# pick a pinch azimuth psi whose IK joint7 allows the rotation
lo, hi = LIMITS[6]
chosen = None
for psi_deg in (-90, 180, 0, 90):
    psi = math.radians(psi_deg)
    yaw = math.atan2(-math.cos(psi), -math.sin(psi))
    R = R_down(yaw)
    tcp_xy = C + WALL_OFF * np.array([math.cos(psi), math.sin(psi)])
    above = np.array([*tcp_xy, 1.12])
    q = r.ik(above, R, seed=q0)
    if q is None:
        log(f"psi={psi_deg}: IK failed"); continue
    dj7 = -alpha
    j7_new = q[6] + dj7
    ok = lo + 0.05 <= j7_new <= hi - 0.05
    log(f"psi={psi_deg}: j7={q[6]:.3f} -> {j7_new:.3f} ok={ok}")
    if ok:
        chosen = (psi, R, tcp_xy, q, dj7); break
    # try the equivalent rotation the other way round
    dj7b = -(alpha - 2 * math.pi) if alpha > 0 else -(alpha + 2 * math.pi)
    j7_new = q[6] + dj7b
    ok = lo + 0.05 <= j7_new <= hi - 0.05
    log(f"psi={psi_deg}: alt dj7={dj7b:.3f} -> {j7_new:.3f} ok={ok}")
    if ok:
        chosen = (psi, R, tcp_xy, q, dj7b); break
if chosen is None:
    log("!! no feasible pinch/rotation"); sys.exit(3)
psi, R, tcp_xy, q_above, dj7 = chosen

log("move above", np.round(tcp_xy, 4))
r.move_joints([q_above])
grasp = np.array([*tcp_xy, RIM_Z - 0.022])
q2, err = r.move_to_pose(grasp, R, seed=q_above)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); sys.exit(2)
lift = np.array([*tcp_xy, 1.13])
q3, err = r.move_to_pose(lift, R, seed=q2)
log("gap after lift", round(r.finger_gap(), 4))

q4 = q3.copy(); q4[6] += dj7
log(f"rotate j7 by {dj7:.3f} -> {q4[6]:.3f}")
r.move_joints([q4], speed=0.6)
t, Rn = r.tcp()
log("tcp", np.round(t, 4), "y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4))

# actual rotation achieved (from hand y axis): planned alpha = -dj7
alpha_done = -dj7
v0 = -WALL_OFF * np.array([math.cos(psi), math.sin(psi)])         # mug centre rel. TCP before
ca, sa = math.cos(alpha_done), math.sin(alpha_done)
v = np.array([ca * v0[0] - sa * v0[1], sa * v0[0] + ca * v0[1]])  # after
tcp_place = S - v
log("place: tcp xy", np.round(tcp_place, 4), "mug centre", S)
q5, err = r.move_to_pose(np.array([*tcp_place, 1.13]), Rn, seed=q4)
q6, err = r.move_to_pose(np.array([*tcp_place, RIM_Z - 0.022 + 0.004]), Rn, seed=q5)
r.gripper(0.04)
q7, err = r.move_to_pose(np.array([*tcp_place, 1.15]), Rn, seed=q6)
log("PHASE1B DONE")
OPENRUA_EOF

# openrua op 42
nohup python3 -u phase1b_rotate_mug.py > phase1b.log 2>&1 &
sleep 50; cat /workspace/phase1b.log

# openrua op 43
sleep 60; tail -20 /workspace/phase1b.log

# openrua op 44
sleep 45; tail -8 /workspace/phase1b.log

# openrua op 45
sed -n 25,40p /workspace/phase1b.log; timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,0]>-0.32)&(xyz[...,0]<-0.10)&(xyz[...,1]>-0.22)&(xyz[...,1]<0.03)&(xyz[...,2]>0.995)&(xyz[...,2]<1.03)
p=xyz[m]
x,y=p[:,0],p[:,1]
A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
print('rim fit centre',round(cx,4),round(cy,4),'radius',round(rad,4),'n',len(p), 'rim z max', p[:,2].max().round(3))
rr=np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)
m=valid&(xyz[...,2]>0.915)&(xyz[...,2]<0.995)&(rr>0.052)&(rr<0.10)
h=xyz[m]
print('handle pts',len(h),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max(),h[:,2].min(),h[:,2].max()))
ang=np.degrees(np.arctan2(h[:,1]-cy,h[:,0]-cx)); print('handle azimuth mean %.1f deg'%ang.mean(), 'std', ang.std().round(1))
"; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py birdview && python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[190:330,240:360]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('crop_bird.png',crop)"

# openrua op 47
timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
m=valid&(xyz[...,0]>-0.20)&(xyz[...,0]<-0.055)&(xyz[...,1]>-0.15)&(xyz[...,1]<0.02)&(xyz[...,2]>1.0)&(xyz[...,2]<1.03)
p=xyz[m]
x,y=p[:,0],p[:,1]
A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rad=np.sqrt(c[2]+cx**2+cy**2)
print('white rim fit centre',round(cx,4),round(cy,4),'radius',round(rad,4),'n',len(p),'z',p[:,2].min().round(3),p[:,2].max().round(3))
rr=np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)
m=valid&(xyz[...,2]>0.915)&(xyz[...,2]<0.995)&(rr>0.052)&(rr<0.095)&(xyz[...,0]<-0.055)
h=xyz[m]
print('handle pts',len(h),'x[%.3f,%.3f] y[%.3f,%.3f]'%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max()))
ang=np.degrees(np.arctan2(h[:,1]-cy,h[:,0]-cx)); print('handle azimuth mean %.1f deg'%ang.mean(), 'std', ang.std().round(1))
# yellow mug
m=valid&(xyz[...,0]>-0.06)&(xyz[...,0]<0.08)&(xyz[...,1]>-0.12)&(xyz[...,1]<0.06)&(xyz[...,2]>0.99)&(xyz[...,2]<1.03)
p=xyz[m]; x,y=p[:,0],p[:,1]
A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print('yellow rim centre',round(c[0],4),round(c[1],4),'r',round(np.sqrt(c[2]+c[0]**2+c[1]**2),4),'n',len(p))
"

# openrua op 48
timeout 120 python3 cloud.py agentview >/dev/null && python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; valid=d['valid']
cx,cy=-0.0954,-0.0511
rr=np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)
m=valid&(xyz[...,2]>0.915)&(xyz[...,2]<0.995)&(rr>0.050)&(rr<0.095)&(xyz[...,0]>-0.10)&(xyz[...,0]<-0.0)&(xyz[...,1]<-0.06)
h=xyz[m]
print('handle pts',len(h),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max(),h[:,2].min(),h[:,2].max()))
ang=np.degrees(np.arctan2(h[:,1]-cy,h[:,0]-cx)); print('azimuth mean %.1f median %.1f std %.1f'%(ang.mean(),np.median(ang),ang.std()))
far=h[rr[m]>0.07]; ang=np.degrees(np.arctan2(far[:,1]-cy,far[:,0]-cx)); print('outer handle pts',len(far),'azimuth median %.1f'%np.median(ang))
"

# openrua op 49
mkdir -p "$(dirname /workspace/phase1c_rotate_mug.py)"
cat > /workspace/phase1c_rotate_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 1c: pinch white mug rim at its -x point, lift a little, rotate via
joint 7 (slowly, fully converging) so the handle faces -y, place at S."""
import math
import sys
import numpy as np
from rob import *

C = np.array([-0.0954, -0.0511])      # mug centre now
HANDLE_AZ = math.radians(-12.0)
TARGET_AZ = math.radians(-90.0)
RIM_Z = 1.012
S = np.array([-0.20, -0.12])          # staging spot for mug centre
WALL_OFF = 0.044
PSI = math.pi                          # pinch at the -x rim point, fingers along x

alpha = (TARGET_AZ - HANDLE_AZ + math.pi) % (2 * math.pi) - math.pi
dj7 = -alpha
log(f"rotation alpha={math.degrees(alpha):.1f} deg -> dj7={dj7:.3f}")

r = Robot("phase1c")
q0 = r.arm_q()
log("start q", np.round(q0, 3), "gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.07:
    r.gripper(0.04)

yaw = math.atan2(-math.cos(PSI), -math.sin(PSI))
R = R_down(yaw)
log("hand y axis (finger opening):", np.round(R[:, 1], 3))
tcp_xy = C + WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
lo, hi = LIMITS[6]

above = np.array([*tcp_xy, 1.12])
q1 = r.ik(above, R, seed=q0)
if q1 is None:
    log("IK fail above"); sys.exit(1)
if not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
    # try other yaw branch (rotate hand 180 deg about z: same pinch, fingers swapped)
    R = R_down(yaw + math.pi)
    q1 = r.ik(above, R, seed=q0)
    log("using flipped yaw; j7", None if q1 is None else round(q1[6], 3))
    if q1 is None or not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
        log("!! j7 range problem"); sys.exit(3)
log(f"j7 {q1[6]:.3f} -> {q1[6]+dj7:.3f}")
r.move_joints([q1], speed=0.3)
t, _ = r.tcp(); log("above tcp", np.round(t, 4))

grasp = np.array([*tcp_xy, RIM_Z - 0.022])
q2, err = r.move_to_pose(grasp, R, seed=q1, speed=0.3)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); sys.exit(2)

lift = np.array([*tcp_xy, RIM_Z - 0.022 + 0.04])
q3, err = r.move_to_pose(lift, R, seed=q2, speed=0.3)
log("gap after lift", round(r.finger_gap(), 4))

q4 = q3.copy(); q4[6] += dj7
log(f"rotate j7 -> {q4[6]:.3f}")
err = r.move_joints([q4], speed=0.15, retries=8)
t, Rn = r.tcp()
log("after rotation tcp", np.round(t, 4), "y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4), "err", round(err, 4))

# actual rotation from achieved joint7
q_now = r.arm_q()
alpha_done = -(q_now[6] - q3[6])
v0 = -WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
ca, sa = math.cos(alpha_done), math.sin(alpha_done)
v = np.array([ca * v0[0] - sa * v0[1], sa * v0[0] + ca * v0[1]])
tcp_place = S - v
log(f"alpha_done={math.degrees(alpha_done):.1f} deg; place tcp xy {np.round(tcp_place,4)}")
q5, err = r.move_to_pose(np.array([*tcp_place, RIM_Z - 0.022 + 0.04]), Rn, seed=q_now, speed=0.3)
q6, err = r.move_to_pose(np.array([*tcp_place, RIM_Z - 0.022 + 0.004]), Rn, seed=q5, speed=0.3)
r.gripper(0.04)
q7, err = r.move_to_pose(np.array([*tcp_place, 1.15]), Rn, seed=q6, speed=0.3)
log("PHASE1C DONE")
OPENRUA_EOF

# openrua op 50
nohup python3 -u phase1c_rotate_mug.py > phase1c.log 2>&1 &
sleep 45; cat /workspace/phase1c.log

# openrua op 51
sleep 90; cat /workspace/phase1c.log | tail -n +12

# openrua op 52
sleep 60; tail -n +30 /workspace/phase1c.log

# openrua op 53
python3 cloud.py birdview && python3 cloud.py agentview && python3 - <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); xyz=d["xyz"]; v=d["valid"]; rgb=d["rgb"]
P=xyz[v]; C=rgb[v]
# table region objects above table, away from microwave (y<0.2)
m=(P[:,2]>0.93)&(P[:,2]<1.09)&(P[:,1]<0.2)&(P[:,0]>-0.5)
Q=P[m]; K=C[m]
print("pts",m.sum())
# cluster by simple grid
from collections import defaultdict
import math
cells=defaultdict(list)
for i,p in enumerate(Q):
    cells[(int(math.floor(p[0]/0.02)),int(math.floor(p[1]/0.02)))].append(i)
# connected components on cells
keys=set(cells); comps=[]
while keys:
    k=keys.pop(); stack=[k]; comp=[k]
    while stack:
        a=stack.pop()
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                n=(a[0]+dx,a[1]+dy)
                if n in keys: keys.remove(n); stack.append(n); comp.append(n)
    comps.append(comp)
for comp in comps:
    idx=np.concatenate([cells[c] for c in comp])
    if len(idx)<50: continue
    q=Q[idx]; k=K[idx]
    print(f"cluster n={len(idx)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}] rgb mean {k.mean(0).round(0)}")
EOF

# openrua op 54
python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); rgb=d["rgb"]
cv2.imwrite("birdview.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
a=np.load("agentview_cloud.npz")["rgb"]
cv2.imwrite("agentview.png", cv2.cvtColor(a, cv2.COLOR_RGB2BGR))
# crop birdview around mugs: world x[-0.3,0.1] y[-0.25,0.25]; image right=+y, down=+x, 3.62mm/px, centre (-0.2,0)
def px(x,y): return int(320+(y-0)/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.30,-0.25);(u1,v1)=px(0.10,0.25)
crop=rgb[v0:v1,u0:u1]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
print(u0,v0,u1,v1)
EOF

# openrua op 55
python3 - <<'EOF'
import numpy as np, math
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; C=d["rgb"][d["valid"]]
m=(P[:,2]>0.995)&(P[:,2]<1.03)&(P[:,0]<-0.05)&(P[:,0]>-0.16)&(P[:,1]<0.02)&(P[:,1]>-0.13)
Q=P[m]; print("rim pts",m.sum(), "z", Q[:,2].min().round(3), Q[:,2].max().round(3))
# circle fit
A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
print(f"circle centre ({cx:.4f},{cy:.4f}) r={r:.4f}")
res=np.hypot(Q[:,0]-cx,Q[:,1]-cy)-r; print("resid std",res.std().round(4))
# agentview handle
a=np.load("agentview_cloud.npz"); P2=a["xyz"][a["valid"]]
m2=(P2[:,2]>0.92)&(P2[:,2]<1.0)&(np.hypot(P2[:,0]-cx,P2[:,1]-cy)>0.052)&(np.hypot(P2[:,0]-cx,P2[:,1]-cy)<0.09)
# exclude yellow mug (centre approx (-0.0116,-0.0144)) region
m2&=np.hypot(P2[:,0]+0.0116,P2[:,1]+0.0144)>0.06
H=P2[m2]; print("handle pts",m2.sum())
if m2.sum():
    az=np.degrees(np.arctan2(H[:,1]-cy,H[:,0]-cx))
    print("x",H[:,0].min().round(3),H[:,0].max().round(3),"y",H[:,1].min().round(3),H[:,1].max().round(3),"z",H[:,2].min().round(3),H[:,2].max().round(3))
    print("az median",np.median(az).round(1),"mean",az.mean().round(1))
    far=np.hypot(H[:,0]-cx,H[:,1]-cy)>0.07
    print("outer pts",far.sum(),"az median",np.median(az[far]).round(1))
EOF

# openrua op 56
python3 - <<'EOF'
import numpy as np, math
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]
for zlo in (0.995,1.0,1.005):
    m=(P[:,2]>zlo)&(P[:,2]<1.03)&(P[:,0]<-0.05)&(P[:,0]>-0.16)&(P[:,1]<0.02)&(P[:,1]>-0.13)
    Q=P[m]
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
    res=np.hypot(Q[:,0]-cx,Q[:,1]-cy)-r
    # robust refit
    k=np.abs(res)<0.006; Q2=Q[k]
    A=np.c_[2*Q2[:,0],2*Q2[:,1],np.ones(len(Q2))]; b=Q2[:,0]**2+Q2[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
    print(f"zlo {zlo}: n={m.sum()} inl={k.sum()} centre ({cx:.4f},{cy:.4f}) r={r:.4f}")
# extents of all white mug pts above table
m=(P[:,2]>0.93)&(P[:,0]<-0.05)&(P[:,0]>-0.16)&(P[:,1]<0.0)&(P[:,1]>-0.13)&(P[:,2]<1.05)
Q=P[m]; print("body x",Q[:,0].min().round(4),Q[:,0].max().round(4),"y",Q[:,1].min().round(4),Q[:,1].max().round(4))
EOF

# openrua op 57
grep -n "def move_joints" -A 45 /workspace/rob.py | head -80

# openrua op 58
python3 - <<'EOF'
import re
s=open("rob.py").read()
s=s.replace("""    def move_to_pose(self, p_world, R, seed=None, at_tcp=True, speed=0.5, **kw):
        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp, **kw)
        if q is None:
            raise RuntimeError(f"IK failed for {p_world}")
        err = self.move_joints([q], speed=speed)""",
"""    def move_to_pose(self, p_world, R, seed=None, at_tcp=True, speed=0.5,
                     retries=3, tol=0.01, **kw):
        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp, **kw)
        if q is None:
            raise RuntimeError(f"IK failed for {p_world}")
        err = self.move_joints([q], speed=speed, retries=retries, tol=tol)""")
s=s.replace("""            err, q = self.settle(waypoints[-1])
            log(f"  code={code} err={err:.4f}")""",
"""            err, q = self.settle(waypoints[-1])
            dq = np.asarray(q) - waypoints[-1]
            log(f"  code={code} err={err:.4f} worst j{int(np.argmax(np.abs(dq)))+1} dq={np.round(dq, 3)}")""")
open("rob.py","w").write(s)
EOF
grep -n "worst j" rob.py; grep -n "def settle" -A 14 rob.py

# openrua op 59
sed -e 's/^C = np.array(\[-0.0954, -0.0511\]).*/C = np.array([-0.091, -0.057])      # mug centre now/' \
 -e 's/^HANDLE_AZ = math.radians(-12.0)/HANDLE_AZ = math.radians(-17.0)/' \
 -e 's/^RIM_Z = 1.012/RIM_Z = 1.013\nGRASP_Z = 0.985/' \
 -e 's/Robot("phase1c")/Robot("phase1d")/' \
 -e 's/RIM_Z - 0.022 + 0.04/GRASP_Z + 0.045/g' -e 's/RIM_Z - 0.022 + 0.004/GRASP_Z + 0.006/g' -e 's/RIM_Z - 0.022/GRASP_Z/g' \
 -e 's/PHASE1C DONE/PHASE1D DONE/' phase1c_rotate_mug.py > phase1d_rotate_mug.py
python3 - <<'EOF'
s=open("/workspace/phase1d_rotate_mug.py").read()
s=s.replace("""q2, err = r.move_to_pose(grasp, R, seed=q1, speed=0.3)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); sys.exit(2)
""","""q2, err = r.move_to_pose(grasp, R, seed=q1, speed=0.3, retries=8, tol=0.004)
t, _ = r.tcp()
if np.linalg.norm(t - grasp) > 0.006:
    log("!! grasp pose not reached, abort"); sys.exit(4)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); r.gripper(0.04); sys.exit(2)
""")
s=s.replace("""log("gap after lift", round(r.finger_gap(), 4))
""","""log("gap after lift", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! mug slipped out; release and retreat"); r.gripper(0.04)
    r.move_to_pose(np.array([*tcp_xy, 1.15]), R, seed=q3, speed=0.3); sys.exit(5)
""")
open("/workspace/phase1d_rotate_mug.py","w").write(s)
EOF
grep -n "GRASP_Z\|C = \|HANDLE_AZ =\|retries=8\|slipped" phase1d_rotate_mug.py

# openrua op 60
nohup python3 -u phase1d_rotate_mug.py > phase1d.log 2>&1 &
sleep 100; cat /workspace/phase1d.log

# openrua op 61
sleep 30; tail -n +25 /workspace/phase1d.log

# openrua op 62
python3 cloud.py birdview >/dev/null && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np, math, cv2
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; rgb=d["rgb"]
cv2.imwrite("birdview.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.35,-0.25);(u1,v1)=px(0.10,0.25)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
a=np.load("agentview_cloud.npz"); cv2.imwrite("agentview.png", cv2.cvtColor(a["rgb"], cv2.COLOR_RGB2BGR))
# clusters on table
m=(P[:,2]>0.93)&(P[:,2]<1.05)&(P[:,1]<0.12)&(P[:,0]>-0.4)&(P[:,1]>-0.3)
Q=P[m]
for name,box in (("white",(-0.30,-0.14,-0.20,0.0)),("yellow",(-0.08,0.05,-0.08,0.05))):
    k=(Q[:,0]>box[0])&(Q[:,0]<box[1])&(Q[:,1]>box[2])&(Q[:,1]<box[3]); q=Q[k]
    print(name,"n",k.sum(),"x",q[:,0].min().round(4),q[:,0].max().round(4),"y",q[:,1].min().round(4),q[:,1].max().round(4),"zmax",q[:,2].max().round(3))
# white rim fit
m=(P[:,2]>0.998)&(P[:,2]<1.03)&(P[:,0]<-0.14)&(P[:,0]>-0.30)&(P[:,1]<0.0)&(P[:,1]>-0.2)
Q=P[m]
for it in range(2):
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
    res=np.hypot(Q[:,0]-cx,Q[:,1]-cy)-r; Q=Q[np.abs(res)<0.006]
print(f"white rim centre ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(Q)}")
P2=a["xyz"][a["valid"]]
rr=np.hypot(P2[:,0]-cx,P2[:,1]-cy)
m2=(P2[:,2]>0.92)&(P2[:,2]<1.0)&(rr>0.052)&(rr<0.095)
H=P2[m2]; az=np.degrees(np.arctan2(H[:,1]-cy,H[:,0]-cx))
print("handle pts",len(H),"x",H[:,0].min().round(3),H[:,0].max().round(3),"y",H[:,1].min().round(3),H[:,1].max().round(3),"z",H[:,2].min().round(3),H[:,2].max().round(3))
print("az median",np.median(az).round(1),"outer az median",np.median(az[rr[m2]>0.07]).round(1))
EOF

# openrua op 63
cat > /workspace/park.py <<'EOF'
#!/usr/bin/env python3
import sys, numpy as np
from rob import *
x,y,z = map(float, sys.argv[1:4])
r = Robot("park")
q0 = r.arm_q(); t, R = r.tcp(); log("tcp", np.round(t,4))
r.move_to_pose(np.array([x,y,z]), R_down(0.0), seed=q0, speed=0.3)
EOF
python3 -u park.py -0.35 -0.25 1.25 2>&1 | tail -3 && python3 cloud.py birdview >/dev/null && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np, math, cv2
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; rgb=d["rgb"]
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.35,-0.25);(u1,v1)=px(0.10,0.25)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
a=np.load("agentview_cloud.npz"); cv2.imwrite("agentview.png", cv2.cvtColor(a["rgb"], cv2.COLOR_RGB2BGR))
m=(P[:,2]>0.93)&(P[:,2]<1.05)&(P[:,1]<0.2)&(P[:,0]>-0.4)&(P[:,1]>-0.3)&(P[:,0]<0.1)
Q=P[m]
for name,box in (("white",(-0.30,-0.10,-0.20,0.0)),("yellow",(-0.10,0.10,-0.05,0.2))):
    k=(Q[:,0]>box[0])&(Q[:,0]<box[1])&(Q[:,1]>box[2])&(Q[:,1]<box[3]); q=Q[k]
    print(name,"n",k.sum(),"x",q[:,0].min().round(4),q[:,0].max().round(4),"y",q[:,1].min().round(4),q[:,1].max().round(4),"zmax",q[:,2].max().round(3))
m=(P[:,2]>0.998)&(P[:,2]<1.03)&(P[:,0]<-0.10)&(P[:,0]>-0.30)&(P[:,1]<0.0)&(P[:,1]>-0.2)
Q=P[m]
for it in range(2):
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
    res=np.hypot(Q[:,0]-cx,Q[:,1]-cy)-r; Q=Q[np.abs(res)<0.006]
print(f"white rim centre ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(Q)} zmean {Q[:,2].mean():.3f}")
P2=a["xyz"][a["valid"]]
rr=np.hypot(P2[:,0]-cx,P2[:,1]-cy)
m2=(P2[:,2]>0.92)&(P2[:,2]<1.0)&(rr>0.052)&(rr<0.095)&(P2[:,1]<0.0)
H=P2[m2]; az=np.degrees(np.arctan2(H[:,1]-cy,H[:,0]-cx))
print("handle pts",len(H),"x",H[:,0].min().round(3),H[:,0].max().round(3),"y",H[:,1].min().round(3),H[:,1].max().round(3),"z",H[:,2].min().round(3),H[:,2].max().round(3))
print("az median",np.median(az).round(1),"outer az median",np.median(az[rr[m2]>0.07]).round(1))
EOF

# openrua op 64
python3 - <<'EOF'
s=open("phase1d_rotate_mug.py").read()
s=s.replace("C = np.array([-0.091, -0.057])","C = np.array([-0.176, -0.070])")
s=s.replace("HANDLE_AZ = math.radians(-17.0)","HANDLE_AZ = math.radians(0.5)")
s=s.replace("S = np.array([-0.20, -0.12])","S = np.array([-0.22, -0.13])")
s=s.replace('Robot("phase1d")','Robot("phase1e")')
s=s.replace("above = np.array([*tcp_xy, 1.12])","above = np.array([*tcp_xy, 1.22])")
# lift high for rotation (above door top 1.11 + mug 0.085 below TCP)
s=s.replace("lift = np.array([*tcp_xy, GRASP_Z + 0.045])","lift = np.array([*tcp_xy, GRASP_Z + 0.045])\nhigh = np.array([*tcp_xy, 1.22])")
s=s.replace("""q4 = q3.copy(); q4[6] += dj7""","""q3, err = r.move_to_pose(high, R, seed=q3, speed=0.3)
log("gap at high", round(r.finger_gap(), 4))
q4 = q3.copy(); q4[6] += dj7""")
s=s.replace("""q5, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.045]), Rn, seed=q_now, speed=0.3)""",
"""q5, err = r.move_to_pose(np.array([*tcp_place, 1.22]), Rn, seed=q_now, speed=0.3)
q5, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.045]), Rn, seed=q5, speed=0.3)""")
s=s.replace("PHASE1D DONE","PHASE1E DONE")
open("phase1e_rotate_mug.py","w").write(s)
EOF
grep -n "1.22\|high\|C = \|S = \|HANDLE" phase1e_rotate_mug.py
nohup python3 -u phase1e_rotate_mug.py > phase1e.log 2>&1 &
sleep 120; cat phase1e.log

# openrua op 65
sleep 20; tail -n +37 phase1e.log; grep -n "def tcp\|def fk\|def joints\|def arm_q" -A 12 rob.py | head -80

# openrua op 66
sleep 15; tail -n 4 phase1e.log; python3 - <<'EOF' 2>&1 | grep -v "^$"
import numpy as np, math
from rob import *
r=Robot("fktest")
q=r.arm_q(); print("q",np.round(q,3))
p,quat=r.fk(q); print("quat",np.round(quat,4), "norm", np.linalg.norm(quat).round(4))
R=quat_to_R(quat); print("R\n",np.round(R,3), "det", np.linalg.det(R).round(3))
for d in (0.5,1.0,1.5708):
    q2=q.copy(); q2[6]+=d
    p2,quat2=r.fk(q2); R2=quat_to_R(quat2)
    ang=math.degrees(math.atan2(np.cross(R[:,1],R2[:,1])@R[:,2], R[:,1]@R2[:,1]))
    print(f"dj7={d}: y_hand rotated {ang:.1f} deg about hand z; p diff {np.round(p2-p,4)}")
EOF

# openrua op 67
python3 -u park.py -0.35 -0.30 1.25 2>&1 | tail -1 && python3 cloud.py birdview >/dev/null && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np, math, cv2
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; rgb=d["rgb"]
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.35,-0.25);(u1,v1)=px(0.10,0.25)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
a=np.load("agentview_cloud.npz"); cv2.imwrite("agentview.png", cv2.cvtColor(a["rgb"], cv2.COLOR_RGB2BGR))
m=(P[:,2]>0.998)&(P[:,2]<1.03)&(P[:,0]<-0.10)&(P[:,0]>-0.35)&(P[:,1]<0.0)&(P[:,1]>-0.25)
Q=P[m]
for it in range(2):
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
    res=np.hypot(Q[:,0]-cx,Q[:,1]-cy)-r; Q=Q[np.abs(res)<0.006]
print(f"white rim centre ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(Q)} zmean {Q[:,2].mean():.3f}")
for name,P2 in (("bird",P),("agent",a["xyz"][a["valid"]])):
    rr=np.hypot(P2[:,0]-cx,P2[:,1]-cy)
    m2=(P2[:,2]>0.92)&(P2[:,2]<1.0)&(rr>0.052)&(rr<0.095)
    H=P2[m2]; az=np.degrees(np.arctan2(H[:,1]-cy,H[:,0]-cx))
    if len(H): print(name,"handle pts",len(H),"x",H[:,0].min().round(3),H[:,0].max().round(3),"y",H[:,1].min().round(3),H[:,1].max().round(3),"az median",np.median(az).round(1),"outer",np.median(az[rr[m2]>0.07]).round(1) if (rr[m2]>0.07).any() else None)
EOF

# openrua op 68
python3 - <<'EOF' 2>&1 | grep -v "^$"
import numpy as np, math
from rob import *
r=Robot("j7test")
q=r.arm_q(); p,R=r.tcp(); print("q",np.round(q,3),"y_hand",np.round(R[:,1],3))
q2=q.copy(); q2[6]+=1.0
err=r.move_joints([q2],speed=0.15,retries=6)
qn=r.arm_q(); p2,R2=r.tcp(); print("q",np.round(qn,3),"y_hand",np.round(R2[:,1],3), "dj7 actual", round(qn[6]-q[6],4))
ang=math.degrees(math.atan2(np.cross(R[:,1],R2[:,1])@R[:,2], R[:,1]@R2[:,1]))
print("rotation about hand z:", round(ang,1))
import subprocess; subprocess.run(["python3","cloud.py","birdview"],capture_output=True)
EOF
python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); rgb=d["rgb"]
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.55,-0.45);(u1,v1)=px(-0.15,-0.15)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_hand.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
EOF

# openrua op 69
python3 - <<'EOF'
import numpy as np, math
from rob import R_down, R_to_quat, quat_to_R
for yaw in (0.0, math.pi/2, 0.3):
    R=R_down(yaw); q=R_to_quat(R); R2=quat_to_R(q)
    print("yaw",yaw,"\nR\n",np.round(R,3),"\nquat",np.round(q,4),"\nR2\n",np.round(R2,3))
EOF
grep -n "def R_to_quat" -A 25 rob.py

# openrua op 70
sed -n 158,215p rob.py

# openrua op 71
python3 - <<'EOF' 2>&1 | grep -v "^$"
import numpy as np, math
from rob import *
r=Robot("linktest")
q=r.arm_q()
for link in ("panda_link8","panda_hand","panda_hand_tcp"):
    try:
        p,quat=r.fk(q,link=link); R=quat_to_R(quat)
        print(link, np.round(p,4), "y", np.round(R[:,1],3), "z", np.round(R[:,2],3))
    except Exception as e: print(link, "ERR", e)
EOF
grep -n "tip\|link" machine.yaml | head

# openrua op 72
python3 - <<'EOF'
s=open("/workspace/rob.py").read()
s=s.replace('''            req.ik_request.group_name = M["planning"]["group"]
''','''            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = IK_LINK
''')
s=s.replace('TCP = M["hand"]["tcp_offset_m"]','TCP = M["hand"]["tcp_offset_m"]\nIK_LINK = "panda_hand"   # group tip is panda_link8, rotated 45 deg about z from the hand')
open("/workspace/rob.py","w").write(s)
EOF
python3 - <<'EOF' 2>&1 | grep -v "^$"
import numpy as np, math
from rob import *
r=Robot("iktest")
q=r.arm_q(); p,R=r.tcp(); print("tcp now",np.round(p,4),"y",np.round(R[:,1],3))
Rt=R_down(0.0)
qs=r.ik(np.array([-0.35,-0.30,1.25]), Rt, seed=q)
print("ik sol", None if qs is None else np.round(qs,3))
if qs is not None:
    p2,R2=r.tcp(qs); print("fk of sol: tcp",np.round(p2,4),"y",np.round(R2[:,1],3),"(want 0,-1,0)")
EOF

# openrua op 73
mkdir -p "$(dirname /workspace/phase1f_rotate_mug.py)"
cat > /workspace/phase1f_rotate_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 1f: rim pinch (now with correct hand orientation), lift high,
observe handle azimuth from cameras, rotate j7, observe again, place at S."""
import math
import subprocess
import sys
import numpy as np
from rob import *

C = np.array([-0.1871, -0.1017])      # mug centre now
HANDLE_AZ = math.radians(-1.0)
TARGET_AZ = math.radians(-90.0)
GRASP_Z = 0.985
S = np.array([-0.22, -0.13])
WALL_OFF = 0.044
PSI = math.pi                          # pinch the -x rim point, fingers along x
HIGH_Z = 1.22

alpha = (TARGET_AZ - HANDLE_AZ + math.pi) % (2 * math.pi) - math.pi
dj7 = -alpha
log(f"rotation alpha={math.degrees(alpha):.1f} deg -> dj7={dj7:.3f}")


def handle_az(centre):
    """Handle azimuth (deg) about `centre` from agentview + birdview clouds."""
    out = []
    for cam in ("agentview", "birdview"):
        subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); P = d["xyz"][d["valid"]]
        rr = np.hypot(P[:, 0] - centre[0], P[:, 1] - centre[1])
        m = (P[:, 2] > centre[2] - 0.075) & (P[:, 2] < centre[2] + 0.01) & (rr > 0.055) & (rr < 0.095)
        H = P[m]
        if len(H) < 15:
            out.append((cam, len(H), None)); continue
        az = np.degrees(np.arctan2(H[:, 1] - centre[1], H[:, 0] - centre[0]))
        out.append((cam, len(H), round(float(np.median(az)), 1)))
    return out


r = Robot("phase1f")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)

yaw = math.atan2(-math.cos(PSI), -math.sin(PSI))
R = R_down(yaw)
tcp_xy = C + WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
lo, hi = LIMITS[6]
above = np.array([*tcp_xy, HIGH_Z])
q1 = r.ik(above, R, seed=q0)
if q1 is None:
    log("IK fail"); sys.exit(1)
if not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
    R = R_down(yaw + math.pi)
    q1 = r.ik(above, R, seed=q0)
    log("flipped yaw; j7", None if q1 is None else round(q1[6], 3))
    if q1 is None or not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
        log("!! j7 range problem"); sys.exit(3)
log(f"j7 {q1[6]:.3f} -> {q1[6]+dj7:.3f}; hand y", np.round(R[:, 1], 3))
r.move_joints([q1], speed=0.3, retries=6)
t, Ra = r.tcp(); log("above tcp", np.round(t, 4), "y_hand", np.round(Ra[:, 1], 3))

grasp = np.array([*tcp_xy, GRASP_Z])
q2, err = r.move_to_pose(grasp, R, seed=q1, speed=0.3, retries=8, tol=0.004)
t, _ = r.tcp()
if np.linalg.norm(t - grasp) > 0.006:
    log("!! grasp pose not reached"); sys.exit(4)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); r.gripper(0.04); sys.exit(2)

q3, err = r.move_to_pose(np.array([*tcp_xy, GRASP_Z + 0.045]), R, seed=q2, speed=0.3)
log("gap after lift", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! slipped"); r.gripper(0.04); r.move_to_pose(np.array([*tcp_xy, 1.15]), R, seed=q3, speed=0.3); sys.exit(5)
q3, err = r.move_to_pose(np.array([*tcp_xy, HIGH_Z]), R, seed=q3, speed=0.3)
t, _ = r.tcp()
centre_air = np.array([t[0] - WALL_OFF * math.cos(PSI), t[1] - WALL_OFF * math.sin(PSI), t[2] + 0.028])  # rim z ~ tcp+0.028
log("in air: gap", round(r.finger_gap(), 4), "handle az", handle_az(centre_air))

q4 = q3.copy(); q4[6] += dj7
err = r.move_joints([q4], speed=0.15, retries=8)
t, Rn = r.tcp()
q_now = r.arm_q()
alpha_done = -(q_now[6] - q3[6])
v0 = -WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
ca, sa = math.cos(alpha_done), math.sin(alpha_done)
v = np.array([ca * v0[0] - sa * v0[1], sa * v0[0] + ca * v0[1]])
centre_air = np.array([t[0] + v[0], t[1] + v[1], t[2] + 0.028])
log(f"rotated {math.degrees(alpha_done):.1f} deg; y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4),
    "handle az", handle_az(centre_air))

tcp_place = S - v
log("place tcp xy", np.round(tcp_place, 4))
q5, err = r.move_to_pose(np.array([*tcp_place, HIGH_Z]), Rn, seed=q_now, speed=0.3)
q5, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.045]), Rn, seed=q5, speed=0.3)
q6, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.006]), Rn, seed=q5, speed=0.3)
r.gripper(0.04)
q7, err = r.move_to_pose(np.array([*tcp_place, 1.15]), Rn, seed=q6, speed=0.3)
q8, err = r.move_to_pose(np.array([-0.35, -0.30, 1.25]), R_down(0.0), seed=q7, speed=0.3)
log("PHASE1F DONE")
OPENRUA_EOF

# openrua op 74
nohup python3 -u phase1f_rotate_mug.py > phase1f.log 2>&1 &
sleep 150; cat phase1f.log

# openrua op 75
sleep 45; tail -n 5 phase1f.log; python3 cloud.py birdview >/dev/null && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np, math, cv2
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; rgb=d["rgb"]
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.35,-0.25);(u1,v1)=px(0.10,0.25)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
a=np.load("agentview_cloud.npz"); cv2.imwrite("agentview.png", cv2.cvtColor(a["rgb"], cv2.COLOR_RGB2BGR))
m=(P[:,2]>0.998)&(P[:,2]<1.03)&(P[:,0]<-0.10)&(P[:,0]>-0.35)&(P[:,1]<0.0)&(P[:,1]>-0.25)
Q=P[m]
for it in range(2):
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
    res=np.hypot(Q[:,0]-cx,Q[:,1]-cy)-r; Q=Q[np.abs(res)<0.006]
print(f"white rim centre ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(Q)} zmean {Q[:,2].mean():.3f} zmin {Q[:,2].min():.3f} zmax {Q[:,2].max():.3f}")
for name,P2 in (("bird",P),("agent",a["xyz"][a["valid"]])):
    rr=np.hypot(P2[:,0]-cx,P2[:,1]-cy)
    m2=(P2[:,2]>0.92)&(P2[:,2]<1.0)&(rr>0.052)&(rr<0.095)
    H=P2[m2]; az=np.degrees(np.arctan2(H[:,1]-cy,H[:,0]-cx))
    if len(H): print(name,"handle pts",len(H),"x",H[:,0].min().round(3),H[:,0].max().round(3),"y",H[:,1].min().round(3),H[:,1].max().round(3),"z",H[:,2].min().round(3),H[:,2].max().round(3),"az median",np.median(az).round(1))
# body extents
m=(P[:,2]>0.93)&(P[:,2]<1.05)&(P[:,0]<-0.10)&(P[:,0]>-0.35)&(P[:,1]<0.0)&(P[:,1]>-0.25)
B=P[m]; print("body x",B[:,0].min().round(3),B[:,0].max().round(3),"y",B[:,1].min().round(3),B[:,1].max().round(3))
EOF

# openrua op 76
cat > /workspace/phase2a_push_yellow.py <<'EOF'
#!/usr/bin/env python3
"""Push the fallen yellow mug +x out of the approach corridor."""
import numpy as np
from rob import *
r = Robot("push")
q0 = r.arm_q()
r.gripper(0.0)
R = R_from_axes(y_hand=np.array([0.0, 1.0, 0.0]), z_hand=np.array([0.0, 0.0, -1.0]))   # fingers along y, thin side along x
start = np.array([-0.105, 0.11, 1.15])
q1, _ = r.move_to_pose(start, R, seed=q0, speed=0.3, retries=6)
q2, _ = r.move_to_pose(np.array([-0.105, 0.11, 0.95]), R, seed=q1, speed=0.3, retries=6)
q3, _ = r.move_to_pose(np.array([0.09, 0.11, 0.95]), R, seed=q2, speed=0.15, retries=8)
q4, _ = r.move_to_pose(np.array([0.09, 0.11, 1.15]), R, seed=q3, speed=0.3, retries=6)
q5, _ = r.move_to_pose(np.array([-0.35, -0.30, 1.25]), R_down(0.0), seed=q4, speed=0.3, retries=6)
r.gripper(0.04)
log("PUSH DONE")
EOF
grep -n "def R_from_axes" -A 12 rob.py | head -16

# openrua op 77
nohup python3 -u phase2a_push_yellow.py > phase2a.log 2>&1 &
sleep 100; grep -v "IK attempt" phase2a.log

# openrua op 78
pkill -f phase2a_push_yellow.py; sleep 1; tail -n 3 phase2a.log; python3 - <<'EOF' 2>&1 | grep -v "^$"
import numpy as np
from rob import *
r=Robot("chk")
q=r.arm_q(); p,R=r.tcp(); print("q",np.round(q,3)); print("tcp",np.round(p,4),"y_hand",np.round(R[:,1],3),"z_hand",np.round(R[:,2],3))
for link in ("panda_link8","panda_link7","panda_link6","panda_link5","panda_link4"):
    pp,_=r.fk(q,link=link); print(link,np.round(pp,3))
EOF

# openrua op 79
pgrep -f phase2a_push_yellow.py || echo "stopped"; tail -n 3 phase2a.log; python3 - <<'EOF' 2>&1 | grep -v "^$"
import numpy as np
from rob import *
r=Robot("chk")
q=r.arm_q(); p,R=r.tcp(); print("q",np.round(q,3)); print("tcp",np.round(p,4),"y_hand",np.round(R[:,1],3),"z_hand",np.round(R[:,2],3))
for link in ("panda_link8","panda_link7","panda_link6","panda_link5","panda_link4"):
    pp,_=r.fk(q,link=link); print(link,np.round(pp,3))
EOF

# openrua op 80
pkill -9 -f "python3 -u phase2a" ; sleep 1; python3 - <<'EOF' 2>&1 | grep -v "IK attempt"
import numpy as np
from rob import *
r=Robot("recover")
q=r.arm_q(); p,R=r.tcp()
q1,_=r.move_to_pose(np.array([0.04,0.15,1.30]), R, seed=q, speed=0.2, retries=6)
q2,_=r.move_to_pose(np.array([-0.20,-0.10,1.30]), R_down(0.0), seed=q1, speed=0.2, retries=6)
q3,_=r.move_to_pose(np.array([-0.35,-0.30,1.25]), R_down(0.0), seed=q2, speed=0.2, retries=6)
r.gripper(0.04)
print("q", np.round(r.arm_q(),3))
EOF

# openrua op 81
python3 -u - > recover.log 2>&1 <<'EOF'
import numpy as np
from rob import *
r=Robot("recover")
q=r.arm_q(); p,R=r.tcp(); log("start tcp", np.round(p,4))
q1,_=r.move_to_pose(np.array([0.04,0.15,1.30]), R, seed=q, speed=0.2, retries=6)
q2,_=r.move_to_pose(np.array([-0.20,-0.10,1.30]), R_down(0.0), seed=q1, speed=0.2, retries=6)
q3,_=r.move_to_pose(np.array([-0.35,-0.30,1.25]), R_down(0.0), seed=q2, speed=0.2, retries=6)
r.gripper(0.04)
log("q", np.round(r.arm_q(),3))
EOF
grep -v "IK attempt" recover.log

# openrua op 82
python3 cloud.py birdview >/dev/null && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np, math, cv2
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; rgb=d["rgb"]
cv2.imwrite("birdview.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
a=np.load("agentview_cloud.npz"); cv2.imwrite("agentview.png", cv2.cvtColor(a["rgb"], cv2.COLOR_RGB2BGR))
def box(P,x0,x1,y0,y1,z0,z1):
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>z0)&(P[:,2]<z1); q=P[m]
    return (len(q), q[:,0].min().round(3),q[:,0].max().round(3),q[:,1].min().round(3),q[:,1].max().round(3),q[:,2].min().round(3),q[:,2].max().round(3)) if len(q) else (0,)
print("white", box(P,-0.35,-0.10,-0.25,-0.0,0.93,1.05))
print("yellow", box(P,-0.15,0.30,-0.05,0.235,0.93,1.06))
print("microwave top", box(P,-0.25,0.20,0.20,0.5,1.09,1.2))
print("door", box(P,-0.25,-0.10,-0.05,0.25,1.09,1.2))
EOF

# openrua op 83
python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); rgb=d["rgb"]; xyz=d["xyz"]; v=d["valid"]
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.35,-0.30);(u1,v1)=px(0.25,0.55)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
P=xyz[v]
# profile along y at x=-0.05 for z>0.95: where does the microwave start?
for y0 in np.arange(0.15,0.30,0.01):
    m=(np.abs(P[:,0]+0.05)<0.01)&(np.abs(P[:,1]-y0)<0.005)&(P[:,2]>0.92)
    if m.sum(): print(f"y={y0:.2f}: n={m.sum()} zmax={P[m][:,2].max():.3f}")
# points in front face region: y in [0.19,0.25], z between 0.95 and 1.09 (a vertical face seen from above yields few points)
m=(P[:,1]>0.17)&(P[:,1]<0.26)&(P[:,2]>0.95)&(P[:,2]<1.095)&(P[:,0]>-0.25)&(P[:,0]<0.2)
Q=P[m]; print("front-ish pts",len(Q)); 
if len(Q): print("x",Q[:,0].min().round(3),Q[:,0].max().round(3),"y",Q[:,1].min().round(3),Q[:,1].max().round(3))
EOF

# openrua op 84
python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"][d["valid"]]; C=d["rgb"][d["valid"]]
    m=(P[:,0]>-0.10)&(P[:,0]<0.12)&(P[:,1]>-0.02)&(P[:,1]<0.185)&(P[:,2]>0.915)&(P[:,2]<1.08)
    Q=P[m]; K=C[m]
    yellow=(K[:,0]>150)&(K[:,1]>120)&(K[:,2]<120)
    print(cam,"pts",len(Q),"yellow",yellow.sum())
    Y=Q[yellow]
    print(" yellow x",Y[:,0].min().round(3),Y[:,0].max().round(3),"y",Y[:,1].min().round(3),Y[:,1].max().round(3),"z",Y[:,2].min().round(3),Y[:,2].max().round(3))
    # z profile along y (max z per y bin), yellow only
    for y0 in np.arange(0.0,0.18,0.02):
        k=(Y[:,1]>=y0)&(Y[:,1]<y0+0.02)
        if k.sum(): print(f"  y[{y0:.2f},{y0+0.02:.2f}] n={k.sum()} x[{Y[k][:,0].min():.3f},{Y[k][:,0].max():.3f}] zmax={Y[k][:,2].max():.3f}")
    top=Y[Y[:,2]>1.03]
    if len(top): print(" top pts (z>1.03):",len(top),"x",top[:,0].min().round(3),top[:,0].max().round(3),"y",top[:,1].min().round(3),top[:,1].max().round(3))
EOF

# openrua op 85
python3 - <<'EOF'
s=open("/workspace/rob.py").read()
s=s.replace("""    def gripper(self, width, timeout=300):""","""    def move_line(self, p_to, R, step=0.03, speed=0.15, retries=6, tol=0.006, R_to=None):
        \"\"\"Straight-line TCP move from the current pose to p_to (optionally
        interpolating hand yaw about world z to R_to), IK per waypoint seeded
        by the previous, sent as one trajectory; then settle on the last point.\"\"\"
        p0, _ = self.tcp()
        p_to = np.asarray(p_to, float)
        n = max(1, int(np.ceil(np.linalg.norm(p_to - p0) / step)))
        if R_to is not None:
            a0 = math.atan2(R[1, 1], R[0, 1]); a1 = math.atan2(R_to[1, 1], R_to[0, 1])
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            n = max(n, int(np.ceil(abs(da) / 0.15)))
        seed = self.arm_q()
        wps, durs, t = [], [], 0.0
        for i in range(1, n + 1):
            f = i / n
            p = p0 + f * (p_to - p0)
            Ri = R
            if R_to is not None:
                c, s_ = math.cos(f * da), math.sin(f * da)
                Rz = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
                Ri = Rz @ R
            q = self.ik(p, Ri, seed=seed, max_dev=0.5)
            if q is None:
                raise RuntimeError(f"IK failed on line at {p}")
            t += max(0.6, np.abs(q - seed).max() / speed)
            wps.append(q); durs.append(round(t, 2)); seed = q
        return self.move_joints(wps, durs, speed=speed, retries=retries, tol=tol)

    def gripper(self, width, timeout=300):""")
open("/workspace/rob.py","w").write(s)
EOF
python3 -c "import rob" && echo ok

# openrua op 86
mkdir -p "$(dirname /workspace/phase2b_clear.py)"
cat > /workspace/phase2b_clear.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 2b: (1) carry the fallen yellow mug away by its handle; (2) move the
white mug into the insertion corridor at (-0.02, -0.25) with a rim pinch."""
import math
import sys
import numpy as np
from rob import *

r = Robot("phase2b")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)

# ---------- yellow mug ----------
YH = np.array([-0.011, 0.078])        # handle top bar (runs along y), top z 1.035
Ry = R_down(math.pi / 2)              # fingers along x
log("hand y", np.round(Ry[:, 1], 3))
q1, _ = r.move_to_pose(np.array([*YH, 1.20]), Ry, seed=q0, speed=0.3, retries=6)
r.move_line(np.array([*YH, 1.012]), Ry)
t, _ = r.tcp(); log("at yellow handle, tcp", np.round(t, 4))
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air (yellow)"); r.gripper(0.04); sys.exit(2)
r.move_line(np.array([*YH, 1.20]), Ry)
log("yellow lifted, gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! yellow slipped"); r.gripper(0.04); sys.exit(5)
YP = np.array([-0.35, -0.40])
r.move_line(np.array([*YP, 1.20]), Ry, step=0.05)
r.move_line(np.array([*YP, 0.965]), Ry)
r.gripper(0.04)
r.move_line(np.array([*YP, 1.20]), Ry)
log("yellow parked")

# ---------- white mug ----------
C = np.array([-0.222, -0.123])
S = np.array([-0.02, -0.25])
Rw = R_down(0.0)                      # fingers along y
tcp_xy = C + np.array([0.0, 0.044])   # pinch the +y rim point (handle is at -y)
r.move_line(np.array([*tcp_xy, 1.20]), Rw, step=0.05)
r.move_line(np.array([*tcp_xy, 0.985]), Rw)
t, _ = r.tcp(); log("at white rim, tcp", np.round(t, 4))
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air (white)"); r.gripper(0.04); sys.exit(2)
r.move_line(np.array([*tcp_xy, 1.07]), Rw)
log("white lifted, gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! white slipped"); r.gripper(0.04); sys.exit(5)
place = S + np.array([0.0, 0.044])
r.move_line(np.array([*place, 1.07]), Rw, step=0.05)
r.move_line(np.array([*place, 0.992]), Rw)
r.gripper(0.04)
r.move_line(np.array([*place, 1.20]), Rw)
r.move_line(np.array([-0.35, -0.20, 1.25]), Rw, step=0.05)
log("PHASE2B DONE")
OPENRUA_EOF

# openrua op 87
nohup python3 -u phase2b_clear.py > phase2b.log 2>&1 &
sleep 120; grep -v "IK attempt" phase2b.log

# openrua op 88
sleep 120; grep -v "IK attempt" phase2b.log | tail -n +16

# openrua op 89
sleep 75; grep -v "IK attempt" phase2b.log | tail -n +36

# openrua op 90
sleep 30; tail -n 2 phase2b.log; python3 cloud.py birdview >/dev/null && python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np, math, cv2
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; rgb=d["rgb"]
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.55,-0.55);(u1,v1)=px(0.25,0.55)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
a=np.load("agentview_cloud.npz"); cv2.imwrite("agentview.png", cv2.cvtColor(a["rgb"], cv2.COLOR_RGB2BGR))
m=(P[:,2]>0.998)&(P[:,2]<1.03)&(P[:,0]<0.10)&(P[:,0]>-0.15)&(P[:,1]<-0.10)&(P[:,1]>-0.40)
Q=P[m]
for it in range(2):
    A=np.c_[2*Q[:,0],2*Q[:,1],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=math.sqrt(c+cx*cx+cy*cy)
    res=np.hypot(Q[:,0]-cx,Q[:,1]-cy)-r; Q=Q[np.abs(res)<0.006]
print(f"white rim centre ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(Q)} z {Q[:,2].min():.3f}-{Q[:,2].max():.3f}")
for name,P2 in (("bird",P),("agent",a["xyz"][a["valid"]])):
    rr=np.hypot(P2[:,0]-cx,P2[:,1]-cy)
    m2=(P2[:,2]>0.92)&(P2[:,2]<1.0)&(rr>0.052)&(rr<0.095)
    H=P2[m2]; az=np.degrees(np.arctan2(H[:,1]-cy,H[:,0]-cx))
    if len(H): print(name,"handle pts",len(H),"x",H[:,0].min().round(3),H[:,0].max().round(3),"y",H[:,1].min().round(3),H[:,1].max().round(3),"z",H[:,2].min().round(3),H[:,2].max().round(3),"az median",np.median(az).round(1))
    # handle top bar: z>0.975
    T=H[H[:,2]>0.975]
    if len(T): print("   top bar x",T[:,0].min().round(3),T[:,0].max().round(3),"y",T[:,1].min().round(3),T[:,1].max().round(3))
m=(P[:,2]>0.93)&(P[:,2]<1.06)&(P[:,0]<-0.2)&(P[:,0]>-0.5)&(P[:,1]<-0.25)&(P[:,1]>-0.55)
Y=P[m]; print("yellow x",Y[:,0].min().round(3),Y[:,0].max().round(3),"y",Y[:,1].min().round(3),Y[:,1].max().round(3),"zmax",Y[:,2].max().round(3))
EOF

# openrua op 91
python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"][d["valid"]]
    m=(P[:,0]>-0.25)&(P[:,0]<0.2)&(P[:,1]>0.12)&(P[:,1]<0.30)&(P[:,2]>0.92)&(P[:,2]<1.095)
    Q=P[m]
    print(cam,"front region pts",len(Q))
    # y histogram
    for y0 in np.arange(0.14,0.26,0.01):
        k=(Q[:,1]>=y0)&(Q[:,1]<y0+0.01)
        if k.sum(): print(f"  y[{y0:.2f}] n={k.sum()} x[{Q[k][:,0].min():.3f},{Q[k][:,0].max():.3f}] z[{Q[k][:,2].min():.3f},{Q[k][:,2].max():.3f}]")
    H=Q[Q[:,1]<0.185]
    if len(H): print("  handle-ish (y<0.185): n",len(H),"x",H[:,0].min().round(3),H[:,0].max().round(3),"y",H[:,1].min().round(3),H[:,1].max().round(3),"z",H[:,2].min().round(3),H[:,2].max().round(3))
EOF

# openrua op 92
mkdir -p "$(dirname /workspace/phase2c_open_door.py)"
cat > /workspace/phase2c_open_door.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 2c: open the microwave door by pinching its handle slab and pulling
along the hinge arc."""
import math
import sys
import numpy as np
from rob import *

HINGE = np.array([-0.19, 0.235])
TCP0 = np.array([0.056, 0.21])        # pinch point on the handle slab (closed door)
Z = 1.02
THETA_END = math.radians(88.0)
STEP = math.radians(10.0)


def R_theta(th):
    z = np.array([math.sin(th), math.cos(th), 0.0])
    y = np.array([math.cos(th), -math.sin(th), 0.0])
    return R_from_axes(y_hand=y, z_hand=z)


def tcp_theta(th):
    v = TCP0 - HINGE
    c, s = math.cos(-th), math.sin(-th)
    return HINGE + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


r = Robot("phase2c")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)
R0 = R_theta(0.0)
log("R0 y", np.round(R0[:, 1], 3), "z", np.round(R0[:, 2], 3), "x", np.round(R0[:, 0], 3))
q1, _ = r.move_to_pose(np.array([TCP0[0], 0.05, 1.25]), R0, seed=q0, speed=0.3, retries=6)
r.move_line(np.array([TCP0[0], 0.05, Z]), R0)
r.move_line(np.array([TCP0[0], TCP0[1], Z]), R0, step=0.02, speed=0.1)
t, Rt = r.tcp(); log("at handle tcp", np.round(t, 4), "z_hand", np.round(Rt[:, 2], 3))
gap = r.gripper(0.0)
log("gap on handle", round(gap, 4))
if gap < 0.005:
    log("!! missed handle"); r.gripper(0.04); sys.exit(2)

th = 0.0
while th < THETA_END - 1e-6:
    th_new = min(th + STEP, THETA_END)
    p = tcp_theta(th_new)
    err = r.move_line(np.array([p[0], p[1], Z]), R_theta(th), R_to=R_theta(th_new), step=0.02, speed=0.12, retries=4)
    t, Rt = r.tcp()
    log(f"theta {math.degrees(th_new):.0f}: tcp {np.round(t,4)} gap {r.finger_gap():.4f} err {err:.4f}")
    if r.finger_gap() < 0.004:
        log("!! lost the handle"); break
    th = th_new

r.gripper(0.04)
t, Rt = r.tcp()
back = t - 0.06 * Rt[:, 2]
r.move_line(back, Rt)
r.move_line(np.array([back[0], back[1], 1.25]), Rt)
log("PHASE2C DONE")
OPENRUA_EOF

# openrua op 93
nohup python3 -u phase2c_open_door.py > phase2c.log 2>&1 &
sleep 150; grep -v "IK attempt" phase2c.log

# openrua op 94
sleep 180; grep "theta\|!!\|DONE\|gripper" phase2c.log

# openrua op 95
sleep 200; grep "theta\|!!\|DONE\|gripper" phase2c.log | tail -n 6

# openrua op 96
sleep 25; tail -n 2 phase2c.log; python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np, math, cv2
d=np.load("birdview_cloud.npz"); P=d["xyz"][d["valid"]]; rgb=d["rgb"]
def px(x,y): return int(320+y/0.00362), int(240+(x+0.2)/0.00362)
(u0,v0)=px(-0.55,-0.55);(u1,v1)=px(0.25,0.55)
crop=cv2.resize(rgb[v0:v1,u0:u1],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST)
cv2.imwrite("crop_bird.png", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
# door top edge: points z in [1.09,1.12] with y<0.23 (in front of microwave body)
m=(P[:,2]>1.085)&(P[:,2]<1.13)&(P[:,1]<0.225)&(P[:,0]>-0.4)&(P[:,0]<0.2)
D=P[m]; print("door top pts",len(D))
if len(D):
    print(" x",D[:,0].min().round(3),D[:,0].max().round(3),"y",D[:,1].min().round(3),D[:,1].max().round(3))
    # fit line x = a*y + b
    a,b=np.polyfit(D[:,1],D[:,0],1); print(f" line x = {a:.3f}*y + {b:.3f}; angle from -y axis: {math.degrees(math.atan(a)):.1f} deg")
    for y0 in np.arange(-0.05,0.25,0.05):
        k=np.abs(D[:,1]-y0)<0.01
        if k.sum(): print(f"  y~{y0:.2f}: x[{D[k][:,0].min():.3f},{D[k][:,0].max():.3f}]")
# any points in the opening region (x -0.16..0.08, y 0.24..0.45, z 0.95..1.08) -> cavity floor visible?
m=(P[:,0]>-0.16)&(P[:,0]<0.08)&(P[:,1]>0.25)&(P[:,1]<0.45)&(P[:,2]<1.08)
print("cavity pts",m.sum(), "z range", (P[m][:,2].min().round(3),P[m][:,2].max().round(3)) if m.sum() else None)
EOF

# openrua op 97
python3 -u - > recover2.log 2>&1 <<'EOF'
import numpy as np
from rob import *
r=Robot("recover2")
q=r.arm_q(); p,R=r.tcp(); log("start tcp", np.round(p,4), "z_hand", np.round(R[:,2],3))
r.move_line(np.array([p[0],-0.13,p[2]]), R, step=0.03, speed=0.15)
q1,_=r.move_to_pose(np.array([-0.30,-0.20,1.25]), R_down(0.0), seed=r.arm_q(), speed=0.2, retries=6)
log("q", np.round(r.arm_q(),3))
EOF
grep -v "IK attempt" recover2.log

# openrua op 98
python3 cloud.py agentview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"][d["valid"]]
    m=(P[:,0]>-0.16)&(P[:,0]<0.08)&(P[:,1]>0.25)&(P[:,1]<0.46)&(P[:,2]<1.085)
    Q=P[m]
    h,e=np.histogram(Q[:,2],bins=np.arange(0.88,1.10,0.01))
    print(cam, [(round(e[i],2),int(h[i])) for i in range(len(h)) if h[i]>0])
    fl=Q[(Q[:,2]>0.93)&(Q[:,2]<0.96)]
    if len(fl): print("  floor-ish pts x",fl[:,0].min().round(3),fl[:,0].max().round(3),"y",fl[:,1].min().round(3),fl[:,1].max().round(3),"z med",np.median(fl[:,2]).round(3))
    # opening top edge: points at y in [0.23,0.26], z>1.05
    m=(P[:,0]>-0.16)&(P[:,0]<0.08)&(P[:,1]>0.22)&(P[:,1]<0.27)&(P[:,2]>1.0)&(P[:,2]<1.12)
    T=P[m]
    if len(T):
        h,e=np.histogram(T[:,2],bins=np.arange(1.0,1.12,0.01)); print("  front top edge z hist",[(round(e[i],2),int(h[i])) for i in range(len(h)) if h[i]>0])
EOF

# openrua op 99
mkdir -p "$(dirname /workspace/phase2d_insert.py)"
cat > /workspace/phase2d_insert.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 2d: grasp the white mug's handle with a horizontal hand from -y,
lift, check tilt, slide it into the microwave cavity, release, retreat."""
import math
import subprocess
import sys
import numpy as np
from rob import *

C = np.array([-0.0218, -0.2426])          # mug centre (rim fit)
BAR_X = -0.021                            # handle top bar x
TIP_Y = -0.300                            # fingertip y at grasp
GRASP_Z = 0.980
LIFT = 0.058                              # -> mug bottom ~0.958
TARGET_C_Y = 0.32                         # mug centre y inside cavity
OFF = C[1] - TIP_Y                        # TCP->mug centre offset along y

R = R_from_axes(y_hand=np.array([1.0, 0.0, 0.0]), z_hand=np.array([0.0, 1.0, 0.0]))
log("R y", np.round(R[:, 1], 3), "z", np.round(R[:, 2], 3), "x", np.round(R[:, 0], 3))


def mug_extents(tag):
    subprocess.run(["python3", "cloud.py", "agentview"], capture_output=True)
    d = np.load("agentview_cloud.npz"); P = d["xyz"][d["valid"]]; K = d["rgb"][d["valid"]]
    t, _ = r.tcp()
    cx, cy = t[0], t[1] + OFF
    rr = np.hypot(P[:, 0] - cx, P[:, 1] - cy)
    m = (rr < 0.06) & (P[:, 2] > t[2] - 0.15) & (P[:, 2] < t[2] + 0.15) & (P[:, 1] > t[1] + 0.02)
    Q = P[m]
    if len(Q) < 20:
        log(tag, "mug not seen", len(Q)); return None
    log(f"{tag}: mug pts {len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] z[{Q[:,2].min():.3f},{Q[:,2].max():.3f}] (tcp z {t[2]:.3f})")
    return Q


r = Robot("phase2d")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)

pre = np.array([BAR_X, -0.46, 1.20])
q1, _ = r.move_to_pose(pre, R, seed=q0, speed=0.3, retries=6)
r.move_line(np.array([BAR_X, -0.46, GRASP_Z]), R)
r.move_line(np.array([BAR_X, TIP_Y, GRASP_Z]), R, step=0.02, speed=0.1)
t, Rt = r.tcp(); log("at handle tcp", np.round(t, 4), "z_hand", np.round(Rt[:, 2], 3))
gap = r.gripper(0.0)
log("gap on mug handle", round(gap, 4))
if gap < 0.004:
    log("!! missed handle"); r.gripper(0.04); r.move_line(np.array([BAR_X, -0.46, GRASP_Z]), R); sys.exit(2)

r.move_line(np.array([BAR_X, TIP_Y, GRASP_Z + LIFT]), R, speed=0.08)
log("lifted; gap", round(r.finger_gap(), 4))
Q = mug_extents("lifted")
if r.finger_gap() < 0.004 or Q is None or Q[:, 2].min() < 0.945 or Q[:, 2].max() > 1.075:
    log("!! bad hold (slipped or tilted); putting it back")
    r.move_line(np.array([BAR_X, TIP_Y, GRASP_Z]), R, speed=0.08)
    r.gripper(0.04)
    r.move_line(np.array([BAR_X, -0.46, GRASP_Z]), R)
    sys.exit(5)

# transport: straight +y into the cavity
z = GRASP_Z + LIFT
r.move_line(np.array([BAR_X, 0.10, z]), R, step=0.04, speed=0.12)
Q = mug_extents("in front of opening")
if Q is not None and (Q[:, 2].min() < 0.947 or Q[:, 2].max() > 1.08):
    log("!! mug too tall/low for the opening; abort before insertion"); sys.exit(6)
tip_target_y = TARGET_C_Y - OFF
r.move_line(np.array([BAR_X, tip_target_y, z]), R, step=0.02, speed=0.08)
t, _ = r.tcp(); log("inserted; tcp", np.round(t, 4), "gap", round(r.finger_gap(), 4))
r.move_line(np.array([BAR_X, tip_target_y, GRASP_Z + 0.045 - 0.041]), R, speed=0.05)   # bottom to ~0.944 floor
r.gripper(0.04)
r.move_line(np.array([BAR_X, 0.10, GRASP_Z + 0.02]), R, step=0.03, speed=0.1)
r.move_line(np.array([BAR_X, 0.05, 1.20]), R)
log("PHASE2D DONE")
OPENRUA_EOF

# openrua op 100
python3 - <<'EOF'
s=open("/workspace/phase2d_insert.py").read()
s=s.replace("""r.move_line(np.array([BAR_X, tip_target_y, GRASP_Z + 0.045 - 0.041]), R, speed=0.05)   # bottom to ~0.944 floor
r.gripper(0.04)
r.move_line(np.array([BAR_X, 0.10, GRASP_Z + 0.02]), R, step=0.03, speed=0.1)""",
"""zr = z - 0.012                                                            # mug bottom ~2 mm above the 0.944 floor
r.move_line(np.array([BAR_X, tip_target_y, zr]), R, speed=0.05)
r.gripper(0.04)
r.move_line(np.array([BAR_X, 0.10, zr]), R, step=0.03, speed=0.1)""")
open("/workspace/phase2d_insert.py","w").write(s)
EOF
grep -n "zr" phase2d_insert.py

# openrua op 101
nohup python3 -u phase2d_insert.py > phase2d.log 2>&1 &
sleep 170; grep -v "IK attempt" phase2d.log

# openrua op 102
python3 - <<'EOF'
s=open("/workspace/phase2d_insert.py").read()
s=s.replace("pre = np.array([BAR_X, -0.46, 1.20])","pre = np.array([BAR_X, -0.36, 1.10])")
s=s.replace("r.move_line(np.array([BAR_X, -0.46, GRASP_Z]), R)\nr.move_line(np.array([BAR_X, TIP_Y, GRASP_Z]), R, step=0.02, speed=0.1)",
            "r.move_line(np.array([BAR_X, -0.36, GRASP_Z]), R)\nr.move_line(np.array([BAR_X, TIP_Y, GRASP_Z]), R, step=0.02, speed=0.1)")
s=s.replace("r.move_line(np.array([BAR_X, -0.46, GRASP_Z]), R); sys.exit(2)","r.move_line(np.array([BAR_X, -0.36, GRASP_Z]), R); sys.exit(2)")
s=s.replace("    r.move_line(np.array([BAR_X, -0.46, GRASP_Z]), R)\n    sys.exit(5)","    r.move_line(np.array([BAR_X, -0.36, GRASP_Z]), R)\n    sys.exit(5)")
open("/workspace/phase2d_insert.py","w").write(s)
EOF
grep -n "0.46" phase2d_insert.py; nohup python3 -u phase2d_insert.py > phase2d.log 2>&1 &
sleep 150; grep -v "IK attempt" phase2d.log

# openrua op 103
python3 - <<'EOF' 2>&1 | grep -v "IK attempt\|IK:"
import numpy as np, math
from rob import *
r=Robot("iktest2")
q=r.arm_q()
Rup = R_from_axes(y_hand=np.array([1.0,0,0]), z_hand=np.array([0,1.0,0]))
Rdn = R_from_axes(y_hand=np.array([-1.0,0,0]), z_hand=np.array([0,1.0,0]))
for name,R in (("x up",Rup),("x down",Rdn)):
    for p in ([-0.021,-0.30,0.98],[-0.021,-0.36,0.98],[-0.021,-0.36,1.10],[-0.021,-0.30,1.038],[-0.021,0.10,1.038],[-0.021,0.263,1.038],[-0.10,-0.30,0.98]):
        qs=r.ik(np.array(p), R, seed=q, attempts=6)
        print(name, p, "OK" if qs is not None else "FAIL", None if qs is None else np.round(qs,2))
EOF

# openrua op 104
timeout 100 python3 - <<'EOF' 2>&1 | grep -v "IK attempt\|IK:"
import numpy as np, math
from rob import *
r=Robot("iktest3")
q=r.arm_q()
Rup = R_from_axes(y_hand=np.array([1.0,0,0]), z_hand=np.array([0,1.0,0]))
for p in ([-0.021,-0.21,0.98],[-0.021,-0.16,0.98],[-0.021,-0.21,1.10],[-0.021,-0.26,0.98]):
    qs=r.ik(np.array(p), Rup, seed=q, attempts=3)
    print(p, "OK" if qs is not None else "FAIL", None if qs is None else np.round(qs,2))
EOF

# openrua op 105
timeout 170 python3 - <<'EOF' 2>&1 | grep -v "IK attempt\|IK:"
import numpy as np, math
from rob import *
r=Robot("iktest4")
q=r.arm_q()
for phi in (20,30,40):
    f=math.radians(phi)
    R = R_from_axes(y_hand=np.array([1.0,0,0]), z_hand=np.array([0,math.cos(f),-math.sin(f)]))
    for p in ([-0.021,-0.30,0.98],[-0.021,-0.24,0.98]):
        qs=r.ik(np.array(p), R, seed=q, attempts=2)
        print(phi, p, "OK" if qs is not None else "FAIL", None if qs is None else np.round(qs,2))
EOF

# openrua op 106
python3 - <<'EOF'
s=open("/workspace/phase2d_insert.py").read()
s=s.replace("""R = R_from_axes(y_hand=np.array([1.0, 0.0, 0.0]), z_hand=np.array([0.0, 1.0, 0.0]))
log("R y", np.round(R[:, 1], 3), "z", np.round(R[:, 2], 3), "x", np.round(R[:, 0], 3))
""","""PHI = math.radians(40.0)                  # hand pitched down 40 deg (reachability)
R = R_from_axes(y_hand=np.array([1.0, 0.0, 0.0]), z_hand=np.array([0.0, math.cos(PHI), -math.sin(PHI)]))
log("R y", np.round(R[:, 1], 3), "z", np.round(R[:, 2], 3), "x", np.round(R[:, 0], 3))
GRASP = np.array([BAR_X, TIP_Y, GRASP_Z])
PRE = GRASP - 0.08 * R[:, 2]
""")
s=s.replace("""pre = np.array([BAR_X, -0.36, 1.10])
q1, _ = r.move_to_pose(pre, R, seed=q0, speed=0.3, retries=6)
r.move_line(np.array([BAR_X, -0.36, GRASP_Z]), R)
r.move_line(np.array([BAR_X, TIP_Y, GRASP_Z]), R, step=0.02, speed=0.1)""",
"""log("pre", np.round(PRE, 4))
q1, _ = r.move_to_pose(PRE, R, seed=q0, speed=0.3, retries=6)
r.move_line(GRASP, R, step=0.02, speed=0.08)""")
s=s.replace("""    log("!! missed handle"); r.gripper(0.04); r.move_line(np.array([BAR_X, -0.36, GRASP_Z]), R); sys.exit(2)""",
"""    log("!! missed handle"); r.gripper(0.04); r.move_line(PRE, R); sys.exit(2)""")
s=s.replace("""    r.move_line(np.array([BAR_X, -0.36, GRASP_Z]), R)
    sys.exit(5)""","""    r.move_line(PRE, R)
    sys.exit(5)""")
s=s.replace("r.move_line(np.array([BAR_X, 0.05, 1.20]), R)","r.move_line(np.array([BAR_X, 0.05, 1.15]), R)")
open("/workspace/phase2d_insert.py","w").write(s)
EOF
grep -n "PRE\|PHI\|0.36" phase2d_insert.py

# openrua op 107
python3 - <<'EOF'
s=open("/workspace/phase2d_insert.py").read()
s=s.replace("""q1, _ = r.move_to_pose(PRE, R, seed=q0, speed=0.3, retries=6)
""","""q1, _ = r.move_to_pose(np.array([-0.10, -0.36, 1.12]), R, seed=q0, speed=0.3, retries=6)
r.move_line(PRE, R, step=0.03, speed=0.1)
""")
open("/workspace/phase2d_insert.py","w").write(s)
EOF
nohup python3 -u phase2d_insert.py > phase2d.log 2>&1 &
sleep 120; grep -v "IK attempt" phase2d.log

# openrua op 108
python3 - <<'EOF'
s=open("/workspace/rob.py").read()
s=s.replace("""            if k > 0:  # perturb seed on retry
                s = s + np.random.uniform(-0.3, 0.3, size=7)
                s = np.clip(s, [l[0] + 0.05 for l in LIMITS], [l[1] - 0.05 for l in LIMITS])""",
"""            if k > 0:  # perturb seed on retry (wider each time)
                s = s + np.random.uniform(-0.3 * k, 0.3 * k, size=7)
                s = np.clip(s, [l[0] + 0.05 for l in LIMITS], [l[1] - 0.05 for l in LIMITS])""")
s=s.replace("""            q = self.ik(p, Ri, seed=seed, max_dev=0.5)
            if q is None:
                raise RuntimeError(f"IK failed on line at {p}")""",
"""            q = self.ik(p, Ri, seed=seed, max_dev=0.5)
            if q is None:
                q = self.ik(p, Ri, seed=seed, max_dev=0.5, attempts=8)
            if q is None:
                raise RuntimeError(f"IK failed on line at {p}")""")
open("/workspace/rob.py","w").write(s)
EOF
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "IK attempt"
import numpy as np, math
from rob import *
r=Robot("iktest5")
q=r.arm_q(); p,_=r.tcp(); print("tcp",np.round(p,4))
PHI=math.radians(40)
R = R_from_axes(y_hand=np.array([1.0,0,0]), z_hand=np.array([0,math.cos(PHI),-math.sin(PHI)]))
for pt in ([-0.060,-0.361,1.076],[-0.021,-0.3613,1.0314],[-0.021,-0.30,0.98]):
    qs=r.ik(np.array(pt), R, seed=q, attempts=8, max_dev=0.5)
    print(pt, None if qs is None else np.round(qs,2))
EOF

# openrua op 109
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "IK attempt\|IK:"
import numpy as np, math
from rob import *
r=Robot("iktest6")
q=r.arm_q()
PHI=math.radians(40)
R = R_from_axes(y_hand=np.array([1.0,0,0]), z_hand=np.array([0,math.cos(PHI),-math.sin(PHI)]))
seed=np.array([-0.83,1.39,0.53,-0.64,0.18,1.47,-1.39])  # known solution at grasp
for z in (1.12,1.08,1.04,1.01,0.98):
    row=[]
    for y in (-0.36,-0.33,-0.30):
        for x in (-0.10,-0.06,-0.021):
            qs=r.ik(np.array([x,y,z]), R, seed=seed, attempts=3, max_dev=0.6)
            row.append("O" if qs is not None else ".")
    print(f"z={z}: y=-0.36[x -0.10,-0.06,-0.021] {''.join(row[0:3])}  y=-0.33 {''.join(row[3:6])}  y=-0.30 {''.join(row[6:9])}")
EOF

# openrua op 110
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/phase2d_insert.py", "old_string": "GRASP = np.array([BAR_X, TIP_Y, GRASP_Z])\\nPRE = GRASP - 0.08 * R[:, 2]", "new_string": "GRASP = np.array([BAR_X, TIP_Y, GRASP_Z])\\n# points further back along -z_hand are unreachable (wrist too far from the base), but the\\n# whole column above the grasp is reachable: approach from above, fingers straddling the bar in x\\nPRE = np.array([BAR_X, TIP_Y, 1.04])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 111
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/phase2d_insert.py", "old_string": "q1, _ = r.move_to_pose(np.array([-0.10, -0.36, 1.12]), R, seed=q0, speed=0.3, retries=6)\\nr.move_line(PRE, R, step=0.03, speed=0.1)\\nr.move_line(GRASP, R, step=0.02, speed=0.08)", "new_string": "q1, _ = r.move_to_pose(np.array([-0.10, -0.36, 1.12]), R, seed=q0, speed=0.3, retries=6)\\nr.move_line(np.array([-0.10, TIP_Y, 1.10]), R, step=0.03, speed=0.1)\\nr.move_line(PRE, R, step=0.03, speed=0.1)\\nr.move_line(GRASP, R, step=0.015, speed=0.06, retries=8, tol=0.005)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 112
nohup python3 -u phase2d_insert.py > phase2d.log 2>&1 &
sleep 240; grep -v "IK attempt\|IK:" /workspace/phase2d.log | tail -30

# openrua op 113
pgrep -f phase2d_insert.py; python3 - <<'EOF'
import re
s=open("phase2d_insert.py").read()
s=s.replace("m = (rr < 0.06) & (P[:, 2] > t[2] - 0.15) & (P[:, 2] < t[2] + 0.15) & (P[:, 1] > t[1] + 0.02)",
            "m = (rr < 0.06) & (P[:, 2] > 0.93) & (P[:, 2] < t[2] + 0.15) & (P[:, 1] > t[1] + 0.02)   # z>0.93 excludes the table")
s=s.replace("if r.finger_gap() < 0.004 or Q is None or Q[:, 2].min() < 0.945 or Q[:, 2].max() > 1.075:",
            "if r.finger_gap() < 0.004 or Q is None or Q[:, 2].max() < 1.05 or Q[:, 2].max() > 1.08:")
open("phase2d_insert.py","w").write(s)
EOF
grep -n "0.93\|1.05" phase2d_insert.py

# openrua op 114
while pgrep -f phase2d_insert.py >/dev/null; do sleep 2; done; tail -2 phase2d.log; nohup python3 -u phase2d_insert.py > phase2d.log 2>&1 &
sleep 200; grep -v "IK attempt\|IK:" /workspace/phase2d.log | tail -30

# openrua op 115
pgrep -af phase2d; grep -v "IK attempt\|IK:" /workspace/phase2d.log | tail -30

# openrua op 116
kill 3004 2>/dev/null; sleep 1; pgrep -af "python3 -u phase2d" ; ps aux | grep -c "[p]ython3 -u phase2d"

# openrua op 117
ps aux | grep "[p]ython3 -u phase2d" | grep -v bash; echo ---; nohup python3 -u phase2d_insert.py > phase2d.log 2>&1 &
sleep 150; grep -v "IK attempt\|IK:" /workspace/phase2d.log | tail -30

# openrua op 118
sleep 45; grep -v "IK attempt\|IK:" /workspace/phase2d.log | tail -6; ps aux | grep -c "[p]ython3 -u phase2d_insert"

# openrua op 119
python3 cloud.py agentview >/dev/null && python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
for cam in ("agentview","birdview"):
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"][d["valid"]]; K=d["rgb"][d["valid"]]
    # points inside the cavity footprint
    m=(P[:,0]>-0.15)&(P[:,0]<0.07)&(P[:,1]>0.25)&(P[:,1]<0.45)&(P[:,2]>0.95)&(P[:,2]<1.08)
    Q=P[m]; C=K[m]
    print(cam,"cavity pts",len(Q))
    if len(Q):
        white=(C.min(axis=1)>150)
        W=Q[white]; print(" white pts",len(W), "x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(W[:,0].min(),W[:,0].max(),W[:,1].min(),W[:,1].max(),W[:,2].min(),W[:,2].max()) if len(W) else "")
    # anything white left near the old mug spot
    m2=(np.hypot(P[:,0]+0.02,P[:,1]+0.24)<0.08)&(P[:,2]>0.92)
    print(" pts at old spot above table:",m2.sum())
    # door top edge
    m3=(P[:,2]>1.05)&(P[:,2]<1.12)&(P[:,0]<-0.14)&(P[:,0]>-0.25)&(P[:,1]>-0.05)&(P[:,1]<0.24)
    D=P[m3]; print(" door-edge pts",len(D), "x[%.3f,%.3f] y[%.3f,%.3f]"%(D[:,0].min(),D[:,0].max(),D[:,1].min(),D[:,1].max()) if len(D) else "")
EOF

# openrua op 120
python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); P=d["xyz"][d["valid"]]; K=d["rgb"][d["valid"]]
m2=(np.hypot(P[:,0]+0.02,P[:,1]+0.24)<0.08)&(P[:,2]>0.92)
Q=P[m2];C=K[m2]
print("x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(Q[:,0].min(),Q[:,0].max(),Q[:,1].min(),Q[:,1].max(),Q[:,2].min(),Q[:,2].max()), "rgb mean",C.mean(axis=0).round())
from PIL import Image
Image.fromarray(d["rgb"]).save("agentview.png")
EOF

# openrua op 121
python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"][d["valid"]]
    for (y0,y1) in ((0.05,0.15),(-0.03,0.03),(0.15,0.22)):
        m=(P[:,2]>0.94)&(P[:,0]>-0.32)&(P[:,0]<-0.10)&(P[:,1]>y0)&(P[:,1]<y1)
        Q=P[m]
        if len(Q):
            hist=np.histogram(Q[:,0],bins=np.arange(-0.32,-0.10,0.01))[0]
            print(cam,f"y[{y0},{y1}] n={len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] z[{Q[:,2].min():.3f},{Q[:,2].max():.3f}]")
            print("   x-hist(1cm from -0.32):",hist)
EOF

# openrua op 122
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "IK attempt\|IK:"
import numpy as np, math
from rob import *
H=np.array([-0.18,0.235]); RHO=0.16; T=0.015; Z=1.01
def R_theta(th):
    z=np.array([math.sin(th),math.cos(th),0.0]); y=np.array([math.cos(th),-math.sin(th),0.0])
    return R_from_axes(y_hand=y,z_hand=z)
def push_pt(th,c):
    d=np.array([math.cos(th),-math.sin(th)]); n=np.array([-math.sin(th),-math.cos(th)])
    p=H+RHO*d+(T+c)*n; return np.array([p[0],p[1],Z])
r=Robot("iktest7"); seed=r.arm_q()
th=math.radians(86)
tests=[("appr1",np.array([-0.25,-0.10,Z]),th),("appr2",np.array([-0.25,0.074,Z]),th),("start",push_pt(th,0.008),th)]
for deg in (75,60,45,30,15,0,-3):
    tests.append((f"arc{deg}",push_pt(math.radians(deg),0.008),math.radians(deg)))
for name,p,t in tests:
    q=r.ik(p,R_theta(t),seed=seed,attempts=6,max_dev=0.8)
    print(name,np.round(p,4),None if q is None else np.round(q,2))
    if q is not None: seed=q
EOF

# openrua op 123
mkdir -p "$(dirname /workspace/phase2e_close_door.py)"
cat > /workspace/phase2e_close_door.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Phase 2e: close the microwave door by pushing its outer face with closed
fingertips along the hinge arc (reverse of phase 2c)."""
import math
import sys
import numpy as np
from rob import *

H = np.array([-0.18, 0.235])          # hinge (refined from the open-door slab measurement)
RHO = 0.16                            # push point distance from the hinge (closed x ~ -0.02, clear of the handle)
T = 0.015                             # half slab thickness (outer face offset from the door centreline)
CLEAR = 0.008                         # fingertips this far outside the modelled face
Z = 1.01
TH0 = math.radians(86.0)              # door currently open this much
TH_END = math.radians(-3.0)           # push a little past closed
STEP = math.radians(8.0)


def R_theta(th):
    z = np.array([math.sin(th), math.cos(th), 0.0])     # fingertips point at the door face
    y = np.array([math.cos(th), -math.sin(th), 0.0])    # hand width along the door
    return R_from_axes(y_hand=y, z_hand=z)


def push_pt(th, c=CLEAR):
    d = np.array([math.cos(th), -math.sin(th)])
    n = np.array([-math.sin(th), -math.cos(th)])
    p = H + RHO * d + (T + c) * n
    return np.array([p[0], p[1], Z])


r = Robot("phase2e")
q0 = r.arm_q()
r.gripper(0.0)
t0, R0 = r.tcp(); log("start tcp", np.round(t0, 4))

# 1. back away from the microwave front, then swing to the approach pose beside the door's free end
r.move_line(np.array([t0[0], -0.12, 1.15]), R0, step=0.03, speed=0.15)
Ra = R_theta(TH0)
q1, _ = r.move_to_pose(np.array([-0.25, -0.12, Z]), Ra, seed=r.arm_q(), speed=0.25, retries=6)
t, Rt = r.tcp(); log("approach tcp", np.round(t, 4), "z_hand", np.round(Rt[:, 2], 3))

# 2. slide along the outer side of the open door, then in to the face
r.move_line(np.array([-0.25, push_pt(TH0)[1], Z]), Ra, step=0.02, speed=0.1)
r.move_line(push_pt(TH0), Ra, step=0.015, speed=0.06)
t, _ = r.tcp(); log("at door face tcp", np.round(t, 4), "model", np.round(push_pt(TH0), 4))

# 3. push along the arc
th = TH0
while th > TH_END + 1e-6:
    th_new = max(th - STEP, TH_END)
    err = r.move_line(push_pt(th_new), R_theta(th), R_to=R_theta(th_new), step=0.02, speed=0.1, retries=4)
    t, _ = r.tcp()
    log(f"theta {math.degrees(th_new):.0f}: tcp {np.round(t, 4)} err {err:.4f}")
    if err > 0.05:
        log("!! door seems jammed; stopping the push"); break
    th = th_new

# 4. retreat straight back (-z_hand) and up
t, Rt = r.tcp()
back = t - 0.07 * Rt[:, 2]
r.move_line(back, Rt, step=0.02, speed=0.1)
r.move_line(np.array([back[0], back[1] - 0.05, 1.20]), Rt, step=0.03, speed=0.15)
r.gripper(0.04)
log("PHASE2E DONE")
OPENRUA_EOF

# openrua op 124
nohup python3 -u phase2e_close_door.py > phase2e.log 2>&1 &
sleep 120; grep -v "IK attempt\|IK:" /workspace/phase2e.log | tail -30

# openrua op 125
sleep 100; grep -v "IK attempt\|IK:" /workspace/phase2e.log | tail -20

# openrua op 126
sleep 40; grep -v "IK attempt\|IK:" /workspace/phase2e.log | tail -8; ps aux | grep -c "[p]ython3 -u phase2e"

# openrua op 127
python3 cloud.py agentview >/dev/null && python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
from PIL import Image
for cam in ("birdview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"][d["valid"]]
    Image.fromarray(d["rgb"]).save(f"{cam}.png")
    # anything door-like left in the open-door region?
    m=(P[:,2]>0.95)&(P[:,2]<1.11)&(P[:,0]>-0.25)&(P[:,0]<-0.13)&(P[:,1]>-0.05)&(P[:,1]<0.19)
    print(cam,"pts in open-door region:",m.sum())
    # front face of the microwave: y of points at door heights, x in door span
    m2=(P[:,2]>0.95)&(P[:,2]<1.10)&(P[:,0]>-0.18)&(P[:,0]<0.07)&(P[:,1]>0.10)&(P[:,1]<0.30)
    Q=P[m2]
    if len(Q):
        print("  front pts",len(Q),"y percentiles 1/50/99: %.3f %.3f %.3f"%tuple(np.percentile(Q[:,1],[1,50,99])))
        for x0 in (-0.15,-0.10,-0.05,0.0,0.05):
            s=Q[(Q[:,0]>x0)&(Q[:,0]<x0+0.05)]
            if len(s): print(f"   x[{x0:.2f},{x0+0.05:.2f}] ymin {s[:,1].min():.3f} n={len(s)}")
    # top edge of door: z>1.09 points with y<0.24
    m3=(P[:,2]>1.09)&(P[:,2]<1.12)&(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>0.15)&(P[:,1]<0.26)
    T=P[m3]
    if len(T): print("  top-edge pts",len(T),"y[%.3f,%.3f]"%(T[:,1].min(),T[:,1].max()))
EOF
