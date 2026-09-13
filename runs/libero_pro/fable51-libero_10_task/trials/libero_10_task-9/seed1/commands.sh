#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---ACTIONS---; ros2 action list; echo ---SERVICES---; ros2 service list | head -60; echo ---NODES---; ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_0.png & done; wait; ls snaps; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|rotation|x:|y:|z:|w:" | head -120; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head -40

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info+TF for cameras, save world-frame point clouds.

Usage: python3 scene.py <cam> [<cam> ...]
Writes snaps/<cam>_rgb.png, snaps/<cam>_xyz.npy (HxWx3 world coords, nan
where invalid) and prints the world->panda_link0 transform.
"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_matrix(tfbuf, node, parent, child):
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform(parent, child, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform(parent, child, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cams = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    bridge = CvBridge()
    T0 = tf_matrix(tfbuf, node, "world", "panda_link0")
    print("world->panda_link0 translation:", T0[:3, 3], "\nR:\n", T0[:3, :3])
    np.save("snaps/T_world_link0.npy", T0)
    for cam in cams:
        rgb = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
        depth = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        T = tf_matrix(tfbuf, node, "world", f"{cam}_optical_frame")
        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth
        valid = np.isfinite(z) & (z > 0)
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        pts = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
        xyz = pts[..., :3]
        xyz[~valid] = np.nan
        cv2.imwrite(f"snaps/{cam}_rgb.png", rgb)
        np.save(f"snaps/{cam}_xyz.npy", xyz)
        print(cam, "cam pos", T[:3, 3], "depth range", np.nanmin(z[valid]), np.nanmax(z[valid]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 300 python3 scene.py birdview agentview sideview frontview 2>&1 | grep -v "^\[" 

# openrua op 8
python3 -c "
import numpy as np, cv2
xyz=np.load('snaps/birdview_xyz.npy'); rgb=cv2.imread('snaps/birdview_rgb.png')
z=xyz[...,2]
# histogram of heights
hist,edges=np.histogram(z[np.isfinite(z)],bins=60)
for h,e in zip(hist,edges): 
    if h>200: print(f'{e:.3f} {h}')
"


# openrua op 9
python3 -c "
import numpy as np, cv2
xyz=np.load('snaps/birdview_xyz.npy'); rgb=cv2.imread('snaps/birdview_rgb.png')
z=xyz[...,2]
tab = z[(z>0.87)&(z<0.90)]
print('table z median', np.median(tab))
# objects above table: 0.91..1.3, on table region
m=(z>0.905)&(z<1.35)
ys,xs=np.nonzero(m)
print('n pts',len(xs))
# cluster by connected components on mask
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<30: continue
    mm=lab==i
    p=xyz[mm]
    print(i,'px',stats[i,:4],'cent px',cent[i].round(0),'world min',np.nanmin(p,0).round(3),'max',np.nanmax(p,0).round(3),'mean',np.nanmean(p,0).round(3))
"


# openrua op 10
python3 -c "
import numpy as np, cv2
xyz=np.load('snaps/birdview_xyz.npy'); rgb=cv2.imread('snaps/birdview_rgb.png')
z=xyz[...,2]
# crop microwave region in image
sub=xyz[220:360,330:470]
zs=sub[...,2]
# print z at grid
np.set_printoptions(linewidth=250, precision=2, suppress=True)
print('z grid rows 225..355 step 10, cols 335..465 step 10')
print(zs[5::10, 5::10])
print('x grid'); print(sub[5::10,5::10,0])
print('y grid'); print(sub[5::10,5::10,1])
"


# openrua op 11
python3 -c "
import numpy as np
pts=[]
for c in ['birdview','agentview','sideview','frontview']:
    xyz=np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)
    xyz=xyz[np.isfinite(xyz).all(1)]
    pts.append(xyz)
P=np.concatenate(pts)
# microwave region, above table
m=(P[:,0]>-0.35)&(P[:,0]<0.25)&(P[:,1]>-0.05)&(P[:,1]<0.55)&(P[:,2]>0.92)&(P[:,2]<1.2)
Q=P[m]
res=0.01
xs=np.arange(-0.35,0.25,res); ys=np.arange(-0.05,0.55,res)
grid=np.zeros((len(xs),len(ys)),int)
ix=((Q[:,0]+0.35)/res).astype(int); iy=((Q[:,1]+0.05)/res).astype(int)
np.add.at(grid,(ix.clip(0,len(xs)-1),iy.clip(0,len(ys)-1)),1)
print('rows=x from -0.35 (top) to 0.25, cols=y from -0.05 (left) to 0.55, 1cm cells; # = occupied')
print('     '+''.join(f'{y:+.1f}'[:4] if i%10==0 else ' ' for i,y in enumerate(ys)))
for i,x in enumerate(xs):
    print(f'{x:+.2f} '+''.join('#' if g>3 else ('.' if g>0 else ' ') for g in grid[i]))
"


# openrua op 12
python3 -c "
import numpy as np
pts={}
for c in ['birdview','agentview','sideview','frontview']:
    xyz=np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)
    pts[c]=xyz[np.isfinite(xyz).all(1)]
P=np.concatenate(list(pts.values()))
# white mug region
m=(P[:,0]>-0.22)&(P[:,0]<-0.02)&(P[:,1]>-0.35)&(P[:,1]<-0.15)&(P[:,2]>0.905)&(P[:,2]<1.2)
Q=P[m]
print('mug z percentiles', np.percentile(Q[:,2],[50,90,95,98,99,99.5,100]).round(3))
for zlo in np.arange(0.91,1.05,0.02):
    s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.02)]
    if len(s)<5: print(f'z {zlo:.2f}: n={len(s)}'); continue
    print(f'z {zlo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}')
# Which face has the cavity? look at the -y face x-z slice, y in [0.20,0.32]
print('--- microwave front face y in[0.2,0.33]: for each x, min/max z occupied and fraction')
m2=(P[:,0]>-0.22)&(P[:,0]<0.2)&(P[:,1]>0.20)&(P[:,1]<0.33)&(P[:,2]>0.905)&(P[:,2]<1.15)
R=P[m2]
for xlo in np.arange(-0.22,0.2,0.02):
    s=R[(R[:,0]>=xlo)&(R[:,0]<xlo+0.02)]
    if len(s)<3: print(f'x {xlo:+.2f}: n={len(s)}'); continue
    print(f'x {xlo:+.2f}: n={len(s)} z[{s[:,2].min():.3f},{s[:,2].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"


# openrua op 13
python3 -c "
import numpy as np
for c in ['agentview','frontview','sideview','birdview']:
    xyz=np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)
    P=xyz[np.isfinite(xyz).all(1)]
    m=(P[:,0]>-0.2)&(P[:,0]<-0.02)&(P[:,1]>-0.33)&(P[:,1]<-0.15)&(P[:,2]>0.85)
    Q=P[m]
    h,e=np.histogram(Q[:,2],bins=np.arange(0.85,1.12,0.01))
    print(c, ' '.join(f'{ee:.2f}:{hh}' for hh,ee in zip(h,e) if hh>0))
"


# openrua op 14
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0 2>&1 | tail -3; ros2 topic echo /joint_states --once | grep -A9 position; ros2 topic echo /clock --once

# openrua op 15
timeout 300 python3 scene.py birdview agentview frontview sideview 2>&1 | grep -v "^\[" ; python3 -c "
import numpy as np
P=np.concatenate([np.load(f'snaps/{c}_xyz.npy').reshape(-1,3) for c in ['birdview','agentview','sideview','frontview']])
P=P[np.isfinite(P).all(1)]
def prof(name,m):
    Q=P[m]
    print(name,'z range',Q[:,2].min().round(3),np.percentile(Q[:,2],99.5).round(3))
    for zlo in np.arange(0.90,1.10,0.02):
        s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.02)]
        if len(s)<5: continue
        print(f'  z {zlo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}')
prof('white mug',(P[:,0]>-0.22)&(P[:,0]<-0.02)&(P[:,1]>-0.35)&(P[:,1]<-0.15)&(P[:,2]>0.905)&(P[:,2]<1.2))
prof('yellow mug',(P[:,0]>-0.08)&(P[:,0]<0.1)&(P[:,1]>-0.12)&(P[:,1]<0.08)&(P[:,2]>0.905)&(P[:,2]<1.2))
"


# openrua op 16
python3 -c "
import numpy as np
P=np.concatenate([np.load(f'snaps/{c}_xyz.npy').reshape(-1,3) for c in ['birdview','agentview','sideview','frontview']])
P=P[np.isfinite(P).all(1)]
# cavity: x in [-0.16,0.06], y in [0.27,0.5], z in [0.9,1.12]
m=(P[:,0]>-0.2)&(P[:,0]<0.2)&(P[:,1]>0.27)&(P[:,1]<0.50)&(P[:,2]>0.90)&(P[:,2]<1.13)
Q=P[m]
print('z hist for x in [-0.16,0.06] (door side)')
Qd=Q[(Q[:,0]>-0.16)&(Q[:,0]<0.06)]
h,e=np.histogram(Qd[:,2],bins=np.arange(0.90,1.13,0.01))
print(' '.join(f'{ee:.2f}:{hh}' for hh,ee in zip(h,e) if hh>0))
# y-depth histogram for interior points at z in [0.95,1.05] (mid-height, so not floor/ceiling)
Qi=Qd[(Qd[:,2]>0.95)&(Qd[:,2]<1.06)]
h,e=np.histogram(Qi[:,1],bins=np.arange(0.27,0.50,0.01))
print('y hist mid-height:', ' '.join(f'{ee:.2f}:{hh}' for hh,ee in zip(h,e) if hh>0))
h,e=np.histogram(Qi[:,0],bins=np.arange(-0.2,0.2,0.01))
print('x hist mid-height:', ' '.join(f'{ee:+.2f}:{hh}' for hh,ee in zip(h,e) if hh>0))
# floor points z<0.95 : y extent
Qf=Qd[Qd[:,2]<0.95]
print('floor pts z<0.95: y range',Qf[:,1].min().round(3),Qf[:,1].max().round(3),'x range',Qf[:,0].min().round(3),Qf[:,0].max().round(3))
print('floor z percentiles',np.percentile(Qf[:,2],[5,50,95]).round(3))
"


# openrua op 17
python3 -c "
import cv2
im=cv2.imread('snaps/agentview_rgb.png'); cv2.imwrite('snaps/crop_agent_mw.png', cv2.resize(im[120:330,320:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/frontview_rgb.png'); cv2.imwrite('snaps/crop_front_mw.png', cv2.resize(im[230:450,320:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 18
python3 -c "
import numpy as np, cv2
xyz=np.load('snaps/agentview_xyz.npy')
# region of crop: rows 120..330, cols 320..640 -> look at points with x in [-0.2,0.2], y in [0.2,0.5], z in [0.9,1.12]
sub=xyz[120:330,320:640]
m=np.isfinite(sub).all(-1)&(sub[...,0]>-0.2)&(sub[...,0]<0.2)&(sub[...,1]>0.2)&(sub[...,1]<0.5)&(sub[...,2]>0.905)&(sub[...,2]<1.10)
Q=sub[m]
h,e=np.histogram(Q[:,1],bins=np.arange(0.2,0.5,0.01))
print('agentview y hist (body region, z 0.905-1.10):',' '.join(f'{ee:.2f}:{hh}' for hh,ee in zip(h,e) if hh>0))
# for points with y>0.31 (interior?), print their x,z ranges
Qi=Q[Q[:,1]>0.31]
if len(Qi): print('interior-ish pts', len(Qi), 'x',Qi[:,0].min().round(3),Qi[:,0].max().round(3),'y',Qi[:,1].min().round(3),Qi[:,1].max().round(3),'z',Qi[:,2].min().round(3),Qi[:,2].max().round(3))
# make a visualization: mark pixels with y>0.31 in red on the crop
im=cv2.imread('snaps/agentview_rgb.png')[120:330,320:640].copy()
im[m&(sub[...,1]>0.31)]=(0,0,255)
im[m&(sub[...,1]<=0.31)&(sub[...,1]>0.25)]=(0,255,0)
cv2.imwrite('snaps/crop_agent_mark.png',cv2.resize(im,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 19
python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy')
sub=xyz[120:330,320:640]
m=np.isfinite(sub).all(-1)&(sub[...,0]>-0.2)&(sub[...,0]<0.07)&(sub[...,1]>0.25)&(sub[...,1]<0.5)&(sub[...,2]>0.93)&(sub[...,2]<1.09)
Q=sub[m]
# 2D histogram x vs y
xs=np.arange(-0.2,0.08,0.02); ys=np.arange(0.25,0.41,0.01)
H,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xs,ys])
print('rows x, cols y from 0.25 to 0.40 step 0.01')
print('      '+' '.join(f'{y:.2f}'[2:] for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f'{x:+.2f} '+' '.join(f'{int(v):3d}' if v else '  .' for v in H[i]))
"

# openrua op 20
python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy')
sub=xyz[120:330,320:640]
m=np.isfinite(sub).all(-1)&(sub[...,0]>-0.15)&(sub[...,0]<0.03)&(sub[...,1]>0.25)&(sub[...,1]<0.5)&(sub[...,2]>0.90)&(sub[...,2]<1.12)
Q=sub[m]
ys=np.arange(0.25,0.41,0.01); zs=np.arange(0.90,1.13,0.01)
H,_,_=np.histogram2d(Q[:,1],Q[:,2],bins=[ys,zs])
print('rows y, cols z from 0.90 to 1.12')
print('     '+' '.join(f'{z:.2f}'[1:] for z in zs[:-1]))
for i,y in enumerate(ys[:-1]):
    print(f'{y:.2f} '+' '.join(f'{int(v):3d}' if v else '  .' for v in H[i]))
"

# openrua op 21
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
" 2>&1 | tail -1; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'link name="panda_leftfinger"' robot.urdf | head -40; grep -n 'mesh filename' robot.urdf | head -5

# openrua op 22
python3 -c "
import numpy as np, struct
def stl_bounds(path):
    d=open(path,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    v=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))['v'].reshape(-1,3)
    return v.min(0).round(4), v.max(0).round(4)
for l in ['hand','finger','link7','link6']:
    p=f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{l}.stl'
    try: print(l, stl_bounds(p))
    except Exception as e: print(l,e)
"; grep -o '<joint name="panda_finger_joint1"[^>]*>.\{0,400\}' robot.urdf | head -3; grep -o 'link name="panda_hand".\{0,600\}' robot.urdf | grep -o 'origin[^/]*' 

# openrua op 23
find / -name "hand*.stl" -o -name "finger*.stl" 2>/dev/null | head; find / -path "*panda*" -name "*.stl" 2>/dev/null | head -3; find / -path "*panda*" -name "*.dae" 2>/dev/null | head -3

# openrua op 24
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK/IK (MoveIt), trajectories, gripper.

World frame = panda_link0 + (-0.66, 0, 0.912) (from TF). MoveIt poses are in
panda_link0 (frame_id left empty as machine.yaml says).
"""
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
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM_JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])
TCP_OFF = 0.1034


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t = time.time()
        while "m" not in self._js and time.time() - t < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM_JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """World pose (pos, quat xyzw) of link for arm config q."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM_JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP_OFF * R[:, 2], quat

    # ---------- planning ----------
    def ik(self, pos_world, quat_xyzw, seed=None, link="panda_hand", at_tcp=True,
           timeout=5.0, attempts=1):
        """IK for the hand (or TCP if at_tcp) at a world pose. Returns q or None."""
        pos = np.asarray(pos_world, float)
        R = Rot.from_quat(quat_xyzw).as_matrix()
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        pos = pos - BASE_IN_WORLD
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = link
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
        req.ik_request.robot_state.joint_state.name = list(ARM_JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        for _ in range(attempts):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM_JOINTS]
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, wait=True):
        return self.move_traj([q], [seconds], wait)

    def move_traj(self, qs, times, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        for q, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        code = rf.result().result.error_code
        q_now = self.arm_q()
        err = float(np.max(np.abs(np.array(q_now) - np.array(qs[-1]))))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


def quat_from_axes(x_h, y_h, z_h):
    """Quaternion (xyzw) of a frame whose axes (in world) are given as columns."""
    R = np.column_stack([x_h, y_h, z_h])
    return Rot.from_matrix(R).as_quat()


def topdown_quat(finger_axis_world):
    """Hand pointing straight down, fingers closing along finger_axis_world."""
    y_h = np.asarray(finger_axis_world, float)
    y_h = y_h / np.linalg.norm(y_h)
    z_h = np.array([0, 0, -1.0])
    x_h = np.cross(y_h, z_h)
    return quat_from_axes(x_h, y_h, z_h)


def approach_quat(approach_world, finger_axis_world):
    """Hand z along approach, fingers along finger_axis (must be perpendicular)."""
    z_h = np.asarray(approach_world, float); z_h /= np.linalg.norm(z_h)
    y_h = np.asarray(finger_axis_world, float)
    y_h = y_h - np.dot(y_h, z_h) * z_h; y_h /= np.linalg.norm(y_h)
    x_h = np.cross(y_h, z_h)
    return quat_from_axes(x_h, y_h, z_h)
OPENRUA_EOF

# openrua op 25
timeout 120 python3 -c "
from rob import *
r=Robot('t1')
q=r.arm_q(); print('q',np.round(q,3))
for l in ['panda_hand','panda_link8']:
    p,qu=r.fk(q,l); print(l,'pos',p.round(4),'quat',qu.round(4))
p,qu=r.tcp(q); print('TCP world',p.round(4))
print('fingers',r.fingers())
" 2>&1 | grep -v '^\['

# openrua op 26
timeout 200 python3 -c "
from rob import *
import rob
r=Robot('t2')
q=r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
req.robot_state.joint_state.name=list(ARM_JOINTS); req.robot_state.joint_state.position=q
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
res=fut.result(); print('FK header frame:', repr(res.pose_stamped[0].header.frame_id), 'raw pos', res.pose_stamped[0].pose.position)
raw=np.array([res.pose_stamped[0].pose.position.x,res.pose_stamped[0].pose.position.y,res.pose_stamped[0].pose.position.z])
quat=np.array([res.pose_stamped[0].pose.orientation.x,res.pose_stamped[0].pose.orientation.y,res.pose_stamped[0].pose.orientation.z,res.pose_stamped[0].pose.orientation.w])
# IK test A: raw pos treated as panda_link0-frame (no offset)
rob.BASE_IN_WORLD=np.zeros(3)
solA=r.ik(raw,quat,seed=q,at_tcp=False)
print('IK A (raw as base frame):', None if solA is None else np.round(solA,3))
# IK test B: raw shifted by base offset
rob.BASE_IN_WORLD=np.array([-0.66,0,0.912])
solB=r.ik(raw,quat,seed=q,at_tcp=False)
print('IK B (raw minus base offset):', None if solB is None else np.round(solB,3))
print('current q', np.round(q,3))
" 2>&1 | grep -v '^\['

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])", "new_string": "# Verified empirically: /compute_fk and /compute_ik here operate directly in\\n# the world frame (FK of the hand matches the camera-observed hand position),\\n# so no base offset is applied.\\nBASE_IN_WORLD = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
timeout 300 python3 -c "
from rob import *
r=Robot('t3')
q0=r.arm_q()
cx,cy,rim=-0.121,-0.2455,1.014
tests={
 'pregrasp': ([cx+0.048, cy, 1.12], topdown_quat([1,0,0])),
 'grasp':    ([cx+0.048, cy, rim-0.02], topdown_quat([1,0,0])),
 'lift':     ([cx+0.048, cy, 1.22], topdown_quat([1,0,0])),
 'lift_yaw': ([cx+0.048, cy, 1.22], topdown_quat([0,1,0])),
 'lift_tilt':([cx+0.048, cy, 1.22], approach_quat([0,1,0],[0,0,1])),
 'preins':   ([-0.03, 0.13, 1.056], approach_quat([0,1,0],[0,0,1])),
 'preins_hi':([-0.03, 0.13, 1.20], approach_quat([0,1,0],[0,0,1])),
 'insert':   ([-0.03, 0.30, 1.056], approach_quat([0,1,0],[0,0,1])),
 'insert_fneg':([-0.03, 0.30, 1.056], approach_quat([0,1,0],[0,0,-1])),
}
sols={}
seed=q0
for k,(p,qu) in tests.items():
    s=r.ik(p,qu,seed=seed,attempts=3)
    print(k, 'OK' if s else 'FAIL', None if s is None else np.round(s,2))
    if s: 
        tp,_=r.tcp(s); print('   fk tcp',tp.round(3)); sols[k]=s; seed=s
" 2>&1 | grep -v '^\['

# openrua op 29
timeout 300 python3 -c "
from rob import *
r=Robot('t4')
q0=r.arm_q()
cx,cy,rim=-0.121,-0.2455,1.014
R0=Rot.from_quat(topdown_quat([1,0,0])).as_matrix()
# mug features in hand frame (grasp at +x wall): axis(bottom->rim)=+z_w, pinch->center=-x_w, handle=+y_w
feat={'axis':[0,0,1],'pinch2center':[-1,0,0],'handle':[0,1,0]}
feat_h={k:R0.T@np.array(v,float) for k,v in feat.items()}
b=np.deg2rad(60)
for a_sign in [+1,-1]:
    y_h=np.array([a_sign*np.sin(b),0,np.cos(b)])
    qi=approach_quat([0,1,0],y_h); Ri=Rot.from_quat(qi).as_matrix()
    print('a_sign',a_sign,{k:(Ri@v).round(2) for k,v in feat_h.items()})
    s=r.ik([-0.031,0.30,1.022],qi,seed=q0,attempts=3)
    print('   insert IK', None if s is None else np.round(s,2))
    s2=r.ik([-0.031,0.13,1.022],qi,seed=s or q0,attempts=3)
    print('   preins IK', None if s2 is None else np.round(s2,2))
    s3=r.ik([cx+0.048,cy,1.22],qi,seed=q0,attempts=3)
    print('   lift-reoriented IK', None if s3 is None else np.round(s3,2))
" 2>&1 | grep -v '^\['

# openrua op 30
timeout 300 python3 -c "
from rob import *
r=Robot('t5')
q0=r.arm_q()
cx,cy,rim=-0.121,-0.2455,1.014
R0=Rot.from_quat(topdown_quat([1,0,0])).as_matrix()
# V2: grasp at -x wall: pinch->center=+x_w
feat={'axis':[0,0,1],'pinch2center':[1,0,0],'handle':[0,1,0]}
feat_h={k:R0.T@np.array(v,float) for k,v in feat.items()}
b=np.deg2rad(60)
y_h=np.array([np.sin(b),0,-np.cos(b)])
qi=approach_quat([0,1,0],y_h); Ri=Rot.from_quat(qi).as_matrix()
print({k:(Ri@v).round(2) for k,v in feat_h.items()})
print('quat',qi.round(4))
# pinch x so mug extent centered: center = pinch + 0.048*(0.87,0,-0.5); body +-0.048, handle to center+0.081*0.5 in x; outer finger to pinch-0.06*0.87
# extent: [pinch-0.052, pinch+0.042+0.048=pinch+0.09] -> center of extent = pinch+0.019 -> want -0.05 => pinch=-0.069
px=-0.069
seed=q0
for name,p in [('lift-reor',[cx-0.048,cy,1.22]),('preins_hi',[px,0.13,1.20]),('preins',[px,0.13,1.022]),('insert',[px,0.30,1.022]),('insert_low',[px,0.30,1.008])]:
    s=r.ik(p,qi,seed=seed,attempts=3)
    print(name, None if s is None else np.round(s,2))
    if s: seed=s; tp,_=r.tcp(s); print('   tcp',tp.round(3))
" 2>&1 | grep -v '^\['

# openrua op 31
timeout 600 python3 -c "
from rob import *
r=Robot('pick1')
q0=r.arm_q()
cx,cy,rim=-0.121,-0.2455,1.014
qg=topdown_quat([1,0,0])
pre=r.ik([cx-0.048,cy,1.12],qg,seed=q0,attempts=3); print('pre',np.round(pre,3))
print('moving to pregrasp'); r.move_q(pre,4.0)
print('tcp',r.tcp()[0].round(4))
gr=r.ik([cx-0.048,cy,rim-0.02],qg,seed=r.arm_q(),attempts=3); print('grasp',np.round(gr,3))
print('descending'); r.move_q(gr,2.5)
print('tcp',r.tcp()[0].round(4),'fingers',r.fingers())
" 2>&1 | grep -v '^\['; for c in robot0_eye_in_hand agentview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_pre.png; done

# openrua op 32
timeout 120 python3 -c "
from rob import *
r=Robot('chk')
q=r.arm_q(); print('q now',np.round(q,3))
tgt=[-0.234,0.427,-0.217,-2.218,0.182,2.629,-1.376]
print('diff',np.round(np.array(q)-tgt,3))
print('tcp',r.tcp(q)[0].round(4))
" 2>&1 | grep -v '^\['

# openrua op 33
timeout 600 python3 -c "
from rob import *
r=Robot('pick1b')
tgt=[-0.234,0.427,-0.217,-2.218,0.182,2.629,-1.376]
r.move_q(tgt,4.0)
import time
for i in range(3):
    q=r.arm_q(); print('q',np.round(q,3),'diff',np.round(np.array(q)-tgt,3)); print('tcp',r.tcp(q)[0].round(4))
" 2>&1 | grep -v '^\['

# openrua op 34
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def gripper(self, width):", "new_string": "    def move_q_corrected(self, q_target, seconds=3.0, iters=3, tol=0.006, tol7=0.03,\\n                         j7_rate=0.15):\\n        \\"\\"\\"Move to q_target, then re-send with residual compensation.\\n\\n        Machine facts observed: joint 7 tracks at only ~0.2 rad/s, and joints\\n        under gravity load settle ~0.01-0.02 rad short of the command. So the\\n        command is offset by the measured residual and joint 7 gets extra time.\\n        \\"\\"\\"\\n        q_target = np.asarray(q_target, float)\\n        offset = np.zeros(7)\\n        q_now = np.asarray(self.arm_q())\\n        for i in range(iters + 1):\\n            d7 = abs(q_target[6] + offset[6] - q_now[6])\\n            dur = max(seconds if i == 0 else 2.0, d7 / j7_rate + 0.5)\\n            self.move_q(list(q_target + offset), dur)\\n            q_now = np.asarray(self.arm_q())\\n            resid = q_target - q_now\\n            print(f\\"  iter{i} resid={np.round(resid, 4)}\\")\\n            if np.max(np.abs(resid[:6])) < tol and abs(resid[6]) < tol7:\\n                break\\n            offset[:6] += resid[:6]\\n            offset[6] = 0.0  # joint 7 has no sag, only lag\\n        return q_now\\n\\n    def gripper(self, width):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 35
timeout 900 python3 -c "
from rob import *
r=Robot('pick1c')
tgt=[-0.234,0.427,-0.217,-2.218,0.182,2.629,-1.376]
q=r.move_q_corrected(tgt,3.0)
print('tcp',r.tcp(list(q))[0].round(4),'fingers',r.fingers())
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_grasp.png

# openrua op 36
timeout 600 python3 -c "
from rob import *
r=Robot('grasp')
r.gripper(0.0)
print('fingers',r.fingers(),'tcp',r.tcp()[0].round(4))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_closed.png

# openrua op 37
timeout 900 python3 -c "
from rob import *
r=Robot('lift')
q0=r.arm_q()
qg=topdown_quat([1,0,0])
lift=r.ik([-0.169,-0.2455,1.22],qg,seed=q0,attempts=3); print('lift',np.round(lift,3))
q=r.move_q_corrected(lift,3.0,iters=1)
print('tcp',r.tcp(list(q))[0].round(4),'fingers',r.fingers())
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_lift.png

# openrua op 38
timeout 1200 python3 -c "
from rob import *
from scipy.spatial.transform import Slerp
r=Robot('reor')
q0=r.arm_q()
p=[-0.169,-0.2455,1.22]
qa=topdown_quat([1,0,0]); qb=np.array([0.6124,0.3536,0.3536,-0.6124])
sl=Slerp([0,1],Rot.from_quat([qa,qb]))
fr=[0.25,0.5,0.75,1.0]
qs=[]; seed=q0
for f in fr:
    qu=sl([f]).as_quat()[0]
    s=r.ik(p,qu,seed=seed,attempts=3)
    if s is None: print('IK fail at',f); raise SystemExit
    print(f, np.round(s,2), 'delta', np.round(np.array(s)-np.array(seed),2))
    qs.append(s); seed=s
# timing: joint7 rate ~0.15 rad/s min 2.5s per segment
ts=[]; t=0; prev=q0
for s in qs:
    d=np.abs(np.array(s)-np.array(prev)); t+=max(2.5, d[6]/0.15+0.5, d[:6].max()/0.5+0.5); ts.append(t); prev=s
print('times',np.round(ts,1))
r.move_traj(qs,ts)
q=r.move_q_corrected(qs[-1],2.0,iters=2)
pos,qu=r.tcp(list(q)); print('tcp',pos.round(4),'quat',qu.round(3),'fingers',r.fingers())
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_reor.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_reor.png

# openrua op 39
timeout 1200 python3 -c "
from rob import *
r=Robot('preins')
q0=r.arm_q()
qi=np.array([0.6124,0.3536,0.3536,-0.6124])
p0=np.array([-0.169,-0.2455,1.22]); p1=np.array([-0.069,0.13,1.20])
qs=[]; seed=q0
for f in [0.33,0.66,1.0]:
    p=p0+(p1-p0)*f
    s=r.ik(p,qi,seed=seed,attempts=3)
    if s is None: print('IK fail',f); raise SystemExit
    print(f,np.round(s,2),'delta',np.round(np.array(s)-np.array(seed),2)); qs.append(s); seed=s
ts=[]; t=0; prev=q0
for s in qs:
    d=np.abs(np.array(s)-np.array(prev)); t+=max(2.0, d[6]/0.15+0.5, d[:6].max()/0.5+0.5); ts.append(t); prev=s
print('times',np.round(ts,1))
r.move_traj(qs,ts)
q=r.move_q_corrected(qs[-1],2.0,iters=2)
pos,qu=r.tcp(list(q)); print('tcp',pos.round(4),'quat',qu.round(3),'fingers',r.fingers())
" 2>&1 | grep -v '^\['; timeout 300 python3 scene.py frontview agentview sideview 2>&1 | grep -v '^\['

# openrua op 40
python3 -c "
import numpy as np, cv2
P=np.concatenate([np.load(f'snaps/{c}_xyz.npy').reshape(-1,3) for c in ['frontview','agentview','sideview']])
P=P[np.isfinite(P).all(1)]
# mug region: expected around tcp (-0.069,0.13,1.20): center at tcp+0.048*(0.87,0,-0.5)=(-0.027,~0.185,1.176), spans y 0.11..0.22
m=(P[:,0]>-0.2)&(P[:,0]<0.12)&(P[:,1]>0.05)&(P[:,1]<0.30)&(P[:,2]>1.05)&(P[:,2]<1.30)
Q=P[m]
print('n',len(Q))
# y-slices along mug axis
for ylo in np.arange(0.05,0.30,0.02):
    s=Q[(Q[:,1]>=ylo)&(Q[:,1]<ylo+0.02)]
    if len(s)<10: continue
    print(f'y {ylo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
# x-z profile for the body region y in [0.15,0.20]
s=Q[(Q[:,1]>0.15)&(Q[:,1]<0.21)]
H,xe,ze=np.histogram2d(s[:,0],s[:,2],bins=[np.arange(-0.2,0.12,0.01),np.arange(1.05,1.30,0.01)])
print('rows x, cols z 1.05..1.29')
for i,x in enumerate(xe[:-1]):
    print(f'{x:+.2f} '+''.join('#' if v>3 else ('.' if v>0 else ' ') for v in H[i]))
"


# openrua op 41
python3 -c "
import numpy as np
# current features in world x-z plane relative to TCP (pinch)
c=np.array([0.04,-0.01])         # mug body center
hdir=np.array([0.0,1.0])          # handle direction from center (unit)
yh=np.array([0.87,-0.5])          # finger axis (toward center side)
R=0.048; HL=0.09                  # body radius, handle reach beyond center (incl thickness)
FLOOR,CEIL=0.94,1.09; XMIN,XMAX=-0.16,0.06
def rot(v,a):
    ca,sa=np.cos(a),np.sin(a); return np.array([ca*v[0]-sa*v[1], sa*v[0]+ca*v[1]])
best=None
for deg in range(-180,181,5):
    a=np.deg2rad(deg)
    cc=rot(c,a); hh=rot(hdir,a); yy=rot(yh,a)
    # candidate extreme points (relative to TCP): body circle, handle tip, outer finger open (0.06 along -yy) with +-0.012 thickness, inner finger open (0.06 along +yy)
    pts=[]
    for th in np.linspace(0,2*np.pi,72): pts.append(cc+R*np.array([np.cos(th),np.sin(th)]))
    pts.append(cc+HL*hh); pts.append(cc+(HL-0.02)*hh+0.015*np.array([-hh[1],hh[0]])); pts.append(cc+(HL-0.02)*hh-0.015*np.array([-hh[1],hh[0]]))
    body_pts=np.array(pts)   # mug only (must be below ceiling, above floor)
    perp=np.array([-yy[1],yy[0]])
    fing=[ -0.062*yy+0.012*perp, -0.062*yy-0.012*perp, 0.062*yy+0.012*perp, 0.062*yy-0.012*perp, np.zeros(2)]
    allp=np.vstack([body_pts,np.array(fing)])
    zmin=body_pts[:,1].min(); zmax=allp[:,1].max(); xmin=allp[:,0].min(); xmax=allp[:,0].max()
    # place TCP z so body bottom at FLOOR+0.012
    tz=FLOOR+0.012-zmin
    top=tz+zmax
    zmargin=CEIL-top
    xspan=xmax-xmin; xmargin=(XMAX-XMIN-xspan)/2
    tx=XMIN+xmargin-xmin
    m=min(zmargin,xmargin)
    if best is None or m>best[0]: best=(m,deg,tz,tx,zmargin,xmargin)
    if deg%15==0: print(f'roll {deg:4d}: tcp z={tz:.3f} top={top:.3f} zmargin={zmargin:+.3f} xspan={xspan:.3f} xmargin={xmargin:+.3f} tcp x={tx:+.3f}')
print('best',best)
"


# openrua op 42
python3 -c "
import numpy as np
c=np.array([0.04,-0.01]); hdir=np.array([0.0,1.0]); yh=np.array([0.87,-0.5])
R=0.048; HL=0.09; FLOOR,CEIL=0.94,1.09; XMIN,XMAX=-0.16,0.06
def rot(v,a):
    ca,sa=np.cos(a),np.sin(a); return np.array([ca*v[0]-sa*v[1], sa*v[0]+ca*v[1]])
best=None
for deg in range(-180,181,5):
    a=np.deg2rad(deg); cc=rot(c,a); hh=rot(hdir,a); yy=rot(yh,a); perp=np.array([-yy[1],yy[0]])
    pts=[cc+R*np.array([np.cos(t),np.sin(t)]) for t in np.linspace(0,2*np.pi,72)]
    pts+= [cc+HL*hh, cc+(HL-0.02)*hh+0.015*perp, cc+(HL-0.02)*hh-0.015*perp]
    fing=[-0.062*yy+0.012*perp,-0.062*yy-0.012*perp,0.062*yy+0.012*perp,0.062*yy-0.012*perp,np.zeros(2)]
    # also hand palm: behind TCP along -z_h (out of the cavity) -> ignore, but finger bodies extend 0.05 back from tips; still inside opening: include tips only
    allp=np.vstack([np.array(pts),np.array(fing)])
    zmin=allp[:,1].min(); zmax=allp[:,1].max(); xmin=allp[:,0].min(); xmax=allp[:,0].max()
    zspan=zmax-zmin; zm=(CEIL-FLOOR-zspan)/2; tz=FLOOR+zm-zmin
    xspan=xmax-xmin; xm=(XMAX-XMIN-xspan)/2; tx=XMIN+xm-xmin
    m=min(zm,xm)
    if best is None or m>best[0]: best=(m,deg,tz,tx,zm,xm)
    if deg%15==0: print(f'roll {deg:4d}: zspan={zspan:.3f} zmargin={zm:+.3f} tcp z={tz:.3f} | xspan={xspan:.3f} xmargin={xm:+.3f} tcp x={tx:+.3f}  body-bottom={tz+min(p[1] for p in pts):.3f}')
print('best',best)
"


# openrua op 43
python3 -c "
import numpy as np
from scipy.optimize import least_squares
P=[]
for cam in ['frontview','agentview','sideview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy').reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]; P.append(xyz)
P=np.vstack(P)
# TCP now ~(-0.0692,0.1296,1.1996). Mug body slice: y in [0.15,0.21], near TCP
sel=(P[:,1]>0.15)&(P[:,1]<0.21)&(P[:,0]>-0.15)&(P[:,0]<0.08)&(P[:,2]>1.10)&(P[:,2]<1.35)
Q=P[sel]; print('pts',len(Q))
# circle fit with fixed radius via robust least squares on x-z
xz=Q[:,[0,2]]
def res(c,r=0.048): return np.linalg.norm(xz-c,axis=1)-r
# Body points only: exclude handle by iterative reweighting (soft_l1)
c0=np.array([-0.03,1.19])
sol=least_squares(res,c0,loss='soft_l1',f_scale=0.005); c=sol.x
d=res(c); body=np.abs(d)<0.008
print('center fixed-r',c.round(4),'inliers',body.sum(),'/',len(Q))
# free radius fit on inliers
def res2(p): return np.linalg.norm(xz[body]-p[:2],axis=1)-p[2]
s2=least_squares(res2,[c[0],c[1],0.048]); print('free fit center,r',s2.x.round(4))
# handle: outliers beyond r+0.01
out=d>0.01; H=Q[out]; print('handle pts',out.sum())
if out.sum():
    v=H[:,[0,2]]-c; ang=np.degrees(np.arctan2(v[:,1],v[:,0])); dist=np.linalg.norm(v,axis=1)
    print('handle angle from +x (deg): median',np.median(ang).round(1),'p10/p90',np.percentile(ang,[10,90]).round(1))
    print('handle radial extent max',dist.max().round(4),'p95',np.percentile(dist,95).round(4))
    print('handle z max',H[:,2].max().round(4),'x range',H[:,0].min().round(3),H[:,0].max().round(3))
# y extents of mug (rim / bottom) for points within body ring in x-z
ring=(np.abs(np.linalg.norm(P[:,[0,2]]-c,axis=1)-0.048)<0.01)&(P[:,2]>1.10)
Y=P[ring,1]; print('mug y range',np.percentile(Y,[1,99]).round(3))
"


# openrua op 44
python3 -c "
import numpy as np
c=np.array([0.036,-0.017]); a0=np.deg2rad(116); hdir=np.array([np.cos(a0),np.sin(a0)]); yh=np.array([0.866,-0.5])
R=0.047; HL=0.088; FLOOR,CEIL=0.94,1.09; XMIN,XMAX=-0.16,0.06
def rot(v,a):
    ca,sa=np.cos(a),np.sin(a); return np.array([ca*v[0]-sa*v[1], sa*v[0]+ca*v[1]])
rows=[]
for deg in range(-180,181,5):
    a=np.deg2rad(deg); cc=rot(c,a); hh=rot(hdir,a); yy=rot(yh,a); perp=np.array([-yy[1],yy[0]])
    pts=[cc+R*np.array([np.cos(t),np.sin(t)]) for t in np.linspace(0,2*np.pi,72)]
    pts+= [cc+HL*hh, cc+(HL-0.02)*hh+0.015*perp, cc+(HL-0.02)*hh-0.015*perp]
    fing=[-0.062*yy+0.012*perp,-0.062*yy-0.012*perp,0.062*yy+0.012*perp,0.062*yy-0.012*perp,np.zeros(2)]
    allp=np.vstack([np.array(pts),np.array(fing)])
    zmin=allp[:,1].min(); zmax=allp[:,1].max(); xmin=allp[:,0].min(); xmax=allp[:,0].max()
    zspan=zmax-zmin; zm=(CEIL-FLOOR-zspan)/2; tz=FLOOR+zm-zmin
    xspan=xmax-xmin; xm=(XMAX-XMIN-xspan)/2; tx=XMIN+xm-xmin
    rows.append((min(zm,xm),deg,tz,tx,zm,xm,tz+min(p[1] for p in pts),hh))
rows.sort(key=lambda r:-r[0])
for r in rows[:8]: print(f'roll {r[1]:4d}: margin {r[0]:.3f} (z {r[4]:.3f}, x {r[5]:.3f}) tcp=({r[3]:+.3f},{r[2]:.3f}) body-bottom {r[6]:.3f} handle dir {r[7].round(2)}')
"


# openrua op 45
cat > roll45.py <<'EOF'
import numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from rob import Robot
r = Robot("roll45")
q0 = r.arm_q(); pos, quat = r.tcp(q0)
print("tcp now", pos.round(4), quat.round(4))
Rcur = Rot.from_quat(quat).as_matrix()
Rnew = Rot.from_euler('y', -45, degrees=True).as_matrix() @ Rcur
# check feature mapping: center offset (0.036,·,-0.017) in world at current pose -> hand frame -> new world
c_w = np.array([0.036, 0.0, -0.017]); c_h = Rcur.T @ c_w; print("center offset after roll:", (Rnew @ c_h).round(4))
h_w = np.array([np.cos(np.deg2rad(116)), 0, np.sin(np.deg2rad(116))]); print("handle dir after roll:", (Rnew @ Rcur.T @ h_w).round(3))
print("new y_h (finger axis):", Rnew[:, 1].round(3), " z_h:", Rnew[:, 2].round(3))
qn = Rot.from_matrix(Rnew).as_quat(); print("new quat", qn.round(4))
np.save("snaps/quat_ins.npy", qn)
q = r.ik(pos, qn, seed=q0)
print("ik", None if q is None else np.round(q, 3))
if q is not None:
    r.move_q_corrected(q, seconds=6.0)
    p2, q2 = r.tcp(); print("tcp after", p2.round(4), q2.round(4)); print("fingers", r.fingers())
EOF
python3 -u roll45.py 2>&1 | grep -v "^\[" 

# openrua op 46
python3 scene.py frontview agentview sideview 2>&1 | grep -v "^\[" | tail -3; python3 -c "
import numpy as np
from scipy.optimize import least_squares
P=np.vstack([ (lambda a:a[np.isfinite(a).all(1)])(np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)) for c in ['frontview','agentview','sideview']])
sel=(P[:,1]>0.15)&(P[:,1]<0.21)&(P[:,0]>-0.15)&(P[:,0]<0.10)&(P[:,2]>1.08)&(P[:,2]<1.35); Q=P[sel]; xz=Q[:,[0,2]]
res=lambda c:np.linalg.norm(xz-c,axis=1)-0.047
c=least_squares(res,np.array([-0.03,1.21]),loss='soft_l1',f_scale=0.005).x; d=res(c)
print('center',c.round(4),'inliers',(np.abs(d)<0.008).sum(),'/',len(Q),' offset from TCP', (c-np.array([-0.0692,1.1995])).round(4))
H=Q[d>0.01]; v=H[:,[0,2]]-c; ang=np.degrees(np.arctan2(v[:,1],v[:,0]))
print('handle pts',len(H),'angle median',np.median(ang).round(1),'reach max',np.linalg.norm(v,axis=1).max().round(3))
body=Q[np.abs(d)<0.008]; print('z range all', Q[:,2].min().round(3), Q[:,2].max().round(3), ' x range', Q[:,0].min().round(3), Q[:,0].max().round(3))
# mug y extent along axis using ring
ring=(np.abs(np.linalg.norm(P[:,[0,2]]-c,axis=1)-0.047)<0.008)&(P[:,2]>1.08)&(P[:,1]>0.0)&(P[:,1]<0.4)
print('mug y range', np.percentile(P[ring,1],[1,99]).round(3))
"


# openrua op 47
cat > insert.py <<'EOF'
import numpy as np, rclpy
from rob import Robot
r = Robot("insert")
qn = np.load("snaps/quat_ins.npy")
Z = 0.997; X = -0.061
def go(pts, per=3.0, corrected_last=True):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    times = [per * (i + 1) for i in range(len(qs))]
    r.move_traj(qs, times)
    if corrected_last: r.move_q_corrected(qs[-1], seconds=2.0)
    p, qq = r.tcp(); print("TCP", p.round(4), "quat", qq.round(3), "fingers", np.round(r.fingers(), 4))
print("-- descend")
go([(-0.065, 0.13, 1.10), (X, 0.13, Z)], per=3.0)
print("-- insert")
go([(X, 0.18, Z), (X, 0.23, Z), (X, 0.27, Z), (X, 0.31, Z)], per=2.5)
EOF
python3 -u insert.py 2>&1 | grep -v "^\[" 

# openrua op 48
python3 scene.py agentview sideview frontview birdview 2>&1 | grep -v "^\[" | tail -4; python3 -c "
import numpy as np
from scipy.optimize import least_squares
P=np.vstack([ (lambda a:a[np.isfinite(a).all(1)])(np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)) for c in ['frontview','agentview','sideview','birdview']])
# mug body slice inside cavity: y in [0.30,0.37]
sel=(P[:,1]>0.30)&(P[:,1]<0.37)&(P[:,0]>-0.15)&(P[:,0]<0.06)&(P[:,2]>0.94)&(P[:,2]<1.09); Q=P[sel]; xz=Q[:,[0,2]]
print('pts',len(Q))
if len(Q)>50:
    res=lambda c:np.linalg.norm(xz-c,axis=1)-0.047
    c=least_squares(res,np.array([-0.02,1.01]),loss='soft_l1',f_scale=0.005).x; d=res(c)
    print('center',c.round(4),'inliers',(np.abs(d)<0.008).sum(), 'z range',Q[:,2].min().round(3),Q[:,2].max().round(3),'x range',Q[:,0].min().round(3),Q[:,0].max().round(3))
# anything near the cavity ceiling/front top edge region
for name,(y0,y1) in {'front of face y<0.26':(0.2,0.26),'inside 0.26-0.30':(0.26,0.30),'inside 0.30-0.40':(0.30,0.40)}.items():
    s=(P[:,1]>y0)&(P[:,1]<y1)&(P[:,0]>-0.17)&(P[:,0]<0.07)&(P[:,2]>0.93)&(P[:,2]<1.10)
    if s.sum(): print(name,'n',s.sum(),'z',P[s,2].min().round(3),P[s,2].max().round(3),'x',P[s,0].min().round(3),P[s,0].max().round(3))
"


# openrua op 49
python3 -c "
import cv2
a=cv2.imread('snaps/agentview_rgb.png'); s=cv2.imread('snaps/sideview_rgb.png'); f=cv2.imread('snaps/frontview_rgb.png'); b=cv2.imread('snaps/birdview_rgb.png')
print(a.shape)
import numpy as np
top=np.hstack([a,s]); bot=np.hstack([f,b]); cv2.imwrite('snaps/quad_insert.png', np.vstack([top,bot]))
"

# openrua op 50
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_ins.png 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py robot0_robotview snaps/rv_ins.png 2>&1 | grep -v "^\["; python3 -c "
import cv2,numpy as np
a=cv2.imread('snaps/eih_ins.png'); b=cv2.imread('snaps/rv_ins.png'); print(a.shape,b.shape)
cv2.imwrite('snaps/pair_ins.png', np.hstack([a,b]))"

# openrua op 51
cat > release.py <<'EOF'
import numpy as np
from rob import Robot
r = Robot("release")
qn = np.load("snaps/quat_ins.npy")
print("-- open"); r.gripper(0.04)
X = -0.061
def go(pts, per=3.0):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=2)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
print("-- retreat"); go([(X, 0.25, 1.00), (X, 0.19, 1.00)], per=2.5)
print("-- lift"); go([(X, 0.19, 1.10), (X, 0.17, 1.22)], per=2.5)
EOF
python3 -u release.py 2>&1 | grep -v "^\["

# openrua op 52
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_rel.png 2>&1 | grep -v "^\["; python3 scene.py sideview agentview birdview 2>&1 | grep -v "^\[" | tail -1; python3 -c "
import numpy as np, cv2
P=np.vstack([ (lambda a:a[np.isfinite(a).all(1)])(np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)) for c in ['sideview','agentview','birdview']])
s=(P[:,1]>0.265)&(P[:,1]<0.44)&(P[:,0]>-0.16)&(P[:,0]<0.06)&(P[:,2]>0.945)&(P[:,2]<1.085)
print('points inside cavity volume:',s.sum())
if s.sum():
    Q=P[s]; print('x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
    # column histogram in y
    h,_=np.histogram(Q[:,1],bins=np.arange(0.26,0.45,0.02)); print('y hist',h)
a=cv2.imread('snaps/eih_rel.png'); b=cv2.imread('snaps/sideview_rgb.png'); cv2.imwrite('snaps/pair_rel.png',np.hstack([a,b]))
"

# openrua op 53
cat > peek.py <<'EOF'
import numpy as np
from rob import Robot
r = Robot("peek")
qn = np.load("snaps/quat_ins.npy")
q = r.ik(np.array([-0.06, 0.08, 1.03]), qn, seed=r.arm_q())
assert q is not None
r.move_q_corrected(q, seconds=4.0, iters=2)
print("TCP", r.tcp()[0].round(4))
EOF
python3 -u peek.py 2>&1 | grep -v "^\[" && python3 scene.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,1]>0.265)&(P[:,1]<0.45)&(P[:,0]>-0.155)&(P[:,0]<0.055)&(P[:,2]>0.945)&(P[:,2]<1.085)
Q=P[s]; print('pts in cavity volume',len(Q))
print('x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
# exclude back wall (y>0.43) and floor (z<0.955)
m=(Q[:,1]<0.42)&(Q[:,2]>0.955)&(Q[:,0]>-0.15)&(Q[:,0]<0.05); M=Q[m]; print('mug-candidate pts',len(M))
if len(M):
  print('x',M[:,0].min().round(3),M[:,0].max().round(3),'y',M[:,1].min().round(3),M[:,1].max().round(3),'z',M[:,2].min().round(3),M[:,2].max().round(3))
  from scipy.optimize import least_squares
  sl=M[(M[:,1]>0.30)&(M[:,1]<0.36)]; xz=sl[:,[0,2]]
  res=lambda c:np.linalg.norm(xz-c,axis=1)-0.047
  c=least_squares(res,np.array([-0.02,0.99]),loss='soft_l1',f_scale=0.005).x; d=res(c); print('circle center',c.round(4),'inliers',(abs(d)<0.008).sum(),'/',len(sl))
"


# openrua op 54
python3 -c "
import numpy as np, cv2
a=np.load('snaps/robot0_eye_in_hand_xyz.npy'); H,W,_=a.shape; P=a.reshape(-1,3); ok=np.isfinite(P).all(1)
# occupancy map (x,y) top-down of points with z in (0.955,1.085), y in (0.265,0.43)
s=ok&(P[:,2]>0.955)&(P[:,2]<1.085)&(P[:,1]>0.265)&(P[:,1]<0.43)&(P[:,0]>-0.16)&(P[:,0]<0.06)
Q=P[s]
xb=np.arange(-0.16,0.061,0.01); yb=np.arange(0.26,0.44,0.01)
h,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xb,yb])
print('rows x from -0.16 step .01; cols y from 0.26 step .01')
for i,row in enumerate(h): print(f'{xb[i]:+.2f} '+''.join('#' if v>30 else ('.' if v>3 else ' ') for v in row))
# z profile of mug region (x -0.07..0.03, y 0.29..0.36)
m=s&(P[:,0]>-0.07)&(P[:,0]<0.03)&(P[:,1]>0.29)&(P[:,1]<0.36); print('z pct in mug region',np.percentile(P[m,2],[1,50,99]).round(3),'n',m.sum())
cv2.imwrite('snaps/eih_peek.png', cv2.imread('snaps/robot0_eye_in_hand_rgb.png'))
"

# openrua op 55
python3 -c "
import numpy as np
a=np.load('snaps/birdview_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,2]>0.905)&(P[:,2]<1.05)&(P[:,0]>-0.3)&(P[:,0]<0.3)&(P[:,1]>-0.2)&(P[:,1]<0.24)
Q=P[s]; print('n',len(Q))
xb=np.arange(-0.3,0.31,0.02); yb=np.arange(-0.2,0.25,0.02)
h,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xb,yb])
print('rows x from -0.30 step .02; cols y from -0.20 step .02')
for i,row in enumerate(h): print(f'{xb[i]:+.2f} '+''.join('#' if v>8 else ('.' if v>1 else ' ') for v in row))
m=(Q[:,1]<0.2); print('yellow-mug-ish pts z range',Q[m,2].min().round(3),Q[m,2].max().round(3))
"

# openrua op 56
cat > nudge.py <<'EOF'
import numpy as np
from rob import Robot, approach_quat
r = Robot("nudge")
phi = np.deg2rad(30)
qn = approach_quat(np.array([0, np.cos(phi), -np.sin(phi)]), np.array([1.0, 0, 0]))
np.save("snaps/quat_push.npy", qn)
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "quat", qq.round(3), "fingers", np.round(r.fingers(), 4))
X = -0.016; Z = 1.00
print("-- approach"); go([(X, 0.12, 1.15), (X, 0.20, 1.02), (X, 0.245, Z)], per=3.0)
EOF
python3 -u nudge.py 2>&1 | grep -v "^\["

# openrua op 57
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_prepush.png 2>&1 | grep -v "^\["

# openrua op 58
cat > nudge2.py <<'EOF'
import numpy as np
from rob import Robot
r = Robot("nudge2")
qn = np.load("snaps/quat_push.npy")
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
X = -0.016; Z = 1.00
print("-- push"); go([(X, 0.27, Z), (X, 0.29, Z)], per=2.5)
print("-- back off"); go([(X, 0.22, 1.02), (X, 0.12, 1.15)], per=3.0, iters=1)
EOF
python3 -u nudge2.py 2>&1 | grep -v "^\[" && python3 scene.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,2]>0.955)&(P[:,2]<1.085)&(P[:,1]>0.265)&(P[:,1]<0.43)&(P[:,0]>-0.16)&(P[:,0]<0.06); Q=P[s]
xb=np.arange(-0.16,0.061,0.01); yb=np.arange(0.26,0.44,0.01)
h,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xb,yb])
print('rows x from -0.16 step .01; cols y from 0.26 step .01')
for i,row in enumerate(h): print(f'{xb[i]:+.2f} '+''.join('#' if v>30 else ('.' if v>3 else ' ') for v in row))
m=s&(P[:,0]>-0.07)&(P[:,0]<0.04); print('mug y min (p1)',np.percentile(P[m,1],1).round(3),'z pct',np.percentile(P[m,2],[1,50,99]).round(3))
"

# openrua op 59
cat > look.py <<'EOF'
import numpy as np, sys
from rob import Robot
r = Robot("look")
qn = np.load("snaps/quat_push.npy")
p = np.array([float(v) for v in sys.argv[1:4]])
q = r.ik(p, qn, seed=r.arm_q()); assert q is not None
r.move_q_corrected(q, seconds=3.0, iters=1)
print("TCP", r.tcp()[0].round(4))
EOF
python3 -u look.py -0.016 0.16 1.03 2>&1 | grep -v "^\[" && python3 scene.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
# side view occupancy (y,z) for x in mug range
s=(P[:,0]>-0.06)&(P[:,0]<0.03)&(P[:,1]>0.20)&(P[:,1]<0.45)&(P[:,2]>0.93)&(P[:,2]<1.10); Q=P[s]
yb=np.arange(0.20,0.46,0.01); zb=np.arange(0.93,1.11,0.01)
h,_,_=np.histogram2d(Q[:,1],Q[:,2],bins=[yb,zb])
print('rows y from 0.20 step .01; cols z from 0.93 step .01')
for i,row in enumerate(h): print(f'{yb[i]:.2f} '+''.join('#' if v>30 else ('.' if v>3 else ' ') for v in row))
# rim: points with z in [0.97,1.03] (mid-height) and x within +-0.02 of center -> the frontmost (min y) among mug body points
m=(P[:,0]>-0.03)&(P[:,0]<0.0)&(P[:,2]>0.975)&(P[:,2]<1.02)&(P[:,1]>0.25)&(P[:,1]<0.45)
print('front-most y at mid-height (p1,p5):',np.percentile(P[m,1],[1,5]).round(3),'n',m.sum())
"

# openrua op 60
python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,1]>0.28)&(P[:,1]<0.35)&(P[:,2]>0.95)&(P[:,2]<1.06)&(P[:,0]>-0.12)&(P[:,0]<0.08); Q=P[s]
# for x bins, max z of body -> ridge
xb=np.arange(-0.12,0.081,0.01)
for i in range(len(xb)-1):
    m=(Q[:,0]>=xb[i])&(Q[:,0]<xb[i+1])
    if m.sum()>5: print(f'x {xb[i]:+.2f}: n={m.sum():4d} zmax={Q[m,2].max():.3f} zmin={Q[m,2].min():.3f}')
# circle fit on y-slice with free center fixed r on all points z>0.95
from scipy.optimize import least_squares
xz=Q[:,[0,2]]; res=lambda c:np.linalg.norm(xz-c,axis=1)-0.047
for c0 in [(-0.02,0.99),(0.0,0.99),(0.02,0.99)]:
    c=least_squares(res,np.array(c0),loss='soft_l1',f_scale=0.004).x; d=res(c); print('fit from',c0,'->',c.round(4),'inliers',(abs(d)<0.006).sum(),'/',len(Q))
"

# openrua op 61
cat > nudge3.py <<'EOF'
import numpy as np
from rob import Robot
r = Robot("nudge3")
qn = np.load("snaps/quat_push.npy")
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
X = 0.0; Z = 1.00
print("-- approach"); go([(X, 0.22, Z)], per=3.0)
print("-- push"); go([(X, 0.26, Z), (X, 0.29, Z), (X, 0.31, Z)], per=2.0)
print("-- back off"); go([(X, 0.16, 1.03)], per=3.0, iters=1)
EOF
python3 -u nudge3.py 2>&1 | grep -v "^\[" && python3 scene.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.06)&(P[:,0]<0.04)&(P[:,1]>0.20)&(P[:,1]<0.45)&(P[:,2]>0.93)&(P[:,2]<1.10); Q=P[s]
yb=np.arange(0.20,0.46,0.01); zb=np.arange(0.93,1.11,0.01)
h,_,_=np.histogram2d(Q[:,1],Q[:,2],bins=[yb,zb])
print('rows y from 0.20 step .01; cols z from 0.93 step .01')
for i,row in enumerate(h): print(f'{yb[i]:.2f} '+''.join('#' if v>30 else ('.' if v>3 else ' ') for v in row))
m=(P[:,0]>-0.03)&(P[:,0]<0.03)&(P[:,2]>0.975)&(P[:,2]<1.02)&(P[:,1]>0.22)&(P[:,1]<0.42)
print('front-most y at mid-height (p1,p5):',np.percentile(P[m,1],[1,5]).round(3),'n',m.sum())
"

# openrua op 62
cat > nudge4.py <<'EOF'
import numpy as np
from rob import Robot
r = Robot("nudge4")
qn = np.load("snaps/quat_push.npy")
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
print("-- close"); r.gripper(0.0)
X = 0.0; Z = 1.048
print("-- approach"); go([(X, 0.20, Z)], per=3.0)
print("-- push"); go([(X, 0.25, Z), (X, 0.28, Z), (X, 0.30, Z)], per=2.0)
print("-- back off"); go([(X, 0.16, 1.03)], per=3.0, iters=1)
EOF
python3 -u nudge4.py 2>&1 | grep -v "^\[" && python3 scene.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.06)&(P[:,0]<0.04)&(P[:,1]>0.20)&(P[:,1]<0.45)&(P[:,2]>0.93)&(P[:,2]<1.10); Q=P[s]
yb=np.arange(0.20,0.46,0.01); zb=np.arange(0.93,1.11,0.01)
h,_,_=np.histogram2d(Q[:,1],Q[:,2],bins=[yb,zb])
print('rows y from 0.20 step .01; cols z from 0.93 step .01')
for i,row in enumerate(h): print(f'{yb[i]:.2f} '+''.join('#' if v>30 else ('.' if v>3 else ' ') for v in row))
m=(P[:,0]>-0.03)&(P[:,0]<0.03)&(P[:,2]>0.975)&(P[:,2]<1.02)&(P[:,1]>0.22)&(P[:,1]<0.42)
print('front-most y at mid-height (p1,p5):',np.percentile(P[m,1],[1,5]).round(3),'n',m.sum())
"

# openrua op 63
python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
# empty floor strip left of mug
s=(P[:,0]>-0.14)&(P[:,0]<-0.09)&(P[:,1]>0.22)&(P[:,1]<0.45)&(P[:,2]>0.90)&(P[:,2]<1.0); Q=P[s]
for y0 in np.arange(0.22,0.45,0.01):
    m=(Q[:,1]>=y0)&(Q[:,1]<y0+0.01)
    if m.sum()>3: print(f'y {y0:.2f}: n={m.sum():4d} z min/med/max {Q[m,2].min():.3f} {np.median(Q[m,2]):.3f} {Q[m,2].max():.3f}')
print('--- mug bottom line (x -0.02..0.02)')
s=(P[:,0]>-0.02)&(P[:,0]<0.02)&(P[:,1]>0.22)&(P[:,1]<0.40)&(P[:,2]>0.90)&(P[:,2]<0.99); Q=P[s]
for y0 in np.arange(0.22,0.40,0.01):
    m=(Q[:,1]>=y0)&(Q[:,1]<y0+0.01)
    if m.sum()>3: print(f'y {y0:.2f}: n={m.sum():4d} z min/med/max {Q[m,2].min():.3f} {np.median(Q[m,2]):.3f} {Q[m,2].max():.3f}')
"

# openrua op 64
cat > nudge5.py <<'EOF'
import numpy as np
from rob import Robot, approach_quat
r = Robot("nudge5")
phi = np.deg2rad(10)
qn = approach_quat(np.array([0, np.cos(phi), -np.sin(phi)]), np.array([1.0, 0, 0]))
np.save("snaps/quat_push10.npy", qn)
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
X = 0.0; Z = 1.050
print("-- approach"); go([(X, 0.12, 1.12), (X, 0.20, Z)], per=3.0)
print("-- push"); go([(X, 0.25, Z), (X, 0.28, Z), (X, 0.30, Z)], per=2.0)
print("-- back off"); go([(X, 0.20, Z)], per=3.0, iters=1)
EOF
python3 -u nudge5.py 2>&1 | grep -v "^\[" && python3 scene.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.06)&(P[:,0]<0.04)&(P[:,1]>0.20)&(P[:,1]<0.45)&(P[:,2]>0.93)&(P[:,2]<1.10); Q=P[s]
yb=np.arange(0.20,0.46,0.01); zb=np.arange(0.93,1.11,0.01)
h,_,_=np.histogram2d(Q[:,1],Q[:,2],bins=[yb,zb])
print('rows y from 0.20 step .01; cols z from 0.93 step .01')
for i,row in enumerate(h): print(f'{yb[i]:.2f} '+''.join('#' if v>30 else ('.' if v>3 else ' ') for v in row))
"

# openrua op 65
cat > regrasp.py <<'EOF'
import numpy as np
from rob import Robot
r = Robot("regrasp")
qn = np.load("snaps/quat_push.npy")   # 30 deg pitched-down approach, fingers along x
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
X, Z = -0.042, 0.982
print("-- open"); r.gripper(0.04)
print("-- approach"); go([(X, 0.16, 1.03), (X, 0.22, Z), (X, 0.275, Z)], per=3.0)
print("-- pinch"); f = r.gripper(0.0)
EOF
python3 -u regrasp.py 2>&1 | grep -v "^\["

# openrua op 66
cat > carry.py <<'EOF'
import numpy as np
from rob import Robot
r = Robot("carry")
qn = np.load("snaps/quat_push.npy")
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
X = -0.042
print("-- lift"); go([(X, 0.275, 0.998)], per=2.5)
print("-- deeper"); go([(X, 0.295, 0.998), (X, 0.315, 0.998)], per=2.5)
print("-- lower"); go([(X, 0.315, 0.985)], per=2.5)
print("-- open"); r.gripper(0.04)
print("-- retreat"); go([(X, 0.22, 0.99), (X, 0.16, 1.03)], per=3.0, iters=1)
EOF
python3 -u carry.py 2>&1 | grep -v "^\[" && python3 scene.py robot0_eye_in_hand 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.06)&(P[:,0]<0.05)&(P[:,1]>0.20)&(P[:,1]<0.45)&(P[:,2]>0.93)&(P[:,2]<1.10); Q=P[s]
yb=np.arange(0.20,0.46,0.01); zb=np.arange(0.93,1.11,0.01)
h,_,_=np.histogram2d(Q[:,1],Q[:,2],bins=[yb,zb])
print('rows y from 0.20 step .01; cols z from 0.93 step .01')
for i,row in enumerate(h): print(f'{yb[i]:.2f} '+''.join('#' if v>30 else ('.' if v>3 else ' ') for v in row))
"

# openrua op 67
cat > doorplan.py <<'EOF'
import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("doorplan")
H = np.array([-0.18, 0.26]); S = 0.20; Z = 1.00; DELTA = 0.015; PITCH = np.deg2rad(45)
def door_pose(alpha_deg, delta=DELTA):
    a = np.deg2rad(alpha_deg)
    d = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([np.sin(a), -np.cos(a), 0.0])
    p = np.array([*(H + S * d[:2]), Z])
    tcp = p + delta * n
    z_h = np.cos(PITCH) * (-n) - np.sin(PITCH) * np.array([0, 0, 1.0])
    y_h = d; x_h = np.cross(y_h, z_h)
    return tcp, quat_from_axes(x_h, y_h, z_h)
if __name__ == "__main__":
    seed = r.arm_q(); prev = None
    alphas = [-125] + list(range(-125, 1, 5))
    deltas = [0.06] + [DELTA] * (len(alphas) - 1)
    for a, dl in zip(alphas, deltas):
        tcp, q = door_pose(a, dl)
        sol = r.ik(tcp, q, seed=seed, timeout=3.0)
        if sol is None:
            print(f"alpha {a:5d} delta {dl:.3f} tcp {tcp.round(3)} IK FAIL"); continue
        jump = 0 if prev is None else np.max(np.abs(np.array(sol) - np.array(prev)))
        print(f"alpha {a:5d} tcp {tcp.round(3)} q {np.round(sol,2)} maxjump {jump:.2f}")
        seed = sol; prev = sol
EOF
python3 -u doorplan.py 2>&1 | grep -v "^\["

# openrua op 68
cat > doorplan2.py <<'EOF'
import numpy as np, sys
from rob import Robot, quat_from_axes
H = np.array([-0.18, 0.26]); Z = 1.00; DELTA = 0.015; PITCH = np.deg2rad(45)
def door_pose(alpha_deg, s, delta=DELTA, z=Z):
    a = np.deg2rad(alpha_deg)
    d = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([np.sin(a), -np.cos(a), 0.0])
    tcp = np.array([*(H + s * d[:2]), z]) + delta * n
    z_h = np.cos(PITCH) * (-n) - np.sin(PITCH) * np.array([0, 0, 1.0])
    y_h = d; x_h = np.cross(y_h, z_h)
    return tcp, quat_from_axes(x_h, y_h, z_h)
if __name__ == "__main__":
    r = Robot("doorplan2")
    S = float(sys.argv[1])
    for seed0 in ([0.5, 0.2, -0.3, -2.2, 0.0, 2.4, 0.8], [0.3, -0.6, 0.2, -2.8, 0.1, 2.3, 0.9], r.arm_q()):
        print("seed", np.round(seed0, 2)); seed = seed0; prev = None
        for a in [-125, -110, -90, -70, -50, -30, -15, 0]:
            tcp, q = door_pose(a, S)
            sol = r.ik(tcp, q, seed=seed, timeout=3.0)
            if sol is None: print(f"  alpha {a:5d} tcp {tcp.round(3)} FAIL"); continue
            jump = 0 if prev is None else np.max(np.abs(np.array(sol) - np.array(prev)))
            print(f"  alpha {a:5d} tcp {tcp.round(3)} q {np.round(sol,2)} jump {jump:.2f}")
            seed = sol; prev = sol
EOF
python3 -u doorplan2.py 0.10 2>&1 | grep -v "^\["

# openrua op 69
python3 scene.py birdview 2>&1 | grep -v "^\[" | tail -1; python3 -c "
import numpy as np
a=np.load('snaps/birdview_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
# door region: x in [-0.40,-0.15], y in [0.0,0.30], z>1.0
s=(P[:,0]>-0.40)&(P[:,0]<-0.15)&(P[:,1]>0.0)&(P[:,1]<0.30)&(P[:,2]>1.0); Q=P[s]
print('door-ish pts',len(Q),'z pct',np.percentile(Q[:,2],[5,50,95]).round(3))
top=Q[Q[:,2]>1.09]
# fit a line to (x,y) of top points
xy=top[:,:2]; c=xy.mean(0); u,sv,vt=np.linalg.svd(xy-c); dirv=vt[0]; print('door top line center',c.round(3),'dir',dirv.round(3),'angle deg',np.degrees(np.arctan2(dirv[1],dirv[0])).round(1))
t=(xy-c)@dirv; print('extent along line',t.min().round(3),t.max().round(3),'ends',(c+t.min()*dirv).round(3),(c+t.max()*dirv).round(3))
w=(xy-c)@np.array([-dirv[1],dirv[0]]); print('thickness across',w.min().round(3),w.max().round(3))
print('door top z', np.percentile(top[:,2],[5,50,95]).round(3))
"

# openrua op 70
python3 -c "
import numpy as np
a=np.load('snaps/birdview_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.45)&(P[:,0]<-0.15)&(P[:,1]>-0.05)&(P[:,1]<0.30)&(P[:,2]>1.09)&(P[:,2]<1.13); Q=P[s]
print('door top pts',len(Q),'z pct',np.percentile(Q[:,2],[5,50,95]).round(4))
xy=Q[:,:2]; c=xy.mean(0); u,sv,vt=np.linalg.svd(xy-c); dirv=vt[0]
if dirv[0]>0: dirv=-dirv
print('center',c.round(3),'dir',dirv.round(3),'angle deg',np.degrees(np.arctan2(dirv[1],dirv[0])).round(1))
t=(xy-c)@dirv; w=(xy-c)@np.array([-dirv[1],dirv[0]])
print('ends',(c+t.min()*dirv).round(3),(c+t.max()*dirv).round(3),'len',(t.max()-t.min()).round(3),'thickness',(w.max()-w.min()).round(3))
# occupancy print
xb=np.arange(-0.45,-0.14,0.02); yb=np.arange(-0.05,0.31,0.02)
h,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xb,yb])
for i,row in enumerate(h): print(f'{xb[i]:+.2f} '+''.join('#' if v>5 else ('.' if v>0 else ' ') for v in row))
"

# openrua op 71
cat > neutral.py <<'EOF'
import numpy as np
from rob import Robot, topdown_quat
r = Robot("neutral")
qn = topdown_quat([1, 0, 0])
q = r.ik(np.array([-0.15, -0.05, 1.40]), qn, seed=r.arm_q())
assert q is not None, "IK fail"
r.move_q_corrected(q, seconds=5.0, iters=1)
print("TCP", r.tcp()[0].round(3), "q", np.round(r.arm_q(), 2))
EOF
python3 -u neutral.py 2>&1 | grep -v "^\[" && python3 scene.py birdview agentview frontview sideview 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
P=np.vstack([ (lambda a:a[np.isfinite(a).all(1)])(np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)) for c in ['birdview','agentview','frontview','sideview']])
s=(P[:,0]>-0.45)&(P[:,0]<-0.17)&(P[:,1]>-0.05)&(P[:,1]<0.25)&(P[:,2]>1.09)&(P[:,2]<1.13); Q=P[s]
print('door top pts',len(Q),'z',np.percentile(Q[:,2],[5,50,95]).round(4))
xy=Q[:,:2]; c=xy.mean(0); u,sv,vt=np.linalg.svd(xy-c); dirv=vt[0]
if dirv[0]>0: dirv=-dirv
t=(xy-c)@dirv; w=(xy-c)@np.array([-dirv[1],dirv[0]])
print('dir',dirv.round(3),'angle',np.degrees(np.arctan2(dirv[1],dirv[0])).round(1),'ends',(c+t.min()*dirv).round(3),(c+t.max()*dirv).round(3),'len',(t.max()-t.min()).round(3),'thick',(w.max()-w.min()).round(3))
# door faces: points with z in [0.95,1.05] in that region -> two parallel lines
s2=(P[:,0]>-0.45)&(P[:,0]<-0.17)&(P[:,1]>-0.05)&(P[:,1]<0.25)&(P[:,2]>0.95)&(P[:,2]<1.05); Q2=P[s2]
w2=(Q2[:,:2]-c)@np.array([-dirv[1],dirv[0]]); t2=(Q2[:,:2]-c)@dirv
print('face pts',len(Q2)); h,e=np.histogram(w2,bins=np.arange(-0.06,0.061,0.005)); print('across-door histogram (m from top-center):'); [print(f'{e[i]:+.3f} {v}') for i,v in enumerate(h) if v>0]
"

# openrua op 72
python3 -c "
import numpy as np
P=np.vstack([ (lambda a:a[np.isfinite(a).all(1)])(np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)) for c in ['birdview','agentview','frontview','sideview']])
H=np.array([-0.18,0.26]); a=np.deg2rad(-111.3); d=np.array([np.cos(a),np.sin(a)]); n=np.array([np.sin(a),-np.cos(a)])
s=(P[:,0]>-0.5)&(P[:,0]<-0.15)&(P[:,1]>-0.1)&(P[:,1]<0.27)&(P[:,2]>0.9)&(P[:,2]<1.12); Q=P[s]
t=(Q[:,:2]-H)@d; w=(Q[:,:2]-H)@n
door=(t>0)&(t<0.26)
for lo,hi,name in [(-0.02,0.005,'inner side'),(0.005,0.02,'outer face'),(0.02,0.08,'handle')]:
    m=door&(w>lo)&(w<hi)
    if m.sum(): print(f'{name}: n={m.sum()} t range {t[m].min():.3f}..{t[m].max():.3f} z range {Q[m,2].min():.3f}..{Q[m,2].max():.3f}  w pct {np.percentile(w[m],[5,50,95]).round(3)}')
m=door&(w>0.02)&(w<0.08)
zb=np.arange(0.9,1.13,0.02); tb=np.arange(0,0.27,0.02)
h,_,_=np.histogram2d(t[m],Q[m,2],bins=[tb,zb]); print('handle occupancy rows t (from hinge), cols z from 0.90 step .02')
for i,row in enumerate(h): print(f't {tb[i]:.2f} '+''.join('#' if v>5 else ('.' if v>0 else ' ') for v in row))
"

# openrua op 73
cat > door.py <<'EOF'
import numpy as np, sys
from rob import Robot, quat_from_axes
H = np.array([-0.18, 0.26]); S = 0.12; ZT = 1.085; W = 0.017
def door_pose(alpha_deg, w=W, z=ZT, s=S):
    a = np.deg2rad(alpha_deg)
    d = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([np.sin(a), -np.cos(a), 0.0])
    tcp = np.array([*(H + s * d[:2]), z]) + w * n
    z_h = np.array([0, 0, -1.0]); y_h = d; x_h = np.cross(y_h, z_h)
    return tcp, quat_from_axes(x_h, y_h, z_h)
def run_arc(r, a0, a1, step=5, per=1.0):
    alphas = list(np.arange(a0, a1 + 1e-6, step if a1 > a0 else -step))
    qs = []; seed = r.arm_q()
    for a in alphas:
        tcp, q = door_pose(a)
        sol = r.ik(tcp, q, seed=seed, timeout=3.0)
        if sol is None: raise SystemExit(f"IK fail at alpha {a}")
        if qs and np.max(np.abs(np.array(sol) - np.array(qs[-1]))) > 0.6:
            raise SystemExit(f"joint jump at alpha {a}: {np.round(np.array(sol)-np.array(qs[-1]),2)}")
        qs.append(sol); seed = sol
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    p, _ = r.tcp(); print("arc end TCP", p.round(4), "target", door_pose(a1)[0].round(4))
if __name__ == "__main__":
    r = Robot("door")
    mode = sys.argv[1]
    if mode == "approach":
        A0 = float(sys.argv[2])
        r.gripper(0.0)
        for (w, z) in [(0.05, 1.30), (0.05, ZT), (W, ZT)]:
            tcp, q = door_pose(A0, w=w, z=z)
            sol = r.ik(tcp, q, seed=r.arm_q(), timeout=3.0); assert sol is not None
            r.move_q_corrected(sol, seconds=4.0, iters=2)
            print("TCP", r.tcp()[0].round(4), "q", np.round(r.arm_q(), 2))
    elif mode == "arc":
        run_arc(r, float(sys.argv[2]), float(sys.argv[3]))
    elif mode == "up":
        p, q = r.tcp(); tcp = p.copy(); tcp[2] = 1.30
        sol = r.ik(tcp, q, seed=r.arm_q()); r.move_q_corrected(sol, seconds=3.0, iters=1)
        print("TCP", r.tcp()[0].round(4))
EOF
python3 -u door.py approach -111 2>&1 | grep -v "^\["

# openrua op 74
python3 -u door.py arc -111 -75 2>&1 | grep -v "^\[" && python3 scene.py birdview 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/birdview_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.45)&(P[:,0]<-0.10)&(P[:,1]>-0.1)&(P[:,1]<0.25)&(P[:,2]>1.09)&(P[:,2]<1.115); Q=P[s]
xy=Q[:,:2]; c=xy.mean(0); u,sv,vt=np.linalg.svd(xy-c); dirv=vt[0]
if dirv[0]>0: dirv=-dirv
t=(xy-c)@dirv
print('door top pts',len(Q),'angle',np.degrees(np.arctan2(dirv[1],dirv[0])).round(1),'ends',(c+t.min()*dirv).round(3),(c+t.max()*dirv).round(3))
"

# openrua op 75
python3 -u door.py arc -75 -40 2>&1 | grep -v "^\[" && python3 scene.py birdview 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
a=np.load('snaps/birdview_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.45)&(P[:,0]<0.10)&(P[:,1]>-0.1)&(P[:,1]<0.25)&(P[:,2]>1.09)&(P[:,2]<1.115); Q=P[s]
xy=Q[:,:2]; c=xy.mean(0); u,sv,vt=np.linalg.svd(xy-c); dirv=vt[0]
if dirv[0]>0: dirv=-dirv
t=(xy-c)@dirv
print('door top pts',len(Q),'angle',np.degrees(np.arctan2(dirv[1],dirv[0])).round(1),'ends',(c+t.min()*dirv).round(3),(c+t.max()*dirv).round(3))
"

# openrua op 76
python3 -c "
import numpy as np, cv2
a=np.load('snaps/birdview_xyz.npy').reshape(-1,3); P=a[np.isfinite(a).all(1)]
s=(P[:,0]>-0.3)&(P[:,0]<0.12)&(P[:,1]>-0.1)&(P[:,1]<0.24)&(P[:,2]>1.09)&(P[:,2]<1.115); Q=P[s]
xb=np.arange(-0.3,0.13,0.02); yb=np.arange(-0.1,0.25,0.02)
h,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xb,yb])
print('rows x, cols y from -0.10 step .02 (door-top-height points)')
for i,row in enumerate(h): print(f'{xb[i]:+.2f} '+''.join('#' if v>3 else ('.' if v>0 else ' ') for v in row))
img=cv2.imread('snaps/birdview_rgb.png'); cv2.imwrite('snaps/bird_crop.png', cv2.resize(img[430:850,600:1000],None,fx=1.5,fy=1.5))
" ; echo

# openrua op 77
python3 tools/perception/cam_snap.py agentview snaps/agent_door.png 2>&1 | grep -v "^\["; python3 -c "
import cv2, numpy as np
b=cv2.imread('snaps/birdview_rgb.png'); a=cv2.imread('snaps/agent_door.png')
crop=b[150:420,200:520]; crop=cv2.resize(crop,None,fx=2,fy=2)
cv2.imwrite('snaps/door_check.png', np.hstack([a, cv2.resize(crop,(640,480))]))"

# openrua op 78
python3 -u door.py up 2>&1 | grep -v "^\[" && python3 scene.py birdview agentview 2>&1 | grep -v "^\[" | tail -1 && python3 -c "
import numpy as np
P=np.vstack([ (lambda a:a[np.isfinite(a).all(1)])(np.load(f'snaps/{c}_xyz.npy').reshape(-1,3)) for c in ['birdview','agentview']])
s=(P[:,2]>0.905)&(P[:,2]<1.05)&(P[:,0]>-0.1)&(P[:,0]<0.3)&(P[:,1]>-0.15)&(P[:,1]<0.2); Q=P[s]
xb=np.arange(-0.1,0.31,0.01); yb=np.arange(-0.15,0.21,0.01)
h,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xb,yb])
print('rows x from -0.10 step .01; cols y from -0.15 step .01')
for i,row in enumerate(h):
    if row.sum()>0: print(f'{xb[i]:+.2f} '+''.join('#' if v>8 else ('.' if v>1 else ' ') for v in row))
m=(Q[:,0]<0.2); print('z max', Q[m,2].max().round(3))
# door free end / handle points at same height
"

# openrua op 79
python3 -c "
import cv2, numpy as np
a=cv2.imread('snaps/agentview_rgb.png'); b=cv2.imread('snaps/birdview_rgb.png')
ca=cv2.resize(a[220:420,300:520],None,fx=3,fy=3); cb=cv2.resize(b[200:360,300:460],None,fx=3.75,fy=3.75)
cv2.imwrite('snaps/mug_door.png', np.hstack([ca, cb[:600,:600]]))"

# openrua op 80
python3 - <<'EOF'
import numpy as np, cv2
for cam in ['agentview','birdview']:
    rgb=cv2.imread(f'snaps/{cam}_rgb.png'); xyz=np.load(f'snaps/{cam}_xyz.npy')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
    yel=(hsv[...,0]>15)&(hsv[...,0]<40)&(hsv[...,1]>90)&(hsv[...,2]>120)
    p=xyz[yel]; p=p[np.isfinite(p).all(1)]
    p=p[(p[:,2]>0.9)&(p[:,2]<1.1)]
    print(cam,'yellow pts',len(p))
    if len(p): print(' x',p[:,0].min().round(3),p[:,0].max().round(3),' y',p[:,1].min().round(3),p[:,1].max().round(3),' z',p[:,2].min().round(3),p[:,2].max().round(3))
    # white-ish mug body (bright, low sat) near the yellow region
    wh=(hsv[...,1]<40)&(hsv[...,2]>170)
    q=xyz[wh]; q=q[np.isfinite(q).all(1)]
    q=q[(q[:,2]>0.905)&(q[:,2]<1.02)&(q[:,0]>-0.05)&(q[:,0]<0.25)&(q[:,1]>-0.05)&(q[:,1]<0.23)]
    if len(q): print(' white body x',q[:,0].min().round(3),q[:,0].max().round(3),' y',q[:,1].min().round(3),q[:,1].max().round(3),' z',q[:,2].min().round(3),q[:,2].max().round(3))
EOF

# openrua op 81
cat /workspace/door.py | head -60; grep -n "def go" -A 25 /workspace/insert.py

# openrua op 82
mkdir -p "$(dirname /workspace/push_yellow.py)"
cat > /workspace/push_yellow.py <<'OPENRUA_EOF'
"""Push the fallen yellow mug out of the door sweep: top-down closed fingertips
on its +y side, then slide it toward -y (away from the microwave)."""
import numpy as np
from rob import Robot, topdown_quat
r = Robot("pushy")
qn = topdown_quat([1, 0, 0])   # fingers side by side along x, palm thin along y
X, Z = 0.09, 0.965

def go(pts, per=3.0, corrected_last=True):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed, timeout=3.0)
        if q is None: raise SystemExit(f"IK failed at {p}")
        if qs and np.max(np.abs(np.array(q) - np.array(qs[-1]))) > 0.8:
            raise SystemExit(f"joint jump at {p}: {np.round(np.array(q)-np.array(qs[-1]),2)}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    if corrected_last: r.move_q_corrected(qs[-1], seconds=2.0, iters=2)
    p, qq = r.tcp(); print("TCP", p.round(4), "q", np.round(r.arm_q(), 2))

r.gripper(0.0)
print("-- over start"); go([(X, 0.19, 1.25)], per=4.0)
print("-- descend");    go([(X, 0.19, 1.05), (X, 0.19, Z)], per=3.0)
print("-- push -y");    go([(X, 0.12, Z), (X, 0.04, Z), (X, -0.04, Z), (X, -0.11, Z)], per=2.5, corrected_last=False)
print("-- lift");       go([(X, -0.11, 1.20)], per=3.0, corrected_last=False)
OPENRUA_EOF

# openrua op 83
timeout 900 python3 -u push_yellow.py 2>&1 | grep -v "^\[" 

# openrua op 84
python3 scene.py agentview birdview frontview 2>&1 | grep -v "^\[" | tail -4 && python3 -c "
import cv2, numpy as np
a=cv2.imread('snaps/agentview_rgb.png'); b=cv2.imread('snaps/birdview_rgb.png'); f=cv2.imread('snaps/frontview_rgb.png')
cv2.imwrite('snaps/tri.png', np.hstack([a,b,f]))"

# openrua op 85
python3 - <<'EOF'
import numpy as np, cv2
H=np.array([-0.18,0.26])
for cam in ['agentview','birdview','frontview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); rgb=cv2.imread(f'snaps/{cam}_rgb.png')
    p=xyz.reshape(-1,3); c=rgb.reshape(-1,3); ok=np.isfinite(p).all(1); p=p[ok]; c=c[ok]
    # door top edge candidates: z 1.09-1.115, y<0.25, x<0.1, dark color
    m=(p[:,2]>1.09)&(p[:,2]<1.12)&(p[:,1]<0.25)&(p[:,1]>-0.1)&(p[:,0]<0.12)&(p[:,0]>-0.2)&(c.max(1)<90)
    d=p[m]
    if len(d)>20:
        rel=d[:,:2]-H; ang=np.degrees(np.arctan2(rel[:,1],rel[:,0]))
        print(cam,'door-top pts',len(d),'angle median',np.median(ang).round(1),'p10/p90',np.percentile(ang,[10,90]).round(1),'t range',np.linalg.norm(rel,axis=1).min().round(3),np.linalg.norm(rel,axis=1).max().round(3))
    else: print(cam,'door-top pts',len(d))
    hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV).reshape(-1,3)[ok]
    yel=(hsv[:,0]>15)&(hsv[:,0]<40)&(hsv[:,1]>90)&(hsv[:,2]>120)&(p[:,2]>0.9)&(p[:,2]<1.1)
    q=p[yel]
    if len(q): print('  yellow x',q[:,0].min().round(3),q[:,0].max().round(3),'y',q[:,1].min().round(3),q[:,1].max().round(3))
EOF

# openrua op 86
python3 - <<'EOF'
import numpy as np, cv2
H=np.array([-0.18,0.26])
for cam in ['agentview','frontview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); rgb=cv2.imread(f'snaps/{cam}_rgb.png')
    p=xyz.reshape(-1,3); c=rgb.reshape(-1,3); ok=np.isfinite(p).all(1); p=p[ok]; c=c[ok]
    m=(p[:,2]>1.09)&(p[:,2]<1.12)&(p[:,1]<0.25)&(p[:,1]>-0.1)&(p[:,0]<0.12)&(p[:,0]>-0.2)&(c.max(1)<90)
    d=p[m][:,:2]
    rel=d-H; t=np.linalg.norm(rel,axis=1)
    for lo,hi in [(0.08,0.16),(0.16,0.24),(0.24,0.31)]:
        s=(t>lo)&(t<hi); a=np.degrees(np.arctan2(rel[s,1],rel[s,0]))
        print(cam,f't {lo}-{hi}: n={s.sum()} ang med {np.median(a):.1f}')
    # PCA line
    mu=d.mean(0); u,sv,vt=np.linalg.svd(d-mu); v=vt[0]
    print(cam,'PCA dir angle',np.degrees(np.arctan2(v[1],v[0])).round(1),'mean',mu.round(3),'far end',d[np.argmax(t)].round(3))
EOF

# openrua op 87
mkdir -p "$(dirname /workspace/push_yellow2.py)"
cat > /workspace/push_yellow2.py <<'OPENRUA_EOF'
"""Push the yellow mug toward -y with a 20deg forward-pitched hand (wrist stays
closer to the base and above/behind the door). Dense waypoints + tight jump
check so the joint-space path cannot swing into the microwave."""
import numpy as np
from rob import Robot, approach_quat
r = Robot("pushy2")
TH = np.deg2rad(20)
qn = approach_quat([np.sin(TH), 0, -np.cos(TH)], [1, 0, 0])
X, Z = 0.12, 0.965

def go(pts, per=2.0, corrected_last=True, jump=0.35):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed, timeout=3.0)
        if q is None: raise SystemExit(f"IK failed at {p}")
        prev = np.array(qs[-1]) if qs else np.array(seed)
        if np.max(np.abs(np.array(q) - prev)) > jump:
            raise SystemExit(f"joint jump at {p}: {np.round(np.array(q)-prev,2)}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    if corrected_last: r.move_q_corrected(qs[-1], seconds=2.0, iters=2)
    p, qq = r.tcp(); print("TCP", p.round(4), "q", np.round(r.arm_q(), 2))

def line(a, b, step):
    a, b = np.array(a, float), np.array(b, float)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    return [tuple(a + (b - a) * k / n) for k in range(1, n + 1)]

r.gripper(0.0)
p0 = r.tcp()[0]
print("-- reorient/travel to start (high)")
go(line(p0, (X, 0.19, 1.20), 0.06), per=1.5, jump=1.0)
print("-- descend"); go(line((X, 0.19, 1.20), (X, 0.19, Z), 0.04), per=1.2)
print("-- push -y"); go(line((X, 0.19, Z), (X, -0.12, Z), 0.04), per=1.2, corrected_last=False)
print("-- lift");    go(line((X, -0.12, Z), (X, -0.12, 1.20), 0.06), per=1.2, corrected_last=False)
OPENRUA_EOF

# openrua op 88
timeout 900 python3 -u push_yellow2.py 2>&1 | grep -v "^\[" 

# openrua op 89
python3 scene.py agentview birdview frontview 2>&1 | grep -v "^\[" | tail -3 && python3 - <<'EOF'
import numpy as np, cv2
H=np.array([-0.18,0.26])
a=cv2.imread('snaps/agentview_rgb.png'); b=cv2.imread('snaps/birdview_rgb.png'); f=cv2.imread('snaps/frontview_rgb.png')
cv2.imwrite('snaps/tri.png', np.hstack([a,b,f]))
for cam in ['agentview','birdview','frontview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); rgb=cv2.imread(f'snaps/{cam}_rgb.png')
    p=xyz.reshape(-1,3); c=rgb.reshape(-1,3); ok=np.isfinite(p).all(1); p=p[ok]; c=c[ok]
    m=(p[:,2]>1.09)&(p[:,2]<1.12)&(p[:,1]<0.235)&(p[:,1]>-0.1)&(p[:,0]<0.12)&(p[:,0]>-0.2)&(c.max(1)<90)
    d=p[m][:,:2]; rel=d-H; t=np.linalg.norm(rel,axis=1); s=(t>0.1)&(t<0.24)
    ang=np.degrees(np.arctan2(rel[s,1],rel[s,0]))
    hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV).reshape(-1,3)[ok]
    yel=(hsv[:,0]>15)&(hsv[:,0]<40)&(hsv[:,1]>90)&(hsv[:,2]>120)&(p[:,2]>0.9)&(p[:,2]<1.1)
    q=p[yel]
    print(cam,'door ang med',np.median(ang).round(1) if s.sum() else None,'n',s.sum(),
          '| yellow' , (q[:,0].min().round(3),q[:,0].max().round(3),q[:,1].min().round(3),q[:,1].max().round(3)) if len(q) else None)
EOF

# openrua op 90
timeout 900 python3 -u door.py approach -52 2>&1 | grep -v "^\[" 

# openrua op 91
timeout 900 python3 -u door.py arc -52 0 2>&1 | grep -v "^\[" 

# openrua op 92
timeout 600 python3 -u door.py up 2>&1 | grep -v "^\[" && python3 scene.py agentview birdview frontview sideview 2>&1 | grep -v "^\[" | tail -4 && python3 -c "
import cv2, numpy as np
ims=[cv2.imread(f'snaps/{c}_rgb.png') for c in ['agentview','birdview','frontview','sideview']]
cv2.imwrite('snaps/quad_closed.png', np.vstack([np.hstack(ims[:2]), np.hstack(ims[2:])]))"

# openrua op 93
python3 - <<'EOF'
import numpy as np, cv2
H=np.array([-0.18,0.26])
for cam in ['agentview','frontview','birdview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); rgb=cv2.imread(f'snaps/{cam}_rgb.png')
    p=xyz.reshape(-1,3); c=rgb.reshape(-1,3); ok=np.isfinite(p).all(1); p=p[ok]; c=c[ok]
    # door top edge: dark points at z 1.09-1.12, x in door range, y<0.30
    m=(p[:,2]>1.09)&(p[:,2]<1.12)&(p[:,0]>-0.17)&(p[:,0]<0.05)&(p[:,1]>-0.1)&(p[:,1]<0.30)&(c.max(1)<90)
    d=p[m]
    if len(d)>10:
        print(cam,'door-top pts',len(d),'y range',np.percentile(d[:,1],[2,50,98]).round(3),'x range',d[:,0].min().round(3),d[:,0].max().round(3))
        rel=d[:,:2]-H; t=np.linalg.norm(rel,axis=1); s=t>0.1
        print('   angle med (t>0.1)',np.median(np.degrees(np.arctan2(rel[s,1],rel[s,0]))).round(1))
    # front face: dark points z 0.95-1.05, x in [-0.15,0.04], y in [0.15,0.35]
    f=(p[:,2]>0.95)&(p[:,2]<1.05)&(p[:,0]>-0.15)&(p[:,0]<0.04)&(p[:,1]>0.1)&(p[:,1]<0.35)&(c.max(1)<90)
    q=p[f]
    if len(q)>10:
        print('   front-face pts',len(q),'y percentiles',np.percentile(q[:,1],[2,25,50,75,98]).round(3))
    # anything dark & door-height in the swept region y<0.23, x>-0.17 (would be an open door)
    o=(p[:,2]>0.95)&(p[:,2]<1.12)&(p[:,0]>-0.17)&(p[:,0]<0.06)&(p[:,1]>-0.05)&(p[:,1]<0.22)&(c.max(1)<90)
    print('   dark pts in swept region (open-door evidence):',o.sum())
EOF

# openrua op 94
python3 - <<'EOF'
import numpy as np, cv2
for cam in ['agentview','frontview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); rgb=cv2.imread(f'snaps/{cam}_rgb.png')
    p=xyz.reshape(-1,3); c=rgb.reshape(-1,3); ok=np.isfinite(p).all(1); p=p[ok]; c=c[ok]
    f=(p[:,2]>0.95)&(p[:,2]<1.08)&(p[:,1]>0.1)&(p[:,1]<0.35)&(c.max(1)<90)
    q=p[f]
    for x0,x1 in [(-0.17,-0.13),(-0.13,-0.09),(-0.09,-0.05),(-0.05,-0.01),(-0.01,0.03),(0.03,0.06),(0.08,0.16)]:
        s=(q[:,0]>x0)&(q[:,0]<x1)
        if s.sum(): print(cam,f'x {x0:+.2f}..{x1:+.2f}: n={s.sum():4d} front y min {q[s,1].min():.3f} p5 {np.percentile(q[s,1],5):.3f}')
EOF
