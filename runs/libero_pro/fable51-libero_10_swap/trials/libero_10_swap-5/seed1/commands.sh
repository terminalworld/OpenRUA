#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | grep -iv parameter; echo ---; ros2 node list 2>&1

# openrua op 4
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; ros2 topic echo /birdview/color/camera_info --once 2>&1 | grep -A12 '^k:'

# openrua op 6
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info from a camera, cache world-point cloud as .npz.
Usage: python3 scan.py <camera>
Then query pixels: python3 scan.py <camera> u v [u v ...]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2


def grab(node, topic, mt, timeout=30.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
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
    if len(sys.argv) > 2:
        d = np.load(f"{cam}_cloud.npz")
        P = d["P"]
        args = list(map(int, sys.argv[2:]))
        for u, v in zip(args[::2], args[1::2]):
            print(f"({u},{v}) -> {P[v, u]}")
        return
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    cv2.imwrite(f"{cam}.png", color)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    P = pc @ R.T + T
    np.savez(f"{cam}_cloud.npz", P=P, depth=depth)
    print(f"saved {cam}_cloud.npz  shape={P.shape}  z range {np.nanmin(P[...,2]):.3f}..{np.nanmax(P[...,2]):.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scan.py birdview && timeout 120 python3 scan.py agentview

# openrua op 8
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); P=d['P']
# table height: sample an empty area
print('table sample', P[300,320], P[200,500])
# height map above table
z=P[...,2]
img=cv2.imread('birdview.png')
# find objects above table in region of interest (rows 140-300, cols 180-420)
tab=np.median(z[280:330, 250:450]); print('table z', tab)
mask=(z>tab+0.01)&(z<tab+0.4)
mask[:130,:]=False  # ignore robot area roughly
ys,xs=np.nonzero(mask)
print('n', len(ys))
# connected components
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<30: continue
    sel=lab==i
    pts=P[sel]
    print(i,'px bbox',x,y,w,h,'area',a,'world min',pts.min(0).round(3),'max',pts.max(0).round(3),'zmax',pts[:,2].max().round(3))
"


# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); P=d['P']; z=P[...,2]
tab=0.880
mask=(z>tab+0.01)&(z<tab+0.4); mask[:130,:]=False
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
sel=lab==5
pts=P[sel]
# top surface points
top=pts[pts[:,2]>pts[:,2].max()-0.02]
print('top n',len(top),'centroid',top.mean(0).round(4))
c=top[:,:2].mean(0); X=top[:,:2]-c
w,v=np.linalg.eigh(X.T@X)
print('eig',w,'axes',v.T)
proj=X@v
print('extent along minor',proj[:,0].min().round(3),proj[:,0].max().round(3),'major',proj[:,1].min().round(3),proj[:,1].max().round(3))
ang=np.degrees(np.arctan2(v[1,1],v[0,1])); print('major axis angle deg from world x',ang)
# all book pts
print('all centroid',pts.mean(0).round(4), 'z hist', np.histogram(pts[:,2],bins=8))
"


# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); P=d['P']; z=P[...,2]
img=cv2.imread('agentview.png')
# caddy region px rows 150-275, cols 110-370
sub=P[150:275,110:370]; zz=sub[...,2]
print('z hist',np.histogram(zz[np.isfinite(zz)],bins=12,range=(0.85,1.3)))
# rim points: z between 0.97 and 1.0? check which z is rim
for lo,hi in [(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.06)]:
    m=(zz>lo)&(zz<hi)
    if m.sum()>20:
        pts=sub[m]; print(lo,hi,m.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
"


# openrua op 11
python3 -c "
import cv2
img=cv2.imread('birdview.png'); crop=img[140:240,190:360]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
img=cv2.imread('agentview.png'); crop=img[140:290,100:380]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 12
python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); P=d['P']
sub=P[140:290,100:380].reshape(-1,3)
m=(sub[:,2]>1.04)&(sub[:,2]<1.07)&(sub[:,0]<-0.3)
pts=sub[m]
print(len(pts))
# 2D occupancy grid of rim points, 1cm cells
xs=np.arange(-0.48,-0.30,0.01); ys=np.arange(-0.38,0.08,0.01)
H,_,_=np.histogram2d(pts[:,0],pts[:,1],bins=[xs,ys])
print('   y:', ' '.join(f'{y:+.2f}'[1:4] for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f'{x:+.2f}', ''.join(' # ' if H[i,j]>3 else ' . ' for j in range(len(ys)-1)))
"


# openrua op 13
python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); P=d['P'].reshape(-1,3)
m=(P[:,0]<-0.30)&(P[:,0]>-0.50)&(P[:,1]>-0.40)&(P[:,1]<0.10)&(P[:,2]>0.90)
pts=P[m]
xs=np.arange(-0.50,-0.30,0.01); ys=np.arange(-0.40,0.10,0.01)
Z=np.full((len(xs)-1,len(ys)-1),np.nan)
ix=np.digitize(pts[:,0],xs)-1; iy=np.digitize(pts[:,1],ys)-1
for a,b,z in zip(ix,iy,pts[:,2]):
    if 0<=a<Z.shape[0] and 0<=b<Z.shape[1]:
        Z[a,b]=np.nanmax([Z[a,b],z])
print('max z per cell (cm above table 0.88); rows x, cols y from -0.40')
print('     '+''.join(f'{int(round(y*100)):>4d}' for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f'{x:+.2f}',''.join(f'{int(round((z-0.88)*100)):>4d}' if np.isfinite(z) else '   .' for z in Z[i]))
"


# openrua op 14
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
# birdview rows correspond to world x; find row for x=-0.44 and -0.37
for xt in (-0.45,-0.43,-0.37):
    col=P[:,320,0]; r=int(np.argmin(abs(col-xt)))
    row=P[r]
    print(f'x={xt} row {r}')
    s=''
    for u in range(200,345,1):
        y=row[u,1]; z=row[u,2]
        if u%1==0: s+=f'{y:+.3f}:{(z-0.88)*100:4.1f} '
    print(s)
"


# openrua op 15
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
row=P[200,:,1]; u=int(np.argmin(abs(row+0.15))); print('u',u)
s=''
for v in range(150,225):
    x=P[v,u,0]; z=P[v,u,2]; s+=f'{x:+.3f}:{(z-0.88)*100:4.1f} '
print(s)
print('--- y=-0.30 column')
u=int(np.argmin(abs(row+0.30))); s=''
for v in range(150,225):
    x=P[v,u,0]; z=P[v,u,2]; s+=f'{x:+.3f}:{(z-0.88)*100:4.1f} '
print(s)
"


# openrua op 16
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']
sub=P[244:284,312:340]
for r in range(sub.shape[0]):
    print(f'{sub[r,0,0]:+.3f} '+''.join(f'{int(round((z-0.88)*100)):3d}' if z>0.9 else '  .' for z in sub[r,:,2]))
print('y:', ' '.join(f'{y:+.2f}' for y in sub[0,::4,1]))
"


# openrua op 17
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); P=d['P']
sub=P[240:290,305:345].reshape(-1,3)
m=(sub[:,2]>0.95)&(sub[:,2]<1.2)
pts=sub[m][:,:2].astype(np.float32)
rect=cv2.minAreaRect(pts)
print('center',rect[0],'size',rect[1],'angle',rect[2])
# convex hull based extents along principal axis
c=pts.mean(0); X=pts-c; w,v=np.linalg.eigh(X.T@X); proj=X@v
print('centroid',c,'minor extent',proj[:,0].min(),proj[:,0].max(),'major',proj[:,1].min(),proj[:,1].max())
print('major axis dir',v[:,1], 'angle deg',np.degrees(np.arctan2(v[1,1],v[0,1])))
"


# openrua op 18
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control library for this Panda: joint state, FK, IK, trajectory, gripper.
World<->base: base = panda_link0 at world (-0.75, 0, 0.912), no rotation.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])
M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = TRAJ["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def topdown_quat(theta):
    """Hand pointing down (-Z world), rotated theta about world Z. Returns x,y,z,w."""
    return (math.cos(theta / 2), math.sin(theta / 2), 0.0, 0.0)


class Rob:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 20
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def fk_world(self, q=None):
        """Hand pose in world: (pos[3], quat xyzw)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, pos, quat, seed=None, tries=5):
        """IK for hand pose in world frame. Returns joint list (manifest order)."""
        pos = np.asarray(pos, float) - BASE_IN_WORLD
        if seed is None:
            seed = self.arm_q()
        best = None
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            s = list(seed) if k == 0 else list(np.array(seed) + np.random.uniform(-0.15, 0.15, len(seed)))
            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is None or res.error_code.val != 1:
                continue
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            q = [sol[n] for n in ARM]
            dist = float(np.abs(np.array(q) - np.array(seed)).max())
            if best is None or dist < best[1]:
                best = (q, dist)
            if dist < 0.6:
                break
        if best is None:
            raise RuntimeError(f"IK failed for world pos {pos + BASE_IN_WORLD}")
        return best[0]

    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via)
            for i, v in enumerate(via):
                t = seconds * (i + 1) / (n + 1)
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        cur = np.array(self.arm_q())
        err = float(np.abs(cur - np.array(q)).max())
        print(f"  traj error_code={code} max joint err={err:.4f}", flush=True)
        return code, err

    def move_world(self, pos, quat, seconds=3.0):
        q = self.ik_world(pos, quat)
        code, err = self.move_q(q, seconds)
        p, o = self.fk_world()
        print(f"  hand now at world {p.round(4)} quat {np.round(o, 3)}", flush=True)
        return p

    def move_tcp(self, tcp_pos, theta, seconds=3.0):
        """Move so the fingertip centre (TCP) is at tcp_pos (world), hand pointing down, yaw theta."""
        hand = np.asarray(tcp_pos, float) + np.array([0, 0, TCP])
        return self.move_world(hand, topdown_quat(theta), seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f


if __name__ == "__main__":
    r = Rob()
    print("joints", r.joints())
    p, o = r.fk_world()
    print("hand world", p, "quat", o)
OPENRUA_EOF

# openrua op 19
timeout 120 python3 -u rob.py 2>&1 | tail -5

# openrua op 20
timeout 60 python3 -c "
import rclpy, numpy as np
from rob import *
r=Rob()
from moveit_msgs.srv import GetPositionFK
for fid in ['', 'panda_link0', 'world']:
    req=GetPositionFK.Request(); req.header.frame_id=fid
    req.fk_link_names=['panda_hand','panda_link0']
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=r.arm_q()
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    res=fut.result()
    for ps in res.pose_stamped:
        p=ps.pose.position; print(repr(fid), ps.header.frame_id, [round(v,4) for v in (p.x,p.y,p.z)])
" 2>&1 | grep -v INFO; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 Translation; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 -i 'translation\|error\|unconnected' | head; ros2 topic echo /tf_static --once 2>&1 | grep -B2 -A12 'child_frame_id: panda_link0' | head -30

# openrua op 21
timeout 120 python3 -c "
import rclpy, numpy as np
from rob import *
r=Rob()
q0=np.array(r.arm_q()); print('current', q0.round(3))
quat=(1.0,0.0,-0.0284,0.0)
for name,pos in [('world-coords',[-0.293,0,1.2696]),('base-coords',[0.457,0,0.3576])]:
    try:
        # bypass BASE_IN_WORLD shift: pass pos + BASE so ik_world subtracts it back
        q=r.ik_world(np.array(pos)+BASE_IN_WORLD, quat, tries=1)
        print(name, np.array(q).round(3), 'maxdiff', np.abs(np.array(q)-q0).max().round(3))
    except Exception as e: print(name,'FAIL',e)
" 2>&1 | grep -v INFO

# openrua op 22
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos,''','''        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK answers in world
        return pos,''')
s=s.replace('''        pos = np.asarray(pos, float) - BASE_IN_WORLD
        if seed is None:''','''        pos = np.asarray(pos, float)  # IK (empty frame_id) is interpreted in world here
        if seed is None:''')
s=s.replace('''            raise RuntimeError(f"IK failed for world pos {pos + BASE_IN_WORLD}")''','''            raise RuntimeError(f"IK failed for world pos {pos}")''')
open('rob.py','w').write(s)
EOF
grep -n "BASE_IN_WORLD" rob.py

# openrua op 23
mkdir -p "$(dirname /workspace/step1_hover.py)"
cat > /workspace/step1_hover.py <<'OPENRUA_EOF'
import numpy as np
from rob import *

BOOK = np.array([-0.122, 0.0195])
BOOK_TOP = 1.07
THETA = np.radians(158.28 - 180)  # fingers close across the book's 3 cm thickness

r = Rob()
print("open gripper", flush=True)
r.gripper(GRIP["open_m"])
print("hover above book", flush=True)
r.move_tcp([BOOK[0], BOOK[1], BOOK_TOP + 0.10], THETA, seconds=4.0)
print("q", np.round(r.arm_q(), 3), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 24
timeout 900 python3 -u step1_hover.py 2>&1 | grep -v INFO

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hover_eih.png && timeout 60 python3 tools/perception/cam_snap.py birdview hover_bird.png && python3 -c "
import cv2; img=cv2.imread('hover_bird.png'); cv2.imwrite('hover_bird_crop.png', cv2.resize(img[180:320,240:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"

# openrua op 26
timeout 300 python3 -c "
import rclpy, numpy as np
from rob import *
r=Rob()
p,o=r.fk_world(); print('now',p.round(4),np.round(o,3))
for th in [np.radians(-21.7), np.radians(21.7), np.radians(90), np.radians(-90), 0.0]:
    quat=topdown_quat(th)
    try:
        q=r.ik_world(p, quat, tries=1)
        p2,o2=r.fk_world(q)
        print(f'theta={np.degrees(th):6.1f} req quat {np.round(quat,3)} -> sol q {np.round(q,2)} fk quat {np.round(o2,3)} pos err {np.abs(p2-p).max():.4f}')
    except Exception as e: print('theta',np.degrees(th),'FAIL',e)
" 2>&1 | grep -v INFO

# openrua op 27
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''def topdown_quat(theta):''','''def qmul(a, b):
    """Hamilton product, quats as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# The IK group's tip is panda_link8; panda_hand = link8 * Rz(-45deg). So a
# desired panda_hand orientation must be sent to IK as hand * Rz(+45deg).
Q_HAND_TO_LINK8 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def topdown_quat(theta):''')
s=s.replace('''            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)''','''            q8 = qmul(quat, Q_HAND_TO_LINK8)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)''')
open('rob.py','w').write(s)
EOF
timeout 300 python3 -c "
import rclpy, numpy as np
from rob import *
r=Rob()
p,o=r.fk_world(); print('now',p.round(4),np.round(o,3))
for th in [np.radians(-21.7), np.radians(90), np.radians(-90), 0.0]:
    quat=topdown_quat(th)
    q=r.ik_world(p, quat, tries=1)
    p2,o2=r.fk_world(q)
    print(f'theta={np.degrees(th):6.1f} req quat {np.round(quat,3)} -> sol q {np.round(q,2)} fk quat {np.round(o2,3)} pos err {np.abs(p2-p).max():.4f}')
" 2>&1 | grep -v INFO

# openrua op 28
timeout 600 python3 -u step1_hover.py 2>&1 | grep -v INFO && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hover_eih.png

# openrua op 29
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()''','''    def move_q(self, q, seconds=3.0, via=None, retries=2):
        code, err = self._move_q(q, seconds, via)
        while err > 0.01 and retries > 0:  # controller lag: resend converges
            print("  resending (lag)", flush=True)
            code, err = self._move_q(q, max(2.0, seconds / 2), None)
            retries -= 1
        return code, err

    def _move_q(self, q, seconds, via):
        goal = FollowJointTrajectory.Goal()''')
open('rob.py','w').write(s)
EOF
timeout 600 python3 -u step1_hover.py 2>&1 | grep -v INFO && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hover_eih.png

# openrua op 30
mkdir -p "$(dirname /workspace/step2_grasp.py)"
cat > /workspace/step2_grasp.py <<'OPENRUA_EOF'
import numpy as np
from rob import *

BOOK = np.array([-0.122, 0.0195])
BOOK_TOP = 1.07
THETA = np.radians(158.28 - 180)

r = Rob()
print("descend to TCP 3 cm below book top", flush=True)
r.move_tcp([BOOK[0], BOOK[1], BOOK_TOP - 0.03], THETA, seconds=3.0)
print("fingers before close", r.fingers(), flush=True)
print("close", flush=True)
f = r.gripper(GRIP["closed_m"])
print("fingers after close", f, flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 31
timeout 800 python3 -u step2_grasp.py 2>&1 | grep -v INFO && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand grasp_eih.png && timeout 60 python3 tools/perception/cam_snap.py agentview grasp_agent.png

# openrua op 32
mkdir -p "$(dirname /workspace/step3_lift.py)"
cat > /workspace/step3_lift.py <<'OPENRUA_EOF'
import numpy as np
from rob import *

BOOK = np.array([-0.122, 0.0195])
THETA = np.radians(158.28 - 180)

r = Rob()
print("lift", flush=True)
r.move_tcp([BOOK[0], BOOK[1], 1.30], THETA, seconds=3.0)
print("fingers", r.fingers(), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 33
timeout 800 python3 -u step3_lift.py 2>&1 | grep -v INFO && timeout 60 python3 tools/perception/cam_snap.py agentview lift_agent.png && timeout 60 python3 tools/perception/cam_snap.py sideview lift_side.png

# openrua op 34
timeout 120 python3 scan.py agentview >/dev/null && python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); P=d['P'].reshape(-1,3)
m=(P[:,0]>-0.20)&(P[:,0]<-0.04)&(P[:,1]>-0.03)&(P[:,1]<0.07)&(P[:,2]>0.9)
pts=P[m]; print('n',len(pts))
print('z hist', np.histogram(pts[:,2],bins=np.arange(0.9,1.12,0.01)))
top=pts[pts[:,2]>1.05]; print('top pts centroid',top.mean(0).round(4),'x range',top[:,0].min().round(3),top[:,0].max().round(3),'y range',top[:,1].min().round(3),top[:,1].max().round(3))
"


# openrua op 35
timeout 120 python3 scan.py birdview >/dev/null && python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    d=np.load(f'{cam}_cloud.npz'); P=d['P'].reshape(-1,3)
    m=(P[:,0]>-0.20)&(P[:,0]<-0.04)&(P[:,1]>-0.03)&(P[:,1]<0.075)&(P[:,2]>0.9)&(P[:,2]<1.25)
    pts=P[m]
    xs=np.arange(-0.20,-0.04,0.01); ys=np.arange(-0.03,0.08,0.01)
    Z=np.full((len(xs)-1,len(ys)-1),np.nan)
    ix=np.digitize(pts[:,0],xs)-1; iy=np.digitize(pts[:,1],ys)-1
    for a,b,z in zip(ix,iy,pts[:,2]):
        if 0<=a<Z.shape[0] and 0<=b<Z.shape[1]: Z[a,b]=np.nanmax([Z[a,b],z])
    print(cam,'max z (cm above table); cols y from -0.03 step 1cm')
    for i,x in enumerate(xs[:-1]):
        print(f'{x:+.2f}',''.join(f'{int(round((z-0.88)*100)):>4d}' if np.isfinite(z) else '   .' for z in Z[i]))
"


# openrua op 36
timeout 120 python3 scan.py sideview >/dev/null; timeout 120 python3 scan.py frontview >/dev/null; python3 -c "
import numpy as np
for cam in ['sideview','frontview','agentview']:
    d=np.load(f'{cam}_cloud.npz'); P=d['P'].reshape(-1,3)
    for name,(x0,x1,y0,y1) in {'book':(-0.20,-0.04,-0.03,0.075),'mug':(-0.27,-0.16,0.08,0.22),'caddy':(-0.48,-0.32,-0.37,0.07)}.items():
        m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.9)&(P[:,2]<1.25)
        pts=P[m]
        if len(pts): print(cam,name,'n',len(pts),'max z',pts[:,2].max().round(3),'99pct',np.percentile(pts[:,2],99).round(3))
        else: print(cam,name,'none')
"


# openrua op 37
mkdir -p "$(dirname /workspace/step2_grasp.py)"
cat > /workspace/step2_grasp.py <<'OPENRUA_EOF'
import numpy as np
from rob import *

BOOK = np.array([-0.122, 0.0195])
BOOK_TOP = 1.023  # consensus of side/front/agent cameras
THETA = np.radians(158.28 - 180)

r = Rob()
print("open", flush=True)
r.gripper(GRIP["open_m"])
print("pre-grasp hover", flush=True)
r.move_tcp([BOOK[0], BOOK[1], BOOK_TOP + 0.06], THETA, seconds=3.0)
print("descend to TCP 3 cm below book top", flush=True)
r.move_tcp([BOOK[0], BOOK[1], BOOK_TOP - 0.03], THETA, seconds=2.5)
print("close", flush=True)
f = r.gripper(GRIP["closed_m"])
print("lift", flush=True)
r.move_tcp([BOOK[0], BOOK[1], 1.28], THETA, seconds=3.0)
print("fingers after lift", r.fingers(), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 38
timeout 1500 python3 -u step2_grasp.py 2>&1 | grep -v INFO && timeout 60 python3 tools/perception/cam_snap.py agentview lift_agent.png && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand lift_eih.png

# openrua op 39
mkdir -p "$(dirname /workspace/step4_transport.py)"
cat > /workspace/step4_transport.py <<'OPENRUA_EOF'
import numpy as np
from rob import *

TARGET = np.array([-0.432, -0.153])
THETA = np.radians(-90)  # fingers close along world x -> book long axis along y

r = Rob()
print("transport high", flush=True)
r.move_tcp([TARGET[0], TARGET[1], 1.28], THETA, seconds=5.0)
print("fingers", r.fingers(), flush=True)
print("lower to TCP 1.15", flush=True)
r.move_tcp([TARGET[0], TARGET[1], 1.15], THETA, seconds=3.0)
print("fingers", r.fingers(), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 40
timeout 1500 python3 -u step4_transport.py 2>&1 | grep -v INFO && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand tr_eih.png && timeout 60 python3 tools/perception/cam_snap.py agentview tr_agent.png

# openrua op 41
timeout 60 python3 scan.py robot0_eye_in_hand 2>&1 | tail -3; timeout 10 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A1 -i 'translation\|error\|unconnected' | head -4

# openrua op 42
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['P'].reshape(-1,3)
m=(P[:,0]>-0.50)&(P[:,0]<-0.30)&(P[:,1]>-0.26)&(P[:,1]<-0.04)&(P[:,2]>0.89)
pts=P[m]
xs=np.arange(-0.50,-0.30,0.005); ys=np.arange(-0.26,-0.04,0.005)
Z=np.full((len(xs)-1,len(ys)-1),np.nan)
ix=np.digitize(pts[:,0],xs)-1; iy=np.digitize(pts[:,1],ys)-1
for a,b,z in zip(ix,iy,pts[:,2]):
    if 0<=a<Z.shape[0] and 0<=b<Z.shape[1]: Z[a,b]=np.nanmax([Z[a,b],z])
print('max z cm above table; cols y from -0.26 step 5mm; rows x step 5mm')
print('      '+''.join(f'{int(round(y*1000)):>4d}' for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f'{x:+.3f}',''.join(f'{int(round((z-0.88)*100)):>4d}' if np.isfinite(z) else '   .' for z in Z[i]))
"


# openrua op 43
timeout 120 python3 scan.py frontview >/dev/null; timeout 120 python3 scan.py agentview >/dev/null; timeout 120 python3 scan.py sideview >/dev/null; python3 -c "
import numpy as np
for cam in ['frontview','agentview','sideview']:
    d=np.load(f'{cam}_cloud.npz'); P=d['P'].reshape(-1,3)
    m=(P[:,0]>-0.50)&(P[:,0]<-0.36)&(P[:,1]>-0.30)&(P[:,1]<0.0)&(P[:,2]>1.075)&(P[:,2]<1.19)
    pts=P[m]
    print(cam,'n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
    # y extent per z slice
    for z0 in np.arange(1.08,1.19,0.02):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
        if len(s): print(f'   z {z0:.2f}: y {s[:,1].min():.3f}..{s[:,1].max():.3f}  x {s[:,0].min():.3f}..{s[:,0].max():.3f} n {len(s)}')
"


# openrua op 44
mkdir -p "$(dirname /workspace/step5_lower.py)"
cat > /workspace/step5_lower.py <<'OPENRUA_EOF'
import numpy as np
from rob import *

# book centre sits 1.8 mm toward -y of the TCP; want book centre y = -0.1525
TCP_XY = np.array([-0.432, -0.1507])
THETA = np.radians(-90)

r = Rob()
print("lower to TCP 1.10", flush=True)
r.move_tcp([TCP_XY[0], TCP_XY[1], 1.10], THETA, seconds=2.5)
print("lower to TCP 1.065 (fingertips 7 mm above back wall top)", flush=True)
r.move_tcp([TCP_XY[0], TCP_XY[1], 1.065], THETA, seconds=2.0)
print("fingers", r.fingers(), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 45
timeout 1200 python3 -u step5_lower.py 2>&1 | grep -v INFO && timeout 120 python3 scan.py frontview >/dev/null && timeout 120 python3 scan.py agentview >/dev/null && python3 -c "
import numpy as np
for cam in ['frontview','agentview']:
    d=np.load(f'{cam}_cloud.npz'); P=d['P'].reshape(-1,3)
    m=(P[:,0]>-0.50)&(P[:,0]<-0.36)&(P[:,1]>-0.30)&(P[:,1]<0.0)&(P[:,2]>1.062)&(P[:,2]<1.10)
    pts=P[m]
    print(cam,'n',len(pts),'x',pts[:,0].min().round(4),pts[:,0].max().round(4),'y',pts[:,1].min().round(4),pts[:,1].max().round(4),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
"

# openrua op 46
python3 -c "
import cv2; img=cv2.imread('agentview.png'); cv2.imwrite('pre_release_crop.png', cv2.resize(img[120:300,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 47
mkdir -p "$(dirname /workspace/step6_release.py)"
cat > /workspace/step6_release.py <<'OPENRUA_EOF'
import numpy as np
from rob import *

TCP_XY = np.array([-0.432, -0.1507])
THETA = np.radians(-90)

r = Rob()
print("open", flush=True)
r.gripper(GRIP["open_m"])
print("retreat up", flush=True)
r.move_tcp([TCP_XY[0], TCP_XY[1], 1.25], THETA, seconds=3.0)
print("move aside so cameras see the caddy", flush=True)
r.move_tcp([-0.25, 0.10, 1.30], np.radians(0), seconds=4.0)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 48
timeout 1500 python3 -u step6_release.py 2>&1 | grep -v INFO && for c in agentview birdview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c final_$c.png; done

# openrua op 49
python3 -c "
import cv2; img=cv2.imread('final_birdview.png'); cv2.imwrite('final_bird_crop.png', cv2.resize(img[140:240,190:360],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 50
timeout 120 python3 scan.py birdview >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P'].reshape(-1,3)
# inside the middle-back compartment: anything above its floor (0.901) and below the rim
m=(P[:,0]>-0.463)&(P[:,0]<-0.405)&(P[:,1]>-0.2135)&(P[:,1]<-0.092)&(P[:,2]>0.93)&(P[:,2]<1.10)
pts=P[m]
print('points inside back compartment above floor:',len(pts))
print('x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z top',pts[:,2].max().round(3))
# and confirm nothing book-like remains at the old spot
m2=(P[:,0]>-0.20)&(P[:,0]<-0.04)&(P[:,1]>-0.03)&(P[:,1]<0.07)&(P[:,2]>0.9)&(P[:,2]<1.2)
print('points at old book location above table:',m2.sum())
"

# openrua op 51
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Non-obvious facts about the /workspace Franka Panda ROS2 sim (IK/FK frames, link8 yaw offset, birdview depth bias) learned 2026-09-12
metadata:
  type: project
---

Verified 2026-09-12 on the /workspace Panda workstation (MoveIt + paused-clock sim):

- `/compute_ik` and `/compute_fk` with EMPTY frame_id both work in the `world`
  frame (panda_link0 sits at world (-0.75, 0, 0.912)). The generic doc note
  "IK poses are in the arm-base frame" is wrong for this machine.
- The IK group's tip is `panda_link8`, not `panda_hand`: a requested hand
  orientation comes out 45° off about the approach axis. Multiply the desired
  panda_hand quaternion by Rz(+45°) (local) before sending to IK. `rob.py` in
  /workspace does this (`Q_HAND_TO_LINK8`).
- The birdview depth camera overestimated small-object heights by ~5 cm (book,
  mug) while getting the caddy right; agentview/frontview/sideview agreed with
  each other. Cross-check heights across cameras before choosing a grasp depth.
- Finger closure on a 3 cm book reads ~0.0147 per finger; closing on air reads
  ~0.004 then ~0.0005. Gripper max_effort is ignored (open/closed only).
- Trajectory result -5 (tolerance violated) is controller lag; resending the
  same goal converged every time.

**Why:** these cost several failed attempts (a grasp on air, a 45° wrong yaw).
**How to apply:** reuse /workspace/rob.py and /workspace/scan.py; trust
multi-camera consensus over any single depth image. See [[libero-caddy-layout]].
OPENRUA_EOF

# openrua op 52
cd /claude-config/projects/-workspace/memory && ls && (test -f MEMORY.md && cat MEMORY.md || echo "(no MEMORY.md yet)")

# openrua op 53
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda workstation quirks](panda-workstation-quirks.md) — IK/FK use world frame, IK tip is link8 (45° yaw offset), birdview depth overestimates small-object height
OPENRUA_EOF
