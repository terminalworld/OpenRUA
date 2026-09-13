#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -v parameter && echo ---- && ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ls -la snaps

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; echo ----; timeout 20 ros2 topic echo /tf_static --once | head -80

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage, "/tf_static", cb, qos)
n.create_subscription(TFMessage, "/tf", cb, 100)
t0 = time.time()
while time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.2)
for (p, c), tr in sorted(seen.items()):
    print(f"{p} -> {c}: t=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f}) q=({tr.rotation.x:.4f},{tr.rotation.y:.4f},{tr.rotation.z:.4f},{tr.rotation.w:.4f})")
EOF
python3 tfdump.py

# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/birdview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'

# openrua op 8
cat > analyze_bird.py <<'EOF'
import numpy as np, cv2
d = np.load("snaps/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
camz = 3.0; camx=-0.2; camy=0.0
# birdview optical: q=(0.7071,0.7071,0,0): rotation about axis (1,1,0)/sqrt2 by 180deg.
# R = [[0,1,0],[1,0,0],[0,0,-1]] -> world = camx + v_cam_y... let's compute properly
def R_from_q(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
R = R_from_q(0.7071,0.7071,0,0)
print(R.round(3))
H,W = d.shape
vs,us = np.mgrid[0:H,0:W]
Z = d
X = (us-cx)*Z/fx; Y=(vs-cy)*Z/fy
P = np.stack([X,Y,Z],-1) @ R.T + np.array([camx,camy,camz])
wz = P[...,2]
print("z stats", np.nanmin(wz), np.nanmax(wz))
# table height: mode
hist,edges = np.histogram(wz[np.isfinite(wz)], bins=200)
i = hist.argmax(); table = (edges[i]+edges[i+1])/2
print("table z ~", table)
mask = (wz > table+0.01) & (wz < table+0.5)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 30: continue
    m = lab==i
    pts = P[m]
    print(f"comp {i}: px area={stats[i,4]} bbox={stats[i,:4]} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
np.save("snaps/bird_wz.npy", wz)
EOF
python3 analyze_bird.py

# openrua op 9
cat > analyze_bird2.py <<'EOF'
import numpy as np, cv2
wz = np.load("snaps/bird_wz.npy")
d = np.load("snaps/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
H,W = d.shape
vs,us = np.mgrid[0:H,0:W]
wx = -0.2 + (vs-cy)*d/fy
wy = 0.0 + (us-cx)*d/fx
region = wz[160:480, 160:480]
hist,edges = np.histogram(region[np.isfinite(region)], bins=400, range=(0,1.7))
i = hist.argmax(); table = (edges[i]+edges[i+1])/2
print("table z ~", table)
# stats of heights above table
mask = ((wz > table+0.008) & (wz < table+0.5)).astype(np.uint8)
mask[:150,:]=0
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 20: continue
    m = lab==i
    print(f"comp {i}: area={stats[i,4]} bbox(u,v,w,h)={stats[i,:4]} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f})")
# table extents
tm = np.abs(wz - table) < 0.005
print("table x", wx[tm].min(), wx[tm].max(), "y", wy[tm].min(), wy[tm].max())
EOF
python3 analyze_bird2.py

# openrua op 10
cat > analyze_pot.py <<'EOF'
import numpy as np
wz = np.load("snaps/bird_wz.npy"); d = np.load("snaps/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
H,W = d.shape; vs,us = np.mgrid[0:H,0:W]
wx = -0.2 + (vs-cy)*d/fy; wy = (us-cx)*d/fx
table=0.899
for name,(u0,v0,w,h) in {"potA":(236,226,43,24),"potB":(361,265,43,24),"knob":(320,292,25,26),"stove":(307,320,53,54)}.items():
    sub = wz[v0:v0+h, u0:u0+w]-table
    print(name); 
    print(np.round(sub*100).astype(int))
    for thr in [0.02,0.05,0.08,0.10,0.12,0.14]:
        m = np.zeros_like(wz,bool); m[v0:v0+h,u0:u0+w] = sub>thr
        if m.sum()==0: continue
        print(f"  >{thr}: n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] c=({wx[m].mean():.3f},{wy[m].mean():.3f})")
EOF
python3 analyze_pot.py

# openrua op 11
cat > analyze_pot2.py <<'EOF'
import numpy as np
np.set_printoptions(linewidth=250)
wz = np.load("snaps/bird_wz.npy"); table=0.899
for name,(u0,v0,w,h) in {"potA":(236,226,43,24),"potB":(361,265,43,24)}.items():
    sub = wz[v0:v0+h, u0:u0+w]-table
    print(name); print(np.round(sub*100).astype(int))
EOF
python3 analyze_pot2.py

# openrua op 12
cat > analyze_pot2.py <<'EOF'
import numpy as np, sys
np.set_printoptions(linewidth=250, threshold=100000)
wz = np.load("snaps/bird_wz.npy"); table=0.899
for name,(u0,v0,w,h) in {"potA":(236,226,43,24),"potB":(361,265,43,24)}.items():
    sub = wz[v0:v0+h, u0:u0+w]-table
    print(name)
    for row in np.round(sub*100).astype(int): print(" ".join(f"{v:2d}" for v in row))
EOF
python3 analyze_pot2.py

# openrua op 13
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw snaps/sideview_depth.png && timeout 20 ros2 topic echo /sideview/color/camera_info --once | grep -A3 '^k:' && timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw snaps/frontview_depth.png && timeout 20 ros2 topic echo /frontview/color/camera_info --once | grep -A3 '^k:'

# openrua op 14
cat > profile.py <<'EOF'
import numpy as np
fx=fy=579.4112549695428; cx=320; cy=240
def R_from_q(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(npy, t, q):
    d = np.load(npy); H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
    P = np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1) @ R_from_q(*q).T + np.array(t)
    return P.reshape(-1,3)
pts = np.concatenate([
  cloud("snaps/sideview_depth.npy", (-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069)),
  cloud("snaps/frontview_depth.npy", (1.0,0.0,1.48),(0.5608,0.5608,-0.4306,-0.4306)),
  cloud("snaps/birdview_depth.npy", (-0.2,0.0,3.0),(0.7071,0.7071,0,0))])
pts = pts[np.isfinite(pts).all(1)]
table=0.899
for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.15,-0.30,-0.13),"potB":(-0.13,-0.02,0.13,0.29)}.items():
    m = (pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>table+0.005)
    p = pts[m]
    print(name, len(p))
    for z0 in np.arange(0.0,0.17,0.01):
        s = p[(p[:,2]-table>=z0)&(p[:,2]-table<z0+0.01)]
        if len(s)<3: continue
        print(f"  h {z0*100:4.0f}-{z0*100+1:.0f}cm n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
EOF
python3 profile.py

# openrua op 15
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth.png && timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A6 '^k:'
cat > eih.py <<'EOF'
import numpy as np
d=np.load("snaps/eih_depth.npy"); H,W=d.shape
import re,subprocess
fx=fy=float(open('/dev/stdin').read()) if False else None
EOF


# openrua op 16
cat > eih.py <<'EOF'
import numpy as np
np.set_printoptions(linewidth=300, threshold=100000)
fx=fy=312.77408948188935; cx=320; cy=240
def R_from_q(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
d=np.load("snaps/eih_depth.npy"); H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
P = np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1) @ R_from_q(0.7070,-0.7066,-0.0201,0.0201).T + np.array([-0.153,0,1.2673])
table=0.899
h = P[...,2]-table
# pot B region: world x[-0.12,-0.03], y[0.13,0.29]
m = (P[...,0]>-0.13)&(P[...,0]<-0.02)&(P[...,1]>0.12)&(P[...,1]<0.30)&(h>0.005)
print("potB px", m.sum())
for z0 in np.arange(0.0,0.17,0.01):
    s = P[m & (h>=z0)&(h<z0+0.01)]
    if len(s)<3: continue
    print(f"  h {z0*100:4.0f}-{z0*100+1:.0f}cm n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
# print height map of pot B region in eih pixels
vv,uu = np.where(m)
v0,v1,u0,u1 = vv.min(),vv.max(),uu.min(),uu.max()
print(v0,v1,u0,u1)
sub = np.round(h[v0:v1+1,u0:u1+1]*100).astype(int)
sub[~m[v0:v1+1,u0:u1+1]] = 0
for row in sub[::2]: print("".join(f"{v:3d}" for v in row[::2]))
EOF
python3 eih.py

# openrua op 17
cat > profile2.py <<'EOF'
import numpy as np
fx=fy=579.4112549695428; cx=320; cy=240
def R_from_q(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(npy, t, q):
    d = np.load(npy); H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
    P = np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1) @ R_from_q(*q).T + np.array(t)
    return P.reshape(-1,3)
cams = {"side":cloud("snaps/sideview_depth.npy", (-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069)),
        "front":cloud("snaps/frontview_depth.npy", (1.0,0.0,1.48),(0.5608,0.5608,-0.4306,-0.4306))}
table=0.899
for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.15,-0.30,-0.13),"potB":(-0.13,-0.02,0.13,0.29)}.items():
  for cam,pts in cams.items():
    m = (pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>table+0.005)
    p = pts[m]
    print(name, cam, len(p))
    for z0 in np.arange(0.0,0.16,0.005):
        s = p[(p[:,2]-table>=z0)&(p[:,2]-table<z0+0.005)]
        if len(s)<3: continue
        print(f"  h {z0*100:5.1f}cm n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] w={100*(s[:,0].max()-s[:,0].min()):.1f}  y[{s[:,1].min():.3f},{s[:,1].max():.3f}] w={100*(s[:,1].max()-s[:,1].min()):.1f}")
EOF
python3 profile2.py

# openrua op 18
cat > rob.py <<'EOF'
"""Shared robot helper: persistent node, IK/FK, trajectory, gripper, servo, joint read."""
import math, time, sys
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])   # world -> panda_link0
TCP = M["hand"]["tcp_offset_m"]
TABLE = 0.899

def R_from_q(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def q_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1)*2; w = s/4; x = (R[2,1]-R[1,2])/s; y = (R[0,2]-R[2,0])/s; z = (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2; w = (R[2,1]-R[1,2])/s; x = s/4; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2; w = (R[0,2]-R[2,0])/s; x = (R[0,1]+R[1,0])/s; y = s/4; z = (R[1,2]+R[2,1])/s
    else:
        s = math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2; w = (R[1,0]-R[0,1])/s; x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = s/4
    return np.array([x, y, z, w])

def grasp_R(yaw=0.0, tilt=0.0):
    """Hand rotation: approach down, fingers close along world x (rotated by yaw about z),
    approach tilted by `tilt` rad about the closing axis."""
    hz = np.array([0, 0, -1.0]); hy = np.array([1.0, 0, 0])
    # tilt about hy
    c, s = math.cos(tilt), math.sin(tilt)
    Rt = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])  # rotate about y (=hy) -> tilts hz toward +x/-x
    hz = Rt @ hz
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    hz = Rz @ hz; hy = Rz @ hy
    hx = np.cross(hy, hz)
    return np.column_stack([hx, hy, hz])

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()) % 100000))
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.wait_js()

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        while not self.js: self.spin(0.2)
        return dict(self.js)

    def arm_q(self):
        js = self.wait_js(); return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.wait_js(); return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def fk_hand(self, q=None):
        """world pose of panda_hand (pos, quat) and TCP position."""
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = q if q is not None else self.arm_q()
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1: return None
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        tcp = pos + R_from_q(*q)[:, 2] * TCP
        return pos, q, tcp

    def ik_tcp(self, tcp_world, R, seed=None, tries=3):
        """IK for a TCP position (world) with rotation R (3x3). returns joint list or None."""
        pos = np.asarray(tcp_world) - R[:, 2] * TCP - BASE
        q = q_from_R(R)
        seed = seed if seed is not None else self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = 1
            req.ik_request.avoid_collisions = False
            fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        return None

    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i+1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v]); pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1)*1e9)); pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1)*1e9)); pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.arm_q()) - np.array(q)))
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, v, n, frame=None):
        """publish n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped(); msg.header.frame_id = frame or TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg); self.spin(0.05)
EOF
cat > test_ik.py <<'EOF'
from rob import *
r = Robot()
print("current q", np.round(r.arm_q(),3))
print("fk", r.fk_hand())
for name, tcp, tilt in [("potA handle", (-0.204,-0.265,1.03),0), ("potB handle",(-0.072,0.162,1.03),0),
                        ("stove near x=.10", (0.10,0.047,1.06),0), ("stove x=.13",(0.13,0.047,1.06),0),
                        ("stove x=.13 tilt30",(0.13,0.047,1.06),0.5), ("stove x=.18 tilt30",(0.18,0.047,1.06),0.5),
                        ("stove x=.18 tilt45",(0.18,0.047,1.06),0.8), ("stove x=.23 tilt45",(0.23,0.047,1.06),0.8)]:
    q = r.ik_tcp(tcp, grasp_R(0, tilt))
    print(name, tilt, None if q is None else np.round(q,3))
EOF
timeout 300 python3 test_ik.py

# openrua op 19
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])   # FK\/IK poses are already in world (verified via FK vs eye-in-hand TF)/' rob.py && grep -n "^BASE" rob.py && timeout 300 python3 test_ik.py

# openrua op 20
cat > test_ik2.py <<'EOF'
from rob import *
r = Robot()
for name, tcp, tilt in [("x=.15 v", (0.15,0.047,1.06),0), ("x=.16 v",(0.16,0.047,1.06),0), ("x=.18 v",(0.18,0.047,1.06),0),
                        ("x=.15 t-20",(0.15,0.047,1.06),-0.35), ("x=.18 t-20",(0.18,0.047,1.06),-0.35), ("x=.18 t-35",(0.18,0.047,1.06),-0.6),
                        ("x=.22 t-35",(0.22,0.047,1.06),-0.6), ("x=.22 t-50",(0.22,0.047,1.06),-0.87), ("x=.13 y=0 v",(0.13,0.0,1.06),0), ("x=.13 y=.10 v",(0.13,0.10,1.06),0),
                        ("x=.13 z1.15 v",(0.13,0.047,1.15),0), ("x=.13 y=.0 z1.15 v",(0.13,0.0,1.15),0), ("x=.13 y=.1 z1.15 v",(0.13,0.10,1.15),0)]:
    q = r.ik_tcp(tcp, grasp_R(0, tilt))
    print(name, None if q is None else np.round(q,3))
EOF
timeout 300 python3 test_ik2.py

# openrua op 21
cat > plan_check.py <<'EOF'
from rob import *
r = Robot()
home = r.arm_q()
POTS = {"A": (-0.204, -0.202), "B": (-0.072, 0.225)}
HANDLE_DY = -0.061          # handle bar centre relative to pot centre (points -y)
GRASP_Z = 1.025             # TCP height for handle bar (~12.6 cm above table)
LIFT_Z = 1.20
PLACE = {"A": (0.18, 0.00), "B": (0.18, 0.094)}
PLACE_Z = 1.060
for pot in ["A", "B"]:
    px, py = POTS[pot]
    seed = home
    print("== pot", pot)
    for name, tcp, yaw in [("pre", (px, py+HANDLE_DY, 1.15), 0), ("grasp", (px, py+HANDLE_DY, GRASP_Z), 0), ("lift", (px, py+HANDLE_DY, LIFT_Z), 0)]:
        for y in (yaw, yaw+math.pi):
            q = r.ik_tcp(tcp, grasp_R(y, 0), seed=seed)
            if q is not None: break
        print(f"  {name} yaw={y:.2f}: {None if q is None else np.round(q,3)}")
        if q is not None: seed = q
    tx, ty = PLACE[pot]
    for name, tcp in [("carry", (tx-0.06, ty, LIFT_Z)), ("place", (tx-0.06, ty, PLACE_Z)), ("retreat", (tx-0.06, ty, LIFT_Z))]:
        for y in (-math.pi/2, math.pi/2):
            q = r.ik_tcp(tcp, grasp_R(y, 0), seed=seed)
            if q is not None: break
        print(f"  {name} yaw={y:.2f}: {None if q is None else np.round(q,3)}")
        if q is not None:
            seed = q
            print("     fk check tcp:", np.round(r.fk_hand(q)[2],4), "hy:", np.round(R_from_q(*r.fk_hand(q)[1])[:,1],2))
EOF
timeout 600 python3 plan_check.py

# openrua op 22
cat > dbg.py <<'EOF'
from rob import *
for yaw in (0, -math.pi/2):
    R = grasp_R(yaw, 0); q = q_from_R(R)
    print("yaw", yaw, "\nR=\n", np.round(R,3), "\nq=", np.round(q,4), "\nR_from_q=\n", np.round(R_from_q(*q),3), "det", np.linalg.det(R))
EOF
python3 dbg.py

# openrua op 23
python3 - <<'EOF'
import re
s = open("rob.py").read()
s = s.replace('''        pos = np.asarray(tcp_world) - R[:, 2] * TCP - BASE
        q = q_from_R(R)''', '''        pos = np.asarray(tcp_world) - R[:, 2] * TCP - BASE
        # IK tip link is panda_link8 = panda_hand rotated +45deg about z (tf: link8->hand is Rz(-45deg))
        c, s_ = math.cos(math.pi/4), math.sin(math.pi/4)
        R8 = R @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        q = q_from_R(R8)''')
open("rob.py","w").write(s)
EOF
cat > dbg2.py <<'EOF'
from rob import *
r = Robot()
for yaw in (0, -math.pi/2):
    q = r.ik_tcp((0.12, 0.0, 1.2), grasp_R(yaw,0))
    pos, quat, tcp = r.fk_hand(q)
    print("yaw", round(yaw,2), "q", np.round(q,3), "tcp", np.round(tcp,4), "hand R=\n", np.round(R_from_q(*quat),3))
EOF
timeout 120 python3 dbg2.py

# openrua op 24
cat > exec.py <<'EOF'
"""Usage: python3 exec.py <pick|place> <A|B>   (logs to stdout; run with -u)"""
import sys
from rob import *

POTS = {"A": (-0.204, -0.202), "B": (-0.072, 0.225)}
HANDLE_DY = -0.0605         # handle bar centre relative to pot centre (handles point -y)
GRASP_Z = 1.025             # TCP height on the handle bar (~12.6 cm above table)
PRE_Z = 1.15
LIFT_Z = 1.20
PLACE = {"A": (0.18, 0.00), "B": (0.18, 0.094)}   # pot body centres on the stove plate
PLACE_Z = 1.060             # base ~0.3 cm above stove top (0.93 + 0.127)
CARRY_DX = -0.06            # after -90deg yaw the handle points -x: TCP sits 6 cm toward robot

def goto(r, tcp, yaw, seconds, seed=None, alt_yaws=()):
    for y in (yaw,) + tuple(alt_yaws):
        q = r.ik_tcp(tcp, grasp_R(y, 0), seed=seed)
        if q is not None: break
    if q is None:
        raise SystemExit(f"IK failed for {tcp}")
    print(f"-> tcp {np.round(tcp,3)} yaw {y:.2f} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)}  (err {np.round(np.linalg.norm(t-np.array(tcp))*1000,1)} mm)", flush=True)
    return q

def main():
    mode, pot = sys.argv[1], sys.argv[2]
    r = Robot()
    print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
    if mode == "pick":
        px, py = POTS[pot]; hx, hy = px, py + HANDLE_DY
        r.gripper(GRIP["open_m"])
        goto(r, (hx, hy, PRE_Z), 0.0, 4.0, alt_yaws=(math.pi,))
        goto(r, (hx, hy, GRASP_Z), 0.0, 2.5, alt_yaws=(math.pi,))
        f = r.gripper(GRIP["closed_m"])
        gap = abs(f[0]) + abs(f[1])
        print(f"  finger gap after close: {gap*1000:.1f} mm", flush=True)
        if gap < 0.004:
            print("GRASP FAILED (closed on air)", flush=True)
            r.gripper(GRIP["open_m"]); goto(r, (hx, hy, PRE_Z), 0.0, 2.5, alt_yaws=(math.pi,)); return
        goto(r, (hx, hy, LIFT_Z), 0.0, 3.0, alt_yaws=(math.pi,))
        print("fingers after lift", r.fingers(), flush=True)
    elif mode == "place":
        tx, ty = PLACE[pot]; cx, cy = tx + CARRY_DX, ty
        goto(r, (cx, cy, LIFT_Z), -math.pi/2, 5.0, alt_yaws=(math.pi/2,))
        print("fingers at carry", r.fingers(), flush=True)
        goto(r, (cx, cy, PLACE_Z), -math.pi/2, 3.0, alt_yaws=(math.pi/2,))
        r.gripper(GRIP["open_m"])
        goto(r, (cx, cy, LIFT_Z), -math.pi/2, 3.0, alt_yaws=(math.pi/2,))
    print("done", flush=True)

main()
EOF
timeout 900 python3 -u exec.py pick A 2>&1 | tee logs_pickA.txt

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/pickA.png

# openrua op 26
timeout 900 python3 -u exec.py place A 2>&1 | tee logs_placeA.txt && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/placeA.png

# openrua op 27
cat > locate.py <<'EOF'
import numpy as np, cv2, subprocess, sys
subprocess.run(["timeout","60","python3","tools/perception/cam_snap.py","/birdview/depth/image_raw","snaps/bird_now.png"],check=True,capture_output=True)
d = np.load("snaps/bird_now.npy"); fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
wx = -0.2 + (vs-cy)*d/fy; wy = (us-cx)*d/fx; wz = 3.0 - d
table=0.899
mask = ((wz > table+0.008) & (wz < table+0.5)).astype(np.uint8); mask[:150,:]=0
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 20: continue
    m = lab==i
    top = m & (wz > wz[m].max()-0.03)   # topmost 3 cm (knob/lid) -> body centre
    print(f"comp area={stats[i,4]:5d} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] ztop={wz[m].max():.3f}  top-centre=({wx[top].mean():.3f},{wy[top].mean():.3f})")
EOF
python3 locate.py

# openrua op 28
cat > home.py <<'EOF'
from rob import *
r = Robot()
HOME = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
code, err = r.move_q(HOME, 4.0)
if err > 0.02: r.move_q(HOME, 4.0)
print("fk", np.round(r.fk_hand()[2],3))
EOF
timeout 600 python3 -u home.py && python3 locate.py

# openrua op 29
cat > potpose.py <<'EOF'
"""Estimate a pot's body centre and handle direction from a fresh birdview depth frame."""
import numpy as np, subprocess, sys
def measure(fresh=True):
    if fresh:
        subprocess.run(["timeout","60","python3","tools/perception/cam_snap.py","/birdview/depth/image_raw","snaps/bird_now.png"],check=True,capture_output=True)
    d = np.load("snaps/bird_now.npy"); fx=fy=579.4112549695428; cx=320; cy=240
    H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
    wx = -0.2 + (vs-cy)*d/fy; wy = (us-cx)*d/fx; wz = 3.0 - d
    return wx, wy, wz
def pot_pose(wx, wy, wz, cx0, cy0, support_z):
    box = (np.abs(wx-cx0)<0.09)&(np.abs(wy-cy0)<0.09)
    h = wz - support_z
    top = box & (h > 0.145)                    # knob
    if top.sum() < 3: return None
    kc = np.array([wx[top].mean(), wy[top].mean()])
    body = box & (h > 0.12) & (h < 0.145)      # lid + handle bar
    far = body & (np.hypot(wx-kc[0], wy-kc[1]) > 0.045)   # beyond the lid radius -> handle
    if far.sum() < 3: return kc, None, wz[top].max()
    hc = np.array([wx[far].mean(), wy[far].mean()])
    ang = np.degrees(np.arctan2(hc[1]-kc[0]*0-kc[1], hc[0]-kc[0]))
    return kc, ang, wz[top].max()
if __name__ == "__main__":
    wx, wy, wz = measure()
    for name,(x,y,sz) in {"A@stove":(0.116,0.003,0.93),"B":(-0.076,0.220,0.899)}.items():
        print(name, pot_pose(wx, wy, wz, x, y, sz))
EOF
python3 potpose.py

# openrua op 30
cat >> rob.py <<'EOF'

def xbar_R(tilt):
    """Fingers close along world y (handle bar runs along x); approach pitched by `tilt` rad
    toward +x so the wrist sits closer to the robot. tilt=0 is the plain top-down grasp."""
    R0 = grasp_R(-math.pi/2, 0)
    c, s = math.cos(-tilt), math.sin(-tilt)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return Ry @ R0
EOF
cat > test_ik3.py <<'EOF'
from rob import *
r = Robot()
R = xbar_R(0.52)
print("xbar_R(30deg)=\n", np.round(R,3))
seed = r.arm_q()
for name, tcp, t in [("A bar", (0.06,0.002,1.057),0.52), ("A bar pre",(0.06,0.002,1.15),0.52),
                     ("place .18 z1.09",(0.18,0.0,1.09),0.52), ("place .18 z1.06",(0.18,0.0,1.06),0.52), ("place .18 z1.20",(0.18,0.0,1.20),0.52),
                     ("place .19 z1.09",(0.19,0.0,1.09),0.52), ("place .18 y.097 z1.09",(0.18,0.097,1.09),0.52), ("place .18 y.097 z1.20",(0.18,0.097,1.20),0.52),
                     ("place .18 z1.09 t40",(0.18,0.0,1.09),0.70), ("place .18 z1.20 t40",(0.18,0.0,1.20),0.70),
                     ("B bar pre t0",(-0.076,0.164,1.15),0.0)]:
    q = r.ik_tcp(tcp, xbar_R(t), seed=seed)
    if q is not None:
        pos, quat, tc = r.fk_hand(q); Rh = R_from_q(*quat)
        print(f"{name}: q={np.round(q,3)} tcp_err={np.linalg.norm(tc-np.array(tcp))*1000:.1f}mm hz={np.round(Rh[:,2],2)} hy={np.round(Rh[:,1],2)}")
    else: print(name, None)
EOF
timeout 600 python3 test_ik3.py

# openrua op 31
cat > exec2.py <<'EOF'
"""Handle-bar pick/place with the bar along x (fingers close along y), approach pitched 30 deg.
Usage: python3 exec2.py pickx <bar_x> <bar_y> <bar_z>
       python3 exec2.py placex <x> <y> <z_release>
"""
import sys
from rob import *
TILT = 0.52
LIFT_Z = 1.20

def goto(r, tcp, seconds, seed=None):
    q = r.ik_tcp(tcp, xbar_R(TILT), seed=seed)
    if q is None: raise SystemExit(f"IK failed for {tcp}")
    print(f"-> tcp {np.round(tcp,3)} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)}  (err {np.linalg.norm(t-np.array(tcp))*1000:.1f} mm)", flush=True)
    return q

r = Robot()
mode = sys.argv[1]; x, y, z = map(float, sys.argv[2:5])
print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
if mode == "pickx":
    r.gripper(GRIP["open_m"])
    goto(r, (x, y, z + 0.10), 4.0)
    goto(r, (x, y, z), 2.5)
    f = r.gripper(GRIP["closed_m"]); gap = abs(f[0]) + abs(f[1])
    print(f"  finger gap after close: {gap*1000:.1f} mm", flush=True)
    if gap < 0.004:
        print("GRASP FAILED (closed on air)", flush=True)
        r.gripper(GRIP["open_m"]); goto(r, (x, y, z + 0.10), 2.5); sys.exit(1)
    goto(r, (x, y, LIFT_Z), 3.0)
    print("fingers after lift", r.fingers(), flush=True)
elif mode == "placex":
    goto(r, (x, y, LIFT_Z), 5.0)
    print("fingers at carry", r.fingers(), flush=True)
    goto(r, (x, y, z + 0.04), 2.5)
    goto(r, (x, y, z), 2.0)
    r.gripper(GRIP["open_m"])
    goto(r, (x, y, LIFT_Z), 3.0)
print("done", flush=True)
EOF
timeout 900 python3 -u exec2.py pickx 0.0595 0.0024 1.057 2>&1 | tee logs_pickA2.txt

# openrua op 32
timeout 900 python3 -u exec2.py placex 0.18 -0.003 1.08 2>&1 | tee logs_placeA2.txt && timeout 600 python3 -u home.py && python3 potpose.py

# openrua op 33
python3 locate.py; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/placeA2.png

# openrua op 34
cat > potpose2.py <<'EOF'
import numpy as np
from potpose import measure, pot_pose
wx, wy, wz = measure(fresh=False)
print("A", pot_pose(wx, wy, wz, 0.217, -0.004, 0.93))
# footprint of everything above the stove top in the stove region
m = (wx>0.05)&(wx<0.32)&(wy>-0.08)&(wy<0.17)&(wz>0.93+0.01)
print("above-stove pixels x[%.3f,%.3f] y[%.3f,%.3f]" % (wx[m].min(), wx[m].max(), wy[m].min(), wy[m].max()))
for h0,h1 in [(0.01,0.05),(0.05,0.10),(0.10,0.13),(0.13,0.145),(0.145,0.2)]:
    mm = m & (wz-0.93>=h0)&(wz-0.93<h1)
    if mm.sum(): print(f"  h{h0*100:.0f}-{h1*100:.0f}: n={mm.sum()} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}] c=({wx[mm].mean():.3f},{wy[mm].mean():.3f})")
EOF
python3 potpose2.py

# openrua op 35
sed -i 's/^PLACE = {"A": (0.18, 0.00), "B": (0.18, 0.094)}.*/PLACE = {"A": (0.18, 0.00), "B": (0.19, 0.097)}   # TCP lands at (x+CARRY_DX, y); pot body settles under the TCP/' exec.py && sed -i 's/^PLACE_Z = 1.060.*/PLACE_Z = 1.075             # contact happens ~1.085 with the hanging pot; press gently/' exec.py && grep -n "^PLACE" exec.py && timeout 900 python3 -u exec.py pick B 2>&1 | tee logs_pickB.txt

# openrua op 36
timeout 900 python3 -u exec.py place B 2>&1 | tee logs_placeB.txt && timeout 600 python3 -u home.py && python3 locate.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/placeB.png

# openrua op 37
cat > test_ik4.py <<'EOF'
from rob import *
r = Robot()
print("fingers", r.fingers())
seed = r.arm_q()
for yaw in (0.0, -math.pi/2):
    for z in (1.20, 1.12):
        row = []
        for x in (0.08, 0.10, 0.12, 0.13, 0.14):
            q = r.ik_tcp((x, 0.097, z), grasp_R(yaw, 0), seed=seed)
            row.append("ok " if q is not None else "-- ")
        print(f"yaw {yaw:5.2f} z {z}: x=.08,.10,.12,.13,.14 -> {''.join(row)}")
for t in (0.35, 0.52):
    row=[]
    for x in (0.10, 0.13, 0.15, 0.18):
        q = r.ik_tcp((x, 0.097, 1.20), xbar_R(t), seed=seed); row.append("ok " if q is not None else "-- ")
    print(f"xbar tilt {t}: x=.10,.13,.15,.18 -> {''.join(row)}")
EOF
timeout 900 python3 test_ik4.py

# openrua op 38
sed -i 's/^PLACE = {"A": (0.18, 0.00), "B": (0.19, 0.097)}.*/PLACE = {"A": (0.18, 0.00), "B": (0.18, 0.097)}   # TCP lands at (x+CARRY_DX, y); pot body settles at/just beyond the TCP/' exec.py && timeout 900 python3 -u exec.py place B 2>&1 | tee logs_placeB.txt && timeout 600 python3 -u home.py && python3 locate.py

# openrua op 39
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/exec.py", "old_string": "PLACE = {\\"A\\": (0.18, 0.00), \\"B\\": (0.19, 0.097)}   # TCP lands at (x+CARRY_DX, y); pot body settles under the TCP", "new_string": "PLACE = {\\"A\\": (0.18, 0.00), \\"B\\": (0.18, 0.097)}   # TCP lands at (x+CARRY_DX, y); pot body settles at or just beyond the TCP", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 40
timeout 900 python3 -u exec.py place B 2>&1 | tee logs_placeB.txt && timeout 600 python3 -u home.py && python3 locate.py

# openrua op 41
cat > stovecheck.py <<'EOF'
import numpy as np
from potpose import measure, pot_pose
wx, wy, wz = measure(fresh=True)
for name,(x,y) in {"A":(0.22,-0.004),"B":(0.13,0.097)}.items():
    print(name, pot_pose(wx, wy, wz, x, y, 0.93))
m = (wx>0.05)&(wx<0.32)&(wy>-0.08)&(wy<0.17)&(wz>0.93+0.01)
for h0,h1 in [(0.01,0.05),(0.05,0.10),(0.10,0.13),(0.13,0.145),(0.145,0.2)]:
    mm = m & (wz-0.93>=h0)&(wz-0.93<h1)
    if mm.sum(): print(f"  h{h0*100:.0f}-{h1*100:.0f}: n={mm.sum()} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}]")
EOF
python3 stovecheck.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/placeB.png

# openrua op 42
timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_color.png && python3 -c "
import cv2; im=cv2.imread('snaps/bird_color.png'); crop=im[290:400,290:420]; cv2.imwrite('snaps/bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front2.png && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side2.png

# openrua op 43
cat > lying.py <<'EOF'
import numpy as np
from potpose import measure
wx, wy, wz = measure(fresh=True)
S = 0.93
m = (wx>0.09)&(wx<0.30)&(wy>0.04)&(wy<0.15)&(wz>S+0.01)
h = wz - S
print("lying pot B: x[%.3f,%.3f] y[%.3f,%.3f] hmax=%.3f" % (wx[m].min(), wx[m].max(), wy[m].min(), wy[m].max(), h[m].max()))
for x0 in np.arange(0.10, 0.29, 0.01):
    mm = m & (wx>=x0) & (wx<x0+0.01)
    if mm.sum()<2: continue
    ridge = mm & (h > h[mm].max()-0.01)
    print(f"  x {x0:.2f}: n={mm.sum():3d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] width={100*(wy[mm].max()-wy[mm].min()):.1f}cm hmax={h[mm].max()*100:.1f}cm ridge_y={wy[ridge].mean():.3f}")
EOF
python3 lying.py

# openrua op 44
cat > test_ik5.py <<'EOF'
from rob import *
r = Robot()
seed = r.arm_q()
tests = [("base grasp t0", (0.143,0.096,0.968), 0.0), ("base grasp pre t0", (0.143,0.096,1.10), 0.0),
         ("base grasp t.35", (0.143,0.096,0.968), 0.35),
         ("vbar t.52", (0.247,0.096,1.03), 0.52), ("vbar t.785", (0.247,0.096,1.03), 0.785), ("vbar t1.0", (0.247,0.096,1.03), 1.0),
         ("vbar pre t.785", (0.247,0.096,1.13), 0.785),
         ("arc0 x.217 z1.089 t.785", (0.217,0.096,1.089), 0.785), ("arc1 x.17 z1.088 t.5", (0.17,0.096,1.088), 0.5),
         ("arc2 x.13 z1.075 t.25", (0.13,0.096,1.075), 0.25), ("arc3 x.099 z1.056 t0", (0.099,0.096,1.056), 0.0)]
for name, tcp, t in tests:
    q = r.ik_tcp(tcp, xbar_R(t), seed=seed)
    print(f"{name}: {'ok '+str(np.round(q,2)) if q is not None else 'NO'}")
EOF
timeout 900 python3 test_ik5.py

# openrua op 45
grep -n "def move_q" -A 40 rob.py | head -60; echo ---; cat lying.py | head -40; echo ---; timeout 300 python3 lying.py 2>&1 | tail -25

# openrua op 46
mkdir -p "$(dirname /workspace/right.py)"
cat > /workspace/right.py <<'OPENRUA_EOF'
"""Right pot B (lying on the stove, base toward robot, handle arch on top).
Usage: python3 -u right.py dry | pick | place
  pick : grasp the handle arch top with a pitched hand and lift (pot swings to hang)
  place: touch down, roll the pot upright about its base edge along an arc, release
Pot frame: a = axis from base, h = toward handle. Grip point G=(11.4, 9.4) cm, CoM~(6.0,0.5).
"""
import sys
from rob import *

Y = 0.096                     # pot axis y (lying) and final pot y
GX, GZ = 0.235, 1.055         # TCP on the arch top (arch top z~1.064)
PRE_Z, LIFT_Z = 1.14, 1.20
T0 = 1.0                      # initial hand pitch toward +x (rad)
# hang geometry (see notes): CoM under G -> pot rotated psi=31.2deg from lying
PSI = math.radians(31.2)
E_OFF = (-0.0307, -0.1693)    # base edge E relative to G while hanging
R_ARC = 0.170                 # |G-E| = 0.172, run 2 mm inside (gentle press)
A0, A1 = math.radians(79.7), math.radians(138.5)   # arc angles of G about E
POT_X = 0.175                 # desired final pot centre x
E_X = POT_X + 0.035
G0 = (E_X - E_OFF[0], Y, 0.93 - E_OFF[1])          # touchdown TCP

def arc_points(n=8):
    pts = []
    for k in range(n + 1):
        th = A0 + (A1 - A0) * k / n
        tcp = (E_X + R_ARC * math.cos(th), Y, 0.93 + R_ARC * math.sin(th))
        tilt = T0 - (th - A0)
        pts.append((tcp, tilt))
    return pts

def solve(r, tcp, tilt, seed):
    q = r.ik_tcp(tcp, xbar_R(tilt), seed=seed)
    if q is None:
        raise SystemExit(f"IK failed for {np.round(tcp,3)} tilt {tilt:.2f}")
    return q

def go(r, tcp, tilt, seconds, seed=None):
    q = solve(r, tcp, tilt, seed if seed is not None else r.arm_q())
    print(f"-> tcp {np.round(tcp,3)} tilt {tilt:.2f} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)} (err {np.linalg.norm(t-np.array(tcp))*1000:.1f} mm)", flush=True)
    return q

def main():
    mode = sys.argv[1]
    r = Robot()
    print("fingers", r.fingers(), "q", np.round(r.arm_q(), 3), flush=True)
    print("touchdown G0", np.round(G0, 4), flush=True)
    if mode == "dry":
        seed = r.arm_q()
        for name, tcp, t in [("pre", (GX, Y, PRE_Z), T0), ("grasp", (GX, Y, GZ), T0), ("lift", (GX, Y, LIFT_Z), T0),
                             ("above G0", (G0[0], Y, LIFT_Z), T0), ("G0", G0, T0)]:
            seed = solve(r, tcp, t, seed); print(f"{name}: {np.round(tcp,3)} t{t:.2f} -> {np.round(seed,2)}")
        prev = seed
        for tcp, t in arc_points():
            q = solve(r, tcp, t, prev)
            print(f"arc {np.round(tcp,3)} tilt {t:.2f} -> {np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(prev))):.2f}")
            prev = q
        return
    if mode == "pick":
        r.gripper(GRIP["open_m"])
        go(r, (GX, Y, PRE_Z), T0, 4.0)
        go(r, (GX, Y, GZ), T0, 2.5)
        f = r.gripper(GRIP["closed_m"])
        gap = abs(f[0]) + abs(f[1])
        print(f"  finger gap after close: {gap*1000:.1f} mm", flush=True)
        if gap < 0.004:
            print("GRASP FAILED", flush=True)
            r.gripper(GRIP["open_m"]); go(r, (GX, Y, PRE_Z), T0, 2.5); return
        go(r, (GX, Y, LIFT_Z), T0, 3.0)
        print("fingers after lift", r.fingers(), flush=True)
    elif mode == "place":
        go(r, (G0[0], Y, LIFT_Z), T0, 3.0)
        go(r, (G0[0], Y, G0[2] + 0.01), T0, 3.0)
        go(r, G0, T0, 2.0)
        prev = r.arm_q()
        for tcp, t in arc_points():
            prev = go(r, tcp, t, 1.5, seed=prev)
        print("fingers at end of arc", r.fingers(), flush=True)
        r.gripper(GRIP["open_m"])
        tcp, t = arc_points()[-1]
        go(r, (tcp[0], Y, LIFT_Z), 0.0, 3.0)
    print("done", flush=True)

main()
OPENRUA_EOF

# openrua op 47
timeout 900 python3 -u right.py dry 2>&1 | tail -25

# openrua op 48
timeout 1500 python3 -u right.py pick 2>&1 | tee logs_rightpick.txt | grep -v "^$" | tail -30

# openrua op 49
timeout 120 python3 tools/perception/cam_snap.py sideview snaps/hang_side.png && timeout 120 python3 tools/perception/cam_snap.py frontview snaps/hang_front.png

# openrua op 50
python3 -c "
import cv2
im=cv2.imread('snaps/hang_side.png'); c=im[150:340,120:320]; cv2.imwrite('snaps/hang_side_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/hang_front.png'); c=im[260:420,300:460]; cv2.imwrite('snaps/hang_front_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 51
cat > hang.py <<'EOF'
import numpy as np
from potpose import measure
wx, wy, wz = measure(fresh=True)
m = (wx>0.05)&(wx<0.40)&(wy>0.05)&(wy<0.145)&(wz>1.0)&(wz<1.25)
print("hanging B footprint: x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f zmin=%.3f" % (wx[m].min(), wx[m].max(), wy[m].min(), wy[m].max(), wz[m].max(), wz[m].min()))
for x0 in np.arange(0.05, 0.40, 0.01):
    mm = m & (wx>=x0) & (wx<x0+0.01)
    if mm.sum()<2: continue
    print(f"  x {x0:.2f}: n={mm.sum():3d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] zmax={wz[mm].max():.3f} zmin={wz[mm].min():.3f}")
EOF
timeout 300 python3 hang.py

# openrua op 52
sed -i 's/(wz>1.0)&(wz<1.25)/(wz>0.95)\&(wz<1.235)/' hang.py && timeout 300 python3 hang.py

# openrua op 53
cat > cloud.py <<'EOF'
"""cloud(cam) -> (X,Y,Z) world-frame arrays from a camera's current depth frame (TF-based)."""
import struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds/1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")

def cloud(cam):
    if not rclpy.ok(): rclpy.init()
    node = rclpy.create_node("cloud_"+cam)
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    d = CvBridge().imgmsg_to_cv2(depth, desired_encoding="passthrough").astype(np.float64)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds/1e9 + 10
    while node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()): break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    P = np.stack([(u-cx)*d/fx, (v-cy)*d/fy, d], -1).reshape(-1, 3)
    W = P @ R.T + T
    ok = np.isfinite(d.reshape(-1)) & (d.reshape(-1) > 0)
    node.destroy_node()
    return W[ok, 0], W[ok, 1], W[ok, 2], (u.reshape(-1)[ok], v.reshape(-1)[ok])

if __name__ == "__main__":
    import sys
    X, Y, Z, _ = cloud(sys.argv[1])
    print("points", X.size, "x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]" % (X.min(), X.max(), Y.min(), Y.max(), Z.min(), Z.max()))
    # table plane sanity
    m = (Z > 0.85) & (Z < 0.95) & (abs(X) < 0.4) & (abs(Y) < 0.4)
    print("table-ish z median", np.median(Z[m]) if m.any() else None)
EOF
timeout 120 python3 cloud.py sideview; timeout 120 python3 cloud.py frontview

# openrua op 54
cat > hang2.py <<'EOF'
import numpy as np
from cloud import cloud
for cam in ("sideview", "frontview"):
    X, Y, Z, _ = cloud(cam)
    m = (Y > 0.045) & (Y < 0.15) & (X > 0.08) & (X < 0.40) & (Z > 0.94) & (Z < 1.30)
    print(cam, "pot B pts", m.sum(), "x[%.3f,%.3f] z[%.3f,%.3f]" % (X[m].min(), X[m].max(), Z[m].min(), Z[m].max()))
    for z0 in np.arange(0.94, 1.30, 0.02):
        mm = m & (Z >= z0) & (Z < z0 + 0.02)
        if mm.sum() < 3: continue
        print(f"  z {z0:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF
timeout 200 python3 hang2.py

# openrua op 55
cat > right2.py <<'EOF'
"""Right the horizontally hanging pot B. Usage: python3 -u right2.py dry|place|release
Measured hang (TCP 0.235,0.096,1.198): base rim low point E at (0.124, 1.070): E-G = (-0.111,-0.128), |.|=0.169.
Upright with handle -x: G - E = (-0.123, 0.110). Arc about E from 49deg to 138deg, hand tilt fixed."""
import sys
from rob import *
Y = 0.096; T = 1.0; LIFT_Z = 1.20
E_X = 0.135                       # base edge on stove -> final pot centre ~0.170
R_ARC = 0.167
A0, A1 = math.atan2(0.128, 0.111), math.atan2(0.110, -0.123)
G0 = (E_X + 0.111, Y, 0.93 + 0.128)

def arc_points(n=10):
    return [(E_X + R_ARC*math.cos(A0 + (A1-A0)*k/n), Y, 0.93 + R_ARC*math.sin(A0 + (A1-A0)*k/n)) for k in range(n+1)]

def go(r, tcp, tilt, seconds, seed=None, retry=True):
    q = r.ik_tcp(tcp, xbar_R(tilt), seed=seed if seed is not None else r.arm_q())
    if q is None: raise SystemExit(f"IK failed {np.round(tcp,3)} t{tilt}")
    print(f"-> tcp {np.round(tcp,3)} tilt {tilt:.2f} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02 and retry:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)} (err {np.linalg.norm(t-np.array(tcp))*1000:.1f} mm) fingers {np.round(r.fingers(),4)}", flush=True)
    return q

r = Robot()
mode = sys.argv[1]
print("fingers", r.fingers(), "G0", np.round(G0,4), "arc end", np.round(arc_points()[-1],4), flush=True)
if mode == "dry":
    seed = r.arm_q()
    for tcp in [(G0[0], Y, LIFT_Z), G0] + arc_points():
        q = r.ik_tcp(tcp, xbar_R(T), seed=seed)
        print(np.round(tcp,3), "NO" if q is None else f"{np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}")
        if q is not None: seed = q
elif mode == "place":
    go(r, (G0[0], Y, LIFT_Z), T, 3.0)
    go(r, (G0[0], Y, G0[2] + 0.015), T, 3.0)
    go(r, G0, T, 2.0, retry=False)
    prev = r.arm_q()
    for tcp in arc_points()[1:]:
        prev = go(r, tcp, T, 1.5, seed=prev, retry=False)
    print("arc done; check pose before release", flush=True)
elif mode == "release":
    r.gripper(GRIP["open_m"])
    pos, quat, t = r.fk_hand()
    go(r, (t[0], t[1], t[2] + 0.06), T, 2.0)
    go(r, (t[0] - 0.05, t[1], LIFT_Z), 0.3, 3.0)
print("done", flush=True)
EOF
timeout 600 python3 -u right2.py dry 2>&1 | tail -16

# openrua op 56
python3 - <<'EOF'
import re
s = open('right2.py').read()
s = s.replace("def arc_points(n=10):\n    return [(E_X + R_ARC*math.cos(A0 + (A1-A0)*k/n), Y, 0.93 + R_ARC*math.sin(A0 + (A1-A0)*k/n)) for k in range(n+1)]",
"def arc_points(n=10):\n    return [((E_X + R_ARC*math.cos(A0 + (A1-A0)*k/n), Y, 0.93 + R_ARC*math.sin(A0 + (A1-A0)*k/n)), T - 0.4*k/n) for k in range(n+1)]")
s = s.replace("""    for tcp in [(G0[0], Y, LIFT_Z), G0] + arc_points():
        q = r.ik_tcp(tcp, xbar_R(T), seed=seed)
        print(np.round(tcp,3), "NO" if q is None else f"{np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}")""",
"""    for tcp, t in [((G0[0], Y, LIFT_Z), T), (G0, T)] + arc_points():
        q = r.ik_tcp(tcp, xbar_R(t), seed=seed)
        print(np.round(tcp,3), f"t{t:.2f}", "NO" if q is None else f"{np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}")""")
s = s.replace("""    for tcp in arc_points()[1:]:
        prev = go(r, tcp, T, 1.5, seed=prev, retry=False)""",
"""    for tcp, t in arc_points()[1:]:
        prev = go(r, tcp, t, 1.5, seed=prev, retry=False)""")
s = s.replace('"arc end", np.round(arc_points()[-1],4)', '"arc end", np.round(arc_points()[-1][0],4)')
open('right2.py','w').write(s)
EOF
timeout 600 python3 -u right2.py dry 2>&1 | tail -14

# openrua op 57
timeout 1500 python3 -u right2.py place 2>&1 | tee logs_right_place.txt | grep -v "^$" | tail -45

# openrua op 58
timeout 120 python3 tools/perception/cam_snap.py sideview snaps/arc_side.png >/dev/null && timeout 120 python3 tools/perception/cam_snap.py frontview snaps/arc_front.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('snaps/arc_side.png'); c=im[140:360,100:340]; cv2.imwrite('snaps/arc_side_crop.png', cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/arc_front.png'); c=im[250:480,240:460]; cv2.imwrite('snaps/arc_front_crop.png', cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
" && timeout 200 python3 hang2.py

# openrua op 59
cat > flip.py <<'EOF'
"""Carry pot B (held rigidly, horizontal, base toward -x) to a free table spot and lay it down
with the lid toward -x. Usage: python3 -u flip.py dry|run"""
import sys
from rob import *
def R_of(yaw, tilt):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ xbar_R(tilt)
SPOT = (0.05, 0.22)
STEPS = [((0.05, 0.10, 1.25), 0.0, 0.6),        # lift clear of knob object
         (SPOT + (1.25,), 0.0, 0.6),            # carry to free spot
         (SPOT + (1.25,), math.pi/4, 0.6), (SPOT + (1.25,), math.pi/2, 0.6),   # yaw 90 (pot base -> -y)
         (SPOT + (1.25,), math.pi/2, 0.3), (SPOT + (1.25,), math.pi/2, 0.0),   # un-pitch (base end rises)
         (SPOT + (1.25,), 3*math.pi/4, 0.0), (SPOT + (1.25,), math.pi, 0.0),   # yaw to 180 (base -> +x)
         (SPOT + (1.05,), math.pi, 0.0), (SPOT + (0.97,), math.pi, 0.0)]       # lower until lid rim touches
r = Robot(); mode = sys.argv[1]
print("fingers", r.fingers(), flush=True)
seed = r.arm_q()
for tcp, yaw, tilt in STEPS:
    q = r.ik_tcp(tcp, R_of(yaw, tilt), seed=seed)
    if q is None: raise SystemExit(f"IK failed {tcp} yaw {yaw:.2f} tilt {tilt:.2f}")
    print(f"-> {np.round(tcp,3)} yaw {yaw:.2f} tilt {tilt:.2f} q {np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}", flush=True)
    if mode == "run":
        code, err = r.move_q(q, 3.0)
        if err > 0.02 and tcp[2] > 1.0:
            print("  retry", flush=True); code, err = r.move_q(q, 3.0)
        pos, quat, t = r.fk_hand(); print(f"  fk tcp {np.round(t,4)} fingers {np.round(r.fingers(),4)}", flush=True)
        seed = r.arm_q()
    else:
        seed = q
if mode == "run":
    r.gripper(GRIP["open_m"])
    pos, quat, t = r.fk_hand()
    q = r.ik_tcp((t[0], t[1], 1.25), R_of(math.pi, 0.0), seed=r.arm_q()); r.move_q(q, 3.0)
print("done", flush=True)
EOF
timeout 600 python3 -u flip.py dry 2>&1 | tail -12

# openrua op 60
cat > flip_dry2.py <<'EOF'
from rob import *
from flip import R_of  # noqa
EOF
python3 - <<'EOF'
s = open('flip.py').read()
s = s.replace("""STEPS = [((0.05, 0.10, 1.25), 0.0, 0.6),        # lift clear of knob object
         (SPOT + (1.25,), 0.0, 0.6),            # carry to free spot
         (SPOT + (1.25,), math.pi/4, 0.6), (SPOT + (1.25,), math.pi/2, 0.6),   # yaw 90 (pot base -> -y)
         (SPOT + (1.25,), math.pi/2, 0.3), (SPOT + (1.25,), math.pi/2, 0.0),   # un-pitch (base end rises)
         (SPOT + (1.25,), 3*math.pi/4, 0.0), (SPOT + (1.25,), math.pi, 0.0),   # yaw to 180 (base -> +x)
         (SPOT + (1.05,), math.pi, 0.0), (SPOT + (0.97,), math.pi, 0.0)]       # lower until lid rim touches""",
"""SGN = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0   # yaw direction
Y8 = [SGN*math.pi*k/8 for k in range(1, 5)]
STEPS = [((0.05, 0.10, 1.25), 0.0, 0.6),        # lift clear of knob object
         (SPOT + (1.25,), 0.0, 0.6)]            # carry to free spot
STEPS += [(SPOT + (1.25,), y, 0.6) for y in Y8]                                 # yaw 90 (pot base -> -/+y)
STEPS += [(SPOT + (1.25,), Y8[-1], 0.3), (SPOT + (1.25,), Y8[-1], 0.0)]         # un-pitch (base end rises)
STEPS += [(SPOT + (1.25,), SGN*math.pi*k/8, 0.0) for k in range(5, 9)]          # yaw to 180 (base -> +x)
STEPS += [(SPOT + (1.05,), SGN*math.pi, 0.0), (SPOT + (0.97,), SGN*math.pi, 0.0)]  # lower until lid rim touches""")
s = s.replace('R_of(math.pi, 0.0), seed=r.arm_q())', 'R_of(SGN*math.pi, 0.0), seed=r.arm_q())')
open('flip.py','w').write(s)
EOF
echo "== +yaw"; timeout 600 python3 -u flip.py dry 1 2>&1 | grep -- "->" | awk '{print $NF, $0}' | cut -c1-140 | sed 's/^[^ ]* //' | grep -o "yaw.*" ; echo "== -yaw"; timeout 600 python3 -u flip.py dry -1 2>&1 | grep -- "->" | grep -o "yaw.*"

# openrua op 61
cat > t_ik6.py <<'EOF'
from rob import *
from flip import R_of
EOF
python3 - <<'EOF'
# make flip.py importable without running: guard main
s = open('flip.py').read()
if "if __name__" not in s:
    head, body = s.split("r = Robot(); mode = sys.argv[1]")
    body = "r = Robot(); mode = sys.argv[1]" + body
    body = "\n".join("    " + l if l.strip() else l for l in body.splitlines())
    s = head + "if __name__ == '__main__':\n" + body + "\n"
    open('flip.py','w').write(s)
EOF
cat >> t_ik6.py <<'EOF'
r = Robot(); seed = r.arm_q()
for tilt in (0.0, 0.2, 0.3, 0.45):
    for z in (1.25, 1.10):
        q = r.ik_tcp((0.05, 0.22, z), R_of(math.pi, tilt), seed=seed)
        print(f"yaw pi tilt {tilt} z {z}: {'NO' if q is None else np.round(q,2)}")
EOF
timeout 600 python3 t_ik6.py 2>&1 | tail -8

# openrua op 62
cat > flip2.py <<'EOF'
"""Lay pot B (held rigidly, horizontal, base toward -x) down along y on free table: base -> -y, knob -> +y.
Usage: python3 -u flip2.py dry|run"""
import sys
from rob import *
from flip import R_of
SPOT = (-0.07, 0.25)
Y4 = [math.pi/2 * k / 4 for k in range(1, 5)]
STEPS = [((0.05, 0.10, 1.25), 0.0, 0.6), (SPOT + (1.25,), 0.0, 0.6)]
STEPS += [(SPOT + (1.25,), y, 0.6) for y in Y4]
STEPS += [(SPOT + (1.10,), math.pi/2, 0.6), (SPOT + (1.03,), math.pi/2, 0.6), (SPOT + (1.022,), math.pi/2, 0.6)]
r = Robot(); mode = sys.argv[1]
print("fingers", r.fingers(), flush=True)
seed = r.arm_q()
for tcp, yaw, tilt in STEPS:
    q = r.ik_tcp(tcp, R_of(yaw, tilt), seed=seed)
    if q is None: raise SystemExit(f"IK failed {tcp} yaw {yaw:.2f} tilt {tilt:.2f}")
    print(f"-> {np.round(tcp,3)} yaw {yaw:.2f} tilt {tilt:.2f} q {np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}", flush=True)
    if mode == "run":
        code, err = r.move_q(q, 3.0)
        if err > 0.02 and tcp[2] > 1.05:
            print("  retry", flush=True); code, err = r.move_q(q, 3.0)
        pos, quat, t = r.fk_hand(); print(f"  fk tcp {np.round(t,4)} fingers {np.round(r.fingers(),4)}", flush=True)
        seed = r.arm_q()
    else:
        seed = q
if mode == "run":
    r.gripper(GRIP["open_m"])
    pos, quat, t = r.fk_hand()
    q = r.ik_tcp((t[0], t[1], 1.25), R_of(math.pi/2, 0.6), seed=r.arm_q()); r.move_q(q, 3.0)
    print("fk", np.round(r.fk_hand()[2],4))
# knob grasp reachability preview
for t in (1.0, 0.8):
    q = r.ik_tcp((SPOT[0], SPOT[1] + 0.043, 0.934), R_of(-math.pi/2, t), seed=r.arm_q())
    print(f"knob grasp preview tilt {t}: {'NO' if q is None else np.round(q,2)}")
print("done", flush=True)
EOF
timeout 600 python3 -u flip2.py dry 2>&1 | tail -13

# openrua op 63
sed -i 's/(1.25,)/(1.30,)/g; s/(0.05, 0.10, 1.25)/(0.05, 0.10, 1.30)/; s/(t\[0\], t\[1\], 1.25)/(t[0], t[1], 1.30)/' flip2.py && timeout 1500 python3 -u flip2.py run 2>&1 | tee logs_flip2.txt | grep -v "^$" | tail -40

# openrua op 64
cat > lying2.py <<'EOF'
import numpy as np
from potpose import measure
wx, wy, wz = measure(fresh=True)
S = 0.899
m = (wx>-0.20)&(wx<0.05)&(wy>0.05)&(wy<0.45)&(wz>S+0.01)
h = wz - S
print("pot B on table: x[%.3f,%.3f] y[%.3f,%.3f] hmax=%.3f" % (wx[m].min(), wx[m].max(), wy[m].min(), wy[m].max(), h[m].max()))
for y0 in np.arange(0.08, 0.36, 0.01):
    mm = m & (wy>=y0) & (wy<y0+0.01)
    if mm.sum()<2: continue
    ridge = mm & (h > h[mm].max()-0.01)
    print(f"  y {y0:.2f}: n={mm.sum():3d} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] width={100*(wx[mm].max()-wx[mm].min()):.1f}cm hmax={h[mm].max()*100:.1f}cm ridge_x={wx[ridge].mean():.3f}")
EOF
timeout 300 python3 lying2.py

# openrua op 65
cat > park.py <<'EOF'
import sys
from rob import *
r = Robot()
tcp = tuple(map(float, sys.argv[1:4])) if len(sys.argv) >= 4 else (0.0, -0.30, 1.25)
q = r.ik_tcp(tcp, grasp_R(-math.pi/2, 0), seed=r.arm_q())
if q is None: raise SystemExit("IK failed")
code, err = r.move_q(q, 4.0)
if err > 0.02: code, err = r.move_q(q, 4.0)
print("fk", np.round(r.fk_hand()[2], 4), "fingers", np.round(r.fingers(), 4))
EOF
timeout 600 python3 park.py 2>&1 | tail -3 && timeout 300 python3 lying2.py

# openrua op 66
timeout 60 python3 -c "
from rob import *
r=Robot(); print('q', np.round(r.arm_q(),3)); print('fk', np.round(r.fk_hand()[2],4))" 2>&1 | tail -2; timeout 120 python3 tools/perception/cam_snap.py frontview snaps/lay_front.png >/dev/null; timeout 120 python3 tools/perception/cam_snap.py agentview snaps/lay_agent.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('snaps/lay_front.png'); print(im.shape)
cv2.imwrite('snaps/lay_front_crop.png', cv2.resize(im[240:480,300:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"

# openrua op 67
cat > knob.py <<'EOF'
import numpy as np
from cloud import cloud
X, Y, Z, _ = cloud("frontview")
m = (X > -0.16) & (X < 0.0) & (Y > 0.22) & (Y < 0.32) & (Z > 0.905) & (Z < 1.05)
print("pts", m.sum())
for y0 in np.arange(0.22, 0.30, 0.005):
    mm = m & (Y >= y0) & (Y < y0 + 0.005)
    if mm.sum() < 2: continue
    print(f"  y {y0:.3f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]  zc={0.5*(Z[mm].min()+Z[mm].max()):.3f}")
EOF
timeout 200 python3 knob.py

# openrua op 68
python3 -c "
import cv2
im=cv2.imread('snaps/lay_front.png'); cv2.imwrite('snaps/lay_front_crop.png', cv2.resize(im[290:400,350:480],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 69
cat > t_ik7.py <<'EOF'
from rob import *
from flip import R_of
r = Robot(); seed = r.arm_q()
H = math.pi/2
tests = [("hook start", (-0.16, 0.205, 1.016), 0.0, H), ("hook in", (-0.077, 0.205, 1.016), 0.0, H),
         ("hook arc mid", (-0.077, 0.16, 1.045), 0.0, H), ("hook arc end", (-0.077, 0.077, 1.048), 0.0, H),
         ("hook retreat", (-0.077, 0.077, 1.15), 0.0, H),
         ("waist pre", (-0.077, 0.26, 0.9615), -H, H), ("waist grasp", (-0.077, 0.14, 0.9615), -H, H),
         ("waist lift", (-0.077, 0.14, 1.20), -H, H), ("yaw mid", (-0.077, 0.14, 1.20), -H/2, H), ("yaw +x", (-0.077, 0.14, 1.20), 0.0, H),
         ("carry", (0.19, 0.10, 1.20), 0.0, H), ("place", (0.19, 0.10, 0.9955), 0.0, H), ("place2", (0.17, 0.10, 0.9955), 0.0, H),
         ("place tilt1.3", (0.19, 0.10, 0.9955), 0.0, 1.3)]
for name, tcp, yaw, tilt in tests:
    q = r.ik_tcp(tcp, R_of(yaw, tilt), seed=seed)
    print(f"{name}: {np.round(tcp,3)} -> {'NO' if q is None else np.round(q,2)}")
    if q is not None: seed = q
EOF
timeout 900 python3 t_ik7.py 2>&1 | tail -14

# openrua op 70
cat > t_ik8.py <<'EOF'
from rob import *
from flip import R_of
from home import HOME
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
r = Robot(); H = math.pi/2
SEEDS = {"home": HOME, "cur": r.arm_q(), "s1": [0.5,0.5,0,-2.0,0,2.5,0.8], "s2": [-0.5,0.8,0.5,-2.0,-1.0,2.0,-1.0], "s3":[0.3,1.0,-0.5,-1.5,0.3,1.6,2.0], "s4":[1.0,1.0,-1.0,-1.5,0,2.0,1.5]}
def Rroll(yaw, tilt, roll):  # roll about hand z
    R = R_of(yaw, tilt); c,s = math.cos(roll), math.sin(roll)
    return R @ np.array([[c,-s,0],[s,c,0],[0,0,1]])
tests = [("hook in", (-0.077, 0.205, 1.016), 0.0, H, 0.0), ("hook in roll", (-0.077, 0.205, 1.016), 0.0, H, H),
         ("hook end", (-0.077, 0.077, 1.048), 0.0, H, 0.0), ("hook end roll", (-0.077, 0.077, 1.048), 0.0, H, H),
         ("waist grasp -y", (-0.077, 0.14, 0.9615), -H, H, 0.0), ("waist grasp +x", (-0.077-0.0, 0.14, 0.9615), 0.0, H, 0.0),
         ("waist pre +x", (-0.16, 0.14, 0.9615), 0.0, H, 0.0),
         ("place +x", (0.19, 0.10, 0.9955), 0.0, H, 0.0), ("carry +x", (0.19, 0.10, 1.20), 0.0, H, 0.0)]
for name, tcp, yaw, tilt, roll in tests:
    best = []
    for sn, sd in SEEDS.items():
        q = r.ik_tcp(tcp, Rroll(yaw, tilt, roll), seed=sd, tries=1)
        if q is not None: best.append((margin(q), sn, np.round(q,2)))
    best.sort(key=lambda b: -b[0])
    print(name, tcp, "->", [(round(m,2), sn, list(q)) for m, sn, q in best[:2]] or "NO", flush=True)
EOF
timeout 1200 python3 t_ik8.py 2>&1 | grep -v "^\[" 

# openrua op 71
cat > t_ik9.py <<'EOF'
from rob import *
from flip import R_of
HOME=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
r = Robot(); H = math.pi/2
print("q now", np.round(r.arm_q(),3))
for yaw in (0.0,):
  for x in (-0.15, -0.05, 0.05, 0.15):
    for z in (0.96, 1.05, 1.20):
        q = r.ik_tcp((x, 0.0, z), R_of(yaw, H), seed=HOME, tries=2)
        print(f"+x horiz tcp ({x},0,{z}) -> {'NO' if q is None else (np.round(q,2), round(margin(q),2))}", flush=True)
# hand pointing +y horizontal (wrist at -y side), closing along x
for (x,y,z) in ((-0.077,0.205,1.016), (-0.077,0.10,1.016), (0.0,0.20,1.0)):
    q = r.ik_tcp((x,y,z), R_of(H, H), seed=HOME, tries=2)
    print(f"+y horiz tcp {(x,y,z)} -> {'NO' if q is None else (np.round(q,2), round(margin(q),2))}", flush=True)
EOF
timeout 1200 python3 t_ik9.py 2>&1 | grep -v "^\[" 

# openrua op 72
cat > relay.py <<'EOF'
"""Phase A: re-lay pot B (lying along y, base -y) along x with base toward -x.
Usage: python3 -u relay.py dry|run"""
import sys
from rob import *
from flip import R_of
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
ARCH = (-0.077, 0.205)     # arch-top ridge of lying pot
GZ, LIFT = 1.032, 1.20
NEWX = 0.0                 # TCP x when setting pot down along x (pot body x in [-0.10, 0.06])
STEPS = [("pre", (ARCH[0], ARCH[1], LIFT), 0.0, 4.0, None),
         ("grasp", (ARCH[0], ARCH[1], GZ), 0.0, 3.0, "close"),
         ("lift", (ARCH[0], ARCH[1], LIFT), 0.0, 3.0, None),
         ("yaw45", (ARCH[0], ARCH[1], LIFT), -math.pi/4, 2.5, None),
         ("yaw90", (ARCH[0], ARCH[1], LIFT), -math.pi/2, 2.5, None),
         ("shift", (NEWX, ARCH[1], LIFT), -math.pi/2, 3.0, None),
         ("down", (NEWX, ARCH[1], GZ+0.03), -math.pi/2, 3.0, None),
         ("set", (NEWX, ARCH[1], GZ+0.002), -math.pi/2, 2.0, "open"),
         ("up", (NEWX, ARCH[1], LIFT), -math.pi/2, 3.0, None)]
def main():
    r = Robot(); mode = sys.argv[1]
    print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
    prev = r.arm_q()
    plan = []
    for name, tcp, yaw, sec, act in STEPS:
        q = r.ik_tcp(tcp, grasp_R(yaw, 0), seed=prev)
        if q is None: raise SystemExit(f"IK fail {name}")
        jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
        print(f"{name:6s} {np.round(tcp,3)} yaw {yaw:.2f} -> {np.round(q,2)} margin {margin(q):.2f} jump {jump:.2f}", flush=True)
        if jump > 1.6 and name != "pre": raise SystemExit("branch jump")
        plan.append((name, q, sec, act)); prev = q
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, q, sec, act in plan:
        code, err = r.move_q(q, sec)
        if err > 0.02: print("  retry", flush=True); code, err = r.move_q(q, sec)
        pos, quat, t = r.fk_hand(); print(f"  {name}: tcp {np.round(t,4)}", flush=True)
        if act == "close":
            f = r.gripper(GRIP["closed_m"]); gap = abs(f[0])+abs(f[1]); print(f"  gap {gap*1000:.1f} mm", flush=True)
            if gap < 0.004: print("GRASP FAILED"); r.gripper(GRIP["open_m"]); r.move_q(plan[0][1], 3.0); return
        elif act == "open":
            print("  fingers", r.gripper(GRIP["open_m"]), flush=True)
    print("done", flush=True)
main()
EOF
timeout 900 python3 -u relay.py dry 2>&1 | grep -v "^\["

# openrua op 73
cat > ridge.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > -0.14) & (wx < -0.01) & (wy > 0.08) & (wy < 0.30) & (h > 0.02)
print("pot B pts", m.sum(), "hmax", h[m].max().round(4))
top = m & (h > h[m].max() - 0.012)
print("ridge x range", wx[top].min().round(3), wx[top].max().round(3), "y range", wy[top].min().round(3), wy[top].max().round(3), "mean", wx[top].mean().round(4), wy[top].mean().round(4))
for y0 in np.arange(0.09, 0.28, 0.01):
    s = m & (wy >= y0) & (wy < y0+0.01)
    if s.sum(): print(f"y {y0:.2f}: hmax {h[s].max():.3f} x@max {wx[s][np.argmax(h[s])]:.3f} xrange {wx[s].min():.3f}..{wx[s].max():.3f}")
EOF
timeout 300 python3 ridge.py 2>&1 | grep -v "^\["

# openrua op 74
sed -i 's/^ARCH = (-0.077, 0.205)/ARCH = (-0.076, 0.21)/' relay.py && timeout 1500 python3 -u relay.py run 2>&1 | grep -v "^\[" | tee logs_relay.txt

# openrua op 75
cat > lying3.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > -0.20) & (wx < 0.12) & (wy > 0.12) & (wy < 0.30) & (h > 0.02)
print("pts", m.sum(), "hmax", h[m].max().round(4), "x range", wx[m].min().round(3), wx[m].max().round(3), "y range", wy[m].min().round(3), wy[m].max().round(3))
for x0 in np.arange(-0.14, 0.09, 0.01):
    s = m & (wx >= x0) & (wx < x0+0.01)
    if s.sum(): print(f"x {x0:+.2f}: hmax {h[s].max():.3f} y@max {wy[s][np.argmax(h[s])]:.3f} yrange {wy[s].min():.3f}..{wy[s].max():.3f}")
EOF
timeout 300 python3 lying3.py 2>&1 | grep -v "^\["

# openrua op 76
cat > park2.py <<'EOF'
"""Checked park: python3 park2.py [x y z]  (default -0.25 -0.30 1.25), home orientation; refuses branch jumps."""
import sys
from rob import *
HOME=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
tcp = tuple(map(float, sys.argv[1:4])) if len(sys.argv) >= 4 else (-0.25, -0.30, 1.25)
r = Robot(); q0 = r.arm_q()
q = r.ik_tcp(tcp, grasp_R(-math.pi/2, 0), seed=q0)
if q is None or np.max(np.abs(np.array(q)-np.array(q0))) > 1.8:
    q = r.ik_tcp(tcp, grasp_R(-math.pi/2, 0), seed=HOME)
    if q is None or np.max(np.abs(np.array(q)-np.array(q0))) > 1.8:
        print("no continuous IK; going HOME instead", flush=True); q = HOME
print("target q", np.round(q,2), "jump", np.round(np.max(np.abs(np.array(q)-np.array(q0))),2), flush=True)
code, err = r.move_q(q, 4.0)
if err > 0.02: code, err = r.move_q(q, 4.0)
print("fk", np.round(r.fk_hand()[2],4), flush=True)
EOF
timeout 600 python3 park2.py 2>&1 | grep -v "^\[" && timeout 300 python3 lying3.py 2>&1 | grep -v "^\["

# openrua op 77
cat > arch_prof.py <<'EOF'
from cloud import cloud
import numpy as np
X, Y, Z, _ = cloud('sideview')
m = (X > -0.12) & (X < 0.08) & (Y > 0.16) & (Y < 0.26) & (Z > 0.90) & (Z < 1.10)
print("pts", m.sum())
# for each x bin, list z clusters of points near the pot mid-plane (y within 1.5 cm of axis 0.207)
mm = m & (np.abs(Y - 0.207) < 0.015)
for x0 in np.arange(-0.10, 0.06, 0.01):
    s = mm & (X >= x0) & (X < x0 + 0.01)
    if not s.sum(): continue
    zs = np.sort(Z[s]); gaps = np.where(np.diff(zs) > 0.008)[0]
    segs = np.split(zs, gaps + 1)
    print(f"x {x0:+.2f}: " + "  ".join(f"[{sg.min():.3f}-{sg.max():.3f}]n{len(sg)}" for sg in segs))
EOF
timeout 300 python3 arch_prof.py 2>&1 | grep -v "^\["

# openrua op 78
cat > hook.py <<'EOF'
"""Phase B: hook-lift pot B (lying along x, base face x=-0.10, axis y=0.207, handle up) upright.
Hand points +y, fingers stacked vertically (open): lower finger slides under arch, rides arc about base rim P.
Usage: python3 -u hook.py dry|run"""
import sys
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
R_HOOK = np.array([[-1.0,0,0],[0,0,1.0],[0,1.0,0]])   # hz=+y, closing axis=z, hx=-x
PX, PY, PZ = -0.10, 0.207, 0.899      # base bottom rim (pivot)
R_ARC = 0.1508                        # |arch underside point - P|
F_OFF = 0.04 - 0.004                  # TCP is 4cm above lower finger top; run finger top 4mm inside arc
PHI0, PHI1, N = math.radians(50), math.radians(143), 16
Y_PRE = 0.125
def tcp_at(phi):
    return (PX + R_ARC*math.cos(phi), PY, PZ + R_ARC*math.sin(phi) + F_OFF)
def main():
    r = Robot(); mode = sys.argv[1]
    print("fingers", r.fingers(), "q", np.round(r.arm_q(),3), flush=True)
    t0 = tcp_at(PHI0)
    steps = [("pre", (t0[0], Y_PRE, t0[2]), 4.0), ("insert", t0, 3.0)]
    for k in range(1, N+1):
        steps.append((f"arc{k}", tcp_at(PHI0 + (PHI1-PHI0)*k/N), 1.5))
    t1 = tcp_at(PHI1)
    steps += [("retreat", (t1[0], Y_PRE, t1[2]), 3.0), ("up", (t1[0], Y_PRE, 1.25), 3.0)]
    prev = r.arm_q(); plan = []
    for name, tcp, sec in steps:
        q = r.ik_tcp(tcp, R_HOOK, seed=prev)
        if q is None: raise SystemExit(f"IK fail {name} {tcp}")
        jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
        print(f"{name:8s} {np.round(tcp,4)} -> {np.round(q,2)} margin {margin(q):.2f} jump {jump:.2f}", flush=True)
        if jump > 1.2 and name != "pre": raise SystemExit("branch jump")
        plan.append((name, q, sec)); prev = q
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, q, sec in plan:
        code, err = r.move_q(q, sec)
        if err > 0.02: print("  retry", flush=True); code, err = r.move_q(q, sec)
        pos, quat, t = r.fk_hand(); print(f"  {name}: tcp {np.round(t,4)} err {err:.4f}", flush=True)
        if err > 0.05: print("  LARGE ERROR - stopping", flush=True); return
    print("done", flush=True)
main()
EOF
timeout 900 python3 -u hook.py dry 2>&1 | grep -v "^\["

# openrua op 79
cat > t_ik10.py <<'EOF'
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
HOME=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
r = Robot()
Rs = {"+y rollA": np.array([[-1.0,0,0],[0,0,1.0],[0,1.0,0]]),
      "+y rollB": np.array([[1.0,0,0],[0,0,-1.0],[0,1.0,0]]),
      "-y rollA": np.array([[1.0,0,0],[0,0,1.0],[0,-1.0,0]]),
      "-y rollB": np.array([[-1.0,0,0],[0,0,-1.0],[0,-1.0,0]])}
pts = [(-0.003, 1.0505), (-0.10, 1.086), (-0.22, 1.026)]
seeds = [HOME, r.arm_q(), [0.5,0.8,0.3,-1.8,0.2,2.3,1.5], [-0.8,0.8,0.8,-1.8,-0.5,2.3,-0.5], [0.3,1.0,-0.3,-1.5,0.5,2.0,2.5]]
for name, R in Rs.items():
    print(name, "det", round(np.linalg.det(R),2))
    for x, z in pts:
        y = 0.207 if name.startswith("+y") else 0.207
        best = None
        for sd in seeds:
            q = r.ik_tcp((x, y, z), R, seed=sd, tries=1)
            if q is not None and (best is None or margin(q) > best[0]): best = (margin(q), np.round(q,2))
        print(f"   ({x},{z}) -> {best}", flush=True)
EOF
timeout 1200 python3 t_ik10.py 2>&1 | grep -v "^\["

# openrua op 80
sed -i 's/^R_HOOK = .*/R_HOOK = np.array([[1.0,0,0],[0,0,1.0],[0,-1.0,0]])   # hz=-y, closing axis=z, hx=+x (wrist on +y side)/; s/^Y_PRE = 0.125/Y_PRE = 0.29/' hook.py && sed -i 's/Hand points +y,/Hand points -y (from +y side),/' hook.py && timeout 900 python3 -u hook.py dry 2>&1 | grep -v "^\["

# openrua op 81
python3 - <<'EOF'
s = open('hook.py').read()
s = s.replace('steps = [("pre", (t0[0], Y_PRE, t0[2]), 4.0), ("insert", t0, 3.0)]',
              'steps = [("above", (t0[0], Y_PRE, 1.25), 4.0), ("pre", (t0[0], Y_PRE, t0[2]), 3.0), ("insert", t0, 3.0)]')
s = s.replace('if jump > 1.2 and name != "pre"', 'if jump > 1.2 and name != "above"')
open('hook.py','w').write(s)
EOF
timeout 1500 python3 -u hook.py run 2>&1 | grep -v "^\[" | grep -v "^arc\|^pre\|^above\|^insert\|^retreat\|^up " | tee logs_hook.txt

# openrua op 82
python3 - <<'EOF'
s = open('hook.py').read()
s = s.replace('''        code, err = r.move_q(q, sec)
        if err > 0.02: print("  retry", flush=True); code, err = r.move_q(q, sec)''',
'''        sec = max(sec, 3.0*float(np.max(np.abs(np.array(q)-np.array(r.arm_q())))))
        code, err = r.move_q(q, sec)
        for _ in range(2):
            if err > 0.02: print("  retry", flush=True); code, err = r.move_q(q, max(sec, 3.0))''')
open('hook.py','w').write(s)
EOF
timeout 1500 python3 -u hook.py run 2>&1 | grep -v "^\[" | grep -v "^arc\|^pre\|^above\|^insert\|^retreat\|^up " | tee logs_hook.txt

# openrua op 83
cat > diag.py <<'EOF'
from rob import *
import rclpy
from geometry_msgs.msg import WrenchStamped
r = Robot()
q = r.arm_q(); print("q now", np.round(q,3))
tgt = np.array([0.21, 0.93, -0.11, -1.54, 1.74, 1.6, -0.11])
print("target", tgt); print("diff", np.round(tgt - np.array(q),3))
print("fk", np.round(r.fk_hand()[2],4), "fingers", r.fingers())
got = []
sub = r.node.create_subscription(WrenchStamped, '/franka_robot_state_broadcaster/external_wrench', lambda m: got.append(m), 1)
import time; t=time.time()
while not got and time.time()-t < 5: rclpy.spin_once(r.node, timeout_sec=0.2)
if got: w = got[0].wrench; print("wrench F", round(w.force.x,2), round(w.force.y,2), round(w.force.z,2), "T", round(w.torque.x,2), round(w.torque.y,2), round(w.torque.z,2))
EOF
timeout 120 python3 diag.py 2>&1 | grep -v "^\["

# openrua op 84
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/hook_agent.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/hook_front.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/hook_side.png 2>&1 | tail -1

# openrua op 85
cat > hangmeas.py <<'EOF'
import sys
from rob import *
from cloud import cloud
R_HOOK = np.array([[1.0,0,0],[0,0,1.0],[0,-1.0,0]])
r = Robot()
if len(sys.argv) > 1 and sys.argv[1] == "lift":
    pos, quat, t = r.fk_hand()
    q = r.ik_tcp((t[0], t[1], 1.22), R_HOOK, seed=r.arm_q())
    print("lift q", np.round(q,2), "jump", np.round(np.max(np.abs(np.array(q)-np.array(r.arm_q()))),2))
    code, err = r.move_q(q, 3.0)
    if err > 0.02: code, err = r.move_q(q, 3.0)
pos, quat, t = r.fk_hand(); print("tcp", np.round(t,4), "fingers", r.fingers(), flush=True)
finger_top = t[2] - 0.04
print("lower finger top z", round(finger_top,4), "upper finger bottom z", round(t[2]+0.04,4))
for cam in ("sideview", "frontview"):
    X, Y, Z, _ = cloud(cam)
    m = (X > -0.15) & (X < 0.25) & (Y > 0.05) & (Y < 0.45) & (Z > 0.905) & (Z < t[2] + 0.02)
    # exclude hand: hand body is at y > t[1]+0.03 roughly; pot hangs below/around fingers
    print(cam, "pts", m.sum())
    P = np.stack([X[m], Y[m], Z[m]], 1)
    lo = P[np.argmin(P[:,2])]; print("  lowest point", np.round(lo,3))
    for z0 in np.arange(0.90, t[2]+0.02, 0.02):
        s = m & (Z >= z0) & (Z < z0+0.02)
        if s.sum() > 3:
            print(f"  z {z0:.2f}-{z0+0.02:.2f}: n{s.sum():4d} x {X[s].min():.3f}..{X[s].max():.3f} (mean {X[s].mean():.3f})  y {Y[s].min():.3f}..{Y[s].max():.3f} (mean {Y[s].mean():.3f})")
EOF
timeout 600 python3 hangmeas.py lift 2>&1 | grep -v "^\["

# openrua op 86
timeout 60 python3 tools/perception/cam_snap.py sideview snaps/hang3_side.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/hang3_front.png 2>&1 | tail -1; python3 -c "
import cv2; im=cv2.imread('snaps/hang3_side.png'); cv2.imwrite('snaps/hang3_side_crop.png', cv2.resize(im[120:400, 120:420], None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/hang3_front.png'); cv2.imwrite('snaps/hang3_front_crop.png', cv2.resize(im[100:400, 300:640], None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))"

# openrua op 87
timeout 600 python3 park2.py 2>&1 | grep -v "^\[" && cat > lying4.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > -0.05) & (wx < 0.20) & (wy > 0.05) & (wy < 0.40) & (h > 0.02)
# exclude stove plate (h 0.031) and knob object region
print("pts", m.sum(), "hmax", h[m].max().round(4))
for y0 in np.arange(0.05, 0.36, 0.01):
    s = m & (wy >= y0) & (wy < y0+0.01)
    if s.sum(): print(f"y {y0:.2f}: hmax {h[s].max():.3f} x@max {wx[s][np.argmax(h[s])]:.3f} xrange {wx[s].min():.3f}..{wx[s].max():.3f}  n{s.sum()}")
EOF
timeout 300 python3 lying4.py 2>&1 | grep -v "^\["

# openrua op 88
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/fall_front.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/fall_agent.png 2>&1 | tail -1; python3 -c "
import cv2; im=cv2.imread('snaps/fall_front.png'); print(im.shape); cv2.imwrite('snaps/fall_front_crop.png', cv2.resize(im[250:480, 250:640], None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))"

# openrua op 89
cat > waist.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
h = wz - 0.899
m = (wx > 0.0) & (wx < 0.20) & (wy > 0.13) & (wy < 0.32) & (h > 0.045) & ~((wx > 0.086) & (wy < 0.14))
P = np.stack([wx[m], wy[m]], 1); hh = h[m]
c = P.mean(0); U, S, Vt = np.linalg.svd(P - c); ax = Vt[0]
if ax[1] < 0: ax = -ax          # +a toward +y (base end)
a = (P - c) @ ax; s = (P - c) @ np.array([-ax[1], ax[0]])
print("centroid", np.round(c,4), "axis dir", np.round(ax,3), "angle deg", round(np.degrees(np.arctan2(ax[1], ax[0])),1))
print("a range", a.min().round(3), a.max().round(3))
for a0 in np.arange(a.min(), a.max(), 0.005):
    sl = (a >= a0) & (a < a0+0.005)
    if sl.sum() > 2:
        print(f"a {a0:+.3f}: n{sl.sum():3d} width {s[sl].max()-s[sl].min():.3f} (s {s[sl].min():+.3f}..{s[sl].max():+.3f}) hmax {hh[sl].max():.3f} s@hmax {s[sl][np.argmax(hh[sl])]:+.3f}")
np.save('waist_axis.npy', np.array([c[0], c[1], ax[0], ax[1]]))
EOF
timeout 300 python3 waist.py 2>&1 | grep -v "^\["

# openrua op 90
cat > waist2.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
h = wz - 0.899
c = np.array([0.088, 0.2038]); ax = np.array([0.589, 0.808]); sv = np.array([-ax[1], ax[0]])
P = np.stack([wx, wy], 1); a = (P - c) @ ax; s = (P - c) @ sv
sel = (a > -0.01) & (a < 0.025) & (h > 0.003)
for s0 in np.arange(-0.09, 0.06, 0.01):
    m = sel & (s >= s0) & (s < s0 + 0.01)
    if m.sum(): print(f"s {s0:+.2f}: n{m.sum():3d} h {h[m].min():.3f}..{h[m].max():.3f}")
print("-- table check near (-0.06,0.36), (-0.04,0.25), (0.0,0.40):")
for px, py in [(-0.06, 0.36), (-0.04, 0.25), (0.0, 0.40), (-0.1, 0.38)]:
    m = (np.abs(wx - px) < 0.02) & (np.abs(wy - py) < 0.02)
    print(px, py, "n", m.sum(), "z", (wz[m].min().round(3), wz[m].max().round(3)) if m.sum() else None)
EOF
timeout 120 python3 waist2.py 2>&1 | grep -v "^\["

# openrua op 91
sed -i 's/P = np.stack(\[wx, wy\], 1)/wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = h.ravel(); P = np.stack([wx, wy], 1)/' waist2.py && timeout 120 python3 waist2.py 2>&1 | grep -v "^\["

# openrua op 92
grep -n "def grasp_R\|def xbar_R\|def R_from_q\|def q_from_R\|def ik_tcp\|def move_q\|def fk_hand" -A6 rob.py | head -80

# openrua op 93
cat > stand.py <<'EOF'
"""Stand pot B (lying on table, handle horizontal toward plate) upright on the table.
Waist grasp from above -> lift -> yaw pot 180deg -> pitch 90deg about closing axis (base down)
-> set down at SPOT -> release -> retreat.   Usage: python3 -u stand.py dry|run"""
import sys
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
C = np.array([0.088, 0.2038]); AX = np.array([0.589, 0.808])      # pot axis (+AX -> base end)
G = C + 0.002*AX                                                    # finger centre on the waist
GZ = 0.941; LIFT = 1.20
SPOT = (-0.04, 0.25)                                                # where the pot will stand
SET_Z = 0.899 + 0.073 + 0.005                                       # TCP (7.3 cm above base) + 5 mm drop
def yaw_perp(sign):   # hand yaw so that closing axis hy = Rz(yaw)*x is perpendicular to AX
    hy = sign*np.array([-AX[1], AX[0]]); return math.atan2(hy[1], hy[0])
def main():
    r = Robot(); mode = sys.argv[1]
    q0 = r.arm_q(); print("fingers", r.fingers(), "q", np.round(q0,3), flush=True)
    # pick the grasp yaw whose IK leaves room for a +/-180deg yaw of j7 later
    cands = []
    for sgn in (1, -1):
        psi = yaw_perp(sgn)
        q = r.ik_tcp((G[0], G[1], LIFT), grasp_R(psi, 0), seed=q0)
        if q is not None: cands.append((psi, q)); print(f"cand yaw {psi:+.3f}: q {np.round(q,2)} margin {margin(q):.2f} j7 {q[6]:+.2f}")
    if not cands: raise SystemExit("no IK above pot")
    # for each candidate, decide the direction of the 180 yaw (toward j7 room) and test the pitch
    plan = None
    for psi, qa in cands:
        for dyaw in (math.pi, -math.pi):
            for tilt in (math.pi/2, -math.pi/2):
                R = grasp_R(psi + dyaw, tilt)
                # pot base direction in world after yaw: Rz(dyaw)*AX ; it must map to -z under the tilt.
                # base dir in hand frame at grasp = grasp_R(psi,0)^T [AX,0]; after the motion, world = R @ that
                b_hand = grasp_R(psi, 0).T @ np.array([AX[0], AX[1], 0.0])
                b_world = R @ b_hand
                if b_world[2] > -0.95: continue
                steps = [("pre", (G[0], G[1], LIFT), psi, 0.0, 4.0, None),
                         ("grasp", (G[0], G[1], GZ), psi, 0.0, 3.0, "close"),
                         ("lift", (G[0], G[1], LIFT), psi, 0.0, 3.0, None)]
                for k in (1, 2, 3):
                    steps.append((f"yaw{k}", (G[0], G[1], LIFT), psi + dyaw*k/3, 0.0, 2.5, None))
                for k in (1, 2, 3):
                    steps.append((f"tilt{k}", (G[0], G[1], LIFT), psi + dyaw, tilt*k/3, 2.5, None))
                steps += [("move", (SPOT[0], SPOT[1], 1.10), psi + dyaw, tilt, 4.0, None),
                          ("down", (SPOT[0], SPOT[1], SET_Z + 0.03), psi + dyaw, tilt, 3.0, None),
                          ("set", (SPOT[0], SPOT[1], SET_Z), psi + dyaw, tilt, 2.0, "open")]
                # retreat along -hz then up
                back = -R[:, 2]*0.09
                steps += [("back", (SPOT[0]+back[0], SPOT[1]+back[1], SET_Z), psi + dyaw, tilt, 2.5, None),
                          ("up", (SPOT[0]+back[0], SPOT[1]+back[1], 1.20), psi + dyaw, tilt, 3.0, None)]
                prev = q0; ok = True; out = []
                for name, tcp, yaw, t, sec, act in steps:
                    q = r.ik_tcp(tcp, grasp_R(yaw, t), seed=prev)
                    if q is None: print(f"  [psi {psi:+.2f} dyaw {dyaw:+.2f} tilt {tilt:+.2f}] IK fail at {name}"); ok = False; break
                    jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
                    if (jump > 1.6 and name != "pre") or margin(q) < 0.15:
                        print(f"  [psi {psi:+.2f} dyaw {dyaw:+.2f} tilt {tilt:+.2f}] bad {name}: jump {jump:.2f} margin {margin(q):.2f}"); ok = False; break
                    out.append((name, tcp, yaw, t, q, sec, act, jump)); prev = q
                if ok:
                    print(f"PLAN psi {psi:+.3f} dyaw {dyaw:+.2f} tilt {tilt:+.2f}: hand final hz {np.round(R[:,2],2)} hy {np.round(R[:,1],2)}")
                    for name, tcp, yaw, t, q, sec, act, jump in out:
                        print(f"  {name:6s} {np.round(tcp,3)} yaw {yaw:+.2f} tilt {t:+.2f} -> {np.round(q,2)} m {margin(q):.2f} jump {jump:.2f}")
                    if plan is None: plan = out
    if plan is None: raise SystemExit("no feasible plan")
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, tcp, yaw, t, q, sec, act, jump in plan:
        sec = max(sec, 3.0*jump)
        for attempt in range(3):
            code, err = r.move_q(q, sec)
            if err < 0.02: break
            print("  retry", flush=True)
        pos, quat, tt = r.fk_hand(); print(f"  {name}: tcp {np.round(tt,4)} err {err:.3f}", flush=True)
        if err > 0.05: print("STOP: not converged"); return
        if act == "close":
            f = r.gripper(GRIP["closed_m"]); gap = abs(f[0])+abs(f[1]); print(f"  gap {gap*1000:.1f} mm", flush=True)
            if gap < 0.04 or gap > 0.078: print("GRASP BAD"); r.gripper(GRIP["open_m"]); r.move_q(plan[0][4], 3.0); return
        elif act == "open":
            print("  fingers", r.gripper(GRIP["open_m"]), flush=True)
    print("done", flush=True)
main()
EOF
timeout 600 python3 -u stand.py dry 2>&1 | grep -v "^\[" | tee logs_stand_dry.txt

# openrua op 94
python3 -c "
from rob import *
AX=np.array([0.589,0.808]); psi=-0.630
for dyaw in (math.pi,-math.pi):
  for tilt in (math.pi/2,-math.pi/2):
    R=grasp_R(psi+dyaw,tilt); b=grasp_R(psi,0).T@np.array([AX[0],AX[1],0.]); print(dyaw,tilt,'b_world',np.round(R@b,2),'hz',np.round(R[:,2],2),'hx',np.round(R[:,0],2))
print(grasp_R(psi,0).round(2))
"

# openrua op 95
python3 - <<'EOF'
import re
s = open('stand.py').read()
s = s.replace("def yaw_perp(sign):", "def hand_R(yaw, t):\n    c, s = math.cos(t), math.sin(t)\n    return grasp_R(yaw, 0) @ np.array([[c,0,s],[0,1,0],[-s,0,c]])   # rotate about the hand's own y (closing axis)\ndef yaw_perp(sign):")
s = s.replace("R = grasp_R(psi + dyaw, tilt)", "R = hand_R(psi + dyaw, tilt)")
s = s.replace("q = r.ik_tcp(tcp, grasp_R(yaw, t), seed=prev)", "q = r.ik_tcp(tcp, hand_R(yaw, t), seed=prev)")
open('stand.py','w').write(s)
EOF
timeout 900 python3 -u stand.py dry 2>&1 | grep -v "^\[" | tee logs_stand_dry.txt

# openrua op 96
python3 - <<'EOF'
s = open('stand.py').read()
s = s.replace("for dyaw in (math.pi, -math.pi):", "for dyaw in DYAWS:")
s = s.replace("SPOT = (-0.04, 0.25)", "SPOT = tuple(map(float, sys.argv[2:4])) if len(sys.argv) >= 4 else (-0.05, 0.25)\nDYAWS = [math.radians(d) for d in ([float(sys.argv[4])] if len(sys.argv) >= 5 else (-100, -90, -110, -80))]")
s = s.replace("                    if plan is None: plan = out", "                    if plan is None: plan = out\n                    break\n            if plan is not None: break\n        if plan is not None: break")
open('stand.py','w').write(s)
EOF
for spot in "-0.05 0.25" "-0.08 0.22" "-0.02 0.28"; do echo "=== SPOT $spot"; timeout 900 python3 -u stand.py dry $spot 2>&1 | grep -v "^\[" | grep -v "^cand\|^fingers"; done | tee logs_stand_dry.txt

# openrua op 97
python3 - <<'EOF'
s = open('stand.py').read()
old = """                for k in (1, 2, 3):
                    steps.append((f"tilt{k}", (G[0], G[1], LIFT), psi + dyaw, tilt*k/3, 2.5, None))
                steps += [("move", (SPOT[0], SPOT[1], 1.10), psi + dyaw, tilt, 4.0, None),
"""
new = """                steps.append(("move", (SPOT[0], SPOT[1], TILT_Z), psi + dyaw, 0.0, 4.0, None))
                for k in (1, 2, 3):
                    steps.append((f"tilt{k}", (SPOT[0], SPOT[1], TILT_Z), psi + dyaw, tilt*k/3, 2.5, None))
                steps += [("move2", (SPOT[0], SPOT[1], 1.10), psi + dyaw, tilt, 3.0, None),
"""
assert old in s
s = s.replace(old, new).replace("GZ = 0.941; LIFT = 1.20", "GZ = 0.941; LIFT = 1.20; TILT_Z = 1.15")
open('stand.py','w').write(s)
EOF
for spot in "-0.05 0.25" "-0.10 0.22" "-0.15 0.25"; do echo "=== SPOT $spot"; timeout 900 python3 -u stand.py dry $spot 2>&1 | grep -v "^\[" | grep -v "^cand\|^fingers\|psi +2.51"; done | tee logs_stand_dry.txt

# openrua op 98
cat > root.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
c = np.array([0.088, 0.2038]); ax = np.array([0.589, 0.808]); sv = np.array([-ax[1], ax[0]])
P = np.stack([wx, wy], 1); a = (P - c) @ ax; s = (P - c) @ sv
for s0 in np.arange(-0.11, -0.03, 0.01):
    m = (s >= s0) & (s < s0+0.01) & (h > 0.036) & (h < 0.06) & (a > -0.1) & (a < 0.1)
    if m.sum():
        aa = np.sort(a[m]); print(f"s {s0:+.2f}: n{m.sum():3d} a range {aa.min():+.3f}..{aa.max():+.3f}  a-hist", np.histogram(aa, bins=np.arange(-0.09,0.09,0.01))[0])
EOF
timeout 60 python3 root.py 2>&1 | grep -v "^\["

# openrua op 99
cat > col.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
c = np.array([0.088, 0.2038]); ax = np.array([0.589, 0.808]); sv = np.array([-ax[1], ax[0]])
P = np.stack([wx, wy], 1); a = (P - c) @ ax; s = (P - c) @ sv
for lo, hi, lab in [(-0.048, -0.032, "handle-side finger column"), (0.032, 0.048, "far-side finger column")]:
    m = (s >= lo) & (s < hi) & (h > 0.02) & (a > -0.1) & (a < 0.1)
    print(lab)
    for a0 in np.arange(-0.09, 0.08, 0.01):
        mm = m & (a >= a0) & (a < a0 + 0.01)
        if mm.sum(): print(f"  a {a0:+.2f}: n{mm.sum():2d} h {h[mm].min():.3f}..{h[mm].max():.3f}  s@hmax {s[mm][np.argmax(h[mm])]:+.3f}")
EOF
timeout 60 python3 col.py 2>&1 | grep -v "^\["

# openrua op 100
sed -i 's/^G = C + 0.002\*AX.*/G = C + 0.0*AX                                                      # finger centre on the waist/' stand.py && timeout 1500 python3 -u stand.py run -0.15 0.25 -100 2>&1 | grep -v "^\[" | tee logs_stand.txt

# openrua op 101
timeout 300 python3 -u park2.py 2>&1 | grep -v "^\[" | tail -3; timeout 120 python3 tools/perception/cam_snap.py frontview snaps/stand_front.png >/dev/null 2>&1; timeout 120 python3 tools/perception/cam_snap.py sideview snaps/stand_side.png > /dev/null 2>&1; timeout 120 python3 tools/perception/cam_snap.py birdview snaps/stand_bird.png >/dev/null 2>&1; ls snaps | grep stand

# openrua op 102
cat > upright.py <<'EOF'
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=True)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
for lab, box in [("B", (-0.30, 0.0, 0.10, 0.40)), ("A", (0.10, -0.10, 0.30, 0.10))]:
    m = (wx > box[0]) & (wy > box[1]) & (wx < box[2]) & (wy < box[3]) & (h > 0.02) & ~((wx > 0.086) & (wy < 0.14) & (h < 0.035))
    top = m & (h > h[m].max() - 0.03)
    print(lab, "n", m.sum(), "hmax", h[m].max().round(3), "top(>hmax-3cm) centre", wx[top].mean().round(4), wy[top].mean().round(4), "top extent x", wx[top].min().round(3), wx[top].max().round(3), "y", wy[top].min().round(3), wy[top].max().round(3))
    body = m & (h > 0.10)
    print("   body(h>0.10) centre", wx[body].mean().round(4), wy[body].mean().round(4))
    low = m & (h > 0.06) & (h < 0.13)
    cx, cy = wx[body].mean(), wy[body].mean()
    d = np.hypot(wx[low]-cx, wy[low]-cy); far = d > 0.05
    if far.sum(): print("   handle pts (r>5cm, 6<h<13):", far.sum(), "dir", np.round(np.arctan2((wy[low][far]-cy).mean(), (wx[low][far]-cx).mean())*180/np.pi,1), "deg, r", d[far].max().round(3))
EOF
timeout 300 python3 upright.py 2>&1 | grep -v "^\["

# openrua op 103
python3 - <<'EOF' 2>&1 | grep -v "^\["
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
m = (h > 0.09) & (h < 0.30) & np.isfinite(h)
print("n high pts", m.sum())
# cluster by 5cm grid
import collections
cnt = collections.Counter(zip(np.round(wx[m]/0.05).astype(int), np.round(wy[m]/0.05).astype(int)))
for k, v in sorted(cnt.items(), key=lambda kv: -kv[1])[:12]:
    mm = m & (np.round(wx/0.05).astype(int) == k[0]) & (np.round(wy/0.05).astype(int) == k[1])
    print(k, v, "centre", wx[mm].mean().round(3), wy[mm].mean().round(3), "hmax", h[mm].max().round(3))
EOF

# openrua op 104
sed -i 's/("B", (-0.30, 0.0, 0.10, 0.40))/("B", (-0.30, -0.25, 0.0, 0.05))/' upright.py && timeout 120 python3 upright.py 2>&1 | grep -v "^\[\|Warning\|ret = "

# openrua op 105
python3 - <<'EOF' 2>&1 | grep -v "^\["
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
m = (wx > -0.22) & (wx < -0.04) & (wy > -0.16) & (wy < 0.0) & (h > 0.02) & (h < 0.20)
print("n", m.sum(), "hmax", h[m].max().round(3))
body = m & (h > 0.10); cx, cy = wx[body].mean(), wy[body].mean()
print("body centre (h>0.10)", round(cx,4), round(cy,4), "n", body.sum())
for lo, hi in [(0.13,0.20),(0.10,0.13),(0.06,0.10),(0.02,0.06)]:
    mm = m & (h > lo) & (h <= hi)
    if mm.sum():
        d = np.hypot(wx[mm]-cx, wy[mm]-cy)
        print(f"h {lo}-{hi}: n{mm.sum()} rmax {d.max():.3f} centre ({wx[mm].mean():.4f},{wy[mm].mean():.4f})")
        far = d > 0.045
        if far.sum(): print("   far pts", far.sum(), "mean dir deg", round(np.degrees(np.arctan2((wy[mm][far]-cy).mean(), (wx[mm][far]-cx).mean())),1), "r", d[far].min().round(3), d[far].max().round(3), "h", h[mm][far].min().round(3), h[mm][far].max().round(3))
EOF

# openrua op 106
python3 - <<'EOF' 2>&1 | grep -v "^\["
from potpose import measure
import numpy as np
wx, wy, wz = measure(fresh=False)
wx, wy, wz = wx.ravel(), wy.ravel(), wz.ravel(); h = wz - 0.899
m = (wx > -0.22) & (wx < -0.04) & (wy > -0.16) & (wy < 0.0)
for lo in (0.135, 0.14, 0.145, 0.15):
    mm = m & (h > lo)
    print(f"h>{lo}: n{mm.sum()} centre ({wx[mm].mean():.4f},{wy[mm].mean():.4f}) extent x {wx[mm].min():.3f}..{wx[mm].max():.3f} y {wy[mm].min():.3f}..{wy[mm].max():.3f}")
# circle fit to rim points h in [0.13,0.145] excluding handle side
mm = m & (h > 0.13) & (h < 0.145)
X, Y = wx[mm], wy[mm]
A = np.column_stack([2*X, 2*Y, np.ones_like(X)]); b = X**2 + Y**2
sol = np.linalg.lstsq(A, b, rcond=None)[0]; cx, cy = sol[0], sol[1]; R = np.sqrt(sol[2] + cx**2 + cy**2)
print("circle fit rim: centre", round(cx,4), round(cy,4), "R", round(R,4))
# handle bar: points at r 0.09-0.12 from centre
d = np.hypot(wx - cx, wy - cy); hb = m & (h > 0.06) & (h < 0.14) & (d > 0.085) & (d < 0.125)
print("bar candidates", hb.sum(), "dir", np.degrees(np.arctan2(wy[hb]-cy, wx[hb]-cx)).round(1) if hb.sum() < 15 else np.degrees(np.arctan2((wy[hb]-cy).mean(), (wx[hb]-cx).mean())).round(1), "r", d[hb].round(3) if hb.sum() < 15 else (d[hb].min().round(3), d[hb].max().round(3)), "h", h[hb].round(3) if hb.sum() < 15 else (h[hb].min().round(3), h[hb].max().round(3)))
EOF

# openrua op 107
grep -n "def cloud" -A3 cloud.py | head; python3 - <<'EOF' 2>&1 | grep -v "^\["
from cloud import cloud
import numpy as np
cx, cy = -0.135, -0.076; phi = np.radians(-145.0); u = np.array([np.cos(phi), np.sin(phi)])
for cam in ("sideview", "frontview", "agentview"):
    try:
        X, Y, Z, _ = cloud(cam)
    except Exception as e:
        print(cam, "fail", e); continue
    X, Y, Z = X.ravel(), Y.ravel(), Z.ravel()
    m = np.isfinite(Z) & (np.hypot(X-cx, Y-cy) < 0.16) & (Z > 0.905) & (Z < 1.07)
    r_rad = (X[m]-cx)*u[0] + (Y[m]-cy)*u[1]; r_tan = -(X[m]-cx)*u[1] + (Y[m]-cy)*u[0]
    print(f"== {cam}: n{m.sum()}")
    for z0 in np.arange(0.90, 1.07, 0.01):
        mm = (Z[m] >= z0) & (Z[m] < z0+0.01)
        if mm.sum():
            rr = r_rad[mm]; print(f"  z {z0:.2f}: n{mm.sum():4d} radial {rr.min():+.3f}..{rr.max():+.3f}  tangential {r_tan[mm].min():+.3f}..{r_tan[mm].max():+.3f}  pts with radial>0.06: {np.sum(rr>0.06)} (r {rr[rr>0.06].min():.3f}..{rr[rr>0.06].max():.3f})" if np.sum(rr>0.06) else f"  z {z0:.2f}: n{mm.sum():4d} radial {rr.min():+.3f}..{rr.max():+.3f}  tangential {r_tan[mm].min():+.3f}..{r_tan[mm].max():+.3f}")
EOF

# openrua op 108
cat > carry.py <<'EOF'
"""Carry upright pot B from the table to the stove with a pitched waist grasp.
Usage: python3 -u carry.py dry|run [yaw_final_deg]"""
import sys
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
B = np.array([-0.135, -0.076]); PHI0 = math.radians(-145.0)      # pot B axis (table) and handle direction
TARGET = np.array([0.19, 0.10])                                   # pot B centre on the stove
PHI1 = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else math.radians(-15.0)   # handle direction at the stove
T = 1.3                                                           # hand pitch (rad) about the tangential closing axis
WAIST_T = 0.899 + 0.0625; WAIST_S = 0.93 + 0.0625 + 0.003
LIFT = 1.20
A_BAR = (np.array([0.150, -0.0045, 0.996]), np.array([0.150, -0.0045, 1.056]))   # pot A handle bar segment (approx)
def R_pitched(phi, sgn=1):
    u = np.array([math.cos(phi), math.sin(phi), 0.0]); tan = np.array([-u[1], u[0], 0.0])*sgn
    hz = math.sin(T)*u - math.cos(T)*np.array([0,0,1.0]); hy = tan; hx = np.cross(hy, hz)
    return np.column_stack([hx, hy, hz])
def body_clearance(tcp, R):
    """min distance from hand-body box (palm..wrist) to pot A bar segment"""
    hx, hy, hz = R[:,0], R[:,1], R[:,2]; best = 9
    for d in np.linspace(0.058, 0.103, 4):
        for a in (-0.035, 0.035):
            for b in np.linspace(-0.10, 0.10, 9):
                p = np.array(tcp) - d*hz + a*hx + b*hy
                s0, s1 = A_BAR; v = s1-s0; t = np.clip(np.dot(p-s0, v)/np.dot(v,v), 0, 1)
                best = min(best, np.linalg.norm(p-(s0+t*v)))
    return best
def main():
    r = Robot(); mode = sys.argv[1]
    q0 = r.arm_q(); print("fingers", r.fingers(), "q", np.round(q0,3), flush=True)
    plan = None
    for sgn in (1, -1):
        R0 = R_pitched(PHI0, sgn); R1 = R_pitched(PHI1, sgn)
        g = np.array([B[0], B[1], WAIST_T]); pre = g - 0.08*R0[:,2]
        steps = [("pre", pre, PHI0, 4.0, None), ("grasp", g, PHI0, 3.0, "close"),
                 ("lift", (g[0], g[1], LIFT), PHI0, 3.0, None)]
        n = 4
        for k in range(1, n+1):
            steps.append((f"yaw{k}", (g[0], g[1], LIFT), PHI0 + (PHI1-PHI0)*k/n, 2.5, None))
        s = np.array([TARGET[0], TARGET[1], WAIST_S])
        steps += [("over", (s[0], s[1], LIFT), PHI1, 4.0, None), ("down", (s[0], s[1], WAIST_S+0.03), PHI1, 3.0, None),
                  ("set", s, PHI1, 2.5, "open")]
        back = s - 0.08*R1[:,2]
        steps += [("back", back, PHI1, 2.5, None), ("up", (back[0], back[1], 1.25), PHI1, 3.0, None)]
        prev = q0; out = []; ok = True
        for name, tcp, phi, sec, act in steps:
            R = R_pitched(phi, sgn)
            q = r.ik_tcp(tcp, R, seed=prev)
            if q is None: print(f"  sgn {sgn}: IK fail {name}"); ok = False; break
            jump = float(np.max(np.abs(np.array(q)-np.array(prev))))
            if (jump > 1.6 and name != "pre") or margin(q) < 0.12:
                print(f"  sgn {sgn}: bad {name} jump {jump:.2f} margin {margin(q):.2f}"); ok = False; break
            out.append((name, np.array(tcp), R, q, sec, act, jump)); prev = q
        if ok:
            print(f"PLAN sgn {sgn}; final hz {np.round(R1[:,2],2)} hy {np.round(R1[:,1],2)}; hand-body clearance to A bar at set: {body_clearance(s, R1)*100:.1f} cm, at down: {body_clearance((s[0],s[1],WAIST_S+0.03), R1)*100:.1f} cm")
            for name, tcp, R, q, sec, act, jump in out:
                print(f"  {name:6s} {np.round(tcp,3)} -> {np.round(q,2)} m {margin(q):.2f} jump {jump:.2f}")
            plan = out; break
    if plan is None: raise SystemExit("no plan")
    if mode != "run": return
    r.gripper(GRIP["open_m"])
    for name, tcp, R, q, sec, act, jump in plan:
        sec = max(sec, 3.0*jump)
        for attempt in range(3):
            code, err = r.move_q(q, sec)
            if err < 0.02: break
            print("  retry", flush=True)
        pos, quat, tt = r.fk_hand(); print(f"  {name}: tcp {np.round(tt,4)} err {err:.3f}", flush=True)
        if err > 0.05: print("STOP: not converged"); return
        if act == "close":
            f = r.gripper(GRIP["closed_m"]); gap = abs(f[0])+abs(f[1]); print(f"  gap {gap*1000:.1f} mm", flush=True)
            if gap < 0.04 or gap > 0.078: print("GRASP BAD"); r.gripper(GRIP["open_m"]); r.move_q(plan[0][3], 3.0); return
        elif act == "open":
            print("  fingers", r.gripper(GRIP["open_m"]), flush=True)
    print("done", flush=True)
main()
EOF
for yf in -15 -30 0; do echo "=== PHI1 $yf"; timeout 600 python3 -u carry.py dry $yf 2>&1 | grep -v "^\[\|^fingers"; done | tee logs_carry_dry.txt

# openrua op 109
cat > scan_carry.py <<'EOF'
from carry import *  # noqa  (module guards? no) 
EOF
python3 - <<'EOF' 2>&1 | grep -v "^\["
import math, numpy as np
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
def R_pitched(phi, T, sgn=1):
    u = np.array([math.cos(phi), math.sin(phi), 0.0]); tan = np.array([-u[1], u[0], 0.0])*sgn
    hz = math.sin(T)*u - math.cos(T)*np.array([0,0,1.0]); hy = tan; hx = np.cross(hy, hz)
    return np.column_stack([hx, hy, hz])
r = Robot(); q0 = r.arm_q()
print("PICK at (-0.135,-0.076,0.9615)")
for T in (1.3, 1.2):
    for deg in range(-215, -70, 15):
        best = None
        for sgn in (1,-1):
            q = r.ik_tcp((-0.135,-0.076,0.9615), R_pitched(math.radians(deg), T, sgn), seed=q0)
            if q is not None and (best is None or margin(q) > best[0]): best = (margin(q), sgn, np.round(q,2))
        print(f"  T {T} pitch {deg:+4d}: ", "none" if best is None else f"margin {best[0]:.2f} sgn {best[1]} q {best[2]}")
print("STOVE at (0.19,0.10,0.9955)")
for T in (1.3, 1.2):
    for deg in range(-90, 91, 15):
        best = None
        for sgn in (1,-1):
            q = r.ik_tcp((0.19,0.10,0.9955), R_pitched(math.radians(deg), T, sgn), seed=q0)
            if q is not None and (best is None or margin(q) > best[0]): best = (margin(q), sgn, np.round(q,2))
        print(f"  T {T} pitch {deg:+4d}: ", "none" if best is None else f"margin {best[0]:.2f} sgn {best[1]} q {best[2]}")
EOF

# openrua op 110
python3 - <<'EOF'
s = open('carry.py').read()
s = s.replace("B = np.array([-0.135, -0.076]); PHI0 = math.radians(-145.0)      # pot B axis (table) and handle direction",
 "B = np.array([-0.135, -0.076]); HANDLE0 = math.radians(-145.0)   # pot B axis (table) and handle direction\nPHI0 = math.radians(-170.0)                                       # pitch direction at pick (palm on opposite side)")
s = s.replace("PHI1 = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else math.radians(-15.0)   # handle direction at the stove",
              "PHI1 = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else math.radians(-45.0)   # pitch direction at the stove")
s = s.replace("WAIST_T = 0.899 + 0.0625; WAIST_S = 0.93 + 0.0625 + 0.003", "WAIST_T = 0.899 + 0.0675; WAIST_S = 0.93 + 0.0675 + 0.003")
old = """        g = np.array([B[0], B[1], WAIST_T]); pre = g - 0.08*R0[:,2]
        steps = [("pre", pre, PHI0, 4.0, None), ("grasp", g, PHI0, 3.0, "close"),
                 ("lift", (g[0], g[1], LIFT), PHI0, 3.0, None)]
        n = 4
        for k in range(1, n+1):
            steps.append((f"yaw{k}", (g[0], g[1], LIFT), PHI0 + (PHI1-PHI0)*k/n, 2.5, None))
"""
new = """        g = np.array([B[0], B[1], WAIST_T]); p0 = np.array([math.cos(PHI0), math.sin(PHI0), 0.0])
        pre = g - 0.08*p0 + np.array([0,0,0.01])
        steps = [("high", pre + np.array([0,0,0.12]), PHI0, 4.0, None), ("pre", pre, PHI0, 3.0, None),
                 ("slide", g + np.array([0,0,0.01]), PHI0, 3.0, None), ("grasp", g, PHI0, 2.0, "close"),
                 ("lift", (g[0], g[1], LIFT), PHI0, 3.0, None)]
        n = 5
        for k in range(1, n+1):
            steps.append((f"yaw{k}", (g[0], g[1], LIFT), PHI0 + (PHI1-PHI0)*k/n, 2.5, None))
"""
assert old in s; s = s.replace(old, new)
old2 = """        back = s - 0.08*R1[:,2]
"""
new2 = """        p1 = np.array([math.cos(PHI1), math.sin(PHI1), 0.0]); back = s - 0.08*p1 + np.array([0,0,0.02])
        print(f"  handle at stove -> {math.degrees(HANDLE0 + PHI1 - PHI0):.0f} deg; bar at {np.round(TARGET + 0.075*np.array([math.cos(HANDLE0+PHI1-PHI0), math.sin(HANDLE0+PHI1-PHI0)]),3)}")
"""
assert old2 in s; s = s.replace(old2, new2)
open('carry.py','w').write(s)
EOF
timeout 900 python3 -u carry.py dry 2>&1 | grep -v "^\[" | tee logs_carry_dry.txt

# openrua op 111
python3 - <<'EOF' 2>&1 | grep -v "^\["
import math, numpy as np
from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
def R_pitched(phi, T, sgn=1):
    u = np.array([math.cos(phi), math.sin(phi), 0.0]); tan = np.array([-u[1], u[0], 0.0])*sgn
    hz = math.sin(T)*u - math.cos(T)*np.array([0,0,1.0]); hy = tan; hx = np.cross(hy, hz)
    return np.column_stack([hx, hy, hz])
r = Robot(); q0 = r.arm_q()
for tcp in [(-0.056,-0.062,1.0965),(-0.056,-0.062,1.05),(-0.056,-0.062,1.02),(-0.10,-0.15,1.05),(-0.10,-0.20,1.10),(-0.056,-0.062,0.9765),(-0.135,-0.076,1.20),(-0.135,-0.076,1.10)]:
    for T in (1.3,):
        best=None
        for sgn in (1,-1):
            q = r.ik_tcp(tcp, R_pitched(math.radians(-170), T, sgn), seed=q0)
            if q is not None and (best is None or margin(q)>best[0]): best=(margin(q),sgn,np.round(q,2))
        print(tcp, T, "none" if best is None else f"margin {best[0]:.2f} sgn {best[1]} q {best[2]}")
EOF

# openrua op 112
python3 - <<'EOF'
s = open('carry.py').read()
old = """        pre = g - 0.08*p0 + np.array([0,0,0.01])
        steps = [("high", pre + np.array([0,0,0.12]), PHI0, 4.0, None), ("pre", pre, PHI0, 3.0, None),
                 ("slide", g + np.array([0,0,0.01]), PHI0, 3.0, None), ("grasp", g, PHI0, 2.0, "close"),
"""
new = """        steps = [("high", (g[0], g[1], LIFT), PHI0, 4.0, None), ("pre", g + np.array([0,0,0.10]), PHI0, 3.0, None),
                 ("mid", g + np.array([0,0,0.03]), PHI0, 3.0, None), ("grasp", g, PHI0, 2.0, "close"),
"""
assert old in s; s = s.replace(old, new)
open('carry.py','w').write(s)
EOF
timeout 900 python3 -u carry.py dry 2>&1 | grep -v "^\[" | tee logs_carry_dry.txt

# openrua op 113
python3 - <<'EOF'
s = open('carry.py').read()
old = """        steps += [("over", (s[0], s[1], LIFT), PHI1, 4.0, None),"""
new = """        for k, (wx_, wy_) in enumerate([(-0.03, -0.02), (0.08, 0.04)]):
            steps.append((f"way{k+1}", (wx_, wy_, 1.22), PHI1, 3.0, None))
        steps += [("over", (s[0], s[1], LIFT), PHI1, 3.0, None),"""
assert old in s; s = s.replace(old, new)
open('carry.py','w').write(s)
EOF
timeout 900 python3 -u carry.py dry 2>&1 | grep -v "^\[" | grep -A30 PLAN | tee logs_carry_dry.txt

# openrua op 114
timeout 1500 python3 -u carry.py run 2>&1 | grep -v "^\[" | grep -v "^  [a-z0-9]* *\[.*-> \[" | tee logs_carry.txt

# openrua op 115
timeout 120 python3 tools/perception/cam_snap.py frontview snaps/carry_stall_front.png >/dev/null 2>&1; timeout 120 python3 tools/perception/cam_snap.py sideview snaps/carry_stall_side.png >/dev/null 2>&1; python3 - <<'EOF' 2>&1 | grep -v "^\["
from rob import *
r = Robot(); pos, quat, t = r.fk_hand(); print("tcp", np.round(t,4), "fingers", r.fingers())
q = r.arm_q(); import math
# lift 8 cm keeping orientation
R = R_from_q(quat.x, quat.y, quat.z, quat.w) if hasattr(quat,'x') else R_from_q(*quat)
q2 = r.ik_tcp(t + np.array([0,0,0.08]), R, seed=q)
print(r.move_q(q2, 2.5)); print("tcp now", np.round(r.fk_hand()[2],4))
EOF

# openrua op 116
python3 -c "
import cv2
im = cv2.imread('snaps/carry_stall_front.png'); h,w = im.shape[:2]
cv2.imwrite('snaps/carry_stall_front_crop.png', cv2.resize(im[150:400, 150:450], None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('snaps/carry_stall_side.png')
cv2.imwrite('snaps/carry_stall_side_crop.png', cv2.resize(im[120:400, 200:560], None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))
"

# openrua op 117
python3 - <<'EOF' 2>&1 | grep -v "^\["
from rob import *
r = Robot(); pos, quat, t = r.fk_hand()
R = R_from_q(quat.x, quat.y, quat.z, quat.w) if hasattr(quat,'x') else R_from_q(*quat)
print("hand pos", np.round(pos,4), "tcp", np.round(t,4)); print("hx", np.round(R[:,0],3), "hy", np.round(R[:,1],3), "hz", np.round(R[:,2],3))
EOF

# openrua op 118
python3 - <<'EOF'
s = open('carry.py').read()
s = s.replace("PHI0 = math.radians(-170.0)", "PHI0 = math.radians(-95.0)")
s = s.replace("TARGET = np.array([0.19, 0.10])", "TARGET = np.array([0.14, 0.10])")
old = """        steps = [("high", (g[0], g[1], LIFT), PHI0, 4.0, None), ("pre", g + np.array([0,0,0.10]), PHI0, 3.0, None),
                 ("mid", g + np.array([0,0,0.03]), PHI0, 3.0, None), ("grasp", g, PHI0, 2.0, "close"),
"""
new = """        pre = g - 0.08*p0 + np.array([0,0,0.01])
        steps = [("high", (pre[0], pre[1], 1.15), PHI0, 4.0, None), ("pre", pre, PHI0, 3.0, None),
                 ("slide", g + np.array([0,0,0.01]), PHI0, 3.0, None), ("grasp", g, PHI0, 2.0, "close"),
"""
assert old in s; s = s.replace(old, new)
s = s.replace("for k, (wx_, wy_) in enumerate([(-0.03, -0.02), (0.08, 0.04)]):", "for k, (wx_, wy_) in enumerate([(-0.05, -0.02), (0.05, 0.04)]):")
open('carry.py','w').write(s)
EOF
timeout 900 python3 -u carry.py dry 2>&1 | grep -v "^\[" | grep -B2 -A30 "PLAN\|fail\|bad" | tee logs_carry_dry.txt

# openrua op 119
sed -i 's/TARGET = np.array(\[0.14, 0.10\])/TARGET = np.array([0.13, 0.10])/' carry.py && timeout 1700 python3 -u carry.py run 2>&1 | grep -v "^\[" | tee logs_carry.txt | grep -v "^  [a-z0-9]* *\[.*-> \["

# openrua op 120
ls snaps | head -30; grep -n "def \|argv" potpose.py | head -20; ls *.py

# openrua op 121
grep -rl "carry_stall_front\|def snap\|imwrite" *.py | head; grep -n "def " cloud.py

# openrua op 122
cat > where.py <<'EOF'
"""Locate objects above the table from a camera cloud: cluster points with z>0.94 by xy grid."""
import sys, numpy as np
from cloud import cloud
cam = sys.argv[1] if len(sys.argv) > 1 else "sideview"
X, Y, Z, _ = cloud(cam)
m = (Z > 0.945) & (Z < 1.30) & (X > -0.45) & (X < 0.45) & (Y > -0.45) & (Y < 0.45)
X, Y, Z = X[m], Y[m], Z[m]
# 2 cm grid occupancy summary
gx = np.round(X / 0.02).astype(int); gy = np.round(Y / 0.02).astype(int)
cells = {}
for a, b, z in zip(gx, gy, Z):
    k = (a, b); cells.setdefault(k, []).append(z)
rows = sorted(((k[0]*0.02, k[1]*0.02, len(v), max(v)) for k, v in cells.items()), key=lambda r: (-r[2]))
print("cells with >=5 points (x, y, n, zmax):")
for r in rows:
    if r[2] >= 5: print("  %.2f %.2f  n=%3d  zmax=%.3f" % r)
EOF
python3 where.py sideview 2>&1 | grep -v "^\[" | head -80; python3 -c "
from rob import *; r=Robot(); print('fingers', r.fingers()); print('tcp', r.fk_hand()[2])" 2>&1 | grep -v "^\["

# openrua op 123
timeout 600 python3 -u park2.py 2>&1 | grep -v "^\[" | tail -8

# openrua op 124
cat > final_check.py <<'EOF'
"""Birdview check: find the two pots, report apex, footprint, and whether they stand on the stove plate."""
import numpy as np
from potpose import measure
wx, wy, wz = [a.ravel() for a in measure(True)]
ok = np.isfinite(wz)
wx, wy, wz = wx[ok], wy[ok], wz[ok]
PLATE = dict(x=(0.086, 0.276), y=(-0.047, 0.140), top=0.93)
scene = (wz > 0.95) & (wx > -0.45) & (wx < 0.45) & (wy > -0.45) & (wy < 0.45)
print("points above 0.95:", scene.sum())
# cluster by simple flood on a 1 cm grid
g = {}
for x, y, z in zip(wx[scene], wy[scene], wz[scene]):
    g.setdefault((round(x/0.01), round(y/0.01)), []).append(z)
seen = set(); clusters = []
for k in g:
    if k in seen: continue
    stack = [k]; comp = []
    while stack:
        c = stack.pop()
        if c in seen: continue
        seen.add(c); comp.append(c)
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                n = (c[0]+dx, c[1]+dy)
                if n in g and n not in seen: stack.append(n)
    clusters.append(comp)
for comp in sorted(clusters, key=len, reverse=True):
    if len(comp) < 20: continue
    xs = np.array([c[0]*0.01 for c in comp]); ys = np.array([c[1]*0.01 for c in comp])
    zs = np.array([max(g[c]) for c in comp])
    top = zs.max(); apex = np.array([xs[zs > top-0.01].mean(), ys[zs > top-0.01].mean()])
    body = zs > 1.0  # lid/upper chamber region
    cx, cy = xs[body].mean(), ys[body].mean()
    ext_x = (xs[body].min(), xs[body].max()); ext_y = (ys[body].min(), ys[body].max())
    on = PLATE["x"][0] < cx-0.035 and cx+0.035 < PLATE["x"][1] and PLATE["y"][0] < cy-0.035 and cy+0.035 < PLATE["y"][1]
    print(f"cluster n={len(comp)} top z={top:.3f} apex=({apex[0]:.3f},{apex[1]:.3f}) body centre=({cx:.3f},{cy:.3f}) "
          f"x[{ext_x[0]:.2f},{ext_x[1]:.2f}] y[{ext_y[0]:.2f},{ext_y[1]:.2f}]  upright={abs(top-1.084)<0.01}  base within plate={on}")
EOF
python3 final_check.py 2>&1 | grep -v "^\["

# openrua op 125
cat > rgb.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cloud import _grab
rclpy.init(); node = rclpy.create_node("rgb_grab")
for cam in sys.argv[1:]:
    m = _grab(node, f"/{cam}/color/image_raw", Image)
    img = CvBridge().imgmsg_to_cv2(m, desired_encoding="bgr8")
    cv2.imwrite(f"snaps/final_{cam}.png", img); print("saved", cam, img.shape)
EOF
python3 rgb.py birdview frontview sideview 2>&1 | grep -v "^\["
python3 -c "
import cv2; im=cv2.imread('snaps/final_birdview.png'); cv2.imwrite('snaps/final_bird_crop.png', cv2.resize(im[150:400,200:480],None,fx=2,fy=2))"

# openrua op 126
python3 -c "
import cv2; im=cv2.imread('snaps/final_frontview.png'); cv2.imwrite('snaps/final_front_crop.png', cv2.resize(im[120:400,150:500],None,fx=1.6,fy=1.6))"

# openrua op 127
python3 -c "
from rob import *; import time; r=Robot(); a=r.arm_q(); time.sleep(1.0); b=r.arm_q()
print('arm settled:', np.max(np.abs(np.array(a)-np.array(b)))<1e-4, 'fingers', r.fingers())" 2>&1 | grep -v "^\["
