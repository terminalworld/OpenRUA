#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ----; ros2 action list 2>&1; echo ----; ros2 service list 2>&1 | head -40; echo ----; ros2 node list 2>&1

# openrua op 4
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls -la *.png; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ----; timeout 10 ros2 topic echo /agentview/color/camera_info --once 2>&1 | grep -A12 "^k:"

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),tr in sorted(seen.items()):
    print(f"{p:>28} -> {c:<28} t=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f}) q=({tr.rotation.x:.4f},{tr.rotation.y:.4f},{tr.rotation.z:.4f},{tr.rotation.w:.4f})")
EOF
python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(cam):
    rclpy.init(); node = rclpy.create_node("cloud")
    got = {}
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    def cb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
    node.create_subscription(TFMessage, "/tf_static", cb, qos)
    node.create_subscription(TFMessage, "/tf", cb, 100)
    while not all(k in got for k in "dci") or "tf" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()
    d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
    c = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
    k = got["i"].k; tr = got["tf"]
    T = np.eye(4); T[:3,:3] = quat_R(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w)
    T[:3,3] = [tr.translation.x,tr.translation.y,tr.translation.z]
    H,W = d.shape
    u,v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u-k[2])*d/k[0]; Y = (v-k[5])*d/k[4]
    P = np.stack([X,Y,d,np.ones_like(d)],-1) @ T.T
    return P[...,:3], c, d

if __name__ == "__main__":
    cam = sys.argv[1]
    P, c, d = grab(cam)
    np.save(f"{cam}_xyz.npy", P)
    z = P[...,2]
    print("z range", np.nanmin(z), np.nanmax(z))
    # histogram of z
    hist, edges = np.histogram(z[np.isfinite(z)], bins=60)
    for h,e in zip(hist, edges): 
        if h>50: print(f"{e:.3f} {h}")
EOF
python3 cloud.py birdview

# openrua op 8
cat > seg.py <<'EOF'
import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
c = cv2.imread("birdview.png")
# table height
tab = z[(z>0.8)&(z<0.9)]
print("table z median", np.median(tab))
mask = ((z>0.895)&(z<1.2)).astype(np.uint8)
# exclude robot region: robot base at world x=-0.75; keep world x > -0.6
mask &= (P[...,0] > -0.62).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 30: continue
    m = lab==i
    pts = P[m]
    print(f"comp {i}: px area {stats[i,4]} bbox(u,v,w,h)={stats[i,:4]} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
EOF
python3 seg.py

# openrua op 9
cat > caddy.py <<'EOF'
import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
c = cv2.imread("birdview.png")
u0,v0,w,h = 205,150,115,70
sub = z[v0:v0+h, u0:u0+w]
# print height map coarse (every 3 px)
for v in range(0,h,3):
    print("".join(" " if sub[v,u]<0.89 else ("." if sub[v,u]<0.93 else ("o" if sub[v,u]<1.0 else "#")) for u in range(0,w,1)))
crop = cv2.resize(c[v0:v0+h, u0:u0+w], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)
cv2.imwrite("caddy_crop.png", crop)
# world coords of corners
for (u,v) in [(217,160),(309,160),(217,207),(309,207)]:
    print((u,v), P[v,u])
EOF
python3 caddy.py

# openrua op 10
python3 -c "
import cv2
for name,(u0,v0,w,h) in {'agentview':(110,150,280,140),'robot0_robotview':(0,0,420,160),'frontview':(170,240,200,100)}.items():
    c=cv2.imread(name+'.png'); crop=cv2.resize(c[v0:v0+h,u0:u0+w],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite(name+'_crop.png',crop)
"

# openrua op 11
cat > book.py <<'EOF'
import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
m = (z>0.9)&(z<1.2)&(P[...,0]>-0.2)&(P[...,0]<0.0)&(P[...,1]>-0.1)&(P[...,1]<0.06)
pts = P[m]
print("n", len(pts), "z", pts[:,2].min(), pts[:,2].max())
top = pts[pts[:,2]>1.06]
print("top n", len(top), "z", top[:,2].min(), top[:,2].max())
xy = top[:,:2].astype(np.float32)
rect = cv2.minAreaRect(xy)
print("minAreaRect center", rect[0], "size", rect[1], "angle", rect[2])
# PCA
c = xy.mean(0); u,s,vt = np.linalg.svd(xy-c)
print("center", c, "axes", vt, "extent along axes", [(xy-c)@vt[i] for i in range(2)][0].ptp(), ((xy-c)@vt[1]).ptp())
# also full footprint
xy2 = pts[:,:2].astype(np.float32)
rect2 = cv2.minAreaRect(xy2); print("footprint rect", rect2)
EOF
python3 book.py

# openrua op 12
cat > comp.py <<'EOF'
import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
u0,v0,w,h = 205,150,115,70
sub = z[v0:v0+h, u0:u0+w]
floor = ((sub>0.885)&(sub<0.93)).astype(np.uint8)
# exclude table outside caddy: floor pixel must be inside caddy bbox: use connected components
n, lab, stats, cent = cv2.connectedComponentsWithStats(floor)
for i in range(1,n):
    if stats[i,4]<20: continue
    m = np.zeros_like(z,bool); m[v0:v0+h,u0:u0+w] = lab==i
    pts = P[m]
    print(f"comp {i} area {stats[i,4]} bbox {stats[i,:4]+np.array([u0,v0,0,0])} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
# wall top heights
walls = P[v0:v0+h,u0:u0+w][(sub>1.0)&(sub<1.2)]
print("wall top z", np.percentile(walls[:,2],[5,50,95]))
EOF
python3 comp.py

# openrua op 13
cat > prof.py <<'EOF'
import numpy as np
P = np.load("birdview_xyz.npy"); z = P[...,2]
print("v-profile (x) at u=275..285 avg:")
for v in range(150,200):
    zz = z[v,275:286].mean(); print(v, f"x={P[v,280,0]:.3f} z={zz:.3f}", "#" if zz>1.0 else ("." if zz<0.93 else "o"))
print("u-profile (y) at v=162..168 avg:")
for u in range(240,320):
    zz = z[162:169,u].mean(); print(u, f"y={P[165,u,1]:.3f} z={zz:.3f}", "#" if zz>1.0 else ("." if zz<0.93 else "o"))
EOF
python3 prof.py

# openrua op 14
cat > prof2.py <<'EOF'
import numpy as np
P = np.load("birdview_xyz.npy"); z = P[...,2]
print("u-profile (y) at v=172..178 avg:")
for u in range(250,310):
    zz = z[172:179,u].mean(); print(u, f"y={P[175,u,1]:.3f} z={zz:.3f}", "#" if zz>1.0 else ("." if zz<0.93 else "o"))
EOF
python3 prof2.py

# openrua op 15
cat > rob.py <<'EOF'
"""Shared robot helpers: joint state, FK/IK, trajectory, gripper."""
import time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_W = np.array([-0.75, 0.0, 0.912])   # world -> panda_link0 (from TF)
TCP = M["hand"]["tcp_offset_m"]

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("pickplace")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m): self._js = dict(zip(m.name, m.position))

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
            while not self._js: rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._js

    def arm_q(self): j = self.joints(); return np.array([j[n] for n in ARM])
    def fingers(self): j = self.joints(); return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self, q):
        s = JointState(); s.name = list(ARM); s.position = [float(x) for x in q]; return s

    def fk_hand(self, q=None):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result(); p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, r.error_code.val, r.pose_stamped[0].header.frame_id

    def ik_world(self, pos_w, quat, seed=None, tcp=False):
        """IK for hand pose given in WORLD frame (quat xyzw). tcp=True: pos is the TCP."""
        pos_w = np.array(pos_w, float)
        if tcp:
            pos_w = pos_w - TCP * Rot.from_quat(quat).as_matrix()[:, 2]
        p_b = pos_w - BASE_W
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, p_b)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float, quat)
        r.robot_state.joint_state = self._seed(self.arm_q() if seed is None else seed)
        r.avoid_collisions = False
        r.timeout.sec = 5
        fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None, (None if res is None else res.error_code.val)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM]), 1

    def move(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.array(q)).max()
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

def topdown_quat(theta):
    """Hand z = world -z, hand y (finger closing axis) = (cos t, sin t, 0). Returns xyzw."""
    yh = np.array([np.cos(theta), np.sin(theta), 0.0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
    return Rot.from_matrix(np.column_stack([xh, yh, zh])).as_quat()
EOF
cat > test_fk.py <<'EOF'
from rob import *
r = Robot()
print("joints", r.joints())
pos, quat, code, frame = r.fk_hand()
print("FK hand world", pos, "quat", quat, "code", code, "frame", frame)
print("hand R:\n", Rot.from_quat(quat).as_matrix().round(3))
# test IK for the current pose (should reproduce)
q, c = r.ik_world(pos, quat)
print("IK roundtrip", c, q)
print("neutral topdown quat theta=-90deg:", topdown_quat(-np.pi/2))
EOF
timeout 120 python3 test_fk.py

# openrua op 16
cat > test_ik.py <<'EOF'
from rob import *
r = Robot()
q0 = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]; req.robot_state.joint_state = r._seed(q0)
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
p = fut.result().pose_stamped[0].pose
raw = np.array([p.position.x, p.position.y, p.position.z]); quat = np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
print("raw FK", raw)
def fk_raw(q):
    req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]; req.robot_state.joint_state = r._seed(q)
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    p = fut.result().pose_stamped[0].pose
    return np.array([p.position.x, p.position.y, p.position.z]), np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
def ik_raw(pos, quat):
    req = GetPositionIK.Request(); rr = req.ik_request
    rr.group_name = "panda_arm"; rr.pose_stamped.header.frame_id = ""
    rr.pose_stamped.pose.position.x, rr.pose_stamped.pose.position.y, rr.pose_stamped.pose.position.z = map(float, pos)
    rr.pose_stamped.pose.orientation.x, rr.pose_stamped.pose.orientation.y, rr.pose_stamped.pose.orientation.z, rr.pose_stamped.pose.orientation.w = map(float, quat)
    rr.robot_state.joint_state = r._seed(q0); rr.avoid_collisions = False; rr.timeout.sec = 5
    fut = r.ik.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res = fut.result()
    if res.error_code.val != 1: return None, res.error_code.val
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return np.array([sol[j] for j in ARM]), 1
qd = topdown_quat(-np.pi/2)
for name, pos in [("raw fk", raw), ("raw+BASE", raw + BASE_W), ("raw-BASE", raw - BASE_W),
                  ("book hand world", [-0.093, -0.019, 1.20]), ("book hand base", np.array([-0.093, -0.019, 1.20]) - BASE_W)]:
    q, c = ik_raw(pos, qd)
    if q is None: print(name, pos, "IK fail", c); continue
    fp, fq = fk_raw(q)
    print(name, "target", np.round(pos,3), "-> FK(sol)", np.round(fp,3), "q", np.round(q,3))
EOF
timeout 300 python3 test_ik.py

# openrua op 17
python3 - <<'EOF'
s = open("rob.py").read()
s = s.replace("BASE_W = np.array([-0.75, 0.0, 0.912])   # world -> panda_link0 (from TF)",
              "# verified by IK/FK roundtrip: compute_ik/compute_fk on this machine work in true\n# WORLD coordinates (robot base offset already included), so no base shift is applied\nBASE_W = np.array([0.0, 0.0, 0.0])")
s = s.replace("""        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM]), 1""",
"""        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = np.array([sol[j] for j in ARM])
        # IK can report success with a bogus solution: verify by FK
        fp, fq, _, _ = self.fk_hand(q)
        perr = np.linalg.norm(fp - pos_w)
        aerr = (Rot.from_quat(fq) * Rot.from_quat(quat).inv()).magnitude()
        if perr > 0.005 or aerr > 0.03:
            return None, f"fk-mismatch pos={perr:.4f} ang={aerr:.3f}"
        return q, 1""")
open("rob.py","w").write(s)
EOF
grep -n "BASE_W\|fk-mismatch" rob.py

# openrua op 18
cat > stepA.py <<'EOF'
from rob import *
r = Robot()
print("start q", np.round(r.arm_q(),3), "fingers", r.fingers(), flush=True)
print("open gripper:", r.gripper(GRIP["open_m"]), flush=True)
BOOK = np.array([-0.094, -0.019]); TOP = 1.075
TH = np.arctan2(0.779, 0.627) - np.pi       # closing axis along book thickness
qd = topdown_quat(TH)
pre = np.array([BOOK[0], BOOK[1], 1.20])
q, c = r.ik_world(pre, qd, tcp=True)
print("IK pre", c, None if q is None else np.round(q,3), flush=True)
if q is None: raise SystemExit("no IK")
code, err = r.move(q, 4.0)
print("move pre: code", code, "max joint err", round(err,4), flush=True)
pos, quat, _, _ = r.fk_hand()
print("hand now", np.round(pos,4), "tcp", np.round(pos + TCP*Rot.from_quat(quat).as_matrix()[:,2],4), flush=True)
EOF
nohup python3 -u stepA.py > stepA.log 2>&1 &
echo started

# openrua op 19
sleep 45; cat stepA.log

# openrua op 20
python3 - <<'EOF'
s = open("rob.py").read()
s = s.replace('r.group_name = M["planning"]["group"]; r.pose_stamped.header.frame_id = ""',
              'r.group_name = M["planning"]["group"]; r.ik_link_name = "panda_hand"; r.pose_stamped.header.frame_id = ""')
open("rob.py","w").write(s)
EOF
nohup python3 -u stepA.py > stepA.log 2>&1 & sleep 40; cat stepA.log

# openrua op 21
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && timeout 60 python3 tools/perception/cam_snap.py agentview agent1.png

# openrua op 22
cat > stepB.py <<'EOF'
from rob import *
r = Robot()
BOOK = np.array([-0.094, -0.019]); TOP = 1.075
TH = np.arctan2(0.779, 0.627) - np.pi
qd = topdown_quat(TH)
def goto(z, secs, tries=2):
    tgt = np.array([BOOK[0], BOOK[1], z])
    q, c = r.ik_world(tgt, qd, tcp=True)
    if q is None: raise SystemExit(f"IK fail {c}")
    for i in range(tries):
        code, err = r.move(q, secs)
        pos, quat, _, _ = r.fk_hand()
        tcp = pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
        print(f"  z={z}: code {code} jerr {err:.4f} tcp {np.round(tcp,4)}", flush=True)
        if err < 0.005: break
goto(1.12, 2.5); goto(1.035, 2.5)
print("close:", r.gripper(GRIP["closed_m"]), flush=True)
f1, f2 = r.fingers(); print("fingers after close", f1, f2, "gap", f1 - f2, flush=True)
goto(1.30, 3.0, tries=1)
print("fingers after lift", r.fingers(), flush=True)
EOF
nohup python3 -u stepB.py > stepB.log 2>&1 & sleep 60; cat stepB.log

# openrua op 23
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png && timeout 60 python3 tools/perception/cam_snap.py robot0_robotview rv2.png

# openrua op 24
timeout 30 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once 2>&1 | grep -A9 "^k:"; python3 tfdump.py 2>/dev/null | grep -E "eye_in_hand|panda_hand "; timeout 60 python3 cloud.py robot0_eye_in_hand > /dev/null; python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_xyz.npy")
z = P[...,2]
print("finite frac", np.isfinite(z).mean(), "z range", np.nanmin(z), np.nanmax(z))
# book region in image approx u 300-335, v 235-335
sub = P[235:335, 300:335]
print("book patch z pct", np.nanpercentile(sub[...,2],[5,50,95]), "x", np.nanmedian(sub[...,0]), "y", np.nanmedian(sub[...,1]))
tab = P[100:200, 100:250]
print("table patch z", np.nanmedian(tab[...,2]))
EOF

# openrua op 25
python3 - <<'EOF'
import numpy as np, cv2
P = np.load("robot0_eye_in_hand_xyz.npy"); z = P[...,2]
m = (z>0.95)&(z<1.15)&(P[...,0]>-0.3)&(P[...,1]<0.05)&(P[...,1]>-0.15)
pts = P[m]; print("n", len(pts))
top = pts[pts[:,2]>pts[:,2].max()-0.012]
print("book top z", pts[:,2].max(), "top pts", len(top), "center", top[:,:2].mean(0))
rect = cv2.minAreaRect(top[:,:2].astype(np.float32)); print("rect", rect)
# birdview now
EOF
timeout 60 python3 cloud.py birdview > /dev/null; python3 book.py

# openrua op 26
python3 prof.py | sed -n '1,52p' | awk 'NR%1==0' | grep -E "^(15[5-9]|16[0-9]|17[0-9]|18[0-9]|19[0-2]) " 

# openrua op 27
timeout 30 ros2 topic echo /robot_description --once --qos-durability transient_local --qos-reliability reliable 2>/dev/null | python3 -c "
import sys,re
s=sys.stdin.read()
# find finger link
i=s.find('panda_leftfinger'); print(s[i-200:i+2500])
" | head -80

# openrua op 28
cat > urdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node("u"); got=[]
n.create_subscription(String, "/robot_description", got.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
while not got: rclpy.spin_once(n, timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
timeout 60 python3 urdf.py; grep -n -A30 '<link name="panda_leftfinger"' robot.urdf | head -60; grep -n "mesh filename" robot.urdf | grep -i finger | head

# openrua op 29
cat > stepB2.py <<'EOF'
from rob import *
r = Robot()
BOOK = np.array([-0.091, -0.017]); TOP = 1.021
TH = np.arctan2(0.779, 0.627) - np.pi
qd = topdown_quat(TH)
def goto(z, secs, tries=2):
    tgt = np.array([BOOK[0], BOOK[1], z])
    q, c = r.ik_world(tgt, qd, tcp=True)
    if q is None: raise SystemExit(f"IK fail {c}")
    for i in range(tries):
        code, err = r.move(q, secs)
        pos, quat, _, _ = r.fk_hand()
        tcp = pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
        print(f"  z={z}: code {code} jerr {err:.4f} tcp {np.round(tcp,4)}", flush=True)
        if err < 0.005: break
print("open:", r.gripper(GRIP["open_m"]), flush=True)
goto(1.12, 2.5); goto(TOP - 0.02, 2.5)
print("close:", r.gripper(GRIP["closed_m"]), flush=True)
f1, f2 = r.fingers(); print("fingers after close", f1, f2, "gap", f1 - f2, flush=True)
if f1 - f2 < 0.02: raise SystemExit("grasp failed (gap too small)")
goto(1.30, 3.0, tries=1)
print("fingers after lift", r.fingers(), flush=True)
EOF
nohup python3 -u stepB2.py > stepB2.log 2>&1 & sleep 75; cat stepB2.log

# openrua op 30
timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png >/dev/null; timeout 60 python3 cloud.py robot0_eye_in_hand > /dev/null; python3 - <<'EOF'
import numpy as np, cv2
P = np.load("robot0_eye_in_hand_xyz.npy"); z = P[...,2]
# book held: top near z 1.28 (TCP 1.301 + 0.02)
m = (z>1.15)&(z<1.32)
pts = P[m]; print("n", len(pts), "z range", pts[:,2].min(), pts[:,2].max())
top = pts[pts[:,2] > 1.25]
print("top n", len(top), "center xy", top[:,:2].mean(0), "z med", np.median(top[:,2]))
rect = cv2.minAreaRect(top[:,:2].astype(np.float32)); print("rect", rect)
c = cv2.imread("robot0_eye_in_hand.png"); cv2.imwrite("eih3.png", c)
EOF

# openrua op 31
timeout 60 python3 cloud.py agentview > /dev/null; python3 - <<'EOF'
import numpy as np
P = np.load("agentview_xyz.npy"); z = P[...,2]
m = (z>1.19)&(z<1.31)&(P[...,0]>-0.2)&(P[...,0]<0.05)&(np.abs(P[...,1])<0.15)
pts = P[m]; print("n", len(pts), "x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max())
L = np.array([0.779,-0.627]); T = np.array([0.627,0.779])
pl = pts[:,:2]@L; pt = pts[:,:2]@T
print("along long axis: min/max/center", pl.min(), pl.max(), (pl.min()+pl.max())/2, "span", pl.max()-pl.min())
print("along thin axis pct", np.percentile(pt,[2,50,98]))
tcp = np.array([-0.0857,-0.0175])
print("tcp along L", tcp@L, "along T", tcp@T)
EOF

# openrua op 32
cat > stepC.py <<'EOF'
from rob import *
r = Robot()
TH0 = np.arctan2(0.779, 0.627) - np.pi       # current yaw of closing axis
TH1 = -np.pi                                  # closing axis along world x
PLACE = np.array([-0.433, -0.1375])
def tcp_now():
    pos, quat, _, _ = r.fk_hand(); return pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
def line(p0, p1, th0, th1, n, secs):
    """straight TCP line with yaw interpolation, one trajectory of n waypoints"""
    seed = r.arm_q(); wps = []
    for i in range(1, n+1):
        s = i/n; p = p0 + (p1-p0)*s; th = th0 + (th1-th0)*s
        q, c = r.ik_world(p, topdown_quat(th), seed=seed, tcp=True)
        if q is None: raise SystemExit(f"IK fail at {p} th={th}: {c}")
        if np.abs(q - seed).max() > 1.0: print("  WARN big joint jump", np.round(q-seed,2), flush=True)
        wps.append(q); seed = q
    code, err = r.move(wps[-1], secs, via=wps[:-1])
    print(f"  line -> {np.round(p1,4)}: code {code} jerr {err:.4f} tcp {np.round(tcp_now(),4)} fingers {np.round(r.fingers(),4)}", flush=True)
    return wps[-1]
p0 = tcp_now(); print("tcp start", np.round(p0,4), flush=True)
q = line(p0, np.array([PLACE[0], PLACE[1], 1.30]), TH0, TH1, 6, 6.0)
print("q above caddy", np.round(q,3), flush=True)
EOF
nohup python3 -u stepC.py > stepC.log 2>&1 & sleep 60; cat stepC.log

# openrua op 33
timeout 60 python3 cloud.py agentview > /dev/null; python3 - <<'EOF'
import numpy as np, cv2
P = np.load("agentview_xyz.npy"); z = P[...,2]
m = (z>1.19)&(z<1.31)&(P[...,0]>-0.6)&(P[...,0]<-0.3)&(P[...,1]<0.0)&(P[...,1]>-0.3)
pts = P[m]; print("n", len(pts), "x pct", np.percentile(pts[:,0],[2,50,98]), "y min/max", pts[:,1].min(), pts[:,1].max(), "y center", (pts[:,1].min()+pts[:,1].max())/2)
# fit line to front face: x as function of y -> yaw
A = np.c_[pts[:,1], np.ones(len(pts))]; front = pts[pts[:,0] > np.percentile(pts[:,0],60)]
k, b = np.linalg.lstsq(np.c_[front[:,1], np.ones(len(front))], front[:,0], rcond=None)[0]
print("front face x = %.4f*y + %.4f -> yaw deg %.2f" % (k, b, np.degrees(np.arctan(k))))
c = cv2.imread("agentview.png"); cv2.imwrite("agent4.png", c)
EOF

# openrua op 34
cat > stepD.py <<'EOF'
from rob import *
r = Robot()
TH1 = -np.pi
def tcp_now():
    pos, quat, _, _ = r.fk_hand(); return pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
def line(p0, p1, th, n, secs, tries=2):
    seed = r.arm_q(); wps = []
    for i in range(1, n+1):
        p = p0 + (p1-p0)*i/n
        q, c = r.ik_world(p, topdown_quat(th), seed=seed, tcp=True)
        if q is None: raise SystemExit(f"IK fail at {p}: {c}")
        wps.append(q); seed = q
    for t in range(tries):
        code, err = r.move(wps[-1], secs, via=wps[:-1] if t == 0 else None)
        print(f"  -> {np.round(p1,4)}: code {code} jerr {err:.4f} tcp {np.round(tcp_now(),4)} fingers {np.round(r.fingers(),4)}", flush=True)
        if err < 0.004: break
p0 = tcp_now(); print("tcp start", np.round(p0,4), flush=True)
line(p0, np.array([-0.433, -0.140, 1.075]), TH1, 3, 4.0)
tcp = tcp_now()
if abs(tcp[0] + 0.433) > 0.006 or abs(tcp[1] + 0.140) > 0.006 or abs(tcp[2] - 1.075) > 0.006:
    raise SystemExit(f"not aligned, aborting before release: {tcp}")
print("release:", r.gripper(GRIP["open_m"]), flush=True)
print("fingers", r.fingers(), flush=True)
line(tcp_now(), np.array([-0.433, -0.140, 1.30]), TH1, 2, 3.0, tries=1)
EOF
nohup python3 -u stepD.py > stepD.log 2>&1 & sleep 60; cat stepD.log

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview front5.png > /dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview side5.png > /dev/null; echo done

# openrua op 36
timeout 60 python3 cloud.py birdview > /dev/null; python3 - <<'EOF'
import numpy as np
P = np.load("birdview_xyz.npy"); z = P[...,2]
# region of the back compartment interior + a bit
m = (P[...,0]>-0.47)&(P[...,0]<-0.39)&(P[...,1]>-0.21)&(P[...,1]<-0.07)&(z>0.88)&(z<1.2)
pts = P[m]
print("max z in back compartment region", pts[:,2].max())
# book top: points z in (1.0,1.06) and not on wall tops -> check they are inside the interior
top = pts[(pts[:,2]>1.005)&(pts[:,2]<1.045)]
print("candidate book-top pts", len(top), "x", top[:,0].min() if len(top) else None, top[:,0].max() if len(top) else None, "y", top[:,1].min() if len(top) else None, top[:,1].max() if len(top) else None, "z", np.median(top[:,2]) if len(top) else None)
# is any of the book outside the slot? points higher than the divider (0.988) between x -0.405..-0.39 that aren't divider
print("front compartment (x -0.39..-0.345) max z:", P[(P[...,0]>-0.39)&(P[...,0]<-0.345)&(P[...,1]>-0.2)&(P[...,1]<-0.08)&(z<1.2)][:,2].max())
# height map of region
u0,v0 = 250,155
sub = z[v0:v0+40, u0:u0+55]
for v in range(0,40,2):
    print("".join(" " if sub[v,u]<0.89 else ("." if sub[v,u]<0.95 else ("o" if sub[v,u]<1.0 else ("B" if sub[v,u]<1.045 else "#"))) for u in range(55)))
EOF

# openrua op 37
cat > stepE.py <<'EOF'
from rob import *
r = Robot()
def tcp_now():
    pos, quat, _, _ = r.fk_hand(); return pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
print("tcp", np.round(tcp_now(),4), "q", np.round(r.arm_q(),3), flush=True)
q, c = r.ik_world([-0.35, 0.30, 1.30], topdown_quat(-np.pi/2), tcp=True)
if q is None: raise SystemExit(f"IK fail {c}")
for t in range(2):
    code, err = r.move(q, 4.0)
    print("park: code", code, "jerr", round(err,4), "tcp", np.round(tcp_now(),4), flush=True)
    if err < 0.01: break
EOF
nohup python3 -u stepE.py > stepE.log 2>&1 & sleep 45; cat stepE.log

# openrua op 38
timeout 60 python3 tools/perception/cam_snap.py agentview agent6.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview rv6.png >/dev/null; timeout 60 python3 cloud.py birdview > /dev/null; python3 - <<'EOF'
import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
m = (P[...,0]>-0.50)&(P[...,0]<-0.30)&(P[...,1]>-0.40)&(P[...,1]<0.0)&(z>0.88)&(z<1.2)
pts = P[m]; print("max z caddy region", pts[:,2].max())
book = pts[(pts[:,2]>1.005)&(pts[:,2]<1.045)]
print("book-top-ish pts", len(book), "x[%.3f,%.3f] y[%.3f,%.3f] z med %.3f" % (book[:,0].min(), book[:,0].max(), book[:,1].min(), book[:,1].max(), np.median(book[:,2])))
# tighter: exclude wall-top edge pixels by requiring neighbors also in band
u0,v0 = 250,155
sub = z[v0:v0+40, u0:u0+55]
for v in range(0,40,2):
    print("".join(" " if sub[v,u]<0.89 else ("." if sub[v,u]<0.95 else ("o" if sub[v,u]<1.0 else ("B" if sub[v,u]<1.045 else "#"))) for u in range(55)))
# anything above table elsewhere near the caddy that could be the book fallen out?
other = P[(P[...,0]>-0.6)&(P[...,0]<0.1)&(np.abs(P[...,1])<0.45)&(z>0.9)&(z<1.2)]
print("above-table pts x range", other[:,0].min(), other[:,0].max(), "y range", other[:,1].min(), other[:,1].max())
EOF

# openrua op 39
python3 -c "
import cv2
c=cv2.imread('agent6.png'); crop=cv2.resize(c[150:290,110:390],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('agent6_crop.png',crop)"

# openrua op 40
pgrep -af "python3 -u step" || echo "no pending motion scripts"; ros2 topic echo /joint_states --once 2>/dev/null | grep -A9 "^position" | tail -9 | tr '\n' ' '
