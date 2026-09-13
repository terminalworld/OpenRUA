#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | head -50; echo ---; timeout 20 ros2 node list

# openrua op 3
mkdir -p snaps && timeout 60 ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 4
cd /workspace/snaps && for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 ../tools/perception/cam_snap.py $c ${c}.png & done; wait; ls -la

# openrua op 5
cd /workspace && timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | head -60

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(msg):
    for t in msg.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 4: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    tr=t.transform.translation; q=t.transform.rotation
    print(f"{p:28s} -> {c:32s} t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
EOF
timeout 60 python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
"""Grab depth+info for a camera, save world-frame xyz array (H,W,3) as <cam>_xyz.npy"""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage, "/tf_static", lambda m: [got.setdefault("t", t) for t in m.transforms if t.child_frame_id == f"{cam}_optical_frame"], qos)
node.create_subscription(TFMessage, "/tf", lambda m: [got.setdefault("t", t) for t in m.transforms if t.child_frame_id == f"{cam}_optical_frame"], 100)
while not all(k in got for k in "dit"): rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["t"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
v, u = np.mgrid[0:d.height, 0:d.width]
X = (u - cx) * depth / fx; Y = (v - cy) * depth / fy; Z = depth
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
P = np.stack([X, Y, Z], -1) @ R.T + T
np.save(f"snaps/{cam}_xyz.npy", P.astype(np.float32))
print(cam, P.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
EOF
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py sideview

# openrua op 8
cat > analyze.py <<'EOF'
import numpy as np
P = np.load("snaps/birdview_xyz.npy")
Z = P[...,2]
print("z histogram (birdview):")
h, e = np.histogram(Z[np.isfinite(Z)], bins=60, range=(0.0, 1.6))
for c, lo in zip(h, e[:-1]):
    if c > 50: print(f"  {lo:.3f}-{lo+e[1]-e[0]:.3f}: {c}")
# table height: mode
# Bottle: birdview pixel ~ (345, 240)? check world coords around candidate pixels
for (u,v) in [(345,240),(348,238),(300,300),(400,300),(320,240),(420,300),(360,300)]:
    print((u,v), P[v,u])
EOF
python3 analyze.py

# openrua op 9
cat > analyze2.py <<'EOF'
import numpy as np
A = np.load("snaps/agentview_xyz.npy")
S = np.load("snaps/sideview_xyz.npy")
print("agentview bottle pixels:")
for (u,v) in [(362,200),(362,215),(360,190),(368,155),(366,170),(365,180)]:
    print((u,v), A[v,u])
print("sideview bottle pixels:")
for (u,v) in [(358,280),(358,265),(360,250),(358,290),(360,240)]:
    print((u,v), S[v,u])
print("agentview drawer pixels (open drawer interior/rim):")
for (u,v) in [(420,300),(400,280),(380,260),(440,320),(370,250),(455,340),(375,330),(360,300),(470,285),(480,300)]:
    print((u,v), A[v,u])
print("agentview cabinet top:", A[250,540], A[220,500], A[200,470])
print("agentview table:", A[400,300], A[420,200])
EOF
python3 analyze2.py

# openrua op 10
cat > analyze3.py <<'EOF'
import numpy as np
A = np.load("snaps/agentview_xyz.npy")
B = np.load("snaps/birdview_xyz.npy")
def ext(P, m, label):
    pts = P[m]
    if len(pts)==0: print(label, "none"); return
    print(f"{label}: n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    return pts
for name,P in [("agent",A),("bird",B)]:
    z=P[...,2]; x=P[...,0]; y=P[...,1]
    ext(P, (z>0.915)&(z<0.935)&(y>0)&(y<0.4)&(x>-0.3)&(x<0.3), name+" drawer floor")
    ext(P, (z>0.975)&(z<0.995)&(y>0)&(y<0.4)&(x>-0.3)&(x<0.3), name+" drawer rim")
    ext(P, (z>1.11)&(z<1.14)&(y>0)&(y<0.6)&(x>-0.3)&(x<0.3), name+" cabinet top")
    ext(P, (z>0.93)&(z<1.20)&(y>0.0)&(y<0.15)&(x>-0.25)&(x<-0.10), name+" bottle")
    ext(P, (z>1.10)&(z<1.20)&(y>0.0)&(y<0.15)&(x>-0.25)&(x<-0.10), name+" bottle top")
    ext(P, (z>0.93)&(z<1.20)&(y>-0.2)&(y<0.05)&(x>-0.1)&(x<0.15), name+" bowl")
EOF
python3 analyze3.py

# openrua op 11
cat > heightmap.py <<'EOF'
import numpy as np, cv2
B = np.load("snaps/birdview_xyz.npy").reshape(-1,3)
A = np.load("snaps/agentview_xyz.npy").reshape(-1,3)
S = np.load("snaps/sideview_xyz.npy").reshape(-1,3)
P = np.concatenate([B,A,S]); P = P[np.isfinite(P).all(1)]
res=0.005; x0,x1,y0,y1 = -0.45,0.35,-0.35,0.55
W=int((y1-y0)/res); H=int((x1-x0)/res)
hm = np.full((H,W), np.nan)
ix=((P[:,0]-x0)/res).astype(int); iy=((P[:,1]-y0)/res).astype(int)
m=(ix>=0)&(ix<H)&(iy>=0)&(iy<W)&(P[:,2]<1.3)
for a,b,z in zip(ix[m],iy[m],P[m,2]):
    if np.isnan(hm[a,b]) or z>hm[a,b]: hm[a,b]=z
np.save("snaps/hm.npy", hm)
img = np.nan_to_num((hm-0.88)/(1.2-0.88),nan=0).clip(0,1)
img = cv2.applyColorMap((img*255).astype(np.uint8), cv2.COLORMAP_JET)
img = cv2.resize(img, (W*2,H*2), interpolation=cv2.INTER_NEAREST)
# grid lines every 0.1 m
for xv in np.arange(-0.4,0.35,0.1):
    r=int((xv-x0)/res)*2; cv2.line(img,(0,r),(W*2,r),(255,255,255),1); cv2.putText(img,f"x={xv:.1f}",(2,r-2),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
for yv in np.arange(-0.3,0.55,0.1):
    c=int((yv-y0)/res)*2; cv2.line(img,(c,0),(c,H*2),(255,255,255),1); cv2.putText(img,f"y={yv:.1f}",(c+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite("snaps/heightmap.png", img)
print("saved; rows=x (down = +x), cols=y (right = +y)")
EOF
python3 heightmap.py

# openrua op 12
cat > analyze4.py <<'EOF'
import numpy as np
hm = np.load("snaps/hm.npy"); res=0.005; x0,y0=-0.45,-0.35
def cell(x,y): return hm[int((x-x0)/res), int((y-y0)/res)]
def row(x, ys): return " ".join(f"{cell(x,y):.3f}" if np.isfinite(cell(x,y)) else "  nan" for y in ys)
print("Bottle region: rows x=-0.22..-0.11, cols y=0.02..0.13")
ys=np.arange(0.02,0.135,0.005)
print("      y:", " ".join(f"{y:5.3f}" for y in ys))
for x in np.arange(-0.22,-0.105,0.005): print(f"x={x:6.3f}:", row(x,ys))
print()
print("Drawer profile along y at x=0.0 (front handle..cabinet):")
ys=np.arange(0.02,0.30,0.005); print("      y:", " ".join(f"{y:5.3f}" for y in ys)); print("        ", row(0.0,ys))
print("Drawer profile along x at y=0.15:")
xs=np.arange(-0.16,0.17,0.005); print("      x:", " ".join(f"{x:6.3f}" for x in xs)); print("        ", " ".join(f"{cell(x,0.15):6.3f}" if np.isfinite(cell(x,0.15)) else "   nan" for x in xs))
EOF
python3 analyze4.py

# openrua op 13
cat > analyze5.py <<'EOF'
import numpy as np
for name in ["agentview","sideview","birdview"]:
    P=np.load(f"snaps/{name}_xyz.npy").reshape(-1,3)
    m=(P[:,2]>0.93)&(P[:,2]<1.135)&(P[:,0]>-0.21)&(P[:,0]<-0.12)&(P[:,1]>0.02)&(P[:,1]<0.12)
    Q=P[m]
    print(name, "bottle pts", len(Q))
    for zlo in np.arange(0.93,1.14,0.02):
        s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.02)]
        if len(s): print(f"  z {zlo:.2f}-{zlo+0.02:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF
python3 analyze5.py

# openrua op 14
cat > analyze6.py <<'EOF'
import numpy as np
for name in ["agentview","sideview","birdview","frontview"]:
    try: P=np.load(f"snaps/{name}_xyz.npy").reshape(-1,3)
    except: continue
    m=(P[:,2]>0.905)&(P[:,2]<1.135)&(P[:,0]>-0.06)&(P[:,0]<0.06)&(P[:,1]>0.15)&(P[:,1]<0.30)
    Q=P[m]; print(name, len(Q))
    for ylo in np.arange(0.15,0.30,0.01):
        s=Q[(Q[:,1]>=ylo)&(Q[:,1]<ylo+0.01)]
        if len(s): print(f"  y {ylo:.2f}: n={len(s):4d} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]", np.histogram(s[:,2],bins=[0.9,0.94,0.98,1.02,1.06,1.10,1.14])[0])
EOF
timeout 60 python3 cloud.py frontview; python3 analyze6.py

# openrua op 15
timeout 20 ros2 node info /robot_bridge 2>&1 | head -60; echo ---; timeout 10 ros2 topic echo /pipeline_state --once 2>&1 | head -20; echo ---; timeout 10 ros2 service type /get_urdf; timeout 10 ros2 service type /load_geometry_from_file; timeout 10 ros2 service type /load_map

# openrua op 16
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Reusable robot helper: joint state, FK, IK, trajectories, gripper, servo, cameras."""
import math, time, sys
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped, WrenchStamped
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from tf2_msgs.msg import TFMessage

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2; w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# Top-down grasp orientation: hand Z points down (-world Z), hand X along world X
# (fingers open along hand Y => along world Y). yaw rotates about world Z.
def R_topdown(yaw=0.0):
    return rotz(yaw) @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._w_cb, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(20); self.grip.wait_for_server(20)
        self.ik.wait_for_service(20); self.fk.wait_for_service(20)
        self.spin(0.5)

    def _js_cb(self, m):
        self.js = m

    def _w_cb(self, m):
        self.wrench = m

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- sensing ----------
    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def force(self):
        self.wrench = None
        end = time.time() + 2
        while self.wrench is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.wrench is None:
            return None
        f = self.wrench.wrench.force
        return np.array([f.x, f.y, f.z])

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame: (pos, R)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk_hand(q)
        return pos + TCP * R[:, 2], R

    # ---------- planning ----------
    def ik_hand(self, pos_world, R, seed=None, timeout=20):
        """IK for hand frame at world pos with rotation R. Returns q or None."""
        if seed is None:
            seed = self.arm_q()
        p_base = np.asarray(pos_world) - BASE
        q = R_to_quat(R)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_tcp(self, tcp_world, R, seed=None):
        hand = np.asarray(tcp_world) - TCP * R[:, 2]
        return self.ik_hand(hand, R, seed)

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        if via is not None:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
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
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"move_q done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None):
        q = self.ik_tcp(tcp_world, R, seed)
        if q is None:
            print(f"IK FAILED for tcp {np.round(tcp_world,3)}", flush=True)
            return None
        self.move_q(q, seconds)
        p, _ = self.tcp()
        print(f"  tcp now {np.round(p,4)} (target {np.round(tcp_world,4)})", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(0.3)
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}", flush=True)
        return r

    def servo(self, lin, ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)

    # ---------- cameras ----------
    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        s1 = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
        s2 = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s3 = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
        end = time.time() + 30
        while not all(k in got for k in "cdi") and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for s in (s1, s2, s3):
            self.node.destroy_subscription(s)
        img = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
        out = out or f"/workspace/snaps/{cam}.png"
        cv2.imwrite(out, img)
        d = got["d"]
        depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        return img, depth, got["i"]

    def cam_tf(self, cam):
        """world -> <cam>_optical_frame (pos, R) from /tf."""
        got = {}
        frame = f"{cam}_optical_frame"
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cb = lambda m: [got.setdefault("t", t) for t in m.transforms if t.child_frame_id == frame]
        s1 = self.node.create_subscription(TFMessage, "/tf", cb, 100)
        s2 = self.node.create_subscription(TFMessage, "/tf_static", cb, qos)
        end = time.time() + 10
        while "t" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(s1); self.node.destroy_subscription(s2)
        t = got["t"].transform
        return (np.array([t.translation.x, t.translation.y, t.translation.z]),
                quat_to_R([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

    def cloud(self, cam):
        """World-frame xyz (H,W,3) for the camera's current depth frame."""
        img, depth, info = self.snap(cam)
        T, R = self.cam_tf(cam)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
        P = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
        return img, P @ R.T + T
OPENRUA_EOF

# openrua op 17
cat > t1.py <<'EOF'
import numpy as np
from rob import *
r = Robot("t1")
q = r.arm_q(); print("q", np.round(q,3))
pos, R = r.fk_hand(); print("hand world", np.round(pos,4)); print(np.round(R,3))
tcp, _ = r.tcp(); print("tcp world", np.round(tcp,4))
T, Rc = r.cam_tf("robot0_eye_in_hand"); print("eih cam", np.round(T,4)); print(np.round(Rc,3))
# camera pose relative to hand
Rh_c = R.T @ Rc; th_c = R.T @ (T - pos)
print("hand->cam t", np.round(th_c,4)); print(np.round(Rh_c,3))
print("gap", r.finger_gap(), "force", r.force())
EOF
timeout 120 python3 t1.py

# openrua op 18
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK already answer in world frame (verified)/' rob.py && cat > t2.py <<'EOF'
import numpy as np
from rob import *
r = Robot("t2")
q0 = r.arm_q()
pos, R = r.fk_hand(); print("hand world", np.round(pos,4))
q = r.ik_hand(pos, R, seed=q0)
print("ik roundtrip q", None if q is None else np.round(q,3), "vs", np.round(q0,3))
# try IK for a top-down pose above the bottle
target = np.array([-0.166, 0.073, 1.25])
for yaw in [0, np.pi/4, -np.pi/4, np.pi/2]:
    q = r.ik_hand(target, R_topdown(yaw), seed=q0)
    print("yaw", yaw, "->", None if q is None else np.round(q,3))
EOF
timeout 180 python3 t2.py

# openrua op 19
cat > peek.py <<'EOF'
import numpy as np, cv2
from rob import *
r = Robot("peek")
q0 = r.arm_q()
d = np.array([0, np.cos(np.radians(30)), -np.sin(np.radians(30))])  # optical axis
Y = np.array([1.0, 0, 0]); X = np.cross(Y, d); R = np.stack([X, Y, d], 1)
cam_target = np.array([0.0, 0.0, 1.12])
hand = cam_target - 0.05 * X
q = r.ik_hand(hand, R, seed=q0)
print("ik", None if q is None else np.round(q, 3))
if q is None: raise SystemExit
r.move_q(q, 4.0)
pos, Rh = r.fk_hand(); print("hand", np.round(pos, 3)); print(np.round(Rh, 2))
img, P = r.cloud("robot0_eye_in_hand")
np.save("snaps/peek_xyz.npy", P)
Z = P[..., 2]; Xw = P[..., 0]; Yw = P[..., 1]
m = (np.abs(Z - 0.925) < 0.01) & (Xw > -0.1) & (Xw < 0.1) & (Yw > 0.05)
print("drawer floor pts:", m.sum(), "y range", Yw[m].min() if m.any() else None, Yw[m].max() if m.any() else None)
for ylo in np.arange(0.10, 0.45, 0.02):
    mm = (Xw > -0.1) & (Xw < 0.1) & (Yw >= ylo) & (Yw < ylo + 0.02) & (Z > 0.90) & (Z < 1.0)
    if mm.any():
        print(f"  y {ylo:.2f}: n={mm.sum():5d} z[{Z[mm].min():.3f},{Z[mm].max():.3f}]  hist", np.histogram(Z[mm], bins=[0.9,0.93,0.95,0.97,0.99,1.0])[0])
EOF
timeout 600 python3 peek.py

# openrua op 20
cat > resend.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("resend")
q = np.array([float(x) for x in sys.argv[1].split(",")]); sec = float(sys.argv[2])
print("now ", np.round(r.arm_q(),3)); print("goal", np.round(q,3))
r.move_q(q, sec)
print("after", np.round(r.arm_q(),3))
pos, Rh = r.fk_hand(); print("hand", np.round(pos,3)); print(np.round(Rh,2))
EOF
timeout 600 python3 resend.py -0.411,0.818,0.338,-1.623,0.889,1.998,-1.499 6

# openrua op 21
cat > look.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("look")
print("q", np.round(r.arm_q(),3), "gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
for c in sys.argv[1:]: r.snap(c)
EOF
timeout 300 python3 look.py agentview sideview robot0_eye_in_hand

# openrua op 22
cat > analyze7.py <<'EOF'
import numpy as np
P = np.load("snaps/peek_xyz.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
fl = (np.abs(Z-0.925)<0.006)&(X>-0.06)&(X<0.06)
print("floor (|x|<6cm): y", Y[fl].min(), Y[fl].max())
fl2 = (np.abs(Z-0.925)<0.006)&(Y>0.12)&(Y<0.20)
print("floor (y .12-.20): x", X[fl2].min(), X[fl2].max())
# back wall: points y>0.2 with z 0.93-0.98
bw = (Z>0.935)&(Z<0.975)&(Y>0.18)&(Y<0.30)&(X>-0.06)&(X<0.06)
print("back wall pts", bw.sum(), "y range", np.percentile(Y[bw],[5,50,95]))
# wall tops
for lab,m in [("front rim",(Z>0.975)&(Z<0.995)&(Y<0.12)&(X>-0.06)&(X<0.06)),("back rim",(Z>0.975)&(Z<1.0)&(Y>0.18)&(Y<0.3)&(X>-0.06)&(X<0.06)),("left rim",(Z>0.975)&(Z<0.995)&(X<-0.08)&(Y>0.1)&(Y<0.22)),("right rim",(Z>0.975)&(Z<0.995)&(X>0.08)&(Y>0.1)&(Y<0.22))]:
    if m.any(): print(lab, m.sum(), "x", np.round(np.percentile(X[m],[2,98]),3), "y", np.round(np.percentile(Y[m],[2,98]),3), "z", np.round(np.percentile(Z[m],[2,98]),3))
# cabinet opening: points above back rim at y~0.22-0.24, z 0.99-1.06
op = (Y>0.21)&(Y<0.26)&(Z>0.99)&(Z<1.08)&(X>-0.06)&(X<0.06)
print("cabinet face above opening: n",op.sum(), "z min", Z[op].min() if op.any() else None, "y", np.round(np.percentile(Y[op],[5,50,95]),3) if op.any() else None)
for zlo in np.arange(0.98,1.06,0.01):
    m=(Y>0.21)&(Y<0.26)&(Z>=zlo)&(Z<zlo+0.01)&(X>-0.06)&(X<0.06)
    if m.any(): print(f"  z {zlo:.2f}: n={m.sum()} y med {np.median(Y[m]):.3f}")
EOF
python3 analyze7.py

# openrua op 23
cat > pick.py <<'EOF'
import numpy as np
from rob import *
r = Robot("pick")
BOT = np.array([-0.166, 0.073])
yaw = np.arctan2(0.57, 0.82)
R = R_topdown(yaw)
r.gripper(0.04)
q0 = r.arm_q()
# hover above bottle
q1 = r.move_tcp([BOT[0], BOT[1], 1.22], R, 5.0, seed=q0)
if q1 is None: raise SystemExit("no IK hover")
print("q1", np.round(q1,3), "force", np.round(r.force(),2))
# descend to neck
q2 = r.move_tcp([BOT[0], BOT[1], 1.105], R, 3.0, seed=q1)
if q2 is None: raise SystemExit("no IK descend")
print("force", np.round(r.force(),2))
r.gripper(0.0)
print("gap after close", round(r.finger_gap(),4))
r.snap("agentview"); r.snap("sideview")
EOF
timeout 900 python3 -u pick.py 2>&1 | tee pick.log

# openrua op 24
cat > lift.py <<'EOF'
import numpy as np
from rob import *
r = Robot("lift")
q0 = r.arm_q(); tcp, R = r.tcp()
print("tcp", np.round(tcp,3))
q = r.move_tcp([tcp[0], tcp[1], 1.32], R, 4.0, seed=q0)
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
img, P = r.cloud("sideview")
Z=P[...,2]; X=P[...,0]; Y=P[...,1]
m=(Z>0.905)&(Z<1.1)&(X>-0.25)&(X<-0.1)&(Y>0.0)&(Y<0.15)
print("bottle-region points left on table (should be ~0):", m.sum())
m2=(Z>1.05)&(Z<1.35)&(X>-0.25)&(X<-0.05)&(Y>-0.05)&(Y<0.2)
print("points 1.05-1.35 (bottle hanging):", m2.sum(), "zmin", Z[m2].min() if m2.any() else None)
EOF
timeout 900 python3 -u lift.py 2>&1 | tee lift.log

# openrua op 25
cat > urdf.py <<'EOF'
import rclpy, re
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("u"); got=[]
n.create_subscription(String,"/robot_description",got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(n,timeout_sec=0.2)
s=got[0].data; open("snaps/robot.urdf","w").write(s)
i=s.find('name="panda_hand"'); print(s[i-20:i+2500])
EOF
timeout 60 python3 urdf.py | grep -iE "box|cylinder|sphere|mesh|origin|collision|link name" | head -30

# openrua op 26
python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(path):
    d=open(path,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    pts=[]
    for i in range(n):
        off=84+i*50
        v=struct.unpack_from('<12f',d,off)
        pts+= [v[3:6],v[6:9],v[9:12]]
    P=np.array(pts); return P.min(0),P.max(0)
for f in ["hand","finger"]:
    lo,hi=stl_bounds(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl")
    print(f, "min",np.round(lo,4),"max",np.round(hi,4))
EOF

# openrua op 27
find / -name "hand.stl" -o -name "hand.dae" 2>/dev/null | head; find / -path "*panda*" -name "*.stl" 2>/dev/null | head -3

# openrua op 28
cat > side_grasp.py <<'EOF'
import numpy as np
from rob import *
r = Robot("sg")
BOT = np.array([-0.166, 0.073]); ZG = 1.04
R = np.stack([[0,0,-1],[0,1,0],[1,0,0]], 1).astype(float)  # cols X,Y,Z
r.gripper(0.04)
q0 = r.arm_q()
# pre-grasp: 7cm short of the bottle axis, same height
q1 = r.ik_tcp([BOT[0]-0.07, BOT[1], ZG], R, seed=q0)
print("pregrasp IK", None if q1 is None else np.round(q1,3))
if q1 is None: raise SystemExit
# also check grasp + lift IK before moving
q2 = r.ik_tcp([BOT[0], BOT[1], ZG], R, seed=q1); print("grasp IK", None if q2 is None else np.round(q2,3))
q3 = r.ik_tcp([BOT[0], BOT[1], ZG+0.11], R, seed=q2 if q2 is not None else q1); print("lift IK", None if q3 is None else np.round(q3,3))
if q2 is None or q3 is None: raise SystemExit
r.move_q(q1, 6.0); p,_=r.tcp(); print("tcp", np.round(p,3), "force", np.round(r.force(),2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png"); r.snap("sideview", "/workspace/snaps/side_pre.png")
EOF
timeout 900 python3 -u side_grasp.py 2>&1 | tee sg.log

# openrua op 29
cat > state.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("st")
q=r.arm_q(); print("q", np.round(q,3))
pos,R=r.fk_hand(); print("hand", np.round(pos,3)); print(np.round(R,2)); t,_=r.tcp(); print("tcp", np.round(t,3))
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
for c in sys.argv[1:]: r.snap(c)
EOF
timeout 300 python3 state.py agentview sideview

# openrua op 30
cat > recover.py <<'EOF'
import numpy as np
from rob import *
r = Robot("rec")
q0 = r.arm_q(); pos, R = r.fk_hand()
best=None
for dz in [0.15, 0.12, 0.10]:
    for _ in range(5):
        q = r.ik_hand(pos + [0,0,dz], R, seed=q0)
        if q is not None:
            d = np.abs(q-q0).max(); print("dz",dz,"joint dist",round(d,3))
            if d < 1.0: best=q; break
    if best is not None: break
if best is None: raise SystemExit("no close IK")
r.move_q(best, 4.0)
t,_=r.tcp(); print("tcp", np.round(t,3), "force", np.round(r.force(),2))
EOF
timeout 600 python3 -u recover.py

# openrua op 31
cat > iksearch.py <<'EOF'
import numpy as np
from rob import *
r = Robot("iks")
q0 = r.arm_q()
BOT = np.array([-0.166, 0.073]); ZG = 1.04
lim = np.array(FJT["limits_rad"])
def R_side(tilt=0.0):
    Z = np.array([np.cos(tilt), 0, -np.sin(tilt)]); Y = np.array([0,1,0.]); X = np.cross(Y, Z)
    return np.stack([X, Y, Z], 1)
rng = np.random.default_rng(0)
seeds = [q0, np.array([0, 0.5, 0, -2.5, 0, 3.0, 0.8]), np.array([0, 0.9, 0, -2.2, 0, 3.1, 0.8]), np.array([0,-0.2,0,-2.4,0,2.2,0.8])]
seeds += [rng.uniform(lim[:,0]*0.8, lim[:,1]*0.8) for _ in range(25)]
for tilt in [0.0, np.radians(15)]:
    R = R_side(tilt)
    sols=[]
    for s in seeds:
        q = r.ik_tcp([BOT[0]-0.07, BOT[1], ZG], R, seed=s)
        if q is None: continue
        if any(np.abs(q-x).max()<0.05 for x in sols): continue
        sols.append(q)
    print(f"tilt {np.degrees(tilt):.0f}: {len(sols)} distinct solutions")
    for q in sols:
        # score: margin to limits, distance to q0
        marg = np.minimum(q-lim[:,0], lim[:,1]-q).min()
        print("  q", np.round(q,2), "limit margin", round(marg,2), "dist q0", round(np.abs(q-q0).max(),2))
EOF
timeout 900 python3 -u iksearch.py 2>&1 | tail -40

# openrua op 32
cat > iksearch2.py <<'EOF'
import numpy as np
from rob import *
r = Robot("iks2")
q0 = r.arm_q()
BOT = np.array([-0.166, 0.073]); ZG = 1.04
lim = np.array(FJT["limits_rad"])
def R_approach(az, tilt):
    Z = np.array([np.cos(az)*np.cos(tilt), np.sin(az)*np.cos(tilt), -np.sin(tilt)])
    Y = np.array([-np.sin(az), np.cos(az), 0.0]); X = np.cross(Y, Z)
    return np.stack([X, Y, Z], 1)
rng = np.random.default_rng(1)
seeds = [q0] + [rng.uniform(lim[:,0]*0.8, lim[:,1]*0.8) for _ in range(30)]
for az in [np.radians(45), np.radians(30), np.radians(60)]:
  for tilt in [0.0, np.radians(10)]:
    R = R_approach(az, tilt)
    sols=[]
    for s in seeds:
        q = r.ik_tcp([BOT[0]-0.07*np.cos(az), BOT[1]-0.07*np.sin(az), ZG], R, seed=s)
        if q is None or any(np.abs(q-x).max()<0.05 for x in sols): continue
        sols.append(q)
    good=[]
    for q in sols:
        marg = np.minimum(q-lim[:,0], lim[:,1]-q).min()
        if marg > 0.2:
            # check grasp & lift reachable from this branch
            qg = r.ik_tcp([BOT[0], BOT[1], ZG], R, seed=q); ql = None if qg is None else r.ik_tcp([BOT[0], BOT[1], ZG+0.11], R, seed=qg)
            ok = qg is not None and ql is not None and np.abs(qg-q).max()<0.6 and np.abs(ql-qg).max()<0.8
            good.append((marg, q, ok))
    print(f"az {np.degrees(az):.0f} tilt {np.degrees(tilt):.0f}: {len(sols)} sols, {len(good)} with margin>0.2")
    for marg,q,ok in sorted(good, key=lambda t:-t[0])[:4]:
        print("   margin", round(marg,2), "chain ok", ok, "q", np.round(q,2))
EOF
timeout 1200 python3 -u iksearch2.py 2>&1 | tail -40

# openrua op 33
cat > side2.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("sg2")
BOT = np.array([-0.166, 0.073]); ZG = 1.04
az = np.radians(60)
Z = np.array([np.cos(az), np.sin(az), 0.0]); Y = np.array([-np.sin(az), np.cos(az), 0.0]); X = np.cross(Y, Z)
R = np.stack([X, Y, Z], 1)
np.save("snaps/R_side.npy", R)
seed = np.array([-0.66, 0.49, 0.21, -2.46, 1.6, 1.68, 0.98])
q0 = r.arm_q()
pre = np.array([BOT[0]-0.07*np.cos(az), BOT[1]-0.07*np.sin(az), ZG])
q1 = r.ik_tcp(pre, R, seed=seed); print("q1", np.round(q1,3), "dist from now", round(np.abs(q1-q0).max(),2))
qg = r.ik_tcp([BOT[0], BOT[1], ZG], R, seed=q1); ql = r.ik_tcp([BOT[0], BOT[1], ZG+0.11], R, seed=qg)
print("qg", np.round(qg,3)); print("ql", np.round(ql,3))
np.save("snaps/q_chain.npy", np.stack([q1,qg,ql]))
if "--go" not in sys.argv: raise SystemExit("dry run")
r.gripper(0.04)
# go via an intermediate high pose to avoid sweeping low: first raise tcp to 1.25 at current xy with current R
pos, Rc = r.fk_hand()
r.move_q(q1, 8.0)
t,_ = r.tcp(); print("tcp", np.round(t,3), "force", np.round(r.force(),2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png"); r.snap("agentview")
EOF
timeout 300 python3 -u side2.py

# openrua op 34
cat >> rob.py <<'EOF'


def slerp_R(R0, R1, t):
    """Interpolate rotation matrices via axis-angle."""
    from scipy.spatial.transform import Rotation as Rot, Slerp
    s = Slerp([0, 1], Rot.from_matrix(np.stack([R0, R1])))
    return s([t])[0].as_matrix()


def cart_path(r, tcp0, R0, tcp1, R1, n, seed, max_step=0.7):
    """IK along a straight Cartesian TCP path; returns list of q or None."""
    qs = []; q = seed
    for i in range(1, n + 1):
        t = i / n
        p = np.asarray(tcp0) * (1 - t) + np.asarray(tcp1) * t
        R = slerp_R(R0, R1, t)
        qn = None
        for _ in range(4):
            qn = r.ik_tcp(p, R, seed=q)
            if qn is not None and np.abs(qn - q).max() < max_step: break
            qn = None
        if qn is None:
            print(f"cart_path: IK/continuity failed at t={t:.2f}", flush=True); return None
        qs.append(qn); q = qn
    return qs
EOF
cat > transit.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("tr")
R1 = np.load("snaps/R_side.npy"); q1, qg, ql = np.load("snaps/q_chain.npy")
q0 = r.arm_q(); tcp0, R0 = r.tcp()
tcp1, _ = r.tcp(q1)
print("from", np.round(tcp0,3), "to", np.round(tcp1,3))
# stage A: rise to z=1.2 keeping orientation
a = cart_path(r, tcp0, R0, [tcp0[0], tcp0[1], 1.2], R0, 2, q0)
# stage B: to above target at 1.2 with target orientation
b = cart_path(r, [tcp0[0], tcp0[1], 1.2], R0, [tcp1[0], tcp1[1], 1.2], R1, 8, a[-1]) if a else None
# stage C: descend to pre-grasp
c = cart_path(r, [tcp1[0], tcp1[1], 1.2], R1, tcp1, R1, 3, b[-1]) if b else None
if not (a and b and c): raise SystemExit("path failed")
path = a + b + c
print("path joint steps:", [round(float(np.abs(path[i]-(path[i-1] if i else q0)).max()),2) for i in range(len(path))])
print("final vs q1 dist", round(np.abs(path[-1]-q1).max(),2))
if "--go" not in sys.argv: raise SystemExit("dry run")
r.gripper(0.04)
r.move_q(path[-1], 3.0*len(path), via=path[:-1])
t,_ = r.tcp(); print("tcp", np.round(t,3), "force", np.round(r.force(),2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png"); r.snap("agentview")
EOF
timeout 600 python3 -u transit.py

# openrua op 35
timeout 900 python3 -u transit.py --go 2>&1 | tail -5

# openrua op 36
cat > grasp2.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("g2")
R1 = np.load("snaps/R_side.npy")
BOT = np.array([-0.166, 0.073]); ZG = 1.04; az=np.radians(60)
q0 = r.arm_q()
pre = np.array([BOT[0]-0.07*np.cos(az), BOT[1]-0.07*np.sin(az), ZG])
for i in range(3):
    q = r.ik_tcp(pre, R1, seed=q0)
    if q is not None and np.abs(q-q0).max() < 0.5: break
r.move_q(q, 3.0); t,_ = r.tcp(); print("pre tcp", np.round(t,3))
if np.abs(t-pre).max() > 0.01:
    r.move_q(q, 3.0); t,_ = r.tcp(); print("pre tcp (resent)", np.round(t,3))
# advance along Z to grasp in 2 steps
path = cart_path(r, pre, R1, [BOT[0], BOT[1], ZG], R1, 3, q, max_step=0.4)
if path is None: raise SystemExit
r.move_q(path[-1], 4.0, via=path[:-1]); t,_ = r.tcp(); print("grasp tcp", np.round(t,3), "force", np.round(r.force(),2))
if np.abs(t-[BOT[0],BOT[1],ZG]).max() > 0.008:
    r.move_q(path[-1], 3.0); t,_ = r.tcp(); print("grasp tcp (resent)", np.round(t,3))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_grasp.png")
r.gripper(0.0)
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
r.snap("sideview")
EOF
timeout 900 python3 -u grasp2.py 2>&1 | grep -v "^move_q" ; echo; grep -c . /dev/null

# openrua op 37
timeout 300 python3 state.py sideview agentview 2>&1 | tail -8

# openrua op 38
grep -n "def " rob.py | head -50 && cat grasp2.py

# openrua op 39
mkdir -p "$(dirname /workspace/goto.py)"
cat > /workspace/goto.py <<'OPENRUA_EOF'
"""Robust TCP goto: IK (verified by FK), move, measure, resend until within tol."""
import numpy as np
from rob import *

LIM = np.array(M["actuators"][0]["limits_rad"])


def margin(q):
    return float(min((q - LIM[:, 0]).min(), (LIM[:, 1] - q).min()))


def ik_checked(r, tcp, R, seed, max_dist=0.8, tries=6):
    best = None
    for i in range(tries):
        s = seed if i == 0 else seed + np.random.uniform(-0.15, 0.15, 7)
        q = r.ik_tcp(tcp, R, seed=s)
        if q is None:
            continue
        t, Rq = r.tcp(q)
        if np.abs(t - tcp).max() > 0.003 or np.abs(Rq - R).max() > 0.05:
            continue
        d = np.abs(q - seed).max()
        if d < max_dist and margin(q) > 0.1:
            if best is None or d < best[0]:
                best = (d, q)
    return None if best is None else best[1]


def goto(r, tcp, R, seconds=4.0, tol=0.008, max_dist=0.8, resend=3, via_n=0):
    tcp = np.asarray(tcp, float)
    q0 = r.arm_q()
    if via_n > 0:
        t0, R0 = r.tcp()
        path = cart_path(r, t0, R0, tcp, R, via_n + 1, q0, max_step=0.5)
        if path is None:
            print("cart_path failed, falling back to direct IK", flush=True)
            via_n = 0
        else:
            q = path[-1]
            via = path[:-1]
    if via_n == 0:
        q = ik_checked(r, tcp, R, q0, max_dist=max_dist)
        via = None
        if q is None:
            print(f"IK failed for {np.round(tcp,3)}", flush=True)
            return None
    print(f"goto {np.round(tcp,3)}: dq={np.round(np.abs(q-q0).max(),3)} margin={margin(q):.2f}", flush=True)
    r.move_q(q, seconds, via=via)
    for i in range(resend):
        t, _ = r.tcp()
        err = np.abs(t - tcp).max()
        print(f"  tcp {np.round(t,4)} err={err:.4f}", flush=True)
        if err < tol:
            break
        r.move_q(q, max(2.0, seconds / 2))
    return q
OPENRUA_EOF

# openrua op 40
mkdir -p "$(dirname /workspace/grasp3.py)"
cat > /workspace/grasp3.py <<'OPENRUA_EOF'
import numpy as np
from rob import *
from goto import goto

r = Robot("g3")
R1 = np.load("snaps/R_side.npy")
BOT = np.array([-0.166, 0.073]); ZG = 1.04; az = np.radians(60)
r.gripper(0.04)
t, Rn = r.tcp(); print("start tcp", np.round(t, 3))
# back off along -Z of hand a bit and go to pre-grasp
pre = np.array([BOT[0] - 0.07 * np.cos(az), BOT[1] - 0.07 * np.sin(az), ZG])
goto(r, [pre[0] - 0.03 * np.cos(az), pre[1] - 0.03 * np.sin(az), ZG + 0.03], R1, seconds=4)
goto(r, pre, R1, seconds=3)
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png")
r.snap("agentview")
OPENRUA_EOF

# openrua op 41
timeout 600 python3 grasp3.py 2>&1 | grep -v "^\[" 

# openrua op 42
cat > dq.py <<'EOF'
import numpy as np
from rob import *
from goto import ik_checked
r = Robot("dq")
R1 = np.load("snaps/R_side.npy")
q0 = r.arm_q()
q = ik_checked(r, [-0.201, 0.012, 1.04], R1, q0)
print("q0", np.round(q0,3)); print("q ", np.round(q,3)); print("diff", np.round(q-q0,3))
print("force", np.round(r.force(),2))
EOF
timeout 120 python3 dq.py 2>&1 | grep -v "^\["

# openrua op 43
cat > links.py <<'EOF'
import numpy as np, sys
from rob import *
from goto import ik_checked
r = Robot("lk")
def fk_links(q):
    req = GetPositionFK.Request(); req.header.frame_id=""
    req.fk_link_names = [f"panda_link{i}" for i in range(1,9)]+["panda_hand","panda_leftfinger","panda_rightfinger"]
    req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[float(v) for v in q]
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    res=fut.result()
    for n,p in zip(req.fk_link_names,res.pose_stamped):
        print(f"  {n:18s} {p.pose.position.x:7.3f} {p.pose.position.y:7.3f} {p.pose.position.z:7.3f}")
q0=r.arm_q(); print("current"); fk_links(q0)
R1=np.load("snaps/R_side.npy")
q=ik_checked(r,[-0.201,0.012,1.04],R1,q0); print("target"); fk_links(q)
EOF
timeout 120 python3 links.py 2>&1 | grep -v "^\["

# openrua op 44
timeout 120 python3 state.py sideview frontview robot0_eye_in_hand 2>&1 | grep -v "^\["

# openrua op 45
grep -n "^from\|^import" rob.py | head -20

# openrua op 46
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace("from trajectory_msgs.msg import JointTrajectoryPoint","from trajectory_msgs.msg import JointTrajectoryPoint\nfrom control_msgs.msg import JointTolerance")
s=s.replace("""        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)""","""        goal.trajectory.points = pts
        # loose path tolerance so lag does not abort the motion mid-way
        goal.path_tolerance = [JointTolerance(name=j, position=2.0) for j in JOINTS]
        goal.goal_tolerance = [JointTolerance(name=j, position=0.01) for j in JOINTS]
        goal.goal_time_tolerance = Duration(sec=5)
        send = self.fjt.send_goal_async(goal)""")
open('rob.py','w').write(s)
EOF
grep -n "tolerance" rob.py

# openrua op 47
cat > try_pre.py <<'EOF'
import numpy as np
from rob import *
from goto import goto
r = Robot("tp")
R1 = np.load("snaps/R_side.npy")
goto(r, [-0.201, 0.012, 1.04], R1, seconds=4, resend=2)
print("q", np.round(r.arm_q(),3), "force", np.round(r.force(),2))
EOF
timeout 300 python3 try_pre.py 2>&1 | grep -v "^\["

# openrua op 48
cat > watch.py <<'EOF'
import numpy as np
from rob import *
from goto import ik_checked
r = Robot("w")
R1 = np.load("snaps/R_side.npy")
q0 = r.arm_q()
q = ik_checked(r, [-0.201, 0.012, 1.04], R1, q0)
print("q0", np.round(q0,3)); print("qt", np.round(q,3))
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = list(JOINTS)
pt = JointTrajectoryPoint(positions=[float(x) for x in q]); pt.time_from_start = Duration(sec=4)
goal.trajectory.points = [pt]
send = r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(r.node, send, timeout_sec=60)
gh = send.result(); rf = gh.get_result_async()
t0 = time.time()
while not rf.done():
    rclpy.spin_once(r.node, timeout_sec=0.05)
    if time.time()-t0 > 0.5:
        t0 = time.time()
        js = r.js
        print(f"{js.header.stamp.sec}.{js.header.stamp.nanosec//10**8} j2={r.arm_q()[1]:.3f} err={np.round(r.arm_q()-q,3)} F={np.round(r.force(),1)}", flush=True)
res = rf.result().result
print("code", res.error_code, res.error_string)
EOF
grep -n "self.js\b\|self.js =" rob.py | head -3; timeout 300 python3 watch.py 2>&1 | grep -v "^\["

# openrua op 49
cat heightmap.py | head -40; python3 - <<'EOF'
import numpy as np
hm=np.load('snaps/hm.npy'); print(hm.shape)
x0,y0,res=-0.45,-0.35,0.005
# print heightmap coarse over x -0.45..0.0, y -0.35..0.0
for xi in range(0,90,5):
    row=""
    for yi in range(0,70,5):
        v=hm[xi:xi+5,yi:yi+5]; v=v[np.isfinite(v)]
        row+=f"{(v.max() if v.size else 0):5.2f}"
    print(f"x={x0+xi*res:5.2f} "+row)
print("cols y=",[round(y0+yi*res,2) for yi in range(0,70,5)])
EOF

# openrua op 50
mkdir -p "$(dirname /workspace/coll.py)"
cat > /workspace/coll.py <<'OPENRUA_EOF'
"""Crude arm-vs-scene collision check using link FK and the table heightmap."""
import numpy as np
from rob import *

HM = np.load("snaps/hm.npy")
X0, Y0, RES = -0.45, -0.35, 0.005
HM = np.where(HM > 1.22, np.nan, HM)  # strip the robot's own points from the old cloud
BOT = np.array([-0.166, 0.073])
LINKS = [f"panda_link{i}" for i in range(1, 9)] + ["panda_hand"]
# capsules: (link a, link b, radius)
CAPS = [("panda_link3", "panda_link4", 0.06), ("panda_link4", "panda_link5", 0.06),
        ("panda_link5", "panda_link6", 0.06), ("panda_link6", "panda_link7", 0.05),
        ("panda_link7", "panda_hand", 0.045)]


def fk_all(r, q):
    req = GetPositionFK.Request(); req.header.frame_id = ""
    req.fk_link_names = LINKS
    req.robot_state.joint_state.name = list(JOINTS)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    return {n: np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
            for n, p in zip(LINKS, res.pose_stamped)}


def height_at(x, y, rad):
    i0 = int((x - rad - X0) / RES); i1 = int((x + rad - X0) / RES) + 1
    j0 = int((y - rad - Y0) / RES); j1 = int((y + rad - Y0) / RES) + 1
    i0, j0 = max(i0, 0), max(j0, 0)
    if i1 <= i0 or j1 <= j0 or i0 >= HM.shape[0] or j0 >= HM.shape[1]:
        return 0.90
    v = HM[i0:i1, j0:j1]; v = v[np.isfinite(v)]
    return float(v.max()) if v.size else 0.90


def clearance(r, q, ignore_bottle=True):
    """Min vertical clearance (m) of arm capsules above scene heightmap. Negative = collision."""
    P = fk_all(r, q)
    worst = 9.0; where = None
    for a, b, rad in CAPS:
        pa, pb = P[a], P[b]
        for t in np.linspace(0, 1, 6):
            p = pa + t * (pb - pa)
            if ignore_bottle and np.hypot(p[0] - BOT[0], p[1] - BOT[1]) < 0.05 + rad:
                continue
            h = height_at(p[0], p[1], rad)
            c = (p[2] - rad) - h
            if c < worst:
                worst, where = c, (a, np.round(p, 3), round(h, 3))
    return worst, where
OPENRUA_EOF

# openrua op 51
cat > search3.py <<'EOF'
import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import clearance
r = Robot("s3")
q0 = r.arm_q()
c, w = clearance(r, q0); print("current clearance", round(c,3), w)
BOT = np.array([-0.166, 0.073]); ZG = 1.045
results = []
for az_deg in [45, 60, 75, 90, 105]:
    for tilt_deg in [0, 20, 35, 50]:
        az = np.radians(az_deg); tilt = np.radians(tilt_deg)
        d = np.array([np.cos(az), np.sin(az), 0.0])
        Z = np.array([np.cos(tilt)*d[0], np.cos(tilt)*d[1], -np.sin(tilt)])
        Y = np.array([-d[1], d[0], 0.0])         # closing axis horizontal
        X = np.cross(Y, Z)
        R = np.column_stack([X, Y, Z])
        g = np.array([BOT[0], BOT[1], ZG])
        pre = g - 0.07 * Z
        qg = ik_checked(r, g, R, q0, max_dist=3.0)
        if qg is None: print(az_deg, tilt_deg, "no IK grasp"); continue
        qp = ik_checked(r, pre, R, qg, max_dist=0.6)
        if qp is None: print(az_deg, tilt_deg, "no IK pre"); continue
        cg, wg = clearance(r, qg); cp, wp = clearance(r, qp)
        print(f"az={az_deg} tilt={tilt_deg} margin={margin(qg):.2f}/{margin(qp):.2f} clear={cg:.3f}/{cp:.3f} dq0={np.abs(qg-q0).max():.2f} worst={wg}")
        results.append((az_deg, tilt_deg, qg, qp, R))
np.save("snaps/search3.npy", np.array(results, dtype=object), allow_pickle=True)
EOF
timeout 900 python3 search3.py 2>&1 | grep -v "^\["

# openrua op 52
python3 - <<'EOF'
import numpy as np
hm=np.load('snaps/hm.npy'); x0,y0,res=-0.45,-0.35,0.005
for xi in range(30,90,4):
    row=""
    for yi in range(30,100,4):
        v=hm[xi:xi+4,yi:yi+4]; v=v[np.isfinite(v)]
        row+=f"{(v.max() if v.size else 0):5.2f}"
    print(f"x={x0+xi*res:5.2f} "+row)
print("cols y=",[round(y0+yi*res,2) for yi in range(30,100,4)])
EOF

# openrua op 53
python3 - <<'EOF'
s=open('coll.py').read()
s=s.replace("HM = np.where(HM > 1.22, np.nan, HM)  # strip the robot's own points from the old cloud",
"""HM = np.where(HM > 1.22, np.nan, HM)  # strip the robot's own points from the old cloud
_xs = X0 + RES * np.arange(HM.shape[0])[:, None]; _ys = Y0 + RES * np.arange(HM.shape[1])[None, :]
_robot = (HM > 1.15) & (_xs > -0.30) & (_xs < -0.14) & (_ys > -0.15) & (_ys < 0.15)
HM = np.where(_robot, np.nan, HM)""")
open('coll.py','w').write(s)
EOF
timeout 900 python3 search3.py 2>&1 | grep -v "^\["

# openrua op 54
cat > search4.py <<'EOF'
import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
r = Robot("s4")
q0 = r.arm_q()
BOT = np.array([-0.166, 0.073]); ZG = 1.045
def dist_boards(p, rad):
    # boards AABB: x[-0.26,-0.03] y[-0.36,-0.19] z[0.90,1.22]
    lo=np.array([-0.26,-0.36,0.90]); hi=np.array([-0.03,-0.19,1.22])
    d=np.linalg.norm(np.maximum(0,np.maximum(lo-p,p-hi))); return d-rad
def dist_bowl(p, rad):
    if p[2]-rad > 1.03: return 9
    return np.hypot(p[0]+0.01,p[1]+0.075)-0.085-rad
def dist_cab(p, rad):
    lo=np.array([-0.18,0.22,0.90]); hi=np.array([0.14,0.42,1.13])
    return np.linalg.norm(np.maximum(0,np.maximum(lo-p,p-hi)))-rad
def check(q):
    P=fk_all(r,q); worst=(9,None)
    for a,b,rad in CAPS:
        for t in np.linspace(0,1,6):
            p=P[a]+t*(P[b]-P[a])
            for nm,f in (("boards",dist_boards),("bowl",dist_bowl),("cab",dist_cab)):
                d=f(p,rad)
                if d<worst[0]: worst=(d,(nm,a,np.round(p,3)))
            d=p[2]-rad-0.902
            if d<worst[0]: worst=(d,("table",a,np.round(p,3)))
    return worst
print("current", check(q0))
res=[]
for az_deg in [75,90,105,120]:
    for tilt_deg in [15,25,35]:
        az=np.radians(az_deg); tilt=np.radians(tilt_deg)
        d=np.array([np.cos(az),np.sin(az),0.0])
        Z=np.array([np.cos(tilt)*d[0],np.cos(tilt)*d[1],-np.sin(tilt)]); Y=np.array([-d[1],d[0],0.0]); X=np.cross(Y,Z)
        R=np.column_stack([X,Y,Z]); g=np.array([BOT[0],BOT[1],ZG]); pre=g-0.08*Z
        qg=ik_checked(r,g,R,q0,max_dist=3.0)
        if qg is None: print(az_deg,tilt_deg,"no IK grasp"); continue
        qp=ik_checked(r,pre,R,qg,max_dist=0.6)
        if qp is None: print(az_deg,tilt_deg,"no IK pre"); continue
        cg=check(qg); cp=check(qp)
        print(f"az={az_deg} tilt={tilt_deg} margin={margin(qg):.2f}/{margin(qp):.2f} dq0={np.abs(qg-q0).max():.2f}\n   grasp {cg}\n   pre   {cp}")
        res.append((az_deg,tilt_deg,qg,qp,R,cg[0],cp[0]))
np.save("snaps/search4.npy", np.array(res,dtype=object), allow_pickle=True)
EOF
timeout 900 python3 search4.py 2>&1 | grep -v "^\["

# openrua op 55
cat > search5.py <<'EOF'
import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
exec(open('search4.py').read().split("print(\"current\"")[0].split("r = Robot")[0])  # imports only
r = Robot("s5"); q0 = r.arm_q()
BOT = np.array([-0.166, 0.073]); ZG = 1.045
def dist_boards(p, rad):
    lo=np.array([-0.26,-0.36,0.90]); hi=np.array([-0.03,-0.19,1.22])
    return np.linalg.norm(np.maximum(0,np.maximum(lo-p,p-hi)))-rad
def dist_bowl(p, rad):
    if p[2]-rad > 1.03: return 9
    return np.hypot(p[0]+0.01,p[1]+0.075)-0.085-rad
def dist_cab(p, rad):
    lo=np.array([-0.18,0.22,0.90]); hi=np.array([0.14,0.42,1.13])
    return np.linalg.norm(np.maximum(0,np.maximum(lo-p,p-hi)))-rad
def check(q):
    P=fk_all(r,q); worst=(9,None)
    for a,b,rad in CAPS:
        for t in np.linspace(0,1,6):
            p=P[a]+t*(P[b]-P[a])
            for nm,f in (("boards",dist_boards),("bowl",dist_bowl),("cab",dist_cab)):
                d=f(p,rad)
                if d<worst[0]: worst=(d,(nm,a,np.round(p,3)))
            d=p[2]-rad-0.902
            if d<worst[0]: worst=(d,("table",a,np.round(p,3)))
    return worst
res=[]
for az_deg in [120,135,150]:
    for tilt_deg in [25,30]:
        az=np.radians(az_deg); tilt=np.radians(tilt_deg)
        d=np.array([np.cos(az),np.sin(az),0.0])
        Z=np.array([np.cos(tilt)*d[0],np.cos(tilt)*d[1],-np.sin(tilt)]); Y=np.array([-d[1],d[0],0.0]); X=np.cross(Y,Z)
        R=np.column_stack([X,Y,Z]); g=np.array([BOT[0],BOT[1],ZG])
        qg=ik_checked(r,g,R,q0,max_dist=3.0)
        if qg is None: print(az_deg,tilt_deg,"no IK grasp"); continue
        cg=check(qg)
        print(f"az={az_deg} tilt={tilt_deg} margin={margin(qg):.2f} dq0={np.abs(qg-q0).max():.2f} grasp {cg}")
        for dz in [0.0,0.03,0.05]:
            pre=g-0.07*Z+np.array([0,0,dz])
            qp=ik_checked(r,pre,R,qg,max_dist=0.6)
            if qp is None: print("   pre dz",dz,"no IK"); continue
            print(f"   pre dz={dz} margin={margin(qp):.2f} dq={np.abs(qp-qg).max():.2f} {check(qp)}")
            res.append((az_deg,tilt_deg,dz,qg,qp,R))
np.save("snaps/search5.npy", np.array(res,dtype=object), allow_pickle=True)
EOF
timeout 900 python3 search5.py 2>&1 | grep -v "^\["

# openrua op 56
sed -n 285,330p rob.py

# openrua op 57
mkdir -p "$(dirname /workspace/grasp4.py)"
cat > /workspace/grasp4.py <<'OPENRUA_EOF'
import numpy as np
from rob import *
from goto import goto, ik_checked, margin

r = Robot("g4")
BOT = np.array([-0.166, 0.073]); ZG = 1.045
az = np.radians(135); tilt = np.radians(30)
d = np.array([np.cos(az), np.sin(az), 0.0])
Z = np.array([np.cos(tilt) * d[0], np.cos(tilt) * d[1], -np.sin(tilt)])
Y = np.array([-d[1], d[0], 0.0]); X = np.cross(Y, Z)
R2 = np.column_stack([X, Y, Z])
np.save("snaps/R_g4.npy", R2)
g = np.array([BOT[0], BOT[1], ZG])
pre = g - 0.07 * Z + np.array([0, 0, 0.03])
above = pre + np.array([0, 0, 0.09])

t0, R0 = r.tcp(); print("start tcp", np.round(t0, 3))
if r.finger_gap() < 0.07:
    r.gripper(0.04)
# 1. lift straight up with current orientation
goto(r, t0 + np.array([0, 0, 0.11]), R0, seconds=3, via_n=1, resend=1)
# 2. transit at height to above pre-grasp, rotating to R2
goto(r, above, R2, seconds=5, via_n=3, resend=1)
# 3. descend to pre
goto(r, pre, R2, seconds=3, via_n=1, resend=2)
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre4.png")
r.snap("agentview")
print("q", np.round(r.arm_q(), 3), "force", np.round(r.force(), 2))
OPENRUA_EOF

# openrua op 58
timeout 900 python3 grasp4.py 2>&1 | grep -v "^\["

# openrua op 59
mkdir -p "$(dirname /workspace/grasp4b.py)"
cat > /workspace/grasp4b.py <<'OPENRUA_EOF'
import numpy as np
from rob import *
from goto import goto

r = Robot("g4b")
R2 = np.load("snaps/R_g4.npy")
BOT = np.array([-0.166, 0.073]); ZG = 1.045
g = np.array([BOT[0], BOT[1], ZG])
goto(r, g, R2, seconds=4, via_n=2, resend=2, tol=0.005)
print("force", np.round(r.force(), 2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_g4.png")
r.snap("sideview")
r.gripper(0.0)
gap = r.finger_gap(); print("gap", round(gap, 4), "force", np.round(r.force(), 2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_g4c.png")
OPENRUA_EOF

# openrua op 60
timeout 900 python3 grasp4b.py 2>&1 | grep -v "^\["

# openrua op 61
cat > lift4.py <<'EOF'
import numpy as np
from rob import *
from goto import goto
r = Robot("l4")
t0, R0 = r.tcp()
goto(r, t0 + np.array([0,0,0.13]), R0, seconds=4, via_n=2, resend=1)
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
r.snap("sideview"); r.snap("agentview")
EOF
timeout 600 python3 lift4.py 2>&1 | grep -v "^\["

# openrua op 62
python3 - <<'EOF'
import numpy as np
hm=np.load('snaps/hm.npy'); x0,y0,res=-0.45,-0.35,0.005
xs=x0+res*np.arange(hm.shape[0]); ys=y0+res*np.arange(hm.shape[1])
# region y 0.15..0.26, x -0.2..0.15
xi=np.where((xs>-0.2)&(xs<0.15))[0]; yi=np.where((ys>0.15)&(ys<0.26))[0]
print("cols y=", [round(ys[j],3) for j in yi[::2]])
for i in xi[::2]:
    print(f"x={xs[i]:6.3f} "+" ".join(f"{hm[i,j]:.2f}" if np.isfinite(hm[i,j]) else " nan" for j in yi[::2]))
EOF

# openrua op 63
cat > profile.py <<'EOF'
import numpy as np
from rob import *
r = Robot("pf")
pts=[]
for cam in ["agentview","frontview","sideview","galleryview"]:
    try:
        img, P = r.cloud(cam)
    except Exception as e:
        print(cam, "fail", e); continue
    P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.23)&(P[:,0]<-0.10)&(P[:,1]>0.0)&(P[:,1]<0.15)&(P[:,2]>1.0)&(P[:,2]<1.30)
    print(cam, m.sum()); pts.append(P[m])
P=np.concatenate(pts)
np.save("snaps/bottle_air.npy",P)
G=np.array([-0.166,0.073])
for z in np.arange(1.02,1.28,0.01):
    s=P[(P[:,2]>=z)&(P[:,2]<z+0.01)]
    if len(s)==0: print(f"z={z:.2f} none"); continue
    rr=np.hypot(s[:,0]-G[0],s[:,1]-G[1])
    print(f"z={z:.2f} n={len(s):4d} cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f} r_med={np.median(rr):.3f} r90={np.percentile(rr,90):.3f}")
EOF
timeout 300 python3 profile.py 2>&1 | grep -v "^\["

# openrua op 64
python3 - <<'EOF'
import numpy as np
P=np.load("snaps/bottle_air.npy")
for z in np.arange(1.02,1.18,0.01):
    s=P[(P[:,2]>=z)&(P[:,2]<z+0.01)]
    if len(s)==0: continue
    print(f"z={z:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] dx={s[:,0].max()-s[:,0].min():.3f} dy={s[:,1].max()-s[:,1].min():.3f}")
EOF

# openrua op 65
python3 - <<'EOF'
import numpy as np
hm=np.load('snaps/hm.npy'); x0,y0,res=-0.45,-0.35,0.005
xs=x0+res*np.arange(hm.shape[0]); ys=y0+res*np.arange(hm.shape[1])
xi=np.where((xs>-0.13)&(xs<0.14))[0]; yi=np.where((ys>0.02)&(ys<0.13))[0]
print("cols y=", [round(ys[j],3) for j in yi[::2]])
for i in xi[::2]:
    print(f"x={xs[i]:6.3f} "+" ".join(f"{hm[i,j]:.2f}" if np.isfinite(hm[i,j]) else " nan" for j in yi[::2]))
EOF

# openrua op 66
mkdir -p "$(dirname /workspace/place_plan.py)"
cat > /workspace/place_plan.py <<'OPENRUA_EOF'
import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
from scipy.spatial.transform import Rotation as Rot

r = Robot("pp")
q0 = r.arm_q(); t0, R0 = r.tcp()
print("tcp", np.round(t0, 3))
b = R0.T @ np.array([0, 0, 1.0])          # bottle axis (base->tip) in hand frame
G_REL = np.array([-0.0775, 0.15, 0.962])   # tip/grasp point at release (1.5 cm above rest)


def dist_aabb(p, lo, hi, rad):
    return np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi))) - rad


def check(q):
    P = fk_all(r, q); worst = (9, None)
    obs = [("cab", np.array([-0.18, 0.215, 0.90]), np.array([0.14, 0.42, 1.13])),
           ("handles", np.array([-0.05, 0.185, 0.99]), np.array([0.04, 0.215, 1.11])),
           ("boards", np.array([-0.26, -0.36, 0.90]), np.array([-0.03, -0.19, 1.22])),
           ("drawerL", np.array([-0.115, 0.075, 0.90]), np.array([-0.10, 0.25, 0.985])),
           ("drawerR", np.array([0.10, 0.075, 0.90]), np.array([0.115, 0.25, 0.985])),
           ("drawerF", np.array([-0.115, 0.075, 0.90]), np.array([0.115, 0.095, 0.985]))]
    for a, bb, rad in CAPS + [("panda_hand", "panda_hand", 0.09)]:
        for t in np.linspace(0, 1, 6):
            p = P[a] + t * (P[bb] - P[a])
            for nm, lo, hi in obs:
                d = dist_aabb(p, lo, hi, rad)
                if d < worst[0]: worst = (d, (nm, a, np.round(p, 3)))
            d = p[2] - rad - 0.902
            if d < worst[0]: worst = (d, ("table", a, np.round(p, 3)))
            if p[2] - rad < 1.03:
                d = np.hypot(p[0] + 0.01, p[1] + 0.075) - 0.085 - rad
                if d < worst[0]: worst = (d, ("bowl", a, np.round(p, 3)))
    return worst


def R_for(u, phi):
    ax = np.cross([0, 0, 1.0], u); ax /= np.linalg.norm(ax)
    R_align = Rot.from_rotvec(ax * np.pi / 2).as_matrix()
    return Rot.from_rotvec(np.asarray(u) * phi).as_matrix() @ R_align @ R0


best = []
for u in ([-1.0, 0, 0], [1.0, 0, 0]):
    u = np.array(u)
    for phi_deg in range(0, 360, 20):
        R = R_for(u, np.radians(phi_deg))
        assert np.allclose(R @ b, u, atol=1e-6)
        Z = R[:, 2]
        if Z[2] > -0.4: continue
        q = ik_checked(r, G_REL, R, q0, max_dist=4.0)
        if q is None: print(u, phi_deg, "Zz", round(Z[2], 2), "no IK"); continue
        c = check(q)
        qh = ik_checked(r, G_REL + [0, 0, 0.14], R, q, max_dist=1.0)
        ch = check(qh) if qh is not None else (None, None)
        print(f"u={u} phi={phi_deg} Z={np.round(Z,2)} Y={np.round(R[:,1],2)} margin={margin(q):.2f} clear={c[0]:.3f} {c[1]} | high {ch}")
        if qh is not None:
            best.append((min(c[0], ch[0]), margin(q), u, phi_deg, R, q, qh))
best.sort(key=lambda x: -x[0])
for bst in best[:5]:
    print("BEST", bst[0], bst[1], bst[2], bst[3])
np.save("snaps/place_best.npy", np.array(best[:5], dtype=object), allow_pickle=True)
OPENRUA_EOF

# openrua op 67
timeout 900 python3 place_plan.py 2>&1 | grep -v "^\["

# openrua op 68
python3 - <<'EOF'
s=open('place_plan.py').read()
s=s.replace("""    for a, bb, rad in CAPS + [("panda_hand", "panda_hand", 0.09)]:
        for t in np.linspace(0, 1, 6):
            p = P[a] + t * (P[bb] - P[a])""","""    _, Rh = r.fk_hand(q); ph = P["panda_hand"]
    hand_pts = [(ph + Rh @ np.array([sx * 0.0316, sy * 0.1013, sz]), 0.005)
                for sx in (-1, 1) for sy in (-1, 1) for sz in (0.0, 0.066)]
    hand_pts += [(ph + Rh @ np.array([sx * 0.01, sy * 0.045, 0.105]), 0.005)
                 for sx in (-1, 1) for sy in (-1, 1)]
    hand_pts += [(ph + Rh @ np.array([0, 0, -0.05]), 0.05)]  # wrist flange behind the hand
    samples = [(P[a] + t * (P[bb] - P[a]), rad) for a, bb, rad in CAPS for t in np.linspace(0, 1, 6)]
    samples += [(p, rad, ) for p, rad in hand_pts]
    for p, rad in samples:
        if True:
            a = "hand" if rad <= 0.05 else "arm\"""")
s=s.replace('for phi_deg in range(0, 360, 20):','for phi_deg in range(0, 360, 10):')
open('place_plan.py','w').write(s)
EOF
grep -n 'a = "hand"' place_plan.py; timeout 900 python3 place_plan.py 2>&1 | grep -v "^\[" | grep -v "no IK"

# openrua op 69
python3 - <<'EOF'
s=open('place_plan.py').read()
s=s.replace("G_REL = np.array([-0.0775, 0.15, 0.962])   # tip/grasp point at release (1.5 cm above rest)",
"C = np.array([0.0, 0.16, 0.962])   # bottle centre at release (1.5 cm above rest)")
s=s.replace("for u in ([-1.0, 0, 0], [1.0, 0, 0]):\n    u = np.array(u)",
"for u in ([-0.866, -0.5, 0], [0.866, -0.5, 0], [-0.94, -0.34, 0], [0.94, -0.34, 0], [-1.0, 0, 0]):\n    u = np.array(u); G_REL = C + 0.0725 * u")
s=s.replace("best.append((min(c[0], ch[0]), margin(q), u, phi_deg, R, q, qh))","best.append((min(c[0], ch[0]), margin(q), u, phi_deg, R, q, qh, G_REL))")
open('place_plan.py','w').write(s)
EOF
timeout 1200 python3 place_plan.py 2>&1 | grep -v "^\[" | grep -v "no IK"

# openrua op 70
sed -i 's/    u = np.array(u); G_REL = C + 0.0725 \* u/    u = np.array(u); u = u \/ np.linalg.norm(u); G_REL = C + 0.0725 * u/' place_plan.py && timeout 1200 python3 place_plan.py 2>&1 | grep -v "^\[" | grep -v "no IK"

# openrua op 71
python3 - <<'EOF'
s=open('place_plan.py').read()
s=s.replace("C = np.array([0.0, 0.16, 0.962])   # bottle centre at release (1.5 cm above rest)",
"CS = {(-0.866,-0.5,0): np.array([0.0, 0.18, 0.972]), (-0.94,-0.34,0): np.array([0.0, 0.17, 0.972]), (-1.0,0,0): np.array([0.0,0.17,0.972])}")
s=s.replace("for u in ([-0.866, -0.5, 0], [0.866, -0.5, 0], [-0.94, -0.34, 0], [0.94, -0.34, 0], [-1.0, 0, 0]):\n    u = np.array(u); u = u / np.linalg.norm(u); G_REL = C + 0.0725 * u",
"for uk, C in CS.items():\n    u = np.array(uk); u = u / np.linalg.norm(u); G_REL = C + 0.0725 * u")
open('place_plan.py','w').write(s)
EOF
timeout 1200 python3 place_plan.py 2>&1 | grep -v "^\[" | grep -v "no IK"

# openrua op 72
cat > handles.py <<'EOF'
import numpy as np
from rob import *
r = Robot("hd")
pts=[]
for cam in ["agentview","frontview","sideview","birdview"]:
    img,P=r.cloud(cam); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.12)&(P[:,0]<0.12)&(P[:,1]>0.15)&(P[:,1]<0.222)&(P[:,2]>0.93)&(P[:,2]<1.13)
    pts.append(P[m]); print(cam, m.sum())
P=np.concatenate(pts)
for z in np.arange(0.93,1.13,0.01):
    s=P[(P[:,2]>=z)&(P[:,2]<z+0.01)]
    if len(s)<5: print(f"z={z:.2f} n={len(s)}"); continue
    print(f"z={z:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF
timeout 300 python3 handles.py 2>&1 | grep -v "^\["

# openrua op 73
cat >> handles.py <<'EOF'
print("handles (y<0.215):")
H=P[P[:,1]<0.215]
for z in np.arange(0.99,1.12,0.01):
    s=H[(H[:,2]>=z)&(H[:,2]<z+0.01)]
    if len(s)<5: continue
    print(f"z={z:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF
timeout 300 python3 handles.py 2>&1 | grep -A20 "handles (y"

# openrua op 74
python3 - <<'EOF'
s=open('place_plan.py').read()
s=s.replace("CS = {(-0.866,-0.5,0): np.array([0.0, 0.18, 0.972]), (-0.94,-0.34,0): np.array([0.0, 0.17, 0.972]), (-1.0,0,0): np.array([0.0,0.17,0.972])}",
"CS = {(-1.0,0,0): np.array([-0.01,0.14,0.972])}")
s=s.replace("for phi_deg in range(0, 360, 10):","for phi_deg in [340,350,0,10,20]:")
s=s.replace("qh = ik_checked(r, G_REL + [0, 0, 0.14], R, q, max_dist=1.0)","qh = ik_checked(r, G_REL + [0, 0, 0.12], R, q, max_dist=1.0)")
open('place_plan.py','w').write(s)
EOF
timeout 600 python3 place_plan.py 2>&1 | grep -v "^\["

# openrua op 75
python3 - <<'EOF'
s=open('place_plan.py').read()
head=s.split("best = []")[0]
# strip the Robot creation & search prelude lines, keep functions
lines=[]
for ln in head.splitlines():
    if ln.startswith(("r = Robot","q0 = r.arm_q","print(\"tcp\"","b = R0.T","CS = ")): continue
    lines.append(ln)
scene="\n".join(lines)
scene=scene.replace("def check(q):","def check(r, q):").replace("def R_for(u, phi):","def R_for(R0, u, phi):")
open('scene.py','w').write(scene+"\n")
EOF
grep -n "^def\|^from\|^import\|r\.\|R0" scene.py | head -30

# openrua op 76
mkdir -p "$(dirname /workspace/place_exec.py)"
cat > /workspace/place_exec.py <<'OPENRUA_EOF'
import numpy as np, sys
from rob import *
from goto import ik_checked, margin
from scene import check, R_for

r = Robot("pe")
q0 = r.arm_q(); t0, R0 = r.tcp()
print("start tcp", np.round(t0, 3), "gap", round(r.finger_gap(), 4))
u = np.array([-1.0, 0, 0])
Rp = R_for(R0, u, 0.0)
np.save("snaps/R_place.npy", Rp)
G_REL = np.array([-0.0825, 0.14, 0.972])
G_HIGH = G_REL + [0, 0, 0.12]


def safe_path(tcp0, Ra, tcp1, Rb, n, seed):
    path = cart_path(r, tcp0, Ra, tcp1, Rb, n, seed, max_step=0.6)
    if path is None:
        return None
    for i, q in enumerate(path):
        c = check(r, q)
        print(f"   via {i}: margin={margin(q):.2f} clear={c[0]:.3f} {c[1]}")
        if c[0] < -0.01:
            print("   !! unsafe via point"); return None
    return path


def run(path, seconds):
    r.move_q(path[-1], seconds, via=path[:-1])
    t, _ = r.tcp(); print("   tcp", np.round(t, 4), flush=True)
    return t


# A. rotate in place
print("A rotate")
pA = safe_path(t0, R0, t0, Rp, 4, q0)
if pA is None: sys.exit("A failed")
run(pA, 6.0)
r.snap("sideview", "/workspace/snaps/sv_rot.png"); r.snap("agentview", "/workspace/snaps/av_rot.png")
print("gap", round(r.finger_gap(), 4))
# B. move to above release
print("B transit")
pB = safe_path(t0, Rp, G_HIGH, Rp, 3, r.arm_q())
if pB is None: sys.exit("B failed")
run(pB, 5.0)
r.snap("agentview", "/workspace/snaps/av_high.png")
# C. descend
print("C descend")
pC = safe_path(G_HIGH, Rp, G_REL, Rp, 3, r.arm_q())
if pC is None: sys.exit("C failed")
run(pC, 4.0)
print("gap", round(r.finger_gap(), 4), "force", np.round(r.force(), 2))
r.snap("agentview", "/workspace/snaps/av_rel.png"); r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_rel.png")
OPENRUA_EOF

# openrua op 77
timeout 1200 python3 place_exec.py 2>&1 | grep -v "^\["

# openrua op 78
cat > rot_dbg.py <<'EOF'
import numpy as np
from rob import *
from goto import ik_checked, margin
from scene import check, R_for
r = Robot("rd"); q0 = r.arm_q(); t0, R0 = r.tcp()
Rp = np.load("snaps/R_place.npy")
print("q0", np.round(q0,2))
for t in [0.25,0.5,0.75,1.0]:
    R = slerp_R(R0, Rp, t)
    for md in [0.6, 1.5, 4.0]:
        q = ik_checked(r, t0, R, q0, max_dist=md, tries=8)
        if q is not None:
            print(f"t={t} md={md} dq={np.round(q-q0,2)} margin={margin(q):.2f} clear={check(r,q)[0]:.3f}"); break
    else: print(f"t={t} no IK")
EOF
timeout 600 python3 rot_dbg.py 2>&1 | grep -v "^\["

# openrua op 79
sed -i 's/pA = safe_path(t0, R0, t0, Rp, 4, q0)/pA = safe_path(t0, R0, t0, Rp, 8, q0)/; s/run(pA, 6.0)/run(pA, 8.0)/' place_exec.py && timeout 1200 python3 place_exec.py 2>&1 | grep -v "^\["

# openrua op 80
cat > release.py <<'EOF'
import numpy as np
from rob import *
from goto import goto
r = Robot("rel")
r.gripper(0.04)
r.spin(0.5)
t0, R0 = r.tcp(); print("tcp", np.round(t0,3), "force", np.round(r.force(),2))
goto(r, t0 + [0,0,0.10], R0, seconds=3, via_n=1, resend=1)
r.snap("agentview", "/workspace/snaps/av_after.png"); r.snap("frontview", "/workspace/snaps/fv_after.png"); r.snap("birdview", "/workspace/snaps/bv_after.png")
print("force", np.round(r.force(),2))
EOF
timeout 600 python3 release.py 2>&1 | grep -v "^\["

# openrua op 81
cat > verify_in.py <<'EOF'
import numpy as np
from rob import *
r = Robot("vi")
pts=[]
for cam in ["agentview","frontview","birdview","sideview"]:
    img,P=r.cloud(cam); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.12)&(P[:,0]<0.12)&(P[:,1]>0.07)&(P[:,1]<0.26)&(P[:,2]>0.928)&(P[:,2]<1.05)
    pts.append(P[m]); print(cam, m.sum())
P=np.concatenate(pts)
# exclude the drawer walls (x beyond +-0.095, y<0.098) and the hand (z>1.0 & ... ) : keep interior points
I=P[(np.abs(P[:,0])<0.095)&(P[:,1]>0.098)&(P[:,1]<0.245)]
print("interior pts", len(I), "z max", I[:,2].max().round(3))
for z in np.arange(0.93,1.0,0.01):
    s=I[(I[:,2]>=z)&(I[:,2]<z+0.01)]
    if len(s)<5: continue
    print(f"z={z:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# anything above 0.98 within the full box footprint (rim height) besides walls?
A=P[(P[:,2]>0.985)]
print("pts above 0.985:", len(A))
if len(A): print(" x", A[:,0].min().round(3), A[:,0].max().round(3), " y", A[:,1].min().round(3), A[:,1].max().round(3), " z", A[:,2].max().round(3))
EOF
timeout 300 python3 verify_in.py 2>&1 | grep -v "^\["

# openrua op 82
mkdir -p "$(dirname /workspace/close_plan.py)"
cat > /workspace/close_plan.py <<'OPENRUA_EOF'
import numpy as np, sys
from rob import *
from goto import ik_checked, margin
from scene import check

r = Robot("cp")
q0 = r.arm_q(); t0, R0 = r.tcp()
print("tcp", np.round(t0, 3), "gap", round(r.finger_gap(), 4))
res = {}
for tilt_deg in [45, 55, 65]:
    for xpush in [0.0, 0.06]:
        t = np.radians(tilt_deg)
        Z = np.array([0, np.cos(t), -np.sin(t)]); Y = np.array([1.0, 0, 0]); X = np.cross(Y, Z)
        R = np.column_stack([X, Y, Z])
        ok = True; qs = []
        for y in [0.02, 0.10, 0.19]:
            tcp = np.array([xpush, y, 0.957])
            q = ik_checked(r, tcp, R, qs[-1] if qs else q0, max_dist=3.0 if not qs else 0.8, tries=8)
            if q is None: print(f"tilt={tilt_deg} x={xpush} y={y}: no IK"); ok = False; break
            c = check(r, q)
            print(f"tilt={tilt_deg} x={xpush} y={y}: margin={margin(q):.2f} clear={c[0]:.3f} {c[1]}")
            qs.append(q)
            if c[0] < 0.0: ok = False
        if ok: res[(tilt_deg, xpush)] = (R, qs)
np.save("snaps/close_cands.npy", np.array([(k, v[0], v[1]) for k, v in res.items()], dtype=object), allow_pickle=True)
print("feasible:", list(res.keys()))
OPENRUA_EOF

# openrua op 83
timeout 600 python3 close_plan.py 2>&1 | tail -40

# openrua op 84
mkdir -p "$(dirname /workspace/close_plan2.py)"
cat > /workspace/close_plan2.py <<'OPENRUA_EOF'
"""Plan the drawer-close push: fingertips on the bottom-drawer handle bar, hand pointing +y,
steep tilt while the wrist is over the bowl, flattening out as the hand nears the cabinet."""
import numpy as np, sys
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
from scene import dist_aabb

BOWL_C = np.array([-0.01, -0.075]); BOWL_R = 0.08; BOWL_Z = 1.02


def bowl_clear(p, rad):
    rho = np.hypot(*(p[:2] - BOWL_C))
    d_rim = np.hypot(rho - BOWL_R, p[2] - BOWL_Z) - 0.008 - rad
    if rho < BOWL_R:
        h = 0.905 + 0.115 * (rho / BOWL_R) ** 2
        return min(d_rim, p[2] - rad - h)
    if p[2] < BOWL_Z:
        return min(d_rim, rho - 0.09 - rad)
    return d_rim


def check2(r, q, shift=0.0):
    """shift = how far the bottom drawer has been pushed in (m)."""
    P = fk_all(r, q); worst = (9, None)
    s = np.array([0, shift, 0])
    obs = [("cab", np.array([-0.18, 0.215, 0.90]), np.array([0.14, 0.42, 1.13])),
           ("handles", np.array([-0.05, 0.188, 0.995]), np.array([0.04, 0.215, 1.11])),
           ("boards", np.array([-0.26, -0.36, 0.90]), np.array([-0.03, -0.19, 1.22])),
           ("drawerL", np.array([-0.115, 0.075, 0.90]) + s, np.array([-0.10, 0.25, 0.985]) + s),
           ("drawerR", np.array([0.10, 0.075, 0.90]) + s, np.array([0.115, 0.25, 0.985]) + s),
           ("drawerF", np.array([-0.115, 0.075, 0.90]) + s, np.array([0.115, 0.095, 0.985]) + s)]
    _, Rh = r.fk_hand(q); ph = P["panda_hand"]
    hand_pts = [(ph + Rh @ np.array([sx * 0.0316, sy * 0.1013, sz]), 0.005)
                for sx in (-1, 1) for sy in (-1, 1) for sz in (0.0, 0.066)]
    hand_pts += [(ph + Rh @ np.array([sx * 0.0316, 0, sz]), 0.005) for sx in (-1, 1) for sz in (0.0, 0.066)]
    hand_pts += [(ph + Rh @ np.array([0, 0, -0.05]), 0.05), (ph + Rh @ np.array([0, 0, -0.10]), 0.05)]
    samples = [(P[a] + t * (P[bb] - P[a]), rad) for a, bb, rad in CAPS for t in np.linspace(0, 1, 6)]
    samples += hand_pts
    for p, rad in samples:
        a = "hand" if rad <= 0.05 else "arm"
        for nm, lo, hi in obs:
            d = dist_aabb(p, lo, hi, rad)
            if d < worst[0]: worst = (d, (nm, a, np.round(p, 3)))
        d = p[2] - rad - 0.902
        if d < worst[0]: worst = (d, ("table", a, np.round(p, 3)))
        d = bowl_clear(p, rad)
        if d < worst[0]: worst = (d, ("bowl", a, np.round(p, 3)))
    return worst


def R_tilt(tilt_deg):
    t = np.radians(tilt_deg)
    Z = np.array([0, np.cos(t), -np.sin(t)]); Y = np.array([1.0, 0, 0]); X = np.cross(Y, Z)
    return np.column_stack([X, Y, Z])


XP, ZP = 0.0, 0.945
WAY = [  # (name, tcp, tilt_deg, shift)
    ("high", np.array([XP, -0.02, 1.20]), 65, 0.0),
    ("pre", np.array([XP, 0.03, ZP]), 65, 0.0),
    ("y06", np.array([XP, 0.06, ZP]), 65, 0.015),
    ("y09", np.array([XP, 0.09, ZP]), 65, 0.045),
    ("y12", np.array([XP, 0.12, ZP]), 58, 0.075),
    ("y15", np.array([XP, 0.15, ZP]), 48, 0.105),
    ("y175", np.array([XP, 0.175, ZP]), 38, 0.13),
    ("y19", np.array([XP, 0.19, ZP]), 30, 0.145),
]

if __name__ == "__main__":
    r = Robot("cp2")
    q0 = r.arm_q(); t0, R0 = r.tcp()
    print("tcp", np.round(t0, 3), "gap", round(r.finger_gap(), 4))
    qs = [q0]; plan = []
    for name, tcp, tilt, shift in WAY:
        R = R_tilt(tilt)
        q = ik_checked(r, tcp, R, qs[-1], max_dist=3.0 if len(qs) == 1 else 1.0, tries=10)
        if q is None: print(name, "no IK"); sys.exit(1)
        c = check2(r, q, shift)
        print(f"{name}: tilt={tilt} margin={margin(q):.2f} dq={np.abs(q - qs[-1]).max():.2f} clear={c[0]:.3f} {c[1]}")
        qs.append(q); plan.append((name, tcp, R, shift, q))
    np.save("snaps/close_plan.npy", np.array(plan, dtype=object), allow_pickle=True)
OPENRUA_EOF

# openrua op 85
timeout 600 python3 close_plan2.py 2>&1 | tail -20

# openrua op 86
grep -n "CAPS" -A 12 coll.py | head -40

# openrua op 87
python3 - <<'EOF'
import re
s = open("close_plan2.py").read()
s = s.replace('''    samples = [(P[a] + t * (P[bb] - P[a]), rad) for a, bb, rad in CAPS for t in np.linspace(0, 1, 6)]
    samples += hand_pts''','''    samples = [(P[a] + t * (P[bb] - P[a]), rad) for a, bb, rad in CAPS[:-1] for t in np.linspace(0, 1, 6)]
    flange = ph - 0.045 * Rh[:, 2]  # link7 cylinder ends ~4.5 cm behind the hand mounting face
    samples += [(P["panda_link7"] + t * (flange - P["panda_link7"]), 0.045) for t in np.linspace(0, 1, 6)]
    samples += [(ph + 0.045 * (np.cos(th) * Rh[:, 0] + np.sin(th) * Rh[:, 1]) - 0.01 * Rh[:, 2], 0.012)
                for th in np.linspace(0, 2 * np.pi, 12, endpoint=False)]  # flange disc
    samples += hand_pts''')
s = s.replace('''    hand_pts += [(ph + Rh @ np.array([0, 0, -0.05]), 0.05), (ph + Rh @ np.array([0, 0, -0.10]), 0.05)]\n''', '')
open("close_plan2.py","w").write(s)
EOF
timeout 600 python3 close_plan2.py 2>&1 | tail -12

# openrua op 88
cat > bowlprof.py <<'EOF'
import numpy as np
from rob import *
r = Robot("bp")
pts = np.vstack([r.cloud(c) for c in ["agentview", "birdview", "frontview"]])
pts = pts[np.isfinite(pts).all(1)]
d = pts[:, :2] - np.array([-0.01, -0.075]); rho = np.hypot(d[:, 0], d[:, 1])
m = (rho < 0.13) & (pts[:, 2] > 0.90) & (pts[:, 2] < 1.06)
p = pts[m]; rho = rho[m]
for lo in np.arange(0, 0.13, 0.01):
    s = (rho >= lo) & (rho < lo + 0.01)
    if s.sum(): print(f"rho {lo:.2f}-{lo+0.01:.2f}: n={s.sum():4d} z min {p[s,2].min():.3f} max {p[s,2].max():.3f} p90 {np.percentile(p[s,2],90):.3f}")
EOF
timeout 300 python3 bowlprof.py 2>&1 | tail -15

# openrua op 89
sed -i 's/pts = np.vstack(\[r.cloud(c) for c in \["agentview", "birdview", "frontview"\]\])/pts = np.vstack([r.cloud(c).reshape(-1, 3) for c in ["agentview", "birdview", "frontview"]])/' bowlprof.py && timeout 300 python3 bowlprof.py 2>&1 | tail -15

# openrua op 90
grep -n "def cloud" -A 25 rob.py | grep -n "return" 

# openrua op 91
sed -i 's/r.cloud(c).reshape(-1, 3)/r.cloud(c)[1].reshape(-1, 3)/' bowlprof.py && timeout 300 python3 bowlprof.py 2>&1 | tail -15

# openrua op 92
cat >> bowlprof.py <<'EOF'
print("--- high pts near bowl")
s = (rho > 0.10) & (p[:, 2] > 0.99)
print(np.round(np.percentile(p[s], [0, 50, 100], axis=0), 3))
s = (p[:, 2] > 0.94) & (rho < 0.10)
print("rim-ish pts z>0.94:", s.sum(), "x", np.round(np.percentile(p[s,0],[0,100]),3), "y", np.round(np.percentile(p[s,1],[0,100]),3))
# bowl outline by angle
ang = np.degrees(np.arctan2(d[m][:,1], d[m][:,0]))
for a0 in range(-180, 180, 45):
    s = (ang >= a0) & (ang < a0+45) & (p[:,2] > 0.93) & (rho < 0.11)
    if s.sum(): print(f"ang {a0:4d}: rho max {rho[s].max():.3f} z max {p[s,2].max():.3f}")
EOF
timeout 300 python3 bowlprof.py 2>&1 | tail -14

# openrua op 93
python3 - <<'EOF'
s = open("close_plan2.py").read()
s = s.replace("BOWL_C = np.array([-0.01, -0.075]); BOWL_R = 0.08; BOWL_Z = 1.02", "BOWL_C = np.array([0.022, -0.062]); BOWL_R = 0.057; BOWL_Z = 0.952  # measured: shallow bowl")
s = s.replace("h = 0.905 + 0.115 * (rho / BOWL_R) ** 2", "h = 0.905 + 0.047 * (rho / BOWL_R) ** 2")
s = s.replace("rho - 0.09 - rad", "rho - 0.065 - rad")
import re
start = s.index("XP, ZP = "); end = s.index("]\n", start) + 2
s = s[:start] + '''XP, ZP = 0.0, 0.948
WAY = [  # (name, tcp, tilt_deg, shift)
    ("high", np.array([XP, -0.02, 1.15]), 40, 0.0),
    ("pre", np.array([XP, 0.03, ZP]), 35, 0.0),
    ("y08", np.array([XP, 0.08, ZP]), 35, 0.035),
    ("y13", np.array([XP, 0.13, ZP]), 35, 0.085),
    ("y17", np.array([XP, 0.17, ZP]), 35, 0.125),
    ("y19", np.array([XP, 0.19, ZP]), 35, 0.145),
]
''' + s[end:]
open("close_plan2.py","w").write(s)
EOF
timeout 600 python3 close_plan2.py 2>&1 | tail -8

# openrua op 94
python3 - <<'EOF'
s = open("close_plan2.py").read()
start = s.index("WAY = ["); end = s.index("]\n", start) + 2
s = s[:start] + '''WAY = [  # (name, tcp, tilt_deg, shift)
    ("high", np.array([XP, -0.02, 1.15]), 45, 0.0),
    ("pre", np.array([XP, 0.03, ZP]), 55, 0.0),
    ("y06", np.array([XP, 0.06, ZP]), 55, 0.015),
    ("y10", np.array([XP, 0.10, ZP]), 45, 0.055),
    ("y14", np.array([XP, 0.14, ZP]), 38, 0.095),
    ("y17", np.array([XP, 0.17, ZP]), 35, 0.125),
    ("y19", np.array([XP, 0.19, ZP]), 35, 0.145),
]
''' + s[end:]
open("close_plan2.py","w").write(s)
EOF
timeout 600 python3 close_plan2.py 2>&1 | tail -8

# openrua op 95
sed -i 's/XP, ZP = 0.0, 0.948/XP, ZP = 0.0, 0.952/; s/("pre", np.array(\[XP, 0.03, ZP\]), 55/("pre", np.array([XP, 0.03, ZP]), 60/; s/("y06", np.array(\[XP, 0.06, ZP\]), 55/("y06", np.array([XP, 0.06, ZP]), 60/; s/- 0.008 - rad/- 0.005 - rad/' close_plan2.py && timeout 600 python3 close_plan2.py 2>&1 | tail -8

# openrua op 96
mkdir -p "$(dirname /workspace/close_exec.py)"
cat > /workspace/close_exec.py <<'OPENRUA_EOF'
import numpy as np, sys
from rob import *
from goto import margin
from close_plan2 import check2, R_tilt, WAY

r = Robot("ce")
if r.finger_gap() > 0.01:
    r.gripper(0.0)
print("gap", round(r.finger_gap(), 4))
t0, R0 = r.tcp(); q = r.arm_q()
print("start tcp", np.round(t0, 3))


def seg(tcp1, R1, n, seconds, shift1, shift0):
    global t0, R0, q
    path = cart_path(r, t0, R0, tcp1, R1, n, q, max_step=0.6)
    if path is None:
        print("   cart_path failed"); return False
    for i, qq in enumerate(path):
        sh = shift0 + (shift1 - shift0) * (i + 1) / len(path)
        c = check2(r, qq, sh)
        print(f"   via {i}: margin={margin(qq):.2f} clear={c[0]:.3f} {c[1]}")
        if c[0] < -0.01:
            print("   !! unsafe"); return False
    r.move_q(path[-1], seconds, via=path[:-1])
    t0, R0 = r.tcp(); q = r.arm_q()
    print("   tcp", np.round(t0, 4), "force", np.round(r.force(), 1), flush=True)
    return True


shift0 = 0.0
for name, tcp, tilt, shift in WAY:
    print(name)
    n = {"high": 3, "pre": 4}.get(name, 2)
    secs = {"high": 5, "pre": 4}.get(name, 3)
    if not seg(tcp, R_tilt(tilt), n, secs, shift, shift0):
        sys.exit(name + " failed")
    shift0 = shift
    if name in ("high", "pre"):
        r.snap("agentview", f"/workspace/snaps/av_{name}.png")
r.snap("agentview", "/workspace/snaps/av_pushed.png"); r.snap("frontview", "/workspace/snaps/fv_pushed.png")
OPENRUA_EOF

# openrua op 97
timeout 900 python3 close_exec.py 2>&1 | tail -45

# openrua op 98
mkdir -p "$(dirname /workspace/close_verify.py)"
cat > /workspace/close_verify.py <<'OPENRUA_EOF'
import numpy as np
from rob import *
from goto import margin
from close_plan2 import check2, R_tilt

r = Robot("cv")
t0, R0 = r.tcp(); q = r.arm_q()
# retreat: back along -y, then up
for tcp, tilt, n, secs in [(np.array([0.0, 0.10, 0.98]), 45, 2, 3), (np.array([0.0, -0.02, 1.15]), 45, 3, 4)]:
    path = cart_path(r, t0, R0, tcp, R_tilt(tilt), n, q, max_step=0.6)
    for qq in path:
        c = check2(r, qq, 0.147); print("   via clear", round(c[0], 3), c[1])
    r.move_q(path[-1], secs, via=path[:-1])
    t0, R0 = r.tcp(); q = r.arm_q(); print("tcp", np.round(t0, 3))

r.snap("agentview", "/workspace/snaps/av_closed.png"); r.snap("frontview", "/workspace/snaps/fv_closed.png")
r.snap("sideview", "/workspace/snaps/sv_closed.png")
pts = np.vstack([r.cloud(c)[1].reshape(-1, 3) for c in ["agentview", "frontview", "sideview", "birdview"]])
pts = pts[np.isfinite(pts).all(1)]
# anything in front of the cabinet face (y<0.215) within the cabinet's x span, above the table?
box = (pts[:, 0] > -0.17) & (pts[:, 0] < 0.13) & (pts[:, 1] > 0.0) & (pts[:, 1] < 0.215) & (pts[:, 2] > 0.905) & (pts[:, 2] < 1.13)
p = pts[box]
print("points in front of cabinet face:", len(p))
for zlo in np.arange(0.90, 1.13, 0.02):
    s = (p[:, 2] >= zlo) & (p[:, 2] < zlo + 0.02)
    if s.sum(): print(f"  z {zlo:.2f}: n={s.sum():5d} y min {p[s,1].min():.3f} x [{p[s,0].min():.3f},{p[s,0].max():.3f}]")
# bottom-drawer panel face: points at z 0.92-0.975 with |x|<0.1 -> min y should be ~0.222 minus handle depth
face = pts[(np.abs(pts[:, 0]) < 0.1) & (pts[:, 2] > 0.92) & (pts[:, 2] < 0.975) & (pts[:, 1] > 0.1) & (pts[:, 1] < 0.30)]
print("lower-front region y percentiles [1,5,50]:", np.round(np.percentile(face[:, 1], [1, 5, 50]), 3))
uh = pts[(np.abs(pts[:, 0]) < 0.1) & (pts[:, 2] > 1.0) & (pts[:, 2] < 1.03) & (pts[:, 1] > 0.1) & (pts[:, 1] < 0.30)]
print("upper-handle region y percentiles [1,5,50]:", np.round(np.percentile(uh[:, 1], [1, 5, 50]), 3))
OPENRUA_EOF

# openrua op 99
timeout 600 python3 close_verify.py 2>&1 | grep -v "^   via" | tail -30
