#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/* tools/perception/*

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | sort | uniq -c; echo ---; timeout 15 ros2 topic echo /agentview/color/camera_info --once

# openrua op 7
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
end = time.time()+5
while time.time()<end: rclpy.spin_once(n, timeout_sec=0.2)
for k,v in sorted(seen.items()):
    tr, q = v.translation, v.rotation
    print(f"{k[0]:>28} -> {k[1]:<28} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 30 python3 tfdump.py

# openrua op 8
cat > cloud.py <<'EOF'
"""Grab color+depth+info from a camera, save world-frame point cloud as npz."""
import sys, time, numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(cam):
    rclpy.init(); n = rclpy.create_node("cloud")
    got = {}
    n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
    n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame" and t.header.frame_id == "world":
                got["tf"] = t.transform
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(TFMessage, "/tf_static", tfcb, qos)
    n.create_subscription(TFMessage, "/tf", tfcb, 100)
    end = time.time()+30
    while len(got) < 4 and time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
    assert len(got) == 4, got.keys()
    br = CvBridge()
    color = br.imgmsg_to_cv2(got["c"], "rgb8")
    depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
    k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u-cx)*depth/fx; Y = (v-cy)*depth/fy; Z = depth
    P = np.stack([X,Y,Z], -1).reshape(-1,3)
    tf = got["tf"]; R = quat_R(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
    t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
    Pw = (P @ R.T + t).reshape(H, W, 3)
    rclpy.shutdown()
    np.savez(f"{cam}_cloud.npz", pw=Pw, color=color, depth=depth)
    return Pw, color, depth

if __name__ == "__main__":
    Pw, color, depth = grab(sys.argv[1])
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    print("center px world", Pw[240,320])
EOF
timeout 60 python3 cloud.py birdview && timeout 60 python3 cloud.py agentview

# openrua op 9
cat > seg.py <<'EOF'
import numpy as np
d = np.load("birdview_cloud.npz"); Pw, color = d["pw"], d["color"]
z = Pw[...,2]
valid = np.isfinite(z)
# table height: mode of z in the table region
hist, edges = np.histogram(z[valid], bins=200, range=(0,1.2))
top = np.argsort(hist)[-6:]
for i in sorted(top): print(f"z~{edges[i]:.3f} count={hist[i]}")
EOF
python3 seg.py

# openrua op 10
cat > seg2.py <<'EOF'
import numpy as np, cv2
from scipy import ndimage
d = np.load("birdview_cloud.npz"); Pw, color = d["pw"], d["color"]
z = Pw[...,2]
table = 0.42
mask = np.isfinite(z) & (z > table + 0.008) & (z < table + 0.35)
# exclude robot: robot in birdview is above y? robot base at world x=-0.51. Objects at x > -0.35 roughly
mask &= Pw[...,0] > -0.40
lab, n = ndimage.label(mask)
print("components", n)
vis = color.copy()
for i in range(1, n+1):
    m = lab == i
    if m.sum() < 30: continue
    pts = Pw[m]; c = color[m].mean(0)
    ys, xs = np.where(m)
    print(f"comp {i}: px={m.sum()} center_uv=({xs.mean():.0f},{ys.mean():.0f}) world x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) rgb={c.astype(int)}")
    cv2.rectangle(vis, (xs.min(), ys.min()), (xs.max(), ys.max()), (0,255,0), 1)
    cv2.putText(vis, str(i), (xs.min(), ys.min()-2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)
cv2.imwrite("birdview_seg.png", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
EOF
python3 seg2.py

# openrua op 11
cat > seg3.py <<'EOF'
import numpy as np, cv2
from scipy import ndimage
for cam in ["agentview"]:
    d = np.load(f"{cam}_cloud.npz"); Pw, color = d["pw"], d["color"]
    hsv = cv2.cvtColor(color, cv2.COLOR_RGB2HSV)
    z = Pw[...,2]; ok = np.isfinite(z) & (z > 0.428) & (z < 0.8) & (Pw[...,0] > -0.35)
    h, s, v = hsv[...,0], hsv[...,1], hsv[...,2]
    red = ok & ((h < 8) | (h > 170)) & (s > 120) & (v > 60)
    brown = ok & (v < 90) & (s > 40) & (h > 5) & (h < 30)
    for name, m in [("red", red), ("brown", brown)]:
        lab, n = ndimage.label(m)
        sizes = ndimage.sum(m, lab, range(1, n+1))
        i = np.argmax(sizes) + 1
        mm = lab == i
        pts = Pw[mm]; ys, xs = np.where(mm)
        print(f"{cam} {name}: px={mm.sum()} uv=({xs.mean():.0f},{ys.mean():.0f}) x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z=[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f})")
        vis = color.copy(); vis[mm] = (0,255,0)
        cv2.imwrite(f"{cam}_{name}.png", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
# birdview: mug body diameter estimation. Take red mug comp near (-0.2, 0.03): points z>0.5
d = np.load("birdview_cloud.npz"); Pw = d["pw"]
z = Pw[...,2]
for name, (x0,x1,y0,y1) in {"redmug":(-0.26,-0.15,-0.06,0.10), "whitemug":(-0.16,-0.04,-0.25,-0.10), "plate":(0.05,0.23,-0.08,0.10), "pudding":(-0.09,0.02,0.03,0.14)}.items():
    m = np.isfinite(z) & (Pw[...,0]>x0)&(Pw[...,0]<x1)&(Pw[...,1]>y0)&(Pw[...,1]<y1)&(z>0.428)
    pts = Pw[m]
    print(name, "n", m.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "z", pts[:,2].min().round(3), pts[:,2].max().round(3))
    # z histogram
    hs, es = np.histogram(pts[:,2], bins=np.arange(0.42, 0.80, 0.01))
    print("   z hist:", {round(e,2):int(c) for e,c in zip(es, hs) if c>0})
EOF
python3 seg3.py

# openrua op 12
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0 && timeout 60 python3 cloud.py birdview && timeout 60 python3 cloud.py agentview && python3 seg3.py

# openrua op 13
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 3.0 && timeout 60 python3 cloud.py birdview >/dev/null && timeout 60 python3 cloud.py agentview > /dev/null && python3 seg3.py

# openrua op 14
cat > seg4.py <<'EOF'
import numpy as np, cv2
d = np.load("birdview_cloud.npz"); Pw, color = d["pw"], d["color"]
z = Pw[...,2]
# table height near the plate ring
m = np.isfinite(z) & (Pw[...,0]>0.0)&(Pw[...,0]<0.4)&(np.abs(Pw[...,1])<0.3)
hs, es = np.histogram(z[m], bins=np.arange(0.38,0.50,0.005)); print("table region z hist:", {round(e,3):int(c) for e,c in zip(es,hs) if c>0})
# plate: ring; centroid of all plate pixels
m = np.isfinite(z) & (Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)
pts = Pw[m]; print("plate centroid", pts[:,0].mean().round(4), pts[:,1].mean().round(4), "n", m.sum(), "radius est", ((pts[:,0].max()-pts[:,0].min())/2).round(3), ((pts[:,1].max()-pts[:,1].min())/2).round(3))
# red mug: top rim points (z>0.54) in birdview to get body center
m = np.isfinite(z) & (Pw[...,0]>-0.27)&(Pw[...,0]<-0.13)&(Pw[...,1]>-0.07)&(Pw[...,1]<0.11)&(z>0.54)&(z<0.60)
pts = Pw[m]; print("redmug rim n", m.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4))
ys, xs = np.where(m); vis = color.copy(); vis[m]=(0,255,0)
# white mug rim
m2 = np.isfinite(z) & (Pw[...,0]>-0.16)&(Pw[...,0]<-0.04)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.10)&(z>0.52)
pts = Pw[m2]; print("whitemug rim n", m2.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4))
vis[m2]=(255,0,0)
# pudding top face (z 0.46-0.47)
m3 = np.isfinite(z) & (Pw[...,0]>-0.10)&(Pw[...,0]<0.02)&(Pw[...,1]>0.02)&(Pw[...,1]<0.14)&(z>0.455)&(z<0.48)
pts = Pw[m3]; print("pudding top n", m3.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4), "z", pts[:,2].mean().round(4))
vis[m3]=(0,0,255)
# PCA for pudding orientation
c = pts[:,:2]-pts[:,:2].mean(0); w,v = np.linalg.eigh(c.T@c); print("pudding long axis (x,y):", v[:,1].round(3), "yaw deg", np.degrees(np.arctan2(v[1,1], v[0,1])).round(1))
crop = vis[200:340, 240:400]; cv2.imwrite("bird_crop.png", cv2.resize(cv2.cvtColor(crop, cv2.COLOR_RGB2BGR), None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF
python3 seg4.py

# openrua op 15
cat > seg5.py <<'EOF'
import numpy as np
d = np.load("agentview_cloud.npz"); Pw = d["pw"]; z = Pw[...,2]
def rep(name, m):
    pts = Pw[m]; print(f"{name}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f},{pts[:,2].mean():.4f})")
ok = np.isfinite(z)
rep("table near plate (y<-0.1, x 0.05..0.25)", ok & (Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(Pw[...,1]<-0.10)&(Pw[...,1]>-0.25)&(z<0.44))
rep("table near pudding (x -0.1..0, y 0.15..0.25)", ok & (Pw[...,0]>-0.1)&(Pw[...,0]<0.0)&(Pw[...,1]>0.15)&(Pw[...,1]<0.25)&(z<0.44))
rep("plate", ok & (Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.425)&(z<0.5))
rep("pudding", ok & (Pw[...,0]>-0.10)&(Pw[...,0]<0.02)&(Pw[...,1]>0.02)&(Pw[...,1]<0.14)&(z>0.43)&(z<0.5))
rep("red mug", ok & (Pw[...,0]>-0.30)&(Pw[...,0]<-0.12)&(Pw[...,1]>-0.07)&(Pw[...,1]<0.11)&(z>0.43)&(z<0.6))
rep("white mug", ok & (Pw[...,0]>-0.16)&(Pw[...,0]<-0.04)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.10)&(z>0.43)&(z<0.6))
# red mug rim points from agentview: highest band
m = ok & (Pw[...,0]>-0.30)&(Pw[...,0]<-0.12)&(Pw[...,1]>-0.07)&(Pw[...,1]<0.11)&(z>0.555)
rep("red mug rim band", m)
EOF
python3 seg5.py

# openrua op 16
cat > fit.py <<'EOF'
import numpy as np
from scipy.optimize import least_squares
def fit(pts, r=None):
    x,y = pts[:,0], pts[:,1]
    if r is None:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - p[2]; p0=[x.mean(), y.mean(), 0.04]
    else:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - r; p0=[x.mean(), y.mean()]
    return least_squares(f, p0, loss="soft_l1", f_scale=0.005).x
for cam, zlo in [("birdview",0.54),("agentview",0.555)]:
    d = np.load(f"{cam}_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]
    m = np.isfinite(z)&(Pw[...,0]>-0.30)&(Pw[...,0]<-0.12)&(Pw[...,1]>-0.03)&(Pw[...,1]<0.11)&(z>zlo)&(z<0.60)
    pts = Pw[m][:,:2]
    print(cam, "red mug rim free fit", fit(pts).round(4), "fixed r=0.0435:", fit(pts,0.0435).round(4))
    m = np.isfinite(z)&(Pw[...,0]>-0.16)&(Pw[...,0]<-0.04)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.10)&(z>zlo-0.02)&(z<0.60)
    pts = Pw[m][:,:2]
    print(cam, "white mug rim free fit", fit(pts).round(4))
EOF
python3 fit.py

# openrua op 17
cat > robot.py <<'EOF'
"""Helpers: FK/IK, trajectory, gripper, joint state, all on one node."""
import time, math, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE_T = np.array([-0.51, 0.0, 0.42])   # world -> panda_link0 (TF, identity rotation)
TCP = M["hand"]["tcp_offset_m"]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return np.array([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2])

def quat_z(yaw):
    return np.array([0, 0, math.sin(yaw/2), math.cos(yaw/2)])

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        self.joints()

    def _on_js(self, m): self._js = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return np.array([d[j] for j in JOINTS]), d

    def fingers(self):
        _, d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def _seed(self, q):
        js = JointState(); js.name = list(JOINTS); js.position = [float(v) for v in q]
        return js

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        if q is None: q, _ = self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result(); assert r is not None and r.error_code.val == 1, r
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_hand(self, pos_w, quat, seed=None, timeout=30):
        """IK for hand at world pos/quat. Returns joint array or None."""
        if seed is None: seed, _ = self.joints()
        p = np.asarray(pos_w, float) - BASE_T
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        po = req.ik_request.pose_stamped.pose
        po.position.x, po.position.y, po.position.z = map(float, p)
        po.orientation.x, po.orientation.y, po.orientation.z, po.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([d[j] for j in JOINTS])

    def ik_tcp(self, tcp_w, quat, **kw):
        R = quat_R(*quat)
        return self.ik_hand(np.asarray(tcp_w) - TCP * R[:, 2], quat, **kw)

    def move(self, waypoints, times):
        """waypoints: list of joint arrays; times: cumulative seconds."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        q, _ = self.joints()
        err = np.abs(q - waypoints[-1]).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def go_tcp(self, tcp_w, quat, seconds, seed=None):
        q = self.ik_tcp(tcp_w, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for tcp {np.round(tcp_w,3)}"); return None
        self.move([q], [seconds])
        pos, _ = self.fk_hand()
        R = quat_R(*quat); tcp = pos + TCP * R[:, 2]
        print(f"  tcp now {np.round(tcp,4)} target {np.round(tcp_w,4)}")
        return q

    def go_tcp_line(self, tcp_from, tcp_to, quat, seconds, n=4, seed=None):
        """Straight-line TCP motion via n IK waypoints in one trajectory."""
        qs, ts = [], []
        s = seed if seed is not None else self.joints()[0]
        for i in range(1, n+1):
            a = i / n
            p = np.asarray(tcp_from) * (1-a) + np.asarray(tcp_to) * a
            q = self.ik_tcp(p, quat, seed=s)
            if q is None:
                print(f"  IK FAILED at line waypoint {np.round(p,3)}"); return None
            qs.append(q); ts.append(seconds * a); s = q
        self.move(qs, ts)
        pos, _ = self.fk_hand()
        R = quat_R(*quat); tcp = pos + TCP * R[:, 2]
        print(f"  tcp now {np.round(tcp,4)} target {np.round(tcp_to,4)}")
        return qs[-1]
EOF
cat > step0.py <<'EOF'
from robot import *
r = Robot()
q, d = r.joints(); print("joints", np.round(q,4)); print("fingers", r.fingers())
pos, quat = r.fk_hand(); print("hand world pos", np.round(pos,4), "quat", np.round(quat,4))
R = quat_R(*quat); print("hand x axis", np.round(R[:,0],3), "y", np.round(R[:,1],3), "z", np.round(R[:,2],3))
print("tcp", np.round(pos + TCP*R[:,2],4))
EOF
timeout 120 python3 step0.py

# openrua op 18
cat > step0b.py <<'EOF'
from robot import *
r = Robot()
q,_ = r.joints()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0", "panda_hand", "panda_leftfinger"]
req.robot_state.joint_state = r._seed(q)
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res = fut.result()
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p = ps.pose.position; print(n, "frame:", repr(ps.header.frame_id), "pos", round(p.x,4), round(p.y,4), round(p.z,4))
EOF
timeout 60 python3 step0b.py; timeout 20 ros2 param get /move_group robot_description_planning.joint_limits 2>/dev/null | head -2; timeout 30 ros2 topic echo /robot_description --once --qos-durability transient_local --qos-reliability reliable 2>/dev/null | grep -oE '<joint name="[^"]*" type="fixed">' | head; timeout 30 ros2 topic echo /robot_description --once --qos-durability transient_local 2>/dev/null | grep -A3 -E 'name="(world|virtual)' | head -20

# openrua op 19
cat > step0c.py <<'EOF'
from robot import *
import robot
r = Robot()
q,_ = r.joints()
robot.BASE_T = np.zeros(3); r_pos, r_quat = r.fk_hand(q)   # true world pose (no offset)
print("hand world", np.round(r_pos,4))
for label, p in [("world coords", r_pos), ("base-relative coords", r_pos - np.array([-0.51,0,0.42]))]:
    robot.BASE_T = np.zeros(3)
    sol = r.ik_hand(p, r_quat, seed=q)
    print(label, "->", None if sol is None else np.round(sol,3), "| current", np.round(q,3))
EOF
timeout 120 python3 step0c.py

# openrua op 20
sed -i 's/^BASE_T = np.array(\[-0.51, 0.0, 0.42\]).*/BASE_T = np.zeros(3)   # FK\/IK on this machine already report in the world frame (verified)/' robot.py
cat > dry.py <<'EOF'
from robot import *
r = Robot()
q0,_ = r.joints()
TOP = np.array([1.0, 0, 0, 0])
MUG = np.array([-0.188, 0.033]); RIM_Z = 0.575; R_MUG = 0.044
grasp_xy = MUG + [0, R_MUG]
PLATE = np.array([0.145, 0.013])
PUD = np.array([-0.021, 0.103]); pud_quat = quat_mul(quat_z(math.radians(-14)), TOP)
tests = {
 "mug pre":   ([*grasp_xy, 0.65], TOP),
 "mug grasp": ([*grasp_xy, 0.545], TOP),
 "mug lift":  ([*grasp_xy, 0.72], TOP),
 "plate above": ([*(PLATE+[0,R_MUG]), 0.72], TOP),
 "plate place": ([*(PLATE+[0,R_MUG]), 0.585], TOP),
 "pud pre":   ([*PUD, 0.56], pud_quat),
 "pud grasp": ([*PUD, 0.445], pud_quat),
 "pud dest above": ([PLATE[0], 0.21, 0.60], pud_quat),
 "pud dest place": ([PLATE[0], 0.21, 0.45], pud_quat),
 "pud dest place2": ([PLATE[0], 0.20, 0.45], TOP),
}
seed = q0
for k,(p,qt) in tests.items():
    s = r.ik_tcp(p, qt, seed=seed)
    print(f"{k:16s} tcp={np.round(p,3)} -> {'OK ' + str(np.round(s,2)) if s is not None else 'FAIL'}")
    if s is not None: seed = s
EOF
timeout 300 python3 dry.py

# openrua op 21
cat > common.py <<'EOF'
from robot import *
TOP = np.array([1.0, 0, 0, 0])
MUG = np.array([-0.188, 0.033]); RIM_Z = 0.575; R_MUG = 0.044
GRASP_XY = MUG + [0, R_MUG]
PLATE = np.array([0.145, 0.013])
PUD = np.array([-0.021, 0.103]); PUD_Q = quat_mul(quat_z(math.radians(-14)), TOP)
EOF
cat > step1.py <<'EOF'
from common import *
r = Robot()
print("pre-grasp above rim"); q = r.go_tcp([*GRASP_XY, 0.65], TOP, 4.0)
EOF
timeout 600 python3 step1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 22
cat > step2.py <<'EOF'
from common import *
r = Robot()
print("descend onto rim"); r.go_tcp_line([*GRASP_XY, 0.65], [*GRASP_XY, 0.545], TOP, 3.0, n=3)
print("fingers before", r.fingers())
EOF
timeout 600 python3 step2.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 23
cat > step3.py <<'EOF'
from common import *
r = Robot()
f = r.gripper(0.0)
# settle a moment: hold pose briefly so the grasp state updates
q,_ = r.joints(); r.move([q], [0.5])
print("fingers after settle", r.fingers())
EOF
timeout 600 python3 step3.py

# openrua op 24
cat > step4.py <<'EOF'
from common import *
r = Robot()
print("lift"); r.go_tcp_line([*GRASP_XY, 0.545], [*GRASP_XY, 0.72], TOP, 3.0, n=2)
print("fingers", r.fingers())
EOF
timeout 600 python3 step4.py && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 25
cat > step5.py <<'EOF'
from common import *
r = Robot()
print("to above plate"); r.go_tcp([*(PLATE + [0, R_MUG]), 0.72], TOP, 4.0)
print("fingers", r.fingers())
EOF
timeout 600 python3 step5.py && timeout 60 python3 cloud.py birdview >/dev/null && cat > chk5.py <<'EOF'
import numpy as np
from fit import fit
d = np.load("birdview_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]
# mug rim in the air above the plate: z between 0.68 and 0.76
m = np.isfinite(z)&(Pw[...,0]>0.0)&(Pw[...,0]<0.3)&(np.abs(Pw[...,1])<0.2)&(z>0.66)&(z<0.75)
pts = Pw[m]; print("mug-rim-ish pts n", m.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "zmax", pts[:,2].max().round(3))
print("circle fit r=0.044:", fit(pts[:,:2], 0.0435).round(4))
m2 = np.isfinite(z)&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)&(z<0.47)
p2=Pw[m2]; print("plate centroid", p2[:,0].mean().round(4), p2[:,1].mean().round(4))
EOF
python3 chk5.py

# openrua op 26
cat > circfit.py <<'EOF'
import numpy as np
from scipy.optimize import least_squares
def fit(pts, r=None):
    x,y = pts[:,0], pts[:,1]
    if r is None:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - p[2]; p0=[x.mean(), y.mean(), 0.04]
    else:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - r; p0=[x.mean(), y.mean()]
    return least_squares(f, p0, loss="soft_l1", f_scale=0.005).x
EOF
sed -i 's/from fit import fit/from circfit import fit/' chk5.py
timeout 600 python3 step5.py && timeout 60 python3 cloud.py birdview >/dev/null && python3 chk5.py

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 28
cat > step6.py <<'EOF'
from common import *
r = Robot()
top = [*(PLATE + [0, R_MUG]), 0.72]; dn = [*(PLATE + [0, R_MUG]), 0.58]
print("lower onto plate"); r.go_tcp_line(top, dn, TOP, 3.0, n=2)
print("open"); r.gripper(0.04)
print("retreat up"); r.go_tcp_line(dn, top, TOP, 2.0, n=1)
print("fingers", r.fingers())
EOF
timeout 900 python3 step6.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 cloud.py birdview > /dev/null

# openrua op 29
cat > chk6.py <<'EOF'
import numpy as np
from circfit import fit
d = np.load("birdview_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]
ok = np.isfinite(z)
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.56)&(z<0.62)
pts=Pw[m]; print("mug rim n", m.sum(), "z", pts[:,2].min().round(3), pts[:,2].max().round(3), "fit", fit(pts[:,:2], 0.0435).round(4), "free", fit(pts[:,:2]).round(4))
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)&(z<0.47)
pts=Pw[m]; print("plate ring n", m.sum(), "fit", fit(pts[:,:2], 0.066).round(4))
m = ok&(Pw[...,0]>-0.10)&(Pw[...,0]<0.03)&(Pw[...,1]>0.02)&(Pw[...,1]<0.16)&(z>0.455)&(z<0.48)
pts=Pw[m]; print("pudding top n", m.sum(), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4), "z", pts[:,2].mean().round(3), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3))
c = pts[:,:2]-pts[:,:2].mean(0); w,v = np.linalg.eigh(c.T@c); print("pudding long axis yaw deg", np.degrees(np.arctan2(v[1,1], v[0,1])).round(1))
EOF
python3 chk6.py

# openrua op 30
sed -i 's/^PUD = np.array(\[-0.021, 0.103\])/PUD = np.array([-0.020, 0.100])/' common.py
cat > step7.py <<'EOF'
from common import *
r = Robot()
print("pudding pre-grasp"); r.go_tcp([*PUD, 0.56], PUD_Q, 4.0)
EOF
timeout 600 python3 step7.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 cloud.py birdview >/dev/null && python3 chk6.py

# openrua op 31
cat > step7b.py <<'EOF'
from common import *
import common
r = Robot()
Q2 = quat_mul(quat_z(math.radians(11)), TOP)
print("re-orient above pudding"); r.go_tcp([*PUD, 0.56], Q2, 3.0)
EOF
timeout 600 python3 step7b.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 32
sed -i 's/PUD_Q = quat_mul(quat_z(math.radians(-14)), TOP)/PUD_Q = quat_mul(quat_z(math.radians(11)), TOP)/' common.py
cat > step8.py <<'EOF'
from common import *
r = Robot()
print("descend"); r.go_tcp_line([*PUD, 0.56], [*PUD, 0.445], PUD_Q, 3.0, n=3)
print("close"); r.gripper(0.0)
q,_ = r.joints(); r.move([q],[0.5]); print("fingers settled", r.fingers())
EOF
timeout 900 python3 step8.py

# openrua op 33
cat > step9.py <<'EOF'
from common import *
r = Robot()
DEST = np.array([PLATE[0], 0.21])
print("lift"); r.go_tcp_line([*PUD, 0.445], [*PUD, 0.60], PUD_Q, 2.5, n=2)
print("fingers", r.fingers())
print("to dest above"); r.go_tcp([*DEST, 0.60], PUD_Q, 4.0)
print("fingers", r.fingers())
EOF
timeout 900 python3 step9.py && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
cat > step10.py <<'EOF'
from common import *
r = Robot()
DEST = np.array([PLATE[0], 0.21])
print("lower"); r.go_tcp_line([*DEST, 0.60], [*DEST, 0.452], PUD_Q, 3.0, n=2)
print("open"); r.gripper(0.04)
print("retreat"); r.go_tcp_line([*DEST, 0.452], [*DEST, 0.65], PUD_Q, 2.5, n=1)
# move the arm clear of the scene for an unoccluded view
print("park"); r.go_tcp([-0.15, 0.0, 0.75], TOP, 4.0)
print("fingers", r.fingers())
EOF
timeout 900 python3 step10.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 cloud.py birdview >/dev/null

# openrua op 35
cat > final.py <<'EOF'
import numpy as np
from circfit import fit
d = np.load("birdview_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]; ok=np.isfinite(z)
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.56)&(z<0.62)
pts=Pw[m]; mug = fit(pts[:,:2], 0.0435); print(f"red mug rim: n={m.sum()} center=({mug[0]:.3f},{mug[1]:.3f}) rim z={pts[:,2].mean():.3f}")
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)&(z<0.47)
pts=Pw[m]; plate = fit(pts[:,:2], 0.066); print(f"plate ring: n={m.sum()} center=({plate[0]:.3f},{plate[1]:.3f})")
print(f"mug offset from plate center: {np.hypot(*(mug-plate)):.3f} m (plate radius 0.066, mug radius 0.044)")
m = ok&(Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(Pw[...,1]>0.13)&(Pw[...,1]<0.30)&(z>0.45)&(z<0.50)
pts=Pw[m]; print(f"pudding top: n={m.sum()} center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) z={pts[:,2].mean():.3f} y-range [{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
print(f"pudding is {pts[:,1].mean()-plate[1]:.3f} m to +y (agentview-right) of plate center; gap to plate edge {pts[:,1].min()-plate[1]-0.066:.3f} m")
EOF
python3 final.py
