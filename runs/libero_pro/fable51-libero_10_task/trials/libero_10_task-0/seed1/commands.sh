#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
cd /workspace; for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ----; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
cd /workspace; cat > tfdump.py <<'EOF'
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
for _ in range(40): rclpy.spin_once(node, timeout_sec=0.1)
for k,v in sorted(seen.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
EOF
timeout 60 python3 tfdump.py

# openrua op 7
cd /workspace; cat > cloud.py <<'EOF'
"""Grab depth+color+info for a camera, save world-frame point cloud arrays."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
tfs = {}
def cb(m):
    for t in m.transforms: tfs[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
while len(got) < 3 or ("world", f"{cam}_optical_frame") not in tfs:
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = tfs[("world", f"{cam}_optical_frame")]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.save(f"{cam}_xyz.npy", pw); np.save(f"{cam}_bgr.npy", color)
print("saved", pw.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
print("intrinsics", fx, fy, cx, cy)
rclpy.shutdown()
EOF
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview

# openrua op 8
cd /workspace; cat > seg.py <<'EOF'
import numpy as np, cv2
pw = np.load("birdview_xyz.npy"); bgr = np.load("birdview_bgr.npy")
z = pw[...,2]
# table height: mode of z in the table region
tab = z[(np.abs(pw[...,0])<0.6)&(np.abs(pw[...,1])<0.6)]
hist, edges = np.histogram(tab[np.isfinite(tab)], bins=200)
zt = edges[np.argmax(hist)]
print("table z ~", zt)
mask = (z > zt + 0.01) & (z < zt + 0.4) & (pw[...,0] > -0.3)  # exclude robot base region roughly
mask = mask.astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 15: continue
    m = lab == i
    pts = pw[m]
    col = bgr[m].mean(0)
    print(f"comp {i}: px area {stats[i,4]}, centroid px ({cent[i][0]:.0f},{cent[i][1]:.0f}), world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} mean {pts[:,0].mean():.3f},{pts[:,1].mean():.3f} color BGR {col.astype(int)}")
EOF
python3 seg.py

# openrua op 9
cd /workspace; cat > seg2.py <<'EOF'
import numpy as np, cv2, sys
cam = sys.argv[1]
pw = np.load(f"{cam}_xyz.npy"); bgr = np.load(f"{cam}_bgr.npy")
z = pw[...,2]; zt = 0.4204
mask = (z > zt + 0.008) & (z < zt + 0.35) & (pw[...,0] > -0.32) & (pw[...,0] < 0.4) & (np.abs(pw[...,1]) < 0.5)
mask = mask.astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = pw[m]; col = bgr[m].mean(0)
    print(f"comp {i}: area {stats[i,4]}, px ({cent[i][0]:.0f},{cent[i][1]:.0f}), x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) BGR {col.astype(int)}")
cv2.imwrite(f"{cam}_mask.png", (lab>0).astype(np.uint8)*255)
EOF
python3 seg2.py agentview

# openrua op 10
cd /workspace; cat > pca.py <<'EOF'
import numpy as np
for cam, box in [("birdview", None), ("agentview", None)]:
    pw = np.load(f"{cam}_xyz.npy")
    # cream cheese
    for name, (x0,x1,y0,y1,zmin) in {"cheese": (0.03,0.15,-0.23,-0.15,0.432), "can": (-0.13,-0.03,0.0,0.09,0.43), "butter": (0.02,0.12,0.0,0.07,0.43), "basket": (-0.1,0.12,0.15,0.36,0.5)}.items():
        m = (pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&(pw[...,2]>zmin)&(pw[...,2]<0.8)
        pts = pw[m]
        if len(pts) < 10: print(cam, name, "few pts", len(pts)); continue
        xy = pts[:,:2]; c = xy.mean(0)
        top = pts[pts[:,2] > pts[:,2].max()-0.012]
        ev, evec = np.linalg.eigh(np.cov((top[:,:2]-top[:,:2].mean(0)).T))
        ang = np.degrees(np.arctan2(evec[1,1], evec[0,1]))
        print(f"{cam} {name}: n={len(pts)} mean xy=({c[0]:.3f},{c[1]:.3f}) top n={len(top)} top mean=({top[:,0].mean():.3f},{top[:,1].mean():.3f}) ztop={pts[:,2].max():.3f} top extents x[{top[:,0].min():.3f},{top[:,0].max():.3f}] y[{top[:,1].min():.3f},{top[:,1].max():.3f}] major axis angle {ang:.1f} deg, sd {np.sqrt(ev)}")
EOF
python3 pca.py

# openrua op 11
cd /workspace; python3 - <<'EOF'
import numpy as np
for cam in ["agentview"]:
    pw = np.load(f"{cam}_xyz.npy")
    m = (pw[...,0]>-0.15)&(pw[...,0]<-0.02)&(pw[...,1]>0.0)&(pw[...,1]<0.1)&(pw[...,2]>0.43)&(pw[...,2]<0.56)
    pts = pw[m]
    top = pts[pts[:,2] > pts[:,2].max()-0.006]
    print(f"{cam} can: n={len(pts)} ztop={pts[:,2].max():.3f} top n={len(top)} top mean=({top[:,0].mean():.3f},{top[:,1].mean():.3f}) top extents x[{top[:,0].min():.3f},{top[:,0].max():.3f}] y[{top[:,1].min():.3f},{top[:,1].max():.3f}]")
    # fit circle to top points
    A = np.c_[2*top[:,0], 2*top[:,1], np.ones(len(top))]; b = (top[:,:2]**2).sum(1)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]; r = np.sqrt(c + cx*cx + cy*cy)
    print(f"circle fit center ({cx:.3f},{cy:.3f}) r={r:.3f}")
    # side points at mid height
    mid = pts[(pts[:,2]>0.46)&(pts[:,2]<0.50)]
    print(f"mid y extents [{mid[:,1].min():.3f},{mid[:,1].max():.3f}] x min {mid[:,0].min():.3f}")
EOF

# openrua op 12
cd /workspace; cat > arm.py <<'EOF'
"""Reusable arm control: FK, IK, trajectory, gripper, joint state. World<->base conversion."""
import sys, time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)
TCP = M["hand"]["tcp_offset_m"]

def quat_to_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def topdown_quat(yaw=0.0):
    """Hand z down; fingers along base y when yaw=0; yaw rotates about world z."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw/2), np.sin(yaw/2)
    # (0,0,s,c) * (1,0,0,0): quaternion product (w1 w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w1, v1 = c, np.array([0, 0, s]); w2, v2 = 0.0, np.array([1.0, 0, 0])
    w = w1*w2 - v1 @ v2; v = w1*v2 + w2*v1 + np.cross(v1, v2)
    return (v[0], v[1], v[2], w)

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
    def _js_cb(self, m): self.js = dict(zip(m.name, m.position))
    def spin(self, t=0.2): rclpy.spin_once(self.node, timeout_sec=t)
    def joints(self):
        self.js = {}
        while not self.js: self.spin()
        return dict(self.js)
    def arm_q(self):
        j = self.joints(); return [j[n] for n in JOINTS]
    def fingers(self):
        j = self.joints(); return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")
    def _call(self, cli, req, timeout=60):
        f = cli.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout); return f.result()
    def fk_world(self, q=None):
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = q or self.arm_q()
        res = self._call(self.fk, req)
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), res.error_code.val
    def ik_world(self, pos_w, quat, at_tcp=False, seed=None):
        pos = np.array(pos_w, float)
        if at_tcp: pos = pos - TCP * quat_to_R(*quat)[:, 2]
        pos = pos - BASE_IN_WORLD
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = pos
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = quat
        r.robot_state.joint_state.name = JOINTS; r.robot_state.joint_state.position = seed or self.arm_q()
        r.avoid_collisions = False
        r.timeout.sec = 5
        res = self._call(self.ik, req)
        if res is None or res.error_code.val != 1:
            return None, (res.error_code.val if res else "timeout")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS], 1
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            for i, vq in enumerate(via):
                t = seconds * (i+1) / (len(via)+1)
                p = JointTrajectoryPoint(positions=list(map(float, vq))); p.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9)); pts.append(p)
        p = JointTrajectoryPoint(positions=list(map(float, q))); p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds%1)*1e9)); pts.append(p)
        goal.trajectory.points = pts
        f = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move: error_code={code} max joint err={err:.4f}", flush=True)
        return code, err
    def move_world(self, pos_w, quat, seconds=3.0, at_tcp=True):
        q, code = self.ik_world(pos_w, quat, at_tcp=at_tcp)
        if q is None:
            print(f"  IK FAILED {code} for {pos_w}", flush=True); return False
        self.move_q(q, seconds)
        p, _, _ = self.fk_world()
        tcp = p + TCP * quat_to_R(*quat)[:, 2] if at_tcp else p
        print(f"  now hand={np.round(p,4)} tcp={np.round(tcp,4)} target={np.round(pos_w,4)}", flush=True)
        return True
    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} fingers={self.fingers()}", flush=True)
        return r
EOF
cat > step0.py <<'EOF'
from arm import *
r = Robot()
print("joints", r.arm_q()); print("fingers", r.fingers())
p, q, c = r.fk_world(); print("hand world", p, "quat", q, "code", c)
print("hand R:\n", np.round(quat_to_R(*q), 3))
print("topdown quat", topdown_quat(0.0), "R:\n", np.round(quat_to_R(*topdown_quat(0.0)),3))
EOF
timeout 120 python3 -u step0.py

# openrua op 13
cd /workspace; cat > step0b.py <<'EOF'
from arm import *
r = Robot()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0", "panda_link1", "panda_hand"]
req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = r.arm_q()
res = r._call(r.fk, req)
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p = ps.pose.position; print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4))
req.header.frame_id = "world"
res = r._call(r.fk, req)
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p = ps.pose.position; print("hdr=world", n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4))
EOF
timeout 120 python3 -u step0b.py; timeout 20 ros2 service call /get_urdf std_srvs/srv/Trigger 2>/dev/null | grep -o 'joint name="[^"]*" type="[^"]*"' | head; 

# openrua op 14
cd /workspace; sed -i 's/^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\]).*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame IS world here (verified via \/compute_fk: panda_link0 at (-0.51,0,0.42))/' arm.py
cat > step0c.py <<'EOF'
from arm import *
r = Robot()
p, q, _ = r.fk_world(); print("hand world", p, q)
sol, code = r.ik_world(p, q, at_tcp=False); print("IK roundtrip code", code, "sol", np.round(sol,3) if sol else None, "cur", np.round(r.arm_q(),3))
# IK for pre-grasp above cream cheese: tcp at (0.09,-0.188, 0.55) top-down
sol, code = r.ik_world([0.09, -0.188, 0.55], topdown_quat(0.0), at_tcp=True); print("IK cheese pregrasp", code, np.round(sol,3) if sol else None)
sol, code = r.ik_world([-0.079, 0.042, 0.60], topdown_quat(0.0), at_tcp=True); print("IK can pregrasp", code, np.round(sol,3) if sol else None)
sol, code = r.ik_world([0.008, 0.253, 0.72], topdown_quat(0.0), at_tcp=True); print("IK basket", code, np.round(sol,3) if sol else None)
EOF
timeout 200 python3 -u step0c.py

# openrua op 15
cd /workspace; cat > step0d.py <<'EOF'
from arm import *
r = Robot()
sol = [0.058, -0.161, -0.058, -2.445, -0.012, 2.227, 0.009]
p, q, _ = r.fk_world(sol); print("FK of IK sol:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
p, q, _ = r.fk_world([-0.18, 0.501, -0.13, -1.914, 0.093, 2.409, -0.364]); print("FK cheese pregrasp:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
print("expected tcp", np.round(p + TCP*quat_to_R(*q)[:,2],4))
EOF
timeout 100 python3 -u step0d.py

# openrua op 16
cd /workspace; cat > step0e.py <<'EOF'
from arm import *
r = Robot()
def qmul(a, b):
    x1,y1,z1,w1 = a; x2,y2,z2,w2 = b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
Q = topdown_quat(0.0)
target = [0.09, -0.188, 0.55]
# Option A: ik_link_name = panda_hand
import moveit_msgs.srv
req = GetPositionIK.Request(); rr = req.ik_request
rr.group_name = "panda_arm"; rr.ik_link_name = "panda_hand"; rr.pose_stamped.header.frame_id = ""
pos = np.array(target) - TCP*quat_to_R(*Q)[:,2]
rr.pose_stamped.pose.position.x, rr.pose_stamped.pose.position.y, rr.pose_stamped.pose.position.z = pos
rr.pose_stamped.pose.orientation.x, rr.pose_stamped.pose.orientation.y, rr.pose_stamped.pose.orientation.z, rr.pose_stamped.pose.orientation.w = Q
rr.robot_state.joint_state.name = JOINTS; rr.robot_state.joint_state.position = r.arm_q(); rr.timeout.sec = 5
res = r._call(r.ik, req); print("A code", res.error_code.val)
if res.error_code.val == 1:
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position)); sol = [sol[j] for j in JOINTS]
    p, q, _ = r.fk_world(sol); print("A FK:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
# Option B: compensate by +45deg about z
s = np.sin(np.pi/8); c = np.cos(np.pi/8)
Qc = qmul(Q, (0,0,s,c))
sol, code = r.ik_world(target, Qc, at_tcp=False)
if sol:
    p, q, _ = r.fk_world(sol); print("B FK:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
Qc2 = qmul(Q, (0,0,-s,c))
sol, code = r.ik_world(target, Qc2, at_tcp=False)
if sol:
    p, q, _ = r.fk_world(sol); print("B2 FK:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
EOF
timeout 200 python3 -u step0e.py

# openrua op 17
cd /workspace; sed -i 's/r.group_name = M\["planning"\]\["group"\]; r.pose_stamped.header.frame_id = ""/r.group_name = M["planning"]["group"]; r.ik_link_name = "panda_hand"; r.pose_stamped.header.frame_id = ""/' arm.py; grep -n ik_link_name arm.py
cat > pick_cheese.py <<'EOF'
from arm import *
r = Robot()
Q = topdown_quat(0.0)
cx, cy = 0.09, -0.188
print("open gripper"); r.gripper(0.04)
print("pregrasp"); assert r.move_world([cx, cy, 0.56], Q, 4.0)
print("descend"); assert r.move_world([cx, cy, 0.435], Q, 2.5)
print("close"); r.gripper(0.0)
f = r.fingers(); print("fingers after close", f)
print("lift"); assert r.move_world([cx, cy, 0.65], Q, 2.5)
print("fingers after lift", r.fingers())
EOF
timeout 900 python3 -u pick_cheese.py 2>&1 | tee pick_cheese.log

# openrua op 18
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview cheese_lift.png

# openrua op 19
cd /workspace; cat > place.py <<'EOF'
from arm import *
r = Robot()
Q = topdown_quat(0.0)
bx, by = 0.008, 0.253
print("fingers before", r.fingers())
print("to basket"); assert r.move_world([bx, by, 0.72], Q, 4.0)
print("fingers at basket", r.fingers())
print("release"); r.gripper(0.04)
print("retreat up"); assert r.move_world([bx, by, 0.80], Q, 2.0)
EOF
timeout 900 python3 -u place.py 2>&1 | tee place_cheese.log; timeout 60 python3 tools/perception/cam_snap.py agentview cheese_placed.png

# openrua op 20
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_basket1.png; timeout 60 python3 tools/perception/cam_snap.py birdview bird1.png

# openrua op 21
cd /workspace; cat > pick_can.py <<'EOF'
from arm import *
r = Robot()
Q = topdown_quat(0.0)
cx, cy = -0.079, 0.042
print("fingers", r.fingers())
print("pregrasp"); assert r.move_world([cx, cy, 0.63], Q, 4.0)
print("descend"); assert r.move_world([cx, cy, 0.475], Q, 2.5)
print("close"); r.gripper(0.0)
print("fingers after close", r.fingers())
print("lift"); assert r.move_world([cx, cy, 0.70], Q, 2.5)
print("fingers after lift", r.fingers())
EOF
timeout 900 python3 -u pick_can.py 2>&1 | tee pick_can.log; timeout 60 python3 tools/perception/cam_snap.py agentview can_lift.png

# openrua op 22
cd /workspace; cat > place_can.py <<'EOF'
from arm import *
r = Robot()
Q = topdown_quat(0.0)
bx, by = 0.008, 0.253
print("fingers before", r.fingers())
print("to basket"); assert r.move_world([bx, by, 0.76], Q, 4.0)
print("fingers at basket", r.fingers())
print("release"); r.gripper(0.04)
print("retreat up"); assert r.move_world([bx, by, 0.85], Q, 2.0)
EOF
timeout 900 python3 -u place_can.py 2>&1 | tee place_can.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_basket2.png; timeout 60 python3 tools/perception/cam_snap.py agentview final_agent.png

# openrua op 23
cd /workspace; cat > home.py <<'EOF'
from arm import *
r = Robot()
Q = topdown_quat(0.0)
print("clear"); assert r.move_world([-0.15, -0.05, 0.75], Q, 4.0)
EOF
timeout 600 python3 -u home.py; timeout 120 python3 cloud.py birdview; python3 - <<'EOF'
import numpy as np
pw = np.load("birdview_xyz.npy"); bgr = np.load("birdview_bgr.npy")
# basket interior footprint
m = (pw[...,0]>-0.06)&(pw[...,0]<0.07)&(pw[...,1]>0.18)&(pw[...,1]<0.32)&(pw[...,2]>0.43)&(pw[...,2]<0.62)
pts = pw[m]; col = bgr[m]
print("points inside basket footprint above floor & below rim:", len(pts))
if len(pts):
    top = pts[pts[:,2] > pts[:,2].max()-0.01]
    print(f"highest object z={pts[:,2].max():.3f} at xy=({top[:,0].mean():.3f},{top[:,1].mean():.3f}); mean BGR {col[pts[:,2] > pts[:,2].max()-0.01].mean(0).astype(int)}")
# is anything left at the old cheese / can spots?
for name,(x0,x1,y0,y1) in {"old cheese spot":(0.03,0.15,-0.23,-0.15),"old can spot":(-0.13,-0.03,0.0,0.09)}.items():
    mm = (pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&(pw[...,2]>0.432)&(pw[...,2]<0.7)
    print(name, "points above table:", mm.sum())
EOF
timeout 60 python3 tools/perception/cam_snap.py birdview bird_final.png
