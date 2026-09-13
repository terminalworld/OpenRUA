#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 3
mkdir -p img && for c in agentview birdview frontview sideview galleryview paperview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c img/$c.png & done; wait; ls img; timeout 20 ros2 topic echo /joint_states --once

# openrua op 4
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60; echo ---; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 5
cat > /workspace/tfdump.py <<'EOF'
import rclpy, sys
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
while time.time()-t0 < 4: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),tr in sorted(seen.items()):
    print(f"{p:>28} -> {c:<28} t=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f}) q=({tr.rotation.x:.4f},{tr.rotation.y:.4f},{tr.rotation.z:.4f},{tr.rotation.w:.4f})")
EOF
timeout 30 python3 /workspace/tfdump.py

# openrua op 6
cat > /workspace/cloud.py <<'EOF'
"""Grab depth+info for a camera, transform to world, save npz of (H,W,3) world xyz."""
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
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf_static", tfcb, qos)
node.create_subscription(TFMessage, "/tf", tfcb, 100)
while not all(k in got for k in "dcitf".replace("tf","")) or "tf" not in got:
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
X = (u - cx) * depth / fx; Y = (v - cy) * depth / fy; Z = depth
t = got["tf"]; q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
P = np.stack([X, Y, Z], -1) @ R.T + np.array([t.translation.x, t.translation.y, t.translation.z])
np.savez(f"/workspace/img/{cam}_cloud.npz", xyz=P, depth=depth, color=color)
print("saved", P.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
rclpy.shutdown()
EOF
timeout 60 python3 /workspace/cloud.py birdview && timeout 60 python3 /workspace/cloud.py agentview

# openrua op 7
python3 - <<'EOF'
import numpy as np
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]; depth = d["depth"]
# table height: mode of z in the lower area
z = P[...,2]
print("table z guess (median of central region):", np.median(z[300:400, 200:450]))
# print height map of caddy region and cup region: birdview pixel ~ caddy (215-330, 155-205), cup (300-335, 245-285)
def show(r0,r1,c0,c1,step=5):
    for v in range(r0,r1,step):
        print(f"{v:4d} " + " ".join(f"{z[v,u]:.3f}" for u in range(c0,c1,step)))
print("cols", list(range(205,345,5)))
show(150,215,205,345)
print("--- cup")
print("cols", list(range(290,345,5)))
show(240,295,290,345)
EOF

# openrua op 8
python3 - <<'EOF'
import numpy as np
d = np.load("img/agentview_cloud.npz"); P = d["xyz"]
for name,(u,v) in {"left-comp floor":(165,200),"mid-back floor":(245,185),"right floor":(320,225),"cup rim":(315,300),"cup body":(315,330),"table front":(320,420),"binder":(430,270)}.items():
    print(f"{name:18s} px({u},{v}) -> world {P[v,u].round(3)}")
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]
print("birdview:")
for name,(u,v) in {"big comp":(240,180),"back-right":(280,172),"front-right":(280,190),"divider":(280,180),"cup center":(318,265),"caddy TL corner":(218,160),"caddy BL":(218,200)}.items():
    print(f"{name:18s} px({u},{v}) -> world {P[v,u].round(3)}")
EOF

# openrua op 9
cat > /workspace/fk.py <<'EOF'
"""FK via MoveIt: python3 fk.py j1,...,j7  (or 'cur' for current) -> hand pose in planning frame + world"""
import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
JOINTS = next(a for a in M["actuators"] if a["kind"]=="joint_trajectory")["joints"]
def main():
    rclpy.init(); node = rclpy.create_node("fk")
    if sys.argv[1] == "cur":
        js = {}
        node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
        while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
        d = dict(zip(js["m"].name, js["m"].position)); q = [d[j] for j in JOINTS]
        print("fingers:", d["panda_finger_joint1"], d["panda_finger_joint2"])
    else:
        q = [float(x) for x in sys.argv[1].split(",")]
    cli = node.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service(10)
    req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = q
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result(); p = r.pose_stamped[0].pose
    print("joints:", [round(x,4) for x in q])
    print(f"hand (base frame): x={p.position.x:.4f} y={p.position.y:.4f} z={p.position.z:.4f} q=({p.orientation.x:.4f},{p.orientation.y:.4f},{p.orientation.z:.4f},{p.orientation.w:.4f})")
    print(f"hand (world): x={p.position.x-0.75:.4f} y={p.position.y:.4f} z={p.position.z+0.912:.4f}")
    rclpy.shutdown()
main()
EOF
timeout 60 python3 /workspace/fk.py cur; timeout 60 python3 /workspace/fk.py 0,-0.6,0,-2.2,0,1.6,0.785

# openrua op 10
cat > /workspace/urdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("urdf")
got = {}
node.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
open("/workspace/robot.urdf","w").write(got["m"].data); print(len(got["m"].data))
EOF
timeout 30 python3 /workspace/urdf.py; grep -n -B2 -A6 'name="world"\|panda_joint0\|virtual_joint\|type="fixed"' robot.urdf | head -60

# openrua op 11
cat > /workspace/ik.py <<'EOF'
"""IK via MoveIt: python3 ik.py x y z qx qy qz qw [seed_j1,...,j7|cur] -> joint solution (no motion)"""
import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
JOINTS = next(a for a in M["actuators"] if a["kind"]=="joint_trajectory")["joints"]
def solve(node, cli, pose, seed, timeout=60):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = pose[:3]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = pose[3:]
    req.ik_request.robot_state.joint_state.name = JOINTS
    req.ik_request.robot_state.joint_state.position = list(seed)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    r = fut.result()
    if r is None: return None, "timeout"
    if r.error_code.val != 1: return None, r.error_code.val
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [d[j] for j in JOINTS], 1
def current(node):
    js = {}
    sub = node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    d = dict(zip(js["m"].name, js["m"].position)); return [d[j] for j in JOINTS]
if __name__ == "__main__":
    rclpy.init(); node = rclpy.create_node("ik")
    cli = node.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service(10)
    pose = [float(x) for x in sys.argv[1:8]]
    seed = current(node) if len(sys.argv) < 9 or sys.argv[8]=="cur" else [float(x) for x in sys.argv[8].split(",")]
    sol, code = solve(node, cli, pose, seed)
    print("code", code, "sol", None if sol is None else ",".join(f"{x:.4f}" for x in sol))
    rclpy.shutdown()
EOF
echo "world-frame pose:"; timeout 90 python3 /workspace/ik.py -0.293 0 1.2696 1 0 0 0
echo "base-frame pose:"; timeout 90 python3 /workspace/ik.py 0.457 0 0.358 1 0 0 0

# openrua op 12
timeout 60 python3 /workspace/fk.py 0.0586,-0.1717,-0.0583,-2.4699,-0.0133,2.2985,0.0100

# openrua op 13
python3 - <<'EOF'
import numpy as np
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]; z = P[...,2]
reg = P[240:295, 285:345]; zz = reg[...,2]
m = zz > 0.9
pts = reg[m]
print("cup pixels above table:", m.sum())
print("x range", pts[:,0].min().round(3), pts[:,0].max().round(3), "y range", pts[:,1].min().round(3), pts[:,1].max().round(3), "zmax", pts[:,2].max().round(3))
# body only (exclude handle): z>1.0 -> rim
rim = reg[zz > 1.0]
print("rim pts", len(rim), "x", rim[:,0].min().round(3), rim[:,0].max().round(3), "y", rim[:,1].min().round(3), rim[:,1].max().round(3))
print("rim center", rim[:,:2].mean(0).round(3))
# per-pixel fine map around cup
for v in range(248, 282, 2):
    print(f"{v:4d} " + "".join(("#" if z[v,u]>1.0 else ("+" if z[v,u]>0.9 else ".")) for u in range(290, 340)))
EOF

# openrua op 14
python3 - <<'EOF'
import numpy as np
d = np.load("img/agentview_cloud.npz"); P = d["xyz"]
for v in range(285, 365, 4):
    print(v, P[v,315].round(3), P[v,300].round(3))
EOF

# openrua op 15
python3 - <<'EOF'
import numpy as np
d = np.load("img/agentview_cloud.npz"); P = d["xyz"]
for v in range(357, 380, 1):
    print(v, P[v,315].round(3), P[v,320].round(3), P[v,310].round(3))
EOF

# openrua op 16
timeout 60 python3 /workspace/fk.py 0.6,-0.5,0,-2.0,0,1.5,0.785

# openrua op 17
timeout 300 python3 tools/action/fjt_send.py 0.6,-0.5,0,-2.0,0,1.5,0.785 3 && timeout 60 python3 fk.py cur

# openrua op 18
timeout 60 python3 cloud.py birdview && timeout 60 python3 tools/perception/cam_snap.py birdview img/birdview2.png && python3 - <<'EOF'
import numpy as np
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]; z = P[...,2]
print("table", np.median(z[300:400,200:450]).round(3))
print("cols", list(range(205,360,5)))
for v in range(150,215,3):
    print(f"{v:4d} " + " ".join(f"{z[v,u]:.2f}" for u in range(205,360,5)))
reg = P[240:295, 285:345]; zz = reg[...,2]; pts = reg[zz>0.9]
print("cup: x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "zmax", pts[:,2].max().round(3))
EOF

# openrua op 19
python3 - <<'EOF'
import numpy as np
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]; z = P[...,2]
# vertical profile through middle section col 280 and right col 320, left col 240
for c in (240, 280, 320):
    print(f"col {c}: y={P[170,c,1]:.3f}")
    print(" ".join(f"{v}:{z[v,c]:.2f}@{P[v,c,0]:.3f}" for v in range(154, 202, 1)))
# horizontal profile through row 170 and 190
for r in (170, 190):
    print(f"row {r}: x={P[r,280,0]:.3f}")
    print(" ".join(f"{u}:{z[r,u]:.2f}@{P[r,u,1]:.3f}" for u in range(216, 346, 1)))
EOF

# openrua op 20
timeout 60 python3 cloud.py frontview && timeout 60 python3 tools/perception/cam_snap.py frontview img/frontview2.png && python3 - <<'EOF'
import numpy as np
d = np.load("img/frontview_cloud.npz"); P = d["xyz"]; z = P[...,2]
# cup at world x~-0.12, y~-0.02; find pixels with x in [-0.2,-0.04], y in [-0.12,0.08], z>0.885
m = (P[...,0]>-0.2)&(P[...,0]<-0.04)&(P[...,1]>-0.12)&(P[...,1]<0.08)&(z>0.885)
pts = P[m]
print("n", m.sum(), "z range", pts[:,2].min().round(3), pts[:,2].max().round(3))
for zlo in np.arange(0.885, 1.0, 0.01):
    sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(sel): print(f"z {zlo:.3f}-{zlo+0.01:.3f}: y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}] width {sel[:,1].max()-sel[:,1].min():.3f}  x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] n={len(sel)}")
EOF

# openrua op 21
timeout 30 ros2 node info /robot_bridge; timeout 20 ros2 topic echo /pipeline_state --once 2>&1 | head -30

# openrua op 22
cat .ros/log/*.log | grep -iv "^$" | head -80; timeout 20 ros2 param list /robot_bridge

# openrua op 23
python3 -c "
import cv2
im = cv2.imread('img/birdview2.png'); crop = im[140:215, 200:360]; cv2.imwrite('img/caddy_zoom.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
im = cv2.imread('img/agentview.png'); crop = im[140:290, 100:380]; cv2.imwrite('img/caddy_zoom2.png', cv2.resize(crop, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))
"

# openrua op 24
timeout 60 python3 cloud.py sideview && python3 - <<'EOF'
import numpy as np
for cam in ("sideview","frontview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.2)&(P[...,0]<-0.04)&(P[...,1]>-0.05)&(P[...,1]<0.08)&(z>0.882)   # exclude handle side y<-0.05
    pts = P[m]
    print(cam, "n", m.sum())
    for zlo in np.arange(0.882, 1.0, 0.005):
        sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.005)]
        if len(sel): print(f" z {zlo:.3f}: x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] dx={sel[:,0].max()-sel[:,0].min():.3f}  y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}] dy={sel[:,1].max()-sel[:,1].min():.3f} n={len(sel)}")
EOF

# openrua op 25
cat > /workspace/lib.py <<'EOF'
"""Reusable robot helpers: IK (world frame, link8 target), FJT, gripper, joint state."""
import math, time, numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
# link8 orientation for hand fingers along world y (hand q = (1,0,0,0))
Q_FINGERS_Y = (0.9239, -0.3827, 0.0, 0.0)
# link8 orientation for fingers along world x (hand q = Rx(180)*Rz(90) = (0.7071,0.7071,0,0))
Q_FINGERS_X = (1.0, 0.0, 0.0, 0.0)

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("agent_lib")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.wr = {}
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: self.wr.__setitem__("m", m), 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js["panda_finger_joint1"], self.js["panda_finger_joint2"]

    def wrench(self):
        self.wr = {}
        while "m" not in self.wr:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        w = self.wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def fk_pose(self, q=None, link="panda_link8"):
        q = q if q is not None else self.joints()
        req = GetPositionFK.Request(); req.fk_link_names = [link]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z,
                         p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def solve_ik(self, pos, quat, seed=None, tries=1):
        seed = seed if seed is not None else self.joints()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = JOINTS
            s = list(seed) if k == 0 else list(np.array(seed) + np.random.uniform(-0.3, 0.3, 7))
            req.ik_request.robot_state.joint_state.position = s
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=2)
            fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [d[j] for j in JOINTS]
        return None

    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pts.append(JointTrajectoryPoint(positions=list(map(float, v)),
                            time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
        pts.append(JointTrajectoryPoint(positions=list(map(float, q)),
                    time_from_start=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))))
        goal.trajectory.points = pts
        fut = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        cur = np.array(self.joints())
        err = np.abs(cur - np.array(q)).max()
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.solve_ik(pos, quat, seed=seed, tries=5)
        if q is None:
            return None, None, None
        code, err = self.move_joints(q, seconds)
        return q, code, err

    def gripper(self, width, timeout=120):
        g = GripperCommand.Goal(); g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def close(self):
        rclpy.shutdown()
EOF
cat > /workspace/step_pregrasp.py <<'EOF'
import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot, Q_FINGERS_Y
r = Robot()
print("start joints", np.round(r.joints(), 3), "fingers", r.fingers())
print("link8 pose now", np.round(r.fk_pose(), 4))
target = [-0.114, 0.057, 1.14]
q = r.solve_ik(target, Q_FINGERS_Y, tries=5)
print("IK pregrasp:", None if q is None else np.round(q, 4))
if q is not None:
    print("FK of solution:", np.round(r.fk_pose(q), 4))
    code, err = r.move_joints(q, 4.0)
    print("moved code", code, "max joint err", round(err, 4))
    print("link8 pose after", np.round(r.fk_pose(), 4))
r.close()
EOF
timeout 600 python3 -u /workspace/step_pregrasp.py

# openrua op 26
cat > /workspace/goto.py <<'EOF'
"""python3 goto.py x y z [qx qy qz qw] [seconds]  -- IK (link8, world) + trajectory, retry until converged"""
import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot, Q_FINGERS_Y
a = [float(x) for x in sys.argv[1:]]
pos = a[:3]; quat = a[3:7] if len(a) >= 7 else Q_FINGERS_Y; secs = a[7] if len(a) >= 8 else (a[3] if len(a) == 4 else 3.0)
r = Robot()
q = r.solve_ik(pos, quat, tries=8)
if q is None:
    print("IK FAILED"); r.close(); sys.exit(1)
print("target joints", np.round(q, 4))
for attempt in range(4):
    code, err = r.move_joints(q, secs)
    print(f"attempt {attempt}: code {code} max joint err {err:.4f}")
    if err < 0.02: break
    secs = max(secs, 3.0)
print("link8 pose", np.round(r.fk_pose(), 4), "fingers", np.round(r.fingers(), 4))
r.close()
EOF
timeout 900 python3 -u /workspace/goto.py -0.114 0.057 1.14 4

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih1.png

# openrua op 28
timeout 900 python3 -u /workspace/goto.py -0.114 0.057 1.058 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih2.png && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent2.png

# openrua op 29
cat > /workspace/grip.py <<'EOF'
import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot
r = Robot()
w0 = r.wrench()
print("fingers before", np.round(r.fingers(), 4))
res = r.gripper(float(sys.argv[1]))
print("reached_goal, stalled, fingers:", res)
r.spin(0.5)
print("fingers after settle", np.round(r.fingers(), 4))
print("wrench before", np.round(w0, 2)); print("wrench after ", np.round(r.wrench(), 2))
r.close()
EOF
timeout 300 python3 -u /workspace/grip.py 0.0

# openrua op 30
timeout 900 python3 -u /workspace/goto.py -0.114 0.057 1.30 3 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent3.png

# openrua op 31
timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]; z = P[...,2]
# region around expected cup (x -0.25..0.0, y -0.15..0.15), z between 1.10 and 1.26 (cup body/rim; hand is higher ~1.30+)
m = (P[...,0]>-0.3)&(P[...,0]<0.05)&(P[...,1]>-0.2)&(P[...,1]<0.2)
pts = P[m]
for zlo in np.arange(1.08, 1.45, 0.02):
    sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
    if len(sel)>5: print(f"z {zlo:.2f}: x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}] n={len(sel)}")
rim = pts[(pts[:,2]>1.20)&(pts[:,2]<1.25)]
print("rim-band pts", len(rim))
# estimate the cup circle: fit using extreme x (handle-free axis)
if len(rim):
    print("cup center est (x from extremes, y from +y edge - 0.05):", ((rim[:,0].min()+rim[:,0].max())/2).round(4), (rim[:,1].max()-0.05).round(4))
EOF

# openrua op 32
timeout 60 python3 cloud.py frontview && timeout 60 python3 cloud.py sideview && python3 - <<'EOF'
import numpy as np
for cam in ("frontview","sideview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.3)&(P[...,0]<0.05)&(P[...,1]>-0.2)&(P[...,1]<0.2)&(z>1.05)&(z<1.26)
    pts = P[m]
    print(cam)
    for zlo in np.arange(1.08, 1.26, 0.01):
        sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
        if len(sel)>5: print(f" z {zlo:.2f}: x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}] n={len(sel)}")
EOF

# openrua op 33
timeout 900 python3 -u /workspace/goto.py -0.445 -0.0915 1.30 4 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent4.png

# openrua op 34
timeout 60 python3 cloud.py frontview && timeout 60 python3 cloud.py sideview && timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
for cam in ("frontview","sideview","birdview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.6)&(P[...,0]<-0.3)&(P[...,1]>-0.3)&(P[...,1]<0.1)&(z>1.07)&(z<1.24)
    pts = P[m]
    print(cam)
    for zlo in np.arange(1.10, 1.24, 0.01):
        sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
        if len(sel)>5: print(f" z {zlo:.2f}: x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}] n={len(sel)}")
EOF

# openrua op 35
cat > /workspace/lower.py <<'EOF'
import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot, Q_FINGERS_Y
r = Robot()
x, y = -0.445, -0.0915
w0 = r.wrench(); print("wrench start", np.round(w0, 2), "fingers", np.round(r.fingers(), 4))
for z in [float(v) for v in sys.argv[1:]]:
    q = r.solve_ik([x, y, z], Q_FINGERS_Y, tries=8)
    if q is None: print("IK fail at", z); break
    for a in range(3):
        code, err = r.move_joints(q, 2.0)
        if err < 0.02: break
    r.spin(0.3)
    w = r.wrench()
    print(f"z={z:.3f} code={code} err={err:.4f} pose={np.round(r.fk_pose()[:3],4)} fingers={np.round(r.fingers(),4)} wrench={np.round(w,2)} dF={np.round(w[:3]-w0[:3],2)}")
r.close()
EOF
timeout 900 python3 -u /workspace/lower.py 1.26 1.245 1.235

# openrua op 36
timeout 300 python3 -u /workspace/grip.py 0.04 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent5.png

# openrua op 37
timeout 900 python3 -u /workspace/goto.py -0.35 0.25 1.45 4 && for c in agentview birdview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c img/${c}_final.png; done

# openrua op 38
timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np, cv2
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]; z = P[...,2]
im = cv2.imread("img/birdview_final.png"); crop = im[140:215, 200:360]; cv2.imwrite("img/caddy_zoom_final.png", cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
# cup points: within caddy area x [-0.5,-0.33], y [-0.36, 0.08], z > 1.07 (above walls) or interior anomalies
m = (P[...,0]>-0.52)&(P[...,0]<-0.30)&(P[...,1]>-0.36)&(P[...,1]<0.08)
pts = P[m]
cup = pts[pts[:,2]>1.065]
print("pts above wall tops:", len(cup))
if len(cup): print(" x [%.3f,%.3f] y [%.3f,%.3f] z [%.3f,%.3f]" % (cup[:,0].min(),cup[:,0].max(),cup[:,1].min(),cup[:,1].max(),cup[:,2].min(),cup[:,2].max()))
print("cols", list(range(255,305,3)))
for v in range(154,204,2):
    print(f"{v:4d} x={P[v,280,0]:.3f} " + " ".join(f"{z[v,u]:.2f}" for u in range(255,305,3)))
EOF

# openrua op 39
cat >> /workspace/lib.py <<'EOF'

# --- orientation helpers -------------------------------------------------
from scipy.spatial.transform import Rotation as Rot

def link8_quat_from_hand_axes(hand_x, hand_y, hand_z):
    """Return link8 quaternion (x,y,z,w) given desired world-frame hand axes."""
    Rh = np.column_stack([hand_x, hand_y, hand_z])
    R8 = Rot.from_matrix(Rh) * Rot.from_euler("z", 45, degrees=True)   # hand = link8 * Rz(-45)
    return R8.as_quat()

# hand pointing down (z=-Z), fingers along world y  -> should equal Q_FINGERS_Y
Q_DOWN_FY = link8_quat_from_hand_axes([1,0,0],[0,-1,0],[0,0,-1])
# hand pointing down, fingers along world x
Q_DOWN_FX = link8_quat_from_hand_axes([0,1,0],[1,0,0],[0,0,-1])
# hand pointing toward -x (horizontal), fingers along world z
Q_BACK_FZ = link8_quat_from_hand_axes([0,-1,0],[0,0,1],[-1,0,0])
EOF
python3 -c "
import sys; sys.path.insert(0,'/workspace')
import lib, numpy as np
print('Q_DOWN_FY', np.round(lib.Q_DOWN_FY,4), 'expected', lib.Q_FINGERS_Y)
print('Q_DOWN_FX', np.round(lib.Q_DOWN_FX,4))
print('Q_BACK_FZ', np.round(lib.Q_BACK_FZ,4))
"

# openrua op 40
cat > /workspace/goto2.py <<'EOF'
"""python3 goto2.py x y z QNAME [seconds] -- IK+trajectory with named orientation from lib"""
import sys, numpy as np
sys.path.insert(0, "/workspace")
import lib
from lib import Robot
pos = [float(x) for x in sys.argv[1:4]]; quat = getattr(lib, sys.argv[4]); secs = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
r = Robot()
q = r.solve_ik(pos, quat, tries=10)
if q is None:
    print("IK FAILED"); r.close(); sys.exit(1)
print("target joints", np.round(q, 4))
for attempt in range(4):
    code, err = r.move_joints(q, secs)
    print(f"attempt {attempt}: code {code} max joint err {err:.4f}")
    if err < 0.02: break
    secs = max(secs, 3.0)
print("link8 pose", np.round(r.fk_pose(), 4), "fingers", np.round(r.fingers(), 4), "wrench", np.round(r.wrench(), 2))
r.close()
EOF
timeout 900 python3 -u /workspace/goto2.py -0.20 -0.14 1.35 Q_BACK_FZ 4

# openrua op 41
timeout 900 python3 -u /workspace/goto2.py -0.20 -0.14 1.11 Q_BACK_FZ 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih3.png && timeout 60 python3 tools/perception/cam_snap.py sideview img/side3.png

# openrua op 42
timeout 60 python3 cloud.py birdview && timeout 60 python3 cloud.py sideview && python3 - <<'EOF'
import numpy as np
for cam in ("birdview","sideview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.50)&(P[...,0]<-0.30)&(P[...,1]>-0.25)&(P[...,1]<-0.05)&(z>1.065)&(z<1.20)
    pts = P[m]; print(cam, len(pts))
    for xlo in np.arange(-0.47, -0.33, 0.01):
        sel = pts[(pts[:,0]>=xlo)&(pts[:,0]<xlo+0.01)]
        if len(sel)>3:
            top = sel[np.argmax(sel[:,2])]
            print(f" x {xlo:.2f}: n={len(sel)} zmax={sel[:,2].max():.3f} at y={top[1]:.3f}; y range [{sel[:,1].min():.3f},{sel[:,1].max():.3f}]")
EOF

# openrua op 43
timeout 900 python3 -u /workspace/goto2.py -0.20 -0.125 1.111 Q_BACK_FZ 2 && timeout 900 python3 -u /workspace/goto2.py -0.272 -0.125 1.111 Q_BACK_FZ 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih4.png && timeout 60 python3 tools/perception/cam_snap.py sideview img/side4.png

# openrua op 44
timeout 300 python3 -u /workspace/grip.py 0.0

# openrua op 45
timeout 900 python3 -u /workspace/goto2.py -0.272 -0.125 1.36 Q_BACK_FZ 3 && timeout 60 python3 tools/perception/cam_snap.py sideview img/side5.png && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent6.png

# openrua op 46
timeout 60 python3 cloud.py sideview && timeout 60 python3 cloud.py birdview && timeout 60 python3 cloud.py frontview && python3 - <<'EOF'
import numpy as np
for cam in ("sideview","birdview","frontview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.55)&(P[...,0]<-0.25)&(P[...,1]>-0.25)&(P[...,1]<-0.05)&(z>1.15)&(z<1.40)
    pts = P[m]; print(cam, len(pts))
    for zlo in np.arange(1.15, 1.40, 0.02):
        sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
        if len(sel)>3: print(f" z {zlo:.2f}: n={len(sel)} x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}]")
EOF

# openrua op 47
cat >> /workspace/lib.py <<'EOF'
# hand pointing down, fingers along x, hand x = -Y (matches Ry(-90) applied to Q_BACK_FZ)
Q_DOWN_FX2 = link8_quat_from_hand_axes([0,-1,0],[-1,0,0],[0,0,-1])
EOF
python3 -c "
import sys; sys.path.insert(0,'/workspace'); import lib, numpy as np
from scipy.spatial.transform import Rotation as R
print('Q_DOWN_FX2', np.round(lib.Q_DOWN_FX2,4))
q8 = R.from_quat(lib.Q_BACK_FZ); r = R.from_euler('y', -90, degrees=True)*q8
print('Ry(-90)*Q_BACK_FZ', np.round(r.as_quat(),4))
"
timeout 900 python3 -u /workspace/goto2.py -0.272 -0.125 1.36 Q_DOWN_FX2 4 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent7.png && timeout 60 python3 tools/perception/cam_snap.py sideview img/side7.png

# openrua op 48
timeout 60 python3 cloud.py sideview && timeout 60 python3 cloud.py frontview && timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
for cam in ("sideview","frontview","birdview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.45)&(P[...,0]<-0.10)&(P[...,1]>-0.25)&(P[...,1]<-0.0)&(z>1.10)&(z<1.34)
    pts = P[m]; print(cam, len(pts))
    for zlo in np.arange(1.10, 1.34, 0.02):
        sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
        if len(sel)>3: print(f" z {zlo:.2f}: n={len(sel)} x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}]")
EOF

# openrua op 49
timeout 900 python3 -u /workspace/goto2.py -0.464 -0.1615 1.30 Q_DOWN_FX2 4 && timeout 60 python3 cloud.py sideview && timeout 60 python3 cloud.py frontview && python3 - <<'EOF'
import numpy as np
for cam in ("sideview","frontview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.60)&(P[...,0]<-0.30)&(P[...,1]>-0.30)&(P[...,1]<-0.0)&(z>1.07)&(z<1.25)
    pts = P[m]; print(cam, len(pts))
    for zlo in np.arange(1.07, 1.25, 0.02):
        sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
        if len(sel)>3: print(f" z {zlo:.2f}: n={len(sel)} x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}]")
EOF

# openrua op 50
timeout 900 python3 -u /workspace/goto2.py -0.475 -0.1365 1.30 Q_DOWN_FX2 3 && timeout 60 python3 cloud.py sideview && python3 - <<'EOF'
import numpy as np
d = np.load("img/sideview_cloud.npz"); P = d["xyz"]; z = P[...,2]
m = (P[...,0]>-0.60)&(P[...,0]<-0.30)&(P[...,1]>-0.30)&(P[...,1]<-0.0)&(z>1.07)&(z<1.25)
pts = P[m]
for zlo in np.arange(1.07, 1.25, 0.02):
    sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
    if len(sel)>3: print(f" z {zlo:.2f}: n={len(sel)} x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}]")
EOF

# openrua op 51
for z in 1.25 1.22 1.205 1.195 1.185; do echo "== z $z"; timeout 300 python3 -u goto2.py -0.475 -0.1365 $z Q_DOWN_FX2 2 | grep -E "wrench|attempt"; done

# openrua op 52
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent8.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py sideview img/side8.png >/dev/null && timeout 60 python3 cloud.py sideview >/dev/null && python3 - <<'EOF'
import numpy as np
d = np.load("img/sideview_cloud.npz"); P = d["xyz"]; z = P[...,2]
m = (P[...,0]>-0.60)&(P[...,0]<-0.30)&(P[...,1]>-0.30)&(P[...,1]<-0.0)&(z>0.95)&(z<1.20)
pts = P[m]
for zlo in np.arange(0.95, 1.20, 0.02):
    sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
    if len(sel)>3: print(f" z {zlo:.2f}: n={len(sel)} x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}]")
EOF

# openrua op 53
timeout 300 python3 -u grip.py 0.04 && timeout 300 python3 -u goto2.py -0.475 -0.1365 1.32 Q_DOWN_FX2 2 | grep -E "attempt|wrench" && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent9.png >/dev/null

# openrua op 54
timeout 300 python3 -u goto2.py -0.30 0.30 1.45 Q_DOWN_FX2 3 | grep -E "attempt" ; timeout 300 python3 -u goto2.py 0.0 0.35 1.40 Q_DOWN_FX2 3 | grep -E "attempt"; for c in agentview birdview sideview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c img/${c}_done.png >/dev/null; done; timeout 60 python3 cloud.py birdview >/dev/null; timeout 60 python3 cloud.py sideview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ("birdview","sideview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]; z = P[...,2]
    m = (P[...,0]>-0.60)&(P[...,0]<-0.20)&(P[...,1]>-0.40)&(P[...,1]<0.10)&(z>0.95)&(z<1.30)
    pts = P[m]; print(cam, "max z", pts[:,2].max() if len(pts) else None)
    for zlo in np.arange(0.95, 1.30, 0.02):
        sel = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.02)]
        if len(sel)>3: print(f" z {zlo:.2f}: n={len(sel)} x [{sel[:,0].min():.3f},{sel[:,0].max():.3f}] y [{sel[:,1].min():.3f},{sel[:,1].max():.3f}]")
EOF

# openrua op 55
python3 -c "
import cv2
im = cv2.imread('img/birdview_done.png'); h,w = im.shape[:2]
# caddy region approx: world x[-0.55,-0.30], y[-0.40,0.12]; birdview center (-0.2,0), 3.62mm/px, right=+y, down=+x
cx, cy = w//2, h//2; s = 0.00362
def px(x,y): return int(cx + y/s), int(cy + (x+0.2)/s)
u0,v0 = px(-0.55,-0.42); u1,v1 = px(-0.28,0.14)
crop = im[max(v0,0):v1, max(u0,0):u1]; crop = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
cv2.imwrite('img/bird_done_zoom.png', crop); print(crop.shape)
"

# openrua op 56
python3 - <<'EOF'
import numpy as np
for cam in ("birdview","sideview"):
    d = np.load(f"img/{cam}_cloud.npz"); P = d["xyz"]
    m = (P[...,0]>-0.50)&(P[...,0]<-0.33)&(P[...,1]>-0.21)&(P[...,1]<-0.06)&(P[...,2]>1.062)
    pts = P[m]; print(cam, len(pts))
    for xlo in np.arange(-0.48,-0.35,0.01):
        s = pts[(pts[:,0]>=xlo)&(pts[:,0]<xlo+0.01)]
        if len(s)>2: print(f"  x {xlo:.2f}: n={len(s)} zmax {s[:,2].max():.3f} zmed {np.median(s[:,2]):.3f} y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 57
timeout 300 python3 -u goto2.py 0.0 0.35 1.35 Q_DOWN_FX2 4 | grep attempt; timeout 300 python3 -u goto2.py 0.0 0.35 1.40 Q_DOWN_FX2 4 | grep attempt; timeout 60 python3 cloud.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_final2.png >/dev/null; python3 - <<'EOF'
import numpy as np
d = np.load("img/birdview_cloud.npz"); P = d["xyz"]
m = (P[...,0]>-0.50)&(P[...,0]<-0.33)&(P[...,1]>-0.21)&(P[...,1]<-0.06)&(P[...,2]>1.062)
pts = P[m]
print("rim pts", len(pts), "x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
print("rim center approx", (pts[:,0].min()+pts[:,0].max())/2, (pts[:,1].min()+pts[:,1].max())/2)
# anything of cup height elsewhere on the table?
m2 = (P[...,2]>0.92)&(P[...,2]<1.3)&(P[...,0]>-0.9)&(P[...,0]<0.4)&(P[...,1]>-0.6)&(P[...,1]<0.6)
q = P[m2]; q = q[~((q[:,0]>-0.50)&(q[:,0]<-0.33)&(q[:,1]>-0.36)&(q[:,1]<0.08))]
print("other raised points outside caddy:", len(q), (q[:,0].min(), q[:,0].max(), q[:,1].min(), q[:,1].max()) if len(q) else "")
EOF
