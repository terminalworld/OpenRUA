#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/ik_move.py; echo ----; cat tools/action/gripper_cmd.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; timeout 20 ros2 topic echo /joint_states --once; timeout 10 ros2 topic info /pipeline_state; timeout 10 ros2 topic echo /pipeline_state --once

# openrua op 5
timeout 20 ros2 node info /robot_bridge; timeout 10 ros2 interface show moveit_msgs/msg/PipelineState | head -30

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 7
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -B2 -A12 "child_frame_id: panda_link0" | head -30

# openrua op 8
cat > /workspace/tfdump.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node("tfdump")
qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
got = {}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
n.create_subscription(TFMessage, "/tf_static", cb, qos)
n.create_subscription(TFMessage, "/tf", cb, 10)
import time
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
for k, t in got.items():
    print(k, f"t=({t.translation.x:.3f},{t.translation.y:.3f},{t.translation.z:.3f}) q=({t.rotation.x:.3f},{t.rotation.y:.3f},{t.rotation.z:.3f},{t.rotation.w:.3f})")
EOF
timeout 40 python3 /workspace/tfdump.py

# openrua op 9
cat > /workspace/px2w_batch.py <<'EOF'
"""Usage: px2w_batch.py <camera> u,v [u,v ...]  -> world xyz for each pixel of current depth frame"""
import struct, sys
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def main():
    cam = sys.argv[1]; pts = [tuple(map(int,a.split(","))) for a in sys.argv[2:]]
    rclpy.init(); n = rclpy.create_node("px2wb")
    got = {}
    n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
    n.create_subscription(TFMessage, "/tf_static", tfcb, qos)
    n.create_subscription(TFMessage, "/tf", tfcb, 10)
    while len(got) < 3: rclpy.spin_once(n, timeout_sec=0.2)
    d, i, t = got["d"], got["i"], got["tf"]
    depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    fx, fy, cx, cy = i.k[0], i.k[4], i.k[2], i.k[5]
    R = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T = np.array([t.translation.x, t.translation.y, t.translation.z])
    for u, v in pts:
        z = depth[v, u]
        p = R @ np.array([(u-cx)*z/fx, (v-cy)*z/fy, z]) + T
        print(f"({u},{v}) depth={z:.3f} world=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    rclpy.shutdown()
main()
EOF
timeout 60 python3 /workspace/px2w_batch.py birdview 250,288 325,315 378,280 378,237 200,200 450,380 320,240

# openrua op 10
cat > /workspace/heightmap.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
sys.path.insert(0, "/workspace")
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
rclpy.init(); n = rclpy.create_node("hm")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf_static", tfcb, qos)
while len(got) < 3: rclpy.spin_once(n, timeout_sec=0.2)
d, i, t = got["d"], got["i"], got["tf"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = i.k[0], i.k[4], i.k[2], i.k[5]
R = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
T = np.array([t.translation.x, t.translation.y, t.translation.z])
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
np.save("/workspace/P_%s.npy" % cam, P)
Z = P[...,2]
mask = ((Z > 0.905) & (Z < 1.3) & (P[...,0] > -0.4)).astype(np.uint8)
nlab, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for k in range(1, nlab):
    if stats[k, cv2.CC_STAT_AREA] < 30: continue
    m = lab == k
    print(f"comp {k}: area={stats[k,4]} px bbox(u,v,w,h)={stats[k,:4]} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] zmax={Z[m].max():.3f} centroid=({P[m][:,0].mean():.3f},{P[m][:,1].mean():.3f})")
rclpy.shutdown()
EOF
timeout 90 python3 /workspace/heightmap.py birdview

# openrua op 11
sed -i 's|n.create_subscription(TFMessage, "/tf_static", tfcb, qos)|n.create_subscription(TFMessage, "/tf_static", tfcb, qos); n.create_subscription(TFMessage, "/tf", tfcb, 10)|' heightmap.py && timeout 120 python3 /workspace/heightmap.py birdview

# openrua op 12
cat > /workspace/analyze.py <<'EOF'
import numpy as np, cv2
P = np.load("/workspace/P_birdview.npy"); Z = P[...,2]
def comps(mask, minarea=30, label=""):
    nlab, lab, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for k in range(1, nlab):
        if stats[k,4] < minarea: continue
        m = lab == k
        print(f"{label} comp {k}: area={stats[k,4]} bbox={stats[k,:4]} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] z[{Z[m].min():.3f},{Z[m].max():.3f}] centroid=({P[m][:,0].mean():.3f},{P[m][:,1].mean():.3f})")
comps((Z > 0.915) & (Z < 0.945) & (P[...,0] > -0.4) & (P[...,1] > 0.0), label="stove-ish")
comps((Z > 0.95) & (Z < 1.05) & (P[...,0] > -0.4) & (P[...,1] > -0.1), label="pan-ish")
# moka pot body: top region
comps((Z > 1.0) & (P[...,1] < -0.1), label="moka top")
comps((Z > 0.905) & (P[...,1] < -0.1) & (P[...,0] > -0.4), label="moka all")
EOF
python3 /workspace/analyze.py

# openrua op 13
python3 -c "
import cv2
im = cv2.imread('agentview.png'); cv2.imwrite('crop_knob_agent.png', cv2.resize(im[190:280, 370:480], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('frontview.png'); cv2.imwrite('crop_knob_front.png', cv2.resize(im[300:370, 370:470], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[210:350, 210:420], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
"

# openrua op 14
cat > /workspace/analyze2.py <<'EOF'
import numpy as np
P = np.load("/workspace/P_birdview.npy"); Z = P[...,2]
reg = (P[...,1] < -0.1) & (P[...,1] > -0.4) & (P[...,0] > -0.2) & (P[...,0] < 0.1)
for zt in [0.92, 0.95, 0.98, 1.0, 1.02, 1.04, 1.05]:
    m = reg & (Z > zt)
    if m.sum()==0: continue
    print(f"z>{zt}: n={m.sum()} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] cx={P[m][:,0].mean():.3f} cy={P[m][:,1].mean():.3f}")
# row/col profile at top: for each y-bin, print max z
m = reg & (Z > 0.91)
ys = P[m][:,1]; zs = Z[m]; xs = P[m][:,0]
for yb in np.arange(-0.33, -0.17, 0.01):
    s = (ys >= yb) & (ys < yb+0.01)
    if s.sum(): print(f"y={yb:.2f}: zmax={zs[s].max():.3f} xrange[{xs[s].min():.3f},{xs[s].max():.3f}] n={s.sum()}")
EOF
python3 /workspace/analyze2.py

# openrua op 15
sed -i 's|mask = ((Z > 0.905) \& (Z < 1.3) \& (P\[...,0\] > -0.4)).astype(np.uint8)|mask = ((Z > 0.905) \& (Z < 1.3) \& (P[...,0] > -0.4) \& (P[...,0] < 0.4)).astype(np.uint8)|' heightmap.py && timeout 120 python3 heightmap.py sideview && timeout 120 python3 heightmap.py frontview

# openrua op 16
cat > /workspace/analyze3.py <<'EOF'
import numpy as np
for cam in ["sideview", "frontview"]:
    P = np.load(f"/workspace/P_{cam}.npy"); Z = P[...,2]
    reg = (P[...,1] < -0.15) & (P[...,1] > -0.35) & (P[...,0] > -0.15) & (P[...,0] < 0.1) & (Z > 0.905)
    print(cam)
    for zb in np.arange(0.90, 1.07, 0.01):
        s = reg & (Z >= zb) & (Z < zb+0.01)
        if s.sum(): print(f" z={zb:.2f}: n={s.sum()} x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}]")
EOF
python3 analyze3.py

# openrua op 17
cat > /workspace/lib.py <<'EOF'
"""Reusable helpers: joint state, FK (via /compute_fk), IK, trajectory, gripper, servo."""
import time
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 translation

def quat_R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class Robot:
    def __init__(self, name="agent"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None; self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.spin(0.5)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m
    def spin(self, sec=0.2):
        end = time.time() + sec
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = None
        while self._js is None: rclpy.spin_once(self.node, timeout_sec=0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d
    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]
    def finger_gap(self):
        d = self.joints(); return abs(d["panda_finger_joint1"]) + abs(d["panda_finger_joint2"])
    def wrench(self):
        self._wr = None
        while self._wr is None: rclpy.spin_once(self.node, timeout_sec=0.1)
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk_world(self, q=None):
        """hand pose in WORLD frame: (pos[3], quat[4])"""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]), r.pose_stamped[0].header.frame_id

    def ik_world(self, pos, quat, seed=None):
        """IK for HAND pose given in WORLD frame -> list of arm joints or None"""
        pos = np.asarray(pos, float) - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        s = JointState(); s.name = list(ARM); s.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None: print("IK: no answer"); return None
        if r.error_code.val != 1: print(f"IK failed code={r.error_code.val}"); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, sec=3.0, verbose=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        if verbose: print(f"move_q: code={code} max_err={err:.4f}")
        return code, err

    def move_qs(self, qs, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, s in zip(qs, secs):
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()
        print(f"move_qs: code={code} max_err={err:.4f}")
        return code, err

    def move_pose(self, pos, quat, sec=3.0, seed=None):
        q = self.ik_world(pos, quat, seed)
        if q is None: return None
        self.move_q(q, sec)
        p, _, _ = self.fk_world()
        print(f"  reached world pos {np.round(p,4)} (target {np.round(pos,4)})")
        return q

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin=(0,0,0), ang=(0,0,0), ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            self.twist.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)
EOF
cat > /workspace/status.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("status")
q = r.arm_q(); print("arm q:", np.round(q, 4))
p, quat, fid = r.fk_world(); print("hand world pos:", np.round(p,4), "quat:", np.round(quat,4), "fk frame:", fid)
print("finger gap:", round(r.finger_gap(),4)); print("wrench:", r.wrench())
EOF
timeout 120 python3 status.py

# openrua op 18
cat >> /workspace/tfdump.py <<'EOF'
import numpy as np
from lib import quat_R
chain = [("world","panda_link0"),("panda_link0","panda_link1"),("panda_link1","panda_link2"),("panda_link2","panda_link3"),("panda_link3","panda_link4"),("panda_link4","panda_link5"),("panda_link5","panda_link6"),("panda_link6","panda_link7"),("panda_link7","panda_link8"),("panda_link8","panda_hand")]
T = np.eye(4)
for k in chain:
    t = got[k]; A = np.eye(4); A[:3,:3] = quat_R([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]); A[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    T = T @ A
    print(k[1], np.round(T[:3,3],4))
EOF
timeout 60 python3 tfdump.py 2>&1 | tail -10

# openrua op 19
sed -i 's|^BASE = np.array(\[-0.660, 0.0, 0.912\]).*|BASE = np.zeros(3)  # FK/IK model frame verified == world (TF chain matches /compute_fk)|' lib.py && cat > /workspace/iktest.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("iktest")
p, q, _ = r.fk_world(); print("hand", np.round(p,4), np.round(q,4))
sol = r.ik_world(p, q); print("ik roundtrip:", np.round(sol,4) if sol else None, "current", np.round(r.arm_q(),4))
# test IK for a top-down pose above the knob
for quat in [(1,0,0,0), (0.7071,0.7071,0,0), (0.7071,-0.7071,0,0)]:
    sol = r.ik_world([-0.21, 0.194, 1.15], quat); print("knob-above", quat, np.round(sol,3) if sol else None)
    if sol: print("   fk check:", np.round(r.fk_world(sol)[0],4), np.round(r.fk_world(sol)[1],3))
EOF
timeout 200 python3 iktest.py

# openrua op 20
cat > /workspace/step_a.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("stepa")
r.gripper(0.04)
print("gap after open:", r.finger_gap())
q = r.move_pose([-0.21, 0.194, 1.15], (1,0,0,0), sec=4.0)
print("q:", np.round(q,4))
print("wrench:", r.wrench())
EOF
timeout 600 python3 step_a.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hand1.png

# openrua op 21
timeout 120 python3 heightmap.py robot0_eye_in_hand; timeout 60 python3 tfdump.py 2>&1 | grep eye_in_hand

# openrua op 22
cat > /workspace/analyze4.py <<'EOF'
import numpy as np, sys
cam = sys.argv[1]
P = np.load(f"/workspace/P_{cam}.npy"); Z = P[...,2]
reg = (P[...,0] > -0.3) & (P[...,0] < -0.12) & (P[...,1] > 0.1) & (P[...,1] < 0.3)
for lo, hi in [(0.905, 0.935), (0.935, 0.948), (0.948, 0.97)]:
    m = reg & (Z > lo) & (Z <= hi)
    if m.sum(): print(f"z({lo},{hi}] n={m.sum()} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] c=({P[m][:,0].mean():.4f},{P[m][:,1].mean():.4f}) mid=({(P[m][:,0].min()+P[m][:,0].max())/2:.4f},{(P[m][:,1].min()+P[m][:,1].max())/2:.4f})")
EOF
python3 analyze4.py robot0_eye_in_hand; python3 analyze4.py birdview

# openrua op 23
cat > /workspace/step_b.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("stepb")
f0, t0 = r.wrench(); print("wrench before:", np.round(f0,3))
r.move_pose([-0.2095, 0.1965, 1.09], (1,0,0,0), sec=3.0)
f, t = r.wrench(); print("wrench @1.09:", np.round(f,3))
r.move_pose([-0.2095, 0.1965, 1.065], (1,0,0,0), sec=2.0)
f, t = r.wrench(); print("wrench @1.065:", np.round(f,3))
r.move_pose([-0.2095, 0.1965, 1.058], (1,0,0,0), sec=2.0)
f, t = r.wrench(); print("wrench @1.058:", np.round(f,3))
print("q:", np.round(r.arm_q(),4))
EOF
timeout 600 python3 step_b.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hand2.png && timeout 60 python3 tools/perception/cam_snap.py sideview side2.png

# openrua op 24
python3 -c "
import cv2
im = cv2.imread('side2.png'); cv2.imwrite('crop_side2.png', cv2.resize(im[200:400, 300:480], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 25
cat > /workspace/step_c.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("stepc")
r.gripper(0.0)
r.spin(0.5)
print("gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3))
EOF
timeout 300 python3 step_c.py

# openrua op 26
cat > /workspace/step_d.py <<'EOF'
import numpy as np, sys
from lib import Robot
delta = float(sys.argv[1])  # radians to ADD to joint7
r = Robot("stepd")
q = r.arm_q(); print("q7 before:", round(q[6],4), "gap:", round(r.finger_gap(),4))
q[6] += delta
r.move_q(q, sec=max(1.5, abs(delta)*2))
print("q7 after:", round(r.arm_q()[6],4), "gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3), np.round(r.wrench()[1],3))
EOF
timeout 300 python3 step_d.py -0.5236 && timeout 60 python3 tools/perception/cam_snap.py agentview agent_k1.png

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py birdview bird_k1.png; python3 -c "
import cv2
im = cv2.imread('bird_k1.png'); cv2.imwrite('crop_bird_k1.png', cv2.resize(im[210:300, 330:420], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))"

# openrua op 28
timeout 300 python3 step_d.py -0.2077

# openrua op 29
timeout 300 python3 step_d.py -0.5236; timeout 300 python3 step_d.py -0.0 ; timeout 60 python3 tools/perception/cam_snap.py sideview side_k2.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent_k2.png

# openrua op 30
python3 -c "
import cv2
im = cv2.imread('side_k2.png'); cv2.imwrite('crop_side_k2.png', cv2.resize(im[200:400, 200:480], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('agent_k2.png'); cv2.imwrite('crop_agent_k2.png', cv2.resize(im[150:360, 300:520], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 31
timeout 300 python3 step_d.py -0.16; timeout 300 python3 step_d.py -0.0

# openrua op 32
cat > /workspace/step_e.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("stepe")
r.gripper(0.04)
p, quat, _ = r.fk_world(); print("hand at", np.round(p,4), np.round(quat,4))
r.move_pose(p + [0,0,0.12], quat, sec=2.5)
r.gripper(0.04)
EOF
timeout 300 python3 step_e.py && timeout 60 python3 tools/perception/cam_snap.py agentview agent_k3.png && python3 -c "
import cv2
im = cv2.imread('agent_k3.png'); cv2.imwrite('crop_agent_k3.png', cv2.resize(im[150:360, 300:520], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 33
cat > /workspace/step_f.py <<'EOF'
import numpy as np, sys
from lib import Robot
r = Robot("stepf")
x, y, z = map(float, sys.argv[1:4]); quat = tuple(map(float, sys.argv[4:8])) if len(sys.argv) >= 8 else (0.7071, 0.7071, 0, 0)
sec = float(sys.argv[8]) if len(sys.argv) > 8 else 4.0
q = r.move_pose([x, y, z], quat, sec=sec)
print("q:", np.round(q,4) if q is not None else None, "gap:", round(r.finger_gap(),4))
EOF
timeout 300 python3 step_f.py -0.033 -0.242 1.35 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hand3.png && timeout 120 python3 heightmap.py robot0_eye_in_hand

# openrua op 34
timeout 300 python3 step_f.py -0.033 -0.242 1.35 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hand3.png && timeout 120 python3 heightmap.py robot0_eye_in_hand

# openrua op 35
timeout 300 python3 step_f.py -0.033 -0.242 1.2 0.7071 0.7071 0 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hand4.png && timeout 120 python3 heightmap.py robot0_eye_in_hand && cat > /workspace/analyze5.py <<'EOF'
import numpy as np
P = np.load("/workspace/P_robot0_eye_in_hand.npy"); Z = P[...,2]
reg = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.35) & (P[...,1] < -0.15) & (Z > 0.905) & (Z < 1.1)
for lo in np.arange(0.95, 1.06, 0.01):
    m = reg & (Z > lo) & (Z <= lo+0.01)
    if m.sum(): print(f"z({lo:.2f},{lo+0.01:.2f}] n={m.sum()} x[{P[m][:,0].min():.4f},{P[m][:,0].max():.4f}] w={P[m][:,0].max()-P[m][:,0].min():.4f} y[{P[m][:,1].min():.4f},{P[m][:,1].max():.4f}] cx={(P[m][:,0].min()+P[m][:,0].max())/2:.4f}")
m = reg & (Z > 1.02)
print("top region x-extent per y-bin:")
for yb in np.arange(-0.30, -0.18, 0.005):
    s = m & (P[...,1] >= yb) & (P[...,1] < yb+0.005)
    if s.sum(): print(f"  y={yb:.3f}: x[{P[s][:,0].min():.4f},{P[s][:,0].max():.4f}] w={P[s][:,0].max()-P[s][:,0].min():.4f} zmax={Z[s].max():.3f}")
EOF
python3 analyze5.py

# openrua op 36
cat > /workspace/analyze6.py <<'EOF'
import numpy as np, cv2
P = np.load("/workspace/P_robot0_eye_in_hand.npy"); Z = P[...,2]
vv, uu = np.mgrid[0:480, 0:640]
reg = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.35) & (P[...,1] < -0.15) & (Z > 0.905) & (Z < 1.08) & (vv < 358)
for lo in np.arange(0.95, 1.06, 0.01):
    m = reg & (Z > lo) & (Z <= lo+0.01)
    if m.sum(): print(f"z({lo:.2f},{lo+0.01:.2f}] n={m.sum()} x[{P[m][:,0].min():.4f},{P[m][:,0].max():.4f}] w={P[m][:,0].max()-P[m][:,0].min():.4f} y[{P[m][:,1].min():.4f},{P[m][:,1].max():.4f}] cx={(P[m][:,0].min()+P[m][:,0].max())/2:.4f}")
m = reg & (Z > 1.02)
print("top region x-extent per y-bin:")
for yb in np.arange(-0.30, -0.18, 0.005):
    s = m & (P[...,1] >= yb) & (P[...,1] < yb+0.005)
    if s.sum(): print(f"  y={yb:.3f}: x[{P[s][:,0].min():.4f},{P[s][:,0].max():.4f}] w={P[s][:,0].max()-P[s][:,0].min():.4f} cx={(P[s][:,0].min()+P[s][:,0].max())/2:.4f} zmax={Z[s].max():.3f}")
im = cv2.imread("/workspace/hand4.png")
ys, xs = np.where(m); print("pixel bbox u[%d,%d] v[%d,%d]" % (xs.min(), xs.max(), ys.min(), ys.max()))
EOF
python3 analyze6.py

# openrua op 37
cat > /workspace/step_g.py <<'EOF'
import numpy as np, sys
from lib import Robot, quat_R
r = Robot("stepg")
for quat in [(0.5556, 0.8315, 0, 0), (0.8315, 0.5556, 0, 0)]:
    quat = np.array(quat)/np.linalg.norm(quat)
    sol = r.ik_world([-0.0296, -0.242, 1.2], quat)
    if sol is None: print(quat, "no IK"); continue
    p, q, _ = r.fk_world(sol); R = quat_R(q)
    print("quat", np.round(quat,4), "sol", np.round(sol,3), "finger axis (hand y) in world:", np.round(R[:,1],3), "hand z:", np.round(R[:,2],3))
EOF
timeout 300 python3 step_g.py

# openrua op 38
cat > /workspace/getdesc.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("gd")
qos = QoSProfile(depth=1); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
got = []
n.create_subscription(String, "/robot_description", got.append, qos)
while not got: rclpy.spin_once(n, timeout_sec=0.5)
open("/workspace/robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
timeout 60 python3 getdesc.py && grep -n -A12 'link name="panda_hand"' robot.urdf | head -40; grep -n -B2 -A14 'link name="panda_leftfinger"' robot.urdf | head -40; grep -n -A6 'joint name="panda_finger_joint1"' robot.urdf

# openrua op 39
timeout 300 python3 step_f.py -0.0296 -0.242 1.2 0.8315 0.5556 0 0 2.5 && timeout 120 python3 heightmap.py robot0_eye_in_hand >/dev/null && python3 analyze6.py | head -8

# openrua op 40
timeout 300 python3 step_f.py -0.0294 -0.237 1.153 0.8315 0.5556 0 0 2.0 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand hand5.png && timeout 60 python3 tools/perception/cam_snap.py frontview front5.png

# openrua op 41
python3 -c "
import cv2
im = cv2.imread('front5.png'); cv2.imwrite('crop_front5.png', cv2.resize(im[180:420, 100:320], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 42
cat > /workspace/step_h.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("steph")
quat = (0.8315, 0.5556, 0, 0)
f0 = r.wrench()[0]; print("wrench0", np.round(f0,3))
for z in [1.13, 1.103]:
    r.move_pose([-0.0294, -0.237, z], quat, sec=1.5)
    f = r.wrench()[0]; print(f"wrench @hand z={z}:", np.round(f,3), "delta", np.round(f-f0,3))
    if np.abs(f - f0).max() > 2.0:
        print("CONTACT detected, stopping descent"); break
EOF
timeout 300 python3 step_h.py && timeout 60 python3 tools/perception/cam_snap.py frontview front6.png && python3 -c "
import cv2
im = cv2.imread('front6.png'); cv2.imwrite('crop_front6.png', cv2.resize(im[180:420, 100:320], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 43
cat > /workspace/step_i.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("stepi")
quat = (0.8315, 0.5556, 0, 0)
r.move_pose([-0.0294, -0.237, 1.124], quat, sec=1.0)
print("wrench:", np.round(r.wrench()[0],3))
r.gripper(0.0)
r.spin(0.3)
print("gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3))
EOF
timeout 300 python3 step_i.py

# openrua op 44
cat > /workspace/step_j.py <<'EOF'
import numpy as np, sys
from lib import Robot
r = Robot("stepj")
dz = float(sys.argv[1])
p, quat, _ = r.fk_world()
r.move_pose(p + [0,0,dz], quat, sec=max(1.0, abs(dz)*8))
print("gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3))
EOF
timeout 300 python3 step_j.py 0.03 && timeout 60 python3 tools/perception/cam_snap.py frontview front7.png && python3 -c "
import cv2
im = cv2.imread('front7.png'); cv2.imwrite('crop_front7.png', cv2.resize(im[180:420, 100:320], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 45
timeout 300 python3 step_j.py 0.10 && timeout 60 python3 tools/perception/cam_snap.py sideview side8.png && timeout 60 python3 tools/perception/cam_snap.py agentview agent8.png

# openrua op 46
cat > /workspace/step_k.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("stepk")
quat = (0.8315, 0.5556, 0, 0)
def go(pos, sec):
    for attempt in range(3):
        q = r.ik_world(pos, quat)
        code, err = r.move_q(q, sec)
        if err < 0.01: break
        print("  retrying (lag)")
    p, _, _ = r.fk_world(); print("  at", np.round(p,4), "gap", round(r.finger_gap(),4), "wrench", np.round(r.wrench()[0],2))
go([-0.0294, -0.237, 1.32], 2.0)
go([-0.058, 0.20, 1.32], 4.0)
go([-0.058, 0.20, 1.20], 2.0)
go([-0.058, 0.20, 1.165], 1.5)
EOF
timeout 600 python3 step_k.py && timeout 60 python3 tools/perception/cam_snap.py agentview agent9.png && timeout 60 python3 tools/perception/cam_snap.py sideview side9.png

# openrua op 47
cat > /workspace/step_l.py <<'EOF'
import numpy as np
from lib import Robot
r = Robot("stepl")
quat = (0.8315, 0.5556, 0, 0)
f0 = r.wrench()[0]
for z in [1.157, 1.152, 1.147, 1.142]:
    q = r.ik_world([-0.058, 0.20, z], quat); code, err = r.move_q(q, 1.0)
    p, _, _ = r.fk_world(); f = r.wrench()[0]
    print(f"  target z={z} at {np.round(p,4)} err={err:.4f} wrench {np.round(f,2)} dz={f[2]-f0[2]:.2f}")
    if f[2] - f0[2] > 1.5 or err > 0.005:
        print("contact -> stop"); break
r.gripper(0.04)
print("gap after open:", round(r.finger_gap(),4))
p, _, _ = r.fk_world()
q = r.ik_world(p + [0,0,0.12], quat); r.move_q(q, 2.0)
print("retreated to", np.round(r.fk_world()[0],4))
EOF
timeout 600 python3 step_l.py && timeout 60 python3 tools/perception/cam_snap.py agentview agent10.png && timeout 60 python3 tools/perception/cam_snap.py sideview side10.png && timeout 60 python3 tools/perception/cam_snap.py birdview bird10.png

# openrua op 48
timeout 120 python3 heightmap.py birdview && python3 - <<'EOF'
import numpy as np
P = np.load("/workspace/P_birdview.npy"); Z = P[...,2]
reg = (P[...,0] > -0.16) & (P[...,0] < 0.05) & (P[...,1] > 0.09) & (P[...,1] < 0.30)
for lo, hi in [(0.905, 0.94), (0.94, 1.05), (1.05, 1.075), (1.075, 1.1)]:
    m = reg & (Z > lo) & (Z <= hi)
    if m.sum(): print(f"z({lo},{hi}] n={m.sum()} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] center=({(P[m][:,0].min()+P[m][:,0].max())/2:.3f},{(P[m][:,1].min()+P[m][:,1].max())/2:.3f}) zmax={Z[m].max():.3f}")
EOF

# openrua op 49
timeout 300 python3 tools/action/fjt_send.py 0,-0.161,0,-2.4446,0,2.2268,0.7854 4 && timeout 300 python3 tools/action/fjt_send.py 0,-0.161,0,-2.4446,0,2.2268,0.7854 2 && timeout 120 python3 heightmap.py birdview && python3 - <<'EOF'
import numpy as np
P = np.load("/workspace/P_birdview.npy"); Z = P[...,2]
reg = (P[...,0] > -0.16) & (P[...,0] < 0.05) & (P[...,1] > 0.09) & (P[...,1] < 0.30)
for lo, hi in [(0.905, 0.94), (0.94, 1.05), (1.05, 1.075), (1.075, 1.1)]:
    m = reg & (Z > lo) & (Z <= hi)
    if m.sum(): print(f"z({lo},{hi}] n={m.sum()} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] center=({(P[m][:,0].min()+P[m][:,0].max())/2:.3f},{(P[m][:,1].min()+P[m][:,1].max())/2:.3f}) zmax={Z[m].max():.3f}")
EOF
timeout 60 python3 tools/perception/cam_snap.py birdview bird11.png; timeout 60 python3 tools/perception/cam_snap.py frontview front11.png
