#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace; find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 6
mkdir -p "$(dirname /workspace/tfdump.py)"
cat > /workspace/tfdump.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump all TF frames and a few useful transforms."""
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer()
TransformListener(buf, node)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_yaml())
for a, b in [("world", "panda_link0"), ("world", "panda_hand"), ("panda_link0", "panda_hand"),
             ("world", "birdview_optical_frame"), ("world", "agentview_optical_frame"),
             ("world", "robot0_eye_in_hand_optical_frame")]:
    try:
        t = buf.lookup_transform(a, b, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{a}->{b}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"{a}->{b}: FAIL {e}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 7
timeout 60 python3 tfdump.py 2>&1 | grep -vE "^\s+(broadcaster|rate|most_recent|oldest|buffer)" 

# openrua op 8
cd /workspace; for p in "250 285" "393 285" "283 265" "345 278" "315 250" "320 285"; do echo -n "birdview $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done; for p in "90 295" "560 295" "210 240" "385 250" "318 190"; do echo -n "agentview $p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p; done

# openrua op 9
mkdir -p "$(dirname /workspace/segment.py)"
cat > /workspace/segment.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment tabletop objects from a top-down camera: cluster world points
with z in a height band, print each cluster's centroid, extent, top z,
and mean color. Usage: python3 segment.py [cam=birdview] [zmin] [zmax]
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.50
zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.66


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
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


rclpy.init()
node = rclpy.create_node("segment")
buf = Buffer()
TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, Time())
q, tr = t.transform.rotation, t.transform.translation
R = quat_R(q.x, q.y, q.z, q.w)
T = np.array([tr.x, tr.y, tr.z])

fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
z = depth.astype(np.float64)
pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1)
pw = pc @ R.T + T
mask = np.isfinite(z) & (pw[..., 2] > zmin) & (pw[..., 2] < zmax)

# connected components on the mask
import cv2
n, lab = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
for i in range(1, n):
    m = lab == i
    if m.sum() < 30:
        continue
    P = pw[m]
    C = color[m].reshape(-1, 3).mean(0)
    us, vs = uu[m], vv[m]
    print(f"cluster {i}: n={m.sum()} centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
          f"x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] "
          f"ztop={P[:,2].max():.3f} px=({us.mean():.0f},{vs.mean():.0f}) rgb={C.astype(int)}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 10
cd /workspace; timeout 90 python3 segment.py birdview 0.50 0.66

# openrua op 11
cd /workspace; timeout 90 python3 segment.py birdview 0.565 0.66

# openrua op 12
cd /workspace; timeout 90 python3 segment.py robot0_eye_in_hand 0.50 0.66; echo; timeout 90 python3 segment.py robot0_eye_in_hand 0.57 0.66

# openrua op 13
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helper for this Panda: joint state, TF, FK/IK, trajectory,
gripper, servo. Build once per process and reuse."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world->panda_link0 (from TF)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(yaw):
    """Hand z pointing down (world -z), hand x rotated by yaw about world z.
    q = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    # (c,0,0,s) * (1,0,0,0) quaternion product (w,x,y,z) convention:
    # q1=(w=c,x=0,y=0,z=s), q2=(w=0,x=1,y=0,z=0)
    w = c * 0 - 0 * 1 - 0 * 0 - s * 0
    x = c * 1 + 0 * 0 + 0 * 0 - s * 0
    y = c * 0 - 0 * 0 + 0 * 1 + s * 1
    z = c * 0 + 0 * 0 - 0 * 1 + s * 0
    return (x, y, z, w)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, next(
            s for s in M["sensors"] if s["kind"] == "wrench")["port"], self._on_w, 1)
        self.tf = Buffer()
        TransformListener(self.tf, self.node)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self.js = m

    def _on_w(self, m):
        self.wrench = m

    def spin(self, n=3, dt=0.05):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=dt)

    # ---- sensing ----
    def joints(self):
        self.spin(3)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.spin(3)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose_world(self):
        """Hand pose in world from FK (fresh, not cached TF)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        seed.name = list(ARM)
        seed.position = self.joints()
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self):
        pos, q = self.hand_pose_world()
        R = quat_R(*q)
        return pos + M["hand"]["tcp_offset_m"] * R[:, 2], q

    def force(self):
        self.spin(3)
        f = self.wrench.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- IK ----
    def ik_solve(self, pos_world, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        s = JointState()
        s.name = list(ARM)
        s.position = list(seed if seed is not None else self.joints())
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_world, quat, seed=None):
        R = quat_R(*quat)
        hand = np.asarray(tcp_world) - M["hand"]["tcp_offset_m"] * R[:, 2]
        return self.ik_solve(hand, quat, seed)

    # ---- action ----
    def move_joints(self, positions, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [positions]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(positions)).max()
        return code, err

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        sol = self.ik_tcp(tcp_world, quat, seed)
        if sol is None:
            raise RuntimeError(f"IK failed for tcp {tcp_world}")
        code, err = self.move_joints(sol, seconds)
        pos, _ = self.tcp_world()
        return code, err, pos

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
OPENRUA_EOF

# openrua op 14
cd /workspace; cat > t1.py <<'EOF'
import numpy as np
from rob import *
r = Robot("t1")
print("joints", np.round(r.joints(), 4))
print("fingers", r.fingers())
pos, q = r.hand_pose_world()
print("FK hand world", np.round(pos, 4), np.round(q, 4))
print("TCP world", np.round(r.tcp_world()[0], 4))
print("force", r.force())
# IK for the current hand pose: does it return ~current joints?
sol = r.ik_solve(pos, q)
print("IK(current hand pose) ->", None if sol is None else np.round(sol, 4))
# IK for a pure down orientation at the same spot
sol2 = r.ik_solve(pos, (1, 0, 0, 0))
print("IK(q=1000) ->", None if sol2 is None else np.round(sol2, 4))
sol3 = r.ik_solve(pos, yaw_down_quat(math.pi / 2))
print("IK(yaw90) ->", None if sol3 is None else np.round(sol3, 4))
EOF
timeout 120 python3 t1.py

# openrua op 15
cd /workspace; cat > t2.py <<'EOF'
import numpy as np
from rob import *
from moveit_msgs.srv import GetPositionFK
r = Robot("t2")
req = GetPositionFK.Request(); req.header.frame_id = ""
req.fk_link_names = ["panda_link0", "panda_link8", "panda_hand"]
s = JointState(); s.name = list(ARM); s.position = r.joints()
req.robot_state.joint_state = s
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res = fut.result()
poses = {}
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p, o = ps.pose.position, ps.pose.orientation
    poses[n] = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
    print(n, ps.header.frame_id, np.round([p.x, p.y, p.z], 4), np.round([o.x, o.y, o.z, o.w], 4))
# IK with link8 pose, raw (no base offset)
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"; req.ik_request.pose_stamped.header.frame_id = ""
(px, py, pz), (qx, qy, qz, qw) = poses["panda_link8"]
pp = req.ik_request.pose_stamped.pose
pp.position.x, pp.position.y, pp.position.z = px, py, pz
pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
req.ik_request.robot_state.joint_state = s
fut = r.ik.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
res = fut.result(); print("err", res.error_code.val)
sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
print("IK(link8 pose) ->", np.round([sol[j] for j in ARM], 4))
print("current        ->", np.round(r.joints(), 4))
EOF
timeout 120 python3 t2.py

# openrua op 16
cd /workspace; python3 - <<'EOF'
import re
s = open("rob.py").read()
s = s.replace('BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world->panda_link0 (from TF)',
 'BASE_IN_WORLD = np.zeros(3)  # verified: FK/IK with empty frame_id are in WORLD (model root)\nfrom scipy.spatial.transform import Rotation as Rot\n# IK tip link is panda_link8 = hand rotated -45deg about z (verified against FK)\nR_HAND2LINK8 = Rot.from_euler("z", math.pi / 4)')
s = s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False''',
'''        q8 = (Rot.from_quat(list(quat)) * R_HAND2LINK8).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        req.ik_request.avoid_collisions = False''')
open("rob.py","w").write(s)
EOF
grep -n "R_HAND2LINK8\|BASE_IN_WORLD" rob.py
cat > t3.py <<'EOF'
import numpy as np
from rob import *
r = Robot("t3")
pos, q = r.hand_pose_world()
print("hand", np.round(pos,4), np.round(q,4), "tcp", np.round(r.tcp_world()[0],4))
print("IK(current hand) ->", np.round(r.ik_solve(pos, q), 4))
print("current          ->", np.round(r.joints(), 4))
for yaw in (0, math.pi/2):
    print("yaw", yaw, "quat", np.round(yaw_down_quat(yaw),4), "R", np.round(quat_R(*yaw_down_quat(yaw)),3).tolist())
EOF
timeout 120 python3 t3.py

# openrua op 17
cd /workspace; cat > s1.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s1")
print("open:", r.gripper(0.04))
Q = yaw_down_quat(0.0)
code, err, pos = r.move_tcp((-0.079, -0.196, 0.66), Q, 3.0)
print("hover: code", code, "jerr", round(err,4), "tcp", np.round(pos,4))
print("force", np.round(r.force(),3))
EOF
timeout 300 python3 s1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 18
mkdir -p "$(dirname /workspace/rimfit.py)"
cat > /workspace/rimfit.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Fit a circle (in world XY) to the rim points of each mug seen by a camera.
Usage: python3 rimfit.py <cam> [zmin] [zmax] — rim points = z within band.
Prints centre, radius, top z for each cluster."""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from rob import quat_R

cam = sys.argv[1] if len(sys.argv) > 1 else "robot0_eye_in_hand"
zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.56
zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.63


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def fit_circle(x, y):
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    b = x * x + y * y
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    r = np.sqrt(c[2] + c[0] ** 2 + c[1] ** 2)
    return c[0], c[1], r


rclpy.init()
node = rclpy.create_node("rimfit")
buf = Buffer()
TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, Time())
q, tr = t.transform.rotation, t.transform.translation
R = quat_R(q.x, q.y, q.z, q.w)
T = np.array([tr.x, tr.y, tr.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
z = depth.astype(np.float64)
pw = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1) @ R.T + T
mask = np.isfinite(z) & (pw[..., 2] > zmin) & (pw[..., 2] < zmax)
n, lab = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
for i in range(1, n):
    m = lab == i
    if m.sum() < 200:
        continue
    P = pw[m]
    # use only the topmost 1.5 cm of the cluster (the rim ring itself)
    top = P[:, 2].max()
    ring = P[P[:, 2] > top - 0.015]
    cxw, cyw, rad = fit_circle(ring[:, 0], ring[:, 1])
    print(f"cluster {i}: n={m.sum()} ring_n={len(ring)} rim_center=({cxw:.4f},{cyw:.4f}) "
          f"radius={rad:.4f} ztop={top:.4f} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] "
          f"y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 19
cd /workspace; timeout 90 python3 rimfit.py robot0_eye_in_hand 0.56 0.63

# openrua op 20
cd /workspace; timeout 90 python3 rimfit.py robot0_eye_in_hand 0.50 0.66; python3 -c "
import numpy as np" ; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png; python3 -c "
import numpy as np; d=np.load('eih_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d), d[240,320], d[300,260])"

# openrua op 21
cd /workspace; timeout 60 python3 tfdump.py 2>&1 | grep -- "->"

# openrua op 22
cd /workspace; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:"; python3 -c "
import numpy as np; d=np.load('eih_depth.npy'); 
print('center', d[240,320]); print('mug px (260,310)', d[310,260]); print('plate px (420,210)', d[210,420]); print('corner', d[20,20], d[20,620])"

# openrua op 23
cd /workspace; cat > s2.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s2")
j = r.joints()
print(r.move_joints(j, 0.5))
print("tcp", np.round(r.tcp_world()[0],4))
EOF
timeout 200 python3 s2.py; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png; python3 -c "
import numpy as np; d=np.load('eih_depth.npy'); print('center', d[240,320])"; timeout 90 python3 rimfit.py robot0_eye_in_hand 0.50 0.66

# openrua op 24
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py sideview; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tfdump.py 2>&1 | grep sideview

# openrua op 25
cd /workspace; for p in "250 285" "393 285" "150 285"; do echo -n "birdview $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done; timeout 90 python3 rimfit.py birdview 0.45 0.62

# openrua op 26
cd /workspace; cat > s3.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s3")
Q = yaw_down_quat(0.0)
gx, gy = -0.0813, -0.197
print("f0", np.round(r.force(),3))
print("pre ", r.move_tcp((gx, gy, 0.60), Q, 2.0))
print("down", r.move_tcp((gx, gy, 0.519), Q, 2.0))
print("f1", np.round(r.force(),3))
print("close", r.gripper(0.0))
print("f2", np.round(r.force(),3))
print("lift", r.move_tcp((gx, gy, 0.65), Q, 2.0))
print("fingers after lift", r.fingers(), "f3", np.round(r.force(),3))
EOF
timeout 600 python3 s3.py; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 27
cd /workspace; cat > s4.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s4")
Q = yaw_down_quat(0.0)
px, py = -0.0015, 0.3221          # right plate centre
tx, ty = px, py - 0.041           # TCP so that mug centre sits over plate centre
print("up  ", r.move_tcp((-0.0812, -0.197, 0.72), Q, 2.0), r.fingers())
print("mid ", r.move_tcp((-0.05, 0.05, 0.72), Q, 3.0), r.fingers())
print("over", r.move_tcp((tx, ty, 0.70), Q, 3.0), r.fingers())
print("down", r.move_tcp((tx, ty, 0.548), Q, 2.5), r.fingers())
print("f", np.round(r.force(),3))
print("open", r.gripper(0.04))
print("lift", r.move_tcp((tx, ty, 0.68), Q, 2.0))
EOF
timeout 900 python3 s4.py; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 28
cd /workspace; cat > s5.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s5")
Q = yaw_down_quat(0.0)
print("hover", r.move_tcp((-0.035, 0.14, 0.66), Q, 3.0), r.fingers())
EOF
timeout 600 python3 s5.py; timeout 90 python3 rimfit.py robot0_eye_in_hand 0.45 0.62; timeout 90 python3 rimfit.py birdview 0.45 0.62

# openrua op 29
cd /workspace; cat > s6.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s6")
Q = yaw_down_quat(0.0)
gx, gy = -0.0315, 0.139
print("pre ", r.move_tcp((gx, gy, 0.60), Q, 2.0))
print("down", r.move_tcp((gx, gy, 0.512), Q, 2.0))
print("f1", np.round(r.force(),3))
print("close", r.gripper(0.0))
print("lift", r.move_tcp((gx, gy, 0.72), Q, 2.5))
print("fingers after lift", r.fingers(), "f", np.round(r.force(),3))
EOF
timeout 600 python3 s6.py; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
cd /workspace; cat > s7.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s7")
Q = yaw_down_quat(0.0)
px, py = -0.0014, -0.3089         # left plate centre
tx, ty = px, py + 0.044           # TCP so mug centre sits over plate centre
print("mid ", r.move_tcp((-0.05, -0.05, 0.72), Q, 3.0), r.fingers())
print("over", r.move_tcp((tx, ty, 0.70), Q, 3.0), r.fingers())
f0 = r.force()
print("down1", r.move_tcp((tx, ty, 0.545), Q, 2.5), r.fingers(), "df", np.round(r.force()-f0,3))
print("down2", r.move_tcp((tx, ty, 0.530), Q, 1.5), r.fingers(), "df", np.round(r.force()-f0,3))
print("open", r.gripper(0.04))
print("lift", r.move_tcp((tx, ty, 0.68), Q, 2.0))
EOF
timeout 900 python3 s7.py; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 31
cd /workspace; cat > s8.py <<'EOF'
import numpy as np
from rob import *
r = Robot("s8")
home = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
print("home", r.move_joints(home, 3.0))
print("tcp", np.round(r.tcp_world()[0],4))
EOF
timeout 600 python3 s8.py; timeout 90 python3 rimfit.py birdview 0.45 0.62; for p in "250 285" "393 285"; do echo -n "plate px $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done; timeout 60 python3 tools/perception/cam_snap.py birdview; timeout 60 python3 tools/perception/cam_snap.py frontview
