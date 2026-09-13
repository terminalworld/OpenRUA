#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | head -40; echo ---; ros2 node list 2>&1

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ---; timeout 20 ros2 topic echo /birdview/color/camera_info --once 2>&1 | grep -A12 "^k:"

# openrua op 6
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
def cb(m, k):
    for t in m.transforms:
        got[(k, t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m, "static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m, "dyn"), 100)
import time
end = time.time() + 5
while time.time() < end: rclpy.spin_once(node, timeout_sec=0.2)
for (k, p, c), t in sorted(got.items()):
    tr, r = t.translation, t.rotation
    print(f"{k:6s} {p:28s} -> {c:28s} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({r.x:.4f},{r.y:.4f},{r.z:.4f},{r.w:.4f})")
EOF
timeout 60 python3 /workspace/tfdump.py

# openrua op 7
cat > /workspace/cloud.py <<'EOF'
"""World-frame point cloud from a camera's depth + color; print blobs above table."""
import sys, struct, time
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf", tfcb, 100)
end = time.time() + 20
while len(got) < 4 and time.time() < end: rclpy.spin_once(node, timeout_sec=0.2)
assert len(got) == 4, got.keys()
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = got["tf"]; q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.save(f"snaps/{cam}_world.npy", pw)
np.save(f"snaps/{cam}_color.npy", color)
print("depth range", np.nanmin(depth), np.nanmax(depth))
zs = pw[..., 2]
# table height = mode of z in central region
hist, edges = np.histogram(zs[np.isfinite(zs)], bins=400)
table_z = edges[np.argmax(hist)]
print("dominant z (table?)", table_z)
if len(sys.argv) > 2:
    lo = float(sys.argv[2])
    mask = (zs > lo).astype(np.uint8)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15: continue
        m = lab == i
        p = pw[m]
        print(f"blob {i}: px centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) area={stats[i,4]} "
              f"x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] "
              f"z[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) "
              f"color={color[m].mean(0).astype(int)}")
EOF
timeout 90 python3 /workspace/cloud.py birdview

# openrua op 8
python3 -c "
import numpy as np
pw=np.load('snaps/birdview_world.npy'); zs=pw[...,2]
z=zs[np.isfinite(zs)]
h,e=np.histogram(z,bins=np.arange(0,1.2,0.01))
for c,lo in zip(h,e[:-1]):
    if c>50: print(f'{lo:.2f} {c}')
"

# openrua op 9
timeout 90 python3 /workspace/cloud.py birdview 0.44

# openrua op 10
timeout 90 python3 /workspace/cloud.py agentview 0.445 2>&1 | grep -v "^depth"

# openrua op 11
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Reusable robot helpers: one node, clients built once (see docs/40 P1)."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from /tf world->panda_link0
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        """Fresh joint dict (arm + fingers)."""
        self._js = None
        end = time.time() + 10
        while self._js is None and time.time() < end:
            self.spin(0.2)
        assert self._js is not None, "no /joint_states"
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- kinematics (base frame = panda_link0, frame_id left empty) ----
    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        js = JointState()
        js.name = list(ARM)
        js.position = list(q if q is not None else self.arm_q())
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id)

    def ik_hand(self, pos_base, quat, seed=None):
        """IK for the HAND frame at pos (base frame). Returns joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        js = JointState()
        js.name = list(ARM)
        js.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp_world(self, pos_world, quat, seed=None):
        """IK with the fingertip-centre (TCP) at a WORLD position."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(pos_world, float) - TCP * R[:, 2]
        return self.ik_hand(hand_world - BASE_IN_WORLD, quat, seed)

    def tcp_world(self):
        p, q, _ = self.fk_hand()
        R = quat_to_R(*q)
        return p + BASE_IN_WORLD + TCP * R[:, 2], q

    # ---- motion ----
    def move_q(self, q, seconds=3.0, tol=0.02, retries=2):
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  move_q: error_code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return False

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, v, ticks=20):
        """Stream a base-frame linear velocity (m/s) for `ticks` messages."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
OPENRUA_EOF

# openrua op 12
timeout 120 python3 -c "
from rob import *
r = Robot()
p,q,f = r.fk_hand()
print('FK hand pos', p, 'quat', q, 'frame', f)
print('TCP world', r.tcp_world()[0])
print('R hand', quat_to_R(*q).round(3))
print('gap', r.finger_gap())
print('wrench', r.wrench())
"

# openrua op 13
timeout 60 python3 - <<'EOF'
import numpy as np, rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from rob import quat_to_R
rclpy.init(); node = rclpy.create_node("chain")
tf = {}
def cb(m):
    for t in m.transforms: tf[t.child_frame_id] = (t.header.frame_id, t.transform)
qos = QoSProfile(depth=100); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
end=time.time()+4
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
def T(tr):
    q=tr.rotation; M=np.eye(4); M[:3,:3]=quat_to_R(q.x,q.y,q.z,q.w); M[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]; return M
def pose(frame):
    M=np.eye(4)
    while frame in tf:
        parent,tr=tf[frame]; M=T(tr)@M; frame=parent
    return frame, M
root,M=pose("panda_hand"); print("root",root); print(M.round(4))
root,M=pose("panda_leftfinger"); print("leftfinger", M[:3,3].round(4))
EOF

# openrua op 14
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from /tf world->panda_link0", "new_string": "# Verified: /compute_fk with empty frame_id returns poses whose frame_id is\\n# \\"world\\" and which match the TF chain world->panda_hand, so the planner\'s\\n# model frame IS world on this machine (base sits at world (-0.51,0,0.42)).\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 15
timeout 200 python3 -c "
from rob import *
r = Robot()
q0 = r.arm_q(); print('q0', np.round(q0,3))
p,q,_ = r.fk_hand()
sol = r.ik_hand(p, q); print('IK roundtrip', None if sol is None else np.round(sol,3))
# pre-grasp above soup: TCP at world (-0.217,-0.135, 0.60), top-down
down = [1,0,0,0]
for z in (0.60, 0.46):
    s = r.ik_tcp_world([-0.217,-0.135,z], down, seed=q0)
    print('soup z',z, None if s is None else np.round(s,3))
    if s is not None:
        p2,q2,_ = r.fk_hand(s); print('   fk check hand', p2.round(3), 'tcp', (p2 + TCP*quat_to_R(*q2)[:,2]).round(3))
"

# openrua op 16
timeout 200 python3 -c "
from rob import *
r = Robot()
print('gap before', r.finger_gap())
r.gripper(0.04)
print('joints', {k:round(v,4) for k,v in r.joints().items() if 'finger' in k})
"

# openrua op 17
mkdir -p "$(dirname /workspace/pick_soup.py)"
cat > /workspace/pick_soup.py <<'OPENRUA_EOF'
"""Pick the alphabet soup can: pre-grasp above, descend, close, lift."""
import sys
from rob import *

SOUP = np.array([-0.217, -0.135])
DOWN = [1.0, 0.0, 0.0, 0.0]  # hand z down, fingers close along world y
r = Robot("pick_soup")
q0 = r.arm_q()

print("== pre-grasp above soup")
q_pre = r.ik_tcp_world([*SOUP, 0.62], DOWN, seed=q0)
assert q_pre is not None
assert r.move_q(q_pre, 4.0)
print("  tcp now", r.tcp_world()[0].round(3))

print("== descend to grasp height")
q_g = r.ik_tcp_world([*SOUP, 0.462], DOWN, seed=q_pre)
assert q_g is not None
assert r.move_q(q_g, 3.0)
print("  tcp now", r.tcp_world()[0].round(3), "wrench", r.wrench())

print("== close")
gap = r.gripper(0.0)
print("  gap after close", gap)

print("== lift")
q_up = r.ik_tcp_world([*SOUP, 0.72], DOWN, seed=q_g)
assert q_up is not None
assert r.move_q(q_up, 3.0)
print("  tcp now", r.tcp_world()[0].round(3), "gap", r.finger_gap())
OPENRUA_EOF

# openrua op 18
timeout 900 python3 -u pick_soup.py 2>&1 | tee pick_soup.log

# openrua op 19
timeout 200 python3 -c "
from rob import *
r = Robot()
q = r.arm_q(); print('q now', np.round(q,3))
p,_ = r.tcp_world(); print('tcp', p.round(3))
print('wrench', r.wrench())
DOWN=[1,0,0,0]
qg = r.ik_tcp_world([-0.217,-0.135,0.462], DOWN, seed=q); print('q_g', np.round(qg,3)); print('diff', np.round(np.array(qg)-np.array(q),3))
" ; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent2.png

# openrua op 20
timeout 600 python3 -c "
from rob import *
r = Robot()
q = r.arm_q()
qp = r.ik_tcp_world([-0.217,-0.135,0.62], [1,0,0,0], seed=q)
print(r.move_q(qp, 3.0)); print('tcp', r.tcp_world()[0].round(3), 'wrench', r.wrench())
" && timeout 90 python3 cloud.py robot0_eye_in_hand 0.45 | grep -v depth && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih3.png

# openrua op 21
python3 - <<'EOF'
import numpy as np
pw=np.load('snaps/robot0_eye_in_hand_world.npy'); zs=pw[...,2]
# can top surface: z>0.50 and in the region near (-0.21,-0.13)
m=(zs>0.505)&(np.abs(pw[...,0]+0.21)<0.06)&(np.abs(pw[...,1]+0.13)<0.06)
p=pw[m]; print('can top n',m.sum(),'x',p[:,0].min().round(4),p[:,0].max().round(4),'y',p[:,1].min().round(4),p[:,1].max().round(4),'center',p[:,:2].mean(0).round(4), 'zmax', p[:,2].max().round(4))
# fingers: z>0.55 in lower image rows
m2=(zs>0.60)&(zs<0.75)
p2=pw[m2]; print('finger n',m2.sum(),'x',p2[:,0].min().round(3),p2[:,0].max().round(3),'y',p2[:,1].min().round(3),p2[:,1].max().round(3))
# per-finger: split by y
for name,mm in (('left(y<-0.135)',p2[:,1]<-0.135),('right',p2[:,1]>=-0.135)):
    q=p2[mm]; print(name,'y range',q[:,1].min().round(4),q[:,1].max().round(4),'x range',q[:,0].min().round(3),q[:,0].max().round(3))
EOF

# openrua op 22
python3 - <<'EOF'
import numpy as np, cv2
pw=np.load('snaps/robot0_eye_in_hand_world.npy'); zs=pw[...,2]
reg=np.zeros_like(zs,bool); reg[240:370,250:400]=True
m=reg&(zs>0.50)
p=pw[m]; print('can top n',m.sum(),'x',p[:,0].min().round(4),p[:,0].max().round(4),'y',p[:,1].min().round(4),p[:,1].max().round(4),'center',p[:,:2].mean(0).round(4),'zmax',p[:,2].max().round(4))
vv,uu=np.nonzero(m); print('px center',uu.mean().round(1),vv.mean().round(1),'px extent',uu.min(),uu.max(),vv.min(),vv.max())
# fingers region
for name,(r0,r1,c0,c1) in {'leftfinger':(355,400,120,210),'rightfinger':(355,400,430,520)}.items():
    reg=np.zeros_like(zs,bool); reg[r0:r1,c0:c1]=True
    m=reg&(zs>0.55)
    p=pw[m]; print(name,'n',m.sum(),'x',p[:,0].min().round(4),p[:,0].max().round(4),'y',p[:,1].min().round(4),p[:,1].max().round(4),'z',p[:,2].min().round(3),p[:,2].max().round(3))
d=np.load('snaps/robot0_eye_in_hand_world.npy')
EOF

# openrua op 23
timeout 120 python3 - <<'EOF'
import numpy as np, rclpy, time
from rob import *
r = Robot()
p,q,_ = r.fk_hand(); print('FK hand', p.round(3), 'quat', q.round(4)); print(quat_to_R(*q).round(3))
from tf2_msgs.msg import TFMessage
got={}
def cb(m):
    for t in m.transforms:
        if t.child_frame_id in ('robot0_eye_in_hand_optical_frame',): got[t.child_frame_id]=t.transform
r.node.create_subscription(TFMessage,'/tf',cb,100)
end=time.time()+3
while time.time()<end: r.spin(0.2)
t=got['robot0_eye_in_hand_optical_frame']; qq=t.rotation
print('cam pos', (t.translation.x,t.translation.y,t.translation.z)); print('cam R', quat_to_R(qq.x,qq.y,qq.z,qq.w).round(3))
print('home cam R', quat_to_R(0.7070,-0.7066,-0.0201,0.0201).round(3))
EOF

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def ik_tcp_world(self, pos_world, quat, seed=None):\\n        \\"\\"\\"IK with the fingertip-centre (TCP) at a WORLD position.\\"\\"\\"\\n        R = quat_to_R(*quat)\\n        hand_world = np.asarray(pos_world, float) - TCP * R[:, 2]\\n        return self.ik_hand(hand_world - BASE_IN_WORLD, quat, seed)", "new_string": "    def ik_tcp_world(self, pos_world, quat, seed=None):\\n        \\"\\"\\"IK with the fingertip-centre (TCP) at a WORLD position.\\"\\"\\"\\n        R = quat_to_R(*quat)\\n        hand_world = np.asarray(pos_world, float) - TCP * R[:, 2]\\n        return self.ik_hand(hand_world - BASE_IN_WORLD, quat, seed)\\n\\n    @staticmethod\\n    def yaw_of(q):\\n        \\"\\"\\"Yaw of the hand x axis in world (deg); 0 = fingers along world y.\\"\\"\\"\\n        R = quat_to_R(*q)\\n        return np.degrees(np.arctan2(R[1, 0], R[0, 0])), np.degrees(np.arccos(-R[2, 2]))\\n\\n    def ik_down(self, pos_world, yaw_deg=0.0, seed=None, verbose=True):\\n        \\"\\"\\"Top-down TCP pose at a world position with a given finger yaw.\\n\\n        Verified machine fact: /compute_ik is POSITION-ONLY (the returned\\n        orientation ignores the request), so orientation is fixed here:\\n        joint 7 spins the hand about its (vertical) z axis, i.e. sets yaw.\\n        Returns joints or None; checks tilt and TCP error via FK.\\n        \\"\\"\\"\\n        pos_world = np.asarray(pos_world, float)\\n        quat = [1.0, 0.0, 0.0, 0.0]\\n        q = list(seed if seed is not None else self.arm_q())\\n        for it in range(4):\\n            sol = self.ik_tcp_world(pos_world, quat, seed=q)\\n            if sol is None:\\n                return None\\n            p, qh, _ = self.fk_hand(sol)\\n            yaw, tilt = self.yaw_of(qh)\\n            # joint7 positive = clockwise seen from above (hand z points down)\\n            d = (yaw_deg - yaw + 180) % 360 - 180\\n            sol[6] -= np.radians(d)\\n            lo, hi = FJT[\\"limits_rad\\"][6]\\n            while sol[6] > hi: sol[6] -= 2 * np.pi\\n            while sol[6] < lo: sol[6] += 2 * np.pi\\n            p, qh, _ = self.fk_hand(sol)\\n            yaw, tilt = self.yaw_of(qh)\\n            tcp = p + TCP * quat_to_R(*qh)[:, 2]\\n            err = np.linalg.norm(tcp - pos_world)\\n            if verbose:\\n                print(f\\"  ik_down it{it}: yaw={yaw:.1f} tilt={tilt:.1f} tcp_err={err*1000:.1f}mm\\")\\n            if tilt < 2.0 and abs((yaw - yaw_deg + 180) % 360 - 180) < 1.0 and err < 0.003:\\n                return sol\\n            q = sol\\n        print(\\"  ik_down: did not converge\\")\\n        return None\\n\\n    def move_line(self, p1, yaw_deg=0.0, seconds=3.0, step=0.02, tol=0.02):\\n        \\"\\"\\"Straight-line TCP move from the current pose to world p1 via a\\n        multi-waypoint trajectory (joint-space interpolation alone bends\\n        the TCP path in XY by centimetres).\\"\\"\\"\\n        p0, _ = self.tcp_world()\\n        p1 = np.asarray(p1, float)\\n        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1)\\n        q = self.arm_q()\\n        qs = []\\n        for i in range(1, n):\\n            p = p0 + (p1 - p0) * i / (n - 1)\\n            q = self.ik_down(p, yaw_deg, seed=q, verbose=False)\\n            if q is None:\\n                print(\\"  move_line: IK failed at\\", p.round(3))\\n                return False\\n            qs.append(q)\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(ARM)\\n        for i, qq in enumerate(qs, 1):\\n            t = seconds * i / len(qs)\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])\\n            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n            goal.trajectory.points.append(pt)\\n        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)\\n        res = send.result().get_result_async()\\n        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)\\n        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()\\n        tcp, _ = self.tcp_world()\\n        print(f\\"  move_line: error_code={code} max_joint_err={err:.4f} tcp={tcp.round(3)} \\"\\n              f\\"(target {p1.round(3)}, off {np.linalg.norm(tcp-p1)*1000:.1f}mm)\\")\\n        return err < tol", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
timeout 300 python3 -c "
from rob import *
r = Robot()
q = r.arm_q()
p,qh,_ = r.fk_hand(); print('now yaw/tilt', r.yaw_of(qh))
s = r.ik_down([-0.2135,-0.135,0.62], 0.0, seed=q)
print('sol', None if s is None else np.round(s,3))
print('move', r.move_q(s, 2.0))
p,qh,_ = r.fk_hand(); print('after yaw/tilt', r.yaw_of(qh), 'tcp', r.tcp_world()[0].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih4.png

# openrua op 26
timeout 90 python3 cloud.py robot0_eye_in_hand 0.50 | grep -v depth; python3 - <<'EOF'
import numpy as np
pw=np.load('snaps/robot0_eye_in_hand_world.npy'); zs=pw[...,2]
reg=np.zeros_like(zs,bool); reg[240:370,250:400]=True
m=reg&(zs>0.50); p=pw[m]
print('can top: x',p[:,0].min().round(4),p[:,0].max().round(4),'y',p[:,1].min().round(4),p[:,1].max().round(4),'center',p[:,:2].mean(0).round(4),'zmax',p[:,2].max().round(4))
# exclude the pull-tab: use only z>0.512 (top rim/lid)
m=reg&(zs>0.512); p=pw[m]
print('lid only: x',p[:,0].min().round(4),p[:,0].max().round(4),'y',p[:,1].min().round(4),p[:,1].max().round(4),'center',p[:,:2].mean(0).round(4))
for name,(r0,r1,c0,c1) in {'leftfinger':(355,400,120,210),'rightfinger':(355,400,430,520)}.items():
    reg=np.zeros_like(zs,bool); reg[r0:r1,c0:c1]=True
    m=reg&(zs>0.55); p=pw[m]; print(name,'x',p[:,0].min().round(4),p[:,0].max().round(4),'y',p[:,1].min().round(4),p[:,1].max().round(4))
EOF

# openrua op 27
mkdir -p "$(dirname /workspace/pick_soup2.py)"
cat > /workspace/pick_soup2.py <<'OPENRUA_EOF'
"""Soup pick, take 2: straight-line descent with yaw fixed, close, lift."""
from rob import *

SOUP = np.array([-0.2117, -0.135])
r = Robot("pick_soup2")
print("tcp", r.tcp_world()[0].round(4), "gap", r.finger_gap())

print("== align above can at z=0.60")
assert r.move_line([*SOUP, 0.60], 0.0, seconds=1.5)
print("== straight descent to grasp height")
assert r.move_line([*SOUP, 0.462], 0.0, seconds=3.0, step=0.015)
print("  wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
print("== lift")
assert r.move_line([*SOUP, 0.72], 0.0, seconds=3.0)
print("  gap after lift", r.finger_gap(), "wrench", r.wrench())
OPENRUA_EOF

# openrua op 28
timeout 1200 python3 -u pick_soup2.py 2>&1 | tee pick_soup2.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent3.png

# openrua op 29
timeout 300 python3 -c "
from rob import *
r = Robot()
q = r.arm_q(); print('q now', np.round(q,3), 'tcp', r.tcp_world()[0].round(3))
p0 = np.array([-0.2117,-0.135,0.60]); p1=np.array([-0.2117,-0.135,0.462])
q = r.ik_down(p0, 0, seed=q, verbose=False); print('start', np.round(q,3))
for i in range(1,11):
    p = p0+(p1-p0)*i/10
    s = r.ik_down(p, 0, seed=q, verbose=False)
    print(f'{p[2]:.3f}', np.round(s,3), 'dq max', np.abs(np.array(s)-np.array(q)).max().round(3))
    q = s
"

# openrua op 30
mkdir -p "$(dirname /workspace/kin.py)"
cat > /workspace/kin.py <<'OPENRUA_EOF'
"""Analytic Panda FK (modified DH, checked against /compute_fk) and a
seed-regularised 6-DoF numeric IK, so straight-line paths stay continuous
in joint space (the machine's /compute_ik is position-only and wanders
through the null space between nearby targets)."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

# (a, d, alpha) per joint, then flange; Franka's published parameters
DH = [(0, 0.333, 0), (0, 0, -np.pi / 2), (0, 0.316, np.pi / 2),
      (0.0825, 0, np.pi / 2), (-0.0825, 0.384, -np.pi / 2),
      (0, 0, np.pi / 2), (0.088, 0, np.pi / 2)]
FLANGE_D = 0.107
HAND_YAW = -np.pi / 4          # panda_link8 -> panda_hand (tf_static)
BASE_T = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (tf)
LIMITS = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07],
                   [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])
TCP = 0.1034


def _mdh(a, d, alpha, th):
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(th), np.sin(th)
    return np.array([[ct, -st, 0, a],
                     [st * ca, ct * ca, -sa, -d * sa],
                     [st * sa, ct * sa, ca, d * ca],
                     [0, 0, 0, 1]])


def fk_hand(q):
    """4x4 world pose of panda_hand for 7 joint angles."""
    T = np.eye(4)
    T[:3, 3] = BASE_T
    for (a, d, al), th in zip(DH, q):
        T = T @ _mdh(a, d, al, th)
    T = T @ _mdh(0, FLANGE_D, 0, HAND_YAW)
    return T


def fk_tcp(q):
    T = fk_hand(q)
    return T[:3, 3] + TCP * T[:3, 2]


def target_R(yaw_deg):
    """Hand rotation: z down, fingers (hand y) along world y when yaw=0."""
    return Rot.from_euler("z", yaw_deg, degrees=True).as_matrix() @ np.diag([1.0, -1.0, -1.0])


def ik(pos_tcp, yaw_deg, seed, reg=0.02, pos_w=1000.0, rot_w=100.0):
    """Joints placing the TCP at world pos with a top-down hand at yaw.
    Regularised toward the seed to pick the nearest branch."""
    Rt = target_R(yaw_deg)
    pos_tcp = np.asarray(pos_tcp, float)
    seed = np.asarray(seed, float)

    def resid(q):
        T = fk_hand(q)
        p = T[:3, 3] + TCP * T[:3, 2]
        dR = Rot.from_matrix(T[:3, :3] @ Rt.T).as_rotvec()
        return np.concatenate([pos_w * (p - pos_tcp), rot_w * dR, reg * (q - seed)])

    sol = least_squares(resid, seed, bounds=(LIMITS[:, 0], LIMITS[:, 1]),
                        xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=2000)
    q = sol.x
    T = fk_hand(q)
    p = T[:3, 3] + TCP * T[:3, 2]
    perr = np.linalg.norm(p - pos_tcp)
    rerr = np.degrees(np.linalg.norm(Rot.from_matrix(T[:3, :3] @ Rt.T).as_rotvec()))
    if perr > 0.002 or rerr > 1.0:
        return None, (perr, rerr)
    return q, (perr, rerr)
OPENRUA_EOF

# openrua op 31
timeout 300 python3 -c "
from rob import *
import kin
r = Robot()
q = r.arm_q()
p,qh,_ = r.fk_hand(); T = kin.fk_hand(q)
print('service hand', p.round(4)); print('analytic hand', T[:3,3].round(4))
print('service R'); print(quat_to_R(*qh).round(3)); print('analytic R'); print(T[:3,:3].round(3))
# test a couple of random configs
rng = np.random.default_rng(0)
for _ in range(3):
    qq = rng.uniform(kin.LIMITS[:,0], kin.LIMITS[:,1])
    p,qh,_ = r.fk_hand(list(qq)); T = kin.fk_hand(qq)
    print('pos diff mm', (np.linalg.norm(p-T[:3,3])*1000).round(3), 'R diff', np.abs(quat_to_R(*qh)-T[:3,:3]).max().round(4))
# IK test: path from current to above soup
import time
t=time.time()
sol,err = kin.ik([-0.2117,-0.135,0.60], 0.0, q); print('ik', None if sol is None else np.round(sol,3), err, 'time', round(time.time()-t,2))
"

# openrua op 32
python3 - <<'EOF'
import re
s = open('rob.py').read()
start = s.index('    def ik_down(')
end = s.index('    def move_line(')
new_ik = '''    def ik_down(self, pos_world, yaw_deg=0.0, seed=None, verbose=True):
        """Top-down TCP pose at a world position with a given finger yaw,
        via the analytic 6-DoF IK in kin.py (the machine's /compute_ik is
        position-only and null-space-unstable, verified)."""
        seed = list(seed if seed is not None else self.arm_q())
        sol, (perr, rerr) = kin.ik(pos_world, yaw_deg, seed)
        if verbose:
            print(f"  ik_down: {'ok' if sol is not None else 'FAIL'} perr={perr*1000:.2f}mm rerr={rerr:.2f}deg")
        return None if sol is None else list(sol)

'''
s = s[:start] + new_ik + s[end:]
s = s.replace('import yaml\n', 'import yaml\nimport kin\n', 1)
open('rob.py','w').write(s)
EOF
grep -n "kin" rob.py | head

# openrua op 33
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "            qs.append(q)\\n        goal = FollowJointTrajectory.Goal()", "new_string": "            qs.append(q)\\n        jumps = np.abs(np.diff(np.array([self.arm_q()] + qs), axis=0)).max(axis=1)\\n        if jumps[1:].max(initial=0) > 0.4:\\n            print(\\"  move_line: discontinuous waypoints, max jump\\", jumps.round(3))\\n            return False\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 34
timeout 1200 python3 -u pick_soup2.py 2>&1 | tee pick_soup2.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent4.png

# openrua op 35
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()\\n        tcp, _ = self.tcp_world()\\n        print(f\\"  move_line: error_code={code} max_joint_err={err:.4f} tcp={tcp.round(3)} \\"\\n              f\\"(target {p1.round(3)}, off {np.linalg.norm(tcp-p1)*1000:.1f}mm)\\")\\n        return err < tol", "new_string": "        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()\\n        tcp, _ = self.tcp_world()\\n        print(f\\"  move_line: error_code={code} max_joint_err={err:.4f} tcp={tcp.round(3)} \\"\\n              f\\"(target {p1.round(3)}, off {np.linalg.norm(tcp-p1)*1000:.1f}mm)\\")\\n        if err >= tol:\\n            # controller lag on the last point: settle with a short single-point goal\\n            print(\\"  move_line: settling on final point\\")\\n            return self.move_q(qs[-1], 2.0, tol=tol, retries=1)\\n        return True", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 36
sed -i 's/seconds=1.5)/seconds=3.0)/' pick_soup2.py && timeout 1200 python3 -u pick_soup2.py 2>&1 | tee pick_soup2.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent4.png

# openrua op 37
timeout 120 python3 -c "
from rob import *
r = Robot()
q = r.arm_q(); print('q now', np.round(q,3), 'tcp', r.tcp_world()[0].round(3), 'wrench', r.wrench())
p0=np.array([-0.2117,-0.135,0.60]); p1=np.array([-0.2117,-0.135,0.462])
qa,_ = kin.ik(p0,0,q); qb,_=kin.ik(p1,0,qa)
print('q@0.60', np.round(qa,3)); print('q@0.462', np.round(qb,3)); print('dq', np.round(np.array(qb)-np.array(qa),3))
# alternative seed: nominal posture
nom=[0,-0.16,0,-2.44,0,2.23,0.785]
qa2,_ = kin.ik(p0,0,nom); qb2,_=kin.ik(p1,0,qa2)
print('nominal-seeded q@0.60', np.round(qa2,3)); print('q@0.462', np.round(qb2,3)); print('dq', np.round(np.array(qb2)-np.array(qa2),3))
"

# openrua op 38
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih5.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side5.png

# openrua op 39
timeout 1200 python3 -u - <<'EOF' 2>&1 | tee test_descent.log
from rob import *
r = Robot("test")
print("tcp", r.tcp_world()[0].round(3))
print("== up"); print(r.move_line([-0.13,-0.15,0.65], 0.0, seconds=3.0))
print("== to free spot"); print(r.move_line([-0.21,-0.26,0.60], 0.0, seconds=3.0))
print("== descend (dense waypoints)"); print(r.move_line([-0.21,-0.26,0.462], 0.0, seconds=3.0, step=0.015))
print("wrench", r.wrench())
EOF

# openrua op 40
timeout 1200 python3 -u - <<'EOF' 2>&1 | tee test_descent2.log
from rob import *
r = Robot("test")
print("== up"); print(r.move_line([-0.21,-0.26,0.60], 0.0, seconds=4.0))
print("== descend slow 6s step 0.01"); print(r.move_line([-0.21,-0.26,0.462], 0.0, seconds=6.0, step=0.01))
print("== up 4s"); print(r.move_line([-0.21,-0.26,0.60], 0.0, seconds=4.0, step=0.01))
EOF

# openrua op 41
mkdir -p "$(dirname /workspace/pick_soup3.py)"
cat > /workspace/pick_soup3.py <<'OPENRUA_EOF'
"""Soup pick, take 3: re-localize from above, slow straight descent, close, lift."""
import subprocess
from rob import *

r = Robot("pick_soup3")
SOUP0 = np.array([-0.2117, -0.135])
print("== above approximate can position")
assert r.move_line([*SOUP0, 0.60], 0.0, seconds=4.0)

# re-localize with the eye-in-hand cloud: lid pixels are z>0.50 in a window
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True, capture_output=True)
pw = np.load("snaps/robot0_eye_in_hand_world.npy")
zs = pw[..., 2]
m = (zs > 0.50) & (zs < 0.53) & (np.abs(pw[..., 0] - SOUP0[0]) < 0.06) & (np.abs(pw[..., 1] - SOUP0[1]) < 0.06)
p = pw[m]
xlo, xhi, ylo, yhi = p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max()
print(f"  lid: n={m.sum()} x[{xlo:.4f},{xhi:.4f}] y[{ylo:.4f},{yhi:.4f}] ztop={p[:,2].max():.4f}")
dia = xhi - xlo  # x extent is free of the pull tab
soup = np.array([(xlo + xhi) / 2, ylo + dia / 2])  # tab is on the +y side
print(f"  can centre {soup.round(4)} dia {dia:.4f}")
assert abs(dia - 0.066) < 0.01

print("== centre above can")
assert r.move_line([*soup, 0.60], 0.0, seconds=3.0)
print("== slow straight descent")
assert r.move_line([*soup, 0.462], 0.0, seconds=6.0, step=0.01)
print("  wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
print("== lift")
assert r.move_line([*soup, 0.72], 0.0, seconds=5.0, step=0.01)
print("  gap after lift", r.finger_gap(), "wrench", r.wrench())
OPENRUA_EOF

# openrua op 42
timeout 1500 python3 -u pick_soup3.py 2>&1 | tee pick_soup3.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent5.png

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih6.png

# openrua op 44
timeout 90 python3 cloud.py robot0_eye_in_hand >/dev/null && python3 - <<'EOF'
import numpy as np
pw=np.load('snaps/robot0_eye_in_hand_world.npy'); zs=pw[...,2]
m=(zs>0.44)&(zs<0.52)&(np.abs(pw[...,0]+0.19)<0.09)&(np.abs(pw[...,1]+0.12)<0.09)
# exclude fingers: rows < 350
m[350:,:]=False
p=pw[m]
print('n',m.sum(),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
c=p[:,:2].mean(0); print('centroid',c.round(4))
u,s,vt=np.linalg.svd(p[:,:2]-c,full_matrices=False); ax=vt[0]; print('axis',ax.round(3),'yaw of axis deg',np.degrees(np.arctan2(ax[1],ax[0])).round(1),'sv',s.round(2))
proj=(p[:,:2]-c)@ax; print('length along axis',(proj.max()-proj.min()).round(4))
perp=(p[:,:2]-c)@np.array([-ax[1],ax[0]]); print('width',(perp.max()-perp.min()).round(4))
# top ridge: z>0.48
t=pw[m&(zs>0.48)]; ct=t[:,:2].mean(0); print('ridge n',len(t),'centroid',ct.round(4),'z',t[:,2].max().round(4))
projt=(t[:,:2]-ct)@ax; print('ridge length',(projt.max()-projt.min()).round(4))
EOF

# openrua op 45
mkdir -p "$(dirname /workspace/pick_soup4.py)"
cat > /workspace/pick_soup4.py <<'OPENRUA_EOF'
"""Soup pick, take 4: can is lying on its side; grasp across its diameter."""
import subprocess
from rob import *


def measure(center_guess):
    subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("snaps/robot0_eye_in_hand_world.npy")
    zs = pw[..., 2]
    m = (zs > 0.44) & (zs < 0.52) & (np.abs(pw[..., 0] - center_guess[0]) < 0.09) \
        & (np.abs(pw[..., 1] - center_guess[1]) < 0.09)
    m[350:, :] = False  # fingers live in the bottom rows
    p = pw[m]
    c = p[:, :2].mean(0)
    _, s, vt = np.linalg.svd(p[:, :2] - c, full_matrices=False)
    ax = vt[0]
    axis_yaw = np.degrees(np.arctan2(ax[1], ax[0]))
    grasp_yaw = (axis_yaw + 90) % 180 - 90  # fingers perpendicular to the axis
    proj = (p[:, :2] - c) @ ax
    perp = (p[:, :2] - c) @ np.array([-ax[1], ax[0]])
    print(f"  can: n={m.sum()} centre={c.round(4)} axis_yaw={axis_yaw:.1f} grasp_yaw={grasp_yaw:.1f} "
          f"len={proj.max()-proj.min():.3f} wid={perp.max()-perp.min():.3f} ztop={p[:,2].max():.3f}")
    return c, grasp_yaw


r = Robot("pick_soup4")
c, yaw = measure(np.array([-0.146, -0.114]))
print("== above can, aligned")
assert r.move_line([*c, 0.60], yaw, seconds=4.0)
c, yaw = measure(c)
assert r.move_line([*c, 0.60], yaw, seconds=2.0)
print("== slow straight descent")
assert r.move_line([*c, 0.46], yaw, seconds=6.0, step=0.01)
print("  tcp", r.tcp_world()[0].round(4), "wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
print("== lift")
assert r.move_line([*c, 0.72], yaw, seconds=5.0, step=0.01)
print("  gap after lift", r.finger_gap(), "wrench", r.wrench())
OPENRUA_EOF

# openrua op 46
timeout 1500 python3 -u pick_soup4.py 2>&1 | tee pick_soup4.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent6.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih7.png

# openrua op 47
timeout 1500 python3 -u - <<'EOF' 2>&1 | tee place_soup.log
from rob import *
r = Robot("place_soup")
BASKET = np.array([-0.01, 0.253])
yaw = 17.1
print("gap", r.finger_gap())
print("== transit over basket at z=0.72")
assert r.move_line([*BASKET, 0.72], yaw, seconds=6.0, step=0.02)
print("== lower into basket")
assert r.move_line([*BASKET, 0.60], yaw, seconds=4.0, step=0.01)
print("  gap", r.finger_gap(), "wrench", r.wrench())
print("== release")
r.gripper(0.04)
print("== retreat up")
assert r.move_line([*BASKET, 0.78], yaw, seconds=4.0, step=0.01)
print("gap", r.finger_gap())
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent7.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih8.png

# openrua op 48
mkdir -p "$(dirname /workspace/pick_cheese.py)"
cat > /workspace/pick_cheese.py <<'OPENRUA_EOF'
"""Pick the cream cheese box (flat, 8x4.3x3 cm) and drop it in the basket."""
import subprocess
from rob import *


def measure(center_guess):
    subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("snaps/robot0_eye_in_hand_world.npy")
    zs = pw[..., 2]
    m = (zs > 0.435) & (zs < 0.48) & (np.abs(pw[..., 0] - center_guess[0]) < 0.07) \
        & (np.abs(pw[..., 1] - center_guess[1]) < 0.07)
    m[350:, :] = False
    p = pw[m]
    c = p[:, :2].mean(0)
    _, s, vt = np.linalg.svd(p[:, :2] - c, full_matrices=False)
    ax = vt[0]
    axis_yaw = np.degrees(np.arctan2(ax[1], ax[0]))
    grasp_yaw = (axis_yaw + 90) % 180 - 90  # fingers across the short side
    proj = (p[:, :2] - c) @ ax
    perp = (p[:, :2] - c) @ np.array([-ax[1], ax[0]])
    print(f"  box: n={m.sum()} centre={c.round(4)} axis_yaw={axis_yaw:.1f} grasp_yaw={grasp_yaw:.1f} "
          f"len={proj.max()-proj.min():.3f} wid={perp.max()-perp.min():.3f} ztop={p[:,2].max():.3f}")
    return c, grasp_yaw


r = Robot("pick_cheese")
BOX0 = np.array([0.0945, -0.178])
BASKET = np.array([-0.01, 0.253])
print("== above box")
assert r.move_line([*BOX0, 0.60], 0.0, seconds=6.0, step=0.02)
c, yaw = measure(BOX0)
assert r.move_line([*c, 0.60], yaw, seconds=2.0)
c, yaw = measure(c)
assert r.move_line([*c, 0.60], yaw, seconds=2.0)
print("== slow straight descent")
assert r.move_line([*c, 0.445], yaw, seconds=6.0, step=0.01)
print("  tcp", r.tcp_world()[0].round(4), "wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
assert gap > 0.03, "closed on air"
print("== lift")
assert r.move_line([*c, 0.72], yaw, seconds=5.0, step=0.01)
print("  gap after lift", r.finger_gap())
assert r.finger_gap() > 0.03, "lost the box"
print("== transit over basket")
assert r.move_line([*BASKET, 0.72], yaw, seconds=6.0, step=0.02)
print("== lower into basket")
assert r.move_line([*BASKET, 0.60], yaw, seconds=4.0, step=0.01)
print("  gap", r.finger_gap())
print("== release")
r.gripper(0.04)
print("== retreat up")
assert r.move_line([*BASKET, 0.80], yaw, seconds=4.0, step=0.01)
print("gap", r.finger_gap())
OPENRUA_EOF

# openrua op 49
timeout 1700 python3 -u pick_cheese.py 2>&1 | tee pick_cheese.log; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent8.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih9.png

# openrua op 50
timeout 90 python3 cloud.py robot0_eye_in_hand >/dev/null && python3 - <<'EOF'
import numpy as np
pw=np.load('snaps/robot0_eye_in_hand_world.npy'); col=np.load('snaps/robot0_eye_in_hand_color.npy'); zs=pw[...,2]
# basket footprint (from earlier measurement): x[-0.09,0.07] y[0.168,0.338], rim 0.628
inside=(pw[...,0]>-0.09)&(pw[...,0]<0.07)&(pw[...,1]>0.168)&(pw[...,1]<0.338)&np.isfinite(zs)
inside[350:,:]=False
print('basket footprint points: z range', zs[inside].min().round(3), zs[inside].max().round(3))
# interior (away from rim/walls): inner 9cm square
inner=inside&(pw[...,0]>-0.055)&(pw[...,0]<0.035)&(pw[...,1]>0.205)&(pw[...,1]<0.30)
z=zs[inner]; print('interior z range', z.min().round(3), z.max().round(3), 'n', inner.sum())
# cream cheese box: blue-ish label pixels (b>r) inside footprint
b,g,r_=col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
blue=inside&(b>r_+30)
p=pw[blue]; print('blue (cream cheese) n',blue.sum(),'centre',p[:,:2].mean(0).round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
orange=inside&(r_>g+40)&(g>b)
p=pw[orange]; print('orange (soup label) n',orange.sum(),'centre',p[:,:2].mean(0).round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
# anything outside basket above the table other than ketchup/tomato?
outside=(~inside)&(zs>0.44)&(zs<0.60); outside[350:,:]=False
import cv2
n,lab,st,cen=cv2.connectedComponentsWithStats(outside.astype(np.uint8))
for i in range(1,n):
    if st[i,4]<100: continue
    q=pw[lab==i]; print('outside blob', 'centre',q[:,:2].mean(0).round(3),'ztop',q[:,2].max().round(3),'n',st[i,4])
EOF

# openrua op 51
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-machine-quirks
description: Verified quirks of the Franka Panda sim workstation (/workspace) that contradict or extend the generic docs — IK is position-only, planner frame is world, controller needs slow dense trajectories
metadata:
  type: project
---

Facts verified on the Panda sim machine in /workspace (Sept 2026 session, pick-and-place into basket):

- `/compute_ik` is POSITION-ONLY: it reports success but ignores the requested orientation and wanders through the 7-DoF null space between nearby targets. `/compute_fk` returns poses in frame `world` (base at world (-0.51, 0, 0.42)), matching the TF chain — docs/30-action.md's "planner frame is the arm base" is wrong here.
- Fix used: analytic Panda FK (modified DH) + scipy 6-DoF IK regularised to the seed (`/workspace/kin.py`, `/workspace/rob.py`); matched the FK service to <0.01 mm.
- Single-point joint trajectories bend the TCP path in XY by centimetres; the controller also lags on fast dense trajectories (error_code -5). Straight-line moves with 1 cm waypoints over ~6 s for 15 cm track to <1 mm.
- Table top is z=0.424 world. Gripper max gap measures 0.078–0.080 m. Eye-in-hand depth cloud + `/tf` gives object positions within ~2 mm of FK.

**Why:** these cost several failed grasps (knocked the soup can over) before being diagnosed.
**How to apply:** on this machine, reuse kin.py/rob.py `move_line` and never trust `/compute_ik` orientation.
OPENRUA_EOF

# openrua op 52
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md || touch MEMORY.md) && echo "- [Panda sim machine quirks](panda-sim-machine-quirks.md) — IK position-only, planner frame = world, slow dense trajectories; reuse kin.py/rob.py" >> MEMORY.md && cat MEMORY.md
