#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -v -E "parameter|describe|get_type" && echo --- && ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once && for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 6
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 10)
from rclpy.qos import QoSProfile, DurabilityPolicy
qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", cb, qos)
import time
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
for k, t in got.items():
    print(k, "t=(%.4f %.4f %.4f)" % (t.translation.x, t.translation.y, t.translation.z), "q=(%.4f %.4f %.4f %.4f)" % (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
EOF
python3 /workspace/tfdump.py

# openrua op 7
cat > /workspace/cloud.py <<'EOF'
"""Build a world-frame point cloud from a camera's depth+color; save npz."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
tfs = {}
def cb(m):
    for t in m.transforms: tfs[t.child_frame_id] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 10)
while not all(k in got for k in "dci") or f"{cam}_optical_frame" not in tfs:
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = tfs[f"{cam}_optical_frame"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
H, W = depth.shape
v, u = np.mgrid[0:H, 0:W]
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.savez(f"/workspace/{cam}_cloud.npz", pw=pw, color=color, depth=depth)
print("saved", pw.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
EOF
python3 /workspace/cloud.py birdview && python3 -c "
import numpy as np
d = np.load('/workspace/birdview_cloud.npz'); pw = d['pw']
z = pw[...,2]
print('z percentiles', np.percentile(z[np.isfinite(z)], [1,5,25,50,75,95,99]))
"

# openrua op 8
python3 -c "
import numpy as np, cv2
d = np.load('birdview_cloud.npz'); pw = d['pw']; color=d['color']
z = pw[...,2]
# table top height
tab = z[(z>0.3)&(z<0.5)]
print('table z mode', np.median(tab))
# objects: above table
mask = (z>0.44)&(z<0.75)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4] < 5: continue
    m = lab==i
    pts = pw[m]
    print(i, 'px area', stats[i,4], 'centroid px', cent[i].round(1), 'world xy', pts[:,0].mean().round(3), pts[:,1].mean().round(3), 'zmin/max', pts[:,2].min().round(3), pts[:,2].max().round(3), 'xrange', pts[:,0].min().round(3), pts[:,0].max().round(3), 'yrange', pts[:,1].min().round(3), pts[:,1].max().round(3))
"


# openrua op 9
python3 cloud.py robot0_robotview && python3 cloud.py agentview && python3 -c "
import numpy as np, cv2
for cam in ['robot0_robotview','agentview']:
    d = np.load(f'{cam}_cloud.npz'); pw = d['pw']; color=d['color']
    z = pw[...,2]
    mask = (z>0.445)&(z<0.70)&np.isfinite(z)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    print('==', cam)
    for i in range(1,n):
        if stats[i,4] < 30: continue
        m = lab==i
        pts = pw[m]
        print(i, 'area', stats[i,4], 'px', cent[i].round(0), 'xy', pts[:,0].mean().round(3), pts[:,1].mean().round(3), 'z', pts[:,2].min().round(3), pts[:,2].max().round(3), 'x', pts[:,0].min().round(3), pts[:,0].max().round(3), 'y', pts[:,1].min().round(3), pts[:,1].max().round(3))
"


# openrua op 10
python3 -c "
import numpy as np
d = np.load('robot0_robotview_cloud.npz'); pw = d['pw']
x,y,z = pw[...,0],pw[...,1],pw[...,2]
# table around the objects
m = (x>-0.3)&(x<0.3)&(y>-0.4)&(y<0.4)&(z<0.445)&np.isfinite(z)
print('table z median/percentiles', np.median(z[m]).round(4), np.percentile(z[m],[5,50,95]).round(4))
# cream cheese box PCA
m = (z>0.445)&(x>0.0)&(x<0.2)&(y<-0.15)&(y>-0.3)
P = pw[m][:,:2]; c = P.mean(0); 
u,s,vt = np.linalg.svd(P-c, full_matrices=False)
print('box center', c.round(4), 'axes', vt.round(3), 'angle deg', np.degrees(np.arctan2(vt[0,1], vt[0,0])).round(1))
proj = (P-c)@vt.T
print('extent along axes', (proj.max(0)-proj.min(0)).round(3))
print('box top z', np.percentile(z[m],[50,95]).round(4))
# alphabet soup can
m = (z>0.445)&(x>-0.3)&(x<-0.1)&(y<-0.05)&(y>-0.25)
P = pw[m]; print('can center', P[:,:2].mean(0).round(4), 'top z', np.percentile(P[:,2],[50,95,100]).round(4))
# top surface of can only
mt = m & (z>0.49); print('can top-surface center', pw[mt][:,:2].mean(0).round(4), 'n', mt.sum())
"


# openrua op 11
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show moveit_msgs/srv/GetPositionIK | grep -v "^#" | head -40

# openrua op 12
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
"""Reusable arm controller: joint state, FK, IK, trajectory, gripper.

World frame = panda_link0 frame + BASE offset (from TF world->panda_link0).
Poses passed to ik()/fk() here are WORLD-frame hand poses; conversion to
the planner's base frame happens inside.
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)
TCP_OFF = float(M["hand"]["tcp_offset_m"])
TABLE_Z = 0.4255
# top-down grasp: hand z down, fingers open along world y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def q_down_yaw(yaw):
    """Top-down grasp rotated by yaw (rad) about world z."""
    qz = (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik_cli.wait_for_service(10), "no ik"
        assert self.fk_cli.wait_for_service(10), "no fk"

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk(self, q=None, link="panda_hand"):
        """World-frame pose (xyz, quat) of link for arm config q."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._call(self.fk_cli, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z,
                     p.orientation.w)

    def tcp(self, q=None):
        xyz, quat = self.fk(q)
        return xyz + TCP_OFF * quat_to_R(quat)[:, 2], quat

    def ik(self, xyz_world, quat, seed=None, at_tcp=True, tries=3):
        """Arm joints placing hand (or TCP) at a world pose; None if fail."""
        xyz = np.array(xyz_world, dtype=float)
        if at_tcp:
            xyz = xyz - TCP_OFF * quat_to_R(quat)[:, 2]
        xyz = xyz - BASE
        if seed is None:
            seed = self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            req.ik_request.ik_link_name = "panda_hand"
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            res = self._call(self.ik_cli, req)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def move(self, q, seconds=3.0, via=None):
        """Send trajectory to q (optionally through via points); wait."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            t = seconds * (i + 1) / len(wps)
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        h = send.result()
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result()
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.result.reached_goal} "
              f"stalled={r.result.stalled} gap={gap:.4f}", flush=True)
        return gap

    def goto(self, xyz, quat=Q_DOWN, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print(f"  IK FAILED for {np.round(xyz,3)}", flush=True)
            return None
        self.move(q, seconds)
        p, _ = self.tcp() if at_tcp else self.fk()
        print(f"  now at {'tcp' if at_tcp else 'hand'}={np.round(p,4)} "
              f"target={np.round(xyz,4)} err={np.linalg.norm(p-np.array(xyz)):.4f}",
              flush=True)
        return q
OPENRUA_EOF

# openrua op 13
timeout 120 python3 -u -c "
from ctl import *
c = Ctl()
print('joints', {k: round(v,3) for k,v in c.joints().items()})
xyz, q = c.fk(); print('hand world', xyz.round(4), np.round(q,4))
print('hand base-frame', (xyz-BASE).round(4))
t, _ = c.tcp(); print('tcp world', t.round(4))
print('finger gap', c.finger_gap())
print('wrench', c.wrench().round(3))
# check IK round trip at current pose
sol = c.ik(xyz, q, at_tcp=False); print('ik roundtrip', None if sol is None else np.round(sol,3))
# test IK at pre-grasp above can
sol = c.ik((-0.196, -0.141, 0.60), Q_DOWN); print('ik above can', None if sol is None else np.round(sol,3))
if sol: print(' fk of that:', c.tcp(sol)[0].round(4))
"


# openrua op 14
timeout 60 python3 -c "
import rclpy, numpy as np
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node('t'); b = Buffer(); TransformListener(b, n)
import time
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
for f in ['panda_link0','panda_link1','panda_link2','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_link8','panda_hand','panda_leftfinger','robot0_eye_in_hand_optical_frame']:
    try:
        t = b.lookup_transform('world', f, rclpy.time.Time())
        tr = t.transform.translation; q=t.transform.rotation
        print(f, round(tr.x,4), round(tr.y,4), round(tr.z,4), '| q', round(q.x,3), round(q.y,3), round(q.z,3), round(q.w,3))
    except Exception as e: print(f, 'ERR', e)
"


# openrua op 15
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node('u')
qos = QoSProfile(depth=1); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
got=[]
n.create_subscription(String, '/robot_description', got.append, qos)
while not got: rclpy.spin_once(n, timeout_sec=0.5)
open('/workspace/robot.urdf','w').write(got[0].data)
" && grep -n -E "<link name|<joint name|<origin|<parent|<child" robot.urdf | head -40

# openrua op 16
sed -i 's/^BASE = np.array(\[-0.51, 0.0, 0.42\]).*/BASE = np.zeros(3)  # FK\/IK service poses are already world-frame (verified vs TF)/' ctl.py && grep -n "^BASE" ctl.py && timeout 120 python3 -u -c "
from ctl import *
c = Ctl()
xyz, q = c.fk(); print('hand world', xyz.round(4), np.round(q,4))
t,_ = c.tcp(); print('tcp world', t.round(4))
for tgt in [(-0.196,-0.141,0.62), (-0.196,-0.141,0.50), (0.090,-0.212,0.62), (0.090,-0.212,0.46), (-0.01,0.25,0.78)]:
    sol = c.ik(tgt, Q_DOWN)
    print('ik tcp', tgt, '->', None if sol is None else np.round(sol,3), '' if sol is None else c.tcp(sol)[0].round(4))
"


# openrua op 17
timeout 300 python3 -u -c "
from ctl import *
c = Ctl()
print('open gripper'); c.gripper(0.04)
print('pre-grasp above can'); c.goto((-0.196,-0.141,0.62), Q_DOWN, seconds=4)
print('wrench', c.wrench().round(2))
" && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && python3 tools/perception/cam_snap.py agentview agent1.png

# openrua op 18
timeout 300 python3 -u -c "
from ctl import *
c = Ctl()
print('descend to 0.53'); c.goto((-0.196,-0.141,0.53), Q_DOWN, seconds=2.5)
print('descend to 0.465'); c.goto((-0.196,-0.141,0.465), Q_DOWN, seconds=2.0)
print('wrench', c.wrench().round(2))
print('close'); gap = c.gripper(0.0)
print('wrench', c.wrench().round(2))
" && python3 tools/perception/cam_snap.py agentview agent2.png

# openrua op 19
timeout 400 python3 -u -c "
from ctl import *
c = Ctl()
print('lift'); c.goto((-0.196,-0.141,0.70), Q_DOWN, seconds=3)
print('gap after lift', round(c.finger_gap(),4), 'wrench', c.wrench().round(2))
" && python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 20
timeout 500 python3 -u -c "
from ctl import *
c = Ctl()
print('to above basket'); c.goto((-0.02,0.25,0.72), Q_DOWN, seconds=4)
print('gap', round(c.finger_gap(),4))
print('release'); c.gripper(0.04)
print('retreat up'); c.goto((-0.02,0.25,0.80), Q_DOWN, seconds=2)
" && python3 tools/perception/cam_snap.py agentview agent4.png && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png

# openrua op 21
timeout 400 python3 -u -c "
from ctl import *
c = Ctl()
print('to above box'); c.goto((0.089,-0.213,0.60), Q_DOWN, seconds=4)
" && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png && python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
d = np.load('robot0_eye_in_hand_cloud.npz'); pw=d['pw']; x,y,z=pw[...,0],pw[...,1],pw[...,2]
m = (z>0.44)&(z<0.47)&(x>0.0)&(x<0.2)&(y<-0.15)&(y>-0.3)&np.isfinite(z)
print('box pts', m.sum(), 'center', pw[m][:,:2].mean(0).round(4), 'x', x[m].min().round(3), x[m].max().round(3), 'y', y[m].min().round(3), y[m].max().round(3), 'ztop', np.percentile(z[m],95).round(4))
"

# openrua op 22
python3 -c "
import numpy as np
d = np.load('robot0_eye_in_hand_cloud.npz'); pw=d['pw']; x,y,z=pw[...,0],pw[...,1],pw[...,2]
m = (z>0.445)&(z<0.47)&(x>0.0)&(x<0.2)&(y<-0.15)&(y>-0.3)&np.isfinite(z)
P = pw[m][:,:2]; c=P.mean(0); u,s,vt=np.linalg.svd(P-c, full_matrices=False)
print('center', c.round(4), 'angle deg', np.degrees(np.arctan2(vt[0,1], vt[0,0])).round(1)); proj=(P-c)@vt.T; print('extent', (proj.max(0)-proj.min(0)).round(3))
"

# openrua op 23
timeout 400 python3 -u -c "
from ctl import *
c = Ctl()
print('descend to 0.50'); c.goto((0.085,-0.214,0.50), Q_DOWN, seconds=2.5)
print('descend to 0.436'); c.goto((0.085,-0.214,0.436), Q_DOWN, seconds=2.5)
print('wrench', c.wrench().round(2))
print('close'); gap = c.gripper(0.0)
print('wrench', c.wrench().round(2))
" && python3 tools/perception/cam_snap.py agentview agent6.png

# openrua op 24
timeout 500 python3 -u -c "
from ctl import *
c = Ctl()
print('lift'); c.goto((0.085,-0.214,0.62), Q_DOWN, seconds=3)
print('gap after lift', round(c.finger_gap(),4))
print('to above basket'); c.goto((-0.01,0.25,0.78), Q_DOWN, seconds=4)
print('gap', round(c.finger_gap(),4))
" && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png && python3 tools/perception/cam_snap.py agentview agent7.png

# openrua op 25
timeout 500 python3 -u -c "
from ctl import *
c = Ctl()
print('to above basket (retry)'); c.goto((-0.01,0.25,0.78), Q_DOWN, seconds=3)
print('gap', round(c.finger_gap(),4))
" && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png && python3 tools/perception/cam_snap.py agentview agent7.png

# openrua op 26
timeout 500 python3 -u -c "
from ctl import *
c = Ctl()
print('lower'); c.goto((-0.03,0.25,0.68), Q_DOWN, seconds=2.5)
print('release'); c.gripper(0.04)
print('retreat'); c.goto((-0.03,0.25,0.82), Q_DOWN, seconds=2.5)
" && python3 tools/perception/cam_snap.py agentview agent8.png && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png

# openrua op 27
python3 cloud.py robot0_eye_in_hand >/dev/null && python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
d = np.load('robot0_eye_in_hand_cloud.npz'); pw=d['pw']; x,y,z=pw[...,0],pw[...,1],pw[...,2]
ok = np.isfinite(z)
# basket interior region (inside rim): points below rim height inside footprint
inside = ok&(x>-0.07)&(x<0.055)&(y>0.18)&(y<0.32)&(z<0.62)&(z>0.43)
print('points inside basket footprint below rim:', inside.sum(), 'z range', z[inside].min().round(3), z[inside].max().round(3))
top = inside&(z>0.50)
print('object-top points (z>0.50) center:', pw[top][:,:2].mean(0).round(3), 'x', x[top].min().round(3), x[top].max().round(3), 'y', y[top].min().round(3), y[top].max().round(3), 'zmax', z[top].max().round(3))
# anything at box/can original spots?
d2 = np.load('birdview_cloud.npz'); pw2=d2['pw']; x2,y2,z2=pw2[...,0],pw2[...,1],pw2[...,2]
for name,(cx,cy) in {'alphabet soup spot':(-0.196,-0.141),'cream cheese spot':(0.085,-0.214)}.items():
    m = np.isfinite(z2)&(np.abs(x2-cx)<0.06)&(np.abs(y2-cy)<0.06)
    print(name, 'max z on table there:', z2[m].max().round(4), '(table=0.4255)')
"
