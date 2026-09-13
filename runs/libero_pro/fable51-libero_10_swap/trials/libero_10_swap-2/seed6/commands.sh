#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/README.md && echo ---- && cat tools/perception/cam_snap.py && echo ---- && cat tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py && echo ---- && cat tools/action/ik_move.py && echo ---- && cat tools/action/gripper_cmd.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /pipeline_state --once; timeout 20 ros2 topic info /pipeline_state; timeout 30 ros2 node info /robot_bridge

# openrua op 6
timeout 20 ros2 topic echo /tf --once | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 20 ros2 topic echo /tf_static --once | grep -E "frame_id" | head -40

# openrua op 7
cat > /workspace/tfdump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
n.create_subscription(TFMessage, "/tf_static", cb, QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
end = time.time()+5
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    print(f"{p:>28} -> {c:<28} t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
EOF
timeout 60 python3 /workspace/tfdump.py

# openrua op 8
for p in "240 283" "378 280" "378 237" "318 305" "200 350" "450 200"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 9
cat > /workspace/heightmap.py <<'EOF'
"""Build a world-frame height map from a top-down camera's depth frame and report object blobs."""
import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
rclpy.init(); node = rclpy.create_node("hm")
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
frame = f"{cam}_optical_frame"
end = time.time()+20
while time.time()<end and not ("d" in got and "i" in got and "c" in got and tfbuf.can_transform("world", frame, rclpy.time.Time())):
    rclpy.spin_once(node, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
info = got["i"]; fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W = d.shape
u,v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)
pw = pc @ R.T + T
np.save(f"{cam}_world.npy", pw)
hz = pw[...,2]
print("height range", np.nanmin(hz), np.nanmax(hz))
mask = ((hz > 0.905) & (hz < 1.5)).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA] < 15: continue
    m = lab==i
    xs, ys, zs = pw[...,0][m], pw[...,1][m], hz[m]
    print(f"blob {i}: area={stats[i,cv2.CC_STAT_AREA]} px centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) "
          f"x=[{xs.min():.3f},{xs.max():.3f}] y=[{ys.min():.3f},{ys.max():.3f}] zmax={zs.max():.3f} xmean={xs.mean():.3f} ymean={ys.mean():.3f}")
EOF
timeout 120 python3 /workspace/heightmap.py birdview

# openrua op 10
python3 -c "
import cv2
im = cv2.imread('birdview.png')
crop = im[200:340, 200:430]
cv2.imwrite('bird_crop.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
im = cv2.imread('agentview.png')
crop = im[200:360, 230:520]
cv2.imwrite('agent_crop.png', cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
"

# openrua op 11
python3 -c "
import numpy as np
pw = np.load('birdview_world.npy')
z = pw[...,2]
# knob region approx pixels x 355-395, y 222-258
sub = z[222:258, 355:395]
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print((sub*100).round(1).astype(int))
"

# openrua op 12
python3 -c "
import numpy as np, sys
np.set_printoptions(threshold=sys.maxsize, linewidth=300)
pw = np.load('birdview_world.npy')
z = pw[...,2]
sub = z[224:256, 358:392]
print((sub*100-90).round(0).astype(int))
print('x row 240:', pw[240,358:392,0].round(3))
print('y col 375:', pw[224:256,375,1].round(3))
"

# openrua op 13
cat > /workspace/fk.py <<'EOF'
"""Print current hand pose (panda_link0 and world) via /compute_fk."""
import rclpy, yaml, sys, time
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import numpy as np
M = yaml.safe_load(open("/workspace/machine.yaml"))
arm = M["actuators"][0]["joints"]
rclpy.init(); n = rclpy.create_node("fk")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
cli = n.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service()
req = GetPositionFK.Request()
req.fk_link_names = ["panda_hand"]
d = dict(zip(js["m"].name, js["m"].position))
req.robot_state.joint_state.name = arm
req.robot_state.joint_state.position = [d[j] for j in arm]
f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
r = f.result()
p = r.pose_stamped[0].pose
print("joints", [round(d[j],4) for j in arm], "fingers", round(d["panda_finger_joint1"],4), round(d["panda_finger_joint2"],4))
print("hand in base: pos", round(p.position.x,4), round(p.position.y,4), round(p.position.z,4), "quat", round(p.orientation.x,4), round(p.orientation.y,4), round(p.orientation.z,4), round(p.orientation.w,4))
q = p.orientation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
base = np.array([-0.66, 0.0, 0.912])
hw = base + np.array([p.position.x, p.position.y, p.position.z])
tcp = hw + 0.1034*R[:,2]
print("hand in world:", hw.round(4), " tcp in world:", tcp.round(4))
print("hand axes in world: x", R[:,0].round(3), "y", R[:,1].round(3), "z", R[:,2].round(3))
EOF
timeout 60 python3 /workspace/fk.py

# openrua op 14
cat > /workspace/iktest.py <<'EOF'
import rclpy, yaml
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml")); arm = M["actuators"][0]["joints"]
rclpy.init(); n = rclpy.create_node("iktest")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
d = dict(zip(js["m"].name, js["m"].position))
fk = n.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = arm; req.robot_state.joint_state.position = [d[j] for j in arm]
f = fk.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
ps = f.result().pose_stamped[0]
print("FK frame:", repr(ps.header.frame_id), ps.pose.position)
ik = n.create_client(GetPositionIK, "/compute_ik"); ik.wait_for_service()
for fid in ["", "world", "panda_link0"]:
    r = GetPositionIK.Request(); r.ik_request.group_name = "panda_arm"
    r.ik_request.pose_stamped.header.frame_id = fid
    r.ik_request.pose_stamped.pose = ps.pose
    r.ik_request.robot_state.joint_state.name = arm
    r.ik_request.robot_state.joint_state.position = [d[j] for j in arm]
    r.ik_request.timeout.sec = 2
    f = ik.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=60)
    res = f.result()
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    print(f"frame_id={fid!r}: code={res.error_code.val}", [round(sol.get(j, float('nan')),3) for j in arm])
EOF
timeout 120 python3 /workspace/iktest.py

# openrua op 15
cat > /workspace/fkof.py <<'EOF'
import rclpy, sys, numpy as np
from moveit_msgs.srv import GetPositionFK
arm = [f"panda_joint{i}" for i in range(1,8)]
rclpy.init(); n = rclpy.create_node("fkof")
fk = n.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = arm; req.robot_state.joint_state.position = [float(x) for x in sys.argv[1].split(",")]
f = fk.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
p = f.result().pose_stamped[0].pose
q = p.orientation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
hw = np.array([p.position.x, p.position.y, p.position.z]); tcp = hw + 0.1034*R[:,2]
print("hand world", hw.round(4), "quat", np.array([x,y,z,w]).round(4), "tcp", tcp.round(4))
print("axes x", R[:,0].round(3), "y", R[:,1].round(3), "z", R[:,2].round(3))
EOF
timeout 60 python3 /workspace/fkof.py 0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009

# openrua op 16
cat > /workspace/geturdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("geturdf")
got = {}
n.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
while "m" not in got: rclpy.spin_once(n, timeout_sec=0.5)
open("/workspace/robot.urdf","w").write(got["m"].data)
print(len(got["m"].data))
EOF
timeout 60 python3 /workspace/geturdf.py && grep -E "<joint |<link |<parent|<child|<origin|<limit|<axis" /workspace/robot.urdf | head -120

# openrua op 17
mkdir -p "$(dirname /workspace/pk.py)"
cat > /workspace/pk.py <<'OPENRUA_EOF'
"""Panda kinematics + motion helpers for this workstation.

FK/IK done locally (numpy/scipy) because /compute_ik ignores orientation
on this machine. World frame: base at (-0.66, 0, 0.912), identity rotation.
"""
import time
import numpy as np
import rclpy
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIM = np.array(FJT["limits_rad"])
BASE = np.array([-0.66, 0.0, 0.912])
TCP = float(M["hand"]["tcp_offset_m"])

# (xyz, rpy) origin of each joint relative to parent, then rotate about z by q
_ORIG = [
    ((0, 0, 0.333), (0, 0, 0)),
    ((0, 0, 0), (-np.pi / 2, 0, 0)),
    ((0, -0.316, 0), (np.pi / 2, 0, 0)),
    ((0.0825, 0, 0), (np.pi / 2, 0, 0)),
    ((-0.0825, 0.384, 0), (-np.pi / 2, 0, 0)),
    ((0, 0, 0), (np.pi / 2, 0, 0)),
    ((0.088, 0, 0), (np.pi / 2, 0, 0)),
]


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


_FIXED = [_T(o[0], o[1]) for o in _ORIG]
_HAND = _T((0, 0, 0.107), (0, 0, 0)) @ _T((0, 0, 0), (0, 0, -np.pi / 4))


def fk(q, tcp=False):
    """4x4 pose of panda_hand (or TCP point) in world."""
    T = np.eye(4)
    T[:3, 3] = BASE
    for i in range(7):
        T = T @ _FIXED[i] @ _T((0, 0, 0), (0, 0, q[i]))
    T = T @ _HAND
    if tcp:
        T = T @ _T((0, 0, TCP), (0, 0, 0))
    return T


def pose_T(pos, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def topdown_R(yaw):
    """Hand rotation with z pointing down (-world z) and hand x axis at `yaw`
    about world z (yaw=0: hand x = world +x, hand y = world -y).
    Finger closing axis is hand y."""
    return Rot.from_euler("xyz", [np.pi, 0, yaw]).as_matrix()


def ik(target, seed, tcp=False, w_rot=0.3):
    """Solve q for target 4x4 (hand or TCP pose). Returns (q, pos_err, rot_err)."""
    seed = np.asarray(seed, float)

    def resid(q):
        T = fk(q, tcp)
        dp = T[:3, 3] - target[:3, 3]
        dR = Rot.from_matrix(T[:3, :3] @ target[:3, :3].T).as_rotvec()
        return np.concatenate([dp * 10, dR * w_rot * 10, (q - seed) * 0.01])

    best = None
    for k in range(6):
        s = seed if k == 0 else np.clip(seed + np.random.uniform(-0.6, 0.6, 7), LIM[:, 0], LIM[:, 1])
        r = least_squares(resid, s, bounds=(LIM[:, 0] + 0.02, LIM[:, 1] - 0.02), xtol=1e-10, ftol=1e-10)
        T = fk(r.x, tcp)
        pe = np.linalg.norm(T[:3, 3] - target[:3, 3])
        re = np.linalg.norm(Rot.from_matrix(T[:3, :3] @ target[:3, :3].T).as_rotvec())
        if best is None or pe + 0.05 * re < best[1] + 0.05 * best[2]:
            best = (r.x, pe, re)
        if pe < 1e-3 and re < 5e-3:
            break
    return best


class Robot:
    def __init__(self, name="pk"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return np.array([d[j] for j in ARM])

    def fingers(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin()
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def move_joints(self, waypoints, seconds, timeout=600):
        """waypoints: list of 7-vectors, evenly timed up to `seconds`."""
        waypoints = [np.asarray(w, float) for w in waypoints]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        n = len(waypoints)
        for i, w in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=list(map(float, w)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        code = res.result().result.error_code
        q = self.joints()
        err = np.abs(q - waypoints[-1]).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos, R, seconds=3.0, tcp=True, via=None, seed=None):
        """IK to pose then trajectory. Returns (q, pos_err, rot_err)."""
        seed = self.joints() if seed is None else seed
        q, pe, re = ik(pose_T(pos, R), seed, tcp=tcp)
        print(f"  ik pos_err={pe*1000:.1f}mm rot_err={np.degrees(re):.2f}deg q={np.round(q,3)}")
        if pe > 0.005 or re > 0.05:
            raise RuntimeError("IK failed")
        wps = ([] if via is None else list(via)) + [q]
        self.move_joints(wps, seconds)
        T = fk(self.joints(), tcp)
        print(f"  reached tcp={np.round(T[:3,3],4)}")
        return q, pe, re

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={np.round(f,4)}")
        return f
OPENRUA_EOF

# openrua op 18
python3 -c "
import numpy as np, pk
q = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
T = pk.fk(q); print(T.round(4)); print('tcp', pk.fk(q, True)[:3,3].round(4))
q2 = np.array([0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009])
print(pk.fk(q2).round(3))
# IK test: topdown above knob
Tt = pk.pose_T([-0.205, 0.194, 1.0], pk.topdown_R(0.0))
import time; t=time.time(); sol = pk.ik(Tt, q, tcp=True); print(sol, time.time()-t)
print(pk.fk(sol[0], True).round(3))
"

# openrua op 19
python3 -c "
import numpy as np
pw = np.load('birdview_world.npy'); z = pw[...,2]
reg = (slice(220,262), slice(352,398))
Z = z[reg]; X = pw[...,0][reg]; Y = pw[...,1][reg]
dial = (Z>0.912)&(Z<0.935); bar = (Z>0.945)
print('dial center', X[dial].mean().round(4), Y[dial].mean().round(4), 'n', dial.sum(), 'x range', X[dial].min().round(3), X[dial].max().round(3), 'y range', Y[dial].min().round(3), Y[dial].max().round(3))
print('bar center ', X[bar].mean().round(4), Y[bar].mean().round(4), 'n', bar.sum(), 'x range', X[bar].min().round(3), X[bar].max().round(3), 'y range', Y[bar].min().round(3), Y[bar].max().round(3), 'zmax', Z[bar].max().round(3))
# moka pot
reg = (slice(265,305), slice(215,265))
Z = z[reg]; X = pw[...,0][reg]; Y = pw[...,1][reg]
m = Z>0.91
print('moka all', X[m].mean().round(4), Y[m].mean().round(4), 'x', X[m].min().round(3), X[m].max().round(3), 'y', Y[m].min().round(3), Y[m].max().round(3), 'zmax', Z[m].max().round(3))
for lo,hi in [(0.91,0.95),(0.95,1.0),(1.0,1.03),(1.03,1.06)]:
    m = (Z>lo)&(Z<hi)
    if m.sum(): print(f' z in [{lo},{hi}]: n={m.sum()} x={X[m].mean():.4f} y={Y[m].mean():.4f} xr=[{X[m].min():.3f},{X[m].max():.3f}] yr=[{Y[m].min():.3f},{Y[m].max():.3f}]')
"

# openrua op 20
python3 -c "
import numpy as np
pw = np.load('birdview_world.npy'); z = pw[...,2]; X=pw[...,0]; Y=pw[...,1]
box = (X>-0.27)&(X<-0.14)&(Y>0.13)&(Y<0.26)
dial = box&(z>0.912)&(z<0.935); bar = box&(z>0.945)&(z<0.97)
print('dial center', X[dial].mean().round(4), Y[dial].mean().round(4), 'n', dial.sum(), 'x', X[dial].min().round(3), X[dial].max().round(3), 'y', Y[dial].min().round(3), Y[dial].max().round(3))
print('bar center ', X[bar].mean().round(4), Y[bar].mean().round(4), 'n', bar.sum(), 'x', X[bar].min().round(3), X[bar].max().round(3), 'y', Y[bar].min().round(3), Y[bar].max().round(3), 'z', z[bar].min().round(3), z[bar].max().round(3))
# bar principal axis
pts = np.stack([X[bar],Y[bar]],1); pts -= pts.mean(0); u,s,vt = np.linalg.svd(pts, full_matrices=False); print('bar axis', vt[0].round(3), 'sv', s.round(3))
"

# openrua op 21
mkdir -p "$(dirname /workspace/step1_knob.py)"
cat > /workspace/step1_knob.py <<'OPENRUA_EOF'
"""Grasp the stove knob bar from above and rotate it."""
import sys, numpy as np, pk

KNOB = np.array([-0.206, 0.195])
YAW = 0.0            # bar along world x -> fingers close along world y
Z_PRE, Z_GRASP = 1.03, 0.930
ROT = float(sys.argv[1]) if len(sys.argv) > 1 else np.pi / 2

r = pk.Robot("knob")
print("start q", np.round(r.joints(), 3), "fingers", r.fingers())
print("open gripper"); r.gripper(pk.GRIP["open_m"])
print("pre-grasp"); r.move_pose([*KNOB, Z_PRE], pk.topdown_R(YAW), 4.0)
print("descend"); r.move_pose([*KNOB, Z_GRASP], pk.topdown_R(YAW), 2.0)
print("close"); f = r.gripper(pk.GRIP["closed_m"])
print("wrench", r.wrench())
q = r.joints()
print("rotate j7 by", ROT)
q2 = q.copy(); q2[6] += ROT
r.move_joints([q2], 4.0)
print("after rot q", np.round(r.joints(), 3), "fingers", r.fingers(), "wrench", r.wrench())
print("DONE")
OPENRUA_EOF

# openrua op 22
nohup python3 -u step1_knob.py 1.5708 > step1.log 2>&1 &
sleep 45; cat /workspace/step1.log

# openrua op 23
sleep 30; cat /workspace/step1.log

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py birdview bird2.png && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png && python3 -c "
import cv2
im = cv2.imread('bird2.png'); crop = im[200:340, 200:430]
cv2.imwrite('bird_crop2.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))"

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py sideview side2.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png; python3 -c "
import cv2
im = cv2.imread('side2.png'); crop = im[280:400, 200:440]; cv2.imwrite('side_crop2.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('agent2.png'); crop = im[180:300, 340:500]; cv2.imwrite('agent_crop2.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))"

# openrua op 26
mkdir -p "$(dirname /workspace/step2_rot.py)"
cat > /workspace/step2_rot.py <<'OPENRUA_EOF'
"""Rotate j7 by a delta while holding, report progress."""
import sys, numpy as np, pk
ROT = float(sys.argv[1]); secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
r = pk.Robot("rot")
q = r.joints(); print("start q", np.round(q, 3), "fingers", np.round(r.fingers(), 4))
q2 = q.copy(); q2[6] += ROT
r.move_joints([q2], secs)
q3 = r.joints()
print("j7 moved", round(q3[6] - q[6], 4), "of", ROT, "fingers", np.round(r.fingers(), 4), "wrench", np.round(r.wrench()[1], 2))
print("DONE")
OPENRUA_EOF

# openrua op 27
timeout 600 python3 -u step2_rot.py -1.5708 4.0

# openrua op 28
timeout 600 python3 -u step2_rot.py -0.9 4.0

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png; timeout 60 python3 tools/perception/cam_snap.py frontview front3.png; python3 -c "
import cv2
im = cv2.imread('agent3.png'); crop = im[180:300, 340:500]; cv2.imwrite('agent_crop3.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('front3.png'); crop = im[280:380, 340:480]; cv2.imwrite('front_crop3.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))"

# openrua op 30
python3 -c "
import numpy as np, sys
np.set_printoptions(threshold=sys.maxsize, linewidth=300)
pw = np.load('birdview_world.npy'); z = pw[...,2]; X=pw[...,0]; Y=pw[...,1]
# rows -> x, cols -> y. find row/col ranges
rows = np.where((X[:,375]>-0.16)&(X[:,375]<0.09))[0]; cols = np.where((Y[300,:]>0.09)&(Y[300,:]<0.31))[0]
print('rows', rows.min(), rows.max(), 'cols', cols.min(), cols.max())
sub = z[rows.min():rows.max()+1:2, cols.min():cols.max()+1:2]
print('x per row:', X[rows.min():rows.max()+1:2, 375].round(3))
print('y per col:', Y[300, cols.min():cols.max()+1:2].round(3))
print((sub*100-90).round(0).astype(int))
"

# openrua op 31
cat > /workspace/step3_release.py <<'EOF'
import numpy as np, pk
r = pk.Robot("rel")
print("open"); r.gripper(pk.GRIP["open_m"])
q = r.joints()
T = pk.fk(q, True); print("tcp now", T[:3,3].round(4))
# lift straight up, keep yaw as is
pos = T[:3,3].copy(); pos[2] = 1.10
r.move_pose(pos, T[:3,:3], 2.5)
# move above moka pot for inspection with eye-in-hand, yaw 0
r.move_pose([-0.054, -0.264, 1.25], pk.topdown_R(0.0), 4.0)
print("DONE")
EOF
timeout 900 python3 -u step3_release.py

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py birdview bird4.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png; python3 -c "
import cv2
im = cv2.imread('bird4.png'); crop = im[200:340, 200:430]; cv2.imwrite('bird_crop4.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))"

# openrua op 33
timeout 120 python3 /workspace/heightmap.py robot0_eye_in_hand 2>&1 | tail -20

# openrua op 34
python3 -c "
import numpy as np
pw = np.load('robot0_eye_in_hand_world.npy'); z = pw[...,2]; X=pw[...,0]; Y=pw[...,1]
box = (X>-0.12)&(X<0.02)&(Y>-0.38)&(Y<-0.19)&(z>0.905)&(z<1.1)
print('n', box.sum())
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.045),(1.045,1.06)]:
    m = box&(z>lo)&(z<hi)
    if m.sum(): print(f'z[{lo:.3f},{hi:.3f}] n={m.sum():5d} x={X[m].mean():.4f} y={Y[m].mean():.4f} xr=[{X[m].min():.3f},{X[m].max():.3f}] yr=[{Y[m].min():.3f},{Y[m].max():.3f}]')
# top face only: restrict to |y - (-0.262)| < 0.03
top = box&(z>1.03)&(np.abs(Y+0.262)<0.03)
print('top body x range', X[top].min().round(4), X[top].max().round(4), 'center', X[top].mean().round(4), Y[top].mean().round(4))
# for each x-slice of the top region print y extent
for yc in np.arange(-0.36,-0.19,0.01):
    m = box&(z>1.03)&(np.abs(Y-yc)<0.005)
    if m.sum(): print(f'  y~{yc:.2f}: x∈[{X[m].min():.3f},{X[m].max():.3f}] zmax={z[m].max():.3f}')
"

# openrua op 35
mkdir -p "$(dirname /workspace/step4_grasp.py)"
cat > /workspace/step4_grasp.py <<'OPENRUA_EOF'
"""Pick the moka pot from above (fingers closing along world x), lift, and hold."""
import subprocess, numpy as np, pk

YAW = np.pi / 2       # hand y (finger axis) -> world x
r = pk.Robot("grasp")


def move(pos, R, secs, tries=3):
    for _ in range(tries):
        q, _, _ = r.move_pose(pos, R, secs)
        if np.abs(r.joints() - q).max() < 0.01:
            return
        print("  resend (lag)")


def pot_top_center():
    """Eye-in-hand height map -> (cx, cy) of the octagonal lid."""
    subprocess.run(["python3", "/workspace/heightmap.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("/workspace/robot0_eye_in_hand_world.npy")
    X, Y, Z = pw[..., 0], pw[..., 1], pw[..., 2]
    box = (X > -0.15) & (X < 0.06) & (Y > -0.40) & (Y < -0.15) & (Z > 1.026) & (Z < 1.06)
    ys = np.arange(-0.40, -0.15, 0.004)
    wide = []
    for yc in ys:
        m = box & (np.abs(Y - yc) < 0.002)
        if m.sum() > 5 and X[m].max() - X[m].min() > 0.04:
            wide.append((yc, X[m].min(), X[m].max()))
    wide = np.array(wide)
    cy = (wide[:, 0].min() + wide[:, 0].max()) / 2
    mid = wide[np.abs(wide[:, 0] - cy) < 0.015]
    cx = (mid[:, 1].mean() + mid[:, 2].mean()) / 2
    print(f"  lid: y∈[{wide[:,0].min():.4f},{wide[:,0].max():.4f}] x∈[{mid[:,1].mean():.4f},{mid[:,2].mean():.4f}] "
          f"width_x={mid[:,2].mean()-mid[:,1].mean():.4f} center=({cx:.4f},{cy:.4f}) zmax={Z[box].max():.4f}")
    return cx, cy


print("open"); r.gripper(pk.GRIP["open_m"])
print("go above pot"); move([-0.0446, -0.268, 1.22], pk.topdown_R(YAW), 4.0)
cx, cy = pot_top_center()
print("re-center"); move([cx, cy, 1.22], pk.topdown_R(YAW), 2.0)
cx2, cy2 = pot_top_center()
print(f"  second estimate ({cx2:.4f},{cy2:.4f})")
cx, cy = (cx + cx2) / 2, (cy + cy2) / 2
f0, _ = r.wrench(); print("  wrench baseline", f0.round(2))
for z in (1.08, 1.04, 1.00):
    print(f"descend to {z}"); move([cx, cy, z], pk.topdown_R(YAW), 1.5)
    f, _ = r.wrench(); print("  wrench", f.round(2), "delta", (f - f0).round(2))
    if abs(f[2] - f0[2]) > 6:
        print("CONTACT - abort before close"); raise SystemExit(1)
print("close"); fg = r.gripper(pk.GRIP["closed_m"])
print("lift"); move([cx, cy, 1.22], pk.topdown_R(YAW), 3.0)
f, _ = r.wrench(); print("  fingers", np.round(r.fingers(), 4), "wrench", f.round(2), "delta", (f - f0).round(2))
print("DONE")
OPENRUA_EOF

# openrua op 36
nohup python3 -u step4_grasp.py > step4.log 2>&1 &
sleep 90; cat /workspace/step4.log

# openrua op 37
sleep 60; cat /workspace/step4.log | tail -20

# openrua op 38
cat > /workspace/step5_lift.py <<'EOF'
import numpy as np, pk
r = pk.Robot("lift")
q = r.joints(); T = pk.fk(q, True); print("tcp", T[:3,3].round(4), "fingers", np.round(r.fingers(),4))
pos = T[:3,3].copy(); pos[2] = 1.22
for _ in range(3):
    qq,_,_ = r.move_pose(pos, pk.topdown_R(np.pi/2), 2.5)
    if np.abs(r.joints()-qq).max() < 0.01: break
print("wrench", r.wrench()[0].round(2))
print("DONE")
EOF
timeout 900 python3 -u step5_lift.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png && timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png

# openrua op 39
mkdir -p "$(dirname /workspace/step6_knobgrasp.py)"
cat > /workspace/step6_knobgrasp.py <<'OPENRUA_EOF'
"""Grasp the moka pot by its lid knob from above, lift, and hold."""
import subprocess, numpy as np, pk

YAW = np.pi / 2
r = pk.Robot("kg")


def move(pos, R, secs, tries=3):
    for _ in range(tries):
        q, _, _ = r.move_pose(pos, R, secs)
        if np.abs(r.joints() - q).max() < 0.01:
            return
        print("  resend (lag)")


def lid_knob():
    subprocess.run(["python3", "/workspace/heightmap.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("/workspace/robot0_eye_in_hand_world.npy")
    X, Y, Z = pw[..., 0], pw[..., 1], pw[..., 2]
    box = (X > -0.15) & (X < 0.06) & (Y > -0.40) & (Y < -0.15)
    lid = box & (Z > 1.026) & (Z < 1.042)
    knob = box & (Z > 1.044) & (Z < 1.07)
    print(f"  lid zmax={Z[lid].max():.4f} n={lid.sum()}  knob: n={knob.sum()} x∈[{X[knob].min():.4f},{X[knob].max():.4f}] "
          f"y∈[{Y[knob].min():.4f},{Y[knob].max():.4f}] z∈[{Z[knob].min():.4f},{Z[knob].max():.4f}] "
          f"center=({X[knob].mean():.4f},{Y[knob].mean():.4f})")
    return X[knob].mean(), Y[knob].mean(), Z[lid].max(), Z[knob].max()


cx, cy, zlid, zknob = lid_knob()
print("go above knob"); move([cx, cy, 1.22], pk.topdown_R(YAW), 3.0)
cx2, cy2, zlid, zknob = lid_knob()
cx, cy = (cx + cx2) / 2, (cy + cy2) / 2
f0, _ = r.wrench(); print("  wrench baseline", f0.round(2))
zg = zlid + 0.004
for z in (1.10, zlid + 0.02, zg):
    print(f"descend to {z:.4f}"); move([cx, cy, z], pk.topdown_R(YAW), 1.5)
    f, _ = r.wrench(); print("  wrench", f.round(2), "delta", (f - f0).round(2))
    if abs(f[2] - f0[2]) > 6:
        print("CONTACT - abort before close"); raise SystemExit(1)
print("close"); fg = r.gripper(pk.GRIP["closed_m"])
if fg[0] > 0.03:
    print("fingers did not close on the knob (gap too wide) - abort"); raise SystemExit(1)
print("lift 2cm"); move([cx, cy, zg + 0.02], pk.topdown_R(YAW), 1.5)
f, _ = r.wrench(); print("  fingers", np.round(r.fingers(), 4), "wrench", f.round(2), "delta", (f - f0).round(2))
print("lift to 1.22"); move([cx, cy, 1.22], pk.topdown_R(YAW), 3.0)
f, _ = r.wrench(); print("  fingers", np.round(r.fingers(), 4), "wrench", f.round(2), "delta", (f - f0).round(2))
print("DONE")
OPENRUA_EOF

# openrua op 40
nohup python3 -u step6_knobgrasp.py > step6.log 2>&1 &
sleep 100; cat /workspace/step6.log

# openrua op 41
timeout 60 python3 tools/perception/cam_snap.py agentview agent6.png && timeout 60 python3 tools/perception/cam_snap.py sideview side6.png

# openrua op 42
mkdir -p "$(dirname /workspace/step7_place.py)"
cat > /workspace/step7_place.py <<'OPENRUA_EOF'
"""Carry the moka pot to the burner, set it down, release, retreat."""
import numpy as np, pk

BURNER = np.array([-0.057, 0.197])
POT_DROP = 0.141          # pot bottom sits this far below the TCP while hanging from the lid knob
Z_BURNER = 0.930
r = pk.Robot("place")


def move(pos, R, secs, tries=3):
    for _ in range(tries):
        q, _, _ = r.move_pose(pos, R, secs)
        if np.abs(r.joints() - q).max() < 0.01:
            return
        print("  resend (lag)")


print("fingers", np.round(r.fingers(), 4))
T = pk.fk(r.joints(), True)
print("raise"); move([T[0, 3], T[1, 3], 1.26], pk.topdown_R(np.pi / 2), 2.0)
print("over burner"); move([*BURNER, 1.26], pk.topdown_R(0.0), 5.0)
print("fingers", np.round(r.fingers(), 4), "wrench", r.wrench()[0].round(2))
f0, _ = r.wrench()
for z in (1.15, Z_BURNER + POT_DROP + 0.004):
    print(f"lower to {z:.4f}"); move([*BURNER, z], pk.topdown_R(0.0), 2.0)
    f, _ = r.wrench(); print("  wrench", f.round(2), "delta", (f - f0).round(2))
print("release"); r.gripper(pk.GRIP["open_m"])
print("retreat"); move([*BURNER, 1.26], pk.topdown_R(0.0), 3.0)
print("DONE")
OPENRUA_EOF

# openrua op 43
nohup python3 -u step7_place.py > step7.log 2>&1 &
sleep 120; cat /workspace/step7.log

# openrua op 44
sleep 45; cat /workspace/step7.log | tail -12

# openrua op 45
timeout 60 python3 tools/perception/cam_snap.py agentview agent7.png && timeout 60 python3 tools/perception/cam_snap.py sideview side7.png && timeout 60 python3 tools/perception/cam_snap.py birdview bird7.png && python3 -c "
import cv2
im = cv2.imread('bird7.png'); crop = im[200:340, 200:430]; cv2.imwrite('bird_crop7.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))"

# openrua op 46
cat > /workspace/step8_home.py <<'EOF'
import numpy as np, pk
r = pk.Robot("home")
home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
for _ in range(3):
    code, err = r.move_joints([home], 4.0)
    if err < 0.01: break
print("DONE")
EOF
timeout 900 python3 -u step8_home.py && timeout 120 python3 /workspace/heightmap.py birdview && timeout 60 python3 tools/perception/cam_snap.py birdview bird8.png && python3 -c "
import cv2
im = cv2.imread('bird8.png'); crop = im[200:340, 200:430]; cv2.imwrite('bird_crop8.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))"

# openrua op 47
python3 -c "
import numpy as np
pw = np.load('birdview_world.npy'); z = pw[...,2]; X=pw[...,0]; Y=pw[...,1]
# moka pot now: tall pixels over the stove area
pot = (X>-0.15)&(X<0.03)&(Y>0.10)&(Y<0.30)&(z>0.98)
print('pot (z>0.98): n', pot.sum(), 'x', X[pot].min().round(3), X[pot].max().round(3), 'y', Y[pot].min().round(3), Y[pot].max().round(3), 'zmax', z[pot].max().round(3))
top = pot&(z>1.05)
print('pot top (z>1.05): center', X[top].mean().round(4), Y[top].mean().round(4), 'zmax', z[top].max().round(4))
# anything left at old pot location?
old = (X>-0.12)&(X<0.02)&(Y>-0.40)&(Y<-0.18)&(z>0.905)
print('pixels above table at old pot spot:', old.sum())
# knob bar orientation
box = (X>-0.27)&(X<-0.14)&(Y>0.13)&(Y<0.26); bar = box&(z>0.945)&(z<0.97)
pts = np.stack([X[bar],Y[bar]],1); pts -= pts.mean(0); u,s,vt = np.linalg.svd(pts, full_matrices=False)
print('knob bar axis (world xy):', vt[0].round(3), ' -> angle from x axis', np.degrees(np.arctan2(vt[0][1], vt[0][0])).round(1), 'deg')
"
