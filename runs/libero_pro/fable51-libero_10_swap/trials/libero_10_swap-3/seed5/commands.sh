#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/README.md && echo ---- && cat tools/perception/cam_snap.py && echo ---- && cat tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py && echo ---- && cat tools/action/gripper_cmd.py && echo ---- && cat tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -v parameter && echo ---- && ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps_$c.png; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60

# openrua op 6
mkdir -p "$(dirname /workspace/pxworld.py)"
cat > /workspace/pxworld.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World coords for many pixels of one camera in one go.

Usage: python3 pxworld.py <camera> u,v [u,v ...]
Also saves <camera>_depth.npy and prints a per-pixel world xyz.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pix = [tuple(int(t) for t in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("pxworld")
    buf = Buffer()
    TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
    np.save(f"{cam}_depth.npy", depth)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    np.save(f"{cam}_T.npy", T)
    np.save(f"{cam}_K.npy", np.array(info.k).reshape(3, 3))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"cam pose world: {T[:3,3]}")
    for (u, v) in pix:
        z = float(depth[v, u])
        if not np.isfinite(z) or z <= 0:
            print(f"({u},{v}): no depth {z}")
            continue
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"({u},{v}) d={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 pxworld.py birdview 340,258 215,265 275,265 310,305 320,180 320,420 250,250 300,250 260,240 290,240 270,285 270,300

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
H,W=d.shape; fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
v,u=np.mgrid[0:H,0:W]
z=d; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
wx,wy,wz=P[...,0],P[...,1],P[...,2]
# height map over region x in [-0.35,0.15], y in [-0.45,0.2]
xs=np.arange(-0.35,0.15,0.02); ys=np.arange(-0.45,0.2,0.02)
print('rows = world x (down), cols = world y (right); values = max height cm above table')
print('      '+' '.join(f'{y*100:4.0f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(np.abs(wx-x)<0.01)&(np.abs(wy-y)<0.01)
        row.append(f'{(wz[m].max()-0.9)*100:4.0f}' if m.any() else '   .')
    print(f'{x*100:5.0f} '+' '.join(row))
"


# openrua op 9
timeout 120 python3 pxworld.py agentview 350,210 && timeout 120 python3 pxworld.py sideview 340,280 && timeout 120 python3 pxworld.py frontview 350,320

# openrua op 10
mkdir -p "$(dirname /workspace/heightmap.py)"
cat > /workspace/heightmap.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Merged height map (cm above table z=0.9) from saved depth/T/K npy files.
Usage: python3 heightmap.py cam1[,cam2,...] x0 x1 y0 y1 [step=0.02]
"""
import sys
import numpy as np

cams = sys.argv[1].split(",")
x0, x1, y0, y1 = map(float, sys.argv[2:6])
step = float(sys.argv[6]) if len(sys.argv) > 6 else 0.02
pts = []
for cam in cams:
    d = np.load(f"{cam}_depth.npy"); T = np.load(f"{cam}_T.npy"); K = np.load(f"{cam}_K.npy")
    H, W = d.shape; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    v, u = np.mgrid[0:H, 0:W]
    ok = np.isfinite(d) & (d > 0)
    X = (u - cx) * d / fx; Y = (v - cy) * d / fy
    P = np.stack([X, Y, d, np.ones_like(d)], -1)[ok] @ T.T
    pts.append(P[:, :3])
P = np.concatenate(pts)
xs = np.arange(x0, x1 + 1e-9, step); ys = np.arange(y0, y1 + 1e-9, step)
print("rows = world x (cm), cols = world y (cm); value = max height cm above table (0.9)")
print("      " + " ".join(f"{y*100:4.0f}" for y in ys))
for x in xs:
    row = []
    for y in ys:
        m = (np.abs(P[:, 0] - x) < step / 2) & (np.abs(P[:, 1] - y) < step / 2)
        row.append(f"{(P[m, 2].max() - 0.9) * 100:4.0f}" if m.any() else "   .")
    print(f"{x*100:5.0f} " + " ".join(row))
OPENRUA_EOF

# openrua op 11
python3 heightmap.py agentview,sideview,frontview -0.30 0.08 -0.45 0.20 0.02

# openrua op 12
for c in agentview frontview sideview birdview; do echo "== $c"; python3 heightmap.py $c -0.18 -0.06 -0.02 0.14 0.01; done

# openrua op 13
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 14
for c in agentview frontview sideview birdview; do timeout 120 python3 pxworld.py $c 320,240 >/dev/null; done; python3 heightmap.py agentview,frontview,sideview,birdview -0.30 0.08 -0.45 0.20 0.02

# openrua op 15
timeout 60 python3 tools/perception/cam_snap.py agentview snaps_agentview.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps_frontview.png

# openrua op 16
python3 heightmap.py birdview,agentview,frontview,sideview -0.26 0.04 -0.26 -0.02 0.01

# openrua op 17
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
got=[]
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
" && grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A14 'link name="panda_leftfinger"' robot.urdf | head -50; grep -n -A8 'joint name="panda_finger_joint1"' robot.urdf; grep -n -A6 'joint name="panda_hand_joint"' robot.urdf

# openrua op 18
python3 - <<'EOF'
import re
s=open('robot.urdf').read()
for name in ['panda_hand_joint','panda_finger_joint1','panda_finger_joint2','panda_joint8']:
    m=re.search(r'<joint name="%s".*?</joint>'%name,s,re.S); print(m.group(0)[:600] if m else name+' none'); print()
for name in ['panda_hand','panda_leftfinger']:
    m=re.search(r'<link name="%s".*?</link>'%name,s,re.S); print(m.group(0)[:800]); print()
EOF
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/

# openrua op 19
find / -name "hand.stl" -o -name "finger.stl" 2>/dev/null | head; find / -iname "*panda*" -maxdepth 4 -type d 2>/dev/null | head

# openrua op 20
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show moveit_msgs/srv/GetCartesianPath | grep -v "^\s*#" | head -60

# openrua op 21
ros2 interface show moveit_msgs/srv/GetCartesianPath 2>/dev/null | grep -E "^(string|geometry_msgs/Pose\[\]|float64|bool|int|moveit_msgs|---|trajectory_msgs|double|uint|Constraints|RobotState|std_msgs|MoveItErrorCodes|RobotTrajectory)" 

# openrua op 22
mkdir -p "$(dirname /workspace/rlib.py)"
cat > /workspace/rlib.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper layer over MoveIt/ros2_control for this Panda.

World frame = panda_link0 + BASE_OFF. All public poses are WORLD frame.
"""
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, WrenchStamped
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_OFF = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 translation
TCP = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def hand_R(finger_axis_world, approach_world):
    """Hand rotation: hand Z = approach dir, hand Y = finger closing axis."""
    z = np.asarray(approach_world, float); z /= np.linalg.norm(z)
    y = np.asarray(finger_axis_world, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)


def top_down_R(yaw=0.0):
    """Hand pointing straight down, finger axis rotated `yaw` from world x."""
    return hand_R([np.cos(yaw), np.sin(yaw), 0.0], [0, 0, -1])


class Robot:
    def __init__(self, name="rlib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.cp_cli = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        for c, n in [(self.fk_cli, "fk"), (self.ik_cli, "ik"), (self.cp_cli, "cartesian")]:
            if not c.wait_for_service(timeout_sec=20):
                raise SystemExit(f"{n} service unavailable")
        if not self.fjt.wait_for_server(timeout_sec=20):
            raise SystemExit("no FJT server")
        if not self.grip.wait_for_server(timeout_sec=20):
            raise SystemExit("no gripper server")

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def _on_wr(self, m):
        self.wr = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                            m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z])

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self.js = {}
            while not self.js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.js

    def arm_q(self):
        js = self.joints()
        return np.array([js[j] for j in ARM])

    def finger(self):
        js = self.joints()
        return js.get("panda_finger_joint1", float("nan"))

    def wrench(self):
        self.wr = {}
        end = time.time() + 5
        while isinstance(self.wr, dict) and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.wr

    def _seed_state(self, q=None):
        js = JointState()
        q = self.arm_q() if q is None else np.asarray(q)
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    # ---------- kinematics (world frame) ----------
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed_state(q)
        res = self._call(self.fk_cli, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_OFF
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP * R[:, 2], R

    def _pose_msg(self, pos_world, R):
        p = Pose()
        pb = np.asarray(pos_world) - BASE_OFF
        p.position.x, p.position.y, p.position.z = map(float, pb)
        qx, qy, qz, qw = R_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        return p

    def ik(self, pos_world, R, seed=None, at_tcp=True, timeout=30, avoid_collisions=False):
        pos = np.asarray(pos_world, float)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.pose_stamped.pose = self._pose_msg(pos, R)
        req.ik_request.robot_state.joint_state = self._seed_state(seed)
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout = Duration(sec=int(timeout % 60 if timeout < 60 else 5))
        res = self._call(self.ik_cli, req, timeout=timeout + 10)
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def cartesian(self, waypoints, start_q=None, at_tcp=True, step=0.005, avoid_collisions=False):
        """waypoints: list of (pos_world, R). Returns (joint positions list, fraction)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = ""
        req.start_state.joint_state = self._seed_state(start_q)
        req.group_name = M["planning"]["group"]
        req.link_name = "panda_hand"
        for pos, R in waypoints:
            pos = np.asarray(pos, float)
            if at_tcp:
                pos = pos - TCP * R[:, 2]
            req.waypoints.append(self._pose_msg(pos, R))
        req.max_step = step
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        res = self._call(self.cp_cli, req, timeout=120)
        if res is None:
            raise RuntimeError("cartesian path: no answer")
        if res.error_code.val != 1:
            raise RuntimeError(f"cartesian path failed code={res.error_code.val}")
        jt = res.solution.joint_trajectory
        idx = [jt.joint_names.index(j) for j in ARM]
        pts = [np.array([p.positions[i] for i in idx]) for p in jt.points]
        return pts, res.fraction

    # ---------- motion ----------
    def move_q(self, targets, seconds, timeout=600):
        """Send one trajectory through the given joint positions (list of arrays)."""
        if isinstance(targets, np.ndarray) and targets.ndim == 1:
            targets = [targets]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(targets)
        for i, q in enumerate(targets):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if rf.result() is None:
            raise RuntimeError("FJT result timeout")
        code = rf.result().result.error_code
        q = self.arm_q()
        err = np.abs(q - np.asarray(targets[-1])).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_cart(self, waypoints, seconds, at_tcp=True, min_fraction=0.99):
        pts, frac = self.cartesian(waypoints, at_tcp=at_tcp)
        print(f"  cartesian fraction={frac:.3f} points={len(pts)}")
        if frac < min_fraction:
            raise RuntimeError(f"cartesian fraction {frac:.3f} < {min_fraction}")
        return self.move_q(pts, seconds)

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        print(f"  gripper -> pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} finger={self.finger():.4f}")
        return r

    def report(self):
        q = self.arm_q()
        pos, R = self.fk(q)
        t = pos + TCP * R[:, 2]
        print(f"  q={np.round(q,3).tolist()}")
        print(f"  hand={np.round(pos,4).tolist()} tcp={np.round(t,4).tolist()} finger={self.finger():.4f}")
        print(f"  hand Z={np.round(R[:,2],3).tolist()} Y(finger axis)={np.round(R[:,1],3).tolist()}")
        return q, pos, R
OPENRUA_EOF

# openrua op 23
timeout 120 python3 -c "
from rlib import *
r=Robot()
r.report()
print('wrench', r.wrench())
"

# openrua op 24
timeout 200 python3 -c "
from rlib import *
import rlib
r=Robot()
q0=r.arm_q()
# FK raw (no BASE_OFF): 
rlib.BASE_OFF=np.zeros(3)
pos,R=r.fk(q0); print('fk raw', pos)
for off,name in [(np.zeros(3),'no offset'),(np.array([-0.66,0,0.912]),'base off')]:
    rlib.BASE_OFF=off
    try:
        q=r.ik(pos+off if name=='base off' else pos, R, at_tcp=False)  # give pos in 'world' as this lib understands
        print(name,'ik ok', np.round(q-q0,3))
    except Exception as e: print(name,'ik fail',e)
"

# openrua op 25
timeout 200 python3 -c "
from rlib import *
import rlib
r=Robot()
q0=r.arm_q()
pos,R=r.fk(q0); print('fk world', np.round(pos,4)); print(np.round(R,3))
qx,qy,qz,qw=R_quat(R); print('quat',np.round([qx,qy,qz,qw],4)); print('roundtrip R ok', np.allclose(quat_R(qx,qy,qz,qw),R,atol=1e-6))
q=r.ik(pos, R, at_tcp=False)
p2,R2=r.fk(q); print('fk of ik', np.round(p2,4)); print(np.round(R2,3)); print('dq',np.round(q-q0,3))
# now test: is the model frame world or base? ask FK with frame_id world explicitly
from moveit_msgs.srv import GetPositionFK
req=GetPositionFK.Request(); req.header.frame_id='world'; req.fk_link_names=['panda_hand','panda_link0']; req.robot_state.joint_state=r._seed_state(q0)
res=r._call(r.fk_cli,req); 
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
"

# openrua op 26
sed -i 's/^BASE_OFF = np.array(\[-0.660, 0.0, 0.912\]).*/BASE_OFF = np.zeros(3)  # MoveIt model frame here IS world (verified: FK(panda_link0) = (-0.66,0,0.912))/' rlib.py && sed -i 's/        req.ik_request.group_name = M\["planning"\]\["group"\]/        req.ik_request.group_name = M["planning"]["group"]\n        req.ik_request.ik_link_name = "panda_hand"/' rlib.py && grep -n "BASE_OFF =\|ik_link_name" rlib.py && timeout 200 python3 -c "
from rlib import *
r=Robot()
q0=r.arm_q()
pos,R=r.fk(q0)
q=r.ik(pos, R, at_tcp=False)
p2,R2=r.fk(q); print('fk of ik', np.round(p2,4)); print(np.round(R2,3)); print('dq',np.round(q-q0,3))
pts,frac=r.cartesian([(pos+np.array([0,0,-0.05]),R)], at_tcp=False)
print('cart frac',frac,len(pts)); p3,R3=r.fk(pts[-1]); print(np.round(p3,4)); print(np.round(R3,3))
"

# openrua op 27
mkdir -p "$(dirname /workspace/step1_pull.py)"
cat > /workspace/step1_pull.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pull the bottom drawer out further using the -y finger behind the front panel."""
import sys
import numpy as np
from rlib import *

r = Robot("step1")
print("start:"); r.report()
R = top_down_R(np.pi / 2)            # fingers along world y
x_c, y_h = -0.11, -0.071
z_hi, z_lo = 1.10, 0.965

if r.finger() < 0.035:
    r.gripper(0.04)

# a. pre-pose above the drawer
q_pre = r.ik([x_c, y_h, z_hi], R)
p, Rk = r.tcp(q_pre); print("pre tcp", np.round(p, 4), "Z", np.round(Rk[:, 2], 3), "Y", np.round(Rk[:, 1], 3))
r.move_q(q_pre, 4.0)
r.report()

# b. descend
r.move_cart([([x_c, y_h, z_lo], R)], 2.5)
r.report()
print("wrench", np.round(r.wrench(), 2))

# c. pull +y
r.move_cart([([x_c, y_h + 0.07, z_lo], R)], 4.0)
r.report()
print("wrench", np.round(r.wrench(), 2))

# e. lift
r.move_cart([([x_c, y_h + 0.07, z_hi], R)], 2.5)
r.report()
OPENRUA_EOF

# openrua op 28
nohup python3 -u step1_pull.py > step1.log 2>&1 &
sleep 60; cat /workspace/step1.log

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py agentview snaps_agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview snaps_frontview.png > /dev/null

# openrua op 30
timeout 300 python3 -u -c "
from rlib import *
import time
r=Robot('t')
q0=r.arm_q(); print('q0',np.round(q0,3))
qt=q0.copy(); qt[0]+=0.4; qt[3]-=0.2
goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names=list(ARM)
pt=JointTrajectoryPoint(positions=[float(v) for v in qt]); pt.time_from_start=Duration(sec=3); goal.trajectory.points=[pt]
t0=time.time()
send=r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(r.node,send,timeout_sec=30); gh=send.result()
rf=gh.get_result_async()
while not rf.done():
    rclpy.spin_once(r.node,timeout_sec=0.2)
    if int((time.time()-t0)*5)%5==0:
        q=r.joints(fresh=False); 
    print(f'{time.time()-t0:5.1f}s j1={r.js.get(\"panda_joint1\",0):.3f} j4={r.js.get(\"panda_joint4\",0):.3f}') if r.js else None
    time.sleep(0.5)
print('result code', rf.result().result.error_code, 'at', time.time()-t0)
for i in range(10):
    q=r.arm_q(); print(f'after {i}: j1={q[0]:.3f} j4={q[3]:.3f}'); time.sleep(1)
"

# openrua op 31
timeout 300 python3 -u -c "
from rlib import *
import time
r=Robot('t')
q0=r.arm_q(); print('q0',np.round(q0,3))
qt=q0.copy(); qt[0]-=1.0
t0=time.time(); code,err=r.move_q(qt,3.0); print('took',time.time()-t0)
q1=r.arm_q(); print('moved',np.round(q1-q0,3))
qt2=q0.copy(); qt2[0]-=0.4  # go back toward original in 2 steps of speed 0.3
t0=time.time(); code,err=r.move_q(qt2,2.0); print('took',time.time()-t0)
print(np.round(r.arm_q()-q0,3))
"

# openrua op 32
python3 - <<'EOF'
import re
s=open('/workspace/rlib.py').read()
old='''    def move_q(self, targets, seconds, timeout=600):
        """Send one trajectory through the given joint positions (list of arrays)."""
        if isinstance(targets, np.ndarray) and targets.ndim == 1:
            targets = [targets]
        goal = FollowJointTrajectory.Goal()'''
new='''    VMAX = 0.12  # rad/s: the sim controller tracks ~0.2 rad/s at most; stay under

    def move_q(self, targets, seconds=None, timeout=900, retry=True):
        """Send one trajectory through the given joint positions (list of arrays).
        seconds=None -> duration from path length at VMAX."""
        if isinstance(targets, np.ndarray) and targets.ndim == 1:
            targets = [targets]
        q0 = self.arm_q()
        path = [q0] + [np.asarray(t) for t in targets]
        length = max(sum(abs(path[i + 1][j] - path[i][j]) for i in range(len(path) - 1)) for j in range(len(ARM)))
        auto = max(1.0, length / self.VMAX)
        if seconds is None or seconds < auto:
            seconds = auto
        print(f"  move: max joint path {length:.3f} rad over {seconds:.1f}s")
        goal = FollowJointTrajectory.Goal()'''
assert old in s; s=s.replace(old,new)
old2='''        code = rf.result().result.error_code
        q = self.arm_q()
        err = np.abs(q - np.asarray(targets[-1])).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err'''
new2='''        code = rf.result().result.error_code
        q = self.arm_q()
        err = np.abs(q - np.asarray(targets[-1])).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        if err > 0.01 and retry:
            print("  -> re-sending final target to converge")
            return self.move_q(targets[-1], None, timeout, retry=False)
        return code, err'''
assert old2 in s; s=s.replace(old2,new2)
old3='''    def move_cart(self, waypoints, seconds, at_tcp=True, min_fraction=0.99):'''
new3='''    def move_cart(self, waypoints, seconds=None, at_tcp=True, min_fraction=0.99):'''
assert old3 in s; s=s.replace(old3,new3)
open('/workspace/rlib.py','w').write(s)
EOF
echo ok

# openrua op 33
cat > step1a.py <<'EOF'
from rlib import *
r = Robot("s1a")
r.report()
R = top_down_R(np.pi / 2)
x_c, y_h, z_hi = -0.11, -0.071, 1.10
q_pre = r.ik([x_c, y_h, z_hi], R)
print("q_pre", np.round(q_pre, 3))
p, Rk = r.tcp(q_pre); print("pre tcp", np.round(p, 4), "Z", np.round(Rk[:, 2], 3), "Y", np.round(Rk[:, 1], 3))
r.move_q(q_pre)
r.report()
EOF
nohup python3 -u step1a.py > step1a.log 2>&1 &
sleep 45; cat step1a.log

# openrua op 34
cat > step1b.py <<'EOF'
from rlib import *
r = Robot("s1b")
r.report()
R = top_down_R(np.pi / 2)
x_c, y_h, z_hi, z_lo = -0.11, -0.071, 1.10, 0.965
r.move_cart([([x_c, y_h, z_lo], R)])
r.report(); print("wrench", np.round(r.wrench(), 2))
r.move_cart([([x_c, y_h + 0.07, z_lo], R)])
r.report(); print("wrench", np.round(r.wrench(), 2))
r.move_cart([([x_c, y_h + 0.07, z_hi], R)])
r.report()
EOF
nohup python3 -u step1b.py > step1b.log 2>&1 &
sleep 100; cat step1b.log

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py agentview snaps_agentview.png >/dev/null; timeout 120 python3 pxworld.py birdview 320,240 >/dev/null; python3 heightmap.py birdview -0.26 0.04 -0.26 0.0 0.01

# openrua op 36
python3 - <<'EOF'
import numpy as np
for cam in ['birdview','agentview','frontview']:
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    H,W=d.shape; fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
    v,u=np.mgrid[0:H,0:W]; ok=np.isfinite(d)&(d>0)
    P=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)[ok]@T.T)[:,:3]
    reg=(P[:,0]>-0.2)&(P[:,0]<-0.05)&(P[:,1]>0.0)&(P[:,1]<0.14)&(P[:,2]>0.9+0.035)&(P[:,2]<0.9+0.07)
    Q=P[reg]
    if len(Q)==0: print(cam,'none'); continue
    print(cam,'rim pts',len(Q),'center',np.round(Q[:,:2].mean(0),4),'x range',np.round([Q[:,0].min(),Q[:,0].max()],3),'y range',np.round([Q[:,1].min(),Q[:,1].max()],3),'zmax',np.round(Q[:,2].max(),4))
    # bowl interior floor
    reg2=(P[:,0]>-0.15)&(P[:,0]<-0.09)&(P[:,1]>0.03)&(P[:,1]<0.09)&(P[:,2]<0.9+0.035)
    if reg2.any(): print('   floor z range',np.round([P[reg2,2].min(),P[reg2,2].max()],4))
EOF

# openrua op 37
ros2 interface show moveit_msgs/srv/GetStateValidity 2>/dev/null | grep -vE "^\s+#" | grep -E "^(RobotState|string|Constraints|---|bool|ContactInformation|CostSource|ConstraintEvalResult)"; ros2 interface show moveit_msgs/srv/ApplyPlanningScene 2>/dev/null | grep -E "^(PlanningScene|---|bool)"

# openrua op 38
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Publish measured scene geometry as MoveIt collision objects (world frame),
and offer a state-validity check helper."""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

TABLE = 0.9


def box(name, xr, yr, zr):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    co.operation = CollisionObject.ADD
    sp = SolidPrimitive(type=SolidPrimitive.BOX,
                        dimensions=[float(xr[1] - xr[0]), float(yr[1] - yr[0]), float(zr[1] - zr[0])])
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(np.mean(xr)), float(np.mean(yr)), float(np.mean(zr)))
    p.orientation.w = 1.0
    co.primitives.append(sp)
    co.primitive_poses.append(p)
    return co


def cyl(name, x, y, r, zr):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    co.operation = CollisionObject.ADD
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(zr[1] - zr[0]), float(r)])
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(np.mean(zr))
    p.orientation.w = 1.0
    co.primitives.append(sp)
    co.primitive_poses.append(p)
    return co


def objects(drawer_front=-0.085, bowl=(-0.138, 0.063), with_bowl=True):
    """drawer_front: y of the front panel's inner face."""
    T = TABLE
    objs = [
        box("table", (-0.6, 0.7), (-0.7, 0.7), (T - 0.05, T)),
        box("cabinet", (-0.245, 0.025), (-0.42, -0.225), (T, T + 0.23)),
        box("top_handle", (-0.155, -0.055), (-0.225, -0.19), (T + 0.175, T + 0.205)),
        box("drawer_floor", (-0.225, 0.005), (-0.225, drawer_front), (T, T + 0.025)),
        box("drawer_wall_l", (-0.225, -0.205), (-0.225, drawer_front), (T, T + 0.08)),
        box("drawer_wall_r", (-0.015, 0.005), (-0.225, drawer_front), (T, T + 0.08)),
        box("drawer_front", (-0.225, 0.005), (drawer_front, drawer_front + 0.015), (T, T + 0.09)),
        box("drawer_handle", (-0.155, -0.065), (drawer_front + 0.015, drawer_front + 0.055), (T + 0.04, T + 0.065)),
        cyl("bottle", 0.04, -0.03, 0.035, (T, T + 0.165)),
    ]
    if with_bowl:
        objs.append(cyl("bowl", bowl[0], bowl[1], 0.056, (T, T + 0.053)))
    return objs


class Scene:
    def __init__(self, node):
        self.node = node
        self.apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = node.create_client(GetStateValidity, "/check_state_validity")
        self.apply.wait_for_service(timeout_sec=20)
        self.valid.wait_for_service(timeout_sec=20)

    def publish(self, objs, remove=()):
        ps = PlanningScene()
        ps.is_diff = True
        for o in objs:
            ps.world.collision_objects.append(o)
        for name in remove:
            co = CollisionObject(); co.id = name; co.header.frame_id = "world"
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(scene=ps)
        fut = self.apply.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success if fut.result() else None

    def check(self, arm_names, q, finger=0.04):
        req = GetStateValidity.Request()
        js = JointState()
        js.name = list(arm_names) + ["panda_finger_joint1", "panda_finger_joint2"]
        js.position = [float(v) for v in q] + [float(finger), float(finger)]
        req.robot_state.joint_state = js
        req.group_name = "panda_arm"
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None:
            return None, ["no answer"]
        contacts = [f"{c.contact_body_1}<->{c.contact_body_2} depth={c.depth:.3f}" for c in res.contacts]
        return res.valid, contacts
OPENRUA_EOF

# openrua op 39
cat > test_poses.py <<'EOF'
from rlib import *
from scene import *
r = Robot("tp")
sc = Scene(r.node)
print("publish", sc.publish(objects()))
q0 = r.arm_q()
print("current valid:", sc.check(ARM, q0, r.finger()))
xb, yb = -0.138, 0.063
Rpick = top_down_R(0.0)
cands = {
  "pick_pre": ([xb - 0.05, yb, 1.05], Rpick),
  "pick_grasp": ([xb - 0.05, yb, 0.925], Rpick),
  "pick_lift": ([xb - 0.05, yb, 1.05], Rpick),
}
th = np.deg2rad(25)
Rplace = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
xp, yp = -0.11, -0.155
cands["place_pre"] = ([xp - 0.05, yp, 1.10], Rplace)
cands["place"] = ([xp - 0.05, yp, 0.985], Rplace)
cands["place_flat"] = ([xp - 0.05, yp, 0.985], Rpick)
seed = q0
for name, (p, R) in cands.items():
    try:
        q = r.ik(p, R, seed=seed)
        seed = q
        tp, Rk = r.tcp(q)
        v, c = sc.check(ARM, q, 0.04)
        print(f"{name}: tcp={np.round(tp,4)} q={np.round(q,3)} valid={v} {c}")
    except Exception as e:
        print(f"{name}: IK error {e}")
EOF
timeout 600 python3 -u test_poses.py

# openrua op 40
cat > test_place.py <<'EOF'
from rlib import *
from scene import *
r = Robot("tp2")
sc = Scene(r.node)
th = np.deg2rad(25)
Rplace = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
yp = -0.155
seed = r.arm_q()
for xh in [-0.16, -0.155, -0.15, -0.145, -0.14, -0.13]:
    for z in [0.975, 0.985, 0.995]:
        try:
            q = r.ik([xh, yp, z], Rplace, seed=seed); seed = q
            v, c = sc.check(ARM, q, 0.04)
            print(f"hand x={xh:.3f} z={z:.3f} valid={v} {c}")
        except Exception as e:
            print(f"hand x={xh:.3f} z={z:.3f} IK error {e}")
EOF
timeout 900 python3 -u test_place.py

# openrua op 41
cat > step2_pick.py <<'EOF'
from rlib import *
from scene import *
r = Robot("s2")
sc = Scene(r.node)
sc.publish(objects(with_bowl=False), remove=["bowl"])
xb, yb = -0.138, 0.063
OFF = 0.047
R = top_down_R(0.0)
pre = [xb - OFF, yb, 1.05]
grasp = [xb - OFF, yb, 0.925]
r.report()
if r.finger() < 0.035:
    r.gripper(0.04)
q_pre = r.ik(pre, R)
print("pre valid", sc.check(ARM, q_pre, 0.04))
r.move_q(q_pre); r.report()
pts, frac = r.cartesian([(grasp, R)])
print("descend frac", frac, "final valid", sc.check(ARM, pts[-1], 0.04))
w0 = r.wrench()
r.move_q(pts); r.report()
print("wrench delta", np.round(r.wrench() - w0, 2))
r.gripper(0.0)
r.spin(1.0)
print("finger after close", r.finger())
r.move_cart([(pre, R)]); r.report()
print("finger after lift", r.finger(), "wrench", np.round(r.wrench(), 2))
EOF
nohup python3 -u step2_pick.py > step2.log 2>&1 &
sleep 120; cat step2.log

# openrua op 42
timeout 60 python3 tools/perception/cam_snap.py agentview snaps_agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview snaps_frontview.png >/dev/null

# openrua op 43
cat > step3_place.py <<'EOF'
from rlib import *
from scene import *
r = Robot("s3")
sc = Scene(r.node)
th = np.deg2rad(25)
Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
xh, yp = -0.14, -0.155
pre = [xh, yp, 1.15]
place = [xh, yp, 0.975]
r.report()
q_pre = r.ik(pre, Rp)
print("pre valid", sc.check(ARM, q_pre, 0.004))
tp, Rk = r.tcp(q_pre); print("pre tcp", np.round(tp, 4), "Z", np.round(Rk[:, 2], 3), "Y", np.round(Rk[:, 1], 3))
r.move_q(q_pre); r.report()
print("finger", r.finger())
pts, frac = r.cartesian([(place, Rp)])
print("descend frac", frac, "final valid", sc.check(ARM, pts[-1], 0.004))
r.move_q(pts); r.report()
print("finger", r.finger(), "wrench", np.round(r.wrench(), 2))
r.gripper(0.04)
r.spin(1.0)
r.move_cart([(pre, Rp)]); r.report()
EOF
nohup python3 -u step3_place.py > step3.log 2>&1 &
sleep 150; cat step3.log

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py agentview snaps_agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview snaps_frontview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview snaps_sideview.png >/dev/null

# openrua op 45
for c in agentview frontview sideview; do timeout 120 python3 pxworld.py $c 320,240 >/dev/null; done; python3 heightmap.py agentview,frontview,sideview -0.24 0.02 -0.25 -0.01 0.01

# openrua op 46
cat > park.py <<'EOF'
import sys
from rlib import *
r = Robot("park")
x, y, z = map(float, sys.argv[1:4])
R = top_down_R(0.0)
q = r.ik([x, y, z], R)
r.move_q(q); r.report()
EOF
timeout 300 python3 -u park.py -0.30 0.25 1.20 && for c in birdview agentview; do timeout 120 python3 pxworld.py $c 320,240 >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py agentview snaps_agentview.png; python3 heightmap.py birdview,agentview -0.24 0.02 -0.25 -0.01 0.01

# openrua op 47
cat > step4_check.py <<'EOF'
from rlib import *
from scene import *
r = Robot("s4c")
sc = Scene(r.node)
th = np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc, ztip = -0.11, 0.953
seed = r.arm_q()
sc.publish(objects(drawer_front=-0.085, bowl=(-0.08, -0.15), with_bowl=True))
for name, y in [("pre", 0.0), ("start_contact", -0.03), ("mid", -0.10)]:
    q = r.ik([xc, y, ztip], Rpush, seed=seed); seed = q
    tp, Rk = r.tcp(q)
    print(name, "tcp", np.round(tp, 4), "hand", np.round(r.fk(q)[0], 4), "valid", sc.check(ARM, q, 0.0), "q", np.round(q, 3))
# closed-drawer scene for the end pose
sc.publish(objects(drawer_front=-0.225, bowl=(-0.08, -0.29), with_bowl=True))
for name, y in [("end", -0.172), ("end_deeper", -0.19)]:
    q = r.ik([xc, y, ztip], Rpush, seed=seed); seed = q
    tp, Rk = r.tcp(q)
    print(name, "tcp", np.round(tp, 4), "hand", np.round(r.fk(q)[0], 4), "valid", sc.check(ARM, q, 0.0), "q", np.round(q, 3))
EOF
timeout 600 python3 -u step4_check.py

# openrua op 48
cat > step4_push.py <<'EOF'
from rlib import *
from scene import *
r = Robot("s4")
th = np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc, ztip = -0.11, 0.953
r.report()
r.gripper(0.0)
q_hi = r.ik([xc, 0.0, 1.10], Rpush)
r.move_q(q_hi); r.report()
r.move_cart([([xc, 0.0, ztip], Rpush)]); r.report()
w0 = r.wrench(); print("wrench0", np.round(w0, 2))
r.move_cart([([xc, -0.10, ztip], Rpush)]); r.report()
print("wrench", np.round(r.wrench() - w0, 2))
r.move_cart([([xc, -0.172, ztip], Rpush)]); r.report()
print("wrench", np.round(r.wrench() - w0, 2))
# retreat
r.move_cart([([xc, -0.10, ztip + 0.02], Rpush), ([xc, 0.0, 1.10], Rpush)]); r.report()
EOF
nohup python3 -u step4_push.py > step4.log 2>&1 &
sleep 240; cat step4.log

# openrua op 49
timeout 60 python3 tools/perception/cam_snap.py agentview snaps_agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview snaps_frontview.png >/dev/null

# openrua op 50
for c in agentview frontview sideview; do timeout 120 python3 pxworld.py $c 320,240 >/dev/null; done; python3 heightmap.py agentview,frontview,sideview -0.24 0.02 -0.25 0.0 0.01

# openrua op 51
cat > diag.py <<'EOF'
from rlib import *
from scene import *
r = Robot("diag")
sc = Scene(r.node)
sc.publish(objects(drawer_front=-0.105, bowl=(-0.075, -0.17), with_bowl=True))
q_stall = np.array([0.351, 0.514, -0.099, -1.958, -1.042, 1.967, 0.257])
q_pre = np.array([0.428, 0.741, -0.166, -1.923, -0.988, 1.983, 0.243])
for name, q in [("stall", q_stall), ("pre", q_pre)]:
    print(name, sc.check(ARM, q, 0.0))
    for link in ["panda_hand", "panda_link7", "panda_link6", "panda_link5", "panda_link4", "panda_link3"]:
        p, R = r.fk(q, link); print("   ", link, np.round(p, 3))
# interpolate between
for a in np.linspace(0, 1, 6):
    q = q_stall + a * (q_pre - q_stall)
    print(f"a={a:.1f}", sc.check(ARM, q, 0.0))
EOF
timeout 300 python3 -u diag.py

# openrua op 52
grep -n "def \|VMAX\|error_code\|err" rlib.py | head -60; ls *.py; cat step4_push.py

# openrua op 53
cat step4.log | head -60

# openrua op 54
head -30 tools/cam_snap.py; ls snaps_* | head

# openrua op 55
ls /workspace /workspace/tools 2>/dev/null; grep -rl "snaps_" /workspace --include=*.py --include=*.md 2>/dev/null | head

# openrua op 56
ls /workspace/tools/perception /workspace/tools/action; grep -n "snap" /workspace/tools/README.md | head

# openrua op 57
cat > step4b.py <<'EOF'
from rlib import *
r = Robot("s4b")
th = np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc = -0.11
r.report()
w0 = r.wrench(); print("wrench@pre", np.round(w0, 2))
z = 1.10
while z > 0.955:
    z -= 0.015
    code, err = r.move_cart([([xc, 0.0, z], Rpush)])
    q, p, R = r.report()
    print(f"target z={z:.3f} tcp={np.round(p + TCP*R[:,2],4)} err={err:.3f} wrench={np.round(r.wrench(),2)}")
    if err > 0.02:
        print("STALL"); break
EOF
timeout 900 python3 -u step4b.py 2>&1 | tee step4b.log

# openrua op 58
grep -h "q=\[" step2.log step3.log step1b.log | sort -u | head -30

# openrua op 59
grep -n -A3 'joint name="panda_joint2"' robot.urdf | head; grep -n "limit" robot.urdf | head -12; grep -n -i "joint2\|effort\|torque" machine.yaml docs/*.md | head -20

# openrua op 60
cat > iksearch.py <<'EOF'
from rlib import *
from scene import *
import sys
r = Robot("iks")
sc = Scene(r.node)
sc.publish(objects(drawer_front=-0.105, bowl=(-0.075, -0.17), with_bowl=True))
th = np.deg2rad(float(sys.argv[1])) if len(sys.argv) > 1 else np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc = -0.11
rng = np.random.default_rng(0)
q_now = r.arm_q()
sols = []
for i in range(60):
    seed = q_now + rng.normal(0, 0.6, 7)
    seed[6] = rng.uniform(-2.5, 2.5)
    try:
        q = r.ik([xc, 0.0, 0.955], Rpush, seed=seed)
    except Exception as e:
        continue
    if any(abs(q - s).max() < 0.05 for s in sols):
        continue
    sols.append(q)
    ok, c = sc.check(ARM, q, 0.0)
    # also end pose reachable from this q?
    try:
        q2 = r.ik([xc, -0.172, 0.955], Rpush, seed=q)
        ok2, c2 = sc.check(ARM, q2, 0.0)
        j2end = q2[1]
    except Exception:
        ok2, j2end = None, None
    print(f"j2={q[1]:.3f} valid={ok} {c[:1]} q={np.round(q,3).tolist()}  end j2={j2end} valid_end={ok2}")
EOF
timeout 600 python3 -u iksearch.py 30 2>&1 | sort -t= -k2 -n | head -40

# openrua op 61
cat > j2test.py <<'EOF'
from rlib import *
from scene import *
r = Robot("j2t")
sc = Scene(r.node)
sc.publish(objects(drawer_front=-0.105, bowl=(-0.075, -0.17), with_bowl=True))
r.report()
# compact posture with j2=0.7: hand high near base
for q in [np.array([0.0, 0.7, 0.0, -2.6, 0.0, 3.3, 0.785]), np.array([0.0, 0.9, 0.0, -2.6, 0.0, 3.5, 0.785])]:
    p, R = r.fk(q); print("fk", np.round(p,3), np.round(R[:,2],2), sc.check(ARM, q, 0.0))
q1 = np.array([0.0, 0.7, 0.0, -2.6, 0.0, 3.3, 0.785])
q_home = np.array([0.0, 0.2, 0.0, -2.2, 0.0, 2.4, 0.785])
r.move_q(q_home); r.report()
r.move_q(q1); r.report()
r.move_q(q_home); r.report()
EOF
timeout 900 python3 -u j2test.py 2>&1 | tee j2test.log

# openrua op 62
python3 - <<'EOF'
import re
s = open("rlib.py").read()
s = s.replace("""    def ik(self, pos_world, R, seed=None, at_tcp=True, timeout=30, avoid_collisions=False):
        pos = np.asarray(pos_world, float)""",
"""    J2MAX = 0.45  # the sim's joint2 stalls at ~0.47 rad whatever the URDF says

    def ik(self, pos_world, R, seed=None, at_tcp=True, timeout=30, avoid_collisions=False, j2max=None):
        \"\"\"j2max: cap panda_joint2 via a joint constraint (default J2MAX; None/inf disables).\"\"\"
        pos = np.asarray(pos_world, float)""")
s = s.replace("""        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout""",
"""        req.ik_request.avoid_collisions = avoid_collisions
        j2max = self.J2MAX if j2max is None else j2max
        if np.isfinite(j2max):
            jc = JointConstraint(joint_name="panda_joint2", position=float((j2max - 1.76) / 2),
                                 tolerance_above=float(j2max - (j2max - 1.76) / 2),
                                 tolerance_below=float((j2max - 1.76) / 2 + 1.76), weight=1.0)
            req.ik_request.constraints.joint_constraints.append(jc)
        req.ik_request.timeout""")
s = s.replace("from moveit_msgs.msg import", "from moveit_msgs.msg import JointConstraint,", 1)
open("rlib.py", "w").write(s)
EOF
grep -n "^from moveit_msgs\|^import\|^from" rlib.py | head

# openrua op 63
sed -i '14i from moveit_msgs.msg import JointConstraint' rlib.py && python3 -c "import rlib" && echo ok

# openrua op 64
cat > iksearch2.py <<'EOF'
from rlib import *
from scene import *
r = Robot("iks2")
sc = Scene(r.node)
rng = np.random.default_rng(1)
q_now = r.arm_q()
xc = -0.11
for tilt in [0, 10, 20, 30, 40]:
    th = np.deg2rad(tilt)
    Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])  # tilt from vertical toward -y
    for ztip in [0.955, 0.975]:
        found = []
        for i in range(25):
            seed = q_now + rng.normal(0, 0.5, 7); seed[1] = 0.2; seed[6] = rng.uniform(-2.5, 2.5)
            try:
                q = r.ik([xc, -0.03, ztip], Rp, seed=seed)
            except Exception:
                continue
            if any(abs(q - s).max() < 0.05 for s in found):
                continue
            found.append(q)
        res = []
        for q in found:
            sc.publish(objects(drawer_front=-0.105, with_bowl=False))
            ok1, c1 = sc.check(ARM, q, 0.0)
            try:
                q2 = r.ik([xc, -0.172, ztip], Rp, seed=q)
            except Exception:
                res.append((q, ok1, c1, None, None, None)); continue
            sc.publish(objects(drawer_front=-0.23, with_bowl=False))
            ok2, c2 = sc.check(ARM, q2, 0.0)
            res.append((q, ok1, c1, q2, ok2, c2))
        print(f"== tilt {tilt} ztip {ztip}: {len(found)} sols")
        for q, ok1, c1, q2, ok2, c2 in res:
            print(f"   start j2={q[1]:.3f} ok={ok1} {c1[:2]}  q={np.round(q,3).tolist()}")
            if q2 is not None:
                print(f"     end j2={q2[1]:.3f} ok={ok2} {c2[:2]} q={np.round(q2,3).tolist()}")
EOF
timeout 900 python3 -u iksearch2.py 2>&1 | tee iks2.log | grep -v "^$" | head -120

# openrua op 65
cat > iksearch3.py <<'EOF'
from rlib import *
from scene import *
import itertools
r = Robot("iks3")
sc = Scene(r.node)
rng = np.random.default_rng(2)
q_now = r.arm_q()
sc.publish(objects(drawer_front=-0.23, with_bowl=False))   # end state (hardest)
for tilt, xc, ztip in itertools.product([0, 20, 40, 60, 80, 90], [-0.11, -0.15, -0.19], [0.96, 0.975]):
    th = np.deg2rad(tilt)
    Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
    found = []
    for i in range(20):
        seed = q_now + rng.normal(0, 0.5, 7); seed[0] = rng.uniform(-0.8, 0.8); seed[1] = rng.uniform(-0.5, 0.4); seed[6] = rng.uniform(-2.5, 2.5)
        try:
            q = r.ik([xc, -0.175, ztip], Rp, seed=seed, timeout=2)
        except Exception:
            continue
        if abs(q[0]) > 1.2 or any(abs(q - s).max() < 0.05 for s in found):
            continue
        found.append(q)
    for q in found:
        ok, c = sc.check(ARM, q, 0.0)
        print(f"tilt={tilt} x={xc} z={ztip} j2={q[1]:.3f} ok={ok} {c[:2]} q={np.round(q,3).tolist()}")
    if not found:
        print(f"tilt={tilt} x={xc} z={ztip}: none")
EOF
timeout 1500 python3 -u iksearch3.py 2>&1 | tee iks3.log | grep -v Traceback

# openrua op 66
cat > iksearch4.py <<'EOF'
from rlib import *
from scene import *
import itertools
r = Robot("iks4")
sc = Scene(r.node)
rng = np.random.default_rng(3)
q_now = r.arm_q()
sc.publish(objects(drawer_front=-0.23, with_bowl=False))
cases = [(20, -0.19, 0.965, -0.215), (20, -0.19, 0.975, -0.215), (20, -0.17, 0.965, -0.215),
         (20, -0.15, 0.955, -0.175), (20, -0.14, 0.955, -0.175), (30, -0.15, 0.955, -0.175), (10, -0.19, 0.965, -0.215),
         (20, -0.21, 0.965, -0.215), (20, -0.19, 0.965, -0.20)]
for tilt, xc, ztip, yend in cases:
    th = np.deg2rad(tilt)
    Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
    found = []
    for i in range(20):
        seed = q_now + rng.normal(0, 0.5, 7); seed[0] = rng.uniform(-0.8, 0.8); seed[1] = rng.uniform(-0.5, 0.4); seed[6] = rng.uniform(-2.5, 2.5)
        try:
            q = r.ik([xc, yend, ztip], Rp, seed=seed, timeout=2, j2max=0.43)
        except Exception:
            continue
        if abs(q[0]) > 1.2 or any(abs(q - s).max() < 0.05 for s in found):
            continue
        found.append(q)
    for q in found:
        ok, c = sc.check(ARM, q, 0.0)
        print(f"tilt={tilt} x={xc} z={ztip} yend={yend} j2={q[1]:.3f} ok={ok} {c[:2]} q={np.round(q,3).tolist()}")
    if not found:
        print(f"tilt={tilt} x={xc} z={ztip} yend={yend}: none")
EOF
timeout 1500 python3 -u iksearch4.py 2>&1 | grep -v Traceback

# openrua op 67
timeout 120 python3 pxworld.py birdview 320,240 >/dev/null && timeout 120 python3 pxworld.py agentview 320,240 > /dev/null; python3 heightmap.py birdview,agentview -0.24 -0.14 -0.13 -0.03 0.01

# openrua op 68
cat > step4c.py <<'EOF'
"""Close the bottom drawer: push its front panel face at x=-0.19 with the closed
fingertips (hand tilted 20 deg so the wrist leans +y), keeping joint2 <= 0.45."""
from rlib import *
r = Robot("s4c")
th = np.deg2rad(20)
Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
xc, ztip, y0 = -0.19, 0.965, -0.04
r.report()
if r.finger() > 0.005:
    r.gripper(0.0)

def cart(waypoints, label):
    pts, frac = r.cartesian(waypoints)
    j2 = max(p[1] for p in pts)
    print(f"[{label}] fraction={frac:.3f} pts={len(pts)} max j2={j2:.3f}")
    if frac < 0.99 or j2 > 0.45:
        raise SystemExit(f"[{label}] path not acceptable")
    code, err = r.move_q(pts)
    q, p, R = r.report()
    t = p + TCP * R[:, 2]
    print(f"[{label}] tcp={np.round(t,4).tolist()} err={err:.4f} wrench={np.round(r.wrench(),2).tolist()}")
    return t, err

# 1. pre pose above, outside the drawer front
q_pre = r.ik([xc, y0, 1.10], Rp, seed=np.array([-0.34, 0.2, -0.09, -2.1, -0.88, 2.5, -0.38]), j2max=0.43)
print("q_pre", np.round(q_pre, 3).tolist())
r.move_q(q_pre); r.report()

# 2. descend to push height; must converge before any push
t, err = cart([([xc, y0, ztip], Rp)], "descend")
if abs(t[2] - ztip) > 0.005 or err > 0.02:
    raise SystemExit("descent did not converge; abort")
w0 = r.wrench()

# 3. push in segments, watch force
for yt in [-0.12, -0.16, -0.19, -0.215, -0.225]:
    t, err = cart([([xc, yt, ztip], Rp)], f"push->{yt}")
    f = r.wrench()[:3] - w0[:3]
    print(f"   dF={np.round(f,2).tolist()} |dF|={np.linalg.norm(f):.1f}")
    if np.linalg.norm(f) > 30 or (abs(t[1] - yt) > 0.01):
        print("   stopped: force/stall"); break

# 4. retreat: back +y a bit, then up
cart([([xc, t[1] + 0.03, ztip + 0.01], Rp), ([xc, y0, 1.10], Rp)], "retreat")
EOF
timeout 1700 python3 -u step4c.py 2>&1 | tee step4c.log

# openrua op 69
timeout 120 python3 pxworld.py birdview 320,240 >/dev/null && timeout 120 python3 pxworld.py agentview 320,240 > /dev/null; python3 heightmap.py birdview,agentview -0.25 0.02 -0.26 -0.06 0.01; timeout 60 ./tools/perception/cam_snap.py frontview snaps_frontview.png; timeout 60 ./tools/perception/cam_snap.py agentview snaps_agentview.png

# openrua op 70
timeout 60 ./tools/perception/cam_snap.py sideview snaps_sideview.png >/dev/null; echo done

# openrua op 71
cat > final.py <<'EOF'
from rlib import *
r = Robot("final")
r.move_q(np.array([0.0, 0.2, 0.0, -2.2, 0.0, 2.4, 0.785])); r.report()
EOF
timeout 600 python3 -u final.py 2>&1 | tail -3
timeout 120 python3 pxworld.py birdview 320,240 >/dev/null && timeout 120 python3 pxworld.py agentview 320,240 > /dev/null
echo "--- whole table: anything >= 3cm outside cabinet/bottle?"
python3 heightmap.py birdview,agentview -0.5 0.5 -0.6 0.6 0.04
echo "--- drawer front region"
python3 heightmap.py birdview,agentview -0.24 0.02 -0.25 -0.15 0.01

# openrua op 72
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-joint2-cap.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-joint2-cap.md <<'OPENRUA_EOF'
---
name: panda-sim-joint2-cap
description: In the /workspace Panda ROS2 sim, panda_joint2 stalls at ~0.47 rad (URDF says 1.76) and the controller only tracks ~0.2 rad/s; cap j2 in IK.
metadata:
  type: project
---

The simulated Franka Panda behind /workspace (robosuite-style cameras, paused clock,
MoveIt services) cannot move panda_joint2 beyond ≈0.47 rad in any posture — FJT
returns error -5 and the arm freezes partway with no external wrench. Independently
of that, the controller tracks at most ~0.2 rad/s (use ≤0.12 rad/s).

**Why:** MoveIt/URDF believe j2 goes to 1.76, so IK and Cartesian paths happily
produce unreachable joint values; several descents "mysteriously" stalled at
tcp z≈1.0–1.07 until this was found (2026-09-12).

**How to apply:** `rlib.Robot.ik(..., j2max=0.43)` (default J2MAX=0.45) adds a
JointConstraint; check `max(p[1] for p in cartesian_path_points) <= 0.45` before
executing. Low tcp targets (z≈0.95) are only reachable near the base (≤~0.5 m
horizontal, e.g. x≈-0.19 for base at x=-0.66). See [[workspace-panda-tools]].
OPENRUA_EOF

# openrua op 73
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true); echo "- [Panda sim joint2 cap](panda-sim-joint2-cap.md) — sim j2 stalls at ~0.47 rad; cap j2 in IK, controller ≤0.12 rad/s" >> MEMORY.md
