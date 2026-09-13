#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A3 "At time" | head -5; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | grep -A3 "At time" | head -5; timeout 20 ros2 run tf2_ros tf2_echo world robot0_robotview_optical_frame 2>&1 | grep -A3 "At time" | head -5

# openrua op 7
python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import cv2, numpy as np
img = cv2.imread('birdview.png'); d = np.load('birdview_depth.npy')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# table region mask: dark brown; objects are brighter
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
mask = (gray > 110).astype(np.uint8)
# restrict to table area (ellipse roughly)
mask[:100,:]=0; mask[360:,:]=0; mask[:, :140]=0; mask[:, 520:]=0
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
fx=579.4112549695428; cx=320; cy=240
for i in range(1,n):
    x,y,w,h,a = stats[i]
    if a < 30: continue
    u,v = cent[i]
    z = d[int(v),int(u)]
    # world: X = cam_y - 0.2 ; Y = cam_x ; Z = 3 - cam_z
    camx=(u-cx)*z/fx; camy=(v-cy)*z/fx
    print(f'blob {i}: px=({u:.0f},{v:.0f}) size={w}x{h} area={a} depth={z:.3f} world=({camy-0.2:.3f},{camx:.3f},{3-z:.3f})')
"


# openrua op 8
python3 -c "
import cv2
img = cv2.imread('birdview.png')
crop = img[200:320, 220:420]
cv2.imwrite('bird_crop.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
"

# openrua op 9
for p in "195 235" "390 240" "90 297" "545 290" "320 185" "195 195" "390 205"; do echo -n "agentview $p -> "; python3 tools/perception/px2world.py agentview $p; done

# openrua op 10
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable helpers for this Panda: joint state, FK, IK, trajectory, gripper.
World frame = panda_link0 + BASE offset (from TF: world->panda_link0).
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([-0.510, 0.0, 0.420])  # world -> panda_link0 translation
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def topdown_quat(yaw):
    """Hand z pointing down (-world z), hand x rotated by yaw about world z.
    Fingers open along hand y."""
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # x->x, y->-y, z->-z
    return R_to_quat(Rz @ Rflip)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js["m"] = m
        self._js["t"] = time.time()

    def joints(self, fresh=True):
        """Return dict name->pos from a fresh joint state."""
        t0 = time.time()
        self._js.pop("m", None) if fresh else None
        while "m" not in self._js and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def _spin(self, fut, timeout):
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._spin(self.fk_cli.call_async(req), 30)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                         p.orientation.w])
        return pos, quat

    def ik(self, pos_world, quat, seed=None, at_tcp=False, timeout=30):
        """IK for the hand (or TCP if at_tcp) at a world pose. Returns q or None."""
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
            map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = \
            [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        res = self._spin(self.ik_cli.call_async(req), timeout)
        if res is None:
            print("IK: no answer", file=sys.stderr)
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}", file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, via=None):
        """Send a trajectory (optionally through via points: list of (q, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        h = self._spin(self.fjt.send_goal_async(goal), 60)
        res = self._spin(h.get_result_async(), 600)
        code = res.result.error_code if res else None
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"traj code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        h = self._spin(self.grip.send_goal_async(g), 60)
        res = self._spin(h.get_result_async(), 300)
        r = res.result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"gap={self.finger_gap():.4f}")
        return r

    def tcp(self):
        pos, quat = self.fk()
        return pos + TCP * quat_to_R(quat)[:, 2], quat
OPENRUA_EOF

# openrua op 11
timeout 120 python3 -c "
from rob import *
r = Robot()
q = r.arm_q(); print('q', [round(v,3) for v in q])
for link in ['panda_hand','panda_leftfinger','panda_rightfinger','panda_link8']:
    pos, quat = r.fk(q, link); print(link, 'world pos', pos.round(4), 'quat', quat.round(4))
pos, quat = r.fk(q); R = quat_to_R(quat); print('hand R (cols = hand axes in world):'); print(R.round(3))
print('tcp', r.tcp()[0].round(4))
print('finger gap', r.finger_gap())
"

# openrua op 12
timeout 120 python3 -c "
from rob import *
r = Robot()
for q in [[0,-0.785,0,-2.356,0,1.571,0.785],[0,0,0,-1.571,0,1.571,0.785],[0,-0.161,0,-2.445,0,2.227,0.785]]:
    pos, quat = r.fk(q); print(q, '-> base-frame', (pos-BASE).round(4), quat.round(3))
" 2>&1 | grep -v Warn

# openrua op 13
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE = np.array([-0.510, 0.0, 0.420])  # world -> panda_link0 translation", "new_string": "# Verified: /compute_fk and /compute_ik already work in the WORLD frame\\n# (model root is \'world\'; Panda home pose FK = (0.307,0,0.59)+(-0.51,0,0.42)).\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from rob import *
r = Robot()
pos, quat = r.fk(); print('hand world', pos.round(4), quat.round(3)); print('tcp', r.tcp()[0].round(4))
q = r.ik(pos, quat, at_tcp=False); print('IK roundtrip q', q and [round(v,3) for v in q]); print('cur q', [round(v,3) for v in r.arm_q()])
# test top-down quat at yaw 0 : hand x along world x, fingers along world y
tq = topdown_quat(0.0); print('topdown quat', tq.round(3)); print(quat_to_R(tq).round(2))
# IK test: TCP above white mug
q2 = r.ik([-0.09,-0.18,0.70], tq, at_tcp=True); print('IK above white mug', q2 and [round(v,3) for v in q2])
if q2: print('FK check tcp', (r.fk(q2)[0] + TCP*quat_to_R(r.fk(q2)[1])[:,2]).round(3))
" 2>&1 | grep -v Warn

# openrua op 15
timeout 15 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 "At time" | head -4; timeout 15 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 "At time" | head -4; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"

# openrua op 16
cat >> /workspace/rob.py << 'EOF'


# ---- eye-in-hand helpers -------------------------------------------------
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

EIH_OFF = np.array([0.050, 0.0, -0.001])      # camera in hand frame
EIH_R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # Rz(+90deg): hand->cam


def grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def eih_snap(r, tag="eih"):
    """Grab eye-in-hand color+depth; return (bgr, depth, K, T_world_cam)."""
    col = grab(r.node, "/robot0_eye_in_hand/color/image_raw", Image)
    dep = grab(r.node, "/robot0_eye_in_hand/depth/image_raw", Image)
    info = grab(r.node, "/robot0_eye_in_hand/color/camera_info", CameraInfo)
    br = CvBridge()
    bgr = br.imgmsg_to_cv2(col, "bgr8")
    depth = br.imgmsg_to_cv2(dep, "passthrough").astype(np.float32)
    K = np.array(info.k).reshape(3, 3)
    hpos, hq = r.fk()
    Rh = quat_to_R(hq)
    T = np.eye(4)
    T[:3, :3] = Rh @ EIH_R
    T[:3, 3] = hpos + Rh @ EIH_OFF
    cv2.imwrite(f"{tag}.png", bgr)
    np.save(f"{tag}_depth.npy", depth)
    return bgr, depth, K, T


def px_to_world(u, v, depth, K, T):
    z = float(depth[int(v), int(u)])
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    return (T @ p)[:3]


def world_to_px(pw, K, T):
    pc = np.linalg.inv(T) @ np.array([*pw, 1.0])
    return (K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2])
EOF
timeout 300 python3 -c "
from rob import *
r = Robot()
r.gripper(GRIP['open_m'])
print('gap', r.finger_gap())
# move TCP above the white mug -y wall at safe height, yaw=0 (fingers along world y)
tq = topdown_quat(0.0)
target = [-0.092, -0.18, 0.72]
print('move to', target); print(r.move_pose(target, tq, seconds=4.0, at_tcp=True))
print('tcp now', r.tcp()[0].round(4), 'hand quat', r.fk()[1].round(3))
" 2>&1 | grep -v Warn

# openrua op 17
timeout 300 python3 -c "
from rob import *
r = Robot()
print('q now', [round(v,3) for v in r.arm_q()])
tq = topdown_quat(0.0)
q = r.ik([-0.092, -0.18, 0.72], tq, at_tcp=True); print('ik q', [round(v,3) for v in q])
print(r.move_q(q, 3.0))
print('q after', [round(v,3) for v in r.arm_q()])
print('tcp now', r.tcp()[0].round(4), 'hand quat', r.fk()[1].round(3))
" 2>&1 | grep -v Warn

# openrua op 18
python3 - << 'EOF'
import re
s = open('rob.py').read()
s = s.replace('''        pos = pos - BASE
        req = GetPositionIK.Request()''', '''        pos = pos - BASE
        # IK tip is panda_link8 = panda_hand rotated +45deg about hand z
        quat = R_to_quat(quat_to_R(quat) @ RZ45)
        req = GetPositionIK.Request()''')
s = s.replace('''TCP = float(M["hand"]["tcp_offset_m"])''', '''TCP = float(M["hand"]["tcp_offset_m"])
RZ45 = np.array([[math.cos(math.pi / 4), -math.sin(math.pi / 4), 0],
                 [math.sin(math.pi / 4), math.cos(math.pi / 4), 0], [0, 0, 1]])''')
open('rob.py','w').write(s)
EOF
timeout 300 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
q = r.ik([-0.092, -0.18, 0.72], tq, at_tcp=True); print('ik q', [round(v,3) for v in q])
p, hq = r.fk(q); print('predicted hand quat', hq.round(3))
print(r.move_q(q, 3.0))
print('tcp now', r.tcp()[0].round(4), 'hand quat', r.fk()[1].round(3))
" 2>&1 | grep -v Warn

# openrua op 19
python3 - << 'EOF'
s = open('rob.py').read()
s = s.replace('''        code = res.result.error_code if res else None
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"traj code={code} max_joint_err={err:.4f}")
        return code, err''', '''        code = res.result.error_code if res else None
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"traj code={code} max_joint_err={err:.4f}")
        if err > 0.01 and retries > 0:  # controller lag: resend converges
            return self.move_q(q, max(1.5, seconds / 2), retries=retries - 1)
        return code, err''')
s = s.replace('def move_q(self, q, seconds=3.0, via=None):', 'def move_q(self, q, seconds=3.0, via=None, retries=2):')
open('rob.py','w').write(s)
EOF
timeout 300 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
q = r.ik([-0.092, -0.18, 0.72], tq, at_tcp=True)
print(r.move_q(q, 2.0))
print('tcp now', r.tcp()[0].round(4), 'hand quat', r.fk()[1].round(3))
bgr, depth, K, T = eih_snap(r, 'eih1')
print('cam pose', T[:3,3].round(3)); print(T[:3,:3].round(2))
# depth stats
print('depth center', depth[240,320], 'min', np.nanmin(depth), 'max', np.nanmax(depth))
" 2>&1 | grep -v Warn

# openrua op 20
cat >> /workspace/rob.py << 'EOF'


def fit_circle(pts):
    """Least-squares circle fit to Nx2 points -> (cx, cy, r)."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x * x + y * y
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = c[0], c[1]
    r = math.sqrt(c[2] + cx * cx + cy * cy)
    return cx, cy, r


def rim_fit(depth, K, T, table_z, zmin, zmax, roi=None, tag=None, bgr=None):
    """World-fit of a mug rim: pixels whose world z lies in [zmin,zmax]."""
    H, W = depth.shape
    vv, uu = np.mgrid[0:H, 0:W]
    z = depth
    ok = np.isfinite(z) & (z > 0.02)
    if roi is not None:
        u0, v0, u1, v1 = roi
        ok &= (uu >= u0) & (uu < u1) & (vv >= v0) & (vv < v1)
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1],
                   z, np.ones_like(z)], -1)
    pw = pc @ T.T
    sel = ok & (pw[..., 2] > zmin) & (pw[..., 2] < zmax)
    pts = pw[sel][:, :3]
    if tag and bgr is not None:
        vis = bgr.copy(); vis[sel] = (0, 255, 0); cv2.imwrite(f"{tag}_rim.png", vis)
    return pts
EOF
timeout 120 python3 -c "
from rob import *
import numpy as np, cv2
depth = np.load('eih1_depth.npy'); bgr = cv2.imread('eih1.png')
K = np.array([[312.77408948188935,0,320],[0,312.77408948188935,240],[0,0,1]])
T = np.eye(4); T[:3,:3] = np.array([[0,-1,0],[-1,0,0],[0,0,-1]]); T[:3,3] = [-0.042,-0.18,0.824]
# world z histogram in the middle region
H,W = depth.shape; vv,uu = np.mgrid[0:H,0:W]
pc = np.stack([(uu-320)*depth/312.774,(vv-240)*depth/312.774,depth,np.ones_like(depth)],-1); pw = pc@T.T
zz = pw[...,2]
hist, edges = np.histogram(zz[(vv<340)], bins=np.arange(0.40,0.90,0.01))
for h,e in zip(hist,edges):
    if h>50: print(f'z {e:.2f}: {h}')
pts = rim_fit(depth,K,T,0.475,0.545,0.60, roi=(200,180,440,380), tag='eih1', bgr=bgr)
print('rim pts', len(pts), 'z range', pts[:,2].min().round(3), pts[:,2].max().round(3), 'z max top', np.percentile(pts[:,2],95).round(4))
cx,cy,r = fit_circle(pts[:,:2]); print('circle', round(cx,4), round(cy,4), round(r,4))
" 2>&1 | grep -v Warn

# openrua op 21
timeout 60 python3 -c "
import numpy as np
d = np.load('eih1_depth.npy')
# columns through the left finger and right finger, print depth along rows
for u in [150, 180, 200, 460, 480]:
    print('u',u, [round(float(d[v,u]),3) for v in range(330,480,10)])
print('table samples', [round(float(d[v,u]),3) for u,v in [(100,100),(500,100),(320,60),(600,300),(60,300)]])
# where does the table plane depth vary? (flat surface should vary with pixel angle if depth=z)
" ; python3 -c "
import numpy as np, cv2
d = np.load('eih1_depth.npy'); cv2.imwrite('eih1_depth_vis.png', ((d-d.min())/(d.max()-d.min())*255).astype('uint8'))"

# openrua op 22
timeout 60 python3 -c "
from rob import *
depth = np.load('eih1_depth.npy'); bgr = cv2.imread('eih1.png')
K = np.array([[312.774,0,320],[0,312.774,240],[0,0,1]])
T = np.eye(4); T[:3,:3] = np.array([[0,-1,0],[-1,0,0],[0,0,-1]]); T[:3,3] = [-0.042,-0.18,0.824]
H,W = depth.shape; vv,uu = np.mgrid[0:H,0:W]
pc = np.stack([(uu-320)*depth/312.774,(vv-240)*depth/312.774,depth,np.ones_like(depth)],-1); pw = pc@T.T
zz = pw[...,2]; roi = (uu>230)&(uu<410)&(vv>220)&(vv<360)&(depth<0.39)
hist, edges = np.histogram(zz[roi], bins=np.arange(0.50,0.60,0.004))
for h,e in zip(hist,edges): print(f'z {e:.3f}: {h}')
pts = rim_fit(depth,K,T,0.475,0.540,0.60, roi=(230,220,410,360), tag='eih1', bgr=bgr)
cx,cy,r = fit_circle(pts[:,:2]); print('rim circle', round(cx,4), round(cy,4), round(r,4), 'n', len(pts), 'ztop', np.percentile(pts[:,2],90).round(4))
# also the plate: pixels in the top-right region with z ~ table
" 2>&1 | grep -v Warn

# openrua op 23
timeout 60 python3 -c "
import numpy as np
d = np.load('birdview_depth.npy')
# white mug around px (277,264); yellow hidden; red mug around (322,235); plates (250,287),(390,282)
for name,(u,v),w in [('white',(277,264),14),('red',(322,235),14),('plateL',(250,287),20),('plateR',(390,282),20),('table',(200,250),5)]:
    win = d[v-w:v+w+1, u-w:u+w+1]
    print(name, 'min depth', win.min().round(4), '-> top z', (3-win.min()).round(4), ' median z', (3-np.median(win)).round(4))
"

# openrua op 24
timeout 60 python3 -c "
import numpy as np
d = np.load('birdview_depth.npy')
u,v,w = 277,264,12
win = d[v-w:v+w+1, u-w:u+w+1]
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print((3-win)[::1, ::1])
"

# openrua op 25
timeout 60 python3 -c "
import numpy as np
d = np.load('eih1_depth.npy')
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print('plate region depth (rows 160..240 step 10, cols 370..470 step 10):')
print(d[160:250:10, 370:480:10])
print('yellow mug region (rows 210..330, cols 0..140):')
print(d[210:340:10, 0:150:10])
"

# openrua op 26
timeout 120 python3 -c "
from rob import *
r = Robot()
node = r.node
col = grab(node, '/agentview/color/image_raw', Image); dep = grab(node, '/agentview/depth/image_raw', Image); info = grab(node, '/agentview/color/camera_info', CameraInfo)
br = CvBridge(); bgr = br.imgmsg_to_cv2(col,'bgr8'); depth = br.imgmsg_to_cv2(dep,'passthrough').astype(np.float32)
K = np.array(info.k).reshape(3,3); print('K', K[0,0], K[0,2], K[1,2])
# agentview TF from tf2_echo: T=(0.607,0,0.96) q=(0.618,0.618,-0.343,-0.343)
T = np.eye(4); T[:3,:3] = quat_to_R([0.618,0.618,-0.343,-0.343]); T[:3,3] = [0.607,0,0.96]
tcp, hq = r.tcp(); Rh = quat_to_R(hq)
print('FK tcp', tcp.round(4))
for name, pw in [('tcp', tcp), ('fingerA', tcp + Rh@[0,0.045,0]), ('fingerB', tcp + Rh@[0,-0.045,0]), ('hand origin', r.fk()[0])]:
    u,v = world_to_px(pw, K, T); z = depth[int(round(v)), int(round(u))]
    pback = px_to_world(int(round(u)), int(round(v)), depth, K, T)
    print(f'{name}: px=({u:.1f},{v:.1f}) depth={z:.3f} -> world {pback.round(3)}  (FK {pw.round(3)})')
cv2.imwrite('agentview2.png', bgr); np.save('agentview2_depth.npy', depth)
# also mark
vis = bgr.copy()
for pw in [tcp, tcp + Rh@[0,0.045,0], tcp + Rh@[0,-0.045,0]]:
    u,v = world_to_px(pw, K, T); cv2.circle(vis, (int(u),int(v)), 4, (0,255,0), 1)
cv2.imwrite('agentview2_marked.png', vis)
" 2>&1 | grep -v Warn

# openrua op 27
timeout 60 python3 -c "
from rob import *
depth = np.load('agentview2_depth.npy')
K = np.array([[579.4112549695428,0,320],[0,579.4112549695428,240],[0,0,1]])
T = np.eye(4); T[:3,:3] = quat_to_R([0.618,0.618,-0.343,-0.343]); T[:3,3] = [0.607,0,0.96]
for name,(u,v) in [('table1',(320,400)),('table2',(100,420)),('table3',(500,200)),('plateL center',(95,320)),('plateR center',(545,312)),('white top',(195,222)),('white mug top edge',(200,218)),('yellow top',(390,230)),('red top',(320,170))]:
    print(name, px_to_world(u,v,depth,K,T).round(4), 'd', depth[v,u].round(3))
# column scan through white mug center u=200 from v=200..240: world z
print('col u=200:', [(v, px_to_world(200,v,depth,K,T)[2].round(3)) for v in range(210,236,2)])
print('col u=390:', [(v, px_to_world(390,v,depth,K,T)[2].round(3)) for v in range(214,244,2)])
" 2>&1 | grep -v Warn

# openrua op 28
timeout 120 python3 -c "
from rob import *
r = Robot()
col = grab(r.node, '/birdview/color/image_raw', Image); dep = grab(r.node, '/birdview/depth/image_raw', Image)
br = CvBridge(); bgr = br.imgmsg_to_cv2(col,'bgr8'); depth = br.imgmsg_to_cv2(dep,'passthrough').astype(np.float32)
cv2.imwrite('birdview2.png', bgr); np.save('birdview2_depth.npy', depth)
K = np.array([[579.4112549695428,0,320],[0,579.4112549695428,240],[0,0,1]])
T = np.eye(4); T[:3,:3] = quat_to_R([0.707,0.707,0,0]); T[:3,3] = [-0.2,0,3.0]
H,W = depth.shape; vv,uu = np.mgrid[0:H,0:W]
pc = np.stack([(uu-320)*depth/579.411,(vv-240)*depth/579.411,depth,np.ones_like(depth)],-1); pw = pc@T.T
zz = pw[...,2]
print('table z (mode):', np.round(np.median(zz[(vv>300)&(vv<340)&(uu>200)&(uu<450)]),4))
# plates: pixels with z in [0.435,0.46] -> cluster left/right by world y
for name, ysel in [('plateL', pw[...,1] < -0.2), ('plateR', pw[...,1] > 0.2)]:
    sel = (zz>0.435)&(zz<0.47)&ysel&(np.abs(pw[...,0])<0.25)
    pts = pw[sel][:,:3]
    if len(pts): 
        cx,cy,rr = fit_circle(pts[:,:2]); print(name, 'n',len(pts),'center', round(cx,4), round(cy,4), 'r', round(rr,4), 'top z', np.percentile(pts[:,2],90).round(4), 'centroid', pts[:,:2].mean(0).round(4))
# mugs: pixels z>0.50
sel = (zz>0.50)&(zz<0.62)&(np.abs(pw[...,0])<0.3)&(np.abs(pw[...,1])<0.4)
pts = pw[sel][:,:3]
# cluster by simple grid on y
for name, m in [('white', pts[:,1]<-0.1), ('mid', (pts[:,1]>=-0.1)&(pts[:,1]<0.05)&(pts[:,0]<-0.15)), ('yellow', (pts[:,1]>=0.05))]:
    p = pts[m]
    if len(p): cx,cy,rr = fit_circle(p[:,:2]); print(name, 'n', len(p), 'circle', round(cx,4), round(cy,4), round(rr,4), 'ztop', np.percentile(p[:,2],95).round(4))
" 2>&1 | grep -v Warn

# openrua op 29
timeout 60 python3 -c "
import cv2, numpy as np
img = cv2.imread('birdview2.png'); crop = img[200:330, 200:440]
cv2.imwrite('bird2_crop.png', cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
d = np.load('birdview2_depth.npy'); z = 3-d
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print('plate L area rows 270..305 step 3, cols 232..270 step 3'); print(z[270:306:3, 232:272:3])
print('plate R area rows 265..300, cols 372..410'); print(z[265:301:3, 372:412:3])
"

# openrua op 30
timeout 60 python3 -c "
from rob import *
depth = np.load('birdview2_depth.npy')
T = np.eye(4); T[:3,:3] = quat_to_R([0.707,0.707,0,0]); T[:3,3] = [-0.2,0,3.0]
H,W = depth.shape; vv,uu = np.mgrid[0:H,0:W]
pc = np.stack([(uu-320)*depth/579.411,(vv-240)*depth/579.411,depth,np.ones_like(depth)],-1); pw = pc@T.T
zz = pw[...,2]
for name,(u0,u1,v0,v1) in [('plateL',(225,275,262,312)),('plateR',(365,415,258,308))]:
    sel = (zz>0.435)&(zz<0.47)&(uu>=u0)&(uu<u1)&(vv>=v0)&(vv<v1)
    pts = pw[sel][:,:3]
    # use only the rim (outer edge) points for circle: z>0.449
    rim = pts[pts[:,2]>0.449]
    cx,cy,rr = fit_circle(rim[:,:2]); print(name,'rim-fit center',round(cx,4),round(cy,4),'r',round(rr,4),'| all-pts centroid',pts[:,:2].mean(0).round(4),'n',len(pts), 'rim top', rim[:,2].max().round(4), 'center z', np.percentile(pts[:,2],10).round(4))
" 2>&1 | grep -v Warn

# openrua op 31
timeout 400 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
cx, cy, rr, rim = -0.0893, -0.1772, 0.0448, 0.549
wall = np.array([cx, cy - rr, 0])
print(r.move_pose([wall[0], wall[1], 0.63], tq, seconds=2.5, at_tcp=True))
tcp, hq = r.tcp(); print('tcp', tcp.round(4), 'quat', hq.round(3))
bgr, depth, K, T = eih_snap(r, 'eih2')
# where does the TCP project at the rim height? draw it
u,v = world_to_px([tcp[0], tcp[1], rim], K, T); print('tcp@rim px', round(u), round(v))
vis = bgr.copy(); cv2.circle(vis,(int(u),int(v)),5,(0,255,0),2)
# also project the fingers' inner faces at rim height
for dy in (0.0387, -0.0387):
    uu_,vv_ = world_to_px([tcp[0], tcp[1]+dy, rim], K, T); cv2.circle(vis,(int(uu_),int(vv_)),4,(255,0,0),2)
cv2.imwrite('eih2_marked.png', vis)
pts = rim_fit(depth,K,T,0.427,0.53,0.60, roi=(100,100,540,400), tag='eih2', bgr=bgr)
c = fit_circle(pts[:,:2]); print('rim circle now', np.round(c,4), 'ztop', np.percentile(pts[:,2],90).round(4))
" 2>&1 | grep -v Warn

# openrua op 32
timeout 600 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
wx, wy, rim = -0.0891, -0.2227, 0.549
print('descend to 1cm above rim'); print(r.move_pose([wx, wy, rim+0.01], tq, seconds=2.0))
print('descend to 2.5cm below rim'); print(r.move_pose([wx, wy, rim-0.025], tq, seconds=1.5))
tcp,_ = r.tcp(); print('tcp', tcp.round(4))
print('close'); r.gripper(GRIP['closed_m'])
print('gap after close', round(r.finger_gap(),4))
eih_snap(r, 'eih3')
" 2>&1 | grep -v Warn

# openrua op 33
timeout 600 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
print(r.move_pose([-0.0891, -0.2227, 0.76], tq, seconds=2.5))
print('tcp', r.tcp()[0].round(4), 'gap', round(r.finger_gap(),4))
col = grab(r.node, '/agentview/color/image_raw', Image); cv2.imwrite('agentview3.png', CvBridge().imgmsg_to_cv2(col,'bgr8'))
" 2>&1 | grep -v Warn

# openrua op 34
timeout 900 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
rr = 0.0442
plateR = np.array([-0.022, 0.302])
tgt = [plateR[0], plateR[1]-rr]
q_mid = r.ik([-0.06, 0.02, 0.80], tq, at_tcp=True)
q_end = r.ik([tgt[0], tgt[1], 0.78], tq, at_tcp=True, seed=q_mid)
print('mid', q_mid and np.round(q_mid,3)); print('end', q_end and np.round(q_end,3))
print(r.move_q(q_end, 5.0, via=[(q_mid, 2.5)]))
print('tcp', r.tcp()[0].round(4), 'gap', round(r.finger_gap(),4))
bgr, depth, K, T = eih_snap(r, 'eih4')
" 2>&1 | grep -v Warn

# openrua op 35
timeout 900 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
x, y = -0.0224, 0.2578
print(r.move_pose([x, y, 0.60], tq, seconds=2.5))
print(r.move_pose([x, y, 0.546], tq, seconds=1.5))
print('tcp', r.tcp()[0].round(4))
r.gripper(GRIP['open_m'])
print('gap', round(r.finger_gap(),4))
print(r.move_pose([x, y, 0.68], tq, seconds=2.0))
col = grab(r.node, '/agentview/color/image_raw', Image); cv2.imwrite('agentview4.png', CvBridge().imgmsg_to_cv2(col,'bgr8'))
" 2>&1 | grep -v Warn

# openrua op 36
timeout 900 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
print(r.move_pose([-0.07, 0.095, 0.70], tq, seconds=3.0))
tcp,_ = r.tcp(); print('tcp', tcp.round(4))
bgr, depth, K, T = eih_snap(r, 'eih5')
pts = rim_fit(depth,K,T,0.427,0.50,0.60, roi=(150,120,500,420), tag='eih5', bgr=bgr)
print('n', len(pts)); 
# histogram of z
h,e = np.histogram(pts[:,2], bins=np.arange(0.50,0.60,0.005)); print([(round(float(a),3),int(b)) for a,b in zip(e,h) if b>0])
top = pts[pts[:,2] > np.percentile(pts[:,2],60)]
c = fit_circle(top[:,:2]); print('rim circle (top pts)', np.round(c,4), 'ztop', np.percentile(pts[:,2],95).round(4))
c2 = fit_circle(pts[:,:2]); print('rim circle (all)', np.round(c2,4))
" 2>&1 | grep -v Warn

# openrua op 37
timeout 60 python3 -c "
from rob import *
depth = np.load('eih5_depth.npy'); bgr = cv2.imread('eih5.png')
K = np.array([[312.774,0,320],[0,312.774,240],[0,0,1]])
T = np.eye(4); T[:3,:3] = np.array([[0,-1,0],[-1,0,0],[0,0,-1]]); T[:3,3] = [-0.0695+0.05, 0.0954, 0.7001+0.1034-0.001]
pts = rim_fit(depth,K,T,0.427,0.536,0.60, roi=(240,220,420,380), tag=None, bgr=None)
c = np.array([-0.07, 0.0968])
for it in range(4):
    d = np.linalg.norm(pts[:,:2]-c, axis=1); sel = (d>0.03)&(d<0.06)
    cx,cy,rr = fit_circle(pts[sel][:,:2]); c = np.array([cx,cy])
    print(f'iter {it}: center ({cx:.4f},{cy:.4f}) r={rr:.4f} n={sel.sum()} resid={np.std(np.linalg.norm(pts[sel][:,:2]-c,axis=1)-rr):.4f}')
print('rim z top', np.percentile(pts[:,2],95).round(4))
# outer vs inner radius: distances histogram
d = np.linalg.norm(pts[:,:2]-c, axis=1); h,e = np.histogram(d[(d>0.03)&(d<0.07)], bins=np.arange(0.03,0.07,0.002)); print([(round(float(a),3),int(b)) for a,b in zip(e,h)])
" 2>&1 | grep -v Warn

# openrua op 38
timeout 900 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
cx, cy, rr, rim = -0.0705, 0.0968, 0.0484, 0.540
wx, wy = cx, cy + rr
print(r.move_pose([wx, wy, 0.62], tq, seconds=2.0))
bgr, depth, K, T = eih_snap(r, 'eih6')
tcp,_ = r.tcp(); u,v = world_to_px([tcp[0], tcp[1], rim], K, T)
vis = bgr.copy(); cv2.circle(vis,(int(u),int(v)),5,(0,255,0),2)
for dy in (0.0387, -0.0387):
    uu_,vv_ = world_to_px([tcp[0], tcp[1]+dy, rim], K, T); cv2.circle(vis,(int(uu_),int(vv_)),4,(255,0,0),2)
cv2.imwrite('eih6_marked.png', vis)
print(r.move_pose([wx, wy, rim+0.01], tq, seconds=2.0))
print(r.move_pose([wx, wy, rim-0.025], tq, seconds=1.5))
print('tcp', r.tcp()[0].round(4))
r.gripper(GRIP['closed_m'])
print('gap after close', round(r.finger_gap(),4))
eih_snap(r, 'eih7')
" 2>&1 | grep -v Warn

# openrua op 39
timeout 900 python3 -c "
from rob import *
r = Robot()
tq = topdown_quat(0.0)
rr = 0.0484
plateL = np.array([0.002, -0.303])
tx, ty = plateL[0], plateL[1] + rr
print(r.move_pose([-0.0711, 0.1453, 0.76], tq, seconds=2.5))
print('lifted tcp', r.tcp()[0].round(4), 'gap', round(r.finger_gap(),4))
q_mid = r.ik([-0.035, -0.05, 0.79], tq, at_tcp=True)
q_end = r.ik([tx, ty, 0.78], tq, at_tcp=True, seed=q_mid)
print(r.move_q(q_end, 5.0, via=[(q_mid, 2.5)]))
print('tcp', r.tcp()[0].round(4), 'gap', round(r.finger_gap(),4))
eih_snap(r, 'eih8')
print(r.move_pose([tx, ty, 0.60], tq, seconds=2.5))
print(r.move_pose([tx, ty, 0.537], tq, seconds=1.5))
print('tcp', r.tcp()[0].round(4))
r.gripper(GRIP['open_m'])
print(r.move_pose([tx, ty, 0.70], tq, seconds=2.0))
col = grab(r.node, '/agentview/color/image_raw', Image); cv2.imwrite('agentview5.png', CvBridge().imgmsg_to_cv2(col,'bgr8'))
" 2>&1 | grep -v Warn

# openrua op 40
timeout 900 python3 -c "
from rob import *
r = Robot()
print(r.move_q([0, -0.161, 0, -2.445, 0, 2.227, 0.785], 3.0))
col = grab(r.node, '/birdview/color/image_raw', Image); dep = grab(r.node, '/birdview/depth/image_raw', Image)
br = CvBridge(); bgr = br.imgmsg_to_cv2(col,'bgr8'); depth = br.imgmsg_to_cv2(dep,'passthrough').astype(np.float32)
cv2.imwrite('birdview_final.png', bgr)
T = np.eye(4); T[:3,:3] = quat_to_R([0.707,0.707,0,0]); T[:3,3] = [-0.2,0,3.0]
H,W = depth.shape; vv,uu = np.mgrid[0:H,0:W]
pc = np.stack([(uu-320)*depth/579.411,(vv-240)*depth/579.411,depth,np.ones_like(depth)],-1); pw = pc@T.T
zz = pw[...,2]
for name, ysign in [('LEFT plate (world -y)', -1), ('RIGHT plate (world +y)', 1)]:
    reg = (ysign*pw[...,1] > 0.15) & (np.abs(pw[...,0]) < 0.2) & (ysign*pw[...,1] < 0.45)
    plate = reg & (zz>0.44) & (zz<0.47)
    mug = reg & (zz>0.50) & (zz<0.65)
    pp = pw[plate][:,:3]; pm = pw[mug][:,:3]
    pc_ = fit_circle(pp[pp[:,2]>0.449][:,:2]) if len(pp)>20 else None
    mc = fit_circle(pm[pm[:,2]>np.percentile(pm[:,2],50)][:,:2]) if len(pm)>20 else None
    print(name)
    print('   plate rim fit', np.round(pc_,4) if pc_ else 'n/a', 'n', len(pp))
    print('   mug rim fit  ', np.round(mc,4) if mc else 'n/a', 'n', len(pm), 'rim z top', np.percentile(pm[:,2],95).round(4) if len(pm) else '')
    if pc_ and mc: print('   mug-center offset from plate center: %.3f m' % np.hypot(mc[0]-pc_[0], mc[1]-pc_[1]))
" 2>&1 | grep -v Warn

# openrua op 41
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified quirks of the /workspace Franka Panda sim workstation (frames, IK tip offset, camera clipping, controller lag) that the shipped docs get wrong or omit
metadata:
  type: project
---

Verified 2026-09-12 on the /workspace Panda (MoveIt + ROS 2, MuJoCo/robosuite-style sim):

- `/compute_fk` and `/compute_ik` already work in the WORLD frame (model root is `world`), not `panda_link0` as docs/30-action.md claims. Do NOT add the world->panda_link0 offset (-0.51, 0, 0.42).
- IK tip link is `panda_link8`, which is yawed +45° about z from `panda_hand`. Request quat = R_hand @ Rz(+45°) or the fingers end up 45° off. Position/TCP offset (0.1034 along z) unaffected.
- Hand quat (1,0,0,0) = top-down, fingers open along world y; hand x = world x.
- First camera frames at session start show objects ~4 cm above the table (unsettled scene). Re-snapshot after the first motion before trusting heights. Settled: table z=0.427, plates ~2 cm tall.
- Eye-in-hand depth (`robot0_eye_in_hand`) clips at 0.399 m; camera sits at hand-frame (0.05, 0, -0.001), rotated 90° (image right = -hand... = -world y at yaw 0, image down = -world x). Near-range depth is accurate.
- Joint 7 lags trajectories → result code -5 with the position reached; resending the same goal converges (helper `rob.py` in /workspace auto-retries).
- "Left/right" in tasks = left/right in the `agentview` image = world -y / +y.
- Mugs (~9 cm dia) exceed the 8 cm gripper opening: rim grasp (one finger inside, one outside, ~2.5 cm below rim) works; closed gap 6–13 mm = holding.

**Why:** these cost most of the session to discover; the docs are generic and partly wrong for this machine.
**How to apply:** reuse /workspace/rob.py if still present; otherwise apply the frame/yaw facts above before the first IK move.
OPENRUA_EOF

# openrua op 42
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)"); echo "- [Panda workstation quirks](panda-workstation-quirks.md) — world-frame FK/IK, link8 45° yaw, unsettled first frames, eye-in-hand depth clip, joint7 lag" >> MEMORY.md
