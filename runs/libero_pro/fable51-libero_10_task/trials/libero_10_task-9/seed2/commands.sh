#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | head -60 && echo ---NODES--- && ros2 node list

# openrua op 3
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 4
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 5
for p in "240 270" "316 293" "425 300" "300 380" "350 250" "365 250"; do echo -n "px $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
print(d.shape, np.nanmin(d), np.nanmax(d))
# print a coarse height map of region around microwave: rows 230..360, cols 300..470 step 5
# camera looks down; depth ~ distance; table depth?
tbl=d[380,300]; print('table depth',tbl)
for v in range(235,360,5):
    print(f'{v:3d} '+''.join(('#' if tbl-d[v,u]>0.15 else ('+' if tbl-d[v,u]>0.03 else '.')) for u in range(300,470,3)))
"

# openrua op 7
for p in "400 255" "452 255" "400 345" "452 345" "395 300" "325 262" "390 247" "360 254" "310 255" "245 245" "410 290"; do echo -n "px $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
tbl=2.10
for v in range(250,295,2):
    print(f'{v:3d} '+''.join(('#' if tbl-d[v,u]>0.12 else ('+' if tbl-d[v,u]>0.03 else '.')) for u in range(215,275)))
# rim pixels
ys,xs=np.where(tbl-d[245:300,215:275]>0.12)
print('rim centroid px', xs.mean()+215, ys.mean()+245, 'n',len(xs))
print('rim height', (tbl-d[245:300,215:275]).max())
"

# openrua op 9
for p in "228 268" "240 258" "240 270" "258 268" "310 293" "316 293"; do echo -n "px $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 10
for p in "150 190" "150 235" "150 250" "120 300"; do echo -n "agentview px $p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p; done; for p in "322 210" "322 235" "300 300"; do echo -n "sideview px $p -> "; timeout 60 python3 tools/perception/px2world.py sideview $p; done

# openrua op 11
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png >/dev/null; python3 -c "
import numpy as np
d=np.load('sideview_depth.npy')
print('sideview col 322 rows 195..265:')
for v in range(195,266,3): print(v, round(float(d[v,322]),3), round(float(d[v,300]),3), round(float(d[v,340]),3))
"

# openrua op 12
for p in "322 208" "322 248" "322 230"; do echo -n "sideview px $p -> "; timeout 60 python3 tools/perception/px2world.py sideview $p; done; ros2 topic echo /sideview/color/camera_info --once | grep -A3 "^k:"

# openrua op 13
timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 "^position"

# openrua op 14
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; python3 -c "
import numpy as np
d=np.load('sideview_depth.npy')
print('sideview col 322:')
for v in range(200,275,3): print(v, round(float(d[v,322]),3))
"; for p in "322 248" "322 262"; do echo -n "sideview px $p -> "; timeout 60 python3 tools/perception/px2world.py sideview $p; done

# openrua op 15
for p in "322 234"; do echo -n "sideview px $p -> "; timeout 60 python3 tools/perception/px2world.py sideview $p; done; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
tbl=2.10
for v in range(250,300,2):
    print(f'{v:3d} '+''.join(('#' if tbl-d[v,u]>0.06 else ('+' if tbl-d[v,u]>0.02 else '.')) for u in range(215,275)))
reg=tbl-d[245:305,215:275]
ys,xs=np.where(reg>0.06)
print('rim centroid px', xs.mean()+215, ys.mean()+245, 'n',len(xs), 'max h', reg.max())
reg2=tbl-d[270:320,290:345]
ys,xs=np.where(reg2>0.06)
print('yellow rim centroid px', xs.mean()+290, ys.mean()+270, 'n',len(xs), 'max h', reg2.max())
"

# openrua op 16
mkdir -p "$(dirname /workspace/px_at_z.py)"
cat > /workspace/px_at_z.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Intersect a camera pixel ray with a world z-plane.

Usage: python3 px_at_z.py <camera> <z> <u> <v> [<u> <v> ...]
Prints "u v -> x y z" for each pixel. Uses intrinsics + TF only (no depth),
so it is exact for points known to lie at height z (e.g. a mug rim).
"""
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam, zp = sys.argv[1], float(sys.argv[2])
    pix = [(int(sys.argv[i]), int(sys.argv[i + 1])) for i in range(3, len(sys.argv), 2)]
    rclpy.init()
    node = rclpy.create_node("px_at_z")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    got = {}
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                             lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    info = got["m"]
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    o = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    for u, v in pix:
        d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d = R @ d_cam
        s = (zp - o[2]) / d[2]
        p = o + s * d
        print(f"{u} {v} -> {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 17
echo "white mug rim (z=1.0): center, left edge, right edge, top, bottom:"; timeout 60 python3 px_at_z.py birdview 1.0 241 267 227 267 255 267 241 254 241 280 265 268; echo "yellow mug rim center (z=1.0):"; timeout 60 python3 px_at_z.py birdview 1.0 316 293; echo "microwave top corners (z=1.107):"; timeout 60 python3 px_at_z.py birdview 1.107 399 251 455 251 399 349 455 349; echo "door top line (z=1.107):"; timeout 60 python3 px_at_z.py birdview 1.107 322 262 392 246

# openrua op 18
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png >/dev/null; cat > cloud.py <<'EOF'
#!/usr/bin/env python3
"""Project a camera's depth image to world points; save as npy (H,W,3)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
sys.path.insert(0, '/workspace')
from px_at_z import quat_to_R
cam = sys.argv[1]
d = np.load(f'{cam}_depth.npy')
rclpy.init(); node = rclpy.create_node('cloud')
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(CameraInfo, f'/{cam}/color/camera_info', lambda m: got.setdefault('m', m), 1)
while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
info = got['m']; frame = f'{cam}_optical_frame'
while not tfbuf.can_transform('world', frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform('world', frame, rclpy.time.Time()); q = t.transform.rotation
R = quat_to_R(q.x, q.y, q.z, q.w); o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = d.shape; v, u = np.mgrid[0:H, 0:W]
pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
pw = pc @ R.T + o
np.save(f'{cam}_cloud.npy', pw); print('cam origin', o, 'saved', pw.shape)
rclpy.shutdown()
EOF
timeout 60 python3 cloud.py frontview && python3 -c "
import numpy as np
P=np.load('frontview_cloud.npy')
# points inside the microwave footprint region
m=(P[...,0]>-0.17)&(P[...,0]<0.17)&(P[...,1]>0.2)&(P[...,1]<0.5)&(P[...,2]>0.9)&(P[...,2]<1.12)
pts=P[m]; print('n',len(pts))
# histogram of y among points with x in [-0.1,0.1]
sel=pts[(abs(pts[:,0])<0.12)]
h,e=np.histogram(sel[:,1],bins=np.arange(0.2,0.5,0.01)); print('y hist'); [print(f'{a:.2f} {c}') for a,c in zip(e,h)]
# points that are inside cavity y>0.28
cav=sel[sel[:,1]>0.29]
print('cavity pts', len(cav)); 
if len(cav): 
  h,e=np.histogram(cav[:,2],bins=np.arange(0.9,1.12,0.01)); print('z hist inside'); [print(f'{a:.2f} {c}') for a,c in zip(e,h)]
  h,e=np.histogram(cav[:,0],bins=np.arange(-0.17,0.17,0.02)); print('x hist inside'); [print(f'{a:.2f} {c}') for a,c in zip(e,h)]
"

# openrua op 19
timeout 60 python3 tools/perception/cam_snap.py galleryview; timeout 60 python3 tools/perception/cam_snap.py paperview

# openrua op 20
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm helper library: joint state, FK, IK, trajectory, gripper.

import arm; a = arm.Arm(); a.move_pose(x,y,z, R=..., seconds=3)
World frame here == panda_link0 frame shifted by BASE (machine facts).
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 translation (TF)
TCP = M["hand"]["tcp_offset_m"]


def R_to_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


def quat_to_R(q):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(q).as_matrix()


def R_from_axes(z, x):
    """Rotation whose hand +Z (approach) is z and hand +X is x (both world)."""
    z = np.asarray(z, float); z /= np.linalg.norm(z)
    x = np.asarray(x, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


# Hand pointing straight down; fingers open along world Y (as at start).
R_DOWN_FY = R_from_axes([0, 0, -1], [1, 0, 0])
# Hand pointing straight down; fingers open along world X.
R_DOWN_FX = R_from_axes([0, 0, -1], [0, 1, 0])


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
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
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        """dict name->position (fresh)."""
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.2)
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---------- kinematics ----------
    def fk_hand(self, q=None):
        """Hand pose in WORLD: (pos, R)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk_hand(q)
        return pos + TCP * R[:, 2], R

    def solve_ik(self, pos_world, R, seed=None, attempts=3):
        """Hand pose (world) -> arm joint list, or None."""
        p = np.asarray(pos_world, float) - BASE
        q = R_to_quat(R)
        seed = self.arm_q() if seed is None else seed
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            ps = req.ik_request.pose_stamped.pose
            ps.position.x, ps.position.y, ps.position.z = map(float, p)
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = list(map(float, seed))
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            print(f"  IK attempt failed: {res and res.error_code.val}", file=sys.stderr)
        return None

    # ---------- motion ----------
    def move_joints(self, q, seconds=3.0, via=None):
        """Send trajectory (optionally through via points list) and wait."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=list(map(float, w)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.arm_q()) - np.array(q)))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos_world, R, seconds=3.0, seed=None):
        q = self.solve_ik(pos_world, R, seed=seed)
        if q is None:
            print("  IK FAILED, no motion"); return None
        code, err = self.move_joints(q, seconds)
        p, _ = self.fk_hand()
        print(f"  hand now at {np.round(p, 4)} (target {np.round(pos_world, 4)})")
        return q

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None):
        """Place the fingertip centre (TCP) at tcp_world with orientation R."""
        hand = np.asarray(tcp_world, float) - TCP * R[:, 2]
        return self.move_pose(hand, R, seconds, seed)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap
OPENRUA_EOF

# openrua op 21
timeout 120 python3 -c "
import arm, numpy as np
a=arm.Arm()
print('q', np.round(a.arm_q(),3))
p,R=a.fk_hand(); print('hand world', np.round(p,4)); print(np.round(R,3))
print('tcp', np.round(a.tcp()[0],4))
print('gap', a.finger_gap(), 'wrench', a.wrench())
print('R_DOWN_FY quat', arm.R_to_quat(arm.R_DOWN_FY))
q=a.solve_ik(p,R); print('ik self', None if q is None else np.round(q,3))
"

# openrua op 22
timeout 200 python3 -c "
import arm, numpy as np
a=arm.Arm()
R=arm.R_DOWN_FY
q0=a.arm_q()
# candidate A: world coords given raw (BASE subtracted inside solve_ik -> so pass p+BASE to get raw world)
for label,p in [('raw=world (-0.203,0,1.27)', np.array([-0.203,0,1.27])+arm.BASE), ('raw=base (0.457,0,0.358)', np.array([0.457,0,0.358])+arm.BASE)]:
    q=a.solve_ik(p,R,seed=q0)
    print(label, '->', None if q is None else np.round(q,3))
    if q is not None:
        pf,_=a.fk_hand(q); print('   FK(raw, +BASE) of that solution:', np.round(pf-arm.BASE,4))
"

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 translation (TF)", "new_string": "# Verified empirically: /compute_fk and /compute_ik with frame_id \\"\\" use the\\n# WORLD frame on this machine (FK of the current q == TF world->panda_hand,\\n# and IK of the world-frame hand pose reproduces the current q).\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
timeout 300 python3 -c "
import arm, numpy as np
a=arm.Arm()
th=np.radians(35)
R=arm.R_from_axes([0,np.cos(th),-np.sin(th)],[1,0,0])
q=a.move_pose([-0.02,-0.12,1.12],R,seconds=3)
print('q', None if q is None else np.round(q,3))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand recon1.png

# openrua op 25
timeout 300 python3 -c "
import arm, numpy as np
a=arm.Arm()
q=[-0.166,0.704,-0.156,-1.621,1.018,1.737,-0.805]
print('before', np.round(a.arm_q(),3))
a.move_joints(q,seconds=4)
print('after', np.round(a.arm_q(),3))
print('hand', np.round(a.fk_hand()[0],4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand recon1.png

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null && timeout 60 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy')
# points inside the microwave footprint
m=(P[...,0]>-0.20)&(P[...,0]<0.20)&(P[...,1]>0.24)&(P[...,1]<0.46)&(P[...,2]>0.85)&(P[...,2]<1.15)
pts=P[m]; print('n',len(pts))
# cavity: points with y>0.30 (behind the front face)
cav=pts[pts[:,1]>0.30]
print('cavity pts',len(cav))
h,e=np.histogram(cav[:,2],bins=np.arange(0.85,1.15,0.01)); print('z hist (cavity y>0.30)'); [print(f'{a:.2f} {c}') for a,c in zip(e,h) if c]
h,e=np.histogram(cav[:,0],bins=np.arange(-0.2,0.2,0.01)); print('x hist'); [print(f'{a:.2f} {c}') for a,c in zip(e,h) if c]
h,e=np.histogram(cav[:,1],bins=np.arange(0.24,0.46,0.01)); print('y hist'); [print(f'{a:.2f} {c}') for a,c in zip(e,h) if c]
# front face points: y in 0.25..0.30
fr=pts[(pts[:,1]>0.25)&(pts[:,1]<0.30)]
h,e=np.histogram(fr[:,2],bins=np.arange(0.85,1.15,0.01)); print('z hist front face'); [print(f'{a:.2f} {c}') for a,c in zip(e,h) if c]
"

# openrua op 27
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('urdf')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
" && grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -A12 'link name="panda_leftfinger"' robot.urdf | head -40; grep -n "mesh filename" robot.urdf | grep -i -E "hand|finger" | head

# openrua op 28
python3 -c "
import numpy as np, struct
def stl_bbox(path):
    data=open(path,'rb').read()
    if data[:5]==b'solid':
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)])
    else:
        n=struct.unpack_from('<I',data,80)[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger']:
    p=f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'
    print(f, [np.round(b,4) for b in stl_bbox(p)])
"; grep -o '<joint name="panda_hand_joint"[^/]*/><origin[^/]*/>' robot.urdf; grep -o 'name="panda_finger_joint1".\{0,400\}' robot.urdf | grep -o '<origin[^/]*/>' | head -2; grep -o 'name="panda_hand_joint".\{0,300\}' robot.urdf | grep -o '<origin[^/]*/>' | head -1

# openrua op 29
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>/dev/null; grep -o 'link name="panda_hand".\{0,600\}' robot.urdf | head -c 800; echo; grep -o 'link name="panda_leftfinger".\{0,500\}' robot.urdf | head -c 600

# openrua op 30
find / -name "hand*.stl" -o -name "hand*.dae" -o -name "finger*.stl" 2>/dev/null | grep -v proc | head

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 cloud.py birdview >/dev/null && timeout 120 python3 -c "
import arm, numpy as np
a=arm.Arm(); p,R=a.fk_hand(); print('hand',np.round(p,4)); print(np.round(R,3))
P=np.load('birdview_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
# points near hand, excluding microwave region
d=P-p
loc=d@R   # coords in hand frame
m=(np.abs(loc[:,0])<0.15)&(np.abs(loc[:,1])<0.15)&(loc[:,2]>-0.02)&(loc[:,2]<0.12)&(P[:,1]<0.24)
L=loc[m]; print('n',len(L))
for name,i in [('hand x',0),('hand y (finger axis)',1),('hand z (approach)',2)]:
    print(name, 'min',round(L[:,i].min(),3),'max',round(L[:,i].max(),3))
# extents of the hand body (z<0.058) vs fingers (z>0.058) along y
for lo,hi in [(-0.02,0.058),(0.058,0.12)]:
    s=L[(L[:,2]>lo)&(L[:,2]<hi)]
    if len(s): print(f'z in [{lo},{hi}]: y range',round(s[:,1].min(),3),round(s[:,1].max(),3),' x range',round(s[:,0].min(),3),round(s[:,0].max(),3), 'n',len(s))
"

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png >/dev/null && timeout 60 python3 cloud.py sideview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png >/dev/null && timeout 60 python3 cloud.py agentview >/dev/null && python3 -c "
import numpy as np
for cam in ['sideview','agentview','birdview']:
    P=np.load(f'{cam}_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.20)&(P[:,0]<-0.02)&(P[:,1]>-0.36)&(P[:,1]<-0.17)&(P[:,2]>0.895)&(P[:,2]<1.02)
    Q=P[m]; print(cam,'mug pts',len(Q))
    for z0 in np.arange(0.90,1.01,0.01):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(s)>5:
            body=s[s[:,1]<-0.22]
            print(f'  z {z0:.2f}: x [{s[:,0].min():.3f},{s[:,0].max():.3f}] w={s[:,0].max()-s[:,0].min():.3f}  y [{s[:,1].min():.3f},{s[:,1].max():.3f}]  n={len(s)}')
"

# openrua op 33
timeout 400 python3 -c "
import arm, numpy as np
a=arm.Arm()
R=arm.R_DOWN_FX
print('R_DOWN_FX', np.round(R,2), 'quat', np.round(arm.R_to_quat(R),3))
q=a.move_tcp([-0.107,-0.196,1.06],R,seconds=4)
print('q', np.round(q,3))
tcp,Rn=a.tcp(); print('tcp', np.round(tcp,4)); print(np.round(Rn,2))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand pregrasp.png

# openrua op 34
timeout 400 python3 -c "
import arm, numpy as np
a=arm.Arm()
q=[-0.113,0.37,-0.231,-2.081,0.129,2.439,-1.999]
for i in range(2):
    print('cur', np.round(a.arm_q(),3))
    code,err=a.move_joints(q,seconds=3)
    if err<0.01: break
tcp,Rn=a.tcp(); print('tcp', np.round(tcp,4)); print(np.round(Rn,2))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand pregrasp.png

# openrua op 35
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace('''            req.ik_request.pose_stamped.header.frame_id = ""
            ps = req.ik_request.pose_stamped.pose''','''            req.ik_request.pose_stamped.header.frame_id = ""
            # the group tip is panda_link8 (45 deg yawed from the hand);
            # ask for the hand link explicitly so R means the hand frame
            req.ik_request.ik_link_name = "panda_hand"
            ps = req.ik_request.pose_stamped.pose''')
open('arm.py','w').write(s)
EOF
timeout 300 python3 -c "
import arm, numpy as np
a=arm.Arm()
p,R=a.fk_hand(); q0=a.arm_q()
q=a.solve_ik(p,R,seed=q0); print('cur q', np.round(q0,3)); print('ik  q', None if q is None else np.round(q,3))
q2=a.solve_ik(p,arm.R_DOWN_FX,seed=q0); print('ik FX', None if q2 is None else np.round(q2,3))
if q2 is not None:
    pf,Rf=a.fk_hand(q2); print('fk of FX sol', np.round(pf,4)); print(np.round(Rf,2))
"

# openrua op 36
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def move_joints(self, q, seconds=3.0, via=None):\\n        \\"\\"\\"Send trajectory (optionally through via points list) and wait.\\"\\"\\"\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move_joints(self, q, seconds=3.0, via=None, tol=0.02, retries=3):\\n        \\"\\"\\"Send trajectory and wait; resend on lag (machine fact: -5 is\\n        usually controller lag, resending converges).\\"\\"\\"\\n        code, err = self._send_traj(q, seconds, via)\\n        for _ in range(retries):\\n            if err <= tol:\\n                break\\n            code, err = self._send_traj(q, max(2.0, seconds * 0.6))\\n        return code, err\\n\\n    def _send_traj(self, q, seconds, via=None):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 37
timeout 500 python3 -c "
import arm, numpy as np
a=arm.Arm()
q=a.move_tcp([-0.107,-0.196,1.06],arm.R_DOWN_FX,seconds=3)
tcp,Rn=a.tcp(); print('tcp', np.round(tcp,4)); print(np.round(Rn,2)); print('q',np.round(a.arm_q(),3))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand pregrasp.png

# openrua op 38
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null && timeout 60 python3 cloud.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy')
# handle: points with z in [0.92,0.99], y > -0.235 (outside the mug wall), near the mug
m=(P[...,2]>0.925)&(P[...,2]<0.995)&(P[...,1]>-0.235)&(P[...,1]<-0.15)&(P[...,0]>-0.16)&(P[...,0]<-0.05)
Q=P[m]; print('handle pts',len(Q))
print('x range',Q[:,0].min().round(3),Q[:,0].max().round(3),'y range',Q[:,1].min().round(3),Q[:,1].max().round(3),'z range',Q[:,2].min().round(3),Q[:,2].max().round(3))
# the outer bar: points with y > -0.205
ob=Q[Q[:,1]>-0.21]; print('outer bar pts',len(ob),'x mean',ob[:,0].mean().round(4),'y mean',ob[:,1].mean().round(4), 'y max', ob[:,1].max().round(4))
for z0 in np.arange(0.93,1.0,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f' z {z0:.2f}: y max {s[:,1].max():.3f} x [{s[:,0].min():.3f},{s[:,0].max():.3f}] n={len(s)}')
# mug rim
m2=(P[...,2]>0.99)&(P[...,2]<1.01)&(P[...,1]<-0.2)&(P[...,1]>-0.35)&(P[...,0]>-0.2)&(P[...,0]<0.0)
r=P[m2]; print('rim pts',len(r),'x',r[:,0].min().round(3),r[:,0].max().round(3),'y',r[:,1].min().round(3),r[:,1].max().round(3))
"

# openrua op 39
timeout 500 python3 -c "
import arm, numpy as np
a=arm.Arm()
print('wrench before', np.round(a.wrench(),2))
q=a.move_tcp([-0.102,-0.196,0.955],arm.R_DOWN_FX,seconds=2.5)
tcp,Rn=a.tcp(); print('tcp', np.round(tcp,4)); print('wrench', np.round(a.wrench(),2))
gap=a.gripper(0.0)
print('gap after close', round(gap,4), 'wrench', np.round(a.wrench(),2))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand grasp.png

# openrua op 40
timeout 500 python3 -c "
import arm, numpy as np
a=arm.Arm()
q=a.move_tcp([-0.102,-0.196,1.10],arm.R_DOWN_FX,seconds=3)
tcp,Rn=a.tcp(); print('tcp', np.round(tcp,4), 'gap', round(a.finger_gap(),4), 'wrench', np.round(a.wrench(),2))
" && timeout 60 python3 tools/perception/cam_snap.py agentview lift.png

# openrua op 41
timeout 60 python3 tools/perception/cam_snap.py frontview lift_front.png; timeout 60 python3 tools/perception/cam_snap.py sideview lift_side.png

# openrua op 42
timeout 600 python3 -c "
import arm, numpy as np
a=arm.Arm()
q=a.arm_q(); print('q', np.round(q,3))
q2=list(q); q2[6]=q[6]+np.pi
print('rotating joint7 to', round(q2[6],3))
a.move_joints(q2,seconds=4)
tcp,R=a.tcp(); print('tcp', np.round(tcp,4)); print(np.round(R,2)); print('gap', round(a.finger_gap(),4))
R180=arm.R_from_axes([0,0,-1],[0,-1,0])
q3=a.move_tcp([0.10,-0.217,1.10],R180,seconds=4)
print('q3', np.round(q3,3), 'gap', round(a.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview stage1.png

# openrua op 43
timeout 300 python3 -c "
import arm, numpy as np
a=arm.Arm()
R180=arm.R_from_axes([0,0,-1],[0,-1,0])
cur=a.arm_q(); print('cur', np.round(cur,3))
hand=np.array([0.10,-0.217,1.10])-arm.TCP*R180[:,2]
for seed in [[0,-0.16,0,-2.44,0,2.23,1.9],[0.3,0.3,-0.3,-1.8,0.2,2.1,1.9],[0.2,0.6,-0.2,-1.5,0.1,2.0,1.9]]:
    q=a.solve_ik(hand,R180,seed=seed)
    if q is not None:
        p,_=a.fk_hand(q); print('sol', np.round(q,3), 'dist', round(float(np.max(np.abs(np.array(q)-np.array(cur)))),3))
    else: print('none for seed', seed)
"

# openrua op 44
timeout 600 python3 -c "
import arm, numpy as np
a=arm.Arm()
R180=arm.R_from_axes([0,0,-1],[0,-1,0])
print('wrench', np.round(a.wrench(),2))
a.move_tcp([0.10,-0.217,1.02],R180,seconds=4)
print('wrench', np.round(a.wrench(),2))
a.move_tcp([0.10,-0.217,0.957],R180,seconds=4)
tcp,_=a.tcp(); print('tcp', np.round(tcp,4), 'wrench', np.round(a.wrench(),2), 'gap', round(a.finger_gap(),4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview stage2.png && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand stage2_eih.png

# openrua op 45
timeout 600 python3 -c "
import arm, numpy as np
a=arm.Arm()
R180=arm.R_from_axes([0,0,-1],[0,-1,0])
a.gripper(0.04)
a.move_tcp([0.10,-0.217,1.12],R180,seconds=3)
print('tcp', np.round(a.tcp()[0],4))
" && timeout 60 python3 tools/perception/cam_snap.py birdview stage3_bird.png && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png > /dev/null && timeout 60 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.0)&(P[:,0]<0.22)&(P[:,1]>-0.32)&(P[:,1]<-0.04)&(P[:,2]>0.92)&(P[:,2]<1.02)
Q=P[m]; print('mug pts',len(Q))
for z0 in np.arange(0.92,1.02,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s)>3: print(f' z {z0:.2f}: x [{s[:,0].min():.3f},{s[:,0].max():.3f}] y [{s[:,1].min():.3f},{s[:,1].max():.3f}] n={len(s)}')
rim=Q[Q[:,2]>0.985]; print('rim center', rim[:,0].mean().round(4), rim[:,1].mean().round(4), 'zmax', Q[:,2].max().round(3))
"

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py agentview stage3.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png > /dev/null && timeout 60 python3 cloud.py agentview >/dev/null && python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.02)&(P[:,0]<0.25)&(P[:,1]>-0.36)&(P[:,1]<-0.04)&(P[:,2]>0.905)&(P[:,2]<1.02)
Q=P[m]; print('mug pts',len(Q))
for z0 in np.arange(0.91,1.02,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s)>3: print(f' z {z0:.2f}: x [{s[:,0].min():.3f},{s[:,0].max():.3f}] y [{s[:,1].min():.3f},{s[:,1].max():.3f}] n={len(s)}')
rim=Q[Q[:,2]>0.99]; print('rim x center', (rim[:,0].min()+rim[:,0].max())/2, 'y center', (rim[:,1].min()+rim[:,1].max())/2, 'zmax', Q[:,2].max().round(3))
"

# openrua op 47
python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.04)&(P[:,0]<0.25)&(P[:,1]>-0.36)&(P[:,1]<-0.20)&(P[:,2]>0.92)&(P[:,2]<0.99)
Q=P[m]; print('handle pts',len(Q)); print('x',Q[:,0].min().round(3),Q[:,0].max().round(3),'mean',Q[:,0].mean().round(4),' y',Q[:,1].min().round(3),Q[:,1].max().round(3),' z',Q[:,2].min().round(3),Q[:,2].max().round(3))
for z0 in np.arange(0.92,0.99,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s)>3: print(f' z {z0:.2f}: x [{s[:,0].min():.3f},{s[:,0].max():.3f}] y min {s[:,1].min():.3f} n={len(s)}')
m=(P[:,0]>0.04)&(P[:,0]<0.25)&(P[:,1]>-0.20)&(P[:,1]<-0.04)&(P[:,2]>0.995)&(P[:,2]<1.02)
r=P[m]; print('rim x',r[:,0].min().round(3),r[:,0].max().round(3),'y',r[:,1].min().round(3),r[:,1].max().round(3))
"

# openrua op 48
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null && timeout 60 python3 cloud.py robot0_eye_in_hand >/dev/null && timeout 120 python3 -c "
import arm, numpy as np
a=arm.Arm(); p,R=a.fk_hand(); tcp=p+arm.TCP*R[:,2]
print('hand',np.round(p,4),'tcp',np.round(tcp,4)); print(np.round(R,2)); print('gap',round(a.finger_gap(),4))
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
loc=(P-p)@R  # hand-frame coords
m=(loc[:,2]>0.05)&(loc[:,2]<0.115)&(np.abs(loc[:,0])<0.05)&(np.abs(loc[:,1])<0.08)
L=loc[m]; print('finger pts',len(L))
for name,i in [('hand x (finger width dir)',0),('hand y (finger axis)',1),('hand z',2)]:
    print(name,'min',L[:,i].min().round(4),'max',L[:,i].max().round(4))
# per finger: y>0 and y<0
for s,f in [('y>0',L[L[:,1]>0]),('y<0',L[L[:,1]<0])]:
    if len(f): print(s,'y range',f[:,1].min().round(4),f[:,1].max().round(4),'x range',f[:,0].min().round(4),f[:,0].max().round(4),'n',len(f))
"

# openrua op 49
python3 -c "
import numpy as np
p=np.array([0.1001,-0.2171,1.2229]); R=np.array([[0,-1,0],[-1,0,0],[0,0,-1]],float)
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
loc=(P-p)@R
m=(loc[:,2]>0.065)&(loc[:,2]<0.115)&(np.abs(loc[:,0])<0.05)&(np.abs(loc[:,1])<0.08)
L=loc[m]; print('finger pts',len(L))
for z0 in np.arange(0.065,0.115,0.005):
    s=L[(L[:,2]>=z0)&(L[:,2]<z0+0.005)]
    if len(s):
        pos=s[s[:,1]>0]; neg=s[s[:,1]<0]
        print(f'z {z0:.3f}: +finger y[{pos[:,1].min():.4f},{pos[:,1].max():.4f}] x[{pos[:,0].min():.4f},{pos[:,0].max():.4f}]  -finger y[{neg[:,1].min():.4f},{neg[:,1].max():.4f}]' if len(pos) and len(neg) else f'z {z0:.3f}: n={len(s)}')
"

# openrua op 50
timeout 120 python3 -c "
import arm, numpy as np
a=arm.Arm()
p,R=a.tcp(); print('tcp',np.round(p,4)); print(np.round(R,2)); print('gap',round(a.finger_gap(),4))
" 2>&1 | grep -v '^\[' | tail -8

# openrua op 51
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
R=np.array([[0,-1,0],[-1,0,0],[0,0,-1.0]])
a.move_tcp([0.1005,-0.218,1.03],R,seconds=3)
a.move_tcp([0.1005,-0.218,0.955],R,seconds=3)
print('tcp',np.round(a.tcp()[0],4))
g=a.gripper(0.0); print('gap after close',round(g,4))
" 2>&1 | grep -v '^\[' | tail -12

# openrua op 52
timeout 600 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
R=np.array([[0,-1,0],[-1,0,0],[0,0,-1.0]])
a.move_tcp([0.1005,-0.218,1.14],R,seconds=3)
print('gap',round(a.finger_gap(),4))
# rotate about z by +90: new hand x=(1,0,0), y=(0,-1,0)
R2=arm.R_DOWN_FY
a.move_tcp([0.0,-0.20,1.14],R2,seconds=5)
print('gap',round(a.finger_gap(),4))
a.move_tcp([-0.125,-0.14,1.14],R2,seconds=3)
p,Rn=a.tcp(); print('tcp',np.round(p,4)); print(np.round(Rn,2)); print('gap',round(a.finger_gap(),4))
" 2>&1 | grep -v '^\[' | tail -14

# openrua op 53
timeout 60 python3 tools/perception/cam_snap.py agentview carry.png 2>&1 | tail -1

# openrua op 54
timeout 600 python3 -u -c "
import arm, numpy as np
a=arm.Arm(); R2=arm.R_DOWN_FY
for z in [1.08,1.03,1.0,0.98,0.965,0.955]:
    a.move_tcp([-0.125,-0.14,z],R2,seconds=3)
print('gap',round(a.finger_gap(),4)); print('tcp',np.round(a.tcp()[0],4))
" 2>&1 | grep -v '^\[' | grep -v traj | tail -3; timeout 60 python3 tools/perception/cam_snap.py agentview stage4.png 2>&1 | tail -1

# openrua op 55
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm(); R2=arm.R_DOWN_FY
a.gripper(0.08)
a.move_tcp([-0.125,-0.14,1.20],R2,seconds=3)
" 2>&1 | grep -v '^\[' | grep gripper; for c in agentview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}_s4.png 2>&1 | tail -1; done

# openrua op 56
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm(); R2=arm.R_DOWN_FY
a.move_tcp([-0.17,-0.14,1.32],R2,seconds=3)
" 2>&1 | grep 'hand now'; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_s4.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1

# openrua op 57
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
# mug: points above table z>0.91 within region x<-0.05, y<-0.05
m=(P[:,2]>0.915)&(P[:,0]<-0.03)&(P[:,0]>-0.35)&(P[:,1]<-0.04)&(P[:,1]>-0.30)
Q=P[m]; print('n',len(Q))
# rim points z>0.99
r=Q[Q[:,2]>0.99]; print('rim x',r[:,0].min().round(4),r[:,0].max().round(4),'y',r[:,1].min().round(4),r[:,1].max().round(4),'ztop',r[:,2].max().round(4))
# slices by z
for z0 in [0.93,0.945,0.96,0.975,0.99]:
    s=Q[(Q[:,2]>z0-0.005)&(Q[:,2]<z0+0.005)]
    if len(s): print(z0,'x',s[:,0].min().round(4),s[:,0].max().round(4),'y',s[:,1].min().round(4),s[:,1].max().round(4),len(s))
"


# openrua op 58
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<1.01)&(P[:,0]<-0.02)&(P[:,0]>-0.35)&(P[:,1]<-0.04)&(P[:,1]>-0.30)
Q=P[m]
# body: points with x<-0.075 ; rim top
body=Q[Q[:,0]<-0.07]
print('body x',body[:,0].min().round(4),body[:,0].max().round(4),'y',body[:,1].min().round(4),body[:,1].max().round(4),'z',body[:,2].min().round(3),body[:,2].max().round(3))
top=body[body[:,2]>0.99]; print('rim center est',((top[:,0].min()+top[:,0].max())/2).round(4),((top[:,1].min()+top[:,1].max())/2).round(4))
# handle: points x>-0.075
h=Q[Q[:,0]>-0.075]
print('handle x',h[:,0].min().round(4),h[:,0].max().round(4),'y',h[:,1].min().round(4),h[:,1].max().round(4),'z',h[:,2].min().round(3),h[:,2].max().round(3))
for x0 in np.arange(-0.075,-0.02,0.005):
    s=h[(h[:,0]>=x0)&(h[:,0]<x0+0.005)]
    if len(s): print(round(x0,3),'y',s[:,1].min().round(4),s[:,1].max().round(4),'z',s[:,2].min().round(3),s[:,2].max().round(3),len(s))
"


# openrua op 59
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<1.01)&(P[:,0]<-0.09)&(P[:,0]>-0.30)&(P[:,1]<-0.06)&(P[:,1]>-0.25)
Q=P[m]
for x0 in np.arange(-0.16,-0.10,0.004):
    s=Q[(Q[:,0]>=x0)&(Q[:,0]<x0+0.004)]
    if len(s): print(round(x0,3),'y',s[:,1].min().round(4),s[:,1].max().round(4),'z',s[:,2].min().round(3),s[:,2].max().round(3),len(s))
print('--- z slices of handle region x>-0.125')
h=Q[Q[:,0]>-0.125]
for z0 in np.arange(0.92,1.0,0.01):
    s=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
    if len(s): print(round(z0,3),'x',s[:,0].min().round(4),s[:,0].max().round(4),'y',s[:,1].min().round(4),s[:,1].max().round(4),len(s))
"


# openrua op 60
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c]); print(np.round(R,3))
q=a.solve_ik(np.array([-0.086,-0.228,0.957])-arm.TCP*R[:,2],R)
print('ik',None if q is None else np.round(q,3))
if q is not None:
    # go via a high waypoint first
    a.move_tcp([-0.086,-0.30,1.15],R,seconds=5)
    a.move_tcp([-0.086,-0.228,0.957],R,seconds=4)
    p,Rn=a.tcp(); print('tcp',np.round(p,4)); print(np.round(Rn,2))
" 2>&1 | grep -v '^\[' | tail -16

# openrua op 61
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_pre.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py agentview agent_pre.png 2>&1 | tail -1

# openrua op 62
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
# handle region: y in [-0.16,-0.12], z in [0.92,1.0], x in [-0.16,-0.09]
h=P[(P[:,1]>-0.17)&(P[:,1]<-0.10)&(P[:,2]>0.92)&(P[:,2]<1.0)&(P[:,0]>-0.16)&(P[:,0]<-0.09)]
print('handle pts',len(h))
for z0 in np.arange(0.925,0.995,0.01):
    s=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
    if len(s):
        xs=np.sort(s[:,0]); print(round(z0,3),'x range',xs.min().round(4),xs.max().round(4),'y',s[:,1].min().round(4),s[:,1].max().round(4),len(s))
# fingers: points close to camera: y < -0.19, z 0.93-0.98
f=P[(P[:,1]>-0.24)&(P[:,1]<-0.19)&(P[:,2]>0.93)&(P[:,2]<0.99)]
print('finger pts',len(f))
for x0 in np.arange(-0.16,-0.02,0.01):
    s=f[(f[:,0]>=x0)&(f[:,0]<x0+0.01)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"


# openrua op 63
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
h=P[(P[:,1]>-0.16)&(P[:,1]<-0.10)&(P[:,2]>0.945)&(P[:,2]<0.968)&(P[:,0]>-0.17)&(P[:,0]<-0.09)]
for x0 in np.arange(-0.17,-0.10,0.002):
    s=h[(h[:,0]>=x0)&(h[:,0]<x0+0.002)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4))
"


# openrua op 64
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
f=P[(P[:,1]>-0.25)&(P[:,1]<-0.19)&(P[:,2]>0.93)&(P[:,2]<0.99)&(P[:,0]<-0.10)]
for x0 in np.arange(-0.16,-0.10,0.002):
    s=f[(f[:,0]>=x0)&(f[:,0]<x0+0.002)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"


# openrua op 65
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
a.move_tcp([-0.0825,-0.228,0.9526],R,seconds=2)
a.move_tcp([-0.0825,-0.175,0.9526],R,seconds=3)
p,Rn=a.tcp(); print('tcp',np.round(p,4))
" 2>&1 | grep -v '^\[' | tail -3; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
f=P[(P[:,1]>-0.20)&(P[:,1]<-0.15)&(P[:,2]>0.93)&(P[:,2]<0.99)&(P[:,0]<-0.10)]
for x0 in np.arange(-0.15,-0.11,0.002):
    s=f[(f[:,0]>=x0)&(f[:,0]<x0+0.002)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
h=P[(P[:,1]>-0.16)&(P[:,1]<-0.10)&(P[:,2]>0.945)&(P[:,2]<0.968)&(P[:,0]>-0.17)&(P[:,0]<-0.09)]
print('handle:')
for x0 in np.arange(-0.16,-0.10,0.002):
    s=h[(h[:,0]>=x0)&(h[:,0]<x0+0.002)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4))
"

# openrua op 66
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
print('wrench before',np.round(a.wrench(),2))
a.move_tcp([-0.0825,-0.175,0.9486],R,seconds=2)
a.move_tcp([-0.0825,-0.155,0.9486],R,seconds=2)
print('wrench mid',np.round(a.wrench(),2))
a.move_tcp([-0.0825,-0.138,0.9486],R,seconds=2)
p,Rn=a.tcp(); print('tcp',np.round(p,4)); print('wrench after',np.round(a.wrench(),2))
" 2>&1 | grep -v '^\[' | grep -v traj | tail -6; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_in.png 2>&1 | tail -1

# openrua op 67
timeout 120 python3 -u -c "
import arm, numpy as np
a=arm.Arm(); a.gripper(0.0); print('wrench',np.round(a.wrench(),2))
" 2>&1 | grep -v '^\[' | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_closed.png 2>&1 | tail -1

# openrua op 68
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
a.move_tcp([-0.0825,-0.138,1.0],R,seconds=3)
a.move_tcp([-0.0825,-0.138,1.08],R,seconds=3)
print('gap',round(a.finger_gap(),4),'wrench',np.round(a.wrench(),2))
" 2>&1 | grep -v '^\[' | grep -v traj | tail -3; timeout 60 python3 tools/perception/cam_snap.py agentview lift2.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py sideview lift2_side.png 2>&1 | tail -1

# openrua op 69
ls /workspace/*_cloud.npy; timeout 60 python3 -c "
import numpy as np
P=np.load('frontview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# cavity interior: points with y>0.27 (inside), z in [0.95,1.06]
c=P[(P[:,1]>0.28)&(P[:,1]<0.45)&(P[:,2]>0.95)&(P[:,2]<1.06)]
print('n',len(c))
for x0 in np.arange(-0.20,0.20,0.01):
    s=c[(c[:,0]>=x0)&(c[:,0]<x0+0.01)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"


# openrua op 70
timeout 60 python3 -c "
import numpy as np
for cam in ['frontview','agentview','birdview']:
    P=np.load(cam+'_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    f=P[(P[:,1]>0.25)&(P[:,1]<0.45)&(P[:,2]>0.93)&(P[:,2]<0.955)]
    print(cam,'floor pts',len(f))
    if len(f):
        for x0 in np.arange(-0.20,0.20,0.02):
            s=f[(f[:,0]>=x0)&(f[:,0]<x0+0.02)]
            if len(s): print(' ',round(x0,3),len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
    # front face plane: points with z in [0.95,1.06] and y in [0.24,0.27]
    ff=P[(P[:,1]>0.20)&(P[:,1]<0.30)&(P[:,2]>0.95)&(P[:,2]<1.07)&(P[:,0]>-0.2)&(P[:,0]<0.2)]
    print(cam,'front-ish pts',len(ff))
    if len(ff):
        for x0 in np.arange(-0.20,0.20,0.02):
            s=ff[(ff[:,0]>=x0)&(ff[:,0]<x0+0.02)]
            if len(s): print(' ',round(x0,3),len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"


# openrua op 71
timeout 600 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
a.move_tcp([0.01,-0.138,1.10],R,seconds=3)
a.move_tcp([0.01,0.0,1.10],R,seconds=3)
a.move_tcp([0.01,0.15,1.10],R,seconds=3)
print('gap',round(a.finger_gap(),4),'tcp',np.round(a.tcp()[0],4))
" 2>&1 | grep -v '^\[' | grep -v traj | tail -4; timeout 60 python3 tools/perception/cam_snap.py agentview transit.png 2>&1 | tail -1

# openrua op 72
timeout 600 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
a.move_tcp([0.01,0.15,1.00],R,seconds=3)
print('w',np.round(a.wrench(),2))
for y in [0.20,0.24,0.27]:
    a.move_tcp([0.01,y,1.00],R,seconds=2)
    print(y,'w',np.round(a.wrench(),2))
print('gap',round(a.finger_gap(),4),'tcp',np.round(a.tcp()[0],4))
" 2>&1 | grep -v '^\[' | grep -v traj | grep -v 'hand now' | tail -6; timeout 60 python3 tools/perception/cam_snap.py agentview insert.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_insert.png 2>&1 | tail -1

# openrua op 73
timeout 60 python3 tools/perception/cam_snap.py frontview ins_front.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py birdview ins_bird.png 2>&1 | tail -1

# openrua op 74
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# mug front region: x in [-0.12,-0.01], y in [0.2,0.30]
m=P[(P[:,0]>-0.13)&(P[:,0]<0.0)&(P[:,1]>0.19)&(P[:,1]<0.32)&(P[:,2]>0.90)&(P[:,2]<1.06)]
print('mug pts',len(m),'zmin',m[:,2].min().round(4),'zmax',m[:,2].max().round(4))
for z0 in np.arange(0.93,0.98,0.005):
    s=m[(m[:,2]>=z0)&(m[:,2]<z0+0.005)]
    if len(s): print(round(z0,3),len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
# floor points visible: z 0.94-0.95, y>0.26
f=P[(P[:,2]>0.935)&(P[:,2]<0.95)&(P[:,1]>0.25)&(P[:,1]<0.45)]
print('floor',len(f), 'z',f[:,2].min().round(4) if len(f) else None, f[:,2].max().round(4) if len(f) else None,'x',f[:,0].min().round(3) if len(f) else None,f[:,0].max().round(3) if len(f) else None)
"


# openrua op 75
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
a.gripper(0.08); print('w',np.round(a.wrench(),2))
a.move_tcp([0.0124,0.22,1.00],R,seconds=2); print('w',np.round(a.wrench(),2))
a.move_tcp([0.0124,0.15,1.02],R,seconds=2); print('w',np.round(a.wrench(),2))
" 2>&1 | grep -v '^\[' | grep -v traj | grep -v 'hand now' | tail -4; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_rel.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py agentview rel.png 2>&1 | tail -1

# openrua op 76
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=P[(P[:,0]>-0.16)&(P[:,0]<0.05)&(P[:,1]>0.15)&(P[:,1]<0.34)&(P[:,2]>0.93)&(P[:,2]<1.06)]
# mug body (exclude handle: choose points with y-min per x)
print('pts',len(m),'z',m[:,2].min().round(3),m[:,2].max().round(3))
for x0 in np.arange(-0.14,0.04,0.01):
    s=m[(m[:,0]>=x0)&(m[:,0]<x0+0.01)]
    if len(s): print(round(x0,3),len(s),'yfront',s[:,1].min().round(4),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"


# openrua op 77
timeout 600 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
a.move_tcp([-0.036,0.15,1.02],R,seconds=3)
a.move_tcp([-0.036,0.183,1.00],R,seconds=2); print('w',np.round(a.wrench(),2))
for y in [0.22,0.25,0.27,0.29]:
    a.move_tcp([-0.036,y,1.00],R,seconds=2); print(y,'w',np.round(a.wrench(),2),'tcp',np.round(a.tcp()[0],4))
" 2>&1 | grep -v '^\[' | grep -v traj | grep -v 'hand now' | tail -6; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_push.png 2>&1 | tail -1

# openrua op 78
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
c,s=np.cos(np.radians(20)),np.sin(np.radians(20))
R=arm.R_from_axes([0,c,-s],[0,s,c])
a.move_tcp([-0.036,0.15,1.02],R,seconds=2)
a.move_tcp([-0.036,0.10,1.06],R,seconds=2)
print('tcp',np.round(a.tcp()[0],4))
" 2>&1 | grep -v '^\[' | grep tcp; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=P[(P[:,0]>-0.16)&(P[:,0]<0.06)&(P[:,1]>0.20)&(P[:,1]<0.42)&(P[:,2]>0.95)&(P[:,2]<1.07)]
print('pts',len(m),'ztop',m[:,2].max().round(3))
for x0 in np.arange(-0.15,0.06,0.01):
    s=m[(m[:,0]>=x0)&(m[:,0]<x0+0.01)]
    if len(s): print(round(x0,3),len(s),'yfront',s[:,1].min().round(4),'ztop',s[:,2].max().round(3))
"

# openrua op 79
timeout 60 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
d=P[(P[:,2]>1.08)&(P[:,0]<-0.09)&(P[:,0]>-0.25)&(P[:,1]<0.27)&(P[:,1]>-0.05)]
print('door top pts',len(d),'z',d[:,2].min().round(3),d[:,2].max().round(3))
for y0 in np.arange(-0.02,0.27,0.02):
    s=d[(d[:,1]>=y0)&(d[:,1]<y0+0.02)]
    if len(s): print(round(y0,3),len(s),'x',s[:,0].min().round(4),s[:,0].max().round(4))
# microwave body top near the corner
b=P[(P[:,2]>1.08)&(P[:,0]<-0.12)&(P[:,0]>-0.20)&(P[:,1]>0.24)&(P[:,1]<0.32)]
print('body corner pts',len(b),'x',b[:,0].min().round(4),'y',b[:,1].min().round(4))
"


# openrua op 80
timeout 60 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
d=P[(P[:,2]>0.95)&(P[:,0]<-0.14)&(P[:,0]>-0.30)&(P[:,1]<0.06)&(P[:,1]>-0.06)]
for x0 in np.arange(-0.30,-0.14,0.01):
    s=d[(d[:,0]>=x0)&(d[:,0]<x0+0.01)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"


# openrua op 81
timeout 60 python3 tools/perception/cam_snap.py birdview bird_now.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py birdview 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
d=P[(P[:,2]>0.95)&(P[:,0]<-0.10)&(P[:,0]>-0.32)&(P[:,1]<0.27)&(P[:,1]>-0.08)]
print('by y:')
for y0 in np.arange(-0.08,0.27,0.02):
    s=d[(d[:,1]>=y0)&(d[:,1]<y0+0.02)]
    if len(s): print(round(y0,3),len(s),'x',s[:,0].min().round(4),s[:,0].max().round(4),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"

# openrua op 82
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
R=arm.R_from_axes([1,0,0],[0,0,1])
a.move_pose([-0.42,0.08,1.10],R,seconds=5)
print('hand',np.round(a.fk_hand()[0],4))
" 2>&1 | grep -v '^\[' | grep hand; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_door.png 2>&1 | tail -1

# openrua op 83
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
d=P[(P[:,2]>0.92)&(P[:,2]<1.12)&(P[:,0]<-0.10)&(P[:,0]>-0.32)&(P[:,1]<0.27)&(P[:,1]>-0.08)]
print('pts',len(d))
for y0 in np.arange(-0.08,0.27,0.02):
    s=d[(d[:,1]>=y0)&(d[:,1]<y0+0.02)]
    if len(s): print(round(y0,3),len(s),'x',s[:,0].min().round(4),s[:,0].max().round(4),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"

# openrua op 84
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
R=arm.R_from_axes([1,0,0],[0,0,1])
q=a.solve_ik([-0.42,0.08,1.10],R)
print('q',np.round(q,3)); print('cur',np.round(a.arm_q(),3))
a.move_joints(q,seconds=6,retries=4)
p,Rn=a.fk_hand(); print('hand',np.round(p,4)); print(np.round(Rn,2))
" 2>&1 | grep -v '^\[' | tail -9; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_door.png 2>&1 | tail -1

# openrua op 85
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
d=P[(P[:,2]>0.92)&(P[:,2]<1.12)&(P[:,0]<-0.10)&(P[:,0]>-0.30)&(P[:,1]<0.27)&(P[:,1]>-0.10)]
print('pts',len(d))
for y0 in np.arange(-0.10,0.27,0.02):
    s=d[(d[:,1]>=y0)&(d[:,1]<y0+0.02)]
    if len(s): print(round(y0,3),len(s),'x',s[:,0].min().round(4),s[:,0].max().round(4),'z',s[:,2].min().round(3),s[:,2].max().round(3))
print('handle-ish (x<-0.21):')
h=d[d[:,0]<-0.21]
for z0 in np.arange(0.92,1.12,0.02):
    s=h[(h[:,2]>=z0)&(h[:,2]<z0+0.02)]
    if len(s): print(round(z0,3),len(s),'x',s[:,0].min().round(4),s[:,0].max().round(4),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"

# openrua op 86
timeout 60 python3 tools/perception/cam_snap.py agentview door_now.png 2>&1 | tail -1

# openrua op 87
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
R=arm.R_from_axes([0.62,0.79,0],[0,0,1])
q=a.solve_ik([-0.45,-0.15,1.03],R)
print('cur',np.round(a.arm_q(),3)); print('q  ',np.round(q,3))
a.move_joints(q,seconds=6,retries=4)
p,Rn=a.fk_hand(); print('hand',np.round(p,4)); print(np.round(Rn,2))
" 2>&1 | grep -v '^\[' | tail -8; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_door2.png 2>&1 | tail -1

# openrua op 88
timeout 120 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
R=arm.R_from_axes([0.62,0.79,0],[0,0,1])
q=a.solve_ik([-0.45,-0.15,1.03],R, seed=a.arm_q())
print('cur',np.round(a.arm_q(),3)); print('q  ',np.round(q,3)); print('w',np.round(a.wrench(),2))
" 2>&1 | grep -v '^\[' | tail -3

# openrua op 89
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
cur=a.arm_q()
v1=list(cur); v1[1]=-1.2; v1[3]=-2.5; v1[5]=1.0
home=[0.0,-0.785,0.0,-2.356,0.0,1.571,0.785]
a.move_joints(home,seconds=10,via=[v1],retries=4)
print('w',np.round(a.wrench(),2)); p,R=a.fk_hand(); print('hand',np.round(p,3))
" 2>&1 | grep -v '^\[' | tail -7; timeout 60 python3 tools/perception/cam_snap.py agentview after_home.png 2>&1 | tail -1

# openrua op 90
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py birdview 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
d=P[(P[:,2]>1.09)&(P[:,2]<1.12)&(P[:,0]<-0.10)&(P[:,0]>-0.45)&(P[:,1]<0.27)&(P[:,1]>-0.10)]
print('door top pts',len(d))
xs=[];ys=[]
for y0 in np.arange(-0.10,0.27,0.02):
    s=d[(d[:,1]>=y0)&(d[:,1]<y0+0.02)]
    if len(s): print(round(y0,3),len(s),'x',s[:,0].min().round(4),s[:,0].max().round(4))
"

# openrua op 91
cat > /workspace/door.py <<'EOF'
import sys, arm, numpy as np
H=np.array([-0.19,0.247]); S=0.17; OFF=0.0145; Z=1.075
def d(th): t=np.radians(th); return np.array([np.cos(t),-np.sin(t)])
def n(th): t=np.radians(th); return np.array([-np.sin(t),-np.cos(t)])
def pose(th, extra=0.0):
    p=H+S*d(th)+(OFF+extra)*n(th)
    R=arm.R_from_axes([0,0,-1],[n(th)[0],n(th)[1],0])
    return np.array([p[0],p[1],Z]),R
a=arm.Arm()
thetas=[float(x) for x in sys.argv[1:]]
first=True
for th in thetas:
    if first:
        p,R=pose(th,0.04); a.move_tcp(p,R,seconds=4); first=False
    p,R=pose(th); a.move_tcp(p,R,seconds=3)
    print(f'theta {th}: tcp {np.round(a.tcp()[0],4)} w {np.round(a.wrench(),2)}')
EOF
timeout 600 python3 -u /workspace/door.py 92 85 78 70 2>&1 | grep -v '^\[' | grep -E 'theta|IK|FAIL'; timeout 60 python3 tools/perception/cam_snap.py agentview door1.png 2>&1 | tail -1

# openrua op 92
timeout 900 python3 -u /workspace/door.py 70 60 50 40 30 20 2>&1 | grep -v '^\[' | grep -E 'theta|IK|FAIL'; timeout 60 python3 tools/perception/cam_snap.py agentview door2.png 2>&1 | tail -1

# openrua op 93
timeout 600 python3 -u /workspace/door.py 20 12 6 2 0 2>&1 | grep -v '^\[' | grep -E 'theta|IK|FAIL'; timeout 60 python3 tools/perception/cam_snap.py agentview door3.png 2>&1 | tail -1

# openrua op 94
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
p,R=a.tcp(); print('tcp',np.round(p,4)); print(np.round(R,2)); print('q',np.round(a.arm_q(),3))
q=a.solve_ik(p+np.array([0,0,0.10])-arm.TCP*R[:,2],R)
print('dq',np.round(np.array(q)-np.array(a.arm_q()),3))
a.move_joints(q,seconds=3)
print('tcp',np.round(a.tcp()[0],4),'w',np.round(a.wrench(),2))
" 2>&1 | grep -v '^\[' | tail -9

# openrua op 95
timeout 300 python3 -u -c "
import arm, numpy as np
a=arm.Arm()
p,R=a.tcp()
q=a.solve_ik(p+np.array([0,0,0.15])-arm.TCP*R[:,2],R)
a.move_joints(q,seconds=3)
cur=a.arm_q(); v1=list(cur); v1[1]=-0.9; v1[3]=-2.0
home=[0.0,-0.785,0.0,-2.356,0.0,1.571,0.785]
a.move_joints(home,seconds=10,via=[v1],retries=4)
print('hand',np.round(a.fk_hand()[0],3),'w',np.round(a.wrench(),2))
" 2>&1 | grep -v '^\[' | tail -3; for c in agentview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c final_$c.png 2>&1 | tail -1; done

# openrua op 96
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png 2>&1 | tail -1; timeout 60 python3 cloud.py birdview 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# anything door-height in front of the microwave face?
front=P[(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,1]<0.235)&(P[:,1]>-0.05)&(P[:,0]>-0.25)&(P[:,0]<0.1)]
print('pts in front of face (excluding table):',len(front))
if len(front):
    print(' x',front[:,0].min().round(3),front[:,0].max().round(3),'y',front[:,1].min().round(3),front[:,1].max().round(3),'z',front[:,2].max().round(3))
# door top strip
d=P[(P[:,2]>1.09)&(P[:,2]<1.12)&(P[:,0]>-0.22)&(P[:,0]<0.10)&(P[:,1]>0.20)&(P[:,1]<0.30)]
for x0 in np.arange(-0.22,0.10,0.04):
    s=d[(d[:,0]>=x0)&(d[:,0]<x0+0.04)]
    if len(s): print(round(x0,3),len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4))
"

# openrua op 97
timeout 60 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
front=P[(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,1]<0.235)&(P[:,1]>-0.05)&(P[:,0]>-0.25)&(P[:,0]<0.1)]
for y0 in np.arange(-0.05,0.235,0.02):
    s=front[(front[:,1]>=y0)&(front[:,1]<y0+0.02)]
    if len(s): print(round(y0,3),len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
"
