#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/*.py tools/perception/*.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -iE "ik|fk|plan|scene" ; echo ---; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -12; echo ---; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | sort | uniq

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && ros2 topic echo /birdview/color/camera_info --once | grep -A3 "^k:" && timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A4 Matrix

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects in the birdview height map; print world centroids."""
import cv2
import numpy as np

depth = np.load("birdview_depth.npy")
color = cv2.imread("birdview.png")
fx = fy = 579.4112549695428
cx, cy = 320.0, 240.0
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
z_cam = depth
X_cam = (uu - cx) * z_cam / fx
Y_cam = (vv - cy) * z_cam / fy
wx = Y_cam - 0.2
wy = X_cam
wz = 3.0 - z_cam

# table height: mode of z in the table region (dark wood)
tbl = wz[(np.abs(wx) < 0.6) & (np.abs(wy) < 0.6)]
hist, edges = np.histogram(tbl[np.isfinite(tbl)], bins=400, range=(0, 2))
table_z = edges[np.argmax(hist)]
print(f"table_z ~ {table_z:.3f}")

mask = (wz > table_z + 0.008) & (np.abs(wx) < 0.7) & (np.abs(wy) < 0.7)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
for i in range(1, n):
    area = stats[i, cv2.CC_STAT_AREA]
    if area < 30:
        continue
    m = lab == i
    zs = wz[m]
    bgr = color[m].mean(axis=0)
    u, v = cents[i]
    print(f"blob {i}: px=({u:.0f},{v:.0f}) area={area} "
          f"world=({wx[m].mean():.3f},{wy[m].mean():.3f}) "
          f"top_z={np.percentile(zs,95):.3f} min_z={zs.min():.3f} "
          f"bgr={bgr.astype(int)} "
          f"bbox_x=({wx[m].min():.3f},{wx[m].max():.3f}) bbox_y=({wy[m].min():.3f},{wy[m].max():.3f})")
OPENRUA_EOF

# openrua op 8
python3 scene.py

# openrua op 9
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); wz=3.0-d
# sample table at a few clean spots (agentview showed empty wood)
for (u,v) in [(320,300),(200,250),(450,250),(320,330),(300,200),(150,300)]:
    print((u,v), round(float(wz[v,u]),4))
tbl=wz[150:350, 150:500].ravel()
h,e=np.histogram(tbl,bins=200,range=(0.3,0.7))
for i in np.argsort(h)[-8:][::-1]: print(round(e[i],3), h[i])
"

# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); color=cv2.imread('birdview.png')
fx=579.4112549695428; H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
wx=(vv-240)*d/fx-0.2; wy=(uu-320)*d/fx; wz=3.0-d
mask=((wz>0.435)&(wz<0.62)&(np.abs(wx)<0.5)&(np.abs(wy)<0.5)).astype(np.uint8)
n,lab,st,ce=cv2.connectedComponentsWithStats(mask,8)
for i in range(1,n):
    if st[i,4]<30: continue
    m=lab==i; zs=wz[m]
    print(f'blob {i}: px=({ce[i][0]:.0f},{ce[i][1]:.0f}) area={st[i,4]} world=({wx[m].mean():.3f},{wy[m].mean():.3f}) top={np.percentile(zs,97):.3f} min={zs.min():.3f} bgr={color[m].mean(axis=0).astype(int)} bx=({wx[m].min():.3f},{wx[m].max():.3f}) by=({wy[m].min():.3f},{wy[m].max():.3f})')
"

# openrua op 11
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -2; timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); color=cv2.imread('birdview.png')
fx=579.4112549695428; H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
wx=(vv-240)*d/fx-0.2; wy=(uu-320)*d/fx; wz=3.0-d
print('table', wz[300,320])
mask=((wz>0.432)&(wz<0.62)&(np.abs(wx)<0.5)&(np.abs(wy)<0.5)).astype(np.uint8)
n,lab,st,ce=cv2.connectedComponentsWithStats(mask,8)
for i in range(1,n):
    if st[i,4]<30: continue
    m=lab==i; zs=wz[m]
    print(f'blob {i}: px=({ce[i][0]:.0f},{ce[i][1]:.0f}) area={st[i,4]} world=({wx[m].mean():.3f},{wy[m].mean():.3f}) top={np.percentile(zs,97):.3f} min={zs.min():.3f} bgr={color[m].mean(axis=0).astype(int)} bx=({wx[m].min():.3f},{wx[m].max():.3f}) by=({wy[m].min():.3f},{wy[m].max():.3f})')
"

# openrua op 12
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854 1.0; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
wx=(vv-240)*d/fx-0.2; wy=(uu-320)*d/fx; wz=3.0-d
mask=((wz>0.432)&(wz<0.62)&(np.abs(wx)<0.5)&(np.abs(wy)<0.5)).astype(np.uint8)
n,lab,st,ce=cv2.connectedComponentsWithStats(mask,8)
for i in range(1,n):
    if st[i,4]<30: continue
    m=lab==i; zs=wz[m]
    print(f'blob {i}: area={st[i,4]} world=({wx[m].mean():.3f},{wy[m].mean():.3f}) top={np.percentile(zs,97):.3f} min={zs.min():.3f} bx=({wx[m].min():.3f},{wx[m].max():.3f}) by=({wy[m].min():.3f},{wy[m].max():.3f})')
"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 13
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
wx=(vv-240)*d/fx-0.2; wy=(uu-320)*d/fx; wz=3.0-d
px=d/fx  # metres per pixel at that depth
mask=((wz>0.432)&(wz<0.62)&(np.abs(wx)<0.5)&(np.abs(wy)<0.5)).astype(np.uint8)
n,lab,st,ce=cv2.connectedComponentsWithStats(mask,8)
names={2:'yellow',3:'white',5:'red'}
for i,nm in names.items():
    m=lab==i
    print(nm, 'm/px', round(float(px[m].mean()),4))
    rows=np.where(m.any(axis=1))[0]
    for r in rows:
        cols=np.where(m[r])[0]
        print(f'  row {r} wx={wx[r,cols[0]]:.3f}: cols {cols.min()}-{cols.max()} width={ (cols.max()-cols.min()+1)*px[r,cols[0]]:.3f} wy=({wy[r,cols.min()]:.3f},{wy[r,cols.max()]:.3f}) ztop={wz[r,cols].max():.3f}')
"

# openrua op 14
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show control_msgs/action/FollowJointTrajectory | grep -B2 -A8 "^---" | head -30

# openrua op 15
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent-client helper for the Panda: FK/IK, trajectories, gripper.

All poses are WORLD frame; converted to panda_link0 for MoveIt.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])
TCP = M["hand"]["tcp_offset_m"]


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
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def down_quat(finger_axis_world):
    """Hand z pointing straight down, hand y (finger axis) along the
    given world direction (projected to horizontal)."""
    y = np.array(finger_axis_world, float)
    y[2] = 0
    y /= np.linalg.norm(y)
    z = np.array([0, 0, -1.0])
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    return R_to_quat(R)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.js = {}
        self.wrench = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self.wrench.update(m=m), 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        for c, n in ((self.fjt, "fjt"), (self.grip, "grip")):
            if not c.wait_for_server(timeout_sec=20):
                raise SystemExit(f"no {n} server")
        for c, n in ((self.ik, "ik"), (self.fk, "fk")):
            if not c.wait_for_service(timeout_sec=20):
                raise SystemExit(f"no {n} service")
        self.spin_until(lambda: "m" in self.js, 10)

    def _on_js(self, m):
        self.js["m"] = m

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while not pred() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- state ----------
    def joints(self):
        self.js.pop("m", None)
        self.spin_until(lambda: "m" in self.js, 10)
        m = self.js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_pose(self, q=None, link="panda_hand"):
        """World-frame (pos, quat) of link for arm config q."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_pose(self, q=None):
        pos, quat = self.fk_pose(q)
        R = quat_to_R(quat)
        return pos + TCP * R[:, 2], quat

    # ---------- IK ----------
    def ik_solve(self, tcp_world, quat, seed=None, tries=3):
        """Joint config placing the TCP at tcp_world (world frame)."""
        R = quat_to_R(quat)
        hand_world = np.array(tcp_world, float) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        seed = self.arm_q() if seed is None else seed
        for t in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_base)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = \
                [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                q = [sol[j] for j in ARM]
                # sanity: FK of solution must match target
                pos, _ = self.tcp_pose(q)
                err = np.linalg.norm(pos - tcp_world)
                if err < 0.005:
                    return q
                print(f"  ik fk-mismatch {err:.4f}, retry", file=sys.stderr)
            else:
                print(f"  ik fail code={res and res.error_code.val}, retry",
                      file=sys.stderr)
            seed = list(np.array(seed) + np.random.uniform(-0.2, 0.2, 7))
        return None

    # ---------- motion ----------
    def move_q(self, q, seconds, verify=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        if verify:
            self.spin(5)
            cur = np.array(self.arm_q())
            err = np.abs(cur - np.array(q)).max()
            print(f"  move done code={code} max_joint_err={err:.4f}")
            return code, err
        return code, None

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.ik_solve(tcp_world, quat, seed)
        if q is None:
            print(f"  IK FAILED for {np.round(tcp_world,3)}")
            return None
        code, err = self.move_q(q, seconds)
        pos, _ = self.tcp_pose()
        print(f"  tcp now {np.round(pos,4)} (target {np.round(tcp_world,4)})")
        return q

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        self.spin(5)
        f = self.finger()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def force(self):
        self.wrench.pop("m", None)
        self.spin_until(lambda: "m" in self.wrench, 5)
        if "m" not in self.wrench:
            return None
        f = self.wrench["m"].wrench.force
        return np.array([f.x, f.y, f.z])
OPENRUA_EOF

# openrua op 16
timeout 300 python3 -c "
from arm import *
a=Arm()
print('q', np.round(a.arm_q(),4))
pos,quat=a.fk_pose(); print('hand world', np.round(pos,4), 'quat', np.round(quat,4))
R=quat_to_R(quat); print('hand x', np.round(R[:,0],3), 'y', np.round(R[:,1],3), 'z', np.round(R[:,2],3))
tp,_=a.tcp_pose(); print('tcp world', np.round(tp,4))
print('finger', a.finger())
print('force', a.force())
print('down_quat x-fingers', np.round(down_quat([1,0,0]),4), 'y-fingers', np.round(down_quat([0,1,0]),4))
"

# openrua op 17
timeout 120 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
" && grep -n -B2 -A6 "<joint name=\"world_joint\|<joint name=\"panda_joint1\"\|<joint name=\"panda_hand_joint\|<joint name=\"virtual\|type=\"fixed\"" robot.urdf | head -80; grep -n "<link name" robot.urdf | head -30

# openrua op 18
python3 -c "
import numpy as np
def rot(rpy):
    r,p,y=rpy
    Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz@Ry@Rx
def T(xyz,rpy):
    M=np.eye(4); M[:3,:3]=rot(rpy); M[:3,3]=xyz; return M
def Rz(t): return T([0,0,0],[0,0,t])
J=[([0,0,0.333],[0,0,0]),([0,0,0],[-np.pi/2,0,0]),([0,-0.316,0],[np.pi/2,0,0]),([0.0825,0,0],[np.pi/2,0,0]),([-0.0825,0.384,0],[-np.pi/2,0,0]),([0,0,0],[np.pi/2,0,0]),([0.088,0,0],[np.pi/2,0,0])]
def fk(q):
    M=np.eye(4)
    for (xyz,rpy),qi in zip(J,q): M=M@T(xyz,rpy)@Rz(qi)
    M=M@T([0,0,0.107],[0,0,0])@T([0,0,0],[0,0,-np.pi/4])
    return M
q=[0,-0.161,0,-2.4446,0,2.2268,0.7854]
M=fk(q); print('hand base', np.round(M[:3,3],4)); print(np.round(M[:3,:3],3))
print('tcp base', np.round(M[:3,3]+0.1034*M[:3,2],4))
"


# openrua op 19
timeout 300 python3 -c "
from arm import *
import rclpy
a=Arm()
q=a.arm_q()
for fid in ['', 'world', 'panda_link0']:
    req=GetPositionFK.Request(); req.header.frame_id=fid; req.fk_link_names=['panda_hand','panda_link0']
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
    fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30); r=fut.result()
    print(repr(fid), r.error_code.val, [(ps.header.frame_id, round(ps.pose.position.x,3), round(ps.pose.position.y,3), round(ps.pose.position.z,3)) for ps in r.pose_stamped])
"

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])", "new_string": "# MoveIt\'s model frame IS world here (verified: FK of panda_link0 with\\n# empty frame_id returns (-0.51, 0, 0.42)); no offset needed.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
# white mug: center (-0.126,-0.144), rim z 0.549, wall radius ~0.043; grasp -x side wall
tgt=np.array([-0.169,-0.144,0.66])
print('step: above white mug', tgt)
q=a.move_tcp(tgt,q_down,seconds=4.0)
print('q', np.round(q,4) if q is not None else None)
pos,quat=a.tcp_pose(); R=quat_to_R(quat); print('hand z', np.round(R[:,2],3),'hand y', np.round(R[:,1],3))
" 2>&1 | tail -8; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview > /dev/null

# openrua op 22
timeout 120 python3 -c "
from arm import *
a=Arm(); q=a.arm_q(); print('q now', np.round(q,4))
pos,quat=a.tcp_pose(); R=quat_to_R(quat); print('tcp', np.round(pos,4), 'hand y', np.round(R[:,1],3))
"

# openrua op 23
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q=[-0.038,-0.4463,-0.3798,-2.7448,-0.2192,2.3145,-1.8047]
for i in range(3):
    code,err=a.move_q(q,3.0)
    if err<0.01: break
print('q now', np.round(a.arm_q(),4))
pos,quat=a.tcp_pose(); R=quat_to_R(quat); print('tcp', np.round(pos,4), 'hand y', np.round(R[:,1],3), 'hand z', np.round(R[:,2],3))
" 2>&1 | tail -6; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 24
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q=[-0.038,-0.4463,-0.3798,-2.7448,-0.2192,2.3145,-1.8047]
for i in range(4):
    code,err=a.move_q(q,2.0)
    if err<0.005: break
print('q now', np.round(a.arm_q(),4))
pos,quat=a.tcp_pose(); R=quat_to_R(quat); print('tcp', np.round(pos,4), 'hand y', np.round(R[:,1],3), 'hand z', np.round(R[:,2],3))
" 2>&1 | tail -6; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            # group tip is panda_link8 (45 deg off panda_hand); be explicit\\n            req.ik_request.ik_link_name = \\"panda_hand\\"\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace("""                pos, _ = self.tcp_pose(q)
                err = np.linalg.norm(pos - tcp_world)
                if err < 0.005:
                    return q""","""                pos, qq = self.tcp_pose(q)
                err = np.linalg.norm(pos - tcp_world)
                Rerr = np.abs(quat_to_R(qq) - R).max()
                if err < 0.005 and Rerr < 0.02:
                    return q
                print(f"  ik mismatch pos={err:.4f} rot={Rerr:.3f}", file=sys.stderr)
                seed = list(np.array(seed) + np.random.uniform(-0.2, 0.2, 7))
                continue""")
s=s.replace("""    def move_q(self, q, seconds, verify=True):
        goal""","""    def move_q(self, q, seconds, verify=True, tol=0.005, resend=6):
        \"\"\"Send trajectory; resend (controller lag) until joints converge.\"\"\"
        for i in range(resend):
            code, err = self._move_once(q, seconds if i == 0 else 2.0, verify)
            if not verify or err < tol:
                return code, err
        return code, err

    def _move_once(self, q, seconds, verify=True):
        goal""")
open('arm.py','w').write(s)
EOF
grep -n "ik_link_name\|def move_q\|def _move_once\|Rerr" arm.py

# openrua op 27
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
tgt=np.array([-0.169,-0.144,0.66])
q=a.move_tcp(tgt,q_down,seconds=3.0)
pos,quat=a.tcp_pose(); R=quat_to_R(quat); print('tcp', np.round(pos,4), 'hand y', np.round(R[:,1],3), 'hand z', np.round(R[:,2],3))
print('q', np.round(a.arm_q(),4))
" 2>&1 | tail -8; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 28
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
print('force before', np.round(a.force(),2))
for z in (0.585, 0.524):
    q=a.move_tcp(np.array([-0.169,-0.144,z]),q_down,seconds=2.0)
    print('force', np.round(a.force(),2), 'fingers', np.round(a.finger(),4))
" 2>&1 | grep -v "^  move"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 29
timeout 600 python3 -u -c "
from arm import *
a=Arm()
f=a.gripper(0.0)
print('force', np.round(a.force(),2))
" 2>&1 | tail -3

# openrua op 30
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
q=a.move_tcp(np.array([-0.169,-0.144,0.70]),q_down,seconds=3.0)
print('force', np.round(a.force(),2), 'fingers', np.round(a.finger(),4))
" 2>&1 | grep -v "^  move"; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 31
timeout 900 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
# left plate center (0.007,-0.324); mug center = TCP + (0.043,0)
hx,hy=0.007-0.043,-0.324
q=a.move_tcp(np.array([hx,hy,0.70]),q_down,seconds=4.0)
print('force', np.round(a.force(),2), 'fingers', np.round(a.finger(),4))
f0=a.force()
for z in (0.60, 0.575, 0.562):
    q=a.move_tcp(np.array([hx,hy,z]),q_down,seconds=2.0)
    f=a.force(); print('z',z,'force', np.round(f,2), 'dFz', round(float(f[2]-f0[2]),2), 'fingers', np.round(a.finger(),4))
    if f[2]-f0[2] > 2.0: print('contact'); break
" 2>&1 | grep -v "^  move"; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 32
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
a.gripper(0.04)
q=a.move_tcp(np.array([-0.036,-0.324,0.70]),q_down,seconds=3.0)
print('fingers', np.round(a.finger(),4))
" 2>&1 | grep -v "^  move"; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null

# openrua op 33
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); color=cv2.imread('birdview.png')
fx=579.4112549695428; H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
wx=(vv-240)*d/fx-0.2; wy=(uu-320)*d/fx; wz=3.0-d
mask=((wz>0.432)&(wz<0.62)&(np.abs(wx)<0.5)&(np.abs(wy)<0.5)).astype(np.uint8)
n,lab,st,ce=cv2.connectedComponentsWithStats(mask,8)
for i in range(1,n):
    if st[i,4]<30: continue
    m=lab==i; zs=wz[m]
    print(f'blob {i}: area={st[i,4]} world=({wx[m].mean():.3f},{wy[m].mean():.3f}) top={np.percentile(zs,97):.3f} min={zs.min():.3f} bgr={color[m].mean(axis=0).astype(int)} bx=({wx[m].min():.3f},{wx[m].max():.3f}) by=({wy[m].min():.3f},{wy[m].max():.3f})')
    if 0.53<np.percentile(zs,97)<0.56 and wy[m].mean()>-0.1:
        rows=np.where(m.any(axis=1))[0]
        for r in rows:
            cols=np.where(m[r])[0]
            print(f'   row wx={wx[r,cols[0]]:.3f}: wy=({wy[r,cols.min()]:.3f},{wy[r,cols.max()]:.3f}) w={(cols.max()-cols.min()+1)*d[r,cols[0]]/fx:.3f}')
"

# openrua op 34
timeout 900 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
q=a.move_tcp(np.array([-0.242,0.019,0.70]),q_down,seconds=4.0)
pos,quat=a.tcp_pose(); R=quat_to_R(quat); print('hand y', np.round(R[:,1],3), 'hand z', np.round(R[:,2],3))
" 2>&1 | grep -v "^  move"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 35
timeout 900 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
f0=a.force()
for z in (0.58, 0.516):
    q=a.move_tcp(np.array([-0.242,0.019,z]),q_down,seconds=2.0)
    f=a.force(); print('z',z,'dF', np.round(f-f0,2))
f=a.gripper(0.0)
print('force', np.round(a.force(),2))
if f[0] > 0.002:
    q=a.move_tcp(np.array([-0.242,0.019,0.76]),q_down,seconds=3.0)
    print('lifted; fingers', np.round(a.finger(),4), 'force', np.round(a.force(),2))
else:
    print('GRASP FAILED (fingers closed on air)')
" 2>&1 | grep -v "^  move"; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 36
timeout 900 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
hx,hy=-0.012-0.045,0.275
q=a.move_tcp(np.array([hx,hy,0.76]),q_down,seconds=4.0)
print('fingers', np.round(a.finger(),4))
f0=a.force()
for z in (0.62, 0.575, 0.555):
    q=a.move_tcp(np.array([hx,hy,z]),q_down,seconds=2.0)
    f=a.force(); print('z',z,'dFz', round(float(f[2]-f0[2]),2), 'fingers', np.round(a.finger(),4))
    if f[2]-f0[2] > 2.0: print('contact'); break
" 2>&1 | grep -v "^  move"; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 37
timeout 900 python3 -u -c "
from arm import *
a=Arm()
q_down=down_quat([1,0,0])
a.gripper(0.04)
q=a.move_tcp(np.array([-0.057,0.275,0.72]),q_down,seconds=3.0)
# park the arm back near home, away from both plates, so cameras see everything
q=a.move_tcp(np.array([-0.25,0.0,0.75]),q_down,seconds=4.0)
print('fingers', np.round(a.finger(),4))
" 2>&1 | grep -v "^  move"; for c in agentview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null

# openrua op 38
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); color=cv2.imread('birdview.png')
fx=579.4112549695428; H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
wx=(vv-240)*d/fx-0.2; wy=(uu-320)*d/fx; wz=3.0-d
def blobs(lo,hi,label):
    mask=((wz>lo)&(wz<hi)&(np.abs(wx)<0.5)&(np.abs(wy)<0.5)).astype(np.uint8)
    n,lab,st,ce=cv2.connectedComponentsWithStats(mask,8)
    for i in range(1,n):
        if st[i,4]<100: continue
        m=lab==i
        print(f'{label} blob: world=({wx[m].mean():.3f},{wy[m].mean():.3f}) top={np.percentile(wz[m],97):.3f} min={wz[m].min():.3f} bx=({wx[m].min():.3f},{wx[m].max():.3f}) by=({wy[m].min():.3f},{wy[m].max():.3f}) area={st[i,4]}')
print('--- plate-height layer (0.435..0.47): plates + anything on them')
blobs(0.435,0.47,'plate')
print('--- mug rim layer (>0.50)')
blobs(0.50,0.62,'mug')
"
