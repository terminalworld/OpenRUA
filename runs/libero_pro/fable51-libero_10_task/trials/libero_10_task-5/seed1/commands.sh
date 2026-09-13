#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | head -50 && echo ---NODES--- && ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:240,180:400]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[140:290,100:380]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 7
for p in "328 265" "280 171" "280 191" "240 180" "300 300" "328 258"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8

# openrua op 9
python3 -c "
import numpy as np
d=np.load('birdview.npy')
print(d.shape, np.nanmin(d), np.nanmax(d))
# table depth
print('table depth sample', d[300,300], d[400,500])
# cup region: find pixels around (328,265) with depth < table-0.03
sub=d[230:300,290:370]
tab=d[300,300]
mask=sub<tab-0.03
ys,xs=np.nonzero(mask)
print('cup pixels', mask.sum(), 'x range',xs.min()+290,xs.max()+290,'y range',ys.min()+230,ys.max()+230)
print('cup min depth', sub[mask].min())
# print a coarse mask
for r in range(0,70,2):
    print(''.join('#' if mask[r,c] else '.' for c in range(0,80,1)))
"


# openrua op 10
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && ls *.npy && ros2 topic echo /birdview/color/camera_info --once 2>/dev/null | grep -A10 '^k:'

# openrua op 11
cat > /workspace/analyze.py <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
# camera at (-0.2, 0, 3.0), image right=+y, image down=+x
def px2w(u,v,z=None):
    if z is None: z=d[v,u]
    X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
print(d.shape)
tab=d[300,300]; print('table z', 3.0-tab)
# cup
sub=d[230:300,290:370]
mask=sub<tab-0.03
ys,xs=np.nonzero(mask)
print('cup px x',xs.min()+290,xs.max()+290,'y',ys.min()+230,ys.max()+230, 'top z', 3.0-sub[mask].min())
for r in range(0,70,2):
    print(''.join('#' if mask[r,c] else '.' for c in range(0,80,1)))
# Cup rim: pixels with depth near the min (rim top)
rim = sub < sub[mask].min()+0.01
ys,xs=np.nonzero(rim)
u=xs.mean()+290; v=ys.mean()+230
print('rim center px',u,v,'world',px2w(int(u),int(v),sub[mask].min()))
print('rim px extents', xs.min()+290, xs.max()+290, ys.min()+230, ys.max()+230)
EOF
python3 analyze.py

# openrua op 12
cat > /workspace/analyze2.py <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def px2w(u,v,z):
    X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
tab=d[300,300]
y0,y1,x0,x1=248,282,300,352
sub=d[y0:y1,x0:x1]
mask=sub<tab-0.03
zmin=sub[mask].min(); print('cup top z', 3.0-zmin)
# body (exclude handle): depth near min
body = sub < zmin+0.008
ys,xs=np.nonzero(body)
u=xs.mean()+x0; v=ys.mean()+y0
print('body px center',u,v,'extent x',xs.min()+x0,xs.max()+x0,'y',ys.min()+y0,ys.max()+y0)
print('cup center world', px2w(u,v,zmin))
diam_px = xs.max()-xs.min()+1
print('diam m', diam_px*zmin/fx)
for r in range(0,y1-y0):
    print(''.join('#' if body[r,c] else ('+' if mask[r,c] else '.') for c in range(0,x1-x0)))
EOF
python3 analyze2.py

# openrua op 13
timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | grep -A1 Translation | head -3; ros2 topic echo /agentview/color/camera_info --once 2>/dev/null | grep -A3 '^k:'

# openrua op 14
cat > /workspace/analyze3.py <<'EOF'
import numpy as np
d=np.load('agentview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
q=np.array([0.638,0.638,-0.305,-0.305]); q/=np.linalg.norm(q)
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([0.459,0.0,1.610])
def px2w(u,v):
    Z=d[v,u]; p=np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z]); return R@p+t
# scan the cup column at u=340 in the agentview image
for v in range(270,360,4):
    print(v, d[v,340].round(3), px2w(340,v).round(3))
print('caddy back wall top?')
for v in range(150,260,5):
    print(v, d[v,240].round(3), px2w(240,v).round(3))
EOF
python3 analyze3.py

# openrua op 15
cat > /workspace/analyze4.py <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; return np.array([-0.2+(v-cy)*z/fy, (u-cx)*z/fx, 3.0-z])
# caddy region approx px x 210-330, y 150-215 ; print height map (z above table) coarse
print('cols(u):', list(range(205,335,5)))
for v in range(150,220,3):
    row=''
    for u in range(205,335,3):
        h=3.0-d[v,u]-0.88
        row+= ' ' if h<0.01 else ('.' if h<0.03 else ('o' if h<0.09 else ('O' if h<0.14 else '#')))
    print(v, row)
# Print world coords of some key points
for (u,v) in [(210,155),(330,155),(210,215),(330,215)]:
    print((u,v), w(u,v).round(3))
EOF
python3 analyze4.py

# openrua op 16
cat > /workspace/analyze5.py <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; return np.array([-0.2+(v-cy)*z/fy, (u-cx)*z/fx, 3.0-z])
print('column u=268 (middle column), v -> world x, z')
for v in range(156,206):
    p=w(268,v); print(v, p[0].round(3), p[2].round(3))
print('row v=175 (back compartment), u -> world y, z')
for u in range(205,300):
    p=w(u,175); print(u, p[1].round(3), p[2].round(3))
EOF
python3 analyze5.py

# openrua op 17
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[260:370,270:400]; cv2.imwrite('cup_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); crop=im[230:340,290:510]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 18
ls / /home 2>/dev/null; find / -iname "*.bddl" -o -iname "*caddy*" 2>/dev/null | grep -v proc | head -20; ps aux | grep -v grep | grep -iE "python|ros|sim" | head -20

# openrua op 19
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK/IK, trajectory,
gripper, and wrench reads, all in one rclpy node.

World <-> base: base (panda_link0) sits at world (-0.75, 0, 0.912),
identity rotation.  IK/FK run in the base frame (frame_id empty).
"""
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

BASE = np.array([-0.75, 0.0, 0.912])
M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Rob:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing -------------------------------------------------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 15:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        self._wr.pop("m", None)
        t0 = time.time()
        while "m" not in self._wr and time.time() - t0 < 10:
            self.spin(0.2)
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def hand_pose_world(self, q=None):
        """FK: hand pose in WORLD (pos, quat xyzw)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w])
        return pos, quat

    # ---- planning ------------------------------------------------------
    def ik_world(self, pos_w, quat, seed=None, at_tcp=False, tries=1):
        """IK for hand (or TCP) pose given in WORLD. Returns joint list or None."""
        pos_w = np.array(pos_w, dtype=float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos_w = pos_w - TCP * R[:, 2]
        pos_b = pos_w - BASE
        if seed is None:
            seed = self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_b)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            print(f"  IK fail code={res and res.error_code.val}", file=sys.stderr)
        return None

    # ---- acting --------------------------------------------------------
    def move_q(self, q, seconds=3.0, via=None):
        """Send a trajectory (optionally through `via` points) and wait."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        qn = self.arm_q()
        err = float(np.max(np.abs(np.array(qn) - np.array(q))))
        print(f"  traj code={code} max_joint_err={err:.4f}")
        return code, err

    def move_world(self, pos_w, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos_w, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"no IK for {pos_w}")
        code, err = self.move_q(q, seconds)
        pos, qt = self.hand_pose_world()
        print(f"  hand now world={pos.round(4)} quat={qt.round(3)}")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
OPENRUA_EOF

# openrua op 20
timeout 300 python3 -u -c "
from rob import *
r=Rob()
print('q', np.round(r.arm_q(),3))
print('fingers', r.fingers())
print('wrench', r.wrench())
pos,quat=r.hand_pose_world(); print('hand world', pos.round(4), quat.round(3))
# test IK for pregrasp above cup, fingers along x
q=r.ik_world([-0.120+0.049, 0.035, 1.22],[0.707,0.707,0,0])
print('IK pregrasp', q and np.round(q,3))
q2=r.ik_world([-0.120+0.049, 0.035, 1.22],[-0.707,0.707,0,0])
print('IK pregrasp alt', q2 and np.round(q2,3))
"

# openrua op 21
timeout 300 python3 -u -c "
from rob import *
import rob
r=Rob()
req = GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=r.arm_q()
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for ps in fut.result().pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK with world coords directly (BASE=0)
rob.BASE[:]=0
q=r.ik_world([-0.120+0.049, 0.035, 1.22],[0.707,0.707,0,0])
print('IK world-coords', q and np.round(q,3))
if q: print('FK of solution', r.hand_pose_world(q))
"

# openrua op 22
sed -i 's/^BASE = np.array(\[-0.75, 0.0, 0.912\])/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK model frame on this machine is world/' rob.py && timeout 600 python3 -u -c "
from rob import *
r=Rob()
q0=r.arm_q()
for quat in ([1,0,0,0],[0.707,0.707,0,0],[0,1,0,0],[0.924,0.383,0,0]):
    q=r.ik_world([-0.071,0.035,1.22],quat,seed=q0)
    if q is None: print(quat,'-> none'); continue
    pos,qt=r.hand_pose_world(q)
    print('req',quat,'-> j7=%.3f'%q[6],'FK quat',qt.round(3),'pos',pos.round(3))
"

# openrua op 23
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('''def quat_to_R(x, y, z, w):''','''def qmul(a, b):
    """Hamilton product of quaternions given as [x, y, z, w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


# IK on this machine solves for a tip link rotated -45 deg about hand z
# relative to panda_hand (link8 vs hand); request = hand (x) Rz(+45deg).
QZ45 = np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])


def quat_to_R(x, y, z, w):''')
s=s.replace('''        pos_b = pos_w - BASE
        if seed is None:''','''        pos_b = pos_w - BASE
        quat = qmul(quat, QZ45)  # hand quat -> IK tip-link quat
        if seed is None:''')
s=s.replace('''World <-> base: base (panda_link0) sits at world (-0.75, 0, 0.912),
identity rotation.  IK/FK run in the base frame (frame_id empty).''','''On THIS machine the FK/IK model frame is `world` (panda_link0 sits at
world (-0.75, 0, 0.912)); poses here are all world-frame hand poses.''')
open('rob.py','w').write(s)
EOF
timeout 600 python3 -u -c "
from rob import *
r=Rob()
q0=r.arm_q()
for quat in ([1,0,0,0],[0.707,0.707,0,0]):
    q=r.ik_world([-0.071,0.035,1.22],quat,seed=q0)
    pos,qt=r.hand_pose_world(q)
    print('hand req',quat,'-> j7=%.3f'%q[6],'FK quat',qt.round(3),'pos',pos.round(3))
"

# openrua op 24
cat > /workspace/fit.py <<'EOF'
import numpy as np
# cup: truncated cone, bottom r=0.037, top r=0.052, h=0.16 (axis frame, bottom at origin)
rb, rt, H = 0.037, 0.052, 0.16
# slot geometry (xz plane, y ignored): back wall inner face x=-0.464 (z<=1.056), floor z=0.904,
# divider face x=-0.405 (z<=0.988), divider top z=0.988 for x in [-0.405,-0.397], front compartment beyond.
# Also back wall top z=1.056 for x in [-0.475,-0.464].
XL, ZLT = -0.464, 1.056
XR, ZRT = -0.405, 0.988
ZF = 0.904
# sample cup surface points in its own frame (x along tilt direction, z up)
hs = np.linspace(0, H, 33); phis = np.linspace(0, 2*np.pi, 72, endpoint=False)
pts = []
for h in hs:
    r = rb + (rt-rb)*h/H
    for p in phis:
        pts.append([r*np.cos(p), r*np.sin(p), h])
# bottom disk
for rr in np.linspace(0, rb, 6):
    for p in phis:
        pts.append([rr*np.cos(p), rr*np.sin(p), 0.0])
P = np.array(pts)
def free(x, z):
    # inside solid? back wall: x<=XL and z<=ZLT ; below floor ; divider region x>=XR and z<=ZRT (treat everything right of XR below ZRT as solid)
    solid = ((x <= XL) & (z <= ZLT)) | (z <= ZF) | ((x >= XR) & (z <= ZRT))
    return ~solid
best = {}
for th_deg in range(-50, 51, 5):
    th = np.deg2rad(th_deg)  # positive: top leans toward +x
    R = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])  # rotate about y in xz
    xz = P[:, [0, 2]] @ R.T
    lo = None
    for xc in np.arange(-0.47, -0.36, 0.001):
        # find lowest zc such that all points free
        for zc in np.arange(0.90, 1.20, 0.001):
            x = xz[:, 0] + xc; z = xz[:, 1] + zc
            if free(x, z).all():
                if lo is None or zc < lo[1]:
                    lo = (xc, zc)
                break
    xc, zc = lo
    # cup mid-height center in world
    cx = xc + (H/2)*np.sin(th); cz = zc + (H/2)*np.cos(th)
    print(f"tilt {th_deg:+3d}: bottom-center=({xc:.3f},{zc:.3f})  mid-center=({cx:.3f},{cz:.3f})")
EOF
python3 fit.py

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png >/dev/null; timeout 20 ros2 run tf2_ros tf2_echo world sideview_optical_frame 2>&1 | grep -A2 Translation | head -3; python3 - <<'EOF'
import numpy as np
d=np.load('sideview_depth.npy')
# cup in sideview color at approx x 305-360, y 250-315
sub=d[240:330,290:380]
# background (table) depth at those pixels is larger; cup is nearer
for v in range(0,90,3):
    row=''.join('#' if sub[v,u]<sub[v,:].max()-0.05 and sub[v,u]<2.0 else '.' for u in range(0,90))
    print(240+v, row, ' min=%.3f'%sub[v].min())
EOF

# openrua op 26
timeout 900 python3 -u -c "
from rob import *
r=Rob()
print('open gripper'); r.gripper(0.04)
CUP=np.array([-0.120,0.035]); QG=np.array([0.707,0.707,0,0])
print('pregrasp'); q=r.move_world([CUP[0]+0.049, CUP[1], 1.26], QG, seconds=4)
print('q', np.round(q,3))
" 2>&1 | tail -8

# openrua op 27
timeout 900 python3 -u -c "
from rob import *
r=Rob()
q=[0.084,0.477,-0.039,-1.619,0.021,2.095,-0.746]
print('now', np.round(r.arm_q(),3))
r.move_q(q, 3)
print('now', np.round(r.arm_q(),3))
pos,qt=r.hand_pose_world(); print('hand', pos.round(4), qt.round(3))
" 2>&1 | tail -6

# openrua op 28
timeout 900 python3 -u -c "
from rob import *
r=Rob()
q=[0.084,0.477,-0.039,-1.619,0.021,2.095,-0.746]
for i in range(3):
    code,err=r.move_q(q, 4)
    print('now', np.round(r.arm_q(),3))
    if err<0.02: break
pos,qt=r.hand_pose_world(); print('hand', pos.round(4), qt.round(3))
" 2>&1 | tail -8

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 30
timeout 900 python3 -u -c "
from rob import *
r=Rob()
CUP=np.array([-0.120,0.035]); QG=np.array([0.707,0.707,0,0])
print('wrench before', r.wrench().round(2))
q=r.ik_world([CUP[0]+0.049, CUP[1], 1.115], QG)
print('q', np.round(q,3))
for i in range(3):
    code,err=r.move_q(q, 3)
    if err<0.01: break
pos,qt=r.hand_pose_world(); print('hand', pos.round(4), qt.round(3))
print('wrench after', r.wrench().round(2))
" 2>&1 | tail -8

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('agent2.png'); crop=im[200:380,220:420]; cv2.imwrite('agent2_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py sideview side2.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('side2.png'); crop=im[150:340,250:450]; cv2.imwrite('side2_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 33
timeout 20 ros2 run tf2_ros tf2_echo world panda_leftfinger 2>&1 | grep -A1 Translation | head -2; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand_tcp 2>&1 | grep -A1 Translation | head -2; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 Translation | head -2

# openrua op 34
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw side2_depth.png >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('side2_depth.npy')
# sideview cam pose
t=np.array([-0.057,1.276,1.488]); q=np.array([0.010,0.806,-0.591,-0.007]); q/=np.linalg.norm(q)
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
fx=579.4112549695428; cx=320; cy=240
def w_(u,v):
    Z=d[v,u]; return R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z])+t
# scan column through left fingertip (crop x=125/3+250=292) and right finger (255/3+250=335), rows 250-300
for u in (292, 335, 312):
    print('col',u)
    for v in range(255,300,2):
        print(' ',v, np.round(w_(u,v),3))
EOF

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v,z=None):
    if z is None: z=d[v,u]
    return np.array([-0.2+(v-cy)*z/fy, (u-cx)*z/fx, 3.0-z])
tab=d[300,300]; print('table z', 3.0-tab)
y0,y1,x0,x1=245,290,300,360
sub=d[y0:y1,x0:x1]
mask=(sub<tab-0.03)
# exclude gripper (very high): cup rim expected z<1.1
mask &= (3.0-sub) < 1.1
zmin=sub[mask].min(); print('cup top z', 3.0-zmin)
body = sub < zmin+0.008
ys,xs=np.nonzero(body)
print('rim px x',xs.min()+x0,xs.max()+x0,'y',ys.min()+y0,ys.max()+y0)
for r in range(0,y1-y0):
    print(''.join('#' if body[r,c] else ('+' if mask[r,c] else ('G' if (3.0-sub[r,c])>=1.1 else '.')) for c in range(0,x1-x0)))
# caddy re-check
print('caddy col u=268')
for v in (160,165,170,175,180,182,185,190,195,200,202):
    p=w(268,v); print(v,p.round(3))
EOF

# openrua op 36
python3 - <<'EOF'
import numpy as np
d=np.load('side2_depth.npy')
t=np.array([-0.057,1.276,1.488]); q=np.array([0.010,0.806,-0.591,-0.007]); q/=np.linalg.norm(q)
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
fx=579.4112549695428; cx=320; cy=240
H,W=d.shape
uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fx,d],-1).reshape(-1,3)@R.T+t
P=P.reshape(H,W,3)
X,Y,Z=P[...,0],P[...,1],P[...,2]
cup=(X>-0.22)&(X<-0.03)&(Y>-0.06)&(Y<0.13)&(Z>0.885)&(Z<1.0)
print('cup pts',cup.sum())
print('x range',X[cup].min(),X[cup].max(),'y range',Y[cup].min(),Y[cup].max(),'z max',Z[cup].max())
# rim: z>0.98
rim=cup&(Z>0.982)
print('rim x',X[rim].min(),X[rim].max(),'y',Y[rim].min(),Y[rim].max())
# lower body z in 0.90-0.92
low=cup&(Z>0.90)&(Z<0.92)
print('low body x',X[low].min(),X[low].max(),'y',Y[low].min(),Y[low].max())
# fingers: z 1.0-1.07, x -0.15..0
fing=(X>-0.16)&(X<0.0)&(Y>-0.0)&(Y<0.1)&(Z>1.0)&(Z<1.07)
print('finger z min',Z[fing].min(), 'x', X[fing].min(), X[fing].max())
EOF

# openrua op 37
sed -i 's/^rb, rt, H = 0.037, 0.052, 0.16/rb, rt, H = 0.036, 0.050, 0.11/' fit.py && python3 fit.py | sed -n '5,17p'

# openrua op 38
timeout 900 python3 -u -c "
from rob import *
r=Rob()
QG=np.array([0.707,0.707,0,0])
q=r.ik_world([-0.0705, 0.036, 1.065], QG)
for i in range(3):
    code,err=r.move_q(q, 2.5)
    if err<0.01: break
pos,qt=r.hand_pose_world(); print('hand', pos.round(4), qt.round(3))
print('wrench', r.wrench().round(2))
" 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py sideview side3.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('side3.png'); crop=im[150:340,250:450]; cv2.imwrite('side3_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 39
timeout 600 python3 -u -c "
from rob import *
r=Rob()
print('wrench before', r.wrench().round(2))
f=r.gripper(0.0)
print('wrench after', r.wrench().round(2))
" 2>&1 | tail -3

# openrua op 40
timeout 900 python3 -u -c "
from rob import *
r=Rob()
QG=np.array([0.707,0.707,0,0])
q=r.ik_world([-0.0705, 0.036, 1.30], QG)
for i in range(3):
    code,err=r.move_q(q, 3)
    if err<0.01: break
pos,qt=r.hand_pose_world(); print('hand', pos.round(4), qt.round(3))
print('fingers', r.fingers()); print('wrench', r.wrench().round(2))
" 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py sideview side4.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png >/dev/null

# openrua op 41
timeout 900 python3 -u -c "
from rob import *
r=Rob()
QG=np.array([0.707,0.707,0,0])
tgt=[-0.42+0.0475, -0.147, 1.30]
q=r.ik_world(tgt, QG)
print('q', np.round(q,3))
for i in range(4):
    code,err=r.move_q(q, 4)
    if err<0.01: break
pos,qt=r.hand_pose_world(); print('hand', pos.round(4), qt.round(3))
print('fingers', r.fingers())
" 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py sideview side5.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw side5_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png >/dev/null

# openrua op 42
python3 - <<'EOF'
import numpy as np
d=np.load('side5_depth.npy')
t=np.array([-0.057,1.276,1.488]); q=np.array([0.010,0.806,-0.591,-0.007]); q/=np.linalg.norm(q)
x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
fx=579.4112549695428; cx=320; cy=240
H,W=d.shape
uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=(np.stack([(uu-cx)*d/fx,(vv-cy)*d/fx,d],-1).reshape(-1,3)@R.T+t).reshape(H,W,3)
X,Y,Z=P[...,0],P[...,1],P[...,2]
# cup expected around x -0.47..-0.36, y -0.2..-0.09, z 1.10..1.22 ; exclude gripper (metal, z>1.19?)
cup=(X>-0.50)&(X<-0.33)&(Y>-0.25)&(Y<-0.04)&(Z>1.09)&(Z<1.235)
print('pts',cup.sum())
for zlo in np.arange(1.09,1.24,0.01):
    m=cup&(Z>=zlo)&(Z<zlo+0.01)
    if m.sum(): print(f'z {zlo:.2f}: n={m.sum():4d} x[{X[m].min():.3f},{X[m].max():.3f}] y[{Y[m].min():.3f},{Y[m].max():.3f}]')
EOF

# openrua op 43
timeout 900 python3 -u -c "
from rob import *
r=Rob()
QG=np.array([0.707,0.707,0,0])
print('wrench before', r.wrench().round(2))
q=r.ik_world([-0.3825, -0.147, 1.175], QG)
for i in range(4):
    code,err=r.move_q(q, 3)
    if err<0.01: break
pos,qt=r.hand_pose_world(); print('hand', pos.round(4), qt.round(3))
print('wrench at place height', r.wrench().round(2))
print('fingers', r.fingers())
" 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py sideview side6.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agent6.png >/dev/null

# openrua op 44
python3 -c "
import cv2
im=cv2.imread('side6.png'); crop=im[150:340,330:530]; cv2.imwrite('side6_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agent6.png'); crop=im[60:280,150:370]; cv2.imwrite('agent6_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 45
timeout 600 python3 -u -c "
from rob import *
r=Rob()
r.gripper(0.04)
print('wrench after release', r.wrench().round(2))
" 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py sideview side7.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agent7.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('agent7.png'); crop=im[60:280,150:370]; cv2.imwrite('agent7_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 46
timeout 900 python3 -u -c "
from rob import *
r=Rob()
QG=np.array([0.707,0.707,0,0])
q=r.ik_world([-0.3754, -0.1455, 1.33], QG)
for i in range(3):
    code,err=r.move_q(q, 3)
    if err<0.01: break
# then move away toward +x so cameras see the caddy
q=r.ik_world([-0.15, 0.0, 1.35], QG)
for i in range(3):
    code,err=r.move_q(q, 4)
    if err<0.01: break
pos,qt=r.hand_pose_world(); print('hand', pos.round(4))
" 2>&1 | tail -3; for c in agentview sideview frontview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}_final.png >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_final_depth.png >/dev/null; ls *_final*

# openrua op 47
python3 - <<'EOF'
import numpy as np
d=np.load('birdview_final_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
uu,vv=np.meshgrid(np.arange(W),np.arange(H))
X=-0.2+(vv-cy)*d/fy; Y=(uu-cx)*d/fx; Z=3.0-d
# cup region: caddy middle column y in [-0.21,-0.08], x in [-0.50,-0.33], z above caddy floor+... find pixels with z>1.0 and not the arm (arm z>1.2)
m=(Y>-0.25)&(Y<-0.05)&(X>-0.52)&(X<-0.30)&(Z>0.995)&(Z<1.15)
print('pts',m.sum(),'z max',Z[m].max())
for zlo in np.arange(0.99,1.15,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f'z {zlo:.2f}: n={mm.sum():3d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
# rim = top ~1.5cm
rim=m&(Z>Z[m].max()-0.015)
print('rim center x=%.3f y=%.3f z=%.3f'%(X[rim].mean(),Y[rim].mean(),Z[rim].mean()))
# cup interior floor visible? check the lowest cup-ish z inside the rim footprint
inside=(Y>Y[rim].min()+0.01)&(Y<Y[rim].max()-0.01)&(X>X[rim].min()+0.01)&(X<X[rim].max()-0.01)
print('inside min z', Z[inside].min())
EOF

# openrua op 48
timeout 900 python3 -u -c "
from rob import *
r=Rob()
QG=np.array([0.707,0.707,0,0])
q=r.ik_world([-0.15, 0.0, 1.40], QG)
for i in range(3):
    code,err=r.move_q(q, 3)
    if err<0.01: break
q=r.ik_world([-0.15, 0.0, 1.35], QG)
for i in range(3):
    code,err=r.move_q(q, 3)
    if err<0.01: break
" 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_final2_depth.png >/dev/null; python3 - <<'EOF'
import numpy as np
for f in ('birdview_final_depth.npy','birdview_final2_depth.npy'):
    d=np.load(f); fx=579.4112549695428; cx=320; cy=240
    H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
    X=-0.2+(vv-cy)*d/fx; Y=(uu-cx)*d/fx; Z=3.0-d
    m=(Y>-0.25)&(Y<-0.05)&(X>-0.52)&(X<-0.30)&(Z>1.062)&(Z<1.15)  # above caddy walls => cup only
    print(f, 'cup-above-walls pts', m.sum(), 'top z %.4f'%Z[m].max(), 'x[%.3f,%.3f] y[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max()))
EOF

# openrua op 49
timeout 60 python3 tools/perception/cam_snap.py agentview agentview_final.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('agentview_final.png'); crop=im[120:290,100:380]; cv2.imwrite('final_crop.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"
