#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; echo ---; find /workspace/tools -type f; echo ---; ls -la /workspace/.ros

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | head -30

# openrua op 7
cat > tf_dump.py <<'EOF'
import rclpy
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
for _ in range(20): rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    tr, r = t.translation, t.rotation
    print(f"{k[0]:>28} -> {k[1]:<28} t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({r.x:.3f},{r.y:.3f},{r.z:.3f},{r.w:.3f})")
EOF
timeout 60 python3 tf_dump.py

# openrua op 8
cat > px2w.py <<'EOF'
"""px2w.py <camera> u,v [u,v ...]  -> world xyz for each pixel (one depth frame)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]
pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
rclpy.init(); node = rclpy.create_node("px2w")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["t"] = t.transform
node.create_subscription(TFMessage, "/tf", tfcb, 10)
while len(got) < 3: rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["t"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
np.save(f"{cam}_depth.npy", depth)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
for (u, v) in pix:
    zc = depth[v, u]
    pc = np.array([(u-cx)*zc/fx, (v-cy)*zc/fy, zc])
    pw = R @ pc + T
    print(f"({u},{v}) depth={zc:.3f} world=({pw[0]:.3f}, {pw[1]:.3f}, {pw[2]:.3f})")
EOF
timeout 60 python3 px2w.py agentview 207,230 322,228 395,235 338,385 320,300 320,450 100,400 540,400

# openrua op 9
timeout 60 python3 px2w.py birdview 283,265 318,283 350,270 326,322 326,200 320,240 && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# table plane depth ~ 3.0-0.426 = 2.574; find things above table
tab = d[240,200]; print('table depth', tab)
mask = (d < tab-0.01).astype(np.uint8)*255
cv2.imwrite('bird_mask.png', mask)
n,lab,stats,cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]>20: print(i, stats[i], cent[i], 'minDepth', d[lab==i].min())
"

# openrua op 10
timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A9 "^k:"; timeout 20 ros2 topic echo /agentview/color/camera_info --once | grep -A9 "^k:"

# openrua op 11
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250, precision=3, suppress=True)
d=np.load('birdview_depth.npy')
print('mug row 265, cols 265..300'); print(3.0-d[265,265:300])
print('mug col 283, rows 250..282'); print(3.0-d[250:282,283])
print('plate row 321, cols 300..350'); print(3.0-d[321,300:350])
print('plate col 324, rows 300..345'); print(3.0-d[300:345,324])
"

# openrua op 12
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position | tail -3; timeout 60 python3 px2w.py birdview 283,265 326,322 320,240 && python3 -c "
import numpy as np
np.set_printoptions(linewidth=250, precision=3, suppress=True)
d=np.load('birdview_depth.npy')
print('mug row 265'); print(3.0-d[265,265:300])
print('plate row 321'); print(3.0-d[321,300:350])
"

# openrua op 13
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Helper library: joint state, FK, IK, trajectory, gripper on this Panda."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_W = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 translation
TCP = M["hand"]["tcp_offset_m"]

# top-down grasp orientation: hand Z pointing down (world -Z), hand X along world +X
# quaternion (x,y,z,w) for rotation of pi about X axis
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def q_down_yaw(yaw):
    """Top-down orientation rotated by yaw about world Z."""
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def spin(self, t=0.1):
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
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, pos_w, quat, seed=None, at_tcp=True, timeout=60):
        """IK for hand pose given in WORLD frame. pos is the TCP point if at_tcp."""
        pos = np.array(pos_w, dtype=float)
        if at_tcp:
            R = _quat_to_R(*quat)
            pos = pos - TCP * R[:, 2]
        pos_b = pos - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos_b
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_q(self, q, seconds=3.0, via=None):
        """Send trajectory; via = list of (q, t) intermediate points."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for vq, vt in (via or []):
            pt = JointTrajectoryPoint(positions=[float(x) for x in vq])
            pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        print(f"  move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos_w, quat, seconds=3.0, seed=None, at_tcp=True):
        q = self.ik(pos_w, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print(f"  IK FAILED for {pos_w}")
            return None
        self.move_q(q, seconds)
        p, _ = self.fk()
        tcp = p + TCP * _quat_to_R(*quat)[:, 2] if at_tcp else p
        print(f"  reached tcp={np.round(tcp, 4)} target={np.round(pos_w, 4)}")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from rob import *
r = Robot()
print('q', np.round(r.arm_q(),3))
p, q = r.fk(); print('hand world', np.round(p,3), 'quat', np.round(q,3))
print('finger gap', r.finger_gap())
# test IK for a pose above the white mug
sol = r.ik([-0.09, -0.163, 0.70], Q_DOWN)
print('ik above mug', sol and np.round(sol,3))
if sol:
    p2,q2 = r.fk(sol); print('fk of sol', np.round(p2,3), np.round(q2,3))
"

# openrua op 15
timeout 60 python3 -c "
import numpy as np
def dh(a,d,alpha,theta):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(theta),np.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161,0,-2.445,0,2.227,0.785]
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
T=np.eye(4)
for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
T=T@dh(0,0.107,0,0)  # flange
print('flange in base', np.round(T[:3,3],3))
"

# openrua op 16
sed -i 's/^BASE_W = np.array(\[-0.51, 0.0, 0.42\]).*/BASE_W = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world here (verified via FK)/' rob.py && grep -n BASE_W rob.py | head -3 && timeout 120 python3 -c "
from rob import *
r = Robot()
p, q = r.fk(); print('hand world', np.round(p,3), 'quat', np.round(q,3))
sol = r.ik([-0.09, -0.163, 0.70], Q_DOWN)
print('ik above mug', sol and np.round(sol,3))
if sol:
    p2,q2 = r.fk(sol); print('fk of sol (hand)', np.round(p2,3), np.round(q2,3))
sol2 = r.ik([0.16, 0.026, 0.70], Q_DOWN)
print('ik above plate', sol2 and np.round(sol2,3))
sol3 = r.ik([0.16, 0.18, 0.60], Q_DOWN)
print('ik right of plate', sol3 and np.round(sol3,3))
"

# openrua op 17
python3 - <<'EOF'
import re
s = open('rob.py').read()
s = s.replace('''Q_DOWN = (1.0, 0.0, 0.0, 0.0)
''', '''Q_DOWN = (1.0, 0.0, 0.0, 0.0)
# IK tip link is panda_link8; panda_hand = link8 * Rz(-45deg). Convert hand quat -> link8 quat.
Q_L8_FROM_HAND = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))  # Rz(+45deg)
''')
s = s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state''', '''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat_mul(quat, Q_L8_FROM_HAND)
        req.ik_request.robot_state''')
open('rob.py','w').write(s)
EOF
timeout 300 python3 -c "
from rob import *
r = Robot()
sol = r.ik([-0.092, -0.161, 0.70], Q_DOWN)
print('ik', np.round(sol,3))
p2,q2 = r.fk(sol); print('fk hand', np.round(p2,3), np.round(q2,3))
r.move_q(sol, 4.0)
p,q = r.fk(); print('hand now', np.round(p,3), np.round(q,3), 'tcp z', p[2]-TCP)
"
for c in birdview robot0_eye_in_hand agentview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 18
timeout 60 python3 px2w.py birdview 322,236 346,270 && python3 -c "
import numpy as np, cv2
np.set_printoptions(linewidth=250, precision=3, suppress=True)
d=np.load('birdview_depth.npy'); f=579.4112549695428
z = 3.0-d
mask = ((z>0.435)&(z<0.60)).astype(np.uint8)*255
n,lab,stats,cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]>15:
        x,y,w,h,a = stats[i]; cu,cv = cent[i]
        zmax = z[lab==i].max(); dep = 3.0-zmax
        wx = -0.2 + (cv-240)*dep/f; wy = (cu-320)*dep/f
        print(f'comp{i} bbox=({x},{y},{w}x{h}) area={a} centroid_px=({cu:.1f},{cv:.1f}) top_z={zmax:.3f} world=({wx:.3f},{wy:.3f}) size~({h*dep/f:.3f}x{w*dep/f:.3f})m')
print('pudding row 236:', z[236,310:335]); print('pudding col 322:', z[225:250,322])
"

# openrua op 19
timeout 60 python3 px2w.py robot0_eye_in_hand 318,297 318,250 318,345 270,297 366,297 && python3 -c "
import numpy as np
np.set_printoptions(linewidth=250, precision=3, suppress=True)
d=np.load('robot0_eye_in_hand_depth.npy')
print('row 297:', d[297,250:390:4]); print('col 318:', d[230:370:4,318])
"; grep -A8 "robot0_eye_in_hand_optical" <(timeout 60 python3 tf_dump.py)

# openrua op 20
sed -i 's/^d, info, t = got\["d"\], got\["i"\], got\["t"\]/d, info, t = got["d"], got["i"], got["t"]\nprint("TF", cam, round(t.translation.x,3), round(t.translation.y,3), round(t.translation.z,3), round(t.rotation.x,3), round(t.rotation.y,3), round(t.rotation.z,3), round(t.rotation.w,3))/' px2w.py && timeout 60 python3 px2w.py robot0_eye_in_hand 318,297 && timeout 60 python3 px2w.py robot0_eye_in_hand 318,297

# openrua op 21
timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:" | head -2; timeout 300 python3 -c "
from rob import *
r = Robot()
print('gap', r.finger_gap())
# above grasp point
q1 = r.move_pose([-0.091, -0.199, 0.62], Q_DOWN, 3.0)
# descend
q2 = r.move_pose([-0.091, -0.199, 0.525], Q_DOWN, 2.5, seed=q1)
"
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 22
timeout 300 python3 -c "
from rob import *
r = Robot()
gap = r.gripper(0.0)
print('joints fingers', {k:round(v,4) for k,v in r.joints().items() if 'finger' in k})
# lift
q = r.move_pose([-0.091, -0.199, 0.70], Q_DOWN, 3.0)
print('gap after lift', r.finger_gap())
"
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 23
timeout 400 python3 -c "
from rob import *
r = Robot()
# mug center = TCP + (0, +0.0405); plate center ~ (0.152, 0.018)
q1 = r.move_pose([0.152, -0.022, 0.70], Q_DOWN, 4.0)
print('gap', r.finger_gap())
q2 = r.move_pose([0.152, -0.022, 0.56], Q_DOWN, 2.5, seed=q1)
print('gap', r.finger_gap())
"
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
timeout 400 python3 -c "
from rob import *
r = Robot()
r.gripper(0.04)
q = r.move_pose([0.152, -0.022, 0.72], Q_DOWN, 3.0)
"
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 25
timeout 60 python3 px2w.py birdview 320,240 >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428; z=3.0-d
def comps(lo,hi,minarea=15):
    mask=((z>lo)&(z<hi)).astype(np.uint8)*255
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    for i in range(1,n):
        if stats[i,4]>minarea:
            x,y,w,h,a=stats[i]; cu,cv=cent[i]; zmax=z[lab==i].max(); dep=3.0-zmax
            print(f'  z in ({lo},{hi}) bbox=({x},{y},{w}x{h}) area={a} top_z={zmax:.3f} world=({-0.2+(cv-240)*dep/f:.3f},{(cu-320)*dep/f:.3f})')
print('mug rim (z>0.53):'); comps(0.53,0.60)
print('plate-level (0.435-0.47):'); comps(0.435,0.47)
"

# openrua op 26
timeout 400 python3 -c "
from rob import *
r = Robot()
q1 = r.move_pose([-0.219, 0.005, 0.62], Q_DOWN, 4.0)
"
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 px2w.py robot0_eye_in_hand 320,240 && python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_depth.npy')
# camera ~ at z 0.72; pudding top at 0.464 -> depth ~0.26; table ~0.30
mask=((d>0.2)&(d<0.285)).astype(np.uint8)*255
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]>50: print(stats[i], cent[i], d[lab==i].min())
"

# openrua op 27
timeout 400 python3 -c "
from rob import *
r = Robot()
q0 = r.arm_q(); print('now ', np.round(q0,3))
sol = r.ik([-0.219, 0.005, 0.62], Q_DOWN)
print('sol ', np.round(sol,3))
print('diff', np.round(np.array(sol)-np.array(q0),3))
code, err = r.move_q(sol, 6.0)
if err > 0.05:
    print('resending'); r.move_q(sol, 6.0)
p,q = r.fk(); print('hand', np.round(p,3), np.round(q,3))
"

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 400 python3 -c "
from rob import *
r = Robot()
q0 = r.arm_q()
for z in [0.55, 0.50, 0.46, 0.44]:
    sol = r.ik([-0.219, 0.005, z], Q_DOWN, seed=q0)
    print(z, sol and np.round(sol,3))
"

# openrua op 29
timeout 60 python3 px2w.py robot0_eye_in_hand 320,297 && python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_depth.npy')
print('depth at center', d[240,240], 'box', d[297,320])
tab = np.median(d[200:230, 200:440])
mask=((d>0.05)&(d<tab-0.02)).astype(np.uint8)*255
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]>50: print(stats[i], cent[i], d[lab==i].min())
" && timeout 60 python3 px2w.py robot0_eye_in_hand 320,296 296,296 344,296 320,255 320,338

# openrua op 30
timeout 400 python3 -c "
from rob import *
r = Robot()
q0 = r.arm_q(); print('now', np.round(q0,3))
seed = q0; chain=[]
for z in [0.60, 0.57, 0.54, 0.51, 0.48, 0.46, 0.44]:
    sol = r.ik([-0.215, 0.006, z], Q_DOWN, seed=seed)
    if sol is None: print(z, 'FAIL'); break
    print(z, np.round(sol,3), 'maxdelta', round(max(abs(a-b) for a,b in zip(sol,seed)),3))
    chain.append(sol); seed = sol
np.save('chain.npy', np.array(chain))
"

# openrua op 31
timeout 600 python3 -c "
from rob import *
r = Robot()
q0 = r.arm_q()
def q_tilt(th):
    qy = (0.0, math.sin(th/2), 0.0, math.cos(th/2))
    return quat_mul(qy, Q_DOWN)
for deg in [15, 25, 35, -15, -25, -35]:
    th = math.radians(deg); Q = q_tilt(th)
    seed = q0; ok=True; out=[]
    for z in [0.60, 0.55, 0.50, 0.47, 0.44]:
        sol = r.ik([-0.215, 0.006, z], Q, seed=seed)
        if sol is None: out.append((z,'FAIL')); ok=False; break
        out.append((z, round(max(abs(a-b) for a,b in zip(sol,seed)),2)))
        seed = sol
    print(deg, out, 'final', ok and np.round(seed,2))
"

# openrua op 32
timeout 900 python3 -c "
from rob import *
r = Robot()
q0 = r.arm_q()
th = math.radians(25); Q = quat_mul((0.0, math.sin(th/2), 0.0, math.cos(th/2)), Q_DOWN)
seed = q0; chain=[]
for z in [0.60, 0.55, 0.50, 0.47, 0.44]:
    sol = r.ik([-0.215, 0.006, z], Q, seed=seed); chain.append(sol); seed = sol
np.save('pud_chain.npy', np.array(chain)); np.save('pud_Q.npy', np.array(Q))
print('gap before', r.finger_gap())
r.move_q(chain[0], 3.0)
p,_ = r.fk(); print('hand at z0.60 step:', np.round(p,3))
via = [(chain[i], 1.5*(i)) for i in range(1, len(chain)-1)]
r.move_q(chain[-1], 1.5*(len(chain)-1), via=via)
p,qq = r.fk(); R=_quat_to_R(*qq); tcp = p + TCP*R[:,2]; print('tcp', np.round(tcp,4), 'approach', np.round(R[:,2],3))
"
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 33
timeout 600 python3 -c "
from rob import *
from rob import _quat_to_R
r = Robot()
gap = r.gripper(0.0)
chain = np.load('pud_chain.npy'); Q = tuple(np.load('pud_Q.npy'))
# lift back up through the chain in reverse
via = [(chain[i], 1.2*(len(chain)-1-i)) for i in range(len(chain)-2, 0, -1)]
r.move_q(chain[0], 1.2*(len(chain)-1), via=via)
print('gap after lift', r.finger_gap())
p,qq = r.fk(); R=_quat_to_R(*qq); print('tcp', np.round(p + TCP*R[:,2],3))
"
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 60 python3 px2w.py birdview 320,240 >/dev/null && python3 -c "
import numpy as np, cv2
np.set_printoptions(linewidth=250, precision=3, suppress=True)
d=np.load('birdview_depth.npy'); f=579.4112549695428; z=3.0-d
mask=((z>0.44)&(z<0.60)).astype(np.uint8)*255
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]>15:
        x,y,w,h,a=stats[i]; cu,cv=cent[i]; zmax=z[lab==i].max(); dep=3.0-zmax
        print(f'bbox=({x},{y},{w}x{h}) area={a} top_z={zmax:.3f} centroid_world=({-0.2+(cv-240)*dep/f:.3f},{(cu-320)*dep/f:.3f}) ymax_world={(x+w-320)*2.5/f:.3f}')
# white mug on plate: rows around 320, cols 300-360
r0 = 320
print('row', r0, z[r0, 300:360])
"

# openrua op 35
timeout 900 python3 -c "
from rob import *
from rob import _quat_to_R
r = Robot()
Q = tuple(np.load('pud_Q.npy'))
tgt = (0.152, 0.16)
q1 = r.move_pose([tgt[0], tgt[1], 0.65], Q, 4.0)
print('gap', r.finger_gap())
seed = q1; chain=[]
for z in [0.58, 0.52, 0.48, 0.445]:
    sol = r.ik([tgt[0], tgt[1], z], Q, seed=seed)
    if sol is None: print('IK fail at', z); break
    print(z, 'delta', round(max(abs(a-b) for a,b in zip(sol,seed)),3)); chain.append(sol); seed=sol
via = [(chain[i], 1.2*(i+1)) for i in range(len(chain)-1)]
r.move_q(chain[-1], 1.2*len(chain), via=via)
p,qq = r.fk(); R=_quat_to_R(*qq); print('tcp', np.round(p + TCP*R[:,2],4), 'gap', r.finger_gap())
"
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 36
timeout 900 python3 -c "
from rob import *
from rob import _quat_to_R
r = Robot()
q0 = r.arm_q(); p,qq = r.fk(); R=_quat_to_R(*qq); tcp = p + TCP*R[:,2]
print('q now', np.round(q0,3)); print('tcp now', np.round(tcp,3), 'quat', np.round(qq,3), 'approach', np.round(R[:,2],3))
Q = tuple(np.load('pud_Q.npy'))
# lift straight up in small steps keeping current xy, target orientation Q
seed=q0; chain=[]
for z in [0.50, 0.56, 0.62]:
    sol = r.ik([tcp[0], tcp[1], z], Q, seed=seed)
    if sol is None: print('IK fail', z); break
    print(z, 'delta', round(max(abs(a-b) for a,b in zip(sol,seed)),3)); chain.append(sol); seed=sol
via = [(chain[i], 2.0*(i+1)) for i in range(len(chain)-1)]
r.move_q(chain[-1], 2.0*len(chain), via=via)
p,qq = r.fk(); R=_quat_to_R(*qq); print('tcp', np.round(p + TCP*R[:,2],4), 'gap', r.finger_gap())
"
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 37
timeout 900 python3 -c "
from rob import *
from rob import _quat_to_R
r = Robot()
Q = tuple(np.load('pud_Q.npy'))
tgt = (0.152, 0.16)
q0 = r.arm_q()
sol = r.ik([tgt[0], tgt[1], 0.62], Q, seed=q0)
print('transit delta', round(max(abs(a-b) for a,b in zip(sol,q0)),3))
code, err = r.move_q(sol, 4.0)
if err > 0.02: print('resend'); r.move_q(sol, 4.0)
seed = r.arm_q(); chain=[]
for z in [0.56, 0.51, 0.475, 0.445]:
    s = r.ik([tgt[0], tgt[1], z], Q, seed=seed)
    if s is None: print('IK fail', z); break
    print(z, 'delta', round(max(abs(a-b) for a,b in zip(s,seed)),3)); chain.append(s); seed=s
via = [(chain[i], 1.5*(i+1)) for i in range(len(chain)-1)]
code, err = r.move_q(chain[-1], 1.5*len(chain), via=via)
if err > 0.02: print('resend'); r.move_q(chain[-1], 3.0)
p,qq = r.fk(); R=_quat_to_R(*qq); print('tcp', np.round(p + TCP*R[:,2],4), 'approach', np.round(R[:,2],3), 'gap', r.finger_gap())
"
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 38
timeout 900 python3 -c "
from rob import *
from rob import _quat_to_R
r = Robot()
Q = tuple(np.load('pud_Q.npy'))
r.gripper(0.04)
seed = r.arm_q(); chain=[]
for z in [0.50, 0.58, 0.66]:
    s = r.ik([0.152, 0.16, z], Q, seed=seed); chain.append(s); seed=s
via = [(chain[i], 1.5*(i+1)) for i in range(len(chain)-1)]
r.move_q(chain[-1], 1.5*len(chain), via=via)
# park: back toward home-ish pose above the base area, away from the objects
home = [0.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.785]
code, err = r.move_q(home, 6.0)
if err > 0.02: r.move_q(home, 6.0)
p,qq = r.fk(); print('hand parked at', np.round(p,3))
"
for c in agentview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 39
timeout 60 python3 px2w.py birdview 320,240 >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428; z=3.0-d
def comps(lo,hi,label,minarea=15):
    mask=((z>lo)&(z<hi)).astype(np.uint8)*255
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    for i in range(1,n):
        if stats[i,4]>minarea and stats[i,2]<200:
            x,y,w,h,a=stats[i]; cu,cv=cent[i]; zmax=z[lab==i].max(); dep=3.0-zmax
            print(f'{label}: bbox=({x},{y},{w}x{h}) top_z={zmax:.3f} center_world=({-0.2+(cv-240)*dep/f:.3f},{(cu-320)*dep/f:.3f}) size=({h*dep/f:.3f}x{w*dep/f:.3f})')
comps(0.44,0.50,'low (plate/pudding)')
comps(0.53,0.62,'mug rims')
# plate region: everything 0.44-0.47 near (0.15,0.02); mug interior floor sits on plate
"
