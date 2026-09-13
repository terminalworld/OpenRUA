#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -v parameter && echo ---- && ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[230:400,90:410]
cv2.imwrite('crop1.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
for p in "robot0_eye_in_hand 278 295" "robot0_eye_in_hand 572 230" "agentview 125 318" "agentview 348 285" "robot0_eye_in_hand 60 200" "agentview 530 260"; do echo "$p -> $(timeout 120 python3 tools/perception/px2world.py $p 2>&1 | tail -1)"; done

# openrua op 7
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ----; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -60

# openrua op 8
for p in "robot0_eye_in_hand 320 240" "robot0_eye_in_hand 400 100" "robot0_eye_in_hand 280 280" "robot0_eye_in_hand 280 310" "robot0_eye_in_hand 265 295" "robot0_eye_in_hand 290 295" "robot0_eye_in_hand 555 230" "robot0_eye_in_hand 590 230" "robot0_eye_in_hand 572 205" "robot0_eye_in_hand 572 255"; do echo "$p -> $(timeout 120 python3 tools/perception/px2world.py $p 2>&1 | tail -1)"; done

# openrua op 9
cat > /workspace/segment.py <<'EOF'
#!/usr/bin/env python3
"""Segment objects above the table in a camera's depth frame; print world centroid, principal axis yaw, extents, top height."""
import sys, struct
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

cam = sys.argv[1] if len(sys.argv) > 1 else "robot0_eye_in_hand"
TABLE_Z = 0.425
rclpy.init(); node = rclpy.create_node("seg")
tfbuf = Buffer(); TransformListener(tfbuf, node)
def grab(topic, T):
    got = {}
    s = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
depth = CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(grab(f"/{cam}/color/image_raw", Image), "bgr8")
info = grab(f"/{cam}/color/camera_info", CameraInfo)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
frame = f"{cam}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1)
pw = pc @ R.T + tr
valid = np.isfinite(depth) & (depth > 0)
mask = valid & (pw[..., 2] > TABLE_Z + 0.008) & (pw[..., 2] < TABLE_Z + 0.35)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1, n):
    m = lab == i
    if m.sum() < 60: continue
    P = pw[m]; c = P.mean(0)
    xy = P[:, :2] - c[:2]
    ev, evec = np.linalg.eigh(np.cov(xy.T))
    ax = evec[:, 1]; yaw = np.degrees(np.arctan2(ax[1], ax[0]))
    proj_l = xy @ ax; proj_w = xy @ evec[:, 0]
    us, vs = uu[m], vv[m]
    bgr = color[m].mean(0)
    print(f"obj{i}: px=({us.mean():.0f},{vs.mean():.0f}) n={m.sum()} world c=({c[0]:.3f},{c[1]:.3f}) top={P[:,2].max():.3f} "
          f"len={proj_l.max()-proj_l.min():.3f} wid={proj_w.max()-proj_w.min():.3f} yaw={yaw:.0f}deg bgr={bgr.astype(int)}")
rclpy.shutdown()
EOF
timeout 300 python3 /workspace/segment.py robot0_eye_in_hand

# openrua op 10
timeout 300 python3 /workspace/segment.py birdview 2>&1 | grep -v "^obj.*n=[0-9]\{6\}"

# openrua op 11
cat > /workspace/arm.py <<'EOF'
#!/usr/bin/env python3
"""Helper: IK moves in WORLD coords (TCP), gripper, joint read. Clients built once."""
import sys, math, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from tf world->panda_link0

def quat_down(yaw=0.0):
    """hand pointing -Z world, fingers along world y (yaw=0); yaw rotates about world z."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = math.cos(yaw/2), math.sin(yaw/2)
    # (w,x,y,z) product of (c,0,0,s) * (0,1,0,0) = (0, c, s, 0)
    return (c, s, 0.0, 0.0)  # x,y,z,w

def R_of(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class Arm:
    def __init__(self):
        rclpy.init(); self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js: rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self.js["m"].name, self.js["m"].position))

    def arm_state(self):
        j = self.joints(); s = JointState(); s.name = list(JOINTS); s.position = [j[n] for n in JOINTS]; return s

    def tcp_world(self):
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        R = R_of(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return hand + TCP * R[:, 2], (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def solve_ik(self, xyz_world, q, at_tcp=True):
        R = R_of(*q); xyz = np.array(xyz_world) - BASE_IN_WORLD
        if at_tcp: xyz = xyz - TCP * R[:, 2]
        req = GetPositionIK.Request(); req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, positions, seconds):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints(); err = max(abs(j[n] - p) for n, p in zip(JOINTS, positions))
        print(f"  traj code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move(self, xyz_world, yaw=0.0, seconds=3.0, retries=2):
        q = quat_down(yaw)
        for k in range(retries + 1):
            pos = self.solve_ik(xyz_world, q)
            code, err = self.traj(pos, seconds)
            if code == 0 and err < 0.01: break
        tcp, _ = self.tcp_world()
        print(f"  TCP now world=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) target={tuple(round(v,3) for v in xyz_world)}", flush=True)
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal(); goal.command.position = float(width); goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=180)
        r = res.result().result; j = self.joints()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers=({j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f})", flush=True)
        return j['panda_finger_joint1']
EOF
cat > /workspace/step_check.py <<'EOF'
from arm import Arm
a = Arm()
print(a.joints())
print(a.tcp_world())
EOF
timeout 120 python3 step_check.py

# openrua op 12
cat > fkcheck.py <<'EOF'
from arm import *
a = Arm()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0", "panda_hand"]
req.robot_state.joint_state = a.arm_state()
fut = a.fk.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
for ps in fut.result().pose_stamped:
    print(ps.header.frame_id, ps.pose.position)
# IK test: hand world pose from TF
q = (1.0, 0.0, -0.0284, 0.0)
for label, xyz in [("world-coords", (-0.053, 0.0, 0.778)), ("base-coords", (0.457, 0.0, 0.358))]:
    try:
        req = GetPositionIK.Request(); req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = a.arm_state()
        fut = a.ik.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=60)
        res = fut.result()
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        print(label, res.error_code.val, [round(sol.get(j, float('nan')), 3) for j in JOINTS])
    except Exception as e:
        print(label, "err", e)
EOF
timeout 200 python3 fkcheck.py; ros2 topic echo /robot_description --once --qos-durability transient_local 2>/dev/null | grep -oE '<link name="[a-z0-9_]+"|<joint name="[a-z0-9_]+" type="[a-z]+"' | head -20

# openrua op 13
cat /tmp/claude-1000/-workspace/2fddc760-4c45-4730-bd41-aa7d29037a17/tasks/bdxd05xfy.output

# openrua op 14
pkill -f "ros2 topic echo /robot_description"; sed -i 's/^BASE_IN_WORLD = .*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK\/IK model frame IS world on this machine (verified: FK of panda_link0 = (-0.51,0,0.42))/' arm.py
cat > fkcheck2.py <<'EOF'
from arm import *
a = Arm()
tcp, q = a.tcp_world()
print("TCP world", tcp.round(3), "hand quat", [round(v,3) for v in q])
EOF
timeout 120 python3 fkcheck2.py

# openrua op 15
timeout 120 python3 fkcheck2.py

# openrua op 16
grep -n "^BASE_IN_WORLD" arm.py; sed -i 's/^BASE_IN_WORLD = .*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK\/IK model frame IS world here (FK of panda_link0 = (-0.51,0,0.42))/' arm.py; grep -n "^BASE_IN_WORLD" arm.py
cat > fkcheck2.py <<'EOF'
from arm import *
a = Arm()
tcp, q = a.tcp_world()
print("TCP world", tcp.round(3), "hand quat", [round(v,3) for v in q])
EOF
timeout 120 python3 fkcheck2.py

# openrua op 17
cat > /workspace/pick_place.py <<'EOF'
import sys
from arm import Arm
OBJ = {"butter": (-0.084, 0.045, 0.449), "cream_cheese": (-0.012, -0.257, 0.455)}
BASKET = (0.007, 0.258)
TABLE = 0.425
x, y, top = OBJ[sys.argv[1]]
grasp_z = TABLE + 0.011
a = Arm()
print("open gripper"); a.gripper(0.04)
print("above object"); a.move((x, y, 0.56), seconds=3)
print("descend"); a.move((x, y, grasp_z), seconds=2.5)
print("close"); f = a.gripper(0.0)
if f < 0.005: raise SystemExit("GRASP FAILED: fingers closed on air")
print("lift"); a.move((x, y, 0.70), seconds=2.5)
j = a.joints(); print(f"  fingers after lift: {j['panda_finger_joint1']:.4f}")
print("to basket"); a.move((BASKET[0], BASKET[1], 0.72), seconds=3)
j = a.joints(); print(f"  fingers over basket: {j['panda_finger_joint1']:.4f}")
print("release"); a.gripper(0.04)
print("retreat up"); a.move((BASKET[0], BASKET[1] - 0.1, 0.75), seconds=2)
print("DONE")
EOF
nohup python3 -u pick_place.py butter > pp_butter.log 2>&1 &
echo started

# openrua op 18
sleep 60; cat pp_butter.log

# openrua op 19
sleep 45; cat pp_butter.log

# openrua op 20
timeout 120 python3 tools/perception/cam_snap.py agentview agent2.png; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 21
nohup python3 -u pick_place.py cream_cheese > pp_cc.log 2>&1 & sleep 100; cat pp_cc.log

# openrua op 22
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png; timeout 120 python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 23
cat > lift.py <<'EOF'
from arm import Arm
a = Arm()
tcp, _ = a.tcp_world(); print("tcp", tcp.round(3))
a.move((tcp[0], tcp[1], 0.60), seconds=2.5)
a.gripper(0.04)
EOF
timeout 300 python3 -u lift.py && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png && timeout 300 python3 segment.py robot0_eye_in_hand

# openrua op 24
for p in "robot0_eye_in_hand 100 300" "robot0_eye_in_hand 500 300" "robot0_eye_in_hand 320 100" "robot0_eye_in_hand 320 400"; do echo "$p -> $(timeout 120 python3 tools/perception/px2world.py $p 2>&1 | tail -1)"; done; ros2 topic echo /joint_states --once | grep -A8 position

# openrua op 25
timeout 120 python3 fkcheck2.py; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 Translation | head -3; timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 Translation | head -4

# openrua op 26
cat >> segment.py <<'EOF'
# debug: draw long-axis for objects near table height
rclpy.init(); 
EOF
python3 - <<'EOF'
import re
s = open('/workspace/segment.py').read()
s = s.replace("# debug: draw long-axis for objects near table height\nrclpy.init(); \n", "")
s = s.replace('''    print(f"obj{i}:''', '''    if P[:,2].max() < TABLE_Z + 0.05:
        e = (ax * (proj_l.max()-proj_l.min())/2)
        # world->pixel for the endpoints: use nearest mask pixels
        for sgn in (1, -1):
            tgt = c[:2] + sgn * e
            d = np.linalg.norm(P[:, :2] - tgt, axis=1); k = d.argmin()
            cv2.circle(color, (int(us[k]), int(vs[k])), 6, (0, 255, 0), 2)
        cv2.imwrite("seg_debug.png", color)
    print(f"obj{i}:''')
open('/workspace/segment.py','w').write(s)
EOF
timeout 300 python3 segment.py robot0_eye_in_hand

# openrua op 27
python3 -c "
import cv2; im=cv2.imread('seg_debug.png'); cv2.imwrite('seg_crop.png', cv2.resize(im[220:400,200:460],None,fx=3,fy=3))"

# openrua op 28
python3 - <<'EOF'
s = open('/workspace/arm.py').read()
s = s.replace('req.ik_request.pose_stamped.header.frame_id = ""\n', 'req.ik_request.pose_stamped.header.frame_id = ""\n        req.ik_request.ik_link_name = "panda_hand"  # default tip is panda_link8 (45deg yaw off)\n')
s = s.replace('OBJ = ', 'OBJ = ')
open('/workspace/arm.py','w').write(s)
s = open('/workspace/pick_place.py').read()
s = s.replace('"cream_cheese": (-0.012, -0.257, 0.455)', '"cream_cheese": (-0.001, -0.274, 0.455)')
open('/workspace/pick_place.py','w').write(s)
EOF
cat > test_orient.py <<'EOF'
from arm import Arm
a = Arm()
a.move((-0.001, -0.274, 0.56), yaw=0.0, seconds=3)
tcp, q = a.tcp_world(); print("hand quat", [round(v,3) for v in q])
EOF
timeout 400 python3 -u test_orient.py

# openrua op 29
nohup python3 -u pick_place.py cream_cheese > pp_cc2.log 2>&1 & sleep 120; cat pp_cc2.log

# openrua op 30
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png; timeout 120 python3 tools/perception/cam_snap.py agentview agent5.png; timeout 300 python3 segment.py birdview 2>&1 | grep -v "n=[0-9]\{4,\}"

# openrua op 31
for p in "robot0_eye_in_hand 232 285" "robot0_eye_in_hand 212 275" "robot0_eye_in_hand 268 292" "robot0_eye_in_hand 160 250"; do echo "$p -> $(timeout 120 python3 tools/perception/px2world.py $p 2>&1 | tail -1)"; done
