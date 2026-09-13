#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/action/fjt_send.py tools/action/ik_move.py tools/action/gripper_cmd.py

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
timeout 30 ros2 topic list; echo "--- actions"; timeout 30 ros2 action list; echo "--- services"; timeout 30 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo "--- nodes"; timeout 30 ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
for p in "255 280" "300 285" "373 283" "376 240" "315 315" "200 200"; do echo -n "px $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p 2>&1 | tail -1; done

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8

# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A 10 "^k:"

# openrua op 8
mkdir -p "$(dirname /workspace/heightmap.py)"
cat > /workspace/heightmap.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Height map from birdview depth: camera at world (-0.2, 0, 3.0) looking
straight down; optical x -> world -y ... derived from TF:
R = [[0,1,0],[1,0,0],[0,0,-1]] (from tf2_echo matrix first row 0 1 0).
"""
import sys
import numpy as np

fx = fy = 579.4112549695428
cx, cy = 320.0, 240.0
cam_t = np.array([-0.2, 0.0, 3.0])
# quaternion (0.707,0.707,0,0) -> R
x, y, z, w = 0.7071068, 0.7071068, 0.0, 0.0
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])

d = np.load(sys.argv[1] if len(sys.argv) > 1 else "birdview_depth.npy")
H, W = d.shape
vs, us = np.mgrid[0:H, 0:W]
pc = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d], -1)
pw = pc @ R.T + cam_t
X, Y, Z = pw[..., 0], pw[..., 1], pw[..., 2]

def region(name, mask):
    if not mask.any():
        print(name, "empty"); return
    print(f"{name}: n={mask.sum()} x[{X[mask].min():.3f},{X[mask].max():.3f}] "
          f"y[{Y[mask].min():.3f},{Y[mask].max():.3f}] z[{Z[mask].min():.3f},{Z[mask].max():.3f}] "
          f"centroid=({X[mask].mean():.3f},{Y[mask].mean():.3f}) ztop={np.percentile(Z[mask],95):.3f}")

table = np.abs(Z - 0.90) < 0.005
print("table z median", np.median(Z[table]))
above = (Z > 0.905) & (X > -0.35) & (X < 0.4) & (np.abs(Y) < 0.5)
# cluster by rough y bands
region("pan region (y<-0.12)", above & (Y < -0.12) & (Y > -0.4) & (X < 0.1))
region("pan handle band (-0.13<y<-0.02, x<0)", above & (Y > -0.13) & (Y < -0.02) & (X < 0.0) & (X > -0.12))
region("moka region", above & (Y > -0.1) & (Y < 0.08) & (X > 0.0))
region("stove region", above & (Y > 0.1) & (Y < 0.3) & (X > -0.15) & (X < 0.1))
region("knob region", above & (Y > 0.1) & (Y < 0.3) & (X < -0.15) & (X > -0.3))
region("stove top plate (z>0.92)", above & (Y > 0.1) & (Y < 0.3) & (X > -0.15) & (X < 0.1) & (Z > 0.925))
region("stove burner (z>0.935)", above & (Y > 0.1) & (Y < 0.3) & (X > -0.15) & (X < 0.1) & (Z > 0.935))
np.save("heightZ.npy", Z); np.save("worldX.npy", X); np.save("worldY.npy", Y)
OPENRUA_EOF

# openrua op 9
python3 heightmap.py

# openrua op 10
python3 -c "
import cv2
img=cv2.imread('birdview.png'); crop=img[200:330,320:420]; cv2.imwrite('knob_bird.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
img=cv2.imread('agentview.png'); crop=img[190:370,340:520]; cv2.imwrite('knob_agent.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
img=cv2.imread('sideview.png'); crop=img[300:380,230:420]; cv2.imwrite('knob_side.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 11
python3 -c "
import numpy as np
X=np.load('worldX.npy');Y=np.load('worldY.npy');Z=np.load('heightZ.npy')
m=(X>-0.3)&(X<-0.15)&(Y>0.1)&(Y<0.26)&(Z>0.905)&(Z<1.05)
print('knob all: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] c=(%.3f,%.3f)'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max(),X[m].mean(),Y[m].mean()))
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,1.0),(1.0,1.05)]:
    mm=m&(Z>=lo)&(Z<hi)
    if mm.any(): print('z[%.3f,%.3f) n=%d x[%.3f,%.3f] y[%.3f,%.3f] c=(%.3f,%.3f)'%(lo,hi,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),X[mm].mean(),Y[mm].mean()))
# fin: highest part
fin=m&(Z>np.percentile(Z[m],80))
print('fin: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] c=(%.3f,%.3f)'%(X[fin].min(),X[fin].max(),Y[fin].min(),Y[fin].max(),Z[fin].min(),Z[fin].max(),X[fin].mean(),Y[fin].mean()))
# stove
s=(X>-0.15)&(X<0.1)&(Y>0.08)&(Y<0.3)&(Z>0.905)&(Z<1.0)
print('stove: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] c=(%.3f,%.3f)'%(X[s].min(),X[s].max(),Y[s].min(),Y[s].max(),Z[s].min(),Z[s].max(),X[s].mean(),Y[s].mean()))
# pan
p=(X>-0.2)&(X<0.1)&(Y>-0.35)&(Y<-0.02)&(Z>0.905)&(Z<1.1)
print('pan: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[p].min(),X[p].max(),Y[p].min(),Y[p].max(),Z[p].min(),Z[p].max()))
body=p&(Y<-0.13)
print('pan body c=(%.3f,%.3f) x[%.3f,%.3f] y[%.3f,%.3f]'%(X[body].mean(),Y[body].mean(),X[body].min(),X[body].max(),Y[body].min(),Y[body].max()))
h=p&(Y>-0.13)
print('pan handle c=(%.3f,%.3f) x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[h].mean(),Y[h].mean(),X[h].min(),X[h].max(),Y[h].min(),Y[h].max(),Z[h].min(),Z[h].max()))
"


# openrua op 12
timeout 300 python3 -u tools/action/gripper_cmd.py 0.04 2>&1 | tail -2; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position: | tail -2

# openrua op 13
nohup python3 -u tools/action/ik_move.py 0.461 0.189 0.108 1 0 0 0 4 --at tcp > move1.log 2>&1 &
sleep 20; cat /workspace/move1.log

# openrua op 14
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm helper: IK (MoveIt) -> FollowJointTrajectory, gripper, FK, joint read.

Usage:
  python3 arm.py js                                   # joint state dict
  python3 arm.py fk                                   # hand + tcp pose (world)
  python3 arm.py tcp <x> <y> <z> <qx> <qy> <qz> <qw> [sec]   # world TCP pose -> move
  python3 arm.py hand <x> <y> <z> <qx> <qy> <qz> <qw> [sec]  # world HAND pose -> move
  python3 arm.py joints <p1,...,p7> [sec]             # raw joint target
  python3 arm.py j7 <delta_rad> [sec]                 # rotate wrist joint7 by delta
  python3 arm.py grip <width_m>                       # gripper per-finger position
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
PLAN = M["planning"]
JOINTS = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(zip(m.name, m.position)), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gripper = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, PLAN["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
        while not all(j in self._js for j in JOINTS):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_state(self):
        js = self.joints()
        s = JointState()
        s.name = list(JOINTS)
        s.position = [js[j] for j in JOINTS]
        return s

    def fk_pose(self):
        self.fk.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP_OFF * quat_to_R(*q)[:, 2]
        return pos, q, tcp

    def solve_ik(self, pos_world, q):
        self.ik.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = PLAN["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = np.asarray(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK timeout")
        if res.error_code.val != 1:
            raise SystemExit(f"IK FAILED error_code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, positions, seconds):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(JOINTS, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_hand(self, pos_world, q, seconds):
        target = self.solve_ik(pos_world, q)
        print("IK ok ->", [round(v, 3) for v in target])
        return self.move_joints(target, seconds)

    def move_tcp(self, tcp_world, q, seconds):
        hand = np.asarray(tcp_world) - TCP_OFF * quat_to_R(*q)[:, 2]
        return self.move_hand(hand, q, seconds)

    def grip(self, width):
        if not self.gripper.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        js = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"f1={js.get('panda_finger_joint1'):.4f} f2={js.get('panda_finger_joint2'):.4f}")

    def report(self):
        pos, q, tcp = self.fk_pose()
        js = self.joints()
        print("joints:", [round(js[j], 4) for j in JOINTS],
              "fingers:", round(js.get("panda_finger_joint1", 0), 4),
              round(js.get("panda_finger_joint2", 0), 4))
        print("hand world:", np.round(pos, 4), "q:", np.round(q, 4))
        print("tcp  world:", np.round(tcp, 4))


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = a[0]
    if cmd == "js":
        js = arm.joints()
        for k in sorted(js):
            print(f"{k}: {js[k]:.5f}")
    elif cmd == "fk":
        arm.report()
    elif cmd in ("tcp", "hand"):
        pos = [float(v) for v in a[1:4]]
        q = [float(v) for v in a[4:8]]
        sec = float(a[8]) if len(a) > 8 else 4.0
        (arm.move_tcp if cmd == "tcp" else arm.move_hand)(pos, q, sec)
        arm.report()
    elif cmd == "joints":
        pos = [float(v) for v in a[1].split(",")]
        sec = float(a[2]) if len(a) > 2 else 4.0
        arm.move_joints(pos, sec)
        arm.report()
    elif cmd == "j7":
        delta = float(a[1])
        sec = float(a[2]) if len(a) > 2 else 3.0
        js = arm.joints()
        pos = [js[j] for j in JOINTS]
        pos[6] += delta
        arm.move_joints(pos, sec)
        arm.report()
    elif cmd == "grip":
        arm.grip(float(a[1]))
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -u arm.py fk 2>&1 | tail -5

# openrua op 16
cat > iktest.py <<'EOF'
import numpy as np, rclpy
from arm import Arm, BASE_IN_WORLD, JOINTS
arm = Arm()
js = arm.joints(); cur = [js[j] for j in JOINTS]
q = (1,0,0,0)
for label, pos in [("world-frame pose", np.array([-0.203,0,1.27])), ("base-frame pose (+BASE so solve_ik subtracts to base)", np.array([-0.203,0,1.27]) + BASE_IN_WORLD)]:
    try:
        sol = arm.solve_ik(pos, q)
        print(label, "-> max dev from current:", round(max(abs(a-b) for a,b in zip(sol,cur)),3), [round(v,3) for v in sol])
    except SystemExit as e:
        print(label, "->", e)
rclpy.shutdown()
EOF
timeout 200 python3 -u iktest.py 2>&1 | tail -4

# openrua op 17
python3 - <<'EOF'
import re
s = open('arm.py').read()
s = s.replace('        req.ik_request.group_name = PLAN["group"]\n',
              '        req.ik_request.group_name = PLAN["group"]\n        req.ik_request.ik_link_name = "panda_hand"\n')
s = s.replace('BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0',
              'BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0\n# NOTE: /compute_fk returns WORLD-frame poses; /compute_ik expects BASE-frame poses.')
s = s.replace('        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD\n',
              '        pos = np.array([p.position.x, p.position.y, p.position.z])  # already world\n')
open('arm.py','w').write(s)
EOF
timeout 200 python3 -u iktest.py 2>&1 | tail -3

# openrua op 18
nohup python3 -u arm.py tcp -0.199 0.189 1.02 1 0 0 0 4 > move1.log 2>&1 &
for i in $(seq 1 30); do sleep 5; if grep -q "tcp  world" move1.log; then break; fi; done; cat move1.log

# openrua op 19
cat > iktest2.py <<'EOF'
import numpy as np, rclpy, sys
from arm import Arm, JOINTS, TCP_OFF, quat_to_R
arm = Arm()
cands = [
 ("q=(1,0,0,0) z1.02", (-0.199,0.189,1.02), (1,0,0,0)),
 ("q cur-ish", (-0.199,0.189,1.02), (0.9996,0,-0.0284,0)),
 ("q yaw 45 (0.924,-0.383,0,0)", (-0.199,0.189,1.02), (0.9239,-0.3827,0,0)),
 ("q yaw -45", (-0.199,0.189,1.02), (0.9239,0.3827,0,0)),
 ("q=(1,0,0,0) z1.10", (-0.199,0.189,1.10), (1,0,0,0)),
 ("q=(1,0,0,0) y=0.10", (-0.199,0.10,1.02), (1,0,0,0)),
 ("q=(1,0,0,0) x=-0.1", (-0.1,0.189,1.02), (1,0,0,0)),
]
for label, tcp, q in cands:
    q = np.array(q)/np.linalg.norm(q)
    hand = np.asarray(tcp) - TCP_OFF * quat_to_R(*q)[:, 2]
    try:
        sol = arm.solve_ik(hand, q)
        print(label, "OK", [round(v,3) for v in sol])
    except SystemExit as e:
        print(label, "->", e)
rclpy.shutdown()
EOF
timeout 300 python3 -u iktest2.py 2>&1 | grep -v "^\[" | tail -8

# openrua op 20
cat > iktest3.py <<'EOF'
import numpy as np, rclpy
from arm import Arm
arm = Arm()
cur = np.array([-0.203,0,1.27]); q=(1,0,0,0)
for label, d in [("cur",(0,0,0)),("z-0.05",(0,0,-0.05)),("z-0.10",(0,0,-0.10)),("z-0.15",(0,0,-0.15)),("y+0.10",(0,0.10,0)),("y+0.19",(0,0.19,0)),("x+0.1",(0.1,0,0)),("x-0.1",(-0.1,0,0))]:
    try:
        sol = arm.solve_ik(cur+np.array(d), q); print(label,"OK",[round(v,3) for v in sol])
    except SystemExit as e: print(label,"->",e)
rclpy.shutdown()
EOF
timeout 300 python3 -u iktest3.py 2>&1 | grep -v "^\[" | tail -8

# openrua op 21
python3 - <<'EOF'
s = open('arm.py').read()
s = s.replace('# NOTE: /compute_fk returns WORLD-frame poses; /compute_ik expects BASE-frame poses.',
              '# NOTE (verified empirically): /compute_fk AND /compute_ik both use WORLD-frame\n# poses with frame_id="" on this machine (IK with base-frame coords fails -31).')
s = s.replace('        b = np.asarray(pos_world) - BASE_IN_WORLD\n        p.position.x, p.position.y, p.position.z = map(float, b)\n',
              '        p.position.x, p.position.y, p.position.z = map(float, np.asarray(pos_world))\n')
open('arm.py','w').write(s)
EOF
timeout 300 python3 -u iktest3.py 2>&1 | grep -v "^\[" | tail -8

# openrua op 22
nohup python3 -u arm.py tcp -0.199 0.189 1.02 1 0 0 0 4 > move1.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|Error|error" move1.log; then break; fi; done; grep -v "^\[" move1.log

# openrua op 23
sleep 5; grep -v "^\[" move1.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 24
nohup python3 -u arm.py tcp -0.199 0.189 0.947 1 0 0 0 3 > move2.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move2.log; then break; fi; done; sleep 2; grep -v "^\[" move2.log

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import cv2; img=cv2.imread('agentview.png'); cv2.imwrite('knob_agent2.png', cv2.resize(img[150:370,320:540],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 26
timeout 300 python3 -u arm.py grip 0.0 2>&1 | grep -v "^\["

# openrua op 27
nohup python3 -u arm.py j7 -1.5708 3 > rot1.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" rot1.log; then break; fi; done; sleep 2; grep -v "^\[" rot1.log; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null

# openrua op 28
python3 -c "
import cv2; img=cv2.imread('agentview.png'); cv2.imwrite('knob_agent3.png', cv2.resize(img[150:370,320:540],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
img=cv2.imread('birdview.png'); cv2.imwrite('knob_bird3.png', cv2.resize(img[200:330,320:420],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 29
timeout 300 python3 -u arm.py grip 0.04 2>&1 | grep -v "^\["; nohup python3 -u arm.py tcp -0.199 0.189 1.08 1 0 0 0 3 > move3.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move3.log; then break; fi; done; sleep 2; grep -v "^\[" move3.log; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png

# openrua op 30
python3 heightmap.py >/dev/null; python3 -c "
import numpy as np
X=np.load('worldX.npy');Y=np.load('worldY.npy');Z=np.load('heightZ.npy')
p=(X>-0.2)&(X<0.12)&(Y>-0.36)&(Y<-0.02)&(Z>0.903)&(Z<1.1)
print('pan all: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[p].min(),X[p].max(),Y[p].min(),Y[p].max(),Z[p].min(),Z[p].max()))
for lo,hi in [(0.903,0.91),(0.91,0.92),(0.92,0.93),(0.93,0.94),(0.94,0.95),(0.95,0.96),(0.96,0.98),(0.98,1.1)]:
    mm=p&(Z>=lo)&(Z<hi)
    if mm.any(): print('z[%.3f,%.3f) n=%d x[%.3f,%.3f] y[%.3f,%.3f] c=(%.3f,%.3f)'%(lo,hi,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),X[mm].mean(),Y[mm].mean()))
body=p&(Y<-0.13)
print('pan body c=(%.3f,%.3f) x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f'%(X[body].mean(),Y[body].mean(),X[body].min(),X[body].max(),Y[body].min(),Y[body].max(),Z[body].max()))
h=p&(Y>-0.13)&(X<0.02)
print('handle c=(%.3f,%.3f) x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[h].mean(),Y[h].mean(),X[h].min(),X[h].max(),Y[h].min(),Y[h].max(),Z[h].min(),Z[h].max()))
# handle cross-section in x at a few y
for yy in [-0.12,-0.10,-0.08,-0.06,-0.04]:
    mm=h&(np.abs(Y-yy)<0.005)
    if mm.any(): print(' y=%.2f: x[%.3f,%.3f] z[%.3f,%.3f]'%(yy,X[mm].min(),X[mm].max(),Z[mm].min(),Z[mm].max()))
s=(X>-0.15)&(X<0.1)&(Y>0.08)&(Y<0.3)&(Z>0.905)&(Z<1.0)
print('stove: c=(%.3f,%.3f) ztop=%.3f'%(X[s].mean(),Y[s].mean(),Z[s].max()))
"


# openrua op 31
nohup python3 -u arm.py tcp -0.058 -0.10 1.05 0.70710678 0.70710678 0 0 4 > move4.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move4.log; then break; fi; done; sleep 2; grep -v "^\[" move4.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 32
nohup python3 -u arm.py joints -0.179,0.437,0.015,-1.991,-0.009,2.427,-0.944 3 > move5.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move5.log; then break; fi; done; sleep 2; grep -v "^\[" move5.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 33
nohup python3 -u arm.py joints -0.179,0.437,0.015,-1.991,-0.009,2.427,-0.944 8 > move6.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move6.log; then break; fi; done; sleep 2; grep -v "^\[" move6.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 34
nohup python3 -u arm.py tcp -0.058 -0.10 0.928 0.70710678 0.70710678 0 0 3 > move7.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move7.log; then break; fi; done; sleep 2; grep -v "^\[" move7.log; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import cv2; img=cv2.imread('agentview.png'); cv2.imwrite('pan_agent.png', cv2.resize(img[150:370,100:380],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 35
timeout 300 python3 -u arm.py grip 0.0 2>&1 | grep -v "^\["

# openrua op 36
nohup python3 -u arm.py tcp -0.058 -0.10 1.14 0.70710678 0.70710678 0 0 4 > move8.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move8.log; then break; fi; done; sleep 2; grep -v "^\[" move8.log; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null

# openrua op 37
nohup python3 -u arm.py tcp -0.046 0.313 1.14 0.70710678 0.70710678 0 0 5 > move9.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move9.log; then break; fi; done; sleep 2; grep -v "^\[" move9.log; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null

# openrua op 38
python3 heightmap.py >/dev/null; python3 -c "
import numpy as np
X=np.load('worldX.npy');Y=np.load('worldY.npy');Z=np.load('heightZ.npy')
# pan body in the air: z between 1.05 and 1.20, exclude arm (arm is higher / at other x)
p=(Z>1.05)&(Z<1.20)&(Y>0.0)&(Y<0.45)&(X>-0.25)&(X<0.15)
print('airborne: n=%d x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p.sum(),X[p].min(),X[p].max(),Y[p].min(),Y[p].max(),Z[p].min(),Z[p].max()))
for lo,hi in [(1.05,1.08),(1.08,1.10),(1.10,1.12),(1.12,1.14),(1.14,1.17),(1.17,1.20)]:
    mm=p&(Z>=lo)&(Z<hi)
    if mm.any(): print('z[%.2f,%.2f) n=%d x[%.3f,%.3f] y[%.3f,%.3f] c=(%.3f,%.3f)'%(lo,hi,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),X[mm].mean(),Y[mm].mean()))
"

# openrua op 39
nohup python3 -u arm.py tcp -0.046 0.313 1.03 0.70710678 0.70710678 0 0 4 > move10.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move10.log; then break; fi; done; sleep 2; grep -v "^\[" move10.log; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 40
timeout 300 python3 -u arm.py grip 0.04 2>&1 | grep -v "^\["; nohup python3 -u arm.py tcp -0.046 0.313 1.15 0.70710678 0.70710678 0 0 4 > move11.log 2>&1 &
for i in $(seq 1 60); do sleep 5; if grep -q -E "tcp  world|FAILED|rror" move11.log; then break; fi; done; sleep 2; grep -v "^\[" move11.log; for c in agentview sideview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null

# openrua op 41
python3 heightmap.py >/dev/null; python3 -c "
import numpy as np
X=np.load('worldX.npy');Y=np.load('worldY.npy');Z=np.load('heightZ.npy')
p=(Z>0.905)&(Z<1.0)&(Y>0.05)&(Y<0.45)&(X>-0.15)&(X<0.1)
print('pan+stove region: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[p].min(),X[p].max(),Y[p].min(),Y[p].max(),Z[p].min(),Z[p].max()))
for lo,hi in [(0.905,0.93),(0.93,0.94),(0.94,0.95),(0.95,0.96),(0.96,0.97),(0.97,0.98),(0.98,1.0)]:
    mm=p&(Z>=lo)&(Z<hi)
    if mm.any(): print('z[%.3f,%.3f) n=%d x[%.3f,%.3f] y[%.3f,%.3f] c=(%.3f,%.3f)'%(lo,hi,mm.sum(),X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),X[mm].mean(),Y[mm].mean()))
rim=p&(Z>0.955)&(Y<0.30)
print('pan rim (z>0.955): x[%.3f,%.3f] y[%.3f,%.3f] center=(%.3f,%.3f) zrange[%.3f,%.3f]'%(X[rim].min(),X[rim].max(),Y[rim].min(),Y[rim].max(),(X[rim].min()+X[rim].max())/2,(Y[rim].min()+Y[rim].max())/2,Z[rim].min(),Z[rim].max()))
print('stove center (-0.046,0.191), stove top 0.930; pan floor expected ~0.935-0.94 if resting on plate')
"

# openrua op 42
python3 -c "
import cv2, numpy as np
img=cv2.imread('agentview.png'); crop=img[230:350,370:520]
cv2.imwrite('final_stove.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
# count strongly red pixels (burner glow) in the stove area
b,g,r=cv2.split(crop.astype(int)); red=((r>150)&(g<80)&(b<80)).sum(); print('red burner pixels visible around pan:', red)"
