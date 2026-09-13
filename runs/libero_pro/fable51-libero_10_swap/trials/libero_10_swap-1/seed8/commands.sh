#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | head -50 && echo ---NODES--- && ros2 node list

# openrua op 3
ros2 topic echo /joint_states --once 2>&1 | head -40; mkdir -p snaps; for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c snaps/$c.png; done

# openrua op 4
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | head -12; echo; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq

# openrua op 5
cd /workspace; echo butter; python3 tools/perception/px2world.py robot0_eye_in_hand 248 300; echo creamcheese; python3 tools/perception/px2world.py robot0_eye_in_hand 565 215; echo table_near_butter; python3 tools/perception/px2world.py robot0_eye_in_hand 320 300; echo basket_rim; python3 tools/perception/px2world.py robot0_eye_in_hand 60 200; echo milk_top; python3 tools/perception/px2world.py robot0_eye_in_hand 420 135

# openrua op 6
cd /workspace; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"; echo basket_center_bird; python3 tools/perception/px2world.py birdview 385 290; echo basket_bird_2; python3 tools/perception/px2world.py birdview 375 280; echo butter_left_edge; python3 tools/perception/px2world.py robot0_eye_in_hand 228 300; echo butter_right_edge; python3 tools/perception/px2world.py robot0_eye_in_hand 268 300; echo butter_top; python3 tools/perception/px2world.py robot0_eye_in_hand 248 268; echo butter_bottom; python3 tools/perception/px2world.py robot0_eye_in_hand 248 335

# openrua op 7
cd /workspace; for p in "548 215" "582 215" "565 178" "565 252" "565 200" "565 230"; do echo -n "cc $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done; echo; for p in "362 290" "408 290" "385 268" "385 312"; do echo -n "basket bird $p: "; python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
cd /workspace; for p in "540 120" "500 120" "575 100" "520 150"; do echo -n "OJ $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done; for p in "505 300" "470 290" "545 300" "505 265" "505 335"; do echo -n "alpha can $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done; for p in "420 135" "390 130" "450 150" "410 100"; do echo -n "milk $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done; for p in "245 140" "215 140" "275 140"; do echo -n "tomato can $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done

# openrua op 9
mkdir -p "$(dirname /workspace/pp.py)"
cat > /workspace/pp.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick-and-place helper: one node, reused clients, verify after each step.

Usage: python3 -u pp.py <cmd> [args]
  open | close                      gripper
  goto <wx> <wy> <wz> [secs]        move TCP to WORLD xyz, hand pointing down
  pose                              print hand/TCP world pose + finger gap
  pick <wx> <wy> <wz_grasp>         open, hover, descend, close, lift, report
  place <wx> <wy> <wz>              hover over target, open, report
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# world -> panda_link0 (read from TF at startup; fallback measured)
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])
# hand pointing straight down, fingers opening along world Y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)
HOVER_Z = 0.60


class PP:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("pp")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"
        self.spin_until(lambda: "m" in self.js, 10)
        global BASE_IN_WORLD
        t = self.tf("world", M["frames"]["base"])
        if t is not None:
            BASE_IN_WORLD = t[0]
        print(f"base in world: {BASE_IN_WORLD}")

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while not pred() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
            self.spin_until(lambda: "m" in self.js, 10)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def tf(self, parent, child):
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform(parent, child, rclpy.time.Time()):
                break
        else:
            return None
        t = self.tfbuf.lookup_transform(parent, child, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def pose(self):
        # TF for the arm chain is sim-stamped; force a fresh listener read
        hp = self.tf("world", M["frames"]["hand"])
        j = self.joints()
        gap = j["panda_finger_joint1"] + abs(j["panda_finger_joint2"])
        if hp is None:
            print("hand pose: TF unavailable")
            return None
        p, q = hp
        R = quat_R(*q)
        tcp = p + TCP_OFF * R[:, 2]
        print(f"hand world {np.round(p, 4)} q {np.round(q, 3)}  TCP world {np.round(tcp, 4)}"
              f"  finger gap {gap:.4f}")
        return tcp, gap

    def solve_ik(self, world_xyz, q=Q_DOWN):
        # target for the HAND frame: TCP shifted back along hand +Z
        R = quat_R(*q)
        hand_w = np.array(world_xyz) - TCP_OFF * R[:, 2]
        hand_b = hand_w - BASE_IN_WORLD  # base frame is a pure translation
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.avoid_collisions = False
        seed = JointState()
        cur = self.joints()
        for n in ARM:
            seed.name.append(n)
            seed.position.append(cur[n])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.timeout = Duration(sec=2)
        for attempt in range(5):
            fut = self.ik.call_async(req)
            self.spin_until(fut.done, 60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print(f"  IK attempt {attempt} failed: "
                  f"{None if res is None else res.error_code.val}")
        raise SystemExit(f"IK failed for world TCP {world_xyz}")

    def move_joints(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.fjt.send_goal_async(goal)
        self.spin_until(f.done, 60)
        rf = f.result().get_result_async()
        self.spin_until(rf.done, 600)
        code = rf.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - p) for j, p in zip(ARM, positions))
        print(f"  traj error_code={code} max joint err={err:.4f}")
        return code, err

    def goto(self, world_xyz, secs=3.0, q=Q_DOWN):
        print(f"goto TCP world {np.round(world_xyz, 4)}")
        sol = self.solve_ik(world_xyz, q)
        code, err = self.move_joints(sol, secs)
        if code != 0 or err > 0.02:
            print("  retrying same goal once")
            self.move_joints(sol, secs)
        return self.pose()

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(goal)
        self.spin_until(f.done, 30)
        rf = f.result().get_result_async()
        self.spin_until(rf.done, 300)
        r = rf.result().result
        j = self.joints()
        gap = j["panda_finger_joint1"] + abs(j["panda_finger_joint2"])
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={gap:.4f}")
        return gap

    def pick(self, x, y, zg):
        self.gripper(GRIP["open_m"])
        self.goto((x, y, HOVER_Z), 4.0)
        self.goto((x, y, zg + 0.08), 2.0)
        self.goto((x, y, zg), 2.0)
        gap = self.gripper(GRIP["closed_m"])
        self.goto((x, y, HOVER_Z), 2.5)
        gap2 = self.joints()
        g = gap2["panda_finger_joint1"] + abs(gap2["panda_finger_joint2"])
        print(f"PICK RESULT: gap after close {gap:.4f}, after lift {g:.4f} "
              f"({'HOLDING' if g > 0.005 else 'EMPTY'})")
        return g > 0.005

    def place(self, x, y, z):
        self.goto((x, y, z), 4.0)
        self.gripper(GRIP["open_m"])
        self.pose()


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    pp = PP()
    cmd = a[0]
    if cmd == "open":
        pp.gripper(GRIP["open_m"])
    elif cmd == "close":
        pp.gripper(GRIP["closed_m"])
    elif cmd == "pose":
        pp.pose()
    elif cmd == "goto":
        secs = float(a[4]) if len(a) > 4 else 3.0
        pp.goto(tuple(map(float, a[1:4])), secs)
    elif cmd == "pick":
        pp.pick(*map(float, a[1:4]))
    elif cmd == "place":
        pp.place(*map(float, a[1:4]))
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
python3 -u pp.py pose 2>&1 | tail -5

# openrua op 11
python3 -u pp.py goto -0.087 0.079 0.60 4 > goto1.log 2>&1; tail -6 goto1.log

# openrua op 12
cd /workspace; cat > fk.py <<'EOF'
import rclpy, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("fk")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
cli = n.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service(10)
req = GetPositionFK.Request()
req.fk_link_names = sys.argv[1:] or ["panda_hand", "panda_link8", "panda_hand_tcp"]
seed = JointState()
for nm, p in zip(js["m"].name, js["m"].position):
    if nm.startswith("panda_joint"): seed.name.append(nm); seed.position.append(p)
req.robot_state.joint_state = seed
f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
r = f.result()
print("error", r.error_code.val)
for nm, ps in zip(r.fk_link_names, r.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    print(nm, ps.header.frame_id, f"{p.x:.4f} {p.y:.4f} {p.z:.4f}", f"q {q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f}")
EOF
python3 fk.py; ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -iE "group name|chain|tip|end_effector" | head

# openrua op 13
cd /workspace; cat > iktest.py <<'EOF'
import rclpy, sys
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
rclpy.init(); n = rclpy.create_node("iktest")
js = {}
n.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(n, timeout_sec=0.2)
cli = n.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service(10)
cur = {nm: p for nm, p in zip(js["m"].name, js["m"].position) if nm.startswith("panda_joint")}
def ik(x, y, z, q, link=""):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    if link: req.ik_request.ik_link_name = link
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    seed = JointState(); seed.name = list(cur); seed.position = list(cur.values())
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.timeout = Duration(sec=2)
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=60)
    r = f.result()
    if r is None: print("  no answer"); return
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    diff = max(abs(sol[j]-cur[j]) for j in cur) if r.error_code.val == 1 else None
    print(f"  code={r.error_code.val} maxdiff_from_current={diff}")
q8 = (0.924, -0.383, -0.026, 0.011)
print("link8 world coords:"); ik(-0.053, 0.0, 0.7776, q8)
print("link8 base coords:"); ik(0.457, 0.0, 0.3576, q8)
print("hand-orientation q=(1,0,0,0) world coords (expected wrong tip orientation):"); ik(-0.053, 0.0, 0.7776, (1.0,0,0,0))
EOF
python3 iktest.py

# openrua op 14
cd /workspace; sed -i 's/p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q/p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)/' iktest.py; python3 iktest.py

# openrua op 15
cd /workspace; python3 - <<'EOF'
import re
s = open("pp.py").read()
s = s.replace('''# hand pointing down, fingers opening along world Y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)''', '''# IK tip link is panda_link8 (= panda_hand yawed +45deg about z). This is
# link8's orientation for hand pointing down, fingers opening along world Y.
Q_DOWN = (0.9238795, -0.3826834, 0.0, 0.0)''')
s = s.replace('''        hand_b = hand_w - BASE_IN_WORLD  # base frame is a pure translation
''', '''        # verified: with empty frame_id this machine's IK takes WORLD coords
        hand_b = hand_w
''')
s = s.replace("p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q",
              "p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)")
open("pp.py","w").write(s)
EOF
grep -n "Q_DOWN =\|hand_b =\|map(float, q)" pp.py

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pp.py", "old_string": "# hand pointing straight down, fingers opening along world Y\\nQ_DOWN = (1.0, 0.0, 0.0, 0.0)", "new_string": "# IK tip link is panda_link8 (= panda_hand yawed +45deg about z). This is\\n# link8\'s orientation for hand pointing straight down, fingers along world Y.\\nQ_DOWN = (0.9238795, -0.3826834, 0.0, 0.0)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
cd /workspace; python3 -u pp.py goto -0.087 0.079 0.60 4 > goto1.log 2>&1; tail -6 goto1.log

# openrua op 18
cd /workspace; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_butter_hover.png

# openrua op 19
cd /workspace; for p in "322 300" "322 262" "322 340" "300 300" "345 300"; do echo -n "$p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done

# openrua op 20
cd /workspace; python3 -u pp.py pick -0.086 0.077 0.44 > pick1.log 2>&1; cat pick1.log

# openrua op 21
cd /workspace; python3 tools/perception/cam_snap.py agentview snaps/agent_butter_lift.png

# openrua op 22
cd /workspace; python3 -u pp.py place 0.02 0.29 0.62 > place1.log 2>&1; cat place1.log; python3 tools/perception/cam_snap.py agentview snaps/agent_after_place1.png

# openrua op 23
cd /workspace; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_after_place1.png; python3 tools/perception/cam_snap.py birdview snaps/bird_after_place1.png

# openrua op 24
cd /workspace; python3 -u pp.py goto 0.004 -0.253 0.60 4 > goto2.log 2>&1; tail -3 goto2.log; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_cc_hover.png

# openrua op 25
cd /workspace; cat goto2.log; ros2 topic echo /joint_states --once | grep -A9 "^position"

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pp.py", "old_string": "        req.ik_request.avoid_collisions = False\\n        seed = JointState()\\n        cur = self.joints()\\n        for n in ARM:\\n            seed.name.append(n)\\n            seed.position.append(cur[n])\\n        req.ik_request.robot_state.joint_state = seed\\n        req.ik_request.timeout = Duration(sec=2)\\n        for attempt in range(5):\\n            fut = self.ik.call_async(req)\\n            self.spin_until(fut.done, 60)\\n            res = fut.result()\\n            if res is not None and res.error_code.val == 1:\\n                sol = dict(zip(res.solution.joint_state.name,\\n                               res.solution.joint_state.position))\\n                return [sol[j] for j in ARM]\\n            print(f\\"  IK attempt {attempt} failed: \\"\\n                  f\\"{None if res is None else res.error_code.val}\\")\\n        raise SystemExit(f\\"IK failed for world TCP {world_xyz}\\")", "new_string": "        req.ik_request.avoid_collisions = False\\n        req.ik_request.timeout = Duration(sec=2)\\n        # Seed from the SRDF \\"ready\\" pose with joint1 aimed at the target:\\n        # seeding from the current config can return a twisted branch\\n        # (huge joint travel, tolerance violations).\\n        d = hand_w - BASE_IN_WORLD\\n        j1 = float(np.arctan2(d[1], d[0]))\\n        natural = np.array([j1, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])\\n        best = None\\n        rng = np.random.default_rng(0)\\n        for attempt in range(8):\\n            seedv = natural if attempt == 0 else natural + rng.normal(0, 0.3, 7)\\n            seed = JointState()\\n            seed.name = list(ARM)\\n            seed.position = [float(v) for v in seedv]\\n            req.ik_request.robot_state.joint_state = seed\\n            fut = self.ik.call_async(req)\\n            self.spin_until(fut.done, 60)\\n            res = fut.result()\\n            if res is None or res.error_code.val != 1:\\n                print(f\\"  IK attempt {attempt} failed: \\"\\n                      f\\"{None if res is None else res.error_code.val}\\")\\n                continue\\n            sol = dict(zip(res.solution.joint_state.name,\\n                           res.solution.joint_state.position))\\n            solv = np.array([sol[j] for j in ARM])\\n            dev = np.max(np.abs(solv - natural))\\n            if best is None or dev < best[0]:\\n                best = (dev, solv)\\n            if dev < 1.0:\\n                break\\n        if best is None:\\n            raise SystemExit(f\\"IK failed for world TCP {world_xyz}\\")\\n        print(f\\"  IK sol {np.round(best[1], 3)} (dev from natural {best[0]:.2f})\\")\\n        return list(best[1])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pp.py", "old_string": "        sol = self.solve_ik(world_xyz, q)\\n        code, err = self.move_joints(sol, secs)", "new_string": "        sol = self.solve_ik(world_xyz, q)\\n        cur = self.joints()\\n        travel = max(abs(cur[j] - p) for j, p in zip(ARM, sol))\\n        secs = max(secs, travel / 0.4)  # cap joint speed ~0.4 rad/s\\n        print(f\\"  travel {travel:.2f} rad -> {secs:.1f}s\\")\\n        code, err = self.move_joints(sol, secs)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
cd /workspace; python3 -u pp.py goto 0.004 -0.253 0.60 4 > goto2.log 2>&1; cat goto2.log

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pp.py", "old_string": "        secs = max(secs, travel / 0.4)  # cap joint speed ~0.4 rad/s\\n        print(f\\"  travel {travel:.2f} rad -> {secs:.1f}s\\")\\n        code, err = self.move_joints(sol, secs)\\n        if code != 0 or err > 0.02:\\n            print(\\"  retrying same goal once\\")\\n            self.move_joints(sol, secs)\\n        return self.pose()", "new_string": "        secs = max(secs, travel / 0.3)  # cap joint speed ~0.3 rad/s\\n        print(f\\"  travel {travel:.2f} rad -> {secs:.1f}s\\")\\n        code, err = self.move_joints(sol, secs)\\n        for _ in range(4):\\n            if code == 0 and err < 0.01:\\n                break\\n            print(\\"  resending same goal (controller lag)\\")\\n            code, err = self.move_joints(sol, max(2.0, secs / 2))\\n        return self.pose()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
cd /workspace; python3 -u pp.py goto 0.004 -0.253 0.60 4 > goto2.log 2>&1; cat goto2.log; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_cc_hover.png

# openrua op 31
cd /workspace; python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
import re
rclpy.init(); n = rclpy.create_node("urdf")
got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), qos)
while "m" not in got: rclpy.spin_once(n, timeout_sec=0.5)
u = got["m"].data
for link in ["panda_hand", "panda_leftfinger", "panda_rightfinger"]:
    m = re.search(rf'<link name="{link}">(.*?)</link>', u, re.S)
    body = m.group(1)
    for c in re.findall(r'<collision>(.*?)</collision>', body, re.S):
        print(link, re.sub(r'\s+', ' ', c)[:300])
m = re.search(r'<joint name="panda_hand_joint".*?</joint>', u, re.S); print(re.sub(r'\s+',' ',m.group(0))[:300])
m = re.search(r'<joint name="panda_finger_joint1".*?</joint>', u, re.S); print(re.sub(r'\s+',' ',m.group(0))[:300])
EOF

# openrua op 32
cd /workspace; python3 - <<'EOF'
import struct, numpy as np
for f in ["hand", "finger"]:
    p = f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl"
    d = open(p, "rb").read()
    if d[:5] == b"solid":
        import re
        v = np.array([list(map(float, m)) for m in re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", d)])
    else:
        n = struct.unpack_from("<I", d, 80)[0]
        v = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([("n", "<3f"), ("v", "<9f"), ("a", "<u2")]))["v"].reshape(-1, 3)
    print(f, "min", v.min(0).round(4), "max", v.max(0).round(4))
EOF

# openrua op 33
cd /workspace; for p in "205 235" "250 232" "160 230" "205 180" "270 180"; do echo -n "OJ $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done; for p in "230 375" "255 370" "265 385" "240 355"; do echo -n "can $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done; for p in "325 300" "325 262" "325 340" "308 300" "342 300"; do echo -n "cc $p: "; python3 tools/perception/px2world.py robot0_eye_in_hand $p; done

# openrua op 34
cd /workspace; python3 -u pp.py pick 0.006 -0.257 0.44 > pick2.log 2>&1; cat pick2.log

# openrua op 35
cd /workspace; python3 tools/perception/cam_snap.py agentview snaps/agent_cc_lift.png

# openrua op 36
cd /workspace; python3 -u pp.py place 0.04 0.30 0.62 > place2.log 2>&1; cat place2.log; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_after_place2.png

# openrua op 37
cd /workspace; python3 -u pp.py goto -0.03 0.29 0.78 4 > goto3.log 2>&1; tail -2 goto3.log; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_final.png; python3 tools/perception/cam_snap.py agentview snaps/agent_final.png; python3 tools/perception/cam_snap.py birdview snaps/bird_final.png

# openrua op 38
cd /workspace; echo -n "cream cheese: "; python3 tools/perception/px2world.py robot0_eye_in_hand 312 225; echo -n "butter: "; python3 tools/perception/px2world.py robot0_eye_in_hand 320 258; echo -n "basket floor: "; python3 tools/perception/px2world.py robot0_eye_in_hand 380 300; echo -n "basket rim: "; python3 tools/perception/px2world.py robot0_eye_in_hand 258 260
