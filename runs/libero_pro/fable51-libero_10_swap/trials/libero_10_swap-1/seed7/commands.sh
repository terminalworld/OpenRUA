#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" 

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
mkdir -p "$(dirname /workspace/grab.py)"
cat > /workspace/grab.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save npz + png.
Usage: grab.py <cam> [out_prefix]
Then: px.py <prefix> u v  -> world xyz (offline)
"""
import sys, json
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else f"snaps/{cam}"
    rclpy.init()
    node = rclpy.create_node("grab")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    img = br.imgmsg_to_cv2(color, "bgr8")
    dep = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    Tm = np.eye(4); Tm[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    Tm[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    cv2.imwrite(out + ".png", img)
    np.savez(out + ".npz", img=img, depth=dep, K=np.array(info.k).reshape(3, 3), T=Tm)
    print(out + ".png/.npz", "depth range", np.nanmin(dep), np.nanmax(dep))
    print("T world<-cam:\n", np.round(Tm, 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
mkdir -p "$(dirname /workspace/px.py)"
cat > /workspace/px.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Offline pixel -> world from a grab.py npz. Usage: px.py <prefix> u v [u v ...]"""
import sys
import numpy as np

d = np.load(sys.argv[1] + ".npz")
K, T, dep = d["K"], d["T"], d["depth"]
args = list(map(int, sys.argv[2:]))
for u, v in zip(args[::2], args[1::2]):
    z = float(dep[v, u])
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    w = T @ p
    print(f"px({u},{v}) depth={z:.4f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
OPENRUA_EOF

# openrua op 8
timeout 90 python3 grab.py robot0_eye_in_hand && timeout 90 python3 grab.py birdview && timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12

# openrua op 9
python3 px.py snaps/robot0_eye_in_hand 258 310 555 200 60 220 320 240 470 90 460 190 250 130 455 310; echo ---; python3 px.py snaps/birdview 320 240 385 288 264 290 290 290 278 312 320 305 335 305 285 262 280 232

# openrua op 10
mkdir -p "$(dirname /workspace/seg.py)"
cat > /workspace/seg.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table in a grab.py npz; print world centroid/extents.
Usage: seg.py <prefix> [min_h=0.01]"""
import sys
import numpy as np, cv2

d = np.load(sys.argv[1] + ".npz")
K, T, dep, img = d["K"], d["T"], d["depth"], d["img"]
min_h = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
TABLE = 0.4249
H, W = dep.shape
vs, us = np.mgrid[0:H, 0:W]
z = dep
X = (us - K[0, 2]) * z / K[0, 0]
Y = (vs - K[1, 2]) * z / K[1, 1]
P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
wz = P[..., 2]
mask = (np.isfinite(z) & (wz > TABLE + min_h) & (wz < TABLE + 0.35)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30:
        continue
    m = lab == i
    pts = P[m][:, :3]
    col = img[m].mean(0)[::-1]
    x0, y0, w, h = stats[i, :4]
    print(f"comp{i}: px bbox u[{x0},{x0+w}] v[{y0},{y0+h}] area={stats[i,4]} "
          f"cent={cents[i].round(0)} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
          f"y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} "
          f"mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) rgb={col.round(0)}")
OPENRUA_EOF

# openrua op 11
python3 seg.py snaps/birdview 0.012; echo ---; python3 seg.py snaps/robot0_eye_in_hand 0.012

# openrua op 12
mkdir -p "$(dirname /workspace/fk.py)"
cat > /workspace/fk.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print current hand pose (panda_hand) in base and world frames via /compute_fk."""
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

BASE_W = np.array([-0.51, 0.0, 0.42])
M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]

rclpy.init()
node = rclpy.create_node("fk")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
cur = dict(zip(js["m"].name, js["m"].position))
print("joints:", {k: round(v, 4) for k, v in cur.items()})
cli = node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(10)
req = GetPositionFK.Request()
req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = ARM
req.robot_state.joint_state.position = [cur[j] for j in ARM]
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
r = fut.result()
p = r.pose_stamped[0].pose
print("err", r.error_code.val, "frame", r.pose_stamped[0].header.frame_id)
pos = np.array([p.position.x, p.position.y, p.position.z])
q = p.orientation
print("hand base:", pos.round(4), "quat xyzw:", np.array([q.x, q.y, q.z, q.w]).round(4))
print("hand world:", (pos + BASE_W).round(4))
x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])
print("R (cols = hand x,y,z in base):\n", R.round(3))
print("tcp world:", (pos + BASE_W + 0.1034 * R[:, 2]).round(4))
rclpy.shutdown()
OPENRUA_EOF

# openrua op 13
timeout 60 python3 fk.py

# openrua op 14
mkdir -p "$(dirname /workspace/iktest.py)"
cat > /workspace/iktest.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""IK probe (no motion). Usage: iktest.py x y z qx qy qz qw"""
import sys
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]
x, y, z, qx, qy, qz, qw = map(float, sys.argv[1:8])
rclpy.init()
node = rclpy.create_node("iktest")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
cur = dict(zip(js["m"].name, js["m"].position))
cli = node.create_client(GetPositionIK, "/compute_ik")
cli.wait_for_service(10)
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"
req.ik_request.pose_stamped.header.frame_id = ""
p = req.ik_request.pose_stamped.pose
p.position.x, p.position.y, p.position.z = x, y, z
p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
req.ik_request.robot_state.joint_state.name = ARM
req.ik_request.robot_state.joint_state.position = [cur[j] for j in ARM]
req.ik_request.timeout.sec = 2
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
r = fut.result()
print("err", r.error_code.val)
if r.error_code.val == 1:
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    print("sol:", [round(sol[j], 4) for j in ARM])
    print("cur:", [round(cur[j], 4) for j in ARM])
rclpy.shutdown()
OPENRUA_EOF

# openrua op 15
echo "world coords:"; timeout 60 python3 iktest.py -0.053 0 0.7776 0.9996 0 -0.0284 0; echo "base coords:"; timeout 60 python3 iktest.py 0.457 0 0.3576 0.9996 0 -0.0284 0

# openrua op 16
python3 - <<'EOF'
import re
s = open("fk.py").read()
s = s.replace('cur = dict(zip(js["m"].name, js["m"].position))',
 'cur = dict(zip(js["m"].name, js["m"].position))\nimport sys\nif len(sys.argv) > 1:\n    cur.update(zip(ARM, map(float, sys.argv[1].split(","))))')
open("fk.py","w").write(s)
EOF
timeout 60 python3 fk.py 0.0585,-0.1613,-0.0579,-2.4447,-0.0116,2.2268,0.0089

# openrua op 17
sed -i 's/req.ik_request.group_name = "panda_arm"/req.ik_request.group_name = "panda_arm"\nreq.ik_request.ik_link_name = "panda_hand"/' iktest.py && timeout 60 python3 iktest.py -0.053 0 0.7776 0.9996 0 -0.0284 0

# openrua op 18
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot step runner. Each arg is one command; runs in order, stops on failure.

  move X Y Z [YAW_DEG] [SECS]   IK+trajectory; X Y Z = world TCP target (fingertips),
                                 hand pointing down; YAW = rotation of finger axis
                                 (0 -> fingers along world y, 90 -> along world x)
  grip open|close               gripper action; prints finger gap afterwards
  state                         joints, hand/TCP world pose, finger gap, wrench
"""
import sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


class Rob:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)

    def _js(self, m): self.js = m
    def _wr(self, m): self.wr = m

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin()
        return dict(zip(self.js.name, self.js.position))

    def gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def hand_pose(self):
        j = self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [j[a] for a in ARM]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def state(self):
        j = self.joints()
        pos, R = self.hand_pose()
        tcp = pos + TCP * R.as_matrix()[:, 2]
        print("arm:", [round(j[a], 4) for a in ARM])
        print("hand world:", pos.round(4), "TCP world:", tcp.round(4),
              "hand z-axis:", R.as_matrix()[:, 2].round(3))
        print("finger gap:", round(self.gap(), 4))
        if self.wr:
            f = self.wr.wrench.force
            print("wrench force:", round(f.x, 2), round(f.y, 2), round(f.z, 2))
        return tcp

    def solve_ik(self, tcp_xyz, yaw_deg):
        # hand pointing straight down; yaw rotates the finger axis about world z
        R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
        hand = np.array(tcp_xyz) - TCP * R.as_matrix()[:, 2]
        q = R.as_quat()
        j = self.joints()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = [j[a] for a in ARM]
        req.ik_request.timeout.sec = 3
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK FAILED", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def traj(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[a] - p) for a, p in zip(ARM, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code == 0 and err < 0.02

    def move(self, x, y, z, yaw=0.0, secs=3.0):
        sol = self.solve_ik([x, y, z], yaw)
        if sol is None:
            return False
        ok = self.traj(sol, secs)
        tcp = self.state()
        d = np.linalg.norm(tcp - np.array([x, y, z]))
        print(f"move -> TCP off-target by {d*1000:.1f} mm")
        return ok and d < 0.01

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GR["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        for _ in range(5):
            self.spin(0.1)
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} gap={self.gap():.4f}")
        return True


def main():
    r = Rob()
    for cmd in sys.argv[1:]:
        parts = cmd.split()
        print(f"\n== {cmd}")
        if parts[0] == "move":
            ok = r.move(*map(float, parts[1:]))
        elif parts[0] == "grip":
            ok = r.gripper(GR["open_m"] if parts[1] == "open" else GR["closed_m"])
        elif parts[0] == "state":
            r.state(); ok = True
        else:
            raise SystemExit(f"unknown {cmd}")
        if not ok:
            print("STEP FAILED, stopping")
            break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 19
timeout 300 python3 -u rob.py state "grip open" "move -0.096 0.063 0.62 0 3" 2>&1 | tee logs_step1.txt

# openrua op 20
timeout 90 python3 grab.py robot0_eye_in_hand snaps/eih1 >/dev/null && python3 seg.py snaps/eih1 0.012

# openrua op 21
timeout 400 python3 -u rob.py "move -0.096 0.063 0.44 0 2.5" "grip close" 2>&1 | tee logs_step2.txt

# openrua op 22
timeout 400 python3 -u rob.py "move -0.096 0.063 0.70 0 2.5" 2>&1 | tee logs_step3.txt && timeout 90 python3 grab.py robot0_eye_in_hand snaps/eih2 >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent2.png

# openrua op 23
timeout 600 python3 -u rob.py "move -0.01 0.27 0.72 0 3" "state" 2>&1 | tee logs_step4.txt

# openrua op 24
timeout 90 python3 grab.py robot0_eye_in_hand snaps/eih3 >/dev/null; python3 seg.py snaps/eih3 0.05

# openrua op 25
timeout 600 python3 -u rob.py "move -0.01 0.27 0.60 0 2" "grip open" "move -0.01 0.27 0.75 0 2" 2>&1 | tee logs_step5.txt

# openrua op 26
timeout 90 python3 grab.py robot0_eye_in_hand snaps/eih4 >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent3.png >/dev/null

# openrua op 27
timeout 600 python3 -u rob.py "move 0.016 -0.244 0.64 0 3" 2>&1 | tee logs_step6.txt && timeout 90 python3 grab.py robot0_eye_in_hand snaps/eih5 >/dev/null && python3 seg.py snaps/eih5 0.012

# openrua op 28
timeout 600 python3 -u rob.py "move 0.016 -0.244 0.44 0 2.5" "grip close" 2>&1 | tee logs_step7.txt

# openrua op 29
timeout 600 python3 -u rob.py "move 0.016 -0.244 0.70 0 2.5" 2>&1 | tee logs_step8.txt; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent4.png >/dev/null

# openrua op 30
timeout 900 python3 -u rob.py "move -0.01 0.27 0.72 0 4" 2>&1 | tee logs_step9.txt

# openrua op 31
timeout 600 python3 -u rob.py "move -0.01 0.27 0.61 0 2" "grip open" "move -0.01 0.27 0.78 0 2" 2>&1 | tee logs_step10.txt

# openrua op 32
timeout 90 python3 grab.py robot0_eye_in_hand snaps/eih6 >/dev/null; timeout 90 python3 grab.py birdview snaps/bird2 >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent5.png >/dev/null; python3 seg.py snaps/bird2 0.012

# openrua op 33
python3 px.py snaps/eih6 320 280 318 260 322 300 260 300 380 260

# openrua op 34
timeout 300 python3 -u tools/action/fjt_send.py 0,-0.161,0,-2.4446,0,2.2268,0.7854 3 && timeout 60 python3 -u rob.py state 2>&1 | tail -4
