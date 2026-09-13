#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ------; cat /workspace/tools/perception/cam_snap.py; echo ------; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ------; cat tools/action/gripper_cmd.py; echo ------; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ----; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump TF world->panda_link0, camera frames, hand pose (via TF), joint states."""
import rclpy, yaml, numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("scene")
buf = Buffer(); TransformListener(buf, node)
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = ["panda_link0", "panda_hand", "agentview_optical_frame", "birdview_optical_frame",
          "frontview_optical_frame", "sideview_optical_frame", "robot0_eye_in_hand_optical_frame",
          "robot0_robotview_optical_frame"]
for f in frames:
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{f:36s} xyz=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"{f:36s} FAIL {str(e)[:80]}")
if "m" in js:
    print(dict(zip(js["m"].name, [round(p, 4) for p in js["m"].position])))
print(buf.all_frames_as_string())
rclpy.shutdown()
OPENRUA_EOF

# openrua op 7
timeout 60 python3 scene.py

# openrua op 8
mkdir -p "$(dirname /workspace/locate.py)"
cat > /workspace/locate.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table from a top-down camera; print world xyz clusters.
Usage: python3 locate.py <camera> [min_height_above_table]
"""
import sys, struct
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from cv_bridge import CvBridge

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
minh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.015

rclpy.init(); node = rclpy.create_node("locate")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while not all(k in got for k in "dci") or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
depth = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
K = np.array(got["i"].k).reshape(3, 3)
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - K[0, 2]) * Z / K[0, 0]; Y = (vv - K[1, 2]) * Z / K[1, 1]
P = np.stack([X, Y, Z], -1) @ R.T + T
valid = np.isfinite(Z) & (Z > 0.05)
# table height = mode of z among valid points in the central region
zs = P[..., 2][valid]
hist, edges = np.histogram(zs, bins=400, range=(0, 2))
table_z = edges[np.argmax(hist)] + (edges[1]-edges[0])/2
print(f"table_z ~ {table_z:.4f}")
mask = valid & (P[..., 2] > table_z + minh) & (P[..., 2] < table_z + 0.5)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
out = color.copy()
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 15: continue
    m = lab == i
    pts = P[m]
    col = color[m].mean(0)
    cx, cy = cents[i]
    print(f"cluster {i}: px=({cx:.0f},{cy:.0f}) area={stats[i,4]} "
          f"world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"ztop={pts[:,2].max():.3f} zmean={pts[:,2].mean():.3f} centre=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}")
    cv2.rectangle(out, (stats[i,0], stats[i,1]), (stats[i,0]+stats[i,2], stats[i,1]+stats[i,3]), (0,255,0), 1)
    cv2.putText(out, str(i), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
cv2.imwrite(f"{cam}_seg.png", out)
rclpy.shutdown()
OPENRUA_EOF

# openrua op 9
timeout 90 python3 locate.py birdview 0.015

# openrua op 10
sed -i 's/^table_z = edges.*/table_z = float(sys.argv[3]) if len(sys.argv) > 3 else edges[np.argmax(hist)] + (edges[1]-edges[0])\/2/' locate.py && timeout 90 python3 locate.py birdview 0.012 0.42

# openrua op 11
for p in "200 230" "450 200" "329 330"; do timeout 60 python3 tools/perception/px2world.py birdview $p; done; echo ---; timeout 60 python3 tools/perception/px2world.py agentview 300 400

# openrua op 12
timeout 90 python3 locate.py agentview 0.02 0.425

# openrua op 13
timeout 90 python3 locate.py robot0_eye_in_hand 0.015 0.425

# openrua op 14
for p in "345 252" "368 252" "356 246" "356 262" "356 257"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p; done

# openrua op 15
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm controller for this Panda: TCP-in-world IK moves, gripper, state.

Usage:
  python3 arm.py state                      # hand/TCP world pose + joints
  python3 arm.py ikcheck                    # IK on current pose: frame sanity check
  python3 arm.py move X Y Z [SECS] [--q qx qy qz qw]   # TCP world pose (default hand pointing down, fingers along world y)
  python3 arm.py grip open|close
"""
import sys, time
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
DOWN_Q = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.buf = Buffer(); TransformListener(self.buf, self.node)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.spin(1.0)
        while not self.buf.can_transform("world", "panda_link0", Time()) or "m" not in self.js:
            self.spin(0.2)
        t = self.buf.lookup_transform("world", "panda_link0", Time()).transform.translation
        self.base = np.array([t.x, t.y, t.z])

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def hand_world(self):
        # force a fresh TF sample after motion
        self.spin(0.3)
        t = self.buf.lookup_transform("world", "panda_hand", Time())
        p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        return p, (q.x, q.y, q.z, q.w), p + TCP * R[:, 2]

    def state(self):
        p, q, tcp = self.hand_world()
        j = self.joints()
        print(f"hand world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) q=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
        print(f"TCP  world=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f})")
        print("arm:", [round(j[n], 4) for n in JOINTS])
        print("fingers:", round(j.get("panda_finger_joint1", 0), 4), round(j.get("panda_finger_joint2", 0), 4))
        return p, q, tcp, j

    def solve_ik(self, hand_world_p, q):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pb = np.asarray(hand_world_p) - self.base
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, pb)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
        seed = JointState()
        j = self.joints()
        for n in JOINTS:
            seed.name.append(n); seed.position.append(j[n])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in JOINTS]

    def traj(self, positions, secs):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - p) for n, p in zip(JOINTS, positions))
        print(f"traj done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, secs=3.0, q=DOWN_Q):
        R = quat_R(*q)
        hand = np.asarray(xyz, float) - TCP * R[:, 2]
        sol = self.solve_ik(hand, q)
        if sol is None:
            print(f"IK FAILED for TCP {xyz}")
            return False
        lim = FJT["limits_rad"]
        for n, p, (lo, hi) in zip(JOINTS, sol, lim):
            if not lo <= p <= hi:
                print(f"IK solution violates limit {n}={p:.3f} not in [{lo},{hi}]")
                return False
        code, err = self.traj(sol, secs)
        if err > 0.02:
            print("large tracking error, resending")
            code, err = self.traj(sol, secs)
        p, _, tcp = self.hand_world()
        d = np.linalg.norm(tcp - np.asarray(xyz))
        print(f"TCP now ({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) target {tuple(round(v,4) for v in xyz)} dist={d:.4f}")
        return d < 0.01

    def gripper(self, open_):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        f1, f2 = j["panda_finger_joint1"], j["panda_finger_joint2"]
        print(f"gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} fingers={f1:.4f},{f2:.4f} gap={abs(f1)+abs(f2):.4f}")
        return f1, f2


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "state":
        a.state()
    elif cmd == "ikcheck":
        p, q, tcp, j = a.state()
        sol = a.solve_ik(p, q)
        print("IK of current pose:", None if sol is None else [round(v, 4) for v in sol])
    elif cmd == "move":
        args = [x for x in sys.argv[2:] if not x.startswith("--")]
        xyz = list(map(float, args[:3])); secs = float(args[3]) if len(args) > 3 else 3.0
        q = DOWN_Q
        if "--q" in sys.argv:
            i = sys.argv.index("--q"); q = tuple(map(float, sys.argv[i+1:i+5]))
        ok = a.move_tcp(xyz, secs, q)
        print("OK" if ok else "NOT AT TARGET")
    elif cmd == "grip":
        a.gripper(sys.argv[2] == "open")
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 16
timeout 120 python3 arm.py ikcheck

# openrua op 17
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        self.spin(0.3)\\n        t = self.buf.lookup_transform(\\"world\\", \\"panda_hand\\", Time())", "new_string": "        self.spin(0.3)\\n        end = time.time() + 15\\n        while not self.buf.can_transform(\\"world\\", \\"panda_hand\\", Time()) and time.time() < end:\\n            self.spin(0.2)\\n        t = self.buf.lookup_transform(\\"world\\", \\"panda_hand\\", Time())", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 18
timeout 120 python3 arm.py ikcheck

# openrua op 19
cat > /workspace/iktest.py <<'EOF'
import sys; sys.argv=["x","state"]
from arm import *
a = Arm()
p, q, tcp, j = a.state()
import numpy as np
for label, off in [("base-frame", a.base), ("world-frame", np.zeros(3))]:
    sol = a.solve_ik(p + a.base - off if label=="world-frame" else p, q)
    # note: solve_ik subtracts base internally; for world-frame test add base back
    print(label, None if sol is None else [round(v,3) for v in sol])
# also try with the exact quaternion but z slightly different
for dz in (0.0, 0.05, -0.05):
    sol = a.solve_ik(p + [0,0,dz], (1.0,0.0,0.0,0.0))
    print("dz", dz, None if sol is None else [round(v,3) for v in sol])
rclpy.shutdown()
EOF
timeout 300 python3 iktest.py

# openrua op 20
cat > /workspace/iktest2.py <<'EOF'
import sys; sys.argv=["x","state"]
from arm import *
a = Arm()
t = a.buf.lookup_transform("world", "panda_link8", Time())
q = t.transform.rotation; tr = t.transform.translation
print("link8 world", (round(tr.x,4),round(tr.y,4),round(tr.z,4)), (round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4)))
p = np.array([tr.x,tr.y,tr.z])
sol = a.solve_ik(p + a.base, (q.x,q.y,q.z,q.w))
print("IK link8 world-frame:", None if sol is None else [round(v,3) for v in sol])
print("current:", [round(a.joints()[n],3) for n in JOINTS])
rclpy.shutdown()
EOF
timeout 300 python3 iktest2.py

# openrua op 21
sed -i 's/^t = a.buf.lookup_transform("world", "panda_link8", Time())/a.hand_world()\nt = a.buf.lookup_transform("world", "panda_link8", Time())/' iktest2.py && timeout 300 python3 iktest2.py

# openrua op 22
python3 - <<'EOF'
s = open("arm.py").read()
s = s.replace('''        pb = np.asarray(hand_world_p) - self.base
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, pb)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)''',
'''        # machine fact (verified): IK tip link is panda_link8, poses are in
        # WORLD coordinates. link8 = hand * Rz(+45deg) (same origin).
        pb = np.asarray(hand_world_p)
        q8 = quat_mul(q, (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, pb)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q8)''')
s = s.replace('''class Arm:''', '''def quat_mul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:''')
open("arm.py","w").write(s)
EOF
timeout 120 python3 arm.py ikcheck

# openrua op 23
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick object at (x,y) grasping at TCP height gz, drop into basket.
Usage: python3 pick_place.py X Y GZ [yaw_deg]  (yaw rotates the fingers about world z)
"""
import sys
sys.argv_saved = list(sys.argv)
from arm import *

x, y, gz = map(float, sys.argv[1:4])
yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
BASKET = (0.0, 0.255)
HOVER = 0.66          # TCP hover above the objects
CARRY = 0.80          # TCP height while carrying (basket rim 0.628, ketchup ~13cm)
q = quat_mul((0.0, 0.0, np.sin(np.radians(yaw) / 2), np.cos(np.radians(yaw) / 2)), DOWN_Q)

a = Arm()
a.state()
print("== open gripper"); a.gripper(True)
print("== hover"); assert a.move_tcp((x, y, HOVER), 3.0, q), "hover failed"
print("== descend"); assert a.move_tcp((x, y, gz), 2.5, q), "descend failed"
print("== close"); f1, f2 = a.gripper(False)
gap = abs(f1) + abs(f2)
print(f"grasp gap={gap:.4f}")
if gap < 0.005:
    print("GRASP FAILED: closed on air"); a.gripper(True); a.move_tcp((x, y, HOVER), 2.5, q); sys.exit(2)
print("== lift"); assert a.move_tcp((x, y, CARRY), 2.5, q), "lift failed"
j = a.joints(); print("fingers after lift:", round(j["panda_finger_joint1"], 4), round(j["panda_finger_joint2"], 4))
print("== to basket"); assert a.move_tcp((BASKET[0], BASKET[1], CARRY), 3.5, q), "basket move failed"
print("== lower a bit"); a.move_tcp((BASKET[0], BASKET[1], 0.74), 2.0, q)
print("== release"); a.gripper(True)
print("== retreat"); a.move_tcp((BASKET[0], BASKET[1], CARRY), 2.0, q)
a.state()
print("DONE")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 24
nohup python3 -u pick_place.py -0.200 -0.126 0.49 > ketchup.log 2>&1 &
sleep 45; cat /workspace/ketchup.log

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 arm.py state

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py frontview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
cat >> /workspace/arm.py <<'EOF'


def plan_only(a, xyz, q=DOWN_Q):
    R = quat_R(*q)
    hand = np.asarray(xyz, float) - TCP * R[:, 2]
    sol = a.solve_ik(hand, q)
    j = a.joints()
    cur = [j[n] for n in JOINTS]
    if sol is None:
        print("IK FAILED", xyz); return None
    print("sol ", [round(v, 3) for v in sol])
    print("cur ", [round(v, 3) for v in cur])
    print("dq  ", [round(s - c, 3) for s, c in zip(sol, cur)])
    return sol
EOF
cat > /workspace/lift.py <<'EOF'
import sys; sys.argv=["x","state"]
from arm import *
a = Arm()
p, q, tcp, j = a.state()
# lift straight up 12 cm keeping current orientation
sol = plan_only(a, tcp + [0, 0, 0.12], q)
if sol: a.traj(sol, 2.5)
a.state()
print("--- IK for hover/descend from here:")
plan_only(a, (-0.200, -0.126, 0.66))
plan_only(a, (-0.200, -0.126, 0.49))
rclpy.shutdown()
EOF
timeout 600 python3 lift.py 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 28
timeout 90 python3 locate.py birdview 0.012 0.425 2>&1 | grep -v "cluster 1:"; echo ---; timeout 90 python3 locate.py agentview 0.02 0.425 | grep -v "cluster 1:"

# openrua op 29
timeout 300 python3 arm.py move -0.30 0.0 0.80 3 2>&1 | tail -3 && timeout 90 python3 locate.py birdview 0.012 0.425 2>&1 | grep -v "cluster 1:" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
cat > /workspace/bottle_axis.py <<'EOF'
import sys, numpy as np, rclpy, cv2
sys.argv = ["x", "birdview", "0.012", "0.425"]
# reuse locate.py internals by exec'ing up to the mask
src = open("locate.py").read().split("n, lab, stats")[0]
exec(src)
# select region of interest: ketchup approx
roi = mask.astype(bool) & (P[...,0] > -0.25) & (P[...,0] < 0.0) & (P[...,1] > -0.26) & (P[...,1] < -0.08) & (P[...,2] < 0.50)
pts = P[roi]
print("n pts", len(pts), "ztop", pts[:,2].max())
xy = pts[:, :2]
c = xy.mean(0)
u, s, vt = np.linalg.svd(xy - c, full_matrices=False)
axis = vt[0]; ang = np.degrees(np.arctan2(axis[1], axis[0]))
proj = (xy - c) @ axis
print(f"centroid=({c[0]:.4f},{c[1]:.4f}) axis angle={ang:.1f} deg length={proj.max()-proj.min():.3f} width={((xy-c)@vt[1]).ptp():.3f}")
# thickness profile along axis
for lo in np.arange(proj.min(), proj.max(), 0.01):
    sel = (proj >= lo) & (proj < lo + 0.01)
    if sel.sum():
        print(f"  s={lo:+.3f} n={sel.sum():3d} zmax={pts[sel,2].max():.3f} width={((xy[sel]-c)@vt[1]).ptp():.3f}")
thick = pts[:,2] > 0.462
cb = xy[thick].mean(0)
print(f"body centroid (z>0.462): ({cb[0]:.4f},{cb[1]:.4f}), n={thick.sum()}")
EOF
timeout 90 python3 bottle_axis.py

# openrua op 31
python3 -c "
import cv2
im = cv2.imread('birdview.png'); crop = im[235:285, 255:315]
cv2.imwrite('bottle_crop.png', cv2.resize(crop, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST))
"

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py birdview && python3 -c "
import cv2
im = cv2.imread('birdview.png'); crop = im[235:285, 255:315]
cv2.imwrite('bottle_crop.png', cv2.resize(crop, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST))
"

# openrua op 33
python3 - <<'EOF'
s = open("arm.py").read()
s = s.replace("    def solve_ik(self, hand_world_p, q):", "    def solve_ik(self, hand_world_p, q, seed_pos=None):")
s = s.replace('''        seed = JointState()
        j = self.joints()
        for n in JOINTS:
            seed.name.append(n); seed.position.append(j[n])''',
'''        seed = JointState()
        if seed_pos is None:
            j = self.joints(); seed_pos = [j[n] for n in JOINTS]
        for n, p_ in zip(JOINTS, seed_pos):
            seed.name.append(n); seed.position.append(float(p_))''')
s = s.replace('''    def traj(self, positions, secs):''', '''    def traj_multi(self, waypoints, total_secs):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = total_secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(p) for p in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - p) for n, p in zip(JOINTS, waypoints[-1]))
        print(f"traj_multi done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp_lin(self, xyz, q=DOWN_Q, step=0.02, speed=0.05, max_jump=0.35):
        """Straight-line TCP move in small steps; IK chained from the previous
        waypoint so the arm stays on one configuration branch."""
        _, _, tcp0 = self.hand_world()
        target = np.asarray(xyz, float)
        dist = np.linalg.norm(target - tcp0)
        n = max(2, int(np.ceil(dist / step)))
        R = quat_R(*q)
        j = self.joints(); seed = [j[k] for k in JOINTS]
        wps = []
        for i in range(1, n + 1):
            p = tcp0 + (target - tcp0) * i / n
            hand = p - TCP * R[:, 2]
            sol = self.solve_ik(hand, q, seed)
            if sol is None:
                print(f"IK FAILED at waypoint {i}/{n} {p}"); return False
            jump = max(abs(a - b) for a, b in zip(sol, seed))
            if jump > max_jump:
                print(f"branch jump {jump:.2f} at waypoint {i}/{n}; aborting"); return False
            wps.append(sol); seed = sol
        secs = max(1.0, dist / speed)
        code, err = self.traj_multi(wps, secs)
        if err > 0.02:
            print("large tracking error, resending last point")
            code, err = self.traj(wps[-1], 1.5)
        _, _, tcp = self.hand_world()
        d = np.linalg.norm(tcp - target)
        print(f"TCP now ({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) target {tuple(round(v,4) for v in xyz)} dist={d:.4f}")
        return d < 0.01

    def traj(self, positions, secs):''')
open("arm.py","w").write(s)
EOF
python3 -c "import ast;ast.parse(open('/workspace/arm.py').read());print('ok')"

# openrua op 34
mkdir -p "$(dirname /workspace/seg.py)"
cat > /workspace/seg.py <<'OPENRUA_EOF'
"""Depth-based object segmentation into world-frame clusters (library form of locate.py)."""
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from rclpy.time import Time
from cv_bridge import CvBridge

TABLE_Z = 0.425


def _quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def cloud(node, buf, cam):
    """Return (P[H,W,3] world points, color[H,W,3], valid mask) for a fresh frame."""
    got = {}
    subs = [node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1),
            node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1),
            node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)]
    frame = f"{cam}_optical_frame"
    while not all(k in got for k in "dci") or not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    for s in subs: node.destroy_subscription(s)
    depth = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
    K = np.array(got["i"].k).reshape(3, 3)
    t = buf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = _quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    vv, uu = np.mgrid[0:H, 0:W]
    X = (uu - K[0, 2]) * depth / K[0, 0]; Y = (vv - K[1, 2]) * depth / K[1, 1]
    P = np.stack([X, Y, depth], -1) @ R.T + T
    valid = np.isfinite(depth) & (depth > 0.05)
    return P, color, valid


def clusters(node, buf, cam, minh=0.012, maxh=0.12, table_z=TABLE_Z, min_area=15):
    """Objects above the table (excluding tall things like the arm): list of dicts."""
    P, color, valid = cloud(node, buf, cam)
    mask = (valid & (P[..., 2] > table_z + minh) & (P[..., 2] < table_z + maxh)).astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_area: continue
        pts = P[lab == i]
        xy = pts[:, :2]; c = xy.mean(0)
        u, s, vt = np.linalg.svd(xy - c, full_matrices=False)
        axis = vt[0]
        out.append(dict(centre=c, ztop=pts[:, 2].max(), area=int(stats[i, 4]), n=len(pts),
                        xmin=pts[:, 0].min(), xmax=pts[:, 0].max(), ymin=pts[:, 1].min(), ymax=pts[:, 1].max(),
                        axis_deg=float(np.degrees(np.arctan2(axis[1], axis[0]))),
                        length=float(((xy - c) @ axis).ptp()), width=float(((xy - c) @ vt[1]).ptp()),
                        pts=pts, bgr=color[lab == i].mean(0)))
    return out


def nearest(cls, xy, maxd=0.06):
    best = None
    for c in cls:
        d = np.linalg.norm(c["centre"] - np.asarray(xy))
        if d < maxd and (best is None or d < best[0]):
            best = (d, c)
    return None if best is None else best[1]


def describe(c):
    return (f"centre=({c['centre'][0]:.4f},{c['centre'][1]:.4f}) ztop={c['ztop']:.3f} n={c['n']} "
            f"x[{c['xmin']:.3f},{c['xmax']:.3f}] y[{c['ymin']:.3f},{c['ymax']:.3f}] "
            f"axis={c['axis_deg']:.1f}deg L={c['length']:.3f} W={c['width']:.3f}")
OPENRUA_EOF

# openrua op 35
mkdir -p "$(dirname /workspace/pick_bottle.py)"
cat > /workspace/pick_bottle.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the lying ketchup bottle and drop it in the basket."""
import sys
from arm import *
import seg

EXPECT = np.array([-0.109, -0.164])   # birdview centroid of the bottle
AXIS = -25.0                           # deg, long axis
BODY_SHIFT = -0.03                     # metres along axis from centroid toward the (wider) body end
GRASP_Z = 0.445
HOVER_Z = 0.60
CARRY_Z = 0.72
BASKET = np.array([0.0, 0.255])
BASKET_YAW = 45.0
RELEASE_Z = 0.63

def yawq(deg):
    return quat_mul((0.0, 0.0, np.sin(np.radians(deg) / 2), np.cos(np.radians(deg) / 2)), DOWN_Q)

a = Arm()
a.state()
print("== open"); a.gripper(True)

def body_centre(c):
    """grasp point: shift from centroid along axis toward the wider end."""
    ax = np.array([np.cos(np.radians(c["axis_deg"])), np.sin(np.radians(c["axis_deg"]))])
    xy = c["pts"][:, :2]; s = (xy - c["centre"]) @ ax
    perp = (xy - c["centre"]) @ np.array([-ax[1], ax[0]])
    wneg = perp[s < -0.02].ptp() if (s < -0.02).sum() > 5 else 0
    wpos = perp[s > 0.02].ptp() if (s > 0.02).sum() > 5 else 0
    sign = -1 if wneg > wpos else 1
    print(f"   widths: s<0 {wneg:.3f}  s>0 {wpos:.3f} -> body on {'negative' if sign<0 else 'positive'} side")
    return c["centre"] + sign * abs(BODY_SHIFT) * ax, c["axis_deg"]

# 1. hover above the birdview estimate
gp = EXPECT + BODY_SHIFT * np.array([np.cos(np.radians(AXIS)), np.sin(np.radians(AXIS))])
yaw = AXIS
print("== hover", gp, yaw)
assert a.move_tcp((gp[0], gp[1], HOVER_Z), 3.0, yawq(yaw)), "hover failed"
p, q, tcp = a.hand_world(); R = quat_R(*q)
print(f"   hand y-axis in world: ({R[0,1]:.3f},{R[1,1]:.3f}); bottle axis dir ({np.cos(np.radians(yaw)):.3f},{np.sin(np.radians(yaw)):.3f}) dot={R[0,1]*np.cos(np.radians(yaw))+R[1,1]*np.sin(np.radians(yaw)):.3f} (want 0)")

# 2. refine from the wrist camera
for attempt in range(2):
    cls = seg.clusters(a.node, a.buf, "robot0_eye_in_hand", minh=0.012, maxh=0.10)
    for c in cls: print("   wrist:", seg.describe(c))
    c = seg.nearest(cls, gp, 0.08)
    if c is None or c["n"] < 200:
        print("   bottle not found in wrist cam; keeping birdview estimate"); break
    gp2, yaw2 = body_centre(c)
    print(f"   refined grasp ({gp2[0]:.4f},{gp2[1]:.4f}) yaw {yaw2:.1f}  (delta {np.linalg.norm(gp2-gp)*100:.1f} cm)")
    if np.linalg.norm(gp2 - gp) < 0.004 and abs(yaw2 - yaw) < 3:
        break
    gp, yaw = gp2, yaw2
    assert a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw), step=0.02, speed=0.05), "re-hover failed"

# 3. descend, grasp
print("== descend")
assert a.move_tcp_lin((gp[0], gp[1], GRASP_Z), yawq(yaw), step=0.02, speed=0.04), "descend failed"
print("== close"); f1, f2 = a.gripper(False)
gap = abs(f1) + abs(f2)
if gap < 0.01:
    print("GRASP FAILED (gap %.4f)" % gap); a.gripper(True)
    a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw)); sys.exit(2)
print("== lift")
assert a.move_tcp_lin((gp[0], gp[1], CARRY_Z), yawq(yaw), step=0.03, speed=0.06), "lift failed"
j = a.joints(); gap2 = abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])
print(f"   gap after lift {gap2:.4f}")
if gap2 < 0.01:
    print("LOST OBJECT"); sys.exit(3)
# 4. to basket (rotate so the bottle lies along the basket diagonal)
print("== to basket")
assert a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), 4.0, yawq(BASKET_YAW)), "basket move failed"
j = a.joints(); print("   gap at basket", round(abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"]), 4))
print("== lower"); a.move_tcp_lin((BASKET[0], BASKET[1], RELEASE_Z), yawq(BASKET_YAW), step=0.03, speed=0.05)
print("== release"); a.gripper(True)
print("== retreat"); a.move_tcp_lin((BASKET[0], BASKET[1], CARRY_Z), yawq(BASKET_YAW), step=0.03, speed=0.06)
a.state()
print("DONE")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 36
nohup python3 -u pick_bottle.py > bottle.log 2>&1 &
sleep 60; cat /workspace/bottle.log

# openrua op 37
python3 - <<'EOF'
s = open("/workspace/pick_bottle.py").read()
s = s.replace('''    return c["centre"] + sign * abs(BODY_SHIFT) * ax, c["axis_deg"]''',
'''    deg = c["axis_deg"]
    while deg > 90: deg -= 180
    while deg <= -90: deg += 180
    return c["centre"] + sign * abs(BODY_SHIFT) * ax, deg''')
open("/workspace/pick_bottle.py","w").write(s)
EOF
nohup python3 -u pick_bottle.py > bottle.log 2>&1 &
sleep 90; cat /workspace/bottle.log

# openrua op 38
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 39
mkdir -p "$(dirname /workspace/pick_box.py)"
cat > /workspace/pick_box.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the cream cheese box and drop it in the basket."""
import sys
from arm import *
import seg

EXPECT = np.array([-0.163, 0.058])
AXIS = 0.0            # long axis roughly along world x
GRASP_Z = 0.440       # box top 0.455, table 0.425
HOVER_Z = 0.58
CARRY_Z = 0.72
BASKET = np.array([0.0, 0.255])
BASKET_YAW = 45.0
RELEASE_Z = 0.66

def yawq(deg):
    return quat_mul((0.0, 0.0, np.sin(np.radians(deg) / 2), np.cos(np.radians(deg) / 2)), DOWN_Q)

def norm_deg(deg):
    while deg > 90: deg -= 180
    while deg <= -90: deg += 180
    return deg

a = Arm()
a.state()
print("== open"); a.gripper(True)
gp, yaw = EXPECT.copy(), AXIS
print("== hover", gp, yaw)
ok = a.move_tcp((gp[0], gp[1], HOVER_Z), 3.5, yawq(yaw))
if not ok:
    ok = a.move_tcp((gp[0], gp[1], HOVER_Z), 3.0, yawq(yaw))
assert ok, "hover failed"

for attempt in range(3):
    cls = seg.clusters(a.node, a.buf, "robot0_eye_in_hand", minh=0.012, maxh=0.06)
    for c in cls: print("   wrist:", seg.describe(c))
    c = seg.nearest(cls, gp, 0.06)
    if c is None or c["n"] < 100:
        print("   box not found in wrist cam; keeping estimate"); break
    gp2, yaw2 = c["centre"], norm_deg(c["axis_deg"])
    print(f"   refined ({gp2[0]:.4f},{gp2[1]:.4f}) yaw {yaw2:.1f} W={c['width']:.3f} L={c['length']:.3f} (delta {np.linalg.norm(gp2-gp)*100:.1f} cm)")
    if np.linalg.norm(gp2 - gp) < 0.003 and abs(yaw2 - yaw) < 3:
        break
    gp, yaw = gp2, yaw2
    assert a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw), step=0.02, speed=0.05), "re-hover failed"

print("== descend")
assert a.move_tcp_lin((gp[0], gp[1], GRASP_Z), yawq(yaw), step=0.02, speed=0.04), "descend failed"
print("== close"); f1, f2 = a.gripper(False)
gap = abs(f1) + abs(f2)
if gap < 0.008:
    print("GRASP FAILED (gap %.4f)" % gap); a.gripper(True)
    a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw)); sys.exit(2)
print("== lift")
assert a.move_tcp_lin((gp[0], gp[1], CARRY_Z), yawq(yaw), step=0.03, speed=0.06), "lift failed"
j = a.joints(); gap2 = abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])
print(f"   gap after lift {gap2:.4f}")
if gap2 < 0.008:
    print("LOST OBJECT"); sys.exit(3)
print("== to basket")
ok = a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), 4.0, yawq(BASKET_YAW))
if not ok: ok = a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), 3.0, yawq(BASKET_YAW))
assert ok, "basket move failed"
j = a.joints(); print("   gap at basket", round(abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"]), 4))
print("== lower"); a.move_tcp_lin((BASKET[0], BASKET[1], RELEASE_Z), yawq(BASKET_YAW), step=0.03, speed=0.05)
print("== release"); a.gripper(True)
print("== retreat"); a.move_tcp_lin((BASKET[0], BASKET[1], CARRY_Z), yawq(BASKET_YAW), step=0.03, speed=0.06)
a.state()
print("DONE")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 40
nohup python3 -u pick_box.py > box.log 2>&1 &
sleep 90; cat /workspace/box.log

# openrua op 41
sleep 45; tail -12 /workspace/box.log

# openrua op 42
grep -E "gap at basket|to basket" -A2 /workspace/box.log | head; timeout 300 python3 arm.py move -0.30 0.0 0.80 3 2>&1 | tail -2; for c in agentview birdview robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 43
python3 -c "
import cv2
im = cv2.imread('birdview.png'); crop = im[255:315, 350:415]
cv2.imwrite('basket_crop.png', cv2.resize(crop, None, fx=8, fy=8, interpolation=cv2.INTER_CUBIC))
"; timeout 90 python3 locate.py birdview 0.012 0.425 | grep -v "cluster 1:"
