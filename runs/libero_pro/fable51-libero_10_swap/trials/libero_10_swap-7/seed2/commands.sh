#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ----; ros2 action list 2>&1; echo ----; ros2 service list 2>&1 | head -50; echo ----; ros2 node list 2>&1

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
cat > tfdump.py <<'EOF'
import rclpy, sys
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
import time
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_string())
for tgt in sys.argv[1:]:
    try:
        t = buf.lookup_transform("world", tgt, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(tgt, f"xyz=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(tgt, "ERR", e)
rclpy.shutdown()
EOF
timeout 60 python3 tfdump.py panda_link0 panda_hand robot0_robotview_optical_frame agentview_optical_frame birdview_optical_frame robot0_eye_in_hand_optical_frame

# openrua op 6
cat > scene3d.py <<'EOF'
"""Project a camera's depth frame to world points; cluster things above the table."""
import sys, numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("scene3d")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while len(got) < 3 or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - cx) * Z / fx; Y = (vv - cy) * Z / fy
P = np.stack([X, Y, Z], -1) @ R.T + T
np.save(f"{cam}_world.npy", P)
valid = np.isfinite(Z) & (Z > 0)
zs = P[..., 2][valid]
print("z percentiles", np.percentile(zs, [1, 5, 25, 50, 75, 95, 99]))
# table height estimate = mode of z
hist, edges = np.histogram(zs, bins=200)
table_z = edges[np.argmax(hist)]
print("table_z ~", table_z)
above = valid & (P[..., 2] > table_z + 0.01) & (P[..., 2] < table_z + 0.5)
# connected components on the mask
import cv2
n, lab = cv2.connectedComponents(above.astype(np.uint8))
for i in range(1, n):
    m = lab == i
    if m.sum() < 30: continue
    pts = P[m]
    col = color[m].mean(0)[::-1]
    us, vs = uu[m], vv[m]
    print(f"blob {i}: n={m.sum()} px u[{us.min()},{us.max()}] v[{vs.min()},{vs.max()}] "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] rgb={col.astype(int)}")
rclpy.shutdown()
EOF
timeout 120 python3 scene3d.py robot0_robotview

# openrua op 7
timeout 120 python3 scene3d.py agentview; echo ----; timeout 120 python3 scene3d.py birdview

# openrua op 8
cat > analyze.py <<'EOF'
import numpy as np, cv2
P = np.load("birdview_world.npy")
Z = P[...,2]
valid = np.isfinite(Z) & (Z > 0.44) & (Z < 1.0)
n, lab = cv2.connectedComponents(valid.astype(np.uint8))
H, W = Z.shape
vv, uu = np.mgrid[0:H, 0:W]
for i in range(1, n):
    m = lab == i
    if m.sum() < 5: continue
    pts = P[m]
    print(f"bird blob {i}: n={m.sum()} u[{uu[m].min()},{uu[m].max()}] v[{vv[m].min()},{vv[m].max()}] x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
print("---- robotview top surfaces")
P = np.load("robot0_robotview_world.npy")
Z = P[...,2]
for name, (x0,x1,y0,y1) in {"alphabet_can":(-0.26,-0.15,-0.22,-0.10), "tomato_can":(-0.22,-0.11,0.0,0.12), "ketchup":(-0.02,0.12,-0.14,-0.02), "cheese_box":(0.05,0.2,-0.24,-0.14), "basket":(-0.12,0.1,0.14,0.36)}.items():
    m = np.isfinite(Z) & (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(Z>0.44)
    pts = P[m]
    top = pts[:,2].max()
    tm = pts[pts[:,2] > top-0.012]
    print(f"{name}: top z={top:.3f} n_top={len(tm)} top center=({tm[:,0].mean():.3f},{tm[:,1].mean():.3f}) x[{tm[:,0].min():.3f},{tm[:,0].max():.3f}] y[{tm[:,1].min():.3f},{tm[:,1].max():.3f}]")
    if name=="cheese_box":
        xy = tm[:,:2] - tm[:,:2].mean(0)
        w,v = np.linalg.eigh(xy.T@xy)
        print("  box principal axis", v[:,1], "angle deg", np.degrees(np.arctan2(v[1,1],v[0,1])), "extents", np.sqrt(w/len(xy))*np.sqrt(12))
EOF
python3 analyze.py

# openrua op 9
python3 -c "
import cv2
im = cv2.imread('robot0_robotview.png')
crop = im[0:120, 140:420]
cv2.imwrite('cans_zoom.png', cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
crop = im[190:300, 60:170]
cv2.imwrite('box_zoom.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
"

# openrua op 10
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper around this machine's ports (see machine.yaml).

  python3 rob.py state                         joints + hand/tcp pose (world)
  python3 rob.py goto X Y Z [SECS] [YAW_DEG]   tcp -> world point, hand Z down
  python3 rob.py joints p1,...,p7 SECS         raw joint trajectory
  python3 rob.py grip WIDTH                    per-finger width (0.04 open, 0 closed)
"""
import sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_W = np.array([-0.51, 0.0, 0.42])       # panda_link0 in world (TF, identity rotation)
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg=0.0):
    """Hand Z down; yaw rotates the finger axis about world Z (0 -> fingers along Y)."""
    # q = Rz(yaw) * Rx(pi)
    h = np.radians(yaw_deg) / 2
    qz = np.array([0, 0, np.sin(h), np.cos(h)])
    qx = np.array([1.0, 0, 0, 0])
    x1, y1, z1, w1 = qz; x2, y2, z2, w2 = qx
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2,
                     w1*w2 - x1*x2 - y1*y2 - z1*z2])


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, n=1, t=0.1):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin()
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_positions(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        self._wr.pop("m", None)
        end = time.time() + 3
        while "m" not in self._wr and time.time() < end:
            self.spin()
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return (f.x, f.y, f.z)

    def hand_pose(self):
        """FK of panda_hand in world (base offset added)."""
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        js = JointState()
        j = self.joints()
        for n in ARM:
            js.name.append(n); js.position.append(j[n])
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_to_R(*q)[:, 2]
        return pos, q, tcp

    def solve_ik(self, tcp_world, quat, seed=None):
        """Joint solution putting the TCP at tcp_world with hand orientation quat."""
        self.ik.wait_for_service(5)
        R = quat_to_R(*quat)
        hand_w = np.array(tcp_world) - TCP * R[:, 2]
        hand_b = hand_w - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        js = JointState()
        seed = seed if seed is not None else self.arm_positions()
        for n, v in zip(ARM, seed):
            js.name.append(n); js.position.append(float(v))
        req.ik_request.robot_state.joint_state = js
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            raise RuntimeError(f"IK failed code={r.error_code.val}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def move_joints(self, positions, seconds, via=None):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for i, (pos, t) in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur = self.arm_positions()
        err = max(abs(a - b) for a, b in zip(cur, positions))
        print(f"traj done error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def goto(self, tcp_world, seconds=4.0, yaw_deg=0.0, seed=None):
        q = topdown_quat(yaw_deg)
        sol = self.solve_ik(tcp_world, q, seed)
        print("ik sol", np.round(sol, 4).tolist(), flush=True)
        code, err = self.move_joints(sol, seconds)
        pose = self.hand_pose()
        if pose:
            print(f"tcp now {np.round(pose[2], 4).tolist()} target {list(tcp_world)}", flush=True)
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"gripper reached_goal={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

    def report(self):
        j = self.joints()
        print("arm", {n: round(j[n], 4) for n in ARM})
        print("fingers", self.fingers())
        pose = self.hand_pose()
        if pose:
            print("hand", np.round(pose[0], 4).tolist(), "q", np.round(pose[1], 4).tolist(),
                  "tcp", np.round(pose[2], 4).tolist())
        print("wrench", self.wrench())

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    a = sys.argv[1:]
    r = Robot()
    try:
        if a[0] == "state":
            r.report()
        elif a[0] == "goto":
            xyz = [float(v) for v in a[1:4]]
            secs = float(a[4]) if len(a) > 4 else 4.0
            yaw = float(a[5]) if len(a) > 5 else 0.0
            r.goto(xyz, secs, yaw)
        elif a[0] == "joints":
            r.move_joints([float(v) for v in a[1].split(",")], float(a[2]))
        elif a[0] == "grip":
            r.gripper(float(a[1]))
        else:
            print(__doc__)
    finally:
        r.close()
OPENRUA_EOF

# openrua op 11
timeout 120 python3 rob.py state

# openrua op 12
cat > iktest.py <<'EOF'
from rob import *
r = Robot()
cur = r.arm_positions()
q = (0.9996, 0.0, -0.0284, 0.0)
R = quat_to_R(*q)
hand_tf = np.array([-0.053, 0.0, 0.7776])
for label, hand in [("world-coords", hand_tf), ("base-coords", hand_tf - BASE_W)]:
    tcp = hand + TCP * R[:, 2]
    # solve_ik subtracts BASE_W itself, so add it back to test raw frames
    try:
        sol = r.solve_ik(tcp + BASE_W, q, cur)
        print(label, "OK sol", np.round(sol, 3).tolist(), "maxdiff", max(abs(a-b) for a,b in zip(sol,cur)))
    except Exception as e:
        print(label, "FAIL", e)
r.close()
EOF
timeout 200 python3 iktest.py

# openrua op 13
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_W = np.array([-0.51, 0.0, 0.42])       # panda_link0 in world (TF, identity rotation)", "new_string": "# Verified empirically: /compute_fk and /compute_ik on this machine take/give\\n# poses that match TF `world` (FK of panda_hand == TF world->panda_hand), so\\n# no base offset is applied.\\nBASE_W = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 14
timeout 600 python3 -u rob.py grip 0.04 2>&1 | tail -5

# openrua op 15
timeout 900 python3 -u rob.py goto -0.208 -0.163 0.62 4 2>&1 | tail -5

# openrua op 16
timeout 120 python3 rob.py state

# openrua op 17
timeout 900 python3 -u rob.py joints -0.2278,-0.4302,-0.3036,-2.8673,-0.1956,2.4451,-0.3543 2 2>&1 | tail -3; timeout 120 python3 rob.py state

# openrua op 18
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        p = req.ik_request.pose_stamped.pose", "new_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        req.ik_request.ik_link_name = \\"panda_hand\\"   # group tip is link8 (45deg yaw off)\\n        p = req.ik_request.pose_stamped.pose", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 19
cat > iktest2.py <<'EOF'
from rob import *
r = Robot()
cur = r.arm_positions()
q = topdown_quat(0)
sol = r.solve_ik([-0.208, -0.163, 0.62], q, cur)
print("sol", np.round(sol, 4).tolist())
# FK of the solution
req = GetPositionFK.Request(); req.header.frame_id = ""; req.fk_link_names = ["panda_hand", "panda_link8"]
js = JointState()
for n, v in zip(ARM, sol): js.name.append(n); js.position.append(float(v))
req.robot_state.joint_state = js
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for ps in fut.result().pose_stamped:
    p = ps.pose; print(ps.header.frame_id, [round(v,4) for v in (p.position.x,p.position.y,p.position.z)], [round(v,4) for v in (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)])
r.close()
EOF
timeout 200 python3 iktest2.py

# openrua op 20
timeout 900 python3 -u rob.py goto -0.208 -0.163 0.62 3 2>&1 | tail -3

# openrua op 21
timeout 900 python3 -u rob.py joints -0.361,-0.4184,-0.1522,-2.8699,-0.0972,2.4535,0.3603 2 2>&1 | tail -3; timeout 120 python3 rob.py state; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 22
timeout 900 python3 -u rob.py goto -0.208 -0.163 0.45 3 2>&1 | tail -3

# openrua op 23
timeout 900 python3 -u rob.py joints -0.4536,0.2407,-0.0175,-2.8551,0.0907,3.0956,0.2242 2 2>&1 | tail -3; timeout 120 python3 rob.py state

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 25
cat > urdf.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("u")
got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), qos)
while "m" not in got: rclpy.spin_once(n, timeout_sec=0.5)
open("robot.urdf","w").write(got["m"].data); print(len(got["m"].data))
rclpy.shutdown()
EOF
timeout 60 python3 urdf.py && python3 - <<'EOF'
import re
u = open("robot.urdf").read()
for name in ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link8"]:
    m = re.search(rf'<link name="{name}">(.*?)</link>', u, re.S)
    print("=== link", name); print(m.group(1)[:1500] if m else "none")
for j in re.finditer(r'<joint name="(panda_hand_joint|panda_finger_joint1|panda_finger_joint2|panda_joint8|world_joint|[^"]*world[^"]*)"[^>]*>(.*?)</joint>', u, re.S):
    print("=== joint", j.group(1)); print(j.group(2)[:600])
EOF

# openrua op 26
timeout 120 python3 scene3d.py robot0_eye_in_hand 2>&1 | tail -8; timeout 60 python3 tfdump.py panda_hand panda_leftfinger panda_rightfinger robot0_eye_in_hand_optical_frame 2>&1 | tail -4

# openrua op 27
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy")
Z = P[...,2]
H,W = Z.shape
vv,uu = np.mgrid[0:H,0:W]
for lo,hi in [(0.44,0.47),(0.47,0.495),(0.495,0.51),(0.51,0.525),(0.525,0.55)]:
    m = np.isfinite(Z)&(Z>lo)&(Z<hi)
    if m.sum()==0: print(lo,hi,"none"); continue
    pts=P[m]
    print(f"z[{lo},{hi}] n={m.sum()} u[{uu[m].min()},{uu[m].max()}] v[{vv[m].min()},{vv[m].max()}] x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
EOF

# openrua op 28
timeout 900 python3 -u rob.py goto -0.208 -0.163 0.487 2 2>&1 | tail -3; timeout 120 python3 rob.py state | tail -2

# openrua op 29
timeout 600 python3 -u rob.py grip 0.0 2>&1 | tail -2; timeout 120 python3 rob.py state | tail -3

# openrua op 30
timeout 900 python3 -u rob.py goto -0.208 -0.163 0.72 3 2>&1 | tail -3; timeout 120 python3 rob.py state | tail -3; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 31
timeout 900 python3 -u rob.py goto -0.013 0.255 0.76 4 2>&1 | tail -3; timeout 120 python3 rob.py state | tail -3

# openrua op 32
timeout 900 python3 -u rob.py goto -0.013 0.255 0.60 3 2>&1 | tail -3; timeout 120 python3 rob.py state | tail -3

# openrua op 33
timeout 600 python3 -u rob.py grip 0.04 2>&1 | tail -1; timeout 900 python3 -u rob.py goto -0.013 0.255 0.76 3 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 900 python3 -u rob.py goto 0.117 -0.187 0.60 4 2>&1 | tail -3; timeout 120 python3 rob.py state | tail -3

# openrua op 35
timeout 900 python3 -u rob.py joints -0.2013,0.4838,-0.0986,-1.7891,0.0599,2.2701,0.4582 2 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 36
timeout 900 python3 -u rob.py goto 0.117 -0.187 0.437 3 2>&1 | tail -3; timeout 120 python3 rob.py state | tail -3

# openrua op 37
timeout 600 python3 -u rob.py grip 0.0 2>&1 | tail -1

# openrua op 38
timeout 900 python3 -u rob.py goto 0.117 -0.187 0.70 3 2>&1 | tail -2; timeout 120 python3 rob.py state | grep fingers; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 39
timeout 900 python3 -u rob.py goto -0.013 0.30 0.76 4 2>&1 | tail -2; timeout 900 python3 -u rob.py goto -0.013 0.30 0.65 2 2>&1 | tail -2; timeout 120 python3 rob.py state | tail -3

# openrua op 40
timeout 600 python3 -u rob.py grip 0.04 2>&1 | tail -1; timeout 900 python3 -u rob.py goto -0.10 0.20 0.80 3 2>&1 | tail -1; for c in agentview robot0_robotview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 41
python3 -c "
import cv2
im = cv2.imread('birdview.png'); crop = im[240:330, 340:430]
cv2.imwrite('basket_zoom.png', cv2.resize(crop, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('robot0_robotview.png'); cv2.imwrite('robotview_now.png', im)
"; timeout 120 python3 scene3d.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); Z = P[...,2]
# inside basket interior footprint
m = np.isfinite(Z)&(P[...,0]>-0.08)&(P[...,0]<0.055)&(P[...,1]>0.18)&(P[...,1]<0.33)&(Z>0.43)&(Z<0.62)
pts = P[m]
print("points inside basket footprint below rim:", len(pts))
for lo,hi in [(0.43,0.47),(0.47,0.51),(0.51,0.55),(0.55,0.62)]:
    s = pts[(pts[:,2]>lo)&(pts[:,2]<hi)]
    if len(s): print(f" z[{lo},{hi}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# anything left on the table outside basket at the old object spots?
for name,(x0,x1,y0,y1) in {"old can spot":(-0.26,-0.15,-0.22,-0.10),"old box spot":(0.05,0.2,-0.24,-0.14)}.items():
    mm = np.isfinite(Z)&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(Z>0.44)
    print(name, "points above table:", mm.sum())
EOF

# openrua op 42
timeout 900 python3 -u rob.py goto -0.30 -0.15 0.85 3 2>&1 | tail -1; for c in birdview robot0_robotview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; python3 -c "
import cv2
im = cv2.imread('birdview.png'); crop = im[240:330, 340:430]
cv2.imwrite('basket_zoom.png', cv2.resize(crop, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC))"

# openrua op 43
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified quirks of the Franka Panda sim workstation in /workspace (IK frame, tip link, trajectory lag) that contradict or extend the shipped docs
metadata:
  type: project
---

Empirical facts about the Panda workstation at /workspace (verified 2026-09-12):

- `/compute_ik` and `/compute_fk` with empty `frame_id` use **TF `world` coordinates**
  (FK of panda_hand == TF world->panda_hand; base offset (-0.51,0,0.42) is already in the
  model). docs/30-action.md says "model frame is the ARM BASE" — that was wrong here;
  base-frame coords gave IK error -31.
- The `panda_arm` group tip is `panda_link8`, which is yawed 45° from `panda_hand`.
  Set `ik_request.ik_link_name = "panda_hand"` to get the requested finger-axis yaw.
- FollowJointTrajectory frequently returns error_code -5 with one joint (often joint7)
  lagging 0.1–0.4 rad; resending the same goal with ~2 s converges (error 0).
- Top-down grasp: hand quat (1,0,0,0) puts fingers along world Y. Gripper open gap
  measured ~7.7–8 cm; a soup can (6.3 cm) and a 4 cm box both grasped cleanly.
- Object-height estimates from the depth cameras ran ~2–3 cm low for the soup can
  (palm hit the can top at hand z = can_top + 0.058); add margin before descending.

**Why:** these cost several turns to discover and are not in machine.yaml or docs.
**How to apply:** reuse `/workspace/rob.py` (Robot.goto / gripper / state) if it still exists;
otherwise re-verify the frame with an FK-vs-TF comparison before the first move.
OPENRUA_EOF

# openrua op 44
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK/FK use world coords, tip link is link8 (45° off hand), traj -5 lag → resend" >> MEMORY.md && cat MEMORY.md
