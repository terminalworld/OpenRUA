#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -20; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 15 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Cluster objects above the table from a camera's depth frame.

Usage: python3 scan.py <camera> [min_height_m]
Prints world-frame cluster centroids, extents, and top heights; also
saves <camera>_labels.png for visual cross-reference.
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, mt, timeout=20.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    minh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
    rclpy.init()
    node = rclpy.create_node("scan")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    z = depth
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    P = np.stack([X, Y, z], -1).reshape(-1, 3) @ R.T + T
    P = P.reshape(H, W, 3)
    valid = np.isfinite(z) & (z > 0)
    # table height = mode of world z among valid points
    zs = P[..., 2][valid]
    hist, edges = np.histogram(zs, bins=400)
    table_z = edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2
    print(f"camera {cam} at world {T.round(3)}; table_z~{table_z:.3f}")
    mask = valid & (P[..., 2] > table_z + minh)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    out = color.copy()
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 30:
            continue
        m = lab == i
        pts = P[m]
        mn, mx = pts.min(0), pts.max(0)
        c = pts.mean(0)
        u, v = cents[i]
        print(f"#{i:2d} px=({u:5.0f},{v:5.0f}) area={area:5d} "
              f"centroid=({c[0]:.3f},{c[1]:.3f}) xy-extent=({mn[0]:.3f}..{mx[0]:.3f}, {mn[1]:.3f}..{mx[1]:.3f}) "
              f"top_z={mx[2]:.3f} (h={mx[2]-table_z:.3f})")
        x0, y0, w, h = stats[i, :4]
        cv2.rectangle(out, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
        cv2.putText(out, str(i), (x0, y0 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.imwrite(f"{cam}_labels.png", out)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scan.py birdview 0.01

# openrua op 8
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/scan.py", "old_string": "    zs = P[..., 2][valid]\\n    hist, edges = np.histogram(zs, bins=400)", "new_string": "    zs = P[..., 2][valid]\\n    zs = zs[zs > 0.2]  # ignore the floor\\n    hist, edges = np.histogram(zs, bins=400)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 9
timeout 120 python3 scan.py birdview 0.01

# openrua op 10
python3 -c "
import cv2; im=cv2.imread('birdview_labels.png'); crop=im[180:340,220:440]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 11
timeout 120 python3 scan.py robot0_eye_in_hand 0.01; echo ----; timeout 120 python3 scan.py agentview 0.01

# openrua op 12
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('cans_agent.png', cv2.resize(im[220:320,180:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('cans_wrist.png', cv2.resize(im[240:360,160:540],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 13
timeout 20 ros2 service list | grep -iE "fk|ik|plan"; echo ----; timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -30

# openrua op 14
mkdir -p "$(dirname /workspace/robolib.py)"
cat > /workspace/robolib.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable control helpers: one node, clients built once.

World-frame poses; IK via /compute_ik (empty frame_id, verified to be
the same frame as TF `world` on this machine via /compute_fk).
"""
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers open along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down(yaw):
    """Quaternion: hand pointing down, fingers rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); Rx(pi) = (1,0,0,0); product (w1w2 - v1.v2, ...)
    # (x,y,z,w) = (c*1, s*1*... ) compute explicitly
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Robot:
    def __init__(self, name="robolib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk.wait_for_service(timeout_sec=20), "no FK"
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper"
        self.spin(0.5)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
            while not self._js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    # ---- kinematics ----
    def hand_pose(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def tcp_pose(self, q=None):
        p, quat = self.hand_pose(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    def solve_ik(self, xyz, quat=DOWN, at_tcp=True, seed=None, tries=3):
        xyz = np.array(xyz, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        seed = list(seed if seed is not None else self.arm_q())
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = seed
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print(f"  IK try {k+1} failed: {None if r is None else r.error_code.val}")
        return None

    # ---- motion ----
    def move_q(self, q, secs=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via, 1):
                t = secs * i / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        assert h is not None and h.accepted, "FJT goal rejected"
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        res = rf.result()
        code = res.result.error_code if res else None
        self.spin(0.3)
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, xyz, quat=DOWN, secs=3.0, at_tcp=True, label=""):
        q = self.solve_ik(xyz, quat, at_tcp)
        if q is None:
            print(f"  !! no IK for {label} {xyz}")
            return False
        code, err = self.move_q(q, secs)
        p, _ = self.tcp_pose()
        print(f"  [{label}] tcp now {p.round(4)} target {np.round(xyz,4)} "
              f"d={np.linalg.norm(p-np.array(xyz)):.4f}")
        return code == 0 and err < 0.02

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(0.3)
        print(f"  gripper({width}) reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -u -c "
from robolib import *
r = Robot('t')
print('hand', r.hand_pose()); print('tcp', r.tcp_pose()); print('gap', r.finger_gap())
q = r.solve_ik((-0.085,-0.14,0.62)); print('ik above can', q)
if q: print('fk check', r.tcp_pose(q))
q2 = r.solve_ik((0.0,0.27,0.70), seed=q); print('ik above basket', q2)
if q2: print('fk check', r.tcp_pose(q2))
"

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robolib.py", "old_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            req.ik_request.ik_link_name = \\"panda_hand\\"\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
timeout 120 python3 -u -c "
from robolib import *
r = Robot('t')
q = r.solve_ik((-0.085,-0.14,0.62)); print('ik above can', q)
if q: print('fk check', r.tcp_pose(q))
"

# openrua op 18
mkdir -p "$(dirname /workspace/phase1_pick_can.py)"
cat > /workspace/phase1_pick_can.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the alphabet soup can at (-0.085, -0.14); can top z=0.502, table 0.426."""
from robolib import *

CAN = (-0.085, -0.14)
r = Robot("p1")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("open gripper"); r.gripper(GRIP["open_m"])
print("above can"); ok = r.move_to((CAN[0], CAN[1], 0.62), secs=3.0, label="above")
print("descend"); ok = r.move_to((CAN[0], CAN[1], 0.466), secs=2.5, label="grasp")
print("close"); r.gripper(GRIP["closed_m"])
gap = r.finger_gap(); print("gap after close", round(gap, 4))
print("lift"); r.move_to((CAN[0], CAN[1], 0.66), secs=2.5, label="lift")
print("gap after lift", round(r.finger_gap(), 4))
print("PHASE1 DONE")
OPENRUA_EOF

# openrua op 19
nohup python3 -u phase1_pick_can.py > phase1.log 2>&1 &
sleep 45; cat /workspace/phase1.log

# openrua op 20
sleep 20; cat /workspace/phase1.log; pgrep -f phase1_pick_can.py || echo "process exited"

# openrua op 21
timeout 60 python3 tools/perception/cam_snap.py agentview agent_p1.png && timeout 60 python3 tools/perception/cam_snap.py sideview side_p1.png

# openrua op 22
mkdir -p "$(dirname /workspace/phase2_place_can.py)"
cat > /workspace/phase2_place_can.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the held can over the basket center (0, 0.27) and release."""
from robolib import *

BASKET = (0.0, 0.27)
r = Robot("p2")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("raise"); r.move_to((-0.085, -0.14, 0.74), secs=2.0, label="raise")
print("via"); r.move_to((-0.04, 0.06, 0.74), secs=2.5, label="via")
print("above basket"); r.move_to((BASKET[0], BASKET[1], 0.74), secs=2.5, label="above basket")
print("lower"); r.move_to((BASKET[0], BASKET[1], 0.69), secs=2.0, label="lower")
print("gap before release", round(r.finger_gap(), 4))
print("open"); r.gripper(GRIP["open_m"])
print("retreat"); r.move_to((BASKET[0], BASKET[1], 0.80), secs=2.0, label="retreat")
print("PHASE2 DONE")
OPENRUA_EOF

# openrua op 23
nohup python3 -u phase2_place_can.py > phase2.log 2>&1 &
sleep 60; cat /workspace/phase2.log

# openrua op 24
sleep 12; tail -3 /workspace/phase2.log; pgrep -f phase2_place_can.py || echo "process exited"; timeout 60 python3 tools/perception/cam_snap.py agentview agent_p2.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand wrist_p2.png

# openrua op 25
timeout 120 python3 scan.py birdview 0.01 2>&1 | grep -E "table|h=0.0[0-4]"

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robolib.py", "old_string": "    def move_to(self, xyz, quat=DOWN, secs=3.0, at_tcp=True, label=\\"\\"):\\n        q = self.solve_ik(xyz, quat, at_tcp)\\n        if q is None:\\n            print(f\\"  !! no IK for {label} {xyz}\\")\\n            return False\\n        code, err = self.move_q(q, secs)\\n        p, _ = self.tcp_pose()", "new_string": "    def move_to(self, xyz, quat=DOWN, secs=3.0, at_tcp=True, label=\\"\\", tol=0.02):\\n        q = self.solve_ik(xyz, quat, at_tcp)\\n        if q is None:\\n            print(f\\"  !! no IK for {label} {xyz}\\")\\n            return False\\n        code, err = self.move_q(q, secs)\\n        for _ in range(3):  # resend to converge when tracking lagged\\n            if err < tol:\\n                break\\n            code, err = self.move_q(q, 1.5)\\n        p, _ = self.tcp_pose()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
mkdir -p "$(dirname /workspace/phase3_pick_butter.py)"
cat > /workspace/phase3_pick_butter.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the butter at (0.030, 0.061); top z=0.454, table 0.426. Fingers along y (3.5 cm side)."""
from robolib import *

B = (0.030, 0.061)
r = Robot("p3")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("above butter"); r.move_to((B[0], B[1], 0.60), secs=3.0, label="above")
print("pre-grasp"); r.move_to((B[0], B[1], 0.48), secs=2.0, label="pre", tol=0.01)
print("descend"); r.move_to((B[0], B[1], 0.437), secs=2.0, label="grasp", tol=0.005)
print("close"); r.gripper(GRIP["closed_m"])
print("gap after close", round(r.finger_gap(), 4))
print("lift"); r.move_to((B[0], B[1], 0.60), secs=2.5, label="lift")
print("gap after lift", round(r.finger_gap(), 4))
print("PHASE3 DONE")
OPENRUA_EOF

# openrua op 28
nohup python3 -u phase3_pick_butter.py > phase3.log 2>&1 &
sleep 60; cat /workspace/phase3.log

# openrua op 29
sleep 25; cat /workspace/phase3.log; pgrep -f phase3_pick_butter.py || echo "process exited"

# openrua op 30
mkdir -p "$(dirname /workspace/phase4_place_butter.py)"
cat > /workspace/phase4_place_butter.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the held butter over the basket and release (offset from the can at center)."""
from robolib import *

DROP = (-0.025, 0.29)
r = Robot("p4")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("raise"); r.move_to((0.03, 0.061, 0.74), secs=2.0, label="raise")
print("above basket"); r.move_to((DROP[0], DROP[1], 0.74), secs=2.5, label="above basket")
print("lower"); r.move_to((DROP[0], DROP[1], 0.665), secs=2.0, label="lower")
print("gap before release", round(r.finger_gap(), 4))
print("open"); r.gripper(GRIP["open_m"])
print("retreat"); r.move_to((DROP[0], DROP[1], 0.82), secs=2.0, label="retreat")
print("PHASE4 DONE")
OPENRUA_EOF

# openrua op 31
nohup python3 -u phase4_place_butter.py > phase4.log 2>&1 &
sleep 55; cat /workspace/phase4.log; pgrep -f phase4_place_butter.py || echo "process exited"

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand wrist_final.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent_final.png; timeout 120 python3 scan.py birdview 0.01 2>&1 | grep -vE "area= ?[0-9]{4,}"

# openrua op 33
timeout 120 python3 -u -c "
import numpy as np, rclpy, sys
sys.argv=['x','robot0_eye_in_hand']
import scan
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
rclpy.init(); node=rclpy.create_node('chk'); buf=Buffer(); TransformListener(buf,node)
cam='robot0_eye_in_hand'
d=scan.grab(node,f'/{cam}/depth/image_raw',Image); info=scan.grab(node,f'/{cam}/color/camera_info',CameraInfo)
depth=CvBridge().imgmsg_to_cv2(d,'passthrough').astype(float)
import time; t0=time.time()
while not buf.can_transform('world',cam+'_optical_frame',rclpy.time.Time()) and time.time()-t0<10: rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',cam+'_optical_frame',rclpy.time.Time()); q=t.transform.rotation
R=scan.quat_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]; H,W=depth.shape
us,vs=np.meshgrid(np.arange(W),np.arange(H)); P=np.stack([(us-cx)*depth/fx,(vs-cy)*depth/fy,depth],-1).reshape(-1,3)@R.T+T
inside=(P[:,0]>-0.06)&(P[:,0]<0.06)&(P[:,1]>0.20)&(P[:,1]<0.33)&(P[:,2]>0.44)&(P[:,2]<0.60)&np.isfinite(P[:,2])
pts=P[inside]; print('points inside basket footprint above its floor:',len(pts)); print('z range',pts[:,2].min().round(3),pts[:,2].max().round(3))
print('xy centroid',pts[:,:2].mean(0).round(3))
"

# openrua op 34
pgrep -af "phase|robolib" || echo "no background motion processes"; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tr -d '\n'; echo
