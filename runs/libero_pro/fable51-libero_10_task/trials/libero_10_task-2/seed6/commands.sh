#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
ros2 topic list; echo ---ACTIONS---; ros2 action list; echo ---SERVICES---; ros2 service list | grep -v -E "parameter|describe_parameters|list_parameters" ; echo ---NODES---; ros2 node list

# openrua op 3
cat tools/perception/cam_snap.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 4
cat tools/action/ik_move.py tools/perception/px2world.py

# openrua op 5
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 6
timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 7
mkdir -p "$(dirname /workspace/px_many.py)"
cat > /workspace/px_many.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""px_many.py <camera> u,v [u,v ...]  -> world xyz per pixel (depth+intrinsics+TF)."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    pts = [tuple(int(v) for v in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_many")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    for u, v in pts:
        z = float(D[v, u])
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 px_many.py birdview 243,280 290,282 300,282 377,282 377,237 316,316 200,200 450,350 320,60

# openrua op 9
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers for this Panda machine (see machine.yaml).

All poses here are in the panda_link0 (arm base) frame unless noted;
world -> base offset is WB (read from TF once at import via tf2_echo).
"""
import math
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
WB = np.array([-0.660, 0.000, 0.912])  # world -> panda_link0 translation
TCP = M["hand"]["tcp_offset_m"]


def w2b(p):
    return np.asarray(p, float) - WB


def b2w(p):
    return np.asarray(p, float) + WB


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def down_quat(yaw):
    """Hand Z pointing down (-Z world), fingers' opening axis rotated by yaw
    about world Z. yaw=0 -> hand X along +X base."""
    return quat_from_rpy(math.pi, 0.0, yaw)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 30
        while self._js is None and time.time() < end:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in JOINTS]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Return (pos_base, quat) of link for arm joints q (default current)."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def hand_world(self):
        p, q = self.fk()
        return b2w(p), q

    def tcp_world(self, q=None):
        p, quat = self.fk(q)
        R = quat_R(*quat)
        return b2w(p + TCP * R[:, 2]), quat

    # ---------- IK ----------
    def ik(self, pos_base, quat, seed=None, at_tcp=False, timeout=5.0, attempts=1):
        pos_base = np.asarray(pos_base, float)
        if at_tcp:
            R = quat_R(*quat)
            pos_base = pos_base - TCP * R[:, 2]
        if seed is None:
            seed = self.arm_q()
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def ik_world(self, pos_world, quat, **kw):
        return self.ik(w2b(pos_world), quat, **kw)

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, via=None):
        """Send a trajectory to joint config q (optionally through via points
        [(q, t), ...]). Returns error_code."""
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for vq, vt in (via or []):
            pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
            pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        r = res.result()
        code = r.result.error_code if r is not None else None
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code

    def move_pose_world(self, pos_world, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos_world, quat, at_tcp=at_tcp, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for world {pos_world}")
        return self.move_q(q, seconds)

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def twist(self, lin=(0, 0, 0), ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg)
            self.spin(dt)
OPENRUA_EOF

# openrua op 10
timeout 300 python3 -u -c "
from rob import *
r = Robot()
print('q', r.arm_q())
p, q = r.hand_world(); print('hand world', p, 'quat', q)
t, _ = r.tcp_world(); print('tcp world', t)
print('gap', r.finger_gap())
r.gripper(GRIP['open_m'])
print('q after', r.arm_q())
t, _ = r.tcp_world(); print('tcp world after', t)
" 2>&1 | grep -v Warning

# openrua op 11
timeout 300 python3 -u -c "
from rob import *
import numpy as np
r = Robot()
q0 = np.array(r.arm_q())
p, quat = r.fk()
print('fk raw', p, quat)
s1 = r.ik(p, quat)
print('IK at raw fk pos ->', None if s1 is None else np.round(np.array(s1)-q0,3))
s2 = r.ik(p - WB, quat)
print('IK at fk pos - WB ->', None if s2 is None else np.round(np.array(s2)-q0,3))
" 2>&1 | grep -v Warning

# openrua op 12
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "WB = np.array([-0.660, 0.000, 0.912])  # world -> panda_link0 translation", "new_string": "# Verified by FK->IK round trip: MoveIt\'s model root here coincides with\\n# the world frame (FK output matches camera-derived world coords), so no\\n# offset is applied.\\nWB = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 13
timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 px_many.py birdview 243,280 290,282 300,282 377,282 377,237 316,316 243,265 243,295 228,280 258,280

# openrua op 14
python3 -c "
import cv2
im = cv2.imread('agentview.png'); cv2.imwrite('knob_agent.png', cv2.resize(im[190:280, 370:480], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('sideview.png'); cv2.imwrite('knob_side.png', cv2.resize(im[300:380, 340:430], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('birdview.png'); cv2.imwrite('knob_bird.png', cv2.resize(im[215:260, 350:405], None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC))
"

# openrua op 15
timeout 20 ros2 topic echo /pipeline_state --once 2>&1 | head -20; timeout 20 ros2 topic info /pipeline_state; timeout 30 ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 16
timeout 120 python3 px_many.py birdview 368,237 386,237 377,228 377,246 377,225 377,250 262,282 270,282 280,282 290,282 300,282 308,282 290,279 290,285 243,264 243,258 377,282 377,270 360,282 395,282

# openrua op 17
timeout 600 python3 -u -c "
from rob import *
import numpy as np
r = Robot()
knob = np.array([-0.213, 0.202])
quat = down_quat(0.0)
print('quat', np.round(quat,4))
q = r.ik_world([knob[0], knob[1], 1.06], quat, at_tcp=True)
print('ik', None if q is None else np.round(q,3))
if q is not None:
    r.move_q(q, 4.0)
    t, qq = r.tcp_world(); print('tcp now', np.round(t,4), 'quat', np.round(qq,4))
" 2>&1 | grep -v Warning

# openrua op 18
timeout 300 python3 -u -c "
from rob import *
import numpy as np
r = Robot()
for l in ['panda_link8','panda_hand','panda_hand_tcp','panda_leftfinger','panda_rightfinger']:
    try:
        p,q = r.fk(link=l); print(l, np.round(p,4), np.round(q,4))
    except Exception as e: print(l, 'ERR', e)
" 2>&1 | grep -v Warning

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "def down_quat(yaw):", "new_string": "def quat_mul(a, b):\\n    \\"\\"\\"Hamilton product of (x,y,z,w) quaternions: a then b (b in a\'s frame).\\"\\"\\"\\n    x1, y1, z1, w1 = a\\n    x2, y2, z2, w2 = b\\n    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,\\n            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,\\n            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,\\n            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)\\n\\n\\n# The IK tip link is panda_link8; panda_hand = link8 rotated -45 deg about\\n# Z (verified by FK on both links). Requests for the HAND are converted.\\nHAND_TO_LINK8 = quat_from_rpy(0.0, 0.0, math.pi / 4) if False else None\\n\\n\\ndef down_quat(yaw):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "HAND_TO_LINK8 = quat_from_rpy(0.0, 0.0, math.pi / 4) if False else None", "new_string": "HAND_TO_LINK8 = quat_from_rpy(0.0, 0.0, math.pi / 4)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        pos_base = np.asarray(pos_base, float)\\n        if at_tcp:\\n            R = quat_R(*quat)\\n            pos_base = pos_base - TCP * R[:, 2]", "new_string": "        pos_base = np.asarray(pos_base, float)\\n        if at_tcp:\\n            R = quat_R(*quat)\\n            pos_base = pos_base - TCP * R[:, 2]\\n        quat = quat_mul(quat, HAND_TO_LINK8)  # hand request -> link8 request", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
timeout 600 python3 -u -c "
from rob import *
import numpy as np
r = Robot()
knob = np.array([-0.213, 0.202])
quat = down_quat(0.0)
q = r.ik_world([knob[0], knob[1], 1.06], quat, at_tcp=True)
print('ik', None if q is None else np.round(q,3))
if q is not None:
    r.move_q(q, 3.0)
    t, qq = r.tcp_world(); print('tcp now', np.round(t,4), 'hand quat', np.round(qq,4))
" 2>&1 | grep -v Warning

# openrua op 23
timeout 600 python3 -u -c "
from rob import *
import numpy as np
r = Robot()
q = [0.304, 0.135, 0.118, -2.431, -0.029, 2.564, 1.231]
r.move_q(q, 3.0)
t, qq = r.tcp_world(); print('tcp now', np.round(t,4), 'hand quat', np.round(qq,4))
print('q', np.round(r.arm_q(),3))
" 2>&1 | grep -v Warning

# openrua op 24
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
timeout 900 python3 -u -c "
from rob import *
import numpy as np
r = Robot()
knob = np.array([-0.213, 0.202])
quat = down_quat(0.0)
q = r.ik_world([knob[0], knob[1], 0.94], quat, at_tcp=True)
print('ik', None if q is None else np.round(q,3))
code = r.move_q(q, 2.5)
if code != 0: code = r.move_q(q, 2.5)
t, qq = r.tcp_world(); print('tcp now', np.round(t,4), 'hand quat', np.round(qq,4))
r.gripper(0.0)
print('gap after close', r.finger_gap())
" 2>&1 | grep -v Warning

# openrua op 26
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
q = r.arm_q(); print('q before', np.round(q,3))
q2 = list(q); q2[6] = q[6] - math.pi/2
code = r.move_q(q2, 3.0)
if code != 0: code = r.move_q(q2, 3.0)
print('q after', np.round(r.arm_q(),3), 'gap', round(r.finger_gap(),4))
t, qq = r.tcp_world(); print('tcp', np.round(t,4), 'quat', np.round(qq,4))
" 2>&1 | grep -v Warning

# openrua op 27
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 -c "
import cv2
im = cv2.imread('agentview.png'); cv2.imwrite('knob_agent.png', cv2.resize(im[150:300, 330:520], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 28
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
r.gripper(GRIP['open_m'])
q = r.arm_q()
# lift straight up 12 cm keeping current orientation
t, quat = r.tcp_world()
q2 = r.ik_world([t[0], t[1], t[2]+0.12], quat, at_tcp=True)
print('ik', None if q2 is None else np.round(q2,3))
code = r.move_q(q2, 2.5)
if code != 0: r.move_q(q2, 2.5)
t, qq = r.tcp_world(); print('tcp', np.round(t,4), 'gap', round(r.finger_gap(),4))
" 2>&1 | grep -v Warning

# openrua op 29
timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 px_many.py birdview 243,280 290,282 280,282 270,282 300,282 377,237 377,282 && python3 -c "
import cv2
im = cv2.imread('birdview.png'); cv2.imwrite('knob_bird.png', cv2.resize(im[215:300, 220:405], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 30
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(math.pi/2)
seed = r.arm_q()
for name, p in [('pre-grasp', [-0.051,-0.12,1.02]), ('grasp', [-0.051,-0.12,0.93]), ('lift', [-0.051,-0.12,1.0]), ('over-stove', [-0.046,0.363,1.0]), ('place', [-0.046,0.363,0.965])]:
    q = r.ik_world(p, quat, at_tcp=True, seed=seed)
    print(name, p, None if q is None else np.round(q,3))
    if q is not None: seed = q
" 2>&1 | grep -v Warning

# openrua op 31
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(math.pi/2)
q = r.ik_world([-0.051,-0.12,1.02], quat, at_tcp=True)
code = r.move_q(q, 4.0)
if code != 0: r.move_q(q, 3.0)
t, qq = r.tcp_world(); print('tcp', np.round(t,4), 'quat', np.round(qq,4), 'gap', round(r.finger_gap(),4))
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 32
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(math.pi/2)
q = r.ik_world([-0.051,-0.12,0.93], quat, at_tcp=True)
code = r.move_q(q, 2.5)
if code != 0: r.move_q(q, 2.5)
t, qq = r.tcp_world(); print('tcp', np.round(t,4))
r.gripper(0.0)
print('gap after close', round(r.finger_gap(),4))
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 33
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(math.pi/2)
q = r.ik_world([-0.051,-0.12,1.0], quat, at_tcp=True)
code = r.move_q(q, 2.5)
if code != 0: r.move_q(q, 2.5)
t, qq = r.tcp_world(); print('tcp', np.round(t,4), 'gap', round(r.finger_gap(),4))
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 34
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(math.pi/2)
q = r.ik_world([-0.046,0.363,1.0], quat, at_tcp=True)
code = r.move_q(q, 4.0)
if code != 0: r.move_q(q, 3.0)
t, qq = r.tcp_world(); print('tcp', np.round(t,4), 'gap', round(r.finger_gap(),4))
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py birdview

# openrua op 35
python3 -c "
import cv2
im = cv2.imread('birdview.png'); cv2.imwrite('zoom.png', cv2.resize(im[220:340, 320:470], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))" && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 36
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(math.pi/2)
seed = r.arm_q()
plan = [([-0.046,0.363,1.12], 2.5), ([-0.096,-0.091,1.12], 4.0), ([-0.096,-0.091,0.935], 2.5)]
for p, s in plan:
    q = r.ik_world(p, quat, at_tcp=True, seed=seed)
    if q is None: print('IK fail', p); break
    seed = q
    code = r.move_q(q, s)
    if code != 0: r.move_q(q, s)
    t, _ = r.tcp_world(); print('tcp', np.round(t,4), 'gap', round(r.finger_gap(),4))
r.gripper(GRIP['open_m'])
q = r.ik_world([-0.096,-0.091,1.05], quat, at_tcp=True, seed=seed)
r.move_q(q, 2.5)
t, _ = r.tcp_world(); print('tcp', np.round(t,4))
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 37
python3 -c "
import cv2
im = cv2.imread('birdview.png'); cv2.imwrite('zoom.png', cv2.resize(im[240:350, 330:420], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))
im = cv2.imread('agentview.png'); cv2.imwrite('zoom2.png', cv2.resize(im[230:400, 340:540], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))" && timeout 120 python3 px_many.py birdview 365,300 365,310 365,320 365,330 358,315 372,315 350,300 362,296 368,296 377,282

# openrua op 38
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(0.0)
r.gripper(0.0)
seed = r.arm_q()
y = 0.155
wps = [([-0.09,y,1.06],3.5), ([-0.09,y,0.955],2.0)]
for p, s in wps:
    q = r.ik_world(p, quat, at_tcp=True, seed=seed)
    if q is None: raise SystemExit('IK fail %s'%p)
    print('dq', np.round(np.array(q)-np.array(seed),2))
    seed = q
    code = r.move_q(q, s)
    if code != 0: r.move_q(q, s)
    t,_ = r.tcp_world(); print('tcp', np.round(t,4))
# push along +x through via points
via = []; tt = 0
for x in [-0.03, 0.03, 0.09, 0.15]:
    q = r.ik_world([x,y,0.955], quat, at_tcp=True, seed=seed)
    if q is None: raise SystemExit('IK fail push %s'%x)
    print('dq', np.round(np.array(q)-np.array(seed),2)); seed = q
    tt += 1.5; via.append((q, tt))
last = via.pop()
code = r.move_q(last[0], last[1], via=via)
if code != 0: r.move_q(last[0], 2.0)
t,_ = r.tcp_world(); print('tcp after push', np.round(t,4))
q = r.ik_world([0.15,y,1.08], quat, at_tcp=True, seed=seed); r.move_q(q, 2.0)
t,_ = r.tcp_world(); print('tcp lifted', np.round(t,4))
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 39
timeout 900 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(0.0)
seed = r.arm_q()
y = 0.155
via = []; tt = 0
for x in [-0.03, 0.03, 0.07, 0.10]:
    q = r.ik_world([x,y,0.955], quat, at_tcp=True, seed=seed)
    if q is None: raise SystemExit('IK fail push %s'%x)
    print('dq', np.round(np.array(q)-np.array(seed),2)); seed = q
    tt += 1.5; via.append((q, tt))
last = via.pop()
code = r.move_q(last[0], last[1], via=via)
if code != 0: r.move_q(last[0], 2.0)
t,_ = r.tcp_world(); print('tcp after push', np.round(t,4))
q = r.ik_world([0.10,y,1.08], quat, at_tcp=True, seed=seed); r.move_q(q, 2.0)
t,_ = r.tcp_world(); print('tcp lifted', np.round(t,4))
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 40
python3 -c "
import cv2, numpy as np
im = cv2.imread('birdview.png')
# dark pan mask in the pan region (left half of table)
g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
m = (g < 60).astype(np.uint8); m[:, 330:] = 0; m[:200,:]=0; m[350:,:]=0
n, lab, st, cen = cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if st[i,4] > 200: print('comp', i, 'area', st[i,4], 'bbox', st[i,:4], 'centroid', cen[i])
" && timeout 120 python3 px_many.py birdview 232,287 262,286 270,286 276,286 285,286 295,286

# openrua op 41
timeout 120 python3 px_many.py birdview 254,268 290,268 300,268 306,268 296,265 296,271 296,262 296,274 240,268 268,268

# openrua op 42
timeout 1500 python3 -u -c "
from rob import *
import numpy as np, math
r = Robot()
quat = down_quat(math.pi/2)
def go(p, s, seed):
    q = r.ik_world(p, quat, at_tcp=True, seed=seed)
    if q is None: raise SystemExit('IK fail %s'%p)
    code = r.move_q(q, s)
    if code != 0: r.move_q(q, s)
    t,_ = r.tcp_world(); print('tcp', np.round(t,4), 'gap', round(r.finger_gap(),4))
    return q
r.gripper(GRIP['open_m'])
seed = r.arm_q()
seed = go([-0.1005,-0.11,1.03], 4.0, seed)
seed = go([-0.1005,-0.11,0.93], 2.5, seed)
r.gripper(0.0)
gap = r.finger_gap(); print('grasp gap', round(gap,4))
if gap < 0.01: raise SystemExit('missed grasp')
seed = go([-0.1005,-0.11,1.12], 2.5, seed)
seed = go([-0.0515,0.339,1.12], 4.0, seed)
seed = go([-0.0515,0.339,0.968], 3.0, seed)
r.gripper(GRIP['open_m'])
seed = go([-0.0515,0.339,1.10], 2.5, seed)
" 2>&1 | grep -v Warning && timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 43
python3 -c "
import cv2, numpy as np
im = cv2.imread('birdview.png')
g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
m = (g < 45).astype(np.uint8); m[:, :340] = 0; m[:, 420:]=0; m[:245,:]=0; m[320:,:]=0
n, lab, st, cen = cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if st[i,4] > 300: print('comp', i, 'area', st[i,4], 'bbox', st[i,:4], 'centroid', cen[i])
" && timeout 120 python3 px_many.py birdview 380,282 380,265 380,299 363,282 397,282 372,282 388,282 380,275 380,289

# openrua op 44
timeout 120 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 tools/perception/cam_snap.py frontview && python3 -c "
import cv2
im = cv2.imread('sideview.png'); cv2.imwrite('zoom.png', cv2.resize(im[280:400, 220:440], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"
