#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; timeout 10 ros2 topic echo /tf --once 2>&1 | head -40

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once | head -30 && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
print(d[240,320], d[290,283], d[280,375])
"

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Compute a world-frame height map from a camera's depth image.

Usage: python3 scene.py <camera>
Saves <camera>_world.npy: HxWx3 world xyz per pixel, and prints table
height estimate + connected blobs above table.
"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    from cv_bridge import CvBridge
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    np.save(f"{cam}_world.npy", P)
    cv2.imwrite(f"{cam}.png", color)
    z = P[..., 2]
    print("camera pos", T)
    # table height: mode of z in the central region
    zz = z[np.isfinite(z)]
    hist, edges = np.histogram(zz, bins=400)
    table = edges[np.argmax(hist)]
    print(f"dominant z (table?) = {table:.3f}")
    # blobs above table
    mask = (z > table + 0.015) & (z < table + 0.5)
    mask = mask.astype(np.uint8)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        m = lab == i
        pts = P[m]
        print(f"blob {i}: px centroid ({cent[i][0]:.0f},{cent[i][1]:.0f}) area {stats[i, cv2.CC_STAT_AREA]} "
              f"world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"ztop {pts[:,2].max():.3f} mean xyz ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scene.py birdview

# openrua op 9
python3 -c "
import numpy as np, cv2
P=np.load('birdview_world.npy'); z=P[...,2]
zz=z[(z>0.3)&(z<0.6)]
h,e=np.histogram(zz,bins=300); table=e[np.argmax(h)]; print('table z',table)
mask=((z>table+0.012)&(z<table+0.4)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<10: continue
    m=lab==i; pts=P[m]
    print(f'blob {i}: px ({cent[i][0]:.0f},{cent[i][1]:.0f}) area {stats[i,cv2.CC_STAT_AREA]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} mean ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f})')
"


# openrua op 10
timeout 60 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" ; echo; timeout 120 python3 scene.py robot0_eye_in_hand 2>&1 | head -5

# openrua op 11
timeout 120 python3 scene.py robot0_eye_in_hand 2>&1 | tail -n +3

# openrua op 12
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver 2>/dev/null; timeout 30 ros2 param list /move_group 2>/dev/null | grep -i -E "tip|kinematics" | head

# openrua op 13
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable controller for this Panda: FK/IK, trajectory, gripper, sensing.

World frame poses are converted to the arm base (panda_link0) for MoveIt
(world -> base offset read from /tf).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])

# top-down grasp: hand z -> -world z, hand x -> +world x (fingers along world y)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(yaw):
    """Top-down grasp with fingers rotated by `yaw` (rad) about world z
    from the Q_DOWN configuration (fingers along world y)."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # qz = (0,0,s,c); qx = (1,0,0,0); product qz*qx:
    # w = c*0 - s*0 = 0? compute properly: (w1w2 - v1.v2, w1v2 + w2v1 + v1xv2)
    w1, v1 = c, np.array([0, 0, s])
    w2, v2 = 0.0, np.array([1.0, 0, 0])
    w = w1 * w2 - v1 @ v2
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_ctl")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(30)
        self.grip.wait_for_server(30)
        self.ik.wait_for_service(30)
        self.fk.wait_for_service(30)
        # world -> base offset
        self.base_off = None
        sub = self.node.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        while self.base_off is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.destroy_subscription(sub)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "world" and t.child_frame_id == "panda_link0":
                tr = t.transform.translation
                self.base_off = np.array([tr.x, tr.y, tr.z])

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return [self.js[j] for j in ARM]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def _arm_state(self):
        st = JointState()
        st.name = list(ARM)
        st.position = self.joints()
        return st

    def fk_hand(self, positions=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        st = JointState()
        st.name = list(ARM)
        st.position = positions if positions is not None else self.joints()
        req.robot_state.joint_state = st
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base_off
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), res.error_code.val

    def ik_world(self, xyz, quat, at_tcp=True, seed=None):
        """IK for the hand (or TCP) at a world-frame pose. Returns joint list or None."""
        xyz = np.array(xyz, dtype=float)
        if at_tcp:
            R = quat_to_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        base = xyz - self.base_off
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = base.tolist()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        st = JointState()
        st.name = list(ARM)
        st.position = list(seed) if seed is not None else self.joints()
        req.ik_request.robot_state.joint_state = st
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed: {None if res is None else res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        t0 = time.time()
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        code = res.result().result.error_code if res.result() else None
        cur = np.array(self.joints())
        err = np.abs(cur - np.array(positions)).max()
        print(f"traj done code={code} max_joint_err={err:.4f} ({time.time()-t0:.0f}s wall)")
        return code, err

    def move_world(self, xyz, quat=Q_DOWN, seconds=3.0, at_tcp=True, seed=None):
        sol = self.ik_world(xyz, quat, at_tcp=at_tcp, seed=seed)
        if sol is None:
            return None
        code, err = self.move_joints(sol, seconds)
        pos, q, _ = self.fk_hand()
        R = quat_to_R(*q)
        tcp = pos + TCP * R[:, 2]
        print(f"  hand at {np.round(pos,4)} tcp at {np.round(tcp,4)} (target {np.round(xyz,4)})")
        return sol

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result if res.result() else None
        f = self.fingers()
        print(f"gripper -> {width}: reached={getattr(r,'reached_goal',None)} stalled={getattr(r,'stalled',None)} fingers={f}")
        return f

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        end = time.time() + 60
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out
OPENRUA_EOF

# openrua op 14
timeout 300 python3 -u -c "
from robot import *
r = Robot()
print('base_off', r.base_off)
print('joints', np.round(r.joints(),3))
print('fingers', r.fingers())
pos, q, code = r.fk_hand()
print('FK hand world', np.round(pos,4), 'quat', np.round(q,4), 'code', code)
R = quat_to_R(*q); print('hand z axis', np.round(R[:,2],3), 'hand y axis', np.round(R[:,1],3))
print('tcp', np.round(pos + TCP*R[:,2],4))
# IK round-trip test at current pose
sol = r.ik_world(pos, q, at_tcp=False)
print('IK roundtrip', None if sol is None else np.round(sol,3))
print('yaw_down_quat(0)', yaw_down_quat(0.0))
"


# openrua op 15
timeout 300 python3 -u -c "
from robot import *
r = Robot()
for fid in ['', 'world', 'panda_link0']:
    req = GetPositionFK.Request(); req.header.frame_id = fid
    req.fk_link_names = ['panda_link0','panda_hand']
    req.robot_state.joint_state = r._arm_state()
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res = fut.result()
    for ps in res.pose_stamped:
        print(repr(fid), ps.header.frame_id, np.round([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z],4), 'code', res.error_code.val)
"


# openrua op 16
python3 - <<'EOF'
import re
s = open('robot.py').read()
s = s.replace('''        # world -> base offset
        self.base_off = None
        sub = self.node.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        while self.base_off is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.destroy_subscription(sub)
''', '''        # verified: MoveIt's model frame is `world` (panda_link0 at
        # (-0.51,0,0.42) inside it), so empty-frame poses ARE world poses
        self.base_off = np.zeros(3)
''')
open('robot.py','w').write(s)
EOF
grep -n base_off robot.py

# openrua op 17
timeout 900 python3 -u -c "
from robot import *
r = Robot()
r.gripper(0.04)
sol = r.move_world([-0.014, -0.268, 0.60], Q_DOWN, seconds=3.0)
print('sol', None if sol is None else np.round(sol,3))
r.snap('robot0_eye_in_hand', 'eih1.png')
" 2>&1 | grep -v Warning

# openrua op 18
timeout 60 python3 -u -c "
from robot import *
r = Robot(); print(np.round(r.joints(),3)); print(r.fingers())
pos,q,_ = r.fk_hand(); print(pos, q)
" 2>&1 | grep -v Warning

# openrua op 19
timeout 60 python3 -u -c "
from robot import *
r = Robot()
cur = r.joints()
sol = [-0.178, 0.258, -0.322, -2.159, 0.12, 2.401, -0.579]
for name, j in [('cur', cur), ('sol', sol)]:
    pos, q, c = r.fk_hand(j); R = quat_to_R(*q)
    print(name, np.round(j,3), 'pos', np.round(pos,3), 'q', np.round(q,3), 'hand x axis', np.round(R[:,0],2), 'hand y', np.round(R[:,1],2))
" 2>&1 | grep -v Warning

# openrua op 20
timeout 600 python3 -u -c "
from robot import *
r = Robot()
cur = r.joints(); print('cur', np.round(cur,3))
tgt = list(cur); tgt[6] = cur[6] - 0.3
r.move_joints(tgt, 2.0)
print('now', np.round(r.joints(),3))
tgt2 = list(cur)
r.move_joints(tgt2, 2.0)
print('back', np.round(r.joints(),3))
" 2>&1 | grep -v Warning

# openrua op 21
timeout 600 python3 -u -c "
from robot import *
r = Robot()
cur = r.joints(); print('cur', np.round(cur,3))
tgt = list(cur); tgt[6] = cur[6] + 0.8
r.move_joints(tgt, 3.0)
print('now', np.round(r.joints(),3))
r.move_joints(list(cur), 3.0)
print('back', np.round(r.joints(),3))
" 2>&1 | grep -v Warning

# openrua op 22
python3 - <<'EOF'
s = open('robot.py').read()
s = s.replace('''def yaw_down_quat(yaw):''', '''def qmul(q1, q2):
    """Hamilton product, quaternions as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# verified: /compute_ik solves for panda_link8, which is rotated -45 deg
# about z from panda_hand (tf_static). hand = link8 * Rz(-45) => link8 = hand * Rz(+45)
Q_HAND_TO_LINK8 = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))

# joint7 tracks at most ~0.19 rad/s on this machine; pace moves by it
MAX_JOINT_VEL = 0.15


def yaw_down_quat(yaw):''')
s = s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        st = JointState()''', '''        q8 = qmul(quat, Q_HAND_TO_LINK8)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q8
        st = JointState()''')
s = s.replace('''        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]''', '''        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        sol = [sol[j] for j in ARM]
        # verify the hand pose of the solution against the request
        pos, q, _ = self.fk_hand(sol)
        R = quat_to_R(*q)
        Rt = quat_to_R(*quat)
        ang = np.degrees(np.arccos(np.clip((np.trace(R.T @ Rt) - 1) / 2, -1, 1)))
        perr = np.linalg.norm(pos - xyz)
        if perr > 0.005 or ang > 3:
            print(f"IK solution mismatch: pos err {perr:.4f} m, ang err {ang:.1f} deg")
            return None
        return sol''')
s = s.replace('''    def move_joints(self, positions, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()''', '''    def move_joints(self, positions, seconds=3.0, via=None):
        delta = np.abs(np.array(positions) - np.array(self.joints())).max()
        seconds = max(seconds, float(delta) / MAX_JOINT_VEL)
        goal = FollowJointTrajectory.Goal()''')
s = s.replace('''        print(f"traj done code={code} max_joint_err={err:.4f} ({time.time()-t0:.0f}s wall)")
        return code, err''', '''        print(f"traj done code={code} max_joint_err={err:.4f} ({seconds:.1f}s traj, {time.time()-t0:.0f}s wall)")
        return code, err''')
open('robot.py','w').write(s)
EOF
timeout 120 python3 -u -c "
from robot import *
r = Robot()
sol = r.ik_world([-0.014, -0.268, 0.60], Q_DOWN)
print('sol', np.round(sol,3), 'cur', np.round(r.joints(),3))
" 2>&1 | grep -v Warning

# openrua op 23
timeout 1200 python3 -u -c "
from robot import *
r = Robot()
r.move_world([-0.014, -0.268, 0.50], Q_DOWN, seconds=2.5)
r.move_world([-0.014, -0.268, 0.437], Q_DOWN, seconds=2.5)
r.snap('robot0_eye_in_hand', 'eih2.png')
r.snap('agentview', 'agent2.png')
" 2>&1 | grep -v Warning

# openrua op 24
timeout 1200 python3 -u -c "
from robot import *
r = Robot()
r.gripper(0.0)
r.move_world([-0.014, -0.268, 0.60], Q_DOWN, seconds=2.5)
print('fingers after lift', r.fingers())
r.snap('agentview', 'agent3.png')
" 2>&1 | grep -v Warning

# openrua op 25
timeout 1500 python3 -u -c "
from robot import *
r = Robot()
r.move_world([-0.014, -0.268, 0.72], Q_DOWN, seconds=2.5)
r.move_world([-0.006, 0.250, 0.72], Q_DOWN, seconds=4.0)
print('fingers', r.fingers())
r.snap('robot0_eye_in_hand', 'eih4.png')
r.snap('agentview', 'agent4.png')
" 2>&1 | grep -v Warning

# openrua op 26
timeout 600 python3 -u -c "
from robot import *
r = Robot()
r.gripper(0.04)
r.snap('agentview', 'agent5.png')
r.snap('robot0_eye_in_hand', 'eih5.png')
" 2>&1 | grep -v Warning

# openrua op 27
timeout 120 python3 scene.py robot0_eye_in_hand 2>&1 | grep -v Warning

# openrua op 28
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
sub=P[300:370, 460:525]; z=sub[...,2]
m=(z>0.435)&(z<0.5)
pts=sub[m]; print(len(pts))
print('x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'ztop',pts[:,2].max(),'mean',pts.mean(0))
"

# openrua op 29
timeout 1500 python3 -u -c "
from robot import *
r = Robot()
r.move_world([-0.072, 0.047, 0.60], Q_DOWN, seconds=3.0)
r.snap('robot0_eye_in_hand', 'eih6.png')
r.move_world([-0.072, 0.047, 0.50], Q_DOWN, seconds=2.5)
r.move_world([-0.072, 0.047, 0.437], Q_DOWN, seconds=2.5)
r.snap('agentview', 'agent6.png')
" 2>&1 | grep -v Warning

# openrua op 30
timeout 1800 python3 -u -c "
from robot import *
r = Robot()
f = r.gripper(0.0)
r.move_world([-0.072, 0.047, 0.60], Q_DOWN, seconds=2.5)
print('fingers after lift', r.fingers())
r.snap('agentview', 'agent7.png')
" 2>&1 | grep -v Warning

# openrua op 31
timeout 1800 python3 -u -c "
from robot import *
r = Robot()
r.move_world([-0.072, 0.047, 0.72], Q_DOWN, seconds=2.5)
r.move_world([0.010, 0.250, 0.72], Q_DOWN, seconds=3.0)
print('fingers', r.fingers())
r.snap('robot0_eye_in_hand', 'eih8.png')
" 2>&1 | grep -v Warning

# openrua op 32
timeout 900 python3 -u -c "
from robot import *
r = Robot()
r.gripper(0.04)
r.snap('robot0_eye_in_hand', 'eih9.png')
# retreat up and back so external cameras can see into the basket
r.move_world([-0.10, 0.10, 0.80], Q_DOWN, seconds=3.0)
r.snap('agentview', 'agent9.png')
r.snap('birdview', 'bird9.png')
" 2>&1 | grep -v Warning

# openrua op 33
timeout 120 python3 scene.py birdview 2>&1 | grep -v Warning | tail -n +2; python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
# objects left at the two original pick locations?
for name,(x,y) in {'cream cheese spot':(-0.014,-0.268),'butter spot':(-0.072,0.047)}.items():
    m=(np.abs(P[...,0]-x)<0.04)&(np.abs(P[...,1]-y)<0.03)&np.isfinite(z)
    print(name, 'max z', z[m].max().round(3), '(table 0.425)')
# inside basket footprint, what is visible (basket floor ~0.425+liner; objects raise it)
m=(np.abs(P[...,0]+0.006)<0.05)&(np.abs(P[...,1]-0.25)<0.05)&np.isfinite(z)
print('basket interior z range', z[m].min().round(3), z[m].max().round(3))
"

# openrua op 34
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
m=(np.abs(P[...,0]+0.072)<0.04)&(np.abs(P[...,1]-0.047)<0.03)&np.isfinite(z)&(z<0.7)
print('butter spot (arm excluded) z range', z[m].min().round(3), z[m].max().round(3), 'n', m.sum())
"
