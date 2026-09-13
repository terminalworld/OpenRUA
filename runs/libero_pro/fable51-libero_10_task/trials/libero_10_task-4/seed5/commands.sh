#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | head -40; echo ---; ros2 node list 2>&1

# openrua op 4
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -8; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
for p in "215 235 whitemug" "320 200 redmug" "412 240 yellowmug" "85 293 leftplate" "562 293 rightplate" "320 330 table"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py agentview $1 $2; done

# openrua op 7
for p in "245 285 leftplate" "395 285 rightplate" "283 265 whitemug" "357 265 yellowmug" "305 237 redmug"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py birdview $1 $2; done; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A1 Translation

# openrua op 8
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table in a top-down camera and print world centroids.

Usage: python3 scan.py [camera=birdview]
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T  # world coords per pixel
    Z = P[..., 2]
    valid = np.isfinite(depth) & (depth > 0)
    table_z = 0.426
    mask = valid & (Z > table_z + 0.015) & (Z < table_z + 0.5)
    # exclude the robot: approx region near base
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 15:
            continue
        m = lab == i
        pts = P[m]
        col = color[m].mean(0)[::-1]
        u, v = cents[i]
        print(f"comp {i}: px=({u:.0f},{v:.0f}) area={area} "
              f"world centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
              f"zmax={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} "
              f"xrange=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yrange=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) rgb={col.astype(int)}")
    # plates: slightly above table
    pm = valid & (Z > table_z + 0.003) & (Z <= table_z + 0.015)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(pm.astype(np.uint8), 8)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 30:
            continue
        m = lab == i
        pts = P[m]
        u, v = cents[i]
        print(f"low comp {i}: px=({u:.0f},{v:.0f}) area={area} "
              f"world centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) zmax={pts[:,2].max():.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
timeout 120 python3 scan.py birdview

# openrua op 10
python3 - <<'EOF'
import re
src = open('scan.py').read()
src = src.replace("mask = valid & (Z > table_z + 0.015) & (Z < table_z + 0.5)", "mask = valid & (Z > 0.52) & (Z < 0.62)")
open('scan2.py','w').write(src)
EOF
timeout 120 python3 scan2.py birdview 2>&1 | grep -v "low comp"

# openrua op 11
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[220:310,220:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"

# openrua op 12
for p in "284 262 whitemug" "355 270 yellowmug" "353 268 yellowmug2" "406 65 redmug"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py birdview $1 $2; done

# openrua op 13
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | head -20; timeout 15 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -5

# openrua op 14
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for the Panda: joints, FK, IK, trajectories, gripper, servo.

All poses are in the arm base frame (panda_link0). World->base offset is
BASE_IN_WORLD; use w2b()/b2w() to convert.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])
TCP = float(M["hand"]["tcp_offset_m"])


def w2b(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def b2w(p):
    return np.asarray(p, float) + BASE_IN_WORLD


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def down_quat(yaw_deg=0.0):
    """Hand z pointing down (-Z base), fingers rotated by yaw about vertical.
    yaw=0 -> hand x axis along base +x (fingers close along base y)."""
    # rotation 180 deg about x: hand z -> -z
    q_flip = np.array([1.0, 0.0, 0.0, 0.0])
    yaw = math.radians(yaw_deg)
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    return quat_mul(q_yaw, q_flip)


class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] + abs(d["panda_finger_joint2"])

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.1)
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        """Return (pos, quat[x,y,z,w]) of link in base frame."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp(self, q=None):
        """TCP (fingertip centre) position in base frame."""
        p, qt = self.fk(q)
        R = quat_R(*qt)
        return p + TCP * R[:, 2], qt

    # ---------- planning ----------
    def ik(self, pos, quat, seed=None, at_tcp=True, attempts=1):
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(attempts):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        raise RuntimeError(f"IK failed ({res and res.error_code.val}) for {pos} {quat}")

    # ---------- action ----------
    def move_q(self, q, seconds=3.0, tol=0.01, retries=2):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        for i in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  move_q: code={code} max_err={err:.4f}", flush=True)
            if err < tol:
                return True
        return err < tol

    def move_qs(self, qs, seconds_each=2.0):
        """Multi-point trajectory."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        t = 0.0
        for q in qs:
            t += seconds_each
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()
        print(f"  move_qs: code={code} max_err={err:.4f}", flush=True)
        return err < 0.01

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed, at_tcp=True)
        ok = self.move_q(q, seconds)
        p, _ = self.tcp()
        print(f"  tcp now {p.round(4)} target {np.asarray(pos).round(4)}", flush=True)
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])

    def servo(self, v, n=20, dt=0.05):
        """Stream a base-frame linear velocity v (m/s) for n ticks."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)
        p, _ = self.tcp()
        print(f"  servo done, tcp {p.round(4)}", flush=True)
        return p
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -c "
from robot import *
r = Robot()
q = r.arm_q(); print('q', np.round(q,3))
p, qt = r.fk(q); print('hand', p.round(4), qt.round(4))
print('tcp', r.tcp(q)[0].round(4))
print('R hand', quat_R(*qt).round(3))
print('down_quat(0)', down_quat(0).round(3), 'down_quat(45)', down_quat(45).round(3))
print('gap', r.finger_gap())
print('wrench', r.wrench())
"

# openrua op 16
for p in "320 95 hand_top" "320 60 hand_upper" "322 140 hand_low"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py agentview $1 $2; done

# openrua op 17
timeout 60 python3 -c "
import numpy as np, math
def dh(a,d,alpha,theta):
    ca,sa,ct,st=math.cos(alpha),math.sin(alpha),math.cos(theta),math.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
P=[(0,0.333,0,q[0]),(0,0,-math.pi/2,q[1]),(0,0.316,math.pi/2,q[2]),(0.0825,0,math.pi/2,q[3]),(-0.0825,0.384,-math.pi/2,q[4]),(0,0,math.pi/2,q[5]),(0.088,0,math.pi/2,q[6]),(0,0.107,0,0)]
T=np.eye(4)
for p in P: T=T@dh(*p)
print('flange (link8)', T[:3,3].round(4)); print(T[:3,:3].round(3))
"


# openrua op 18
timeout 200 python3 -c "
from robot import *
r = Robot()
q0 = r.arm_q()
p, qt = r.fk(q0)
print('fk', p.round(4))
try:
    q = r.ik(p, qt, at_tcp=False); print('IK world-frame pose ->', np.round(q,3))
except Exception as e: print('world-frame IK fail', e)
try:
    q = r.ik(p - BASE_IN_WORLD, qt, at_tcp=False); print('IK base-frame pose ->', np.round(q,3))
except Exception as e: print('base-frame IK fail', e)
print('link0 fk', r.fk(q0, link='panda_link0')[0])
"

# openrua op 19
python3 -c "
import cv2
im=cv2.imread('sideview.png'); crop=im[300:480,230:430]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 20
python3 - <<'EOF'
s = open('robot.py').read()
s = s.replace('''All poses are in the arm base frame (panda_link0). World->base offset is
BASE_IN_WORLD; use w2b()/b2w() to convert.''', '''All poses are in the WORLD frame: MoveIt's model frame here is `world`
(verified: FK of panda_link0 returns (-0.51, 0, 0.42)), so IK/FK poses with
an empty frame_id are world-frame poses.''')
s = s.replace('"""Return (pos, quat[x,y,z,w]) of link in base frame."""', '"""Return (pos, quat[x,y,z,w]) of link in world frame."""')
s = s.replace('"""TCP (fingertip centre) position in base frame."""', '"""TCP (fingertip centre) position in world frame."""')
open('robot.py','w').write(s)
EOF
echo ok

# openrua op 21
mkdir -p "$(dirname /workspace/eyescan.py)"
cat > /workspace/eyescan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab the eye-in-hand colour+depth, save PNGs, and print world-frame
segments in a height band. Camera pose from FK (not TF, which can be stale).

Usage: python3 eyescan.py [zlo zhi] [tag]
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from robot import Robot, quat_R, quat_mul

CAM = "robot0_eye_in_hand"
# camera optical frame in panda_hand: from tf_static
CAM_T = np.array([0.050, 0.0, -0.001])
CAM_Q = np.array([0.0, 0.0, 0.7071068, 0.7071068])


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def cam_pose(r):
    p, q = r.fk()
    R = quat_R(*q)
    Rc = quat_R(*quat_mul(q, CAM_Q))
    return p + R @ CAM_T, Rc


def cloud(r, depth, info):
    """World-frame point cloud (H,W,3) for a depth image."""
    T, Rc = cam_pose(r)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    return np.stack([X, Y, depth], -1) @ Rc.T + T


def snap(r, tag="eye"):
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(r.node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(r.node, f"/{CAM}/color/image_raw", Image), "bgr8")
    info = grab(r.node, f"/{CAM}/color/camera_info", CameraInfo)
    cv2.imwrite(f"{tag}.png", color)
    P = cloud(r, depth, info)
    return color, depth, info, P


def segments(P, depth, color, zlo, zhi, min_area=40):
    valid = np.isfinite(depth) & (depth > 0)
    Z = P[..., 2]
    mask = valid & (Z > zlo) & (Z < zhi)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    out = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area:
            continue
        m = lab == i
        pts = P[m]
        col = color[m].mean(0)[::-1]
        u, v = cents[i]
        out.append(dict(px=(u, v), area=int(area), c=pts.mean(0), zmax=pts[:, 2].max(),
                        zmin=pts[:, 2].min(), xr=(pts[:, 0].min(), pts[:, 0].max()),
                        yr=(pts[:, 1].min(), pts[:, 1].max()), rgb=col.astype(int), mask=m))
    return out


if __name__ == "__main__":
    zlo, zhi = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) >= 3 else (0.5, 0.7)
    tag = sys.argv[3] if len(sys.argv) > 3 else "eye"
    r = Robot("eyescan")
    color, depth, info, P = snap(r, tag)
    T, Rc = cam_pose(r)
    print("cam at", T.round(4))
    v = np.isfinite(depth) & (depth > 0)
    print("z range in view", P[..., 2][v].min().round(3), P[..., 2][v].max().round(3))
    for s in segments(P, depth, color, zlo, zhi):
        print(f"px=({s['px'][0]:.0f},{s['px'][1]:.0f}) area={s['area']} c={s['c'].round(3)} "
              f"z=({s['zmin']:.3f},{s['zmax']:.3f}) xr=({s['xr'][0]:.3f},{s['xr'][1]:.3f}) "
              f"yr=({s['yr'][0]:.3f},{s['yr'][1]:.3f}) rgb={s['rgb']}")
    # table height estimate: most common z
    zs = P[..., 2][v]
    hist, edges = np.histogram(zs, bins=200)
    print("mode z", edges[hist.argmax()].round(3))
OPENRUA_EOF

# openrua op 22
timeout 600 python3 -u -c "
from robot import *
r = Robot()
q = r.ik([-0.105, -0.156, 0.80], down_quat(0))
print('ik', np.round(q,3))
r.move_q(q, 4.0)
print('tcp', r.tcp()[0].round(4), r.tcp()[1].round(3))
" && timeout 120 python3 eyescan.py 0.5 0.75 eye_white

# openrua op 23
timeout 300 python3 -u -c "
from robot import *
r = Robot()
seed = r.arm_q()
for yaw in [0, 45, 90, -45]:
    q = r.ik([-0.105, -0.156, 0.80], down_quat(yaw), seed=seed)
    p, qt = r.tcp(q)
    print(yaw, 'want', down_quat(yaw).round(3), 'got', qt.round(3), 'q7', round(q[6],3), 'pos', p.round(4))
"

# openrua op 24
python3 - <<'EOF'
s = open('robot.py').read()
s = s.replace('''        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        if seed is None:''', '''        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        # the IK tip link is panda_link8, which is panda_hand rotated +45deg
        # about z (hand = link8 * Rz(-45)); convert the hand quat to link8
        quat = quat_mul(quat, np.array([0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)]))
        if seed is None:''')
open('robot.py','w').write(s)
EOF
timeout 300 python3 -u -c "
from robot import *
r = Robot()
seed = r.arm_q()
for yaw in [0, 90]:
    q = r.ik([-0.105, -0.156, 0.80], down_quat(yaw), seed=seed)
    p, qt = r.tcp(q)
    print(yaw, 'want', down_quat(yaw).round(3), 'got', qt.round(3), 'q7', round(q[6],3), 'pos', p.round(4))
"

# openrua op 25
timeout 600 python3 -u -c "
from robot import *
from eyescan import *
r = Robot()
q = r.ik([-0.107, -0.150, 0.80], down_quat(0))
r.move_q(q, 3.0)
print('tcp', r.tcp()[0].round(4), r.tcp()[1].round(3))
color, depth, info, P = snap(r, 'eye_white2')
np.save('P_white.npy', P); np.save('depth_white.npy', depth)
segs = segments(P, depth, color, 0.44, 0.75)
for s in segs:
    print(f\"px=({s['px'][0]:.0f},{s['px'][1]:.0f}) area={s['area']} c={s['c'].round(3)} z=({s['zmin']:.3f},{s['zmax']:.3f}) xr=({s['xr'][0]:.3f},{s['xr'][1]:.3f}) yr=({s['yr'][0]:.3f},{s['yr'][1]:.3f}) rgb={s['rgb']}\")
# white mug analysis: component nearest to (-0.107,-0.15)
s = min(segs, key=lambda s: np.hypot(s['c'][0]+0.107, s['c'][1]+0.15))
pts = P[s['mask']]
# rim: top 1cm
top = pts[pts[:,2] > s['zmax']-0.008]
cx, cy = top[:,0].mean(), top[:,1].mean()
print('rim center', round(cx,4), round(cy,4), 'rim n', len(top))
for zlo in np.arange(0.44, s['zmax'], 0.01):
    band = pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(band)<5: continue
    rr = np.hypot(band[:,0]-cx, band[:,1]-cy)
    print(f'z {zlo:.2f}-{zlo+0.01:.2f}: n={len(band)} r_med={np.median(rr):.3f} r_max={rr.max():.3f} r_min={rr.min():.3f}')
"

# openrua op 26
mkdir -p "$(dirname /workspace/measure.py)"
cat > /workspace/measure.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Hover 0.25 m above a guessed mug position and measure its rim centre/radius.

Usage: python3 measure.py <x> <y> [tag]
"""
import sys

import numpy as np

from eyescan import segments, snap
from robot import Robot, down_quat


def measure(r, gx, gy, tag="mug", hover_z=0.80):
    q = r.ik([gx, gy, hover_z], down_quat(0))
    r.move_q(q, 3.0)
    print("tcp", r.tcp()[0].round(4))
    color, depth, info, P = snap(r, tag)
    segs = segments(P, depth, color, 0.46, 0.75)
    for s in segs:
        print(f"  seg px=({s['px'][0]:.0f},{s['px'][1]:.0f}) area={s['area']} c={s['c'].round(3)} "
              f"z=({s['zmin']:.3f},{s['zmax']:.3f}) xr=({s['xr'][0]:.3f},{s['xr'][1]:.3f}) "
              f"yr=({s['yr'][0]:.3f},{s['yr'][1]:.3f}) rgb={s['rgb']}")
    s = min(segs, key=lambda s: np.hypot(s["c"][0] - gx, s["c"][1] - gy))
    pts = P[s["mask"]]
    zmax = s["zmax"]
    top = pts[pts[:, 2] > zmax - 0.008]
    cx, cy = top[:, 0].mean(), top[:, 1].mean()
    rr = np.hypot(top[:, 0] - cx, top[:, 1] - cy)
    # handle direction: points beyond r=0.055
    far = pts[np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) > 0.055]
    hdir = (far[:, :2].mean(0) - [cx, cy]) if len(far) else np.zeros(2)
    print(f"rim centre ({cx:.4f},{cy:.4f}) z_top={zmax:.3f} r_out~{np.percentile(rr, 95):.3f} "
          f"r_in~{np.percentile(rr, 5):.3f} handle_dir={hdir.round(3)} n_far={len(far)}")
    return np.array([cx, cy]), zmax, hdir


if __name__ == "__main__":
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    tag = sys.argv[3] if len(sys.argv) > 3 else "mug"
    measure(Robot("measure"), gx, gy, tag)
OPENRUA_EOF

# openrua op 27
timeout 600 python3 -u measure.py -0.073 0.138 eye_yellow

# openrua op 28
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-pinch pick of a mug and place onto a plate.

Usage: python3 pick_place.py <mug_cx> <mug_cy> <rim_z> <wall_r> <side:+y|-y|+x|-x> <plate_x> <plate_y> [tag]
The pinch point is the mug wall on the given side of the rim centre; one
finger goes inside the mug, one outside.
"""
import sys

import numpy as np

from robot import Robot, down_quat

TABLE_Z = 0.423
PLATE_TOP = 0.455
CARRY_Z = 0.78
GRASP_DEPTH = 0.035  # fingertips this far below the rim


def main():
    cx, cy, rim_z, wall_r = map(float, sys.argv[1:5])
    side = sys.argv[5]
    px, py = float(sys.argv[6]), float(sys.argv[7])
    r = Robot("pick_place")
    off = {"+y": (0, wall_r), "-y": (0, -wall_r), "+x": (wall_r, 0), "-x": (-wall_r, 0)}[side]
    yaw = 0.0 if side in ("+y", "-y") else 90.0
    qd = down_quat(yaw)
    gx, gy = cx + off[0], cy + off[1]
    grasp_z = rim_z - GRASP_DEPTH
    mug_bottom_below_tcp = grasp_z - TABLE_Z

    print("== open gripper")
    r.open()
    print("== hover above pinch point")
    r.move_tcp([gx, gy, rim_z + 0.12], qd, 3.0)
    print("== descend")
    r.move_tcp([gx, gy, grasp_z], qd, 2.5)
    p, _ = r.tcp()
    if np.linalg.norm(p[:2] - [gx, gy]) > 0.006 or abs(p[2] - grasp_z) > 0.006:
        print("!! descend inaccurate", p)
    f0, _ = r.wrench()
    print("== close")
    gap = r.close()
    f1, _ = r.wrench()
    print(f"  gap after close {gap:.4f} (closed_m=0 -> >0 means holding); wrench dF={np.round(f1 - f0, 2)}")
    if gap < 0.003:
        print("!! nothing grasped")
        sys.exit(2)
    print("== lift")
    r.move_tcp([gx, gy, CARRY_Z], qd, 3.0)
    gap2 = r.finger_gap()
    print(f"  gap after lift {gap2:.4f}")
    if gap2 < 0.003:
        print("!! lost the mug on lift")
        sys.exit(3)
    print("== carry above plate")
    tx, ty = px + off[0], py + off[1]
    r.move_tcp([tx, ty, CARRY_Z], qd, 4.0)
    print("== lower onto plate")
    rel_z = PLATE_TOP + mug_bottom_below_tcp + 0.008
    r.move_tcp([tx, ty, rel_z + 0.05], qd, 2.5)
    r.move_tcp([tx, ty, rel_z], qd, 2.0)
    print("== release")
    r.open()
    print("== retreat")
    r.move_tcp([tx, ty, rel_z + 0.15], qd, 2.5)
    print("done")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 29
timeout 1500 python3 -u pick_place.py -0.0746 0.1319 0.542 0.047 +y -0.015 -0.317 2>&1 | tee pp_yellow.log

# openrua op 30
timeout 120 python3 tools/perception/cam_snap.py agentview agent2.png && timeout 120 python3 tools/perception/cam_snap.py birdview bird2.png

# openrua op 31
timeout 600 python3 -u measure.py -0.1025 -0.1514 eye_white3 2>&1 | tail -3

# openrua op 32
timeout 1500 python3 -u pick_place.py -0.1034 -0.1513 0.549 0.045 -y -0.010 0.322 2>&1 | tee pp_white.log

# openrua op 33
timeout 120 python3 tools/perception/cam_snap.py agentview agent3.png && timeout 120 python3 tools/perception/cam_snap.py frontview front3.png && timeout 200 python3 scan2.py birdview 2>&1 | grep -v "low comp"

# openrua op 34
timeout 600 python3 -u -c "
from robot import *
r = Robot()
r.move_q([0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483], 4.0)
print('gap', r.finger_gap())
" && timeout 200 python3 scan2.py birdview 2>&1 | grep -v "low comp" && timeout 120 python3 tools/perception/cam_snap.py birdview bird4.png
