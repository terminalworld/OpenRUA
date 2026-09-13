#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
find /workspace/tools /workspace/.ros -type f; echo ---; ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 2
mkdir -p snaps && for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 3
for p in "283 237" "296 293" "268 311" "385 290" "320 280" "200 200"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done; ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 4
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Cluster above-table points from a camera's depth into object blobs.

Usage: python3 scan.py <camera> [zmin=0.435] [zmax=0.9]
Prints, per blob: pixel centroid, world centroid, world bbox, size.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2


def grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.435
    zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.9
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    z = depth
    ok = np.isfinite(z) & (z > 0)
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    P = np.stack([X, Y, z], -1) @ R.T + T
    mask = ok & (P[..., 2] > zmin) & (P[..., 2] < zmax)
    m8 = mask.astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(m8, 8)
    print(f"camera {cam} at world {T.round(3)}; table z approx {np.median(P[ok][:,2]):.3f}")
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        sel = lab == i
        pts = P[sel]
        col = color[sel].mean(0)[::-1].astype(int)
        lo, hi = pts.min(0), pts.max(0)
        print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) n={sel.sum()} "
              f"rgb={col} centre={pts.mean(0).round(3)} "
              f"xmin/max=({lo[0]:.3f},{hi[0]:.3f}) ymin/max=({lo[1]:.3f},{hi[1]:.3f}) "
              f"z=({lo[2]:.3f},{hi[2]:.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 5
timeout 120 python3 scan.py birdview 0.435 0.75; echo ----; timeout 120 python3 scan.py agentview 0.435 0.75

# openrua op 6
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helper: FK/IK, trajectory, gripper, joint-state reads.

All poses in WORLD frame; converted to the planner's base frame using the
static world->panda_link0 transform read from TF (machine.yaml planning
facts: IK wants frame_id empty = base frame).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
PLAN = M["planning"]
TCP_OFF = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])  # tf2_echo world panda_link0


def down_quat(yaw_deg=0.0):
    """Hand Z pointing down (world -Z); fingers open along world X when
    yaw=0 (Rz(90)*Rx(180)); yaw rotates the finger axis about world Z."""
    r = Rot.from_euler("z", 90 + yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    return r.as_quat()  # x y z w


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.ik = self.node.create_client(GetPositionIK, PLAN["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.joints()

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            end = time.time() + 15
            while self._js is None and time.time() < end:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def fk_world(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_world(self, pos_world, quat, seed=None, tcp=True, tries=3):
        """IK for a world-frame pose. If tcp, pos is the fingertip point
        (offset TCP_OFF along hand +Z)."""
        pos = np.array(pos_world, float)
        if tcp:
            R = Rot.from_quat(quat).as_matrix()
            pos = pos - TCP_OFF * R[:, 2]
        pb = pos - BASE_IN_WORLD
        last = None
        for i in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = PLAN["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.timeout = Duration(sec=2)
            req.ik_request.avoid_collisions = False
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            last = None if res is None else res.error_code.val
            # perturb the seed for the retry
            seed = list(np.array(self.arm_q() if seed is None else seed) + np.random.uniform(-0.3, 0.3, 7))
        raise RuntimeError(f"IK failed (code {last}) for world {pos_world}")

    def move_q(self, q, seconds=3.0, via=None):
        """One trajectory goal to joint config q (optionally through via
        points as list of (q, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = None if r is None else r.result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, seed=None, tcp=True):
        q = self.ik_world(pos_world, quat, seed=seed, tcp=tcp)
        code, err = self.move_q(q, seconds)
        p, _ = self.fk_world(link="panda_hand")
        R = Rot.from_quat(quat).as_matrix()
        tcp_p = p + TCP_OFF * R[:, 2]
        print(f"  TCP now at world {tcp_p.round(4)} (target {np.round(pos_world,4)})", flush=True)
        return q

    def gripper(self, width, effort=None):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(effort if effort is not None else GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        g = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(g,4)}", flush=True)
        return g
OPENRUA_EOF

# openrua op 7
timeout 120 python3 -c "
from arm import *
a = Arm()
print('joints', a.arm_q())
p,q = a.fk_world()
print('hand world', p.round(4), 'quat', q.round(4))
R = Rot.from_quat(q).as_matrix()
print('hand Z axis (approach) in world', R[:,2].round(3), 'hand Y (finger axis)', R[:,1].round(3))
print('TCP world', (p + TCP_OFF*R[:,2]).round(4))
print('down_quat(0)', down_quat(0).round(4), Rot.from_quat(down_quat(0)).as_matrix().round(2))
print('fingers', a.finger_gap())
print('wrench', a.wrench())
"

# openrua op 8
timeout 60 python3 -c "
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
dh=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
M=np.eye(4)
for (a,d,al),th in zip(dh,q): M=M@T(a,d,al,th)
M=M@T(0,0.107,0,0)  # flange
print('flange in base', M[:3,3].round(4))
print(M[:3,:3].round(3))
"


# openrua op 9
timeout 200 python3 -c "
from arm import *
a = Arm()
q0 = np.array(a.arm_q())
# FK raw (as returned by service) 
req = GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand','panda_link0']; req.robot_state.joint_state=a._seed()
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30); res=fut.result()
for n,ps in zip(res.fk_link_names,res.pose_stamped): print(n, ps.header.frame_id, [round(v,4) for v in (ps.pose.position.x,ps.pose.position.y,ps.pose.position.z)])
hand_raw = res.pose_stamped[0].pose
quat=[hand_raw.orientation.x,hand_raw.orientation.y,hand_raw.orientation.z,hand_raw.orientation.w]
pos_raw=np.array([hand_raw.position.x,hand_raw.position.y,hand_raw.position.z])
# IK test A: send raw FK pose back (what frame does IK assume?)
for label,p in [('raw',pos_raw),('raw+base',pos_raw+BASE_IN_WORLD),('raw-base',pos_raw-BASE_IN_WORLD)]:
    try:
        BASE_SAVE = BASE_IN_WORLD.copy()
        import arm as A; A.BASE_IN_WORLD = np.zeros(3)
        q = a.ik_world(p, quat, tcp=False, tries=1)
        A.BASE_IN_WORLD = BASE_SAVE
        print(label, 'IK ok, joint diff from current', np.round(np.array(q)-q0,3))
    except Exception as e:
        print(label, 'IK fail', e)
"

# openrua op 10
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])  # tf2_echo world panda_link0", "new_string": "# Verified on this machine: compute_fk returns panda_link0 at world\\n# (-0.51, 0, 0.42) and IK accepts the raw FK pose back -> the planner\'s\\n# model frame IS world here, so no base offset is applied.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 11
cat >> scan.py <<'EOF'


def profile(cam, cx_, cy_, r=0.06, zmin=0.43):
    """Print width-per-height of points near world (cx_, cy_)."""
    rclpy.init()
    node = rclpy.create_node("scan2")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    ok = np.isfinite(depth) & (depth > 0)
    P = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1) @ R.T + T
    sel = ok & (np.hypot(P[..., 0] - cx_, P[..., 1] - cy_) < r) & (P[..., 2] > zmin)
    pts = P[sel]
    for z0 in np.arange(zmin, pts[:, 2].max() + 0.01, 0.01):
        s = pts[(pts[:, 2] >= z0) & (pts[:, 2] < z0 + 0.01)]
        if len(s):
            print(f"z {z0:.2f}-{z0+0.01:.2f}: n={len(s):4d} x({s[:,0].min():.3f},{s[:,0].max():.3f}) "
                  f"y({s[:,1].min():.3f},{s[:,1].max():.3f}) cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
    rclpy.shutdown()
EOF
timeout 120 python3 -c "
import sys; sys.argv=['x']; from scan import profile; profile('agentview', -0.20, -0.16)"; echo ---; timeout 120 python3 -c "
import sys; sys.argv=['x']; from scan import profile; profile('sideview', -0.20, -0.16)"

# openrua op 12
timeout 60 ros2 param get /robot_state_publisher robot_description 2>/dev/null | grep -n -A6 -E 'joint name="panda_finger_joint1"|joint name="panda_hand_joint"|joint name="panda_hand_tcp_joint"|link name="panda_leftfinger"' | grep -E 'joint name|origin|link name|geometry|box|mesh|cylinder' | head -30

# openrua op 13
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick one object top-down and drop it into the basket.

Usage: python3 -u pick_place.py <x> <y> <grasp_tcp_z> [yaw_deg=0]
"""
import sys
import numpy as np
from arm import Arm, down_quat, GRIP

BASKET = np.array([0.0, 0.26])
HOVER_Z = 0.62         # pre-grasp / transit TCP height (above bottle cap 0.556)
BASKET_Z = 0.72        # TCP height over the basket when releasing (rim 0.627)


def main():
    x, y, gz = map(float, sys.argv[1:4])
    yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    q = down_quat(yaw)
    a = Arm()
    print("start joints", np.round(a.arm_q(), 3), flush=True)

    print("open gripper", flush=True)
    a.gripper(GRIP["open_m"])

    print(f"pre-grasp above ({x},{y}) at z={HOVER_Z}", flush=True)
    q_hover = a.move_pose([x, y, HOVER_Z], q, seconds=4.0)

    print(f"descend to grasp z={gz}", flush=True)
    mid = (HOVER_Z + gz) / 2
    q_mid = a.ik_world([x, y, mid], q, seed=q_hover)
    q_grasp = a.ik_world([x, y, gz], q, seed=q_mid)
    a.move_q(q_grasp, seconds=4.0, via=[(q_mid, 2.0)])
    p, _ = a.fk_world()
    print("  hand at", p.round(4), "wrench", a.wrench(), flush=True)

    print("close gripper", flush=True)
    g = a.gripper(GRIP["closed_m"])
    gap = abs(g[0]) + abs(g[1])
    print(f"  finger gap = {gap:.4f} m", flush=True)
    if gap < 0.005:
        print("GRASP FAILED: fingers closed on air", flush=True)
        a.gripper(GRIP["open_m"])
        a.move_q(q_hover, seconds=3.0)
        sys.exit(2)

    print("lift", flush=True)
    a.move_q(q_hover, seconds=3.0)
    g = a.finger_gap()
    print("  fingers after lift", np.round(g, 4), flush=True)

    print("transit to basket", flush=True)
    q_b_hi = a.ik_world([BASKET[0], BASKET[1], BASKET_Z + 0.05], q, seed=q_hover)
    q_b = a.ik_world([BASKET[0], BASKET[1], BASKET_Z], q, seed=q_b_hi)
    a.move_q(q_b, seconds=5.0, via=[(q_b_hi, 4.0)])
    p, _ = a.fk_world()
    print("  hand at", p.round(4), "fingers", np.round(a.finger_gap(), 4), flush=True)

    print("release", flush=True)
    a.gripper(GRIP["open_m"])

    print("retreat up", flush=True)
    a.move_q(q_b_hi, seconds=2.0)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 14
nohup python3 -u pick_place.py -0.194 -0.155 0.525 > logs_ketchup.txt 2>&1 &
sleep 45; cat /workspace/logs_ketchup.txt

# openrua op 15
sleep 30; cat /workspace/logs_ketchup.txt; 

# openrua op 16
for c in agentview robot0_eye_in_hand sideview; do python3 tools/perception/cam_snap.py $c snaps/${c}_1.png; done

# openrua op 17
timeout 15 ros2 topic hz /joint_states 2>&1 | head -3; timeout 10 ros2 topic echo /clock --once; sleep 5; timeout 10 ros2 topic echo /clock --once

# openrua op 18
timeout 60 python3 -c "
from arm import *
a=Arm()
import time
for i in range(4):
    print(np.round(a.arm_q(),4), np.round(a.finger_gap(),4)); time.sleep(0.6)
p,q=a.fk_world(); print('hand', p.round(4))
"

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        rf = gh.get_result_async()\\n        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)\\n        r = rf.result()\\n        code = None if r is None else r.result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\", flush=True)\\n        return code, err", "new_string": "        rf = gh.get_result_async()\\n        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)\\n        r = rf.result()\\n        code = None if r is None else r.result.error_code\\n        # the result can come back (often -5) while the sim is still\\n        # executing: wait until the joints stop changing\\n        err = self.settle(q)\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\", flush=True)\\n        return code, err\\n\\n    def settle(self, q_target=None, timeout=240.0, tol=0.01):\\n        \\"\\"\\"Poll /joint_states (2 Hz wall) until arm joints are stationary\\n        for 3 consecutive reads (or within tol of q_target).\\"\\"\\"\\n        prev, same = None, 0\\n        end = time.time() + timeout\\n        err = float(\\"nan\\")\\n        while time.time() < end:\\n            cur = np.array(self.arm_q())\\n            if q_target is not None:\\n                err = np.abs(cur - np.array(q_target)).max()\\n                if err < tol:\\n                    same += 1\\n                    if same >= 2:\\n                        return err\\n            if prev is not None and np.abs(cur - prev).max() < 1e-5:\\n                same += 1\\n                if same >= 3:\\n                    return err\\n            elif prev is not None:\\n                same = 0\\n            prev = cur\\n            time.sleep(0.3)\\n        print(\\"  settle: timeout\\", flush=True)\\n        return err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        r = rf.result().result\\n        g = self.finger_gap()", "new_string": "        r = rf.result().result\\n        # fingers may still be moving when the result returns\\n        prev, same, end = None, 0, time.time() + 120\\n        while time.time() < end and same < 3:\\n            g = np.array(self.finger_gap())\\n            same = same + 1 if prev is not None and np.abs(g - prev).max() < 1e-5 else 0\\n            prev = g\\n            time.sleep(0.3)\\n        g = self.finger_gap()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
timeout 120 python3 scan.py birdview 0.435 0.75; echo ---; timeout 120 python3 -c "
import sys; sys.argv=['x']; from scan import profile; profile('birdview', 0.06, 0.06, r=0.10)"

# openrua op 22
for c in frontview birdview; do python3 tools/perception/cam_snap.py $c snaps/${c}_1.png; done

# openrua op 23
timeout 300 python3 -u -c "
from arm import *
a=Arm()
q=down_quat(0)
a.move_pose([-0.3,-0.3,0.70], q, seconds=4.0)
print('fingers', a.finger_gap())
" 2>&1 | tail -5

# openrua op 24
cat >> scan.py <<'EOF'


def blob_pca(cam, cx_, cy_, r=0.10, zmin=0.435, zmax=0.75):
    """Principal axis of the above-table points near world (cx_, cy_)."""
    rclpy.init()
    node = rclpy.create_node("scan3")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    ok = np.isfinite(depth) & (depth > 0)
    P = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1) @ R.T + T
    sel = ok & (np.hypot(P[..., 0] - cx_, P[..., 1] - cy_) < r) & (P[..., 2] > zmin) & (P[..., 2] < zmax)
    pts = P[sel]
    c = pts.mean(0)
    xy = pts[:, :2] - c[:2]
    w, v = np.linalg.eigh(xy.T @ xy / len(xy))
    axis = v[:, 1]
    along = xy @ axis
    across = xy @ v[:, 0]
    print(f"n={len(pts)} centre={c.round(4)} ztop={pts[:,2].max():.3f}")
    print(f"axis={axis.round(3)} yaw_deg={np.degrees(np.arctan2(axis[1], axis[0])):.1f} "
          f"length={along.max()-along.min():.3f} width={across.max()-across.min():.3f}")
    # centre of the widest part (body): use points in the lower 60% of along-range? print histogram
    for lo in np.arange(along.min(), along.max(), 0.01):
        s = across[(along >= lo) & (along < lo + 0.01)]
        if len(s):
            print(f"  along {lo:+.3f}: n={len(s):3d} width={s.max()-s.min():.3f}")
    rclpy.shutdown()
EOF
timeout 120 python3 scan.py birdview 0.435 0.75; echo ---; timeout 120 python3 -c "
import sys; sys.argv=['x']; from scan import blob_pca; blob_pca('birdview', 0.03, 0.10, r=0.09)"

# openrua op 25
timeout 300 python3 -u -c "
from arm import *
a=Arm()
for yaw,pos in [(37.4,[0.064,0.052,0.62]),(37.4,[0.064,0.052,0.445]),(37.4,[0.062,0.318,0.80]),(37.4,[0.062,0.318,0.76]),(37.4,[0.062,0.318,0.72]),(90,[-0.169,0.06,0.445]),(90,[0.062,0.318,0.76])]:
    try:
        q=a.ik_world(pos, down_quat(yaw), tries=2); print(yaw,pos,'OK', np.round(q,2))
    except Exception as e: print(yaw,pos,'FAIL',e)
"

# openrua op 26
timeout 300 python3 -u -c "
from arm import *
a=Arm()
q0=np.array(a.arm_q()); print('current', q0.round(3))
p,quat=a.fk_world(); print('hand pose', p.round(4))
# IK for same pose with perturbed seed
sol=a.ik_world(p, quat, seed=list(q0+np.array([0.6,0.2,-0.6,0.1,0.5,0.1,-0.5])), tcp=False, tries=1)
print('alt IK sol', np.round(sol,3), 'diff', np.round(np.array(sol)-q0,3))
a.move_q(sol, seconds=3.0)
q1=np.array(a.arm_q()); print('after', q1.round(3), 'diff from sol', np.round(q1-np.array(sol),3))
p1,_=a.fk_world(); print('hand pose after', p1.round(4))
"

# openrua op 27
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q0=np.array(a.arm_q())
for j7 in [-2.8, -2.0, -1.0, 0.0, 1.0, 2.0, 2.8, -1.862]:
    q=q0.copy(); q[6]=j7
    a.move_q(q, seconds=2.0)
    print('target j7', j7, 'actual', round(a.arm_q()[6],3))
"

# openrua op 28
timeout 600 python3 -u -c "
from arm import *
a=Arm()
q0=np.array(a.arm_q()); print('j7 now', round(q0[6],3))
q=q0.copy(); q[6]=q0[6]-1.0
a.move_q(q, seconds=8.0)
print('target j7', round(q[6],3), 'actual', round(a.arm_q()[6],3))
q[6]=q0[6]-1.5
a.move_q(q, seconds=3.0)
print('target j7', round(q[6],3), 'actual', round(a.arm_q()[6],3))
"

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def move_q(self, q, seconds=3.0, via=None):\\n        \\"\\"\\"One trajectory goal to joint config q (optionally through via\\n        points as list of (q, t)).\\"\\"\\"\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    # measured on this machine: joint7 tracks at most ~0.2 rad/s (others\\n    # comfortably faster); a goal whose duration is too short for the j7\\n    # travel returns -5 and stops short.\\n    VMAX = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.18])\\n\\n    def move_q(self, q, seconds=3.0, via=None, retries=2):\\n        cur = np.array(self.arm_q())\\n        pts = ([np.array(v[0]) for v in via] if via else []) + [np.array(q)]\\n        need = 0.0\\n        prev = cur\\n        for p in pts:\\n            need += (np.abs(p - prev) / self.VMAX).max()\\n            prev = p\\n        total = max(seconds, need + 0.5)\\n        if via:\\n            scale = total / seconds\\n            via = [(vq, vt * scale) for vq, vt in via]\\n        seconds = total\\n        code, err = self._move_q(q, seconds, via)\\n        n = 0\\n        while err > 0.02 and n < retries:\\n            n += 1\\n            print(f\\"  resend ({n}) to converge\\", flush=True)\\n            code, err = self._move_q(q, max(2.0, seconds / 2), None)\\n        return code, err\\n\\n    def _move_q(self, q, seconds=3.0, via=None):\\n        \\"\\"\\"One trajectory goal to joint config q (optionally through via\\n        points as list of (q, t)).\\"\\"\\"\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def move_q(self, q, seconds=3.0, via=None, retries=2):", "new_string": "    LIMITS = np.array(FJT[\\"limits_rad\\"])\\n\\n    def ik_best(self, pos_world, yaw_deg, seed=None, tcp=True, n=4):\\n        \\"\\"\\"Top-down grasp IK: fingers are symmetric so yaw and yaw+180 are\\n        the same grasp; sample both and several seeds, return the solution\\n        closest (j7-weighted) to the seed/current config, away from limits.\\"\\"\\"\\n        base = np.array(self.arm_q() if seed is None else seed)\\n        best, best_cost = None, None\\n        for yaw in (yaw_deg, yaw_deg + 180.0, yaw_deg - 180.0):\\n            quat = down_quat(yaw)\\n            for i in range(n):\\n                s = base if i == 0 else base + np.random.uniform(-0.4, 0.4, 7)\\n                try:\\n                    q = np.array(self.ik_world(pos_world, quat, seed=list(s), tcp=tcp, tries=1))\\n                except RuntimeError:\\n                    continue\\n                margin = np.minimum(q - self.LIMITS[:, 0], self.LIMITS[:, 1] - q).min()\\n                if margin < 0.1:\\n                    continue\\n                cost = (np.abs(q - base) / self.VMAX).sum()\\n                if best is None or cost < best_cost:\\n                    best, best_cost = q, cost\\n        if best is None:\\n            raise RuntimeError(f\\"no acceptable IK for {pos_world} yaw {yaw_deg}\\")\\n        return list(best)\\n\\n    def move_q(self, q, seconds=3.0, via=None, retries=2):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 31
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick one object top-down and drop it into the basket.

Usage: python3 -u pick_place.py <x> <y> <grasp_tcp_z> <yaw_deg> <bx> <by> <release_tcp_z>
yaw_deg: world yaw of the finger-opening axis (0 = fingers along world X).
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from arm import Arm, GRIP, TCP_OFF

HOVER_Z = 0.62         # pre-grasp TCP height
TRANSIT_Z = 0.80       # carry height (hanging object must clear the rim)


def finger_axis_yaw(a):
    p, quat = a.fk_world()
    R = Rot.from_quat(quat).as_matrix()
    y = R[:, 1]  # hand Y = finger axis
    return np.degrees(np.arctan2(y[1], y[0])), p + TCP_OFF * R[:, 2]


def main():
    x, y, gz, yaw, bx, by, rz = map(float, sys.argv[1:8])
    a = Arm()
    print("start joints", np.round(a.arm_q(), 3), flush=True)

    print("open gripper", flush=True)
    a.gripper(GRIP["open_m"])

    print(f"pre-grasp above ({x},{y}) at z={HOVER_Z}, yaw {yaw}", flush=True)
    q_hover = a.ik_best([x, y, HOVER_Z], yaw)
    a.move_q(q_hover, seconds=4.0)
    fy, tcp = finger_axis_yaw(a)
    print(f"  TCP {tcp.round(4)} finger-axis yaw {fy:.1f} (want {yaw} mod 180)", flush=True)
    if abs(((fy - yaw) + 90) % 180 - 90) > 5:
        print("YAW OFF: aborting before descent", flush=True)
        sys.exit(3)

    print(f"descend to grasp z={gz}", flush=True)
    mid = (HOVER_Z + gz) / 2
    q_mid = a.ik_best([x, y, mid], yaw, seed=q_hover, n=2)
    q_grasp = a.ik_best([x, y, gz], yaw, seed=q_mid, n=2)
    a.move_q(q_grasp, seconds=4.0, via=[(q_mid, 2.0)])
    fy, tcp = finger_axis_yaw(a)
    print(f"  TCP {tcp.round(4)} finger-axis yaw {fy:.1f} wrench {a.wrench()}", flush=True)

    print("close gripper", flush=True)
    g = a.gripper(GRIP["closed_m"])
    gap = abs(g[0]) + abs(g[1])
    print(f"  finger gap = {gap:.4f} m", flush=True)
    if gap < 0.005:
        print("GRASP FAILED: fingers closed on air", flush=True)
        a.gripper(GRIP["open_m"])
        a.move_q(q_hover, seconds=3.0)
        sys.exit(2)

    print("lift", flush=True)
    a.move_q(q_hover, seconds=3.0)
    g = a.finger_gap()
    print("  fingers after lift", np.round(g, 4), flush=True)
    if abs(g[0]) + abs(g[1]) < 0.005:
        print("LOST OBJECT on lift", flush=True)
        sys.exit(2)

    print("transit to basket", flush=True)
    q_up = a.ik_best([x, y, TRANSIT_Z], yaw, seed=q_hover, n=2)
    q_b_hi = a.ik_best([bx, by, TRANSIT_Z], yaw, seed=q_up, n=2)
    q_b = a.ik_best([bx, by, rz], yaw, seed=q_b_hi, n=2)
    a.move_q(q_up, seconds=2.0)
    a.move_q(q_b_hi, seconds=4.0)
    print("  fingers over basket", np.round(a.finger_gap(), 4), flush=True)
    a.move_q(q_b, seconds=3.0)
    fy, tcp = finger_axis_yaw(a)
    print(f"  TCP {tcp.round(4)} fingers {np.round(a.finger_gap(), 4)}", flush=True)

    print("release", flush=True)
    a.gripper(GRIP["open_m"])

    print("retreat up", flush=True)
    a.move_q(q_b_hi, seconds=3.0)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 32
nohup python3 -u pick_place.py 0.0645 0.0529 0.445 37.4 0.062 0.318 0.74 > logs_ketchup2.txt 2>&1 &
sleep 60; cat /workspace/logs_ketchup2.txt

# openrua op 33
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "            req.ik_request.group_name = PLAN[\\"group\\"]\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "            req.ik_request.group_name = PLAN[\\"group\\"]\\n            # the group\'s default tip is panda_link8, which is yawed 45 deg\\n            # from panda_hand (finger axis = hand Y): target the hand itself\\n            req.ik_request.ik_link_name = \\"panda_hand\\"\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 34
nohup python3 -u pick_place.py 0.0645 0.0529 0.445 37.4 0.062 0.318 0.74 > logs_ketchup3.txt 2>&1 &
sleep 75; cat /workspace/logs_ketchup3.txt

# openrua op 35
sleep 60; cat /workspace/logs_ketchup3.txt | tail -8

# openrua op 36
for c in agentview robot0_eye_in_hand frontview; do python3 tools/perception/cam_snap.py $c snaps/${c}_2.png; done

# openrua op 37
timeout 120 python3 scan.py birdview 0.435 0.75

# openrua op 38
nohup python3 -u pick_place.py -0.169 0.06 0.445 90 0.062 0.318 0.70 > logs_box.txt 2>&1 &
sleep 90; cat /workspace/logs_box.txt

# openrua op 39
sleep 75; tail -10 /workspace/logs_box.txt

# openrua op 40
timeout 300 python3 -u -c "
from arm import *
a=Arm()
q=a.ik_best([-0.3,-0.3,0.70], 0)
a.move_q(q, seconds=4.0)
print('hand', a.fk_world()[0].round(3))
" 2>&1 | tail -3
for c in agentview birdview frontview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c snaps/${c}_3.png; done
timeout 120 python3 scan.py birdview 0.435 0.75
