#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/README.md && echo ----- && cat tools/perception/cam_snap.py && echo ----- && cat tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py && echo ----- && cat tools/action/gripper_cmd.py && echo ----- && cat tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -iv parameter && echo ---- && ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 265 310; echo "butter ^"; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 565 225; echo "cream cheese ^"; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 60 230; echo "basket ^"; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 320 240; echo "table center ^"

# openrua op 6
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; timeout 30 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | head -12

# openrua op 7
for p in "383 287" "375 280" "395 295" "300 300" "320 260"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"

# openrua op 8
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table from a camera's depth frame and print
their world-frame bounding boxes. Usage: python3 scan.py <camera> [zmin]
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


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
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.435
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
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    valid = np.isfinite(depth) & (depth > 0)
    mask = valid & (P[..., 2] > zmin) & (P[..., 2] < 0.75)
    lab, n = ndimage.label(mask)
    print(f"camera at {T}, {n} blobs")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 40:
            continue
        pts = P[m]
        col = color[m].mean(0)[::-1].astype(int)
        vv, uu = np.nonzero(m)
        print(f"blob {i}: px {m.sum()} u[{uu.min()},{uu.max()}] v[{vv.min()},{vv.max()}] "
              f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] centre ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) rgb {col}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
timeout 120 python3 scan.py robot0_eye_in_hand 0.43

# openrua op 10
timeout 120 python3 tools/action/gripper_cmd.py 0.04 && timeout 300 python3 tools/action/ik_move.py 0.413 0.057 0.13 1 0 0 0 3 --at tcp && ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 11
mkdir -p "$(dirname /workspace/move.py)"
cat > /workspace/move.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Move the fingertip point (TCP) to world-frame targets, straight-down grasp
orientation. IK (base frame) -> FollowJointTrajectory per waypoint.

Usage: python3 move.py x,y,z[,yaw_deg][,seconds] [x,y,z[,yaw][,sec] ...]
       python3 move.py grip <per_finger_m>
       python3 move.py js
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
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = TRAJ["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])  # from tf2_echo world panda_link0


class Mover:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("mover")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def solve(self, x, y, z, yaw_deg=0.0):
        """TCP world target -> arm joint dict. Hand points straight down,
        fingers along world y when yaw=0 (yaw rotates about world z)."""
        # hand z = -world z; hand origin sits TCP above the fingertip point
        hand_w = np.array([x, y, z + TCP]) - BASE_IN_WORLD
        # quaternion: Rz(yaw) * Rx(pi)  -> (cos(y/2), sin(y/2), 0, 0)*... compute
        half = np.deg2rad(yaw_deg) / 2
        # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin,cos); product q = Rz*Rx:
        qx, qy, qz, qw = np.cos(half), np.sin(half), 0.0, 0.0
        cur = self.joints()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_w)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        seed = JointState()
        for j in JOINTS:
            seed.name.append(j)
            seed.position.append(cur[j])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK failed for {x,y,z}: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def execute(self, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - p) for j, p in zip(JOINTS, positions))
        print(f"  traj error_code={code} max joint err={err:.4f}", flush=True)
        return code

    def goto(self, x, y, z, yaw=0.0, seconds=3.0):
        print(f"goto TCP world ({x:.3f},{y:.3f},{z:.3f}) yaw {yaw}", flush=True)
        q = self.solve(x, y, z, yaw)
        print("  ik:", np.round(q, 3).tolist(), flush=True)
        return self.execute(q, seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}", flush=True)


def main():
    mv = Mover()
    args = sys.argv[1:]
    if args and args[0] == "grip":
        mv.gripper(float(args[1]))
    elif args and args[0] == "js":
        print(mv.joints())
    else:
        for a in args:
            v = [float(s) for s in a.split(",")]
            x, y, z = v[:3]
            yaw = v[3] if len(v) > 3 else 0.0
            sec = v[4] if len(v) > 4 else 3.0
            mv.goto(x, y, z, yaw, sec)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 12
timeout 600 python3 -u move.py -0.097,0.057,0.55

# openrua op 13
timeout 300 python3 -u -c "
import move, numpy as np
mv = move.Mover()
# current hand pose in world: (-0.053,0,0.778) -> TCP z = 0.778-0.1034
for tgt in [(-0.053,0.0,0.6746), (-0.097,0.057,0.60), (-0.097,0.057,0.55), (-0.05,0.0,0.55)]:
    try:
        q = mv.solve(*tgt); print(tgt, 'OK', np.round(q,3).tolist())
    except SystemExit as e:
        print(tgt, e)
"

# openrua op 14
timeout 120 python3 -u -c "
import rclpy, move
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
mv = move.Mover()
cli = mv.node.create_client(GetPositionFK, '/compute_fk'); cli.wait_for_service(10)
cur = mv.joints()
req = GetPositionFK.Request()
req.fk_link_names = ['panda_link8','panda_hand','panda_hand_tcp']
js = JointState()
for j in move.JOINTS: js.name.append(j); js.position.append(cur[j])
req.robot_state.joint_state = js
fut = cli.call_async(req); rclpy.spin_until_future_complete(mv.node, fut, timeout_sec=60)
r = fut.result()
print('err', r.error_code.val)
for n,p in zip(r.fk_link_names, r.pose_stamped):
    print(n, p.header.frame_id, p.pose)
"; ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -iE "group name|chain|tip" | head

# openrua op 15
timeout 300 python3 -u -c "
import rclpy, move, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
mv = move.Mover()
cur = mv.joints()
def ik(pos, quat, link):
    req = GetPositionIK.Request()
    req.ik_request.group_name='panda_arm'; req.ik_request.ik_link_name=link
    req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=pos
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat
    seed=JointState()
    for j in move.JOINTS: seed.name.append(j); seed.position.append(cur[j])
    req.ik_request.robot_state.joint_state=seed
    req.ik_request.timeout=Duration(sec=1)
    fut=mv.ik.call_async(req); rclpy.spin_until_future_complete(mv.node,fut,timeout_sec=60)
    r=fut.result()
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position)) if r.error_code.val==1 else {}
    print(link, pos, r.error_code.val, [round(sol.get(j,0),3) for j in move.JOINTS])
hq=(0.9996,0,-0.0284,0)
ik((-0.053,0,0.7776), hq, 'panda_hand')
ik((0.457,0,0.3576), hq, 'panda_hand')
ik((-0.053,0,0.7776), (1,0,0,0), 'panda_hand')
ik((0.457,0,0.3576), (1,0,0,0), 'panda_hand')
ik((-0.097,0.057,0.55+0.1034), (1,0,0,0), 'panda_hand')
ik((0.413,0.057,0.55+0.1034-0.42), (1,0,0,0), 'panda_hand')
"

# openrua op 16
cat > /workspace/iktest.py <<'EOF'
import rclpy, move, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
mv = move.Mover()
cur = mv.joints()
def ik(pos, quat, link):
    req = GetPositionIK.Request()
    req.ik_request.group_name='panda_arm'; req.ik_request.ik_link_name=link
    req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=map(float,pos)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
    seed=JointState()
    for j in move.JOINTS: seed.name.append(j); seed.position.append(cur[j])
    req.ik_request.robot_state.joint_state=seed
    req.ik_request.timeout=Duration(sec=1)
    fut=mv.ik.call_async(req); rclpy.spin_until_future_complete(mv.node,fut,timeout_sec=60)
    r=fut.result()
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position)) if r.error_code.val==1 else {}
    print(link, pos, r.error_code.val, [round(sol.get(j,0),3) for j in move.JOINTS], flush=True)
hq=(0.9996,0,-0.0284,0)
ik((-0.053,0,0.7776), hq, 'panda_hand')
ik((0.457,0,0.3576), hq, 'panda_hand')
ik((-0.053,0,0.7776), (1,0,0,0), 'panda_hand')
ik((0.457,0,0.3576), (1,0,0,0), 'panda_hand')
ik((-0.097,0.057,0.55+0.1034), (1,0,0,0), 'panda_hand')
ik((0.413,0.057,0.55+0.1034-0.42), (1,0,0,0), 'panda_hand')
EOF
timeout 300 python3 -u iktest.py

# openrua op 17
python3 - <<'EOF'
s = open('move.py').read()
s = s.replace('BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])  # from tf2_echo world panda_link0',
              '# machine fact (verified via FK/IK probe): IK with empty frame_id takes WORLD coords\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])')
s = s.replace('req.ik_request.group_name = M["planning"]["group"]',
              'req.ik_request.group_name = M["planning"]["group"]\n        req.ik_request.ik_link_name = "panda_hand"')
open('move.py','w').write(s)
EOF
timeout 600 python3 -u move.py -0.097,0.057,0.55

# openrua op 18
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "At time" | head -3 && timeout 120 python3 scan.py robot0_eye_in_hand 0.43

# openrua op 19
timeout 600 python3 -u move.py -0.098,0.058,0.48,0,2 -0.098,0.058,0.44,0,2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 20
timeout 300 python3 -u move.py grip 0.0 && ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 21
timeout 600 python3 -u move.py -0.098,0.058,0.65,0,2 0.0,0.27,0.70,0,3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 22
timeout 600 python3 -u move.py 0.0,0.27,0.64,0,2 && timeout 300 python3 -u move.py grip 0.04 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 23
timeout 600 python3 -u move.py 0.0,0.27,0.70,0,2 -0.006,-0.254,0.58,0,4 && timeout 120 python3 scan.py robot0_eye_in_hand 0.43 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
timeout 600 python3 -u move.py -0.006,-0.254,0.58,0,3 && python3 - <<'EOF'
s=open('scan.py').read()
s=s.replace('        print(f"blob {i}:', '        xy = pts[:, :2] - pts[:, :2].mean(0); ev, evec = np.linalg.eigh(xy.T @ xy); ax = evec[:, 1]; yaw = np.degrees(np.arctan2(ax[1], ax[0]))\n        print(f"blob {i}: yaw {yaw:.1f}deg')
open('scan.py','w').write(s)
EOF
timeout 120 python3 scan.py robot0_eye_in_hand 0.43

# openrua op 25
timeout 600 python3 -u move.py -0.006,-0.259,0.48,0,2 -0.006,-0.259,0.44,0,2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 26
timeout 300 python3 -u move.py grip 0.0 && timeout 600 python3 -u move.py -0.006,-0.259,0.65,0,3 && ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 27
timeout 600 python3 -u move.py 0.0,0.27,0.70,0,4 && timeout 600 python3 -u move.py 0.02,0.29,0.66,0,2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 28
timeout 300 python3 -u move.py grip 0.04 && timeout 600 python3 -u move.py 0.0,0.27,0.75,0,2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 scan.py robot0_eye_in_hand 0.47
