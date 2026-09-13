#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; ros2 topic echo /joint_states --once

# openrua op 5
ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12 & sleep 4; kill %1 2>/dev/null; echo ---; timeout 60 python3 tools/perception/px2world.py agentview 340 265; echo "cream cheese:"; timeout 60 python3 tools/perception/px2world.py agentview 138 395; echo "basket center:"; timeout 60 python3 tools/perception/px2world.py agentview 530 320

# openrua op 6
cat /tmp/claude-1000/-workspace/0ad270d1-ed93-4a62-9a48-c79cc035e866/tasks/bgoalbo19.output

# openrua op 7
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort -u; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | sort -u; echo ---; timeout 10 ros2 topic echo /agentview/color/camera_info --once

# openrua op 8
mkdir -p "$(dirname /workspace/cam_capture.py)"
cat > /workspace/cam_capture.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Capture color, depth, intrinsics and world<-optical TF for a camera.

Usage: python3 cam_capture.py <camera>
Writes <camera>.png, <camera>_depth.npy, <camera>_meta.npz (K, T_world_cam).
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cam_capture")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    got = {}
    node.create_subscription(Image, f"/{cam}/color/image_raw",
                             lambda m: got.setdefault("color", m), 1)
    node.create_subscription(Image, f"/{cam}/depth/image_raw",
                             lambda m: got.setdefault("depth", m), 1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                             lambda m: got.setdefault("info", m), 1)
    frame = f"{cam}_optical_frame"
    t0 = time.time()
    while time.time() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(k in got for k in ("color", "depth", "info")) and \
                tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    else:
        raise SystemExit(f"timeout; have {list(got)}")
    br = CvBridge()
    color = br.imgmsg_to_cv2(got["color"], "bgr8")
    depth = br.imgmsg_to_cv2(got["depth"], "passthrough").astype(np.float32)
    K = np.array(got["info"].k).reshape(3, 3)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_depth.npy", depth)
    np.savez(f"{cam}_meta.npz", K=K, T=T)
    print(f"saved {cam}: depth {depth.shape} range "
          f"{np.nanmin(depth):.3f}..{np.nanmax(depth):.3f}; cam at {T[:3,3]}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
mkdir -p "$(dirname /workspace/px.py)"
cat > /workspace/px.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Offline pixel -> world using files from cam_capture.py.

Usage: python3 px.py <camera> u v [u v ...]
"""
import sys

import numpy as np

cam = sys.argv[1]
depth = np.load(f"{cam}_depth.npy")
m = np.load(f"{cam}_meta.npz")
K, T = m["K"], m["T"]
vals = list(map(int, sys.argv[2:]))
for u, v in zip(vals[::2], vals[1::2]):
    z = depth[v, u]
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    w = T @ p
    print(f"({u},{v}) depth={z:.3f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
OPENRUA_EOF

# openrua op 10
timeout 90 python3 cam_capture.py agentview & timeout 90 python3 cam_capture.py birdview & wait

# openrua op 11
echo "agentview: tomato-can-top, cream-cheese-top, basket-interior, table"; python3 px.py agentview 340 262 138 392 530 300 320 450 460 340; echo "birdview: tomato can?, blue can?, cream cheese?, basket center, table"; python3 px.py birdview 336 285 288 252 270 312 383 288 300 330

# openrua op 12
mkdir -p "$(dirname /workspace/segment.py)"
cat > /workspace/segment.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment above-table blobs from a captured camera and print world stats.

Usage: python3 segment.py <camera> [z_table=0.425] [min_h=0.012]
"""
import sys

import cv2
import numpy as np

cam = sys.argv[1]
z_table = float(sys.argv[2]) if len(sys.argv) > 2 else 0.425
min_h = float(sys.argv[3]) if len(sys.argv) > 3 else 0.012
depth = np.load(f"{cam}_depth.npy")
m = np.load(f"{cam}_meta.npz")
K, T = m["K"], m["T"]
color = cv2.imread(f"{cam}.png")
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
z = depth
X = (uu - K[0, 2]) * z / K[0, 0]
Y = (vv - K[1, 2]) * z / K[1, 1]
P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
wz = P[..., 2]
mask = (wz > z_table + min_h) & (wz < z_table + 0.35) & np.isfinite(z)
# restrict to table region (world radius) to drop the robot body & walls
wx, wy = P[..., 0], P[..., 1]
mask &= (wx > -0.35) & (wx < 0.45) & (np.abs(wy) < 0.5)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
print(f"{cam}: {n-1} blobs (z_table={z_table})")
for i in range(1, n):
    sel = lab == i
    if sel.sum() < 15:
        continue
    pts = P[sel]
    bgr = color[sel].mean(0)
    us, vs = uu[sel], vv[sel]
    print(f" blob {i}: n={sel.sum():4d} px=({us.mean():.0f},{vs.mean():.0f}) "
          f"centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
          f"y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"ztop={np.percentile(pts[:,2],95):.3f} "
          f"rgb=({bgr[2]:.0f},{bgr[1]:.0f},{bgr[0]:.0f})")
OPENRUA_EOF

# openrua op 13
python3 segment.py birdview

# openrua op 14
python3 segment.py agentview

# openrua op 15
cat > topface.py <<'EOF'
import sys, numpy as np, cv2
cam=sys.argv[1]; zmin=float(sys.argv[2]); x0,x1,y0,y1=map(float,sys.argv[3:7])
depth=np.load(f"{cam}_depth.npy"); m=np.load(f"{cam}_meta.npz"); K,T=m["K"],m["T"]
H,W=depth.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-K[0,2])*depth/K[0,0],(vv-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)@T.T
sel=(P[...,2]>zmin)&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
pts=P[sel]
print(f"n={sel.sum()} center=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
EOF
echo "tomato can top (agentview):"; python3 topface.py agentview 0.505 -0.2 0.0 -0.05 0.12
echo "cream cheese top (agentview):"; python3 topface.py agentview 0.445 0.05 0.2 -0.3 -0.15
echo "cream cheese top (birdview):"; python3 topface.py birdview 0.445 0.05 0.2 -0.3 -0.15
echo "basket (birdview) rim:"; python3 topface.py birdview 0.58 -0.15 0.15 0.1 0.4

# openrua op 16
cat > topface.py <<'EOF'
import sys, numpy as np, cv2
cam=sys.argv[1]; zmin=float(sys.argv[2]); zmax=float(sys.argv[3]); x0,x1,y0,y1=map(float,sys.argv[4:8])
depth=np.load(f"{cam}_depth.npy"); m=np.load(f"{cam}_meta.npz"); K,T=m["K"],m["T"]
H,W=depth.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-K[0,2])*depth/K[0,0],(vv-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)@T.T
sel=(P[...,2]>zmin)&(P[...,2]<zmax)&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
pts=P[sel]
print(f"n={sel.sum()} center=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
EOF
echo "tomato can top (agentview):"; python3 topface.py agentview 0.505 0.53 -0.2 0.0 -0.05 0.12
echo "basket rim (birdview):"; python3 topface.py birdview 0.58 0.64 -0.15 0.15 0.1 0.4
echo "basket floor (birdview):"; python3 topface.py birdview 0.40 0.47 -0.07 0.06 0.18 0.32

# openrua op 17
python3 px.py birdview 383 288 375 280 390 295 370 300 395 275

# openrua op 18
timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A1 Translation | head -2; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A2 Translation | head -3

# openrua op 19
python3 topface.py birdview 0.47 0.56 -0.15 0.15 0.12 0.4; python3 topface.py agentview 0.47 0.60 -0.15 0.15 0.12 0.4

# openrua op 20
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm control helpers for this Panda: IK -> trajectory, gripper, FK.

World<->base: base (panda_link0) sits at world (-0.51, 0, 0.42), no rotation.
TCP is hand-frame +Z * tcp_offset. Downward hand with fingers along world Y
is quaternion (1,0,0,0); yaw rotates that about world Z.
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])


def quat_down(yaw=0.0):
    """Quaternion (x,y,z,w) for hand pointing down, fingers along world Y
    rotated by yaw about world Z."""
    # R = Rz(yaw) @ Rx(pi); q = qz * qx ; qx = (1,0,0,0), qz = (0,0,s,c)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # (0,0,s,c) * (1,0,0,0):
    # w = c*0 - 0 = 0? do full Hamilton product:
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Arm:
    def __init__(self):
        M = yaml.safe_load(open("/workspace/machine.yaml"))
        self.traj = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in M["actuators"] if a["kind"] == "gripper")
        self.joints = self.traj["joints"]
        self.tcp_off = float(M["hand"]["tcp_offset_m"])
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.gc.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"

    def _on_js(self, m):
        self._js["m"] = m

    # ---------- sensing ----------
    def joint_state(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def finger_gap(self):
        js = self.joint_state()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        t0 = time.time()
        while "m" not in self._wr and time.time() - t0 < 5:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return (f.x, f.y, f.z)

    def hand_pose_world(self, q=None):
        """FK of panda_hand -> (xyz in world, quat)."""
        q = q or self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(self.joints)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self):
        xyz, q = self.hand_pose_world()
        x, y, z, w = q
        # hand z axis in world = third column of R(q)
        zax = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        return xyz + self.tcp_off * zax

    # ---------- IK ----------
    def ik_hand(self, hand_xyz_world, quat, seed=None, attempts=3):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = np.array(hand_xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        seed = seed or self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(self.joints)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in self.joints]
        raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")

    def ik_tcp(self, tcp_xyz_world, yaw=0.0, seed=None):
        quat = quat_down(yaw)
        x, y, z, w = quat
        zax = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        hand = np.array(tcp_xyz_world) - self.tcp_off * zax
        return self.ik_hand(hand, quat, seed)

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        if via:
            n = len(via) + 1
            for i, vq in enumerate(via, 1):
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                t = seconds * i / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, tcp_xyz_world, yaw=0.0, seconds=3.0):
        q = self.ik_tcp(tcp_xyz_world, yaw)
        code, err = self.move_q(q, seconds)
        tcp = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} target {np.round(tcp_xyz_world, 4)} "
              f"d={np.linalg.norm(tcp - tcp_xyz_world):.4f}", flush=True)
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(self.grip.get("max_effort", 30.0))
        fut = self.gc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={gap:.4f}", flush=True)
        return gap

    def open(self):
        return self.gripper(self.grip["open_m"])

    def close(self):
        return self.gripper(self.grip["closed_m"])
OPENRUA_EOF

# openrua op 21
timeout 120 python3 -u -c "
from arm import *
a = Arm()
print('q', np.round(a.arm_q(),3))
print('gap', a.finger_gap())
print('hand FK world', a.hand_pose_world())
print('tcp world', a.tcp_world())
print('wrench', a.wrench())
q = quat_down(0.0); print('quat_down(0)', q)
print('quat_down(pi/2)', np.round(quat_down(math.pi/2),4))
# test IK for pre-grasp above can
sol = a.ik_tcp([-0.094, 0.032, 0.65], 0.0)
print('IK pregrasp', np.round(sol,3))
print('FK check', a.hand_pose_world(sol))
"

# openrua op 22
timeout 200 python3 -u -c "
from arm import *
import arm as A
a = Arm()
req = GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state.name=list(a.joints); req.robot_state.joint_state.position=a.arm_q()
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
for ps in fut.result().pose_stamped: print(ps.header.frame_id, ps.pose.position)
# Try IK in world coords (BASE_IN_WORLD zero)
A.BASE_IN_WORLD = np.zeros(3)
for target in ([-0.094,0.032,0.65+0.1034], [-0.053,0.0,0.778]):
    try:
        sol = a.ik_hand(target, (1,0,0,0)); print('IK world-frame OK', target, np.round(sol,3))
    except Exception as e: print('IK world-frame fail', target, e)
"

# openrua op 23
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\])/BASE_IN_WORLD = np.zeros(3)  # MoveIt model frame == world on this machine (verified via FK)/' arm.py && grep -n BASE_IN_WORLD arm.py | head -2
timeout 200 python3 -u -c "
from arm import *
a = Arm()
sol = a.ik_hand([-0.053,0.0,0.778], (1,0,0,0))
print('sol', np.round(sol,3)); print('FK', a.hand_pose_world(sol))
sol2 = a.ik_hand([-0.053,0.0,0.778], (1,0,0,0), seed=[0,-0.161,0,-2.445,0,2.227,0.785])
print('sol2', np.round(sol2,3)); print('FK', a.hand_pose_world(sol2))
"

# openrua op 24
sed -i 's/        req.ik_request.group_name = "panda_arm"/        req.ik_request.group_name = "panda_arm"\n        req.ik_request.ik_link_name = "panda_hand"/' arm.py
timeout 200 python3 -u -c "
from arm import *
a = Arm()
sol = a.ik_hand([-0.053,0.0,0.778], (1,0,0,0))
print('sol', np.round(sol,3)); print('FK', a.hand_pose_world(sol))
sol = a.ik_tcp([-0.094, 0.032, 0.65], 0.0)
print('pregrasp sol', np.round(sol,3)); xyz,q=a.hand_pose_world(sol); print('FK hand', xyz.round(4), np.round(q,4))
"

# openrua op 25
timeout 600 python3 -u -c "
from arm import *
a = Arm()
print('gap', a.finger_gap())
a.move_tcp([-0.094, 0.032, 0.65], 0.0, 3.0)
" > step1.log 2>&1; cat step1.log; timeout 90 python3 cam_capture.py robot0_eye_in_hand

# openrua op 26
echo "can top:"; python3 topface.py robot0_eye_in_hand 0.505 0.53 -0.2 0.0 -0.05 0.12; echo "blue can top:"; python3 topface.py robot0_eye_in_hand 0.48 0.53 -0.2 0.0 -0.2 -0.08; python3 px.py robot0_eye_in_hand 320 305 320 240

# openrua op 27
timeout 900 python3 -u -c "
from arm import *
a = Arm()
print('wrench before', a.wrench())
a.move_tcp([-0.0947, 0.0319, 0.465], 0.0, 2.5)
print('wrench at grasp height', a.wrench())
gap = a.close()
print('gap after close', gap)
a.move_tcp([-0.0947, 0.0319, 0.65], 0.0, 2.5)
print('gap after lift', a.finger_gap(), 'wrench', a.wrench())
" > step2.log 2>&1; cat step2.log; timeout 90 python3 cam_capture.py agentview

# openrua op 28
timeout 900 python3 -u -c "
from arm import *
a = Arm()
a.move_tcp([-0.0947, 0.0319, 0.76], 0.0, 2.0)
a.move_tcp([0.0, 0.235, 0.76], 0.0, 3.0)
print('gap in transit', a.finger_gap())
a.move_tcp([0.0, 0.235, 0.60], 0.0, 2.5)
print('gap before release', a.finger_gap(), 'wrench', a.wrench())
a.open()
a.move_tcp([0.0, 0.235, 0.78], 0.0, 2.5)
" > step3.log 2>&1; cat step3.log; timeout 90 python3 cam_capture.py agentview & timeout 90 python3 cam_capture.py birdview & wait

# openrua op 29
echo "objects inside basket footprint above floor(0.49):"; python3 topface.py birdview 0.50 0.60 -0.06 0.06 0.18 0.33

# openrua op 30
timeout 600 python3 -u -c "
from arm import *
a = Arm()
a.move_tcp([0.12, -0.214, 0.62], 0.0, 3.5)
" > step4.log 2>&1; cat step4.log; timeout 90 python3 cam_capture.py robot0_eye_in_hand && cat > pca.py <<'EOF'
import sys, numpy as np
cam=sys.argv[1]; zmin,zmax,x0,x1,y0,y1=map(float,sys.argv[2:8])
depth=np.load(f"{cam}_depth.npy"); m=np.load(f"{cam}_meta.npz"); K,T=m["K"],m["T"]
H,W=depth.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-K[0,2])*depth/K[0,0],(vv-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)@T.T
sel=(P[...,2]>zmin)&(P[...,2]<zmax)&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
pts=P[sel][:,:2]; c=pts.mean(0); ev,evec=np.linalg.eigh(np.cov((pts-c).T))
long=evec[:,1]; ang=np.degrees(np.arctan2(long[1],long[0]))
proj=(pts-c)@evec
print(f"n={len(pts)} center=({c[0]:.4f},{c[1]:.4f}) long-axis angle from +x: {ang:.1f} deg; extents long={proj[:,1].max()-proj[:,1].min():.3f} short={proj[:,0].max()-proj[:,0].min():.3f} ztop={P[sel][:,2].max():.3f}")
EOF
python3 pca.py robot0_eye_in_hand 0.44 0.47 0.05 0.2 -0.3 -0.15

# openrua op 31
timeout 900 python3 -u -c "
from arm import *
a = Arm()
print('gap', a.finger_gap())
a.move_tcp([0.1231, -0.2137, 0.52], 0.0, 2.0)
a.move_tcp([0.1231, -0.2137, 0.443], 0.0, 2.0)
print('wrench at grasp height', a.wrench())
gap = a.close()
a.move_tcp([0.1231, -0.2137, 0.60], 0.0, 2.5)
print('gap after lift', a.finger_gap(), 'wrench', a.wrench())
" > step5.log 2>&1; cat step5.log; timeout 90 python3 cam_capture.py agentview

# openrua op 32
python3 topface.py agentview 0.50 0.56 -0.07 0.07 0.17 0.34; python3 pca.py agentview 0.50 0.56 -0.07 0.07 0.17 0.34

# openrua op 33
python3 topface.py agentview 0.56 0.60 -0.07 0.07 0.17 0.34; python3 px.py agentview 505 268 510 262

# openrua op 34
python3 topface.py agentview 0.515 0.545 -0.07 0.07 0.17 0.34

# openrua op 35
timeout 900 python3 -u -c "
from arm import *
a = Arm()
a.move_tcp([0.1231, -0.2137, 0.76], 0.0, 2.0)
a.move_tcp([0.0, 0.305, 0.76], 0.0, 3.5)
print('gap in transit', a.finger_gap())
a.move_tcp([0.0, 0.305, 0.655], 0.0, 2.0)
print('gap before release', a.finger_gap())
a.open()
a.move_tcp([0.0, 0.305, 0.80], 0.0, 2.0)
# park the arm out of the cameras' view of the basket
a.move_tcp([-0.15, 0.0, 0.80], 0.0, 3.0)
" > step6.log 2>&1; cat step6.log; timeout 90 python3 cam_capture.py agentview & timeout 90 python3 cam_capture.py birdview & timeout 90 python3 cam_capture.py sideview & wait

# openrua op 36
python3 -c "
import cv2; im=cv2.imread('birdview.png'); crop=im[250:330,340:420]; cv2.imwrite('basket_zoom.png', cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))"
echo "cream cheese-colored (bluish) points inside basket:"; python3 - <<'EOF'
import numpy as np, cv2
cam='birdview'; depth=np.load(f"{cam}_depth.npy"); m=np.load(f"{cam}_meta.npz"); K,T=m["K"],m["T"]
img=cv2.imread(f"{cam}.png"); H,W=depth.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-K[0,2])*depth/K[0,0],(vv-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)@T.T
inb=(P[...,0]>-0.06)&(P[...,0]<0.06)&(P[...,1]>0.18)&(P[...,1]<0.33)&(P[...,2]<0.60)
b,g,r=img[...,0].astype(int),img[...,1].astype(int),img[...,2].astype(int)
blue=inb&(b>r+20)
pts=P[blue]; print("bluish n=",blue.sum(), "center", pts[:,:3].mean(0).round(3) if len(pts) else None, "z range", (pts[:,2].min().round(3), pts[:,2].max().round(3)) if len(pts) else None)
dark=inb&(r<90)&(g<90)&(b<90)
pts=P[dark]; print("dark(can top) n=",dark.sum(), "center", pts[:,:3].mean(0).round(3), "z range", (pts[:,2].min().round(3), pts[:,2].max().round(3)))
# anything left on the table at the cheese's old spot?
old=(P[...,0]>0.05)&(P[...,0]<0.2)&(P[...,1]>-0.3)&(P[...,1]<-0.15)&(P[...,2]>0.435)
print("points above table at old cheese location:", old.sum())
EOF
