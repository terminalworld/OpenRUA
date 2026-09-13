#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/*.py tools/perception/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 40 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60; echo ---; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects on the table from one camera's depth + color.

Usage: python3 scene.py <camera> [min_height_above_table=0.01]
Prints world-frame cluster centroids / bounding boxes and writes
<camera>_labels.png with cluster ids drawn on the color image.
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


def grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
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
    hmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T  # world coords HxWx3
    valid = np.isfinite(depth) & (depth > 0)
    # table height: the most common z among valid points
    zs = P[..., 2][valid]
    hist, edges = np.histogram(zs, bins=400)
    table_z = edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2
    print(f"camera {cam} at world {T.round(3)}; table_z ~ {table_z:.3f}")
    mask = valid & (P[..., 2] > table_z + hmin) & (P[..., 2] < table_z + 0.5)
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    out = color.copy()
    rows = []
    for i in range(1, n + 1):
        sel = lab == i
        if sel.sum() < 30:
            continue
        pts = P[sel]
        vv, uu = np.nonzero(sel)
        c = pts.mean(0)
        lo, hi = pts.min(0), pts.max(0)
        rows.append((i, int(uu.mean()), int(vv.mean()), c, lo, hi, sel.sum()))
        cv2.putText(out, str(i), (int(uu.mean()), int(vv.mean())),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.rectangle(out, (uu.min(), vv.min()), (uu.max(), vv.max()),
                      (0, 255, 0), 1)
    for i, u, v, c, lo, hi, npx in rows:
        print(f"[{i:2d}] px=({u},{v}) n={npx:5d} "
              f"center=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f}) "
              f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
              f"z[{lo[2]:.3f},{hi[2]:.3f}]")
    cv2.imwrite(f"{cam}_labels.png", out)
    np.save(f"{cam}_P.npy", P)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scene.py agentview 0.01

# openrua op 8
timeout 120 python3 scene.py birdview 0.015; echo ---; timeout 120 python3 scene.py robot0_robotview 0.015; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 9
mkdir -p "$(dirname /workspace/cluster_xy.py)"
cat > /workspace/cluster_xy.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Cluster saved world point cloud (<cam>_P.npy) in world XY grid."""
import sys
import numpy as np
from scipy import ndimage

cam = sys.argv[1]
table_z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.43
hmin = float(sys.argv[3]) if len(sys.argv) > 3 else 0.015
res = 0.005
P = np.load(f"{cam}_P.npy")
pts = P.reshape(-1, 3)
pts = pts[np.isfinite(pts).all(1)]
sel = (pts[:, 2] > table_z + hmin) & (pts[:, 2] < table_z + 0.4) & \
      (pts[:, 0] > -0.45) & (pts[:, 0] < 0.5) & (abs(pts[:, 1]) < 0.6)
pts = pts[sel]
x0, y0 = pts[:, 0].min(), pts[:, 1].min()
ix = ((pts[:, 0] - x0) / res).astype(int)
iy = ((pts[:, 1] - y0) / res).astype(int)
grid = np.zeros((ix.max() + 1, iy.max() + 1), bool)
grid[ix, iy] = True
grid = ndimage.binary_closing(grid, np.ones((3, 3)))
lab, n = ndimage.label(grid, structure=np.ones((3, 3)))
cell_lab = lab[ix, iy]
for i in range(1, n + 1):
    q = pts[cell_lab == i]
    if len(q) < 40:
        continue
    c = q.mean(0); lo = q.min(0); hi = q.max(0)
    print(f"[{i:2d}] n={len(q):5d} center=({c[0]:.3f},{c[1]:.3f}) "
          f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
          f"ztop={hi[2]:.3f} size=({hi[0]-lo[0]:.3f},{hi[1]-lo[1]:.3f})")
OPENRUA_EOF

# openrua op 10
echo robotview; python3 cluster_xy.py robot0_robotview 0.43 0.015; echo agentview; python3 cluster_xy.py agentview 0.43 0.015

# openrua op 11
timeout 120 python3 scene.py robot0_eye_in_hand 0.015 && python3 cluster_xy.py robot0_eye_in_hand 0.43 0.012

# openrua op 12
mkdir -p "$(dirname /workspace/fk.py)"
cat > /workspace/fk.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print the current panda_hand pose (base frame and world frame) via /compute_fk."""
import rclpy
import numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_W = np.array([-0.51, 0.0, 0.42])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    print("joints:", [round(d[j], 4) for j in ARM])
    print("fingers:", round(d["panda_finger_joint1"], 4), round(d["panda_finger_joint2"], 4))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [d[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        raise SystemExit(f"FK failed: {res}")
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    R = quat_R(*q)
    tcp = pos + 0.1034 * R[:, 2]
    print("frame:", res.pose_stamped[0].header.frame_id)
    print("hand base:", pos.round(4), "quat xyzw:", np.round(q, 4))
    print("hand world:", (pos + BASE_W).round(4))
    print("tcp world:", (tcp + BASE_W).round(4))
    print("hand axes (world): x=", R[:, 0].round(3), "y=", R[:, 1].round(3), "z=", R[:, 2].round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 13
timeout 60 python3 fk.py

# openrua op 14
python3 -c "
import numpy as np
q=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
a=[0,0,0,0.0825,-0.0825,0,0.088,0]; d=[0.333,0,0.316,0,0.384,0,0,0.107]
al=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2,0]
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
M=np.eye(4)
for i in range(8): M=M@T(a[i],d[i],al[i],q[i] if i<7 else 0)
print('link8 in base:',M[:3,3].round(4))
print(M[:3,:3].round(3))
"


# openrua op 15
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helpers: joint state, FK, IK, trajectory, gripper, servo.

Kept in one long-lived node so clients are built once.
"""
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP_OFF = 0.1034
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07),
          (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# hand pointing straight down, fingers closing along world Y
# (hand X = world X, hand Y = world -Y, hand Z = world -Z): 180 deg about X
Q_DOWN_X = (1.0, 0.0, 0.0, 0.0)
# hand pointing down, fingers closing along world X: rotate Q_DOWN_X by 90deg about Z
Q_DOWN_Y = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)


def q_down_yaw(yaw):
    """Hand pointing down with the hand X axis rotated by yaw about world Z."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # qz = (0,0,s,c); qx180 = (1,0,0,0); product qz*qx180:
    # w = c*0 - 0*1 - 0*0 - s*0 = 0 ; x = c*1 + 0 + 0*0 - s*0 = c
    # y = c*0 - 0*0 + 0*1 + s*0 ... careful: use generic multiply
    return quat_mul((0, 0, s, c), (1, 0, 0, 0))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.traj = ActionClient(self.node, FollowJointTrajectory,
                                 "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand,
                                 "/franka_gripper/gripper_action")
        self.twist_pub = self.node.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fk_cli.wait_for_service(10)
        self.ik_cli.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m
        self._js["t"] = time.time()

    def spin(self, sec=0.2):
        rclpy.spin_once(self.node, timeout_sec=sec)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        d = dict(zip(self._js["m"].name, self._js["m"].position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = (p.orientation.x, p.orientation.y, p.orientation.z,
                p.orientation.w)
        return pos, quat

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP_OFF * R[:, 2], quat

    def ik(self, pos, quat, seed=None, tcp=True, attempts=3):
        """pos: target position (world frame). If tcp, pos is the fingertip
        centre; converted to hand frame origin."""
        pos = np.array(pos, float)
        if tcp:
            R = quat_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(attempts):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        code = None if res is None else res.error_code.val
        raise RuntimeError(f"IK failed code={code} for pos={pos} quat={quat}")

    def move_q(self, q, seconds=3.0, via=None):
        """Send one trajectory to joint config q (optionally through via
        points, each (q, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=list(map(float, vq)))
                pt.time_from_start = Duration(sec=int(vt),
                                              nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=list(map(float, q)))
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        handle = send.result()
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(now, q))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        code, err = self.move_q(q, seconds)
        tp, _ = self.tcp()
        print(f"  tcp now {tp.round(4)} target {np.array(pos).round(4)}")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, vx=0, vy=0, vz=0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = "panda_link0"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
OPENRUA_EOF

# openrua op 16
timeout 120 python3 -c "
from arm import *
a=Arm()
pos,quat=a.fk(); print('hand world', pos.round(4), np.round(quat,4))
q0=a.arm_q()
# test 1: IK for the current hand pose as given (world coords)
try:
    q=a.ik(pos,quat,tcp=False); print('IK(world coords) ->', np.round(q,3), 'seed', np.round(q0,3))
    print('  FK check', a.fk(q)[0].round(4))
except Exception as e: print('IK world coords:', e)
# test 2: same pose expressed in base coords
pb=pos-np.array([-0.51,0,0.42])
try:
    q=a.ik(pb,quat,tcp=False); print('IK(base coords) ->', np.round(q,3))
    print('  FK check', a.fk(q)[0].round(4))
except Exception as e: print('IK base coords:', e)
"


# openrua op 17
timeout 120 python3 -c "
from arm import *
a=Arm()
q=[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009]
for l in ['panda_link8','panda_hand']:
    p,qu=a.fk(q,link=l); print(l, p.round(4), np.round(qu,4))
p,qu=a.fk(a.arm_q(),link='panda_link8'); print('current link8', p.round(4), np.round(qu,4))
"


# openrua op 18
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        p = req.ik_request.pose_stamped.pose\\n        p.position.x, p.position.y, p.position.z = map(float, pos)\\n        (p.orientation.x, p.orientation.y, p.orientation.z,\\n         p.orientation.w) = map(float, quat)", "new_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        p = req.ik_request.pose_stamped.pose\\n        p.position.x, p.position.y, p.position.z = map(float, pos)\\n        # the IK tip link is panda_link8 = panda_hand rotated +45deg about z\\n        q8 = quat_mul(quat, (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))\\n        (p.orientation.x, p.orientation.y, p.orientation.z,\\n         p.orientation.w) = map(float, q8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 19
timeout 120 python3 -c "
from arm import *
a=Arm()
pos,quat=a.fk()
q=a.ik(pos,quat,tcp=False); print('IK ->', np.round(q,3))
p,qu=a.fk(q); print('hand FK', p.round(4), np.round(qu,4))
# test target: pre-grasp above cream cheese, TCP at (0.012,-0.258,0.60), fingers along y
tq=Q_DOWN_X
q=a.ik((0.012,-0.258,0.60),tq); print('pregrasp IK', np.round(q,3))
tp,qu=a.tcp(q); print('tcp FK', tp.round(4), np.round(qu,4))
q2=a.ik((0.012,-0.258,0.44),tq,seed=q); print('grasp IK', np.round(q2,3)); print('tcp', a.tcp(q2)[0].round(4))
"


# openrua op 20
timeout 300 python3 -c "
from arm import *
a=Arm()
def yaw_of(R): return math.degrees(math.atan2(R[1,0],R[0,0]))
pos=(0.012,-0.258,0.60)
for deg in [0,15,30,45,60,90,-30,-45]:
    qh=q_down_yaw(math.radians(deg))
    try:
        q=a.ik(pos,qh)
    except Exception as e:
        print(deg,'IK fail'); continue
    _,qhand=a.fk(q); _,q8=a.fk(q,'panda_link8')
    print(f'req hand yaw {deg:4d}: sol j7={q[6]:.3f}  hand yaw={yaw_of(quat_R(*qhand)):7.1f}  link8 yaw={yaw_of(quat_R(*q8)):7.1f}  hand z={quat_R(*qhand)[:,2].round(2)}')
"


# openrua op 21
timeout 300 python3 -c "
from arm import *
import arm
a=Arm()
def yaw_of(R): return math.degrees(math.atan2(R[1,0],R[0,0]))
pos=(0.012,-0.258,0.60)
# bypass conversion: send raw quaternion as the request
def raw_ik(pos,quat,seed):
    req=GetPositionIK.Request(); req.ik_request.group_name='panda_arm'
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=pos
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat
    req.ik_request.robot_state.joint_state.name=ARM
    req.ik_request.robot_state.joint_state.position=seed
    req.ik_request.timeout=Duration(sec=1)
    fut=a.ik_cli.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=60)
    r=fut.result(); 
    if r.error_code.val!=1: return None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)); return [sol[j] for j in ARM]
seed=a.arm_q()
for deg in [0,30,60,90]:
    qh=q_down_yaw(math.radians(deg))
    hz=(0.012,-0.258,0.60-TCP_OFF)
    for s in [seed]:
        q=raw_ik(hz,qh,s)
        if q is None: print(deg,'fail'); continue
        _,qhand=a.fk(q); _,q8=a.fk(q,'panda_link8')
        print(f'raw req yaw {deg}: j7={q[6]:.3f} hand yaw={yaw_of(quat_R(*qhand)):.1f} link8 yaw={yaw_of(quat_R(*q8)):.1f}')
"


# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        q8 = quat_mul(quat, (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))", "new_string": "        q8 = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8),\\n                             math.cos(math.pi / 8)))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 300 python3 -c "
from arm import *
a=Arm()
def yaw_of(R): return math.degrees(math.atan2(R[1,0],R[0,0]))
pos=(0.012,-0.258,0.60)
for deg in [0,45,90,-90]:
    qh=q_down_yaw(math.radians(deg)); q=a.ik(pos,qh)
    tp,qhand=a.fk(q); R=quat_R(*qhand)
    tcp,_=a.tcp(q)
    print(f'req yaw {deg}: j7={q[6]:.3f} hand yaw={yaw_of(R):.1f} hand y-axis={R[:,1].round(2)} tcp={tcp.round(3)} q={np.round(q,2)}')
"


# openrua op 24
python3 -c "
import numpy as np
for cam in ['robot0_eye_in_hand','agentview','robot0_robotview']:
    P=np.load(f'{cam}_P.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    for (x,y) in [(0.012,-0.258),(-0.106,0.029),(0.0,-0.15),(-0.2,0.1)]:
        s=(abs(P[:,0]-x)<0.06)&(abs(P[:,1]-y)<0.06)
        z=P[s,2]
        print(cam,(x,y),'z pct 5/50/95:',np.percentile(z,[5,50,95]).round(4),'max',z.max().round(3))
"


# openrua op 25
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -n -A12 '<link name=\"panda_hand\"' robot.urdf | head -40; grep -n -B2 -A6 'panda_finger_joint1\|panda_hand_joint\|panda_hand_tcp' robot.urdf | grep -E 'origin|name|parent|child|axis' | head -20

# openrua op 26
python3 -c "
import numpy as np,struct
for f in ['hand','finger']:
    p=f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'
    d=open(p,'rb').read()
    n=struct.unpack('<I',d[80:84])[0]
    v=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))['v'].reshape(-1,3)
    print(f,'min',v.min(0).round(4),'max',v.max(0).round(4))
"

# openrua op 27
mkdir -p "$(dirname /workspace/pick.py)"
cat > /workspace/pick.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick a flat box at (x,y) and lift it.  Usage: pick.py x y grasp_z [yaw_deg]"""
import sys
import numpy as np
from arm import *

x, y, gz = map(float, sys.argv[1:4])
yaw = math.radians(float(sys.argv[4])) if len(sys.argv) > 4 else 0.0
Q = q_down_yaw(yaw)
a = Arm()
print("start tcp", a.tcp()[0].round(4), "fingers", a.fingers(), flush=True)

print("open gripper", flush=True)
a.gripper(0.04)

print("pregrasp above", flush=True)
q_pre = a.ik((x, y, 0.60), Q)
a.move_q(q_pre, 4.0)
tp, _ = a.tcp(); print("  tcp", tp.round(4), flush=True)

print("descend", flush=True)
seed = q_pre
via = []
for i, z in enumerate([0.54, 0.49, 0.46]):
    seed = a.ik((x, y, z), Q, seed=seed)
    via.append((seed, 1.0 + i * 1.0))
q_g = a.ik((x, y, gz), Q, seed=seed)
a.move_q(q_g, 4.5, via=via)
tp, _ = a.tcp(); print("  tcp", tp.round(4), "target", (x, y, gz), flush=True)
err = np.array([x, y, gz]) - tp
if np.linalg.norm(err[:2]) > 0.004 or abs(err[2]) > 0.004:
    print("  correcting", err.round(4), flush=True)
    q_g = a.ik((x, y, gz), Q, seed=a.arm_q())
    a.move_q(q_g, 1.5)
    tp, _ = a.tcp(); print("  tcp", tp.round(4), flush=True)

print("close gripper", flush=True)
f = a.gripper(0.0)
print("  finger gap after close:", round(f[0] - f[1], 4), flush=True)

print("lift", flush=True)
q_up = a.ik((x, y, 0.62), Q, seed=a.arm_q())
a.move_q(q_up, 3.0)
tp, _ = a.tcp(); print("  tcp", tp.round(4), flush=True)
f = a.fingers(); print("  fingers after lift:", f, "gap", round(f[0] - f[1], 4), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 28
nohup python3 -u pick.py 0.012 -0.258 0.437 > pick_cc.log 2>&1 &
sleep 45; cat /workspace/pick_cc.log

# openrua op 29
sleep 20; cat /workspace/pick_cc.log; timeout 40 python3 tools/perception/cam_snap.py agentview cc_lift.png

# openrua op 30
mkdir -p "$(dirname /workspace/place.py)"
cat > /workspace/place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the held object over the basket, measure the interior, release.
Usage: place.py bx by [release_z=0.65]"""
import sys
import subprocess
import numpy as np
from arm import *

bx, by = map(float, sys.argv[1:3])
rz = float(sys.argv[3]) if len(sys.argv) > 3 else 0.65
Q = q_down_yaw(0.0)
a = Arm()
tp, _ = a.tcp()
print("start tcp", tp.round(4), "fingers", a.fingers(), flush=True)

print("transit over basket", flush=True)
mid = a.ik((tp[0], (tp[1] + by) / 2, 0.72), Q)
q_over = a.ik((bx, by, 0.75), Q, seed=mid)
a.move_q(q_over, 5.0, via=[(mid, 2.5)])
tp, _ = a.tcp(); print("  tcp", tp.round(4), "fingers", a.fingers(), flush=True)

print("measuring basket from eye-in-hand", flush=True)
subprocess.run([sys.executable, "scene.py", "robot0_eye_in_hand", "0.05"])
P = np.load("robot0_eye_in_hand_P.npy").reshape(-1, 3)
P = P[np.isfinite(P).all(1)]
rim = P[(P[:, 2] > 0.56) & (P[:, 2] < 0.66) & (abs(P[:, 0] - bx) < 0.15)
        & (abs(P[:, 1] - by) < 0.15)]
if len(rim):
    print(f"  rim pts n={len(rim)} x[{rim[:,0].min():.3f},{rim[:,0].max():.3f}] "
          f"y[{rim[:,1].min():.3f},{rim[:,1].max():.3f}] ztop={rim[:,2].max():.3f}",
          flush=True)
    cx, cy = (rim[:, 0].min() + rim[:, 0].max()) / 2, (rim[:, 1].min() + rim[:, 1].max()) / 2
    print(f"  rim centre ({cx:.3f},{cy:.3f})", flush=True)
else:
    cx, cy = bx, by
floor = P[(P[:, 2] < 0.5) & (abs(P[:, 0] - cx) < 0.05) & (abs(P[:, 1] - cy) < 0.05)]
if len(floor):
    print(f"  basket floor z ~ {np.percentile(floor[:,2],50):.3f} (n={len(floor)})", flush=True)

print("centre and lower", flush=True)
q_rel = a.ik((cx, cy, rz), Q, seed=a.arm_q())
a.move_q(q_rel, 3.0)
tp, _ = a.tcp(); print("  tcp", tp.round(4), "fingers", a.fingers(), flush=True)

print("release", flush=True)
a.gripper(0.04)
print("retreat up", flush=True)
q_up = a.ik((cx, cy, 0.75), Q, seed=a.arm_q())
a.move_q(q_up, 2.5)
print("  tcp", a.tcp()[0].round(4), flush=True)
print("DONE", flush=True)
OPENRUA_EOF

# openrua op 31
nohup python3 -u place.py 0.012 0.24 0.65 > place_cc.log 2>&1 &
sleep 60; cat /workspace/place_cc.log

# openrua op 32
timeout 40 python3 tools/perception/cam_snap.py agentview cc_placed.png; timeout 40 python3 tools/perception/cam_snap.py robot0_eye_in_hand cc_placed_eih.png

# openrua op 33
nohup python3 -u pick.py -0.106 0.029 0.434 > pick_butter.log 2>&1 &
sleep 60; cat /workspace/pick_butter.log

# openrua op 34
sleep 15; cat /workspace/pick_butter.log | tail -5

# openrua op 35
nohup python3 -u place.py 0.015 0.253 0.65 > place_butter.log 2>&1 &
sleep 60; cat /workspace/place_butter.log

# openrua op 36
timeout 40 python3 tools/perception/cam_snap.py robot0_eye_in_hand final_eih.png; timeout 40 python3 tools/perception/cam_snap.py agentview final_agent.png

# openrua op 37
timeout 120 python3 scene.py robot0_robotview 0.012 >/dev/null && echo "objects on table (robotview, world XY clusters):" && python3 cluster_xy.py robot0_robotview 0.425 0.012; echo; echo "basket interior contents (eye-in-hand):"; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_P.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
s=(P[:,0]>-0.04)&(P[:,0]<0.07)&(P[:,1]>0.19)&(P[:,1]<0.31)&(P[:,2]<0.56)
q=P[s]; print('n',len(q),'z range',q[:,2].min().round(3),q[:,2].max().round(3))
top=q[q[:,2]>0.47]; print('raised contents: x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max(),top[:,2].max()))
"
