#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | head -40 && echo --- && ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | sort | uniq; echo ---; ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/px.py)"
cat > /workspace/px.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Batch pixel->world for one camera. Usage: px.py <cam> u,v [u,v ...]
Also prints the camera pose and the world->panda_link0 transform."""
import struct, sys
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def q2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tfmat(t):
    T = np.eye(4); T[:3, :3] = q2R(t.transform.rotation)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    pix = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
    rclpy.init(); node = rclpy.create_node("px")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    import time
    end = time.time() + 10
    frame = f"{cam}_optical_frame"
    while time.time() < end and not (buf.can_transform("world", frame, rclpy.time.Time()) and buf.can_transform("world", "panda_link0", rclpy.time.Time())):
        rclpy.spin_once(node, timeout_sec=0.2)
    Tc = tfmat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    Tb = tfmat(buf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
    print("cam pos", Tc[:3, 3].round(3)); print("base pos", Tb[:3, 3].round(3))
    print("base R\n", Tb[:3, :3].round(3))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    for (u, v) in pix:
        z = float(d[v, u])
        p = Tc @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        pb = np.linalg.inv(Tb) @ p
        print(f"({u},{v}) depth={z:.3f} world={p[:3].round(4)} base={pb[:3].round(4)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 90 python3 px.py agentview 195,290 340,330 455,285 340,395 455,350 150,340 250,340 && echo ---- && timeout 90 python3 px.py birdview 260,285 328,313 383,280 260,240 450,380

# openrua op 8
for c in birdview agentview sideview; do timeout 60 python3 tools/perception/cam_snap.py /$c/depth/image_raw ${c}_depth.png & done; wait; ls *.npy

# openrua op 9
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Build a world-frame point cloud from a saved depth .npy + live intrinsics/TF,
then report the extents of objects above the table in a region.
Usage: cloud.py <cam> <depth.npy> xmin xmax ymin ymax"""
import sys, time
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from px import grab, tfmat


def main():
    cam, npy = sys.argv[1], sys.argv[2]
    xmin, xmax, ymin, ymax = map(float, sys.argv[3:7])
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    Tc = tfmat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    rclpy.shutdown()
    d = np.load(npy)
    h, w = d.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    vs, us = np.mgrid[0:h, 0:w]
    z = d
    pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)], -1).reshape(-1, 4)
    ok = np.isfinite(pts[:, 2]) & (pts[:, 2] > 0.05)
    P = (Tc @ pts[ok].T).T[:, :3]
    sel = (P[:, 0] > xmin) & (P[:, 0] < xmax) & (P[:, 1] > ymin) & (P[:, 1] < ymax)
    Q = P[sel]
    print("n", len(Q), "z range", Q[:, 2].min().round(3), Q[:, 2].max().round(3))
    table = np.median(Q[:, 2])
    print("table z (median)", table.round(3))
    for lo in np.arange(0.905, 1.10, 0.01):
        s = Q[(Q[:, 2] >= lo) & (Q[:, 2] < lo + 0.01)]
        if len(s) < 3:
            continue
        print(f"z[{lo:.3f},{lo+0.01:.3f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] "
              f"y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
echo "== pot A (birdview)"; timeout 60 python3 cloud.py birdview birdview_depth.npy -0.05 0.20 -0.08 0.12; echo "== stove (birdview)"; timeout 60 python3 cloud.py birdview birdview_depth.npy -0.20 0.10 -0.35 -0.08

# openrua op 11
echo "== pot A (agentview)"; timeout 60 python3 cloud.py agentview agentview_depth.npy -0.05 0.20 -0.08 0.13; echo "== pot B (agentview)"; timeout 60 python3 cloud.py agentview agentview_depth.npy -0.20 0.10 0.13 0.32

# openrua op 12
echo "== pot A (sideview)"; timeout 60 python3 cloud.py sideview sideview_depth.npy -0.01 0.20 -0.08 0.13

# openrua op 13
python3 -c "
import cv2, numpy as np
im = cv2.imread('birdview.png'); b,g,r = cv2.split(im.astype(int))
m = (r>150)&(g<90)&(b<90)
ys,xs = np.nonzero(m); print('red px n',len(xs),'center', xs.mean(), ys.mean(), 'x range', xs.min(), xs.max(), 'y range', ys.min(), ys.max())
im = cv2.imread('agentview.png'); b,g,r = cv2.split(im.astype(int))
m = (r>150)&(g<90)&(b<90)
ys,xs = np.nonzero(m); print('agent red px n',len(xs),'center', xs.mean(), ys.mean())
"

# openrua op 14
python3 -c "
import cv2, numpy as np
im = cv2.imread('birdview.png'); b,g,r = cv2.split(im.astype(int))
m = (r>120)&(r-g>50)&(r-b>50)
ys,xs = np.nonzero(m); print('red px n',len(xs),'center', xs.mean(), ys.mean(), 'x range', xs.min(), xs.max(), 'y range', ys.min(), ys.max())
"; timeout 60 python3 px.py birdview 260,285 261,283

# openrua op 15
mkdir -p "$(dirname /workspace/fk.py)"
cat > /workspace/fk.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print current hand + TCP pose (world frame) via MoveIt FK, and finger gap."""
import time
import numpy as np, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from px import q2R

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation (TF, R=I)
TCP = 0.1034


def main():
    rclpy.init(); node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    end = time.time() + 10
    while "m" not in js and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    m = js["m"]; pos = dict(zip(m.name, m.position))
    print("arm:", ", ".join(f"{pos[j]:.4f}" for j in ARM))
    print("fingers:", pos["panda_finger_joint1"], pos["panda_finger_joint2"],
          "gap=", round(pos["panda_finger_joint1"] - pos["panda_finger_joint2"], 4))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [pos[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        print("FK failed", r); return
    p = r.pose_stamped[0].pose
    hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE
    R = q2R(p.orientation)
    tcp = hand + TCP * R[:, 2]
    print("hand world:", hand.round(4), "quat xyzw:", [round(v, 4) for v in (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)])
    print("tcp  world:", tcp.round(4))
    print("hand axes (world): x", R[:, 0].round(3), "y", R[:, 1].round(3), "z", R[:, 2].round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 16
timeout 60 python3 fk.py

# openrua op 17
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm helper: IK/FK/trajectory/gripper with clients built once.

  arm.py ik x y z qx qy qz qw            -> print joint solution (planner frame coords)
  arm.py go x y z qx qy qz qw [sec] [--tcp] -> IK then execute; verify via joint state
  arm.py joints p1,...,p7 [sec]          -> execute joint target
  arm.py grip <per-finger m>
  arm.py fk
"""
import sys, time
import numpy as np, rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from px import q2R

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034


class Arm:
    def __init__(self):
        rclpy.init(); self.node = rclpy.create_node("arm")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip_cli = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def joints(self, fresh=True):
        if fresh:
            self.js = {}
        end = time.time() + 15
        while not self.js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self.js)

    def arm_pos(self):
        j = self.joints(); return [j[n] for n in ARM]

    def fk(self):
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = self.arm_pos()
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        hand = np.array([p.position.x, p.position.y, p.position.z])
        R = q2R(p.orientation)
        return hand, hand + TCP * R[:, 2], R, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, x, y, z, qx, qy, qz, qw, seed=None):
        self.ik_cli.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed or self.arm_pos()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move(self, target, sec=3.0, verify=True):
        self.traj.wait_for_server(10)
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        g.trajectory.points = [pt]
        fut = self.traj.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = self.arm_pos()
        err = max(abs(a - b) for a, b in zip(cur, target))
        print(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def grip(self, w):
        self.grip_cli.wait_for_server(10)
        g = GripperCommand.Goal(); g.command.position = float(w); g.command.max_effort = 30.0
        fut = self.grip_cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        j = self.joints()
        gap = j["panda_finger_joint1"] - j["panda_finger_joint2"]
        print(f"grip done reached={r.reached_goal} stalled={r.stalled} f1={j['panda_finger_joint1']:.4f} f2={j['panda_finger_joint2']:.4f} gap={gap:.4f}")
        return gap

    def go(self, x, y, z, q, sec=3.0, tcp=False):
        if tcp:
            R = q2R(type("Q", (), dict(x=q[0], y=q[1], z=q[2], w=q[3]))())
            x, y, z = np.array([x, y, z]) - TCP * R[:, 2]
        sol = self.ik(x, y, z, *q)
        if sol is None:
            return False
        print("ik sol:", [round(v, 4) for v in sol])
        code, err = self.move(sol, sec)
        hand, tcpp, R, qq = self.fk()
        print("now hand:", hand.round(4), "tcp:", tcpp.round(4), "quat:", [round(v, 3) for v in qq])
        return code == 0 and err < 0.02


def main():
    a = Arm(); cmd = sys.argv[1]; args = [v for v in sys.argv[2:] if not v.startswith("--")]
    if cmd == "fk":
        hand, tcp, R, q = a.fk(); print("hand", hand.round(4), "tcp", tcp.round(4), "quat", [round(v, 4) for v in q])
        print("axes x", R[:, 0].round(3), "y", R[:, 1].round(3), "z", R[:, 2].round(3))
        j = a.joints(); print("fingers", j["panda_finger_joint1"], j["panda_finger_joint2"])
    elif cmd == "ik":
        print(a.ik(*map(float, args[:7])))
    elif cmd == "go":
        v = list(map(float, args)); sec = v[7] if len(v) > 7 else 3.0
        print("OK" if a.go(v[0], v[1], v[2], v[3:7], sec, tcp="--tcp" in sys.argv) else "FAIL")
    elif cmd == "joints":
        a.move([float(v) for v in args[0].split(",")], float(args[1]) if len(args) > 1 else 3.0)
    elif cmd == "grip":
        a.grip(float(args[0]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 18
timeout 120 python3 arm.py ik -0.863 0 2.1816 0.9996 0 -0.0284 0; echo ---; timeout 120 python3 arm.py ik -0.203 0 1.2696 0.9996 0 -0.0284 0

# openrua op 19
timeout 120 python3 -c "
import rclpy, numpy as np
from arm import Arm, ARM
from moveit_msgs.srv import GetPositionFK
from px import q2R
a = Arm()
def fkj(js):
    req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = js
    fut = a.fk_cli.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
    p = fut.result().pose_stamped[0].pose
    R = q2R(p.orientation)
    print('pos', np.round([p.position.x,p.position.y,p.position.z],4), 'quat', np.round([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],4), 'hand y axis', R[:,1].round(3))
fkj([0.05846165151957628, -0.16131001035395545, -0.0579201015032298, -2.4446575328760303, -0.01162123465199098, 2.226770915891463, 0.00890601661941885])
fkj([0,-0.161,0,-2.4446,0,2.2268,0.7854])
sol = a.ik(-0.203, 0, 1.2696, 0.7071, 0.7071, 0, 0); print('ik fingers-along-x:', sol); 
if sol: fkj(sol)
"

# openrua op 20
timeout 300 python3 -c "
import rclpy, numpy as np, time
from arm import Arm, ARM
from moveit_msgs.srv import GetPositionFK
from px import q2R
a = Arm(); a.fk_cli.wait_for_service(10)
def fkj(js):
    req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = js
    t=time.time(); fut = a.fk_cli.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=90)
    r = fut.result(); print('fk took', round(time.time()-t,1))
    if r is None: print('FK none'); return
    p = r.pose_stamped[0].pose
    R = q2R(p.orientation)
    print('pos', np.round([p.position.x,p.position.y,p.position.z],4), 'quat', np.round([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],4), 'hand y axis', R[:,1].round(3))
fkj([0.05846165151957628, -0.16131001035395545, -0.0579201015032298, -2.4446575328760303, -0.01162123465199098, 2.226770915891463, 0.00890601661941885])
fkj([0,-0.161,0,-2.4446,0,2.2268,0.7854])
sol = a.ik(-0.203, 0, 1.2696, 0.7071, 0.7071, 0, 0); print('ik fingers-along-x:', sol); 
if sol: fkj(sol)
"

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw", "new_string": "        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
cat > fkj.py <<'EOF'
import rclpy, numpy as np, time, sys
from arm import Arm, ARM
from moveit_msgs.srv import GetPositionFK
from px import q2R
a = Arm(); a.fk_cli.wait_for_service(10)
def fkj(js):
    req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = [float(v) for v in js]
    t=time.time(); fut = a.fk_cli.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=90)
    r = fut.result(); print('fk took', round(time.time()-t,1))
    if r is None: print('FK none'); return
    p = r.pose_stamped[0].pose
    R = q2R(p.orientation)
    print('pos', np.round([p.position.x,p.position.y,p.position.z],4), 'quat', np.round([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],4), 'hand y axis', R[:,1].round(3), 'z', R[:,2].round(3))
fkj([0.05846165151957628, -0.16131001035395545, -0.0579201015032298, -2.4446575328760303, -0.01162123465199098, 2.226770915891463, 0.00890601661941885])
fkj([0,-0.161,0,-2.4446,0,2.2268,0.7854])
sol = a.ik(-0.203, 0, 1.2696, 0.7071, 0.7071, 0, 0); print('ik fingers-along-x:', sol)
if sol: fkj(sol)
EOF
timeout 300 python3 -u fkj.py

# openrua op 23
cat > fkj2.py <<'EOF'
import rclpy, numpy as np
from arm import Arm, ARM
from fkj import fkj, a
sol = a.ik(-0.203, 0, 1.2696, 0.9239, 0.3827, 0, 0); print('ik:', sol)
if sol: fkj(sol)
EOF
sed -i 's/^fkj(\[0.058/#&/; s/^fkj(\[0,-0.161/#&/; s/^sol = a.ik/#&/; s/^if sol: fkj/#&/' fkj.py
timeout 300 python3 -u fkj2.py

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def go(self, x, y, z, q, sec=3.0, tcp=False):\\n        if tcp:\\n            R = q2R(type(\\"Q\\", (), dict(x=q[0], y=q[1], z=q[2], w=q[3]))())\\n            x, y, z = np.array([x, y, z]) - TCP * R[:, 2]\\n        sol = self.ik(x, y, z, *q)", "new_string": "    def go(self, x, y, z, q, sec=3.0, tcp=False):\\n        \\"\\"\\"q is the desired panda_hand orientation (xyzw, world). The IK tip is\\n        panda_link8 = hand rotated +45 deg about its z, so convert before asking.\\"\\"\\"\\n        R = q2R(type(\\"Q\\", (), dict(x=q[0], y=q[1], z=q[2], w=q[3]))())\\n        if tcp:\\n            x, y, z = np.array([x, y, z]) - TCP * R[:, 2]\\n        c, s = np.cos(np.pi / 8), np.sin(np.pi / 8)\\n        qx, qy, qz, qw = q\\n        q8 = (qw * 0 + qx * c + qy * s, qy * c - qx * s, qz * c + qw * s, qw * c - qz * s)\\n        sol = self.ik(x, y, z, *q8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
timeout 600 python3 -u arm.py grip 0.04 && timeout 600 python3 -u arm.py go 0.050 0.035 1.15 0.7071 0.7071 0 0 4 --tcp

# openrua op 26
timeout 600 python3 -u arm.py joints 0.0745,0.5868,-0.0325,-1.4589,0.0202,2.0453,-0.7472 4 && timeout 100 python3 -u arm.py fk

# openrua op 27
timeout 600 python3 -u arm.py joints 0.0745,0.5868,-0.0325,-1.4589,0.0202,2.0453,-0.7472 3 && timeout 100 python3 -u arm.py fk

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 29
timeout 600 python3 -u arm.py go 0.050 0.035 0.975 0.7071 0.7071 0 0 3 --tcp

# openrua op 30
timeout 600 python3 -u arm.py joints 0.0705,0.8402,-0.026,-1.5037,0.027,2.3436,-0.751 3 && timeout 100 python3 -u arm.py fk && timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -20

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png & timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png & timeout 60 python3 tools/perception/cam_snap.py sideview side2.png & wait

# openrua op 32
timeout 60 python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("rd")
got=[]
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String, "/robot_description", lambda m: got.append(m.data), qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n, timeout_sec=0.5)
open("robot.urdf","w").write(got[0] if got else "")
print(len(got[0]) if got else "none")
EOF
grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A14 'name="panda_finger_joint1"' robot.urdf | head -40; grep -n -A12 '<link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 33
python3 - <<'EOF'
import re
u=open("robot.urdf").read()
for name in ["panda_hand","panda_leftfinger","panda_rightfinger","panda_link8","panda_hand_tcp"]:
    m=re.search(r'<link name="%s">(.*?)</link>'%name,u,re.S)
    print(name, m.group(1)[:600] if m else None); print()
for j in ["panda_joint8","panda_hand_joint","panda_finger_joint1","panda_hand_tcp_joint","virtual_joint"]:
    m=re.search(r'<joint name="%s"[^>]*>(.*?)</joint>'%j,u,re.S)
    print(j, m.group(1)[:500] if m else None); print()
print(re.findall(r'<joint name="([^"]+)" type="([^"]+)"',u))
EOF
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/

# openrua op 34
mkdir -p "$(dirname /workspace/geom.py)"
cat > /workspace/geom.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rough clearance check of a pitched side-grasp on pot A against boxes.
Hand model in the hand frame (origin = TCP, axis a = approach, c = closing):
 - fingers: s in [0,0.045], c in +-[0.0387, 0.0587], hx +-0.01
 - hand body: s in [0.045,0.1034], c +-0.1, hx +-0.035
 - wrist: s in [0.1034, 0.30], radius 0.05
Points sampled on each obstacle box; report min signed distance per part."""
import numpy as np, sys

OBST = {
    "potA_body_low": ([0.013, -0.002, 0.90], [0.088, 0.072, 0.945]),
    "potA_waist": ([0.018, 0.003, 0.945], [0.084, 0.067, 0.98]),
    "potA_upper": ([0.015, 0.0, 0.98], [0.087, 0.071, 1.03]),
    "potA_lid": ([0.012, -0.003, 1.03], [0.088, 0.073, 1.046]),
    "potA_knob": ([0.043, 0.028, 1.046], [0.058, 0.043, 1.062]),
    "potA_spout": ([0.038, 0.067, 0.985], [0.062, 0.095, 1.04]),
    "potA_handle": ([0.038, -0.05, 0.945], [0.062, 0.0, 1.04]),
    "potB_body": ([-0.09, 0.19, 0.90], [-0.03, 0.27, 1.062]),
    "potB_handle": ([-0.075, 0.14, 0.945], [-0.045, 0.19, 1.04]),
    "table": ([-0.5, -0.5, 0.80], [0.6, 0.6, 0.90]),
}


def frame(phi_deg, elev_deg):
    ph, el = np.radians(phi_deg), np.radians(elev_deg)
    src = np.array([np.cos(ph) * np.cos(el), np.sin(ph) * np.cos(el), np.sin(el)])  # -approach
    a = -src
    c = np.array([np.sin(ph), -np.cos(ph), 0.0])
    hx = np.cross(c, a)
    return a, c, hx


def box_pts(lo, hi, n=8):
    g = [np.linspace(lo[i], hi[i], n) for i in range(3)]
    return np.array(np.meshgrid(*g, indexing="ij")).reshape(3, -1).T


def check(tcp, phi, elev, verbose=True):
    a, c, hx = frame(phi, elev)
    worst = {}
    for name, (lo, hi) in OBST.items():
        P = box_pts(lo, hi) - np.array(tcp)
        s = -(P @ a); pc = P @ c; ph = P @ hx
        # fingers (grasp target is potA waist so skip finger/potA_waist check)
        d_f = np.maximum.reduce([-s, s - 0.045, 0.0387 - np.abs(pc), np.abs(pc) - 0.0587, np.abs(ph) - 0.01])
        d_h = np.maximum.reduce([0.045 - s, s - 0.1034, np.abs(pc) - 0.1, np.abs(ph) - 0.035])
        d_w = np.maximum.reduce([0.1034 - s, s - 0.30, np.hypot(pc, ph) - 0.05])
        worst[name] = (d_f.min(), d_h.min(), d_w.min())
    if verbose:
        print(f"phi={phi} elev={elev} tcp={tcp}")
        for k, v in worst.items():
            flag = " <-- COLLISION" if min(v) < 0 else ""
            print(f"  {k:15s} finger={v[0]:+.3f} hand={v[1]:+.3f} wrist={v[2]:+.3f}{flag}")
    return worst


if __name__ == "__main__":
    tcp = [float(v) for v in sys.argv[1:4]]
    check(tcp, float(sys.argv[4]), float(sys.argv[5]))
OPENRUA_EOF

# openrua op 35
python3 geom.py 0.049 0.02 0.962 85 30 && python3 geom.py 0.049 0.02 0.962 85 20 && python3 geom.py 0.049 0.015 0.958 80 15 && python3 geom.py 0.049 0.02 0.962 90 30

# openrua op 36
timeout 600 python3 -u arm.py go 0.050 0.036 1.15 0.7071 0.7071 0 0 3 --tcp; timeout 600 python3 -u arm.py fk

# openrua op 37
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih3_depth.png && timeout 60 python3 cloud.py robot0_eye_in_hand eih3_depth.npy -0.05 0.20 -0.10 0.13

# openrua op 38
timeout 600 python3 -u arm.py go 0.051 0.037 1.08 0.7071 0.7071 0 0 3 --tcp && timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 39
timeout 600 python3 -u arm.py go 0.051 0.037 1.049 0.7071 0.7071 0 0 2 --tcp && timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png

# openrua op 40
timeout 600 python3 -u arm.py grip 0.0 && timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 41
timeout 600 python3 -u arm.py go 0.051 0.037 1.12 0.7071 0.7071 0 0 3 --tcp && timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 30 ros2 topic echo /joint_states --once | grep -A9 position | tail -2 && timeout 60 python3 tools/perception/cam_snap.py sideview side3.png

# openrua op 42
timeout 600 python3 -u arm.py go -0.045 -0.212 1.16 0.7071 0.7071 0 0 4 --tcp && timeout 30 ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png && timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 44
timeout 600 python3 -u arm.py go -0.045 -0.212 1.092 0.7071 0.7071 0 0 3 --tcp && timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 45
timeout 600 python3 -u arm.py grip 0.04 && timeout 600 python3 -u arm.py go -0.045 -0.212 1.20 0.7071 0.7071 0 0 3 --tcp

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png & timeout 60 python3 tools/perception/cam_snap.py sideview side4.png & timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird4_depth.png & wait; timeout 60 python3 cloud.py birdview bird4_depth.npy -0.20 0.10 -0.35 -0.08

# openrua op 47
timeout 600 python3 -u arm.py go -0.20 0.0 1.30 0.7071 0.7071 0 0 4 --tcp && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird5_depth.png && timeout 60 python3 cloud.py birdview bird5_depth.npy -0.20 0.10 -0.35 -0.08 && timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agent5_depth.png && timeout 60 python3 cloud.py agentview agent5_depth.npy -0.15 0.10 -0.30 -0.12

# openrua op 48
timeout 60 python3 tools/perception/cam_snap.py frontview front_final.png
