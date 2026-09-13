#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ----; timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12

# openrua op 6
timeout 60 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | grep -A3 "Translation" | head -5; timeout 60 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A3 "Translation" | head -5; timeout 30 ros2 topic echo /agentview/color/camera_info --once | head -20

# openrua op 7
find / -iname "*.bddl" 2>/dev/null | head; find / -maxdepth 4 -iname "*libero*" 2>/dev/null | head; pip list 2>/dev/null | grep -i -E "libero|robosuite|mujoco"

# openrua op 8
mkdir -p "$(dirname /workspace/locate.py)"
cat > /workspace/locate.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save arrays for offline use.

Usage: python3 locate.py <camera>
Writes <camera>_color.png, <camera>_depth.npy, <camera>_meta.npy (K, T_world_cam).
"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


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
    node = rclpy.create_node("locate")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    bridge = CvBridge()
    color = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    K = np.array(info.k).reshape(3, 3)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 20
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    cv2.imwrite(f"{cam}_color.png", color)
    np.save(f"{cam}_depth.npy", depth)
    np.save(f"{cam}_meta.npy", {"K": K, "T": T}, allow_pickle=True)
    print(cam, "saved; depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
mkdir -p "$(dirname /workspace/px.py)"
cat > /workspace/px.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Offline pixel -> world using saved arrays from locate.py.

Usage: python3 px.py <camera> u v [u v ...]
"""
import sys
import numpy as np

cam = sys.argv[1]
depth = np.load(f"{cam}_depth.npy")
meta = np.load(f"{cam}_meta.npy", allow_pickle=True).item()
K, T = meta["K"], meta["T"]
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]


def to_world(u, v):
    z = float(depth[v, u])
    p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
    return (T @ p)[:3], z


vals = list(map(int, sys.argv[2:]))
for u, v in zip(vals[::2], vals[1::2]):
    w, z = to_world(u, v)
    print(f"({u},{v}) depth={z:.3f} -> world {w[0]:.3f} {w[1]:.3f} {w[2]:.3f}")
OPENRUA_EOF

# openrua op 10
timeout 120 python3 locate.py birdview && timeout 120 python3 locate.py agentview && python3 px.py birdview 320 240 320 315 285 262 325 235 325 280

# openrua op 11
python3 -c "
import cv2, numpy as np
img = cv2.imread('birdview_color.png')
crop = img[200:360, 240:400]
cv2.imwrite('bird_crop.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
d = np.load('birdview_depth.npy')
# table height estimate: depth at empty table pixel
print('table px depth', d[330,250], d[200,320])
"


# openrua op 12
python3 px.py birdview 250 330 300 330 320 315 310 315 330 315 320 305 320 325 326 235 318 235 334 235 326 228 326 243 286 259 278 259 294 259 286 251 286 267

# openrua op 13
python3 -c "
import cv2
img = cv2.imread('agentview_color.png')
cv2.imwrite('agent_crop.png', cv2.resize(img[120:420, 150:470], None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
"; python3 px.py agentview 320 440 320 365 320 340 320 390 405 285 405 275 400 265 335 190 335 160 320 210 220 220 220 190

# openrua op 14
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 15
timeout 120 python3 locate.py birdview && timeout 120 python3 locate.py agentview && python3 px.py birdview 250 330 320 315 326 235 318 235 286 259 && python3 px.py agentview 320 440 320 365 405 285 335 160

# openrua op 16
python3 -c "
import numpy as np, cv2
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
h = 3.0 - d
mask = (h > 0.435) & (h < 0.70)   # objects above table, below arm bulk
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    x,y,w,hh,a = stats[i]
    if a < 30: continue
    ys,xs = np.where(lab==i)
    z = d[ys,xs]
    X = (xs-cx)*z/fx; Y=(ys-cy)*z/fy
    P = (T[:3,:3] @ np.vstack([X,Y,z]) + T[:3,3:4])
    print(f'comp {i}: px bbox u[{x},{x+w}] v[{y},{y+hh}] area={a}  world x[{P[0].min():.3f},{P[0].max():.3f}] y[{P[1].min():.3f},{P[1].max():.3f}] zmax={P[2].max():.3f} centroid=({P[0].mean():.3f},{P[1].mean():.3f})')
vis = np.clip((h-0.42)/0.3*255,0,255).astype(np.uint8)
cv2.imwrite('bird_height.png', cv2.resize(vis[200:360,240:400], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
"


# openrua op 17
python3 -c "
import numpy as np, cv2
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape
us,vs = np.meshgrid(np.arange(W), np.arange(H))
z = d
X=(us-cx)*z/fx; Y=(vs-cy)*z/fy
P = (T[:3,:3] @ np.vstack([X.ravel(),Y.ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
# pudding: world region x in [-0.15,0.05], y in [0.05,0.2], z in [0.435,0.5]
m = (P[0]>-0.15)&(P[0]<0.05)&(P[1]>0.05)&(P[1]<0.2)&(P[2]>0.435)&(P[2]<0.50)
ys,xs = np.where(m)
print('pudding pixels', len(ys), 'u',xs.min(),xs.max(),'v',ys.min(),ys.max())
pts = np.vstack([P[0][m],P[1][m]]).T
print('x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max(),'z',P[2][m].min(),P[2][m].max())
print('centroid', pts.mean(0))
# PCA for orientation
c = pts.mean(0); u,s,vt = np.linalg.svd(pts-c)
print('principal axis', vt[0], 'sv', s/np.sqrt(len(pts)))
# top-face only (z>0.455)
m2 = m & (P[2]>0.455)
pts2 = np.vstack([P[0][m2],P[1][m2]]).T
c2 = pts2.mean(0); u,s,vt = np.linalg.svd(pts2-c2)
print('top face centroid', c2, 'axis', vt[0], 'sv', s/np.sqrt(len(pts2)), 'n', len(pts2))
print('x',pts2[:,0].min(),pts2[:,0].max(),'y',pts2[:,1].min(),pts2[:,1].max())
"


# openrua op 18
mkdir -p "$(dirname /workspace/fk.py)"
cat > /workspace/fk.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print the panda_hand pose (base frame and world frame) via /compute_fk."""
import rclpy, yaml
import numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")["joints"]
BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (tf2_echo)


def main():
    rclpy.init()
    node = rclpy.create_node("fk")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    seed = JointState()
    for n, p in zip(js["m"].name, js["m"].position):
        if n in ARM:
            seed.name.append(n); seed.position.append(p)
    req.robot_state.joint_state = seed
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    print("joints", dict(zip(seed.name, [round(x, 4) for x in seed.position])))
    print("hand in base :", pos.round(4), "quat", [round(v, 4) for v in (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)])
    print("hand in world:", (pos + BASE).round(4), " tcp in world:", end=" ")
    q = p.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    zaxis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
    print((pos + BASE + 0.1034 * zaxis).round(4), "hand z-axis", zaxis.round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 19
timeout 120 python3 fk.py

# openrua op 20
python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161037,0,-2.444597,0,2.226752,0.785398]
A=[0,0,0,0.0825,-0.0825,0,0.088]; D=[0.333,0,0.316,0,0.384,0,0]; AL=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
T=np.eye(4)
for i in range(7): T=T@dh(A[i],D[i],AL[i],q[i])
T=T@dh(0,0.107,0,0)  # flange (link8)
print('link8 in base', T[:3,3].round(4)); print('z axis', T[:3,2].round(3))
print('tcp', (T[:3,3]+0.1034*T[:3,2]).round(4))
"
python3 px.py birdview 320 282 312 282 328 282 320 270 320 295

# openrua op 21
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helper: joint state, FK, IK, trajectory, gripper. Import or run.

python3 arm.py ik x y z qx qy qz qw          -> print IK solution (no motion)
python3 arm.py goto x y z qx qy qz qw [sec]  -> IK + trajectory for TCP pose (world)
python3 arm.py joints p1,...,p7 [sec]        -> trajectory
python3 arm.py grip open|close
python3 arm.py state                         -> joints + hand/tcp world pose
"""
import sys, time
import numpy as np
import rclpy, yaml
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


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli.wait_for_service(timeout_sec=20)
        self.ik_cli.wait_for_service(timeout_sec=20)
        self.traj.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def _seed(self, q):
        s = JointState(); s.name = list(ARM); s.position = [float(v) for v in q]
        return s

    def fk(self, q=None):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        R = quat_to_R(*quat)
        return pos, quat, pos + TCP * R[:, 2]

    def ik(self, pos, quat, at_tcp=True, seed=None):
        pos = np.array(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(*quat)[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed if seed is not None else self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, pos, quat, seconds=3.0, at_tcp=True):
        q = self.ik(pos, quat, at_tcp=at_tcp)
        code, err = self.move_joints(q, seconds)
        _, _, tcp = self.fk()
        print("tcp now", tcp.round(4), "target", np.array(pos).round(4))
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand z down, hand y = -world y (closing axis along y)


def main():
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "state":
        q = a.arm_q(); pos, quat, tcp = a.fk(q)
        print("q", np.round(q, 4).tolist()); print("hand", pos.round(4), quat.round(4)); print("tcp", tcp.round(4)); print("fingers", a.fingers())
    elif cmd == "ik":
        v = list(map(float, sys.argv[2:9]))
        print(np.round(a.ik(v[:3], v[3:]), 4).tolist())
    elif cmd == "goto":
        v = list(map(float, sys.argv[2:9])); sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        a.goto(v[:3], v[3:], sec)
    elif cmd == "joints":
        q = list(map(float, sys.argv[2].split(","))); sec = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
        a.move_joints(q, sec)
    elif cmd == "grip":
        a.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 22
timeout 200 python3 arm.py state && echo "--- IK for current hand pose expressed in WORLD (expect ~current joints):" && timeout 120 python3 arm.py ik -0.0589 0 0.6746 0.9996 0 -0.0284 0 && echo "--- IK for same pose in BASE frame coordinates:" && timeout 120 python3 arm.py ik 0.4511 0 0.2544 0.9996 0 -0.0284 0

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand z down, hand y = -world y (closing axis along y)", "new_string": "# IK poses are for panda_link8 (= panda_hand rotated +45deg about z), in WORLD frame.\\n# Both point z straight down; they differ in which world axis the fingers close along.\\nDOWN_Y = np.array([0.92388, -0.38268, 0.0, 0.0])  # fingers close along world y\\nDOWN_X = np.array([0.38268, -0.92388, 0.0, 0.0])  # fingers close along world x", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
timeout 120 python3 arm.py ik -0.0589 0 0.6746 0.92388 -0.38268 0 0 && echo "--- above red mug rim grasp point, tcp z=0.62:" && timeout 120 python3 arm.py ik -0.223 0.066 0.62 0.92388 -0.38268 0 0 && echo "--- above plate:" && timeout 120 python3 arm.py ik 0.123 -0.005 0.62 0.92388 -0.38268 0 0 && echo "--- pudding (close along y):" && timeout 120 python3 arm.py ik -0.041 0.114 0.55 0.92388 -0.38268 0 0

# openrua op 25
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
h = 3.0-d
# rim pixels of red mug: high points in bbox
vs,us = np.where(h>0.50); sel = (us>300)&(us<345)&(vs>220)&(vs<250)
us,vs = us[sel],vs[sel]; z=d[vs,us]
X=(us-cx)*z/fx; Y=(vs-cy)*z/fy
P=(T[:3,:3]@np.vstack([X,Y,z])+T[:3,3:4])
x,y=P[0],P[1]
# algebraic circle fit
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('red mug rim: center (%.4f, %.4f) r=%.4f n=%d, zmax=%.3f'%(c[0],c[1],r,len(x),P[2].max()))
print('rim y range', y.min(), y.max(), 'x range', x.min(), x.max())
# white mug too
vs,us = np.where(h>0.50); sel = (us>270)&(us<310)&(vs>245)&(vs<275)
us,vs = us[sel],vs[sel]; z=d[vs,us]
X=(us-cx)*z/fx; Y=(vs-cy)*z/fy
P=(T[:3,:3]@np.vstack([X,Y,z])+T[:3,3:4]); x,y=P[0],P[1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('white mug rim: center (%.4f, %.4f) r=%.4f'%(c[0],c[1],r))
"


# openrua op 26
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
h = 3.0-d
def fit(ub,vb,lo=0.50,hi=0.60):
    vs,us = np.where((h>lo)&(h<hi)); sel = (us>ub[0])&(us<ub[1])&(vs>vb[0])&(vs<vb[1])
    us,vs = us[sel],vs[sel]; z=d[vs,us]
    X=(us-cx)*z/fx; Y=(vs-cy)*z/fy
    P=(T[:3,:3]@np.vstack([X,Y,z])+T[:3,3:4]); x,y=P[0],P[1]
    A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print('center (%.4f, %.4f) r=%.4f n=%d zmax=%.3f  x[%.3f,%.3f] y[%.3f,%.3f]'%(c[0],c[1],r,len(x),P[2].max(),x.min(),x.max(),y.min(),y.max()))
print('red  :',end=' '); fit((300,345),(220,250))
print('white:',end=' '); fit((270,310),(245,275))
"


# openrua op 27
mkdir -p "$(dirname /workspace/pick_mug.py)"
cat > /workspace/pick_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-grasp the red mug: hover -> descend -> close -> lift, verifying each step."""
import numpy as np, rclpy
from arm import Arm, DOWN_Y

GRASP = np.array([-0.223, 0.067])   # +y rim of the red mug (world)
RIM_Z = 0.547
HOVER_Z = 0.65
GRASP_Z = RIM_Z - 0.03

a = Arm()
print("fingers before", a.fingers())
print("== hover")
a.goto([*GRASP, HOVER_Z], DOWN_Y, seconds=4.0)
print("== descend")
a.goto([*GRASP, GRASP_Z], DOWN_Y, seconds=3.0)
print("== close")
f = a.gripper(0.0)
print("== lift")
a.goto([*GRASP, HOVER_Z], DOWN_Y, seconds=3.0)
print("fingers after lift", a.fingers())
rclpy.shutdown()
OPENRUA_EOF

# openrua op 28
timeout 900 python3 -u pick_mug.py 2>&1 | tee pick_mug.log

# openrua op 29
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 30
timeout 120 python3 locate.py agentview >/dev/null && python3 -c "
import numpy as np
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
# points near the mug's xy footprint
m = (P[0]>-0.30)&(P[0]<-0.15)&(P[1]>-0.03)&(P[1]<0.09)&(P[2]>0.43)&(P[2]<0.75)
print('object points in mug column: z min %.3f max %.3f, n=%d'%(P[2][m].min(), P[2][m].max(), m.sum()))
# is there anything within 3cm of the table under the mug?
low = m & (P[2]<0.50); print('points below z=0.50 in mug column:', low.sum())
"

# openrua op 31
python3 -c "
import numpy as np
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
m = (P[0]>-0.32)&(P[0]<-0.12)&(P[1]>-0.05)&(P[1]<0.12)&(P[2]>0.54)&(P[2]<0.70)
x,y,zz = P[0][m],P[1][m],P[2][m]
print('mug body x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(x.min(),x.max(),y.min(),y.max(),zz.min(),zz.max()))
# the camera sees the front (+x) face of the mug; the front-most x at each y gives the silhouette
# estimate center y from y-extent of the body (excluding the handle at -y): take points with z in 0.56..0.62 (below the rim, above bottom)
s = (zz>0.56)&(zz<0.62)
print('body band y[%.3f,%.3f] x[%.3f,%.3f]'%(y[s].min(),y[s].max(),x[s].min(),x[s].max()))
"
timeout 100 python3 arm.py state

# openrua op 32
mkdir -p "$(dirname /workspace/place_mug.py)"
cat > /workspace/place_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the held mug over the plate, lower it onto the plate, release, retreat."""
import numpy as np, rclpy
from arm import Arm, DOWN_Y

PLATE = np.array([0.123, -0.005])
PLATE_TOP = 0.444
MUG_OFF = np.array([0.0, -0.036])   # mug center relative to TCP (measured)
BOTTOM_BELOW_TCP = 0.1016           # mug bottom = tcp z - this (measured)

tcp_xy = PLATE - MUG_OFF
place_z = PLATE_TOP + BOTTOM_BELOW_TCP + 0.003

a = Arm()
print("fingers", a.fingers())
print("== up to transport height")
a.goto([-0.223, 0.067, 0.72], DOWN_Y, seconds=2.5)
print("== via midpoint")
a.goto([-0.05, 0.05, 0.74], DOWN_Y, seconds=3.0)
print("== hover over plate")
a.goto([*tcp_xy, 0.72], DOWN_Y, seconds=3.0)
print("fingers", a.fingers())
print("== lower onto plate, target tcp z", round(place_z, 4))
a.goto([*tcp_xy, place_z], DOWN_Y, seconds=3.5)
print("== release")
a.gripper(0.04)
print("== retreat")
a.goto([*tcp_xy, 0.70], DOWN_Y, seconds=3.0)
rclpy.shutdown()
OPENRUA_EOF

# openrua op 33
timeout 1200 python3 -u place_mug.py 2>&1 | tee place_mug.log

# openrua op 34
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 locate.py birdview

# openrua op 35
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
h = 3.0-d
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
# red mug rim near plate: points z in [0.50,0.60] within plate neighborhood
m = (P[0]>0.0)&(P[0]<0.25)&(P[1]>-0.12)&(P[1]<0.12)&(P[2]>0.50)&(P[2]<0.60)
x,y,zz=P[0][m],P[1][m],P[2][m]
print('red mug rim: x[%.3f,%.3f] y[%.3f,%.3f] zmax=%.3f n=%d'%(x.min(),x.max(),y.min(),y.max(),zz.max(),len(x)))
print('  body center est: (%.3f, %.3f)'%((x.min()+x.max())/2, y.max()-0.036))
# plate (z 0.435..0.46) in the same area
m2 = (P[0]>0.0)&(P[0]<0.25)&(P[1]>-0.12)&(P[1]<0.12)&(P[2]>0.435)&(P[2]<0.47)
x2,y2=P[0][m2],P[1][m2]
print('plate visible: x[%.3f,%.3f] y[%.3f,%.3f]'%(x2.min(),x2.max(),y2.min(),y2.max()))
# pudding
m3 = (P[0]>-0.15)&(P[0]<0.05)&(P[1]>0.05)&(P[1]<0.2)&(P[2]>0.44)&(P[2]<0.50)
x3,y3,z3=P[0][m3],P[1][m3],P[2][m3]
print('pudding: x[%.3f,%.3f] y[%.3f,%.3f] zmax=%.3f center (%.3f,%.3f) n=%d'%(x3.min(),x3.max(),y3.min(),y3.max(),z3.max(),(x3.min()+x3.max())/2,(y3.min()+y3.max())/2,len(x3)))
"


# openrua op 36
timeout 300 python3 arm.py goto -0.15 0.0 0.75 0.92388 -0.38268 0 0 3 && timeout 120 python3 locate.py birdview && timeout 120 python3 locate.py agentview && python3 -c "
import numpy as np
for cam in ['birdview','agentview']:
    d = np.load(cam+'_depth.npy'); meta = np.load(cam+'_meta.npy', allow_pickle=True).item()
    K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
    H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
    P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
    m = (P[0]>0.0)&(P[0]<0.25)&(P[1]>-0.12)&(P[1]<0.12)&(P[2]>0.47)&(P[2]<0.65)
    x,y,zz=P[0][m],P[1][m],P[2][m]
    print(cam,'red mug: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] n=%d'%(x.min(),x.max(),y.min(),y.max(),zz.min(),zz.max(),len(x)))
    top = zz>zz.max()-0.015
    print('   rim ring center (%.3f,%.3f)'%(x[top].mean(), y[top].mean()), 'rim z %.3f'%zz.max())
    m2 = (P[0]>0.0)&(P[0]<0.25)&(P[1]>-0.12)&(P[1]<0.12)&(P[2]>0.435)&(P[2]<0.465)
    x2,y2=P[0][m2],P[1][m2]
    print('   plate visible: x[%.3f,%.3f] y[%.3f,%.3f] center (%.3f,%.3f)'%(x2.min(),x2.max(),y2.min(),y2.max(),(x2.min()+x2.max())/2,(y2.min()+y2.max())/2))
"

# openrua op 37
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
m = (P[0]>0.0)&(P[0]<0.25)&(P[1]>-0.12)&(P[1]<0.12)&(P[2]>0.50)&(P[2]<0.65)
x,y,zz=P[0][m],P[1][m],P[2][m]
cx_,cy_=0.115,-0.002
ang = np.degrees(np.arctan2(y-cy_,x-cx_)); r=np.hypot(x-cx_,y-cy_)
rim = r>0.028
for a0 in range(-180,180,45):
    s = rim&(ang>=a0)&(ang<a0+45)
    if s.sum(): print('angle %4d..%4d: n=%3d z max %.3f mean %.3f r mean %.3f'%(a0,a0+45,s.sum(),zz[s].max(),zz[s].mean(),r[s].mean()))
# interior floor
inner = r<0.02
print('interior: n',inner.sum(), 'z', zz[inner].min() if inner.sum() else None)
m2 = (P[0]>0.08)&(P[0]<0.15)&(P[1]>-0.03)&(P[1]<0.03)
print('all z in mug center column: min %.3f max %.3f'%(P[2][m2].min(),P[2][m2].max()))
"; timeout 100 python3 arm.py state

# openrua op 38
python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    d = np.load(cam+'_depth.npy'); meta = np.load(cam+'_meta.npy', allow_pickle=True).item()
    K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
    H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
    P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
    m = (P[0]>-0.15)&(P[0]<0.05)&(P[1]>0.05)&(P[1]<0.2)&(P[2]>0.435)&(P[2]<0.50)
    x,y,zz=P[0][m],P[1][m],P[2][m]
    print(cam,'pudding all: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] n=%d'%(x.min(),x.max(),y.min(),y.max(),zz.min(),zz.max(),len(x)))
    t = zz>zz.max()-0.008
    pts=np.c_[x[t],y[t]]; c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c)
    ax=vt[0]; ang=np.degrees(np.arctan2(ax[1],ax[0]))
    proj=(pts-c)@vt.T
    print('   top face: center (%.3f,%.3f) axis angle %.1f deg, extents along/across: %.3f / %.3f, n=%d'%(c[0],c[1],ang,proj[:,0].max()-proj[:,0].min(),proj[:,1].max()-proj[:,1].min(),len(pts)))
"


# openrua op 39
mkdir -p "$(dirname /workspace/move_pudding.py)"
cat > /workspace/move_pudding.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the chocolate pudding (grasp across y) and place it to the +y side of the plate."""
import numpy as np, rclpy
from arm import Arm, DOWN_Y

PUD = np.array([-0.045, 0.115])
TABLE = 0.425
GRASP_Z = TABLE + 0.012          # fingertips 1.2 cm above the table
CARRY_Z = 0.62
PLATE = np.array([0.121, -0.004])
TARGET = np.array([PLATE[0], PLATE[1] + 0.12])   # pudding center, right (+y) of the plate
PLACE_Z = GRASP_Z + 0.005

a = Arm()
a.gripper(0.04)
print("== hover over pudding")
a.goto([*PUD, 0.58], DOWN_Y, seconds=4.0)
print("== descend")
a.goto([*PUD, GRASP_Z], DOWN_Y, seconds=3.0)
print("== close")
a.gripper(0.0)
print("== lift")
a.goto([*PUD, CARRY_Z], DOWN_Y, seconds=3.0)
print("fingers after lift", a.fingers())
print("== carry to target")
a.goto([*TARGET, CARRY_Z], DOWN_Y, seconds=3.5)
print("== lower")
a.goto([*TARGET, PLACE_Z], DOWN_Y, seconds=3.0)
print("== release")
a.gripper(0.04)
print("== retreat")
a.goto([*TARGET, 0.65], DOWN_Y, seconds=3.0)
rclpy.shutdown()
OPENRUA_EOF

# openrua op 40
timeout 1500 python3 -u move_pudding.py 2>&1 | tee move_pudding.log

# openrua op 41
timeout 120 python3 locate.py agentview && timeout 120 python3 locate.py birdview && python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    d = np.load(cam+'_depth.npy'); meta = np.load(cam+'_meta.npy', allow_pickle=True).item()
    K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
    H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
    P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
    m = (P[0]>0.0)&(P[0]<0.35)&(P[1]>0.06)&(P[1]<0.30)&(P[2]>0.435)&(P[2]<0.52)
    if m.sum()==0: print(cam,'nothing'); continue
    x,y,zz=P[0][m],P[1][m],P[2][m]
    print(cam,'pudding?: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] n=%d'%(x.min(),x.max(),y.min(),y.max(),zz.min(),zz.max(),len(x)))
    t = zz>zz.max()-0.008
    pts=np.c_[x[t],y[t]]; c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c); ax=vt[0]
    proj=(pts-c)@vt.T
    print('   top face: center (%.3f,%.3f) angle %.1f deg extents %.3f / %.3f'%(c[0],c[1],np.degrees(np.arctan2(ax[1],ax[0])),proj[:,0].max()-proj[:,0].min(),proj[:,1].max()-proj[:,1].min()))
"

# openrua op 42
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def goto(self, pos, quat, seconds=3.0, at_tcp=True):\\n        q = self.ik(pos, quat, at_tcp=at_tcp)\\n        code, err = self.move_joints(q, seconds)\\n        _, _, tcp = self.fk()\\n        print(\\"tcp now\\", tcp.round(4), \\"target\\", np.array(pos).round(4))\\n        return tcp", "new_string": "    def ik_near(self, pos, quat, at_tcp=True, max_jump=0.8, tries=8):\\n        \\"\\"\\"IK solution closest (in joint space) to the current configuration.\\n        Retries the (randomised) solver; raises if every solution jumps branches.\\"\\"\\"\\n        cur = np.array(self.arm_q())\\n        best, best_d = None, np.inf\\n        for _ in range(tries):\\n            try:\\n                q = np.array(self.ik(pos, quat, at_tcp=at_tcp, seed=cur))\\n            except RuntimeError:\\n                continue\\n            d = np.abs(q - cur).max()\\n            if d < best_d:\\n                best, best_d = q, d\\n            if d < max_jump:\\n                break\\n        if best is None:\\n            raise RuntimeError(\\"IK failed\\")\\n        if best_d >= max_jump:\\n            raise RuntimeError(f\\"IK only found far solutions (max joint jump {best_d:.2f} rad)\\")\\n        return best.tolist(), best_d\\n\\n    def goto(self, pos, quat, seconds=3.0, at_tcp=True, max_jump=0.8, tol=0.01):\\n        q, jump = self.ik_near(pos, quat, at_tcp=at_tcp, max_jump=max_jump)\\n        seconds = max(seconds, jump / 0.4)  # never faster than ~0.4 rad/s on the biggest joint\\n        for attempt in range(3):\\n            code, err = self.move_joints(q, seconds)\\n            if err < tol:\\n                break\\n            print(f\\"  not converged (err {err:.3f}); re-sending\\")\\n            seconds = max(2.0, seconds * 0.7)\\n        _, _, tcp = self.fk()\\n        print(\\"tcp now\\", tcp.round(4), \\"target\\", np.array(pos).round(4), f\\"jump={jump:.2f}\\")\\n        if err >= tol:\\n            raise RuntimeError(f\\"move did not converge (joint err {err:.3f})\\")\\n        return tcp", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 43
python3 -c "
import numpy as np, cv2
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
m = (P[0]>0.0)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.435)&(P[2]<0.60)
x,y,zz=P[0][m],P[1][m],P[2][m]
print('all objects near plate: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(x.min(),x.max(),y.min(),y.max(),zz.min(),zz.max()))
# histogram of heights
hist,edges=np.histogram(zz,bins=np.arange(0.43,0.61,0.01))
for h_,e in zip(hist,edges): print('  z %.2f: %d'%(e,h_))
img = cv2.imread('birdview_color.png'); crop = img[250:360, 280:420]
cv2.imwrite('bird_crop2.png', cv2.resize(crop, None, fx=5, fy=5, interpolation=cv2.INTER_NEAREST))
hm = np.clip((P[2]-0.43)/0.17*255,0,255).astype(np.uint8)[250:360,280:420]
cv2.imwrite('bird_h2.png', cv2.resize(hm, None, fx=5, fy=5, interpolation=cv2.INTER_NEAREST))
"

# openrua op 44
timeout 600 python3 arm.py joints 0,-0.161,0,-2.4446,0,2.2268,0.7854 5 && timeout 120 python3 locate.py birdview && timeout 120 python3 locate.py agentview && python3 -c "
import numpy as np, cv2
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
img = cv2.imread('birdview_color.png'); crop = img[250:360, 280:420]
cv2.imwrite('bird_crop2.png', cv2.resize(crop, None, fx=5, fy=5, interpolation=cv2.INTER_NEAREST))
hm = np.clip((P[2]-0.43)/0.12*255,0,255).astype(np.uint8)[250:360,280:420]
cv2.imwrite('bird_h2.png', cv2.resize(hm, None, fx=5, fy=5, interpolation=cv2.INTER_NEAREST))
m = (P[0]>0.0)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.435)&(P[2]<0.60)
print('objects near plate z max %.3f'%P[2][m].max())
"

# openrua op 45
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
# mug: red-ish region south-east of plate. Use height 0.47..0.53 and x>0.05,y>-0.1
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.10)&(P[1]<0.30)&(P[2]>0.468)&(P[2]<0.535)
x,y,zz=P[0][m],P[1][m],P[2][m]
pts=np.c_[x,y]
# split into mug and pudding by clustering on position: pudding near (0.19,0.16)
import cv2
lab = np.zeros(len(pts),int)
# simple: distance to pudding guess
dp = np.hypot(x-0.19,y-0.16)
for name,sel in [('mug',dp>0.045),('pudding',dp<=0.045)]:
    p=pts[sel]; c=p.mean(0); u,s,vt=np.linalg.svd(p-c); ax=vt[0]
    proj=(p-c)@vt.T
    print(name,'center (%.3f,%.3f) axis (%.2f,%.2f) ang %.1f  along [%.3f,%.3f] across [%.3f,%.3f] zmax %.3f n=%d'%(c[0],c[1],ax[0],ax[1],np.degrees(np.arctan2(ax[1],ax[0])),proj[:,0].min(),proj[:,0].max(),proj[:,1].min(),proj[:,1].max(),zz[sel].max(),sel.sum()))
    if name=='mug':
        # width across at slices along axis
        for a0 in np.arange(proj[:,0].min(),proj[:,0].max(),0.01):
            s=(proj[:,0]>=a0)&(proj[:,0]<a0+0.01)
            if s.sum(): print('   along %.3f: across [%.3f,%.3f] width %.3f zmax %.3f n=%d'%(a0,proj[s,1].min(),proj[s,1].max(),proj[s,1].max()-proj[s,1].min(),zz[sel][s].max(),s.sum()))
"

# openrua op 46
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
def report(name, m):
    x,y,zz=P[0][m],P[1][m],P[2][m]; pts=np.c_[x,y]; c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c); ax=vt[0]
    if ax[0]<0: ax=-ax; vt[0]=ax
    proj=(pts-c)@vt.T
    print(name,'center (%.3f,%.3f) axis (%.2f,%.2f) ang %.1f along [%.3f,%.3f] across [%.3f,%.3f] z[%.3f,%.3f] n=%d'%(c[0],c[1],ax[0],ax[1],np.degrees(np.arctan2(ax[1],ax[0])),proj[:,0].min(),proj[:,0].max(),proj[:,1].min(),proj[:,1].max(),zz.min(),zz.max(),len(x)))
    return c, vt, proj, zz
# mug body top surface (exclude handle which is lower)
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.10)&(P[1]<0.30)&(P[2]>0.50)&(P[2]<0.535)&~((P[0]>0.17)&(P[0]<0.27)&(P[1]>0.08)&(P[1]<0.18))
c,vt,proj,zz = report('mug body top', m)
for a0 in np.arange(proj[:,0].min(),proj[:,0].max(),0.01):
    s=(proj[:,0]>=a0)&(proj[:,0]<a0+0.01)
    if s.sum(): print('   along %.3f: across [%.3f,%.3f] w %.3f zmax %.3f z at peak? n=%d'%(a0,proj[s,1].min(),proj[s,1].max(),proj[s,1].max()-proj[s,1].min(),zz[s].max(),s.sum()))
m2 = (P[0]>0.17)&(P[0]<0.27)&(P[1]>0.08)&(P[1]<0.18)&(P[2]>0.44)&(P[2]<0.52)
report('pudding', m2)
m3 = (P[0]>0.17)&(P[0]<0.27)&(P[1]>0.08)&(P[1]<0.18)&(P[2]>0.49)&(P[2]<0.52)
report('pudding top', m3)
"

# openrua op 47
python3 -c "
import cv2
img = cv2.imread('agentview_color.png'); crop = img[300:480, 220:560]
cv2.imwrite('agent_crop2.png', cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
"

# openrua op 48
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
c=np.array([0.204,0.040]); u=np.array([0.82,0.57]); u/=np.linalg.norm(u); v=np.array([-u[1],u[0]])
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.472)&(P[2]<0.535)
x,y,zz=P[0][m],P[1][m],P[2][m]
al=(x-c[0])*u[0]+(y-c[1])*u[1]; ac=(x-c[0])*v[0]+(y-c[1])*v[1]
# exclude pudding region
pud=(x>0.17)&(x<0.27)&(y>0.09)&(y<0.19)&(al>0.02)
print('along  across_min across_max width  zmax  (silhouette z>0.472)')
for a0 in np.arange(-0.10,0.10,0.01):
    s=(al>=a0)&(al<a0+0.01)&~pud
    if s.sum(): print('%.3f  %.3f %.3f  %.3f  %.3f n=%d'%(a0,ac[s].min(),ac[s].max(),ac[s].max()-ac[s].min(),zz[s].max(),s.sum()))
"

# openrua op 49
python3 -c "
import numpy as np
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
c=np.array([0.204,0.040]); u=np.array([0.82,0.57]); u/=np.linalg.norm(u); v=np.array([-u[1],u[0]])
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.445)&(P[2]<0.535)
x,y,zz=P[0][m],P[1][m],P[2][m]
al=(x-c[0])*u[0]+(y-c[1])*u[1]; ac=(x-c[0])*v[0]+(y-c[1])*v[1]
pud=(x>0.17)&(x<0.30)&(y>0.09)&(y<0.19)&(al>0.03)
def circfit(a,b):
    A=np.c_[2*a,2*b,np.ones_like(a)]; rhs=a**2+b**2
    s=np.linalg.lstsq(A,rhs,rcond=None)[0]; r=np.sqrt(s[2]+s[0]**2+s[1]**2); return s[0],s[1],r
print('along: n  across_c  z_c  radius   | across range, z range')
for a0 in np.arange(-0.08,0.10,0.01):
    s=(al>=a0)&(al<a0+0.01)&~pud
    if s.sum()>20:
        # exclude handle: keep points with across > -0.03 unless z high
        a,b=ac[s],zz[s]
        keep=(a>-0.045)
        ca,cb,r=circfit(a[keep],b[keep])
        print('%.3f: n=%3d  %.3f  %.3f  r=%.3f | ac[%.3f,%.3f] z[%.3f,%.3f]'%(a0,s.sum(),ca,cb,r,a.min(),a.max(),b.min(),b.max()))
"

# openrua op 50
python3 -c "
import numpy as np
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
c=np.array([0.204,0.040]); u=np.array([0.82,0.57]); u/=np.linalg.norm(u); v=np.array([-u[1],u[0]])
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.44)&(P[2]<0.535)
x,y,zz=P[0][m],P[1][m],P[2][m]
al=(x-c[0])*u[0]+(y-c[1])*u[1]; ac=(x-c[0])*v[0]+(y-c[1])*v[1]
# rim: the mug's near end. Take the most +along points: for each (across,z) bin, the rim is where along is max.
# Simpler: histogram of along for points with across in [-0.05,0.05]
s=(ac>-0.06)&(ac<0.06)
h,e=np.histogram(al[s],bins=np.arange(-0.10,0.12,0.005))
for hh,ee in zip(h,e): print('along %.3f: %d'%(ee,hh))
"

# openrua op 51
python3 -c "
import numpy as np
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
c=np.array([0.204,0.040]); u=np.array([0.82,0.57]); u/=np.linalg.norm(u); v=np.array([-u[1],u[0]])
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.44)&(P[2]<0.535)
x,y,zz=P[0][m],P[1][m],P[2][m]
al=(x-c[0])*u[0]+(y-c[1])*u[1]; ac=(x-c[0])*v[0]+(y-c[1])*v[1]
s=(al>0.045)&(al<0.095)&(ac>-0.06)&(ac<0.06)
Q=np.c_[x[s],y[s],zz[s]]; cm=Q.mean(0); U,S,Vt=np.linalg.svd(Q-cm)
n=Vt[2]; print('rim plane normal (mug axis):', n.round(3), 'sv', (S/np.sqrt(len(Q))).round(4), 'n pts', len(Q))
# in-plane coords
e1,e2=Vt[0],Vt[1]; a=(Q-cm)@e1; b=(Q-cm)@e2
A=np.c_[2*a,2*b,np.ones_like(a)]; rhs=a**2+b**2; sol=np.linalg.lstsq(A,rhs,rcond=None)[0]; r=np.sqrt(sol[2]+sol[0]**2+sol[1]**2)
center=cm+sol[0]*e1+sol[1]*e2
print('rim circle center', center.round(3), 'radius %.4f'%r)
res=np.hypot(a-sol[0],b-sol[1])-r; print('radial residual rms %.4f'%np.sqrt((res**2).mean()))
np.save('mug_axis.npy', {'center':center,'axis':n,'r':r}, allow_pickle=True)
"

# openrua op 52
python3 -c "
import cv2, numpy as np
img = cv2.imread('bird_crop.png')  # 4x crop of birdview[200:360,240:400], mug upright
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# red mask
m = ((hsv[...,0]<12)|(hsv[...,0]>168)) & (hsv[...,1]>80) & (hsv[...,2]>60)
m = m.astype(np.uint8)
# restrict to red mug region around (345,140)
sub = np.zeros_like(m); sub[60:220, 260:430] = m[60:220, 260:430]
ys,xs = np.where(sub)
print('red px bbox: x[%d,%d] y[%d,%d]'%(xs.min(),xs.max(),ys.min(),ys.max()))
# the handle is on the left (-y). Rim ring: rows where the red is annular. Measure width per row
for r in range(ys.min(), ys.max()+1, 5):
    row = np.where(sub[r])[0]
    if len(row): print('row %d: x[%d,%d] width %d'%(r,row.min(),row.max(),row.max()-row.min()))
" ; python3 -c "
import numpy as np
# birdview pixel size at mug rim depth 2.45m
print('mm per orig px', 2.45/579.41*1000, ' per crop px', 2.45/579.41*1000/4)
"

# openrua op 53
python3 -c "
import numpy as np
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
cam = T[:3,3]
c=np.array([0.204,0.040,0.484]); u=np.array([0.844,0.527,-0.105]); u/=np.linalg.norm(u)
view = c-cam; view/=np.linalg.norm(view)
w = np.cross(u, view); w/=np.linalg.norm(w)
print('cam',cam.round(3),'view',view.round(3),'w',w.round(3))
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.44)&(P[2]<0.535)
Q=np.c_[P[0][m],P[1][m],P[2][m]]-c
al=Q@u; pw=Q@w
v=np.array([-0.527,0.844,0]); ac=Q@v
sel=(ac<0.06)  # drop pudding side
for a0 in np.arange(-0.06,0.08,0.01):
    s=(al>=a0)&(al<a0+0.01)&sel
    if s.sum()>30:
        # robust extent: 1st/99th percentiles
        lo,hi=np.percentile(pw[s],[0.5,99.5])
        print('along %.3f: silhouette width %.4f  (n=%d) w-range[%.3f,%.3f]'%(a0,hi-lo,s.sum(),lo,hi))
"

# openrua op 54
python3 -c "
import cv2, numpy as np
img = cv2.imread('agent_crop.png')  # 2x crop of agentview[120:420,150:470], mug upright
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# red mug region approx x[300,460] y[30,230] in crop
sub = img[20:240, 280:470]
h = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
red = (((h[...,0]<10)|(h[...,0]>170)) & (h[...,1]>120) & (h[...,2]>80)).astype(np.uint8)
# also white pattern on mug: high V low S -> but table is brown; mug white pattern S<60,V>150
white = ((h[...,1]<70)&(h[...,2]>150)).astype(np.uint8)
mug = cv2.morphologyEx(red|white, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
n,lab,st,cen = cv2.connectedComponentsWithStats(mug)
i = 1+np.argmax(st[1:,4]); comp=(lab==i)
ys,xs=np.where(comp); print('mug comp bbox x[%d,%d] y[%d,%d]'%(xs.min(),xs.max(),ys.min(),ys.max()))
for r in range(ys.min(), ys.max()+1, 10):
    row=np.where(comp[r])[0]
    if len(row): print('row %3d: x[%3d,%3d] width %3d px'%(r,row.min(),row.max(),row.max()-row.min()+1))
cv2.imwrite('mug_mask.png', (comp*255).astype(np.uint8))
"

# openrua op 55
python3 -c "
import numpy as np
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
c=np.array([0.204,0.040,0.484]); u=np.array([0.844,0.527,-0.105]); u/=np.linalg.norm(u); v=np.array([-0.527,0.844,0]); 
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.43)&(P[2]<0.535)
Q=np.c_[P[0][m],P[1][m],P[2][m]]-c
al=Q@u; ac=Q@v; zz=Q[:,2]+c[2]
# handle candidates: across < -0.038 (beyond body), not plate (plate is at along<-0.06 or z<0.47 near plate)
s=(ac<-0.036)&(ac>-0.08)&(al>-0.07)&(al<0.09)
print('points beyond -across body edge: n',s.sum())
for a0 in np.arange(-0.07,0.09,0.01):
    t=s&(al>=a0)&(al<a0+0.01)
    if t.sum(): print('along %.3f: n=%3d across[%.3f,%.3f] z[%.3f,%.3f]'%(a0,t.sum(),ac[t].min(),ac[t].max(),zz[t].min(),zz[t].max()))
print('--- +across side beyond body edge (pudding etc):')
s=(ac>0.036)&(ac<0.12)&(al>-0.07)&(al<0.09)
for a0 in np.arange(-0.07,0.09,0.01):
    t=s&(al>=a0)&(al<a0+0.01)
    if t.sum(): print('along %.3f: n=%3d across[%.3f,%.3f] z[%.3f,%.3f]'%(a0,t.sum(),ac[t].min(),ac[t].max(),zz[t].min(),zz[t].max()))
"

# openrua op 56
python3 -c "
import numpy as np, cv2
d = np.load('agentview_depth.npy'); meta = np.load('agentview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
c=np.array([0.204,0.040,0.484]); u=np.array([0.844,0.527,-0.105]); u/=np.linalg.norm(u); v=np.array([-0.527,0.844,0]); v-= (v@u)*u; v/=np.linalg.norm(v); wz=np.cross(u,v)
m = (P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.15)&(P[1]<0.30)&(P[2]>0.436)&(P[2]<0.535)
Q=np.c_[P[0][m],P[1][m],P[2][m]]-c
al=Q@u; ac=Q@v; zz=Q@wz
s=(al>0.045)&(al<0.095)&(ac<0.03)&(ac>-0.07)
pts=np.c_[ac[s],zz[s]].astype(np.float32)
hull=cv2.convexHull(pts).reshape(-1,2)
# keep hull points on the -across side and top/bottom only (across<0.03 already)
a,b=hull[:,0],hull[:,1]
A=np.c_[2*a,2*b,np.ones_like(a)]; rhs=a**2+b**2; sol=np.linalg.lstsq(A,rhs,rcond=None)[0]; r=np.sqrt(sol[2]+sol[0]**2+sol[1]**2)
print('hull pts',len(hull),'circle center (across %.4f, z %.4f) r=%.4f'%(sol[0],sol[1],r))
print('hull across range [%.3f,%.3f] z range [%.3f,%.3f]'%(a.min(),a.max(),b.min(),b.max()))
res=np.hypot(a-sol[0],b-sol[1])-r; print('residuals', res.round(3))
"

# openrua op 57
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
m = (P[0]>0.15)&(P[0]<0.30)&(P[1]>0.07)&(P[1]<0.20)&(P[2]>0.49)&(P[2]<0.515)
x,y,zz=P[0][m],P[1][m],P[2][m]
print('n',len(x),'z[%.3f,%.3f]'%(zz.min(),zz.max()))
pts=np.c_[x,y]; c=pts.mean(0); u_,s,vt=np.linalg.svd(pts-c); ax=vt[0]
proj=(pts-c)@vt.T
print('top face center (%.4f,%.4f) axis (%.2f,%.2f) ang %.1f  extents %.3f x %.3f'%(c[0],c[1],ax[0],ax[1],np.degrees(np.arctan2(ax[1],ax[0])),proj[:,0].max()-proj[:,0].min(),proj[:,1].max()-proj[:,1].min()))
print('x[%.3f,%.3f] y[%.3f,%.3f]'%(x.min(),x.max(),y.min(),y.max()))
"


# openrua op 58
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
m = (P[0]>0.12)&(P[0]<0.32)&(P[1]>0.04)&(P[1]<0.22)&(P[2]>0.44)
x,y,zz=P[0][m],P[1][m],P[2][m]
# print a coarse height map: rows = x bins (1cm), cols = y bins (1cm)
xb=np.arange(0.12,0.32,0.01); yb=np.arange(0.04,0.22,0.01)
print('     y:'+' '.join('%4d'%int(v*100) for v in yb[:-1]))
for i in range(len(xb)-1):
    row=[]
    for j in range(len(yb)-1):
        s=(x>=xb[i])&(x<xb[i+1])&(y>=yb[j])&(y<yb[j+1])
        row.append('%4d'%int((zz[s].max()-0.4)*1000) if s.any() else '   .')
    print('x=%3d '%int(xb[i]*100)+' '.join(row))
"


# openrua op 59
find / -iname "*panda*gripper*.xml" -o -iname "*hand*.xml" -path "*panda*" 2>/dev/null | head; find / -iname "*panda_hand*" 2>/dev/null | head

# openrua op 60
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
m = (P[0]>-0.20)&(P[0]<0.10)&(P[1]>-0.15)&(P[1]<0.15)&(P[2]>0.60)
x,y,zz=P[0][m],P[1][m],P[2][m]
xb=np.arange(-0.20,0.10,0.01); yb=np.arange(-0.15,0.15,0.01)
print('      y:'+' '.join('%3d'%int(round(v*100)) for v in yb[:-1]))
for i in range(len(xb)-1):
    row=[]
    for j in range(len(yb)-1):
        s=(x>=xb[i])&(x<xb[i+1])&(y>=yb[j])&(y<yb[j+1])
        row.append('%3d'%int((zz[s].min()-0.6)*100) if s.any() else '  .')
    print('x=%4d '%int(round(xb[i]*100))+' '.join(row))
print('fingers region: z in [0.67,0.72]:'); s=(zz>0.66)&(zz<0.72); print('x[%.3f,%.3f] y[%.3f,%.3f]'%(x[s].min(),x[s].max(),y[s].min(),y[s].max()))
s=(zz>0.72)&(zz<0.79); print('hand body z[0.72,0.79]: x[%.3f,%.3f] y[%.3f,%.3f]'%(x[s].min(),x[s].max(),y[s].min(),y[s].max()))
"


# openrua op 61
python3 -c "
import numpy as np
d = np.load('birdview_depth.npy'); meta = np.load('birdview_meta.npy', allow_pickle=True).item()
K,T = meta['K'], meta['T']; fx,fy,cx,cy = K[0,0],K[1,1],K[0,2],K[1,2]
H,W = d.shape; us,vs = np.meshgrid(np.arange(W), np.arange(H)); z=d
P = (T[:3,:3] @ np.vstack([((us-cx)*z/fx).ravel(),((vs-cy)*z/fy).ravel(),z.ravel()]) + T[:3,3:4]).reshape(3,H,W)
np.save('P_bird.npy', P)
m = (P[0]>0.10)&(P[0]<0.32)&(P[1]>-0.10)&(P[1]<0.13)&(P[2]>0.512)
x,y,zz=P[0][m],P[1][m],P[2][m]
pts=np.c_[x,y]; c=pts.mean(0); _,_,vt=np.linalg.svd(pts-c); u=vt[0]
if u[0]<0: u=-u
proj=(pts-c)@u
print('ridge n',len(x),'center',c.round(4),'u',u.round(3),'ang %.1f'%np.degrees(np.arctan2(u[1],u[0])),'along [%.3f,%.3f]'%(proj.min(),proj.max()))
# lateral extent of whole mug body (z>0.45) relative to this axis
m2=(P[0]>0.10)&(P[0]<0.32)&(P[1]>-0.10)&(P[1]<0.125)&(P[2]>0.445)&(P[2]<0.53)
q=np.c_[P[0][m2],P[1][m2]]-c; v=np.array([-u[1],u[0]]); al=q@u; ac=q@v; zq=P[2][m2]
for lo in np.arange(-0.08,0.09,0.02):
    s=(al>=lo)&(al<lo+0.02)
    if s.any(): print('along [%5.2f,%5.2f]  across [%6.3f,%6.3f]  zmax %.3f'%(lo,lo+0.02,ac[s].min(),ac[s].max(),zq[s].max()))
"


# openrua op 62
python3 -c "
import numpy as np
P=np.load('P_bird.npy')
m=(P[0]>0.15)&(P[0]<0.30)&(P[1]>-0.10)&(P[1]<0.125)&(P[2]>0.44)
x,y,z=P[0][m],P[1][m],P[2][m]
pts=[]
for lo in np.arange(0.16,0.27,0.005):
    s=(x>=lo)&(x<lo+0.005)
    if not s.any(): continue
    # points within 3mm of slice max -> ridge; take mean y
    zs=z[s]; ys=y[s]; top=zs.max(); r=ys[zs>top-0.003]
    pts.append((lo+0.0025,r.mean(),top,r.min(),r.max()))
pts=np.array(pts)
for p in pts: print('x %.3f ridge y %.4f (%.3f..%.3f) ztop %.3f'%(p[0],p[1],p[3],p[4],p[2]))
A=np.c_[pts[:,0],np.ones(len(pts))]; k,b=np.linalg.lstsq(A,pts[:,1],rcond=None)[0]
print('ridge line y=%.3f x + %.4f  angle %.1f deg'%(k,b,np.degrees(np.arctan(k))))
"


# openrua op 63
python3 -c "
import numpy as np, cv2
P=np.load('P_bird.npy')
x0,x1,y0,y1=0.10,0.32,-0.12,0.22; res=0.0025
W=int((y1-y0)/res); H=int((x1-x0)/res)
img=np.full((H,W),np.nan)
m=(P[0]>x0)&(P[0]<x1)&(P[1]>y0)&(P[1]<y1)&(P[2]>0.43)
xi=((P[0][m]-x0)/res).astype(int); yi=((P[1][m]-y0)/res).astype(int); z=P[2][m]
for a,b,c in zip(xi,yi,z):
    if np.isnan(img[a,b]) or c>img[a,b]: img[a,b]=c
v=np.nan_to_num((img-0.43)/(0.53-0.43),nan=0); v=np.clip(v,0,1)
col=cv2.applyColorMap((v*255).astype(np.uint8),cv2.COLORMAP_JET); col[np.isnan(img)]=0
col=cv2.resize(col,(W*5,H*5),interpolation=cv2.INTER_NEAREST)
# grid lines every 5 cm
for gx in np.arange(x0,x1+1e-9,0.05):
    r=int((gx-x0)/res)*5; cv2.line(col,(0,r),(W*5,r),(255,255,255),1); cv2.putText(col,'x=%.2f'%gx,(2,r-2),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,255,255),1)
for gy in np.arange(y0,y1+1e-9,0.05):
    cc=int((gy-y0)/res)*5; cv2.line(col,(cc,0),(cc,H*5),(255,255,255),1); cv2.putText(col,'y=%.2f'%gy,(cc+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,255,255),1)
cv2.imwrite('hmap.png',col); print(col.shape)
"

# openrua op 64
python3 -c "
import numpy as np
P=np.load('P_bird.npy')
m=(P[0]>0.14)&(P[0]<0.30)&(P[1]>-0.12)&(P[1]<0.13)&(P[2]>0.47)
x,y,z=P[0][m],P[1][m],P[2][m]
E=[]
for lo in np.arange(0.15,0.28,0.005):
    s=(x>=lo)&(x<lo+0.005)
    if s.sum()<3: continue
    E.append((lo+0.0025,y[s].max(),y[s].min()))
E=np.array(E)
for e in E: print('x %.3f  +edge y %.4f   -edge y %.4f'%tuple(e))
# fit +edge on straight part
s=(E[:,0]>0.165)&(E[:,0]<0.235)
k,b=np.polyfit(E[s,0],E[s,1],1); print('+v edge: slope %.3f angle %.1f  intercept %.4f'%(k,np.degrees(np.arctan(k)),b))
s2=(E[:,0]>0.17)&(E[:,0]<0.215)
k2,b2=np.polyfit(E[s2,0],E[s2,2],1); print('-v edge (may incl handle): slope %.3f angle %.1f intercept %.4f'%(k2,np.degrees(np.arctan(k2)),b2))
"


# openrua op 65
mkdir -p "$(dirname /workspace/mugpose.py)"
cat > /workspace/mugpose.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Measure the lying red mug from a fresh birdview capture.

Body points (z > 0.47 excludes plate rim, handle on the table, and the flat pudding)
are fit with a minimum-area rectangle -> axis direction u (toward the open end),
center, length, width. Returns a dict; run standalone to print it.
"""
import subprocess, sys
import numpy as np, cv2

BIRD = dict(cam="birdview")


def capture():
    subprocess.run([sys.executable, "/workspace/locate.py", "birdview"], check=True,
                   stdout=subprocess.DEVNULL)


def cloud(cam="birdview"):
    d = np.load(f"{cam}_depth.npy"); meta = np.load(f"{cam}_meta.npy", allow_pickle=True).item()
    K, T = meta["K"], meta["T"]; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    H, W = d.shape; us, vs = np.meshgrid(np.arange(W), np.arange(H))
    P = (T[:3, :3] @ np.vstack([((us - cx) * d / fx).ravel(), ((vs - cy) * d / fy).ravel(), d.ravel()])
         + T[:3, 3:4]).reshape(3, H, W)
    return P


def measure(P, region=(0.05, 0.40, -0.25, 0.25), zmin=0.47, zmax=0.60, rim_toward=None):
    x0, x1, y0, y1 = region
    m = (P[0] > x0) & (P[0] < x1) & (P[1] > y0) & (P[1] < y1) & (P[2] > zmin) & (P[2] < zmax)
    pts = np.c_[P[0][m], P[1][m]].astype(np.float32)
    if len(pts) < 20:
        raise RuntimeError("mug body not found")
    (cx, cy), (w, h), ang = cv2.minAreaRect(pts)
    ang = np.radians(ang)
    if w >= h:
        L, Wd, u = w, h, np.array([np.cos(ang), np.sin(ang)])
    else:
        L, Wd, u = h, w, np.array([-np.sin(ang), np.cos(ang)])
    c = np.array([cx, cy])
    # open end: default = the end farther from the robot base (+x), or nearest rim_toward point
    if rim_toward is not None:
        if np.dot(np.array(rim_toward) - c, u) < 0:
            u = -u
    elif u[0] < 0:
        u = -u
    v = np.array([-u[1], u[0]])
    zs = P[2][m]
    top = float(np.percentile(zs, 99))
    rim = c + u * L / 2
    bottom = c - u * L / 2
    return dict(center=c, u=u, v=v, length=float(L), width=float(Wd), top=top,
                rim=rim, bottom=bottom, angle_deg=float(np.degrees(np.arctan2(u[1], u[0]))), n=int(m.sum()))


def report(mp):
    print("mug body: center (%.3f,%.3f) axis %.1f deg  len %.3f width %.3f top z %.3f  n=%d"
          % (*mp["center"], mp["angle_deg"], mp["length"], mp["width"], mp["top"], mp["n"]))
    print("  rim end (%.3f,%.3f)  bottom end (%.3f,%.3f)  u=%s v=%s"
          % (*mp["rim"], *mp["bottom"], mp["u"].round(3), mp["v"].round(3)))


if __name__ == "__main__":
    if "--nocap" not in sys.argv:
        capture()
    P = cloud()
    np.save("P_bird.npy", P)
    report(measure(P))
OPENRUA_EOF

# openrua op 66
python3 mugpose.py --nocap

# openrua op 67
python3 -c "
import numpy as np
P=np.load('P_bird.npy')
m=(P[0]>0.05)&(P[0]<0.40)&(P[1]>-0.25)&(P[1]<0.25)&(P[2]>0.47)&(P[2]<0.6)
x,y,z=P[0][m],P[1][m],P[2][m]
for lo in np.arange(0.05,0.30,0.02):
    s=(x>=lo)&(x<lo+0.02)
    if s.any(): print('x[%.2f,%.2f] n=%3d y[%.3f,%.3f] z[%.3f,%.3f]'%(lo,lo+0.02,s.sum(),y[s].min(),y[s].max(),z[s].min(),z[s].max()))
"


# openrua op 68
sed -i 's/def measure(P, region=(0.05, 0.40, -0.25, 0.25), zmin=0.47/def measure(P, region=(0.12, 0.40, -0.25, 0.25), zmin=0.48/' mugpose.py && python3 mugpose.py --nocap

# openrua op 69
mkdir -p "$(dirname /workspace/push.py)"
cat > /workspace/push.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Straight push with the closed gripper, hand vertical.

python3 push.py sx sy ex ey z yaw_deg [hover_z]
yaw_deg: direction (world, from +x) along which the fingers close (= palm long axis).
"""
import sys
import numpy as np, rclpy
from arm import Arm, DOWN_X


def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def down_yaw(yaw_deg, base=DOWN_X):
    """Hand pointing down, fingers closing along world direction yaw_deg (base=DOWN_X)."""
    h = np.radians(yaw_deg) / 2
    return qmul(np.array([0, 0, np.sin(h), np.cos(h)]), base)


def main():
    sx, sy, ex, ey, z, yaw = map(float, sys.argv[1:7])
    hover = float(sys.argv[7]) if len(sys.argv) > 7 else 0.60
    Q = down_yaw(yaw)
    a = Arm()
    a.gripper(0.0)
    print("== hover above start")
    a.goto([sx, sy, hover], Q, seconds=4.0)
    print("== descend")
    a.goto([sx, sy, z], Q, seconds=3.0)
    n = max(1, int(np.hypot(ex - sx, ey - sy) / 0.035 + 0.999))
    for i in range(1, n + 1):
        p = [sx + (ex - sx) * i / n, sy + (ey - sy) * i / n, z]
        print(f"== push step {i}/{n} -> {np.round(p, 3)}")
        a.goto(p, Q, seconds=2.5)
    print("== lift")
    a.goto([ex, ey, hover], Q, seconds=3.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 70
timeout 600 python3 push.py 0.175 0.106 0.229 0.061 0.482 50.5 2>&1 | grep -v "^\[" 

# openrua op 71
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/push.py", "old_string": "    a.goto([sx, sy, hover], Q, seconds=4.0)", "new_string": "    a.goto([sx, sy, hover], Q, seconds=4.0, max_jump=2.5)  # free-space move, larger jump OK", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 72
timeout 600 python3 push.py 0.175 0.106 0.229 0.061 0.482 50.5 2>&1 | grep -v "^\[" 

# openrua op 73
timeout 300 python3 arm.py joints 0,-0.161,0,-2.4446,0,2.2268,0.7854 5 2>&1 | grep -v "^\[" && python3 mugpose.py 2>&1 | grep -v "^\["

# openrua op 74
timeout 300 python3 arm.py joints 0,-0.161,0,-2.4446,0,2.2268,0.7854 4 2>&1 | grep -v "^\[" && python3 mugpose.py 2>&1 | grep -v "^\[" && python3 arm.py state 2>&1 | grep -v "^\["

# openrua op 75
python3 -c "
import numpy as np
from mugpose import measure
P=np.load('P_bird.npy'); mp=measure(P)
c,u,v=mp['center'],mp['u'],mp['v']
m=(P[0]>0.12)&(P[0]<0.40)&(P[1]>-0.25)&(P[1]<0.25)&(P[2]>0.455)&(P[2]<0.6)
q=np.c_[P[0][m],P[1][m]]-c; al=q@u; ac=q@v; z=P[2][m]
print('axis top z', mp['top'])
for lo in np.arange(-0.075,0.075,0.01):
    s=(al>=lo)&(al<lo+0.01)
    if s.sum()<3: continue
    body=s&(z>0.48)
    print('along [%6.3f,%6.3f] all z>0.455: across [%6.3f,%6.3f]  body z>0.48: across [%6.3f,%6.3f] width %.3f  zmax %.3f'%(lo,lo+0.01,ac[s].min(),ac[s].max(),ac[body].min() if body.any() else 0,ac[body].max() if body.any() else 0,(ac[body].max()-ac[body].min()) if body.any() else 0,z[s].max()))
"


# openrua op 76
python3 -c "
import numpy as np
from mugpose import measure
P=np.load('P_bird.npy'); mp=measure(P)
c,u,v=mp['center'],mp['u'],mp['v']
m=(P[0]>0.12)&(P[0]<0.40)&(P[1]>-0.25)&(P[1]<0.25)&(P[2]>0.44)&(P[2]<0.6)
q=np.c_[P[0][m],P[1][m]]-c; al=q@u; ac=q@v; z=P[2][m]
print('handle side (across < -0.04):')
for lo in np.arange(-0.075,0.075,0.01):
    s=(al>=lo)&(al<lo+0.01)&(ac<-0.04)&(ac>-0.12)
    if s.sum()<2: continue
    print('along [%6.3f,%6.3f] across [%6.3f,%6.3f] z [%.3f,%.3f] n=%d'%(lo,lo+0.01,ac[s].min(),ac[s].max(),z[s].min(),z[s].max(),s.sum()))
print('bottom end region (along < -0.06):')
s=(al<-0.055)&(al>-0.10)&(abs(ac)<0.05)
for lo in np.arange(-0.10,-0.055,0.005):
    t=s&(al>=lo)&(al<lo+0.005)
    if t.any(): print('along [%6.3f,%6.3f] across [%6.3f,%6.3f] z[%.3f,%.3f] n=%d'%(lo,lo+0.005,ac[t].min(),ac[t].max(),z[t].min(),z[t].max(),t.sum()))
"


# openrua op 77
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "DOWN_Y = np.array([0.92388, -0.38268, 0.0, 0.0])  # fingers close along world y\\nDOWN_X = np.array([0.38268, -0.92388, 0.0, 0.0])  # fingers close along world x", "new_string": "DOWN_Y = np.array([0.92388, -0.38268, 0.0, 0.0])  # fingers close along world y\\nDOWN_X = np.array([0.38268, -0.92388, 0.0, 0.0])  # fingers close along world x\\n\\n\\ndef qmul(a, b):\\n    \\"\\"\\"Quaternion product a*b (xyzw): apply b first, then a (both extrinsic/world).\\"\\"\\"\\n    ax, ay, az, aw = a; bx, by, bz, bw = b\\n    return np.array([aw * bx + ax * bw + ay * bz - az * by,\\n                     aw * by - ax * bz + ay * bw + az * bx,\\n                     aw * bz + ax * by - ay * bx + az * bw,\\n                     aw * bw - ax * bx - ay * by - az * bz])\\n\\n\\ndef quat_axis(axis, deg):\\n    \\"\\"\\"Quaternion (xyzw) for a rotation of deg about a world axis.\\"\\"\\"\\n    ax = np.array(axis, float); ax = ax / np.linalg.norm(ax)\\n    h = np.radians(deg) / 2\\n    return np.array([*(ax * np.sin(h)), np.cos(h)])\\n\\n\\ndef down_closing_along(yaw_deg):\\n    \\"\\"\\"Hand pointing straight down, fingers closing along the world direction yaw_deg (from +x).\\"\\"\\"\\n    return qmul(quat_axis([0, 0, 1], yaw_deg), DOWN_X)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 78
mkdir -p "$(dirname /workspace/grasp_mug.py)"
cat > /workspace/grasp_mug.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Body-grasp the lying red mug near its bottom end, stand it up, place it on the plate.

Stages (run one or more, in order):  plan approach grasp rotate carry place
State between stages is kept in grasp.json.
"""
import json, sys, subprocess
import numpy as np, rclpy
from arm import Arm, qmul, quat_axis, down_closing_along
from mugpose import cloud, measure, report

PLATE = np.array([0.121, -0.004])
GRASP_FROM_BOTTOM = 0.028      # pad centre this far from the mug bottom (along axis)
BODY_R = 0.035                 # body radius near the bottom end
TABLE = 0.425
PLATE_RIM_Z = 0.456
PALM_HALF = 0.030              # palm half-thickness (hand x)
STATE = "grasp.json"


def load():
    s = json.load(open(STATE)); return {k: (np.array(v) if isinstance(v, list) else v) for k, v in s.items()}


def save(s):
    json.dump({k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in s.items()}, open(STATE, "w"), indent=1)


def plan():
    subprocess.run([sys.executable, "/workspace/locate.py", "birdview"], check=True, stdout=subprocess.DEVNULL)
    P = cloud(); mp = measure(P); report(mp)
    c, u, v = mp["center"], mp["u"], mp["v"]
    along = -mp["length"] / 2 + GRASP_FROM_BOTTOM
    # lateral centre + axis height from body points in the grasp slice
    m = (P[0] > 0.12) & (P[0] < 0.40) & (P[1] > -0.25) & (P[1] < 0.25) & (P[2] > 0.48) & (P[2] < 0.6)
    q = np.c_[P[0][m], P[1][m]] - c; al = q @ u; ac = q @ v; z = P[2][m]
    s = (al > along - 0.012) & (al < along + 0.012)
    lat = 0.5 * (ac[s].min() + ac[s].max()); width = ac[s].max() - ac[s].min(); top = z[s].max()
    axis_z = top - BODY_R
    g = c + along * u + lat * v
    theta = mp["angle_deg"]
    st = dict(c=c, u=u, v=v, theta=theta, g=g, gz=axis_z + 0.003, width=width, top=top, along=along)
    print("grasp slice: along %.3f lateral centre %.4f width %.3f top %.3f -> axis z %.3f" % (along, lat, width, top, axis_z))
    print("grasp TCP (%.4f,%.4f,%.4f), closing along v (yaw %.1f)" % (*g, st["gz"], theta + 90))
    save(st)
    return st


def q_grasp(st):
    return down_closing_along(st["theta"] + 90)


def approach(a, st):
    Q = q_grasp(st); g = st["g"]
    a.gripper(0.04)
    print("== hover"); a.goto([*g, 0.60], Q, seconds=4.0, max_jump=2.5)
    print("== pre-grasp (fingertips 2 cm above axis)"); a.goto([*g, st["gz"] + 0.022], Q, seconds=3.0)
    subprocess.run([sys.executable, "/workspace/locate.py", "agentview"], check=True, stdout=subprocess.DEVNULL)
    print("agentview captured for check")


def grasp(a, st):
    Q = q_grasp(st); g = st["g"]
    print("== descend to grasp height"); a.goto([*g, st["gz"]], Q, seconds=3.0)
    f = a.gripper(0.0)
    gap = f[0] + f[1]
    print("finger gap %.4f (expect ~%.3f)" % (gap, st["width"]))
    if gap < 0.055 or gap > 0.078:
        raise RuntimeError("grasp looks wrong (gap %.4f)" % gap)
    print("== lift 1.5 cm"); a.goto([*g, st["gz"] + 0.015], Q, seconds=2.0)
    f = a.fingers(); print("fingers after small lift", f)
    if f[0] + f[1] < 0.055:
        raise RuntimeError("lost the mug")


def rotate(a, st):
    """Rotate about the closing axis so the mug bottom points down; hand ends pointing +u."""
    Q0 = q_grasp(st); g = st["g"]; v = st["v"]; z = st["gz"] + 0.015
    for deg in (-30, -60, -90):
        Q = qmul(quat_axis([v[0], v[1], 0.0], deg), Q0)
        print(f"== rotate {deg} deg"); a.goto([*g, z], Q, seconds=3.0)
        print("fingers", a.fingers())
    st["q_up"] = qmul(quat_axis([v[0], v[1], 0.0], -90), Q0); save(st)
    print("== lift to 0.60"); a.goto([*g, 0.60], st["q_up"], seconds=3.0)
    subprocess.run([sys.executable, "/workspace/locate.py", "agentview"], check=True, stdout=subprocess.DEVNULL)


def carry(a, st):
    """Yaw so the hand points +x, then move over the plate."""
    Qx = qmul(quat_axis([0, 0, 1], -st["theta"]), st["q_up"]); st["q_x"] = Qx; save(st)
    g = st["g"]
    print("== yaw to +x"); a.goto([*g, 0.60], Qx, seconds=3.0, max_jump=1.5)
    print("== over plate"); a.goto([*PLATE, 0.60], Qx, seconds=3.5)
    print("fingers", a.fingers())


def place(a, st):
    Qx = st["q_x"]
    tcp_z = PLATE_RIM_Z + PALM_HALF + 0.004          # palm bottom 4 mm above the plate rim
    print("mug bottom will be %.3f above plate top" % (tcp_z - GRASP_FROM_BOTTOM - 0.444))
    print("== lower"); a.goto([*PLATE, tcp_z], Qx, seconds=3.5)
    print("== release"); a.gripper(0.04)
    print("== retreat -x"); a.goto([PLATE[0] - 0.07, PLATE[1], tcp_z], Qx, seconds=2.5)
    print("== up"); a.goto([PLATE[0] - 0.07, PLATE[1], 0.62], Qx, seconds=3.0)


if __name__ == "__main__":
    stages = sys.argv[1:]
    st = plan() if "plan" in stages else load()
    todo = [s for s in stages if s != "plan"]
    if todo:
        a = Arm()
        for s in todo:
            globals()[s](a, st)
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 79
timeout 600 python3 grasp_mug.py plan approach 2>&1 | grep -v "^\["

# openrua op 80
python3 arm.py state 2>&1 | grep -v "^\[" | tail -2

# openrua op 81
sed -i 's/gap = f\[0\] + f\[1\]/gap = abs(f[0]) + abs(f[1])/; s/if f\[0\] + f\[1\] < 0.055/if abs(f[0]) + abs(f[1]) < 0.055/' grasp_mug.py && python3 -c "
import cv2, numpy as np
im=cv2.imread('agentview_color.png'); print(im.shape)
# find pixel of TCP in agentview to crop around it
meta=np.load('agentview_meta.npy',allow_pickle=True).item(); K,T=meta['K'],meta['T']
Tinv=np.linalg.inv(T); p=Tinv[:3,:3]@np.array([0.2206,-0.007,0.50])+Tinv[:3,3]; uv=K@(p/p[2]); print('tcp px',uv[:2].round(1))
u,v=int(uv[0]),int(uv[1]); crop=im[max(0,v-120):v+120, max(0,u-160):u+160]
cv2.imwrite('agent_pregrasp.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 82
timeout 600 python3 grasp_mug.py grasp 2>&1 | grep -v "^\["

# openrua op 83
timeout 600 python3 grasp_mug.py rotate 2>&1 | grep -v "^\["

# openrua op 84
python3 -c "
import cv2, numpy as np
im=cv2.imread('agentview_color.png')
meta=np.load('agentview_meta.npy',allow_pickle=True).item(); K,T=meta['K'],meta['T']
Tinv=np.linalg.inv(T); p=Tinv[:3,:3]@np.array([0.2206,-0.007,0.58])+Tinv[:3,3]; uv=K@(p/p[2])
u,v=int(uv[0]),int(uv[1]); crop=im[max(0,v-170):v+130, max(0,u-200):u+200]
cv2.imwrite('agent_rot.png', cv2.resize(crop,None,fx=2.4,fy=2.4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 85
timeout 600 python3 grasp_mug.py carry place 2>&1 | grep -v "^\["

# openrua op 86
timeout 600 python3 -c "
import numpy as np, rclpy, json
from arm import Arm, qmul, quat_axis
from grasp_mug import load, save
st=load(); a=Arm(); g=st['g']
for k in (1,2,3):
    Q=qmul(quat_axis([0,0,1], -st['theta']*k/3), st['q_up'])
    print('== yaw step',k)
    try:
        a.goto([*g,0.60], Q, seconds=2.5, max_jump=1.2)
    except RuntimeError as e:
        print('FAILED', e); break
    print('fingers', a.fingers())
else:
    st['q_x']=Q; save(st); print('yaw complete')
rclpy.shutdown()
" 2>&1 | grep -v "^\["

# openrua op 87
python3 arm.py state 2>&1 | grep -v "^\["; grep -A12 "limits" machine.yaml | head -30

# openrua op 88
timeout 300 python3 -c "
import numpy as np, rclpy
from arm import Arm, qmul, quat_axis
from grasp_mug import load
st=load(); a=Arm(); cur=np.array(a.arm_q())
PL=[0.121,-0.004]
for yaw in (0,-15,-30,-44,15,30,45):
    Q=qmul(quat_axis([0,0,1], yaw), st['q_up'])
    for pos in ([*st['g'],0.60],[*PL,0.60],[*PL,0.49]):
        best=None
        for t in range(6):
            try: q=np.array(a.ik(pos,Q,seed=cur))
            except RuntimeError: continue
            d=np.abs(q-cur).max()
            if best is None or d<best[0]: best=(d,q)
        print('yaw %4d pos %s -> %s'%(yaw,np.round(pos,3), 'no IK' if best is None else 'jump %.2f q=%s'%(best[0],np.round(best[1],2))))
rclpy.shutdown()
" 2>&1 | grep -v "^\["

# openrua op 89
timeout 300 python3 -c "
import numpy as np, rclpy
from arm import Arm
from moveit_msgs.srv import GetPositionFK
a=Arm()
def fk_links(q, links):
    req=GetPositionFK.Request(); req.fk_link_names=links; req.robot_state.joint_state=a._seed(q)
    fut=a.fk_cli.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
    return {n:np.array([p.pose.position.x,p.pose.position.y,p.pose.position.z]) for n,p in zip(fut.result().fk_link_names,fut.result().pose_stamped)}
L=['panda_link4','panda_link5','panda_link6','panda_link7','panda_link8','panda_hand']
for name,q in [('yaw0 low',[-1.45,1.76,1.24,-2.07,0.78,2.93,-1.91]),('yaw15 low',[-1.35,1.57,1.13,-1.94,0.58,2.58,-1.62]),('yaw30 low',[-1.2,1.45,1.02,-1.74,0.52,2.21,-1.5])]:
    r=fk_links(q,L); print(name)
    for n in L: print('   %-12s %s  dist to white mug axis %.3f'%(n[6:],r[n].round(3),np.hypot(r[n][0]+0.122,r[n][1]+0.137)))
rclpy.shutdown()
" 2>&1 | grep -v "^\["

# openrua op 90
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/grasp_mug.py", "old_string": "def carry(a, st):\\n    \\"\\"\\"Yaw so the hand points +x, then move over the plate.\\"\\"\\"\\n    Qx = qmul(quat_axis([0, 0, 1], -st[\\"theta\\"]), st[\\"q_up\\"]); st[\\"q_x\\"] = Qx; save(st)\\n    g = st[\\"g\\"]\\n    print(\\"== yaw to +x\\"); a.goto([*g, 0.60], Qx, seconds=3.0, max_jump=1.5)\\n    print(\\"== over plate\\"); a.goto([*PLATE, 0.60], Qx, seconds=3.5)\\n    print(\\"fingers\\", a.fingers())\\n\\n\\ndef place(a, st):\\n    Qx = st[\\"q_x\\"]\\n    tcp_z = PLATE_RIM_Z + PALM_HALF + 0.004          # palm bottom 4 mm above the plate rim\\n    print(\\"mug bottom will be %.3f above plate top\\" % (tcp_z - GRASP_FROM_BOTTOM - 0.444))\\n    print(\\"== lower\\"); a.goto([*PLATE, tcp_z], Qx, seconds=3.5)\\n    print(\\"== release\\"); a.gripper(0.04)\\n    print(\\"== retreat -x\\"); a.goto([PLATE[0] - 0.07, PLATE[1], tcp_z], Qx, seconds=2.5)\\n    print(\\"== up\\"); a.goto([PLATE[0] - 0.07, PLATE[1], 0.62], Qx, seconds=3.0)", "new_string": "CARRY_YAW = 30.0   # extra yaw about world z; IK probe: feasible without branch change, wrist clears the white mug\\n\\n\\ndef carry(a, st):\\n    \\"\\"\\"Yaw a little (+y) so the wrist stays clear of the white mug, then move over the plate.\\"\\"\\"\\n    Qx = qmul(quat_axis([0, 0, 1], CARRY_YAW), st[\\"q_up\\"]); st[\\"q_x\\"] = Qx; save(st)\\n    g = st[\\"g\\"]\\n    print(\\"== yaw\\"); a.goto([*g, 0.60], Qx, seconds=3.0, max_jump=1.2)\\n    print(\\"== over plate\\"); a.goto([*PLATE, 0.60], Qx, seconds=3.5, max_jump=1.2)\\n    print(\\"fingers\\", a.fingers())\\n\\n\\ndef place(a, st):\\n    Qx = st[\\"q_x\\"]\\n    hz = quat_to_R(*Qx)[:, 2]                         # hand z (horizontal, pointing from the palm to the mug)\\n    tcp_z = PLATE_RIM_Z + PALM_HALF + 0.004          # palm bottom 4 mm above the plate rim\\n    print(\\"mug bottom will be %.3f above plate top\\" % (tcp_z - GRASP_FROM_BOTTOM - 0.444))\\n    print(\\"== lower\\"); a.goto([*PLATE, tcp_z], Qx, seconds=3.5)\\n    print(\\"== release\\"); a.gripper(0.04)\\n    back = np.array([PLATE[0], PLATE[1], tcp_z]) - 0.07 * np.array([hz[0], hz[1], 0.0])\\n    print(\\"== retreat along -hand z\\"); a.goto(back, Qx, seconds=2.5)\\n    print(\\"== up\\"); a.goto([back[0], back[1], 0.62], Qx, seconds=3.0)\\n    subprocess.run([sys.executable, \\"/workspace/locate.py\\", \\"agentview\\"], check=True, stdout=subprocess.DEVNULL)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 91
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/grasp_mug.py", "old_string": "from arm import Arm, qmul, quat_axis, down_closing_along", "new_string": "from arm import Arm, qmul, quat_axis, down_closing_along, quat_to_R", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 92
timeout 900 python3 grasp_mug.py carry place 2>&1 | grep -v "^\[" 

# openrua op 93
timeout 300 python3 mugpose.py 2>&1 | grep -v "^\[" ; timeout 120 python3 -c "
import numpy as np
P=np.load('P_bird.npy')
# objects above plate top near the plate
m=(np.hypot(P[0]-0.121,P[1]+0.004)<0.10)&(P[2]>0.46)&(P[2]<0.7)
print('pts above 0.46 near plate:',m.sum(),'x %.3f..%.3f y %.3f..%.3f z max %.3f'%(P[0][m].min(),P[0][m].max(),P[1][m].min(),P[1][m].max(),P[2][m].max()))
hi=m&(P[2]>P[2][m].max()-0.01)
print('rim (top 1cm) centre (%.3f,%.3f) extent x %.3f y %.3f'%(P[0][hi].mean(),P[1][hi].mean(),P[0][hi].max()-P[0][hi].min(),P[1][hi].max()-P[1][hi].min()))
"

# openrua op 94
timeout 300 python3 -c "
import numpy as np, rclpy
from arm import Arm, DOWN_Y
a=Arm()
# park: go up and back toward the base at a safe height, hand pointing down
a.goto([-0.15,-0.30,0.75], DOWN_Y, seconds=5.0, max_jump=3.0)
print('q',np.round(a.arm_q(),4).tolist())
rclpy.shutdown()
" 2>&1 | grep -v "^\[" && timeout 300 python3 mugpose.py 2>&1 | grep -v "^\[" ; timeout 120 python3 -c "
import numpy as np
P=np.load('P_bird.npy')
m=(np.hypot(P[0]-0.121,P[1]+0.004)<0.12)&(P[2]>0.46)&(P[2]<0.7)
print('pts above 0.46 near plate:',m.sum(),'x %.3f..%.3f y %.3f..%.3f z max %.3f'%(P[0][m].min(),P[0][m].max(),P[1][m].min(),P[1][m].max(),P[2][m].max()))
hi=m&(P[2]>P[2][m].max()-0.012)
print('rim (top 1.2cm) centre (%.3f,%.3f) extent x %.3f y %.3f n=%d'%(P[0][hi].mean(),P[1][hi].mean(),P[0][hi].max()-P[0][hi].min(),P[1][hi].max()-P[1][hi].min(),hi.sum()))
lo=m&(P[2]<0.52)
print('lower body/handle x %.3f..%.3f y %.3f..%.3f'%(P[0][lo].min(),P[0][lo].max(),P[1][lo].min(),P[1][lo].max()))
# pudding
mp=(P[0]>0.10)&(P[0]<0.30)&(P[1]>0.08)&(P[1]<0.25)&(P[2]>0.44)&(P[2]<0.48)
print('pudding pts',mp.sum(),'x %.3f..%.3f y %.3f..%.3f top %.3f'%(P[0][mp].min(),P[0][mp].max(),P[1][mp].min(),P[1][mp].max(),P[2][mp].max()))
"

# openrua op 95
timeout 120 python3 -c "
import numpy as np
P=np.load('P_bird.npy')
def occ(x0,x1,y0,y1,zmin,label):
    m=(P[0]>x0)&(P[0]<x1)&(P[1]>y0)&(P[1]<y1)&(P[2]>zmin)&(P[2]<0.9)
    print(label,'pts',m.sum(), ('' if m.sum()==0 else 'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(P[0][m].min(),P[0][m].max(),P[1][m].min(),P[1][m].max(),P[2][m].max())))
occ(0.16,0.24,0.04,0.28,0.44,'palm footprint at pudding grasp (x=0.20, palm along y), above table')
occ(0.16,0.24,0.04,0.28,0.482,'  ...above palm bottom 0.482')
occ(0.0,0.25,0.09,0.16,0.44,'palm footprint at place (y=0.125, palm along x)')
occ(0.08,0.17,0.07,0.18,0.44,'pudding target footprint (should be empty)')
# everything on table above 0.44 excluding known stuff: list clusters coarse
m=(P[2]>0.44)&(P[2]<0.9)&(np.abs(P[0])<0.5)&(np.abs(P[1])<0.5)
xs,ys=P[0][m],P[1][m]
H,xe,ye=np.histogram2d(xs,ys,bins=[np.arange(-0.5,0.51,0.05),np.arange(-0.5,0.51,0.05)])
for i in range(H.shape[0]):
    for j in range(H.shape[1]):
        if H[i,j]>15: print('cell x %.2f y %.2f n=%d'%(xe[i],ye[j],H[i,j]))
"

# openrua op 96
timeout 120 python3 -c "
import numpy as np
P=np.load('P_bird.npy')
m=(P[0]>0.0)&(P[0]<0.145)&(P[1]>0.062)&(P[1]<0.25)&(P[2]>0.435)&(P[2]<0.9)
print(m.sum())
if m.sum(): 
    for x,y,z in np.c_[P[0][m],P[1][m],P[2][m]][::max(1,m.sum()//25)]: print('%.3f %.3f %.3f'%(x,y,z))
# heights around plate at y>0.06
m2=(P[0]>0.05)&(P[0]<0.2)&(P[1]>0.055)&(P[1]<0.13)&(P[2]>0.3)
print('z range there %.3f..%.3f'%(P[2][m2].min(),P[2][m2].max()))
"

# openrua op 97
timeout 120 python3 -c "
import numpy as np
P=np.load('P_bird.npy')
m=(P[0]>-0.05)&(P[0]<0.30)&(P[1]>-0.15)&(P[1]<0.12)&(P[2]>0.446)&(P[2]<0.459)
x,y=P[0][m],P[1][m]
print('rim-height pts',m.sum(),'x %.3f..%.3f y %.3f..%.3f'%(x.min(),x.max(),y.min(),y.max()))
# row profile through y at x~0.12
r=(np.abs(P[0]-0.121)<0.004)
ys=P[1][r]; zs=P[2][r]; o=np.argsort(ys)
for yy,zz in zip(ys[o][::3],zs[o][::3]):
    if -0.13<yy<0.13: print('%.3f %.3f'%(yy,zz))
" | head -80

# openrua op 98
mkdir -p "$(dirname /workspace/move_pudding.py)"
cat > /workspace/move_pudding.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the chocolate pudding (lying flat, long axis along x) across y, yaw 90 deg,
place it flat to the +y side of the plate with its long axis along y.

python3 move_pudding.py dry   -> IK feasibility only
python3 move_pudding.py go    -> execute
"""
import sys, subprocess
import numpy as np, rclpy
from arm import Arm, DOWN_Y, down_closing_along

PUD = np.array([0.205, 0.1575])          # grasp point (1.5 cm toward +x of the box centre)
TABLE = 0.425
GRASP_Z = TABLE + 0.012                  # fingertips 1.2 cm above the table (box is 4 cm tall)
CARRY_Z = 0.60
TARGET = np.array([0.123, 0.140])        # box centre: 2 cm beyond the plate edge (y=0.079)
PLACE_Z = GRASP_Z + 0.006

a = Arm()
Qg = DOWN_Y
cands = [down_closing_along(0), down_closing_along(180)]
poses = [([*PUD, CARRY_Z], Qg), ([*PUD, GRASP_Z], Qg)]
print("IK check from current q", np.round(a.arm_q(), 3).tolist())
for p, Q in poses:
    q, j = a.ik_near(p, Q, max_jump=9); print("  grasp pose", np.round(p, 3), "jump %.2f" % j)
best = None
for i, Qp in enumerate(cands):
    try:
        q1, j1 = a.ik_near([*PUD, CARRY_Z], Qp, max_jump=9)
        q2, j2 = a.ik_near([*TARGET, CARRY_Z], Qp, max_jump=9)
        q3, j3 = a.ik_near([*TARGET, PLACE_Z], Qp, max_jump=9)
        print("  place yaw cand %d: jumps from park %.2f %.2f %.2f" % (i, j1, j2, j3))
        if best is None or j1 < best[1]:
            best = (Qp, j1)
    except RuntimeError as e:
        print("  place yaw cand %d: %s" % (i, e))
Qp = best[0]
if sys.argv[1:2] != ["go"]:
    rclpy.shutdown(); sys.exit()

a.gripper(0.04)
print("== hover over pudding"); a.goto([*PUD, CARRY_Z], Qg, seconds=5.0, max_jump=3.0)
print("== descend"); a.goto([*PUD, GRASP_Z], Qg, seconds=3.0)
f = a.gripper(0.0); gap = abs(f[0]) + abs(f[1])
print("gap %.4f (box is ~0.053 across y)" % gap)
if not 0.040 < gap < 0.065:
    raise RuntimeError("grasp looks wrong")
print("== lift"); a.goto([*PUD, CARRY_Z], Qg, seconds=3.0)
f = a.fingers(); print("fingers after lift", f)
if abs(f[0]) + abs(f[1]) < 0.040:
    raise RuntimeError("lost the box")
print("== yaw 90"); a.goto([*PUD, CARRY_Z], Qp, seconds=3.0, max_jump=2.5)
print("== carry"); a.goto([*TARGET, CARRY_Z], Qp, seconds=3.5)
print("== lower"); a.goto([*TARGET, PLACE_Z], Qp, seconds=3.0)
print("== release"); a.gripper(0.04)
print("== up"); a.goto([*TARGET, CARRY_Z], Qp, seconds=3.0)
print("== park"); a.goto([-0.15, -0.30, 0.75], DOWN_Y, seconds=5.0, max_jump=3.0)
subprocess.run([sys.executable, "/workspace/locate.py", "agentview"], check=True, stdout=subprocess.DEVNULL)
rclpy.shutdown()
OPENRUA_EOF

# openrua op 99
timeout 300 python3 move_pudding.py dry 2>&1 | grep -v "^\["

# openrua op 100
timeout 900 python3 move_pudding.py go 2>&1 | grep -v "^\["

# openrua op 101
timeout 300 python3 locate.py birdview >/dev/null 2>&1; timeout 120 python3 -c "
import numpy as np
from mugpose import cloud
P=cloud()
# plate rim
m=(P[0]>-0.05)&(P[0]<0.30)&(P[1]>-0.15)&(P[1]<0.12)&(P[2]>0.446)&(P[2]<0.459)
pc=np.array([(P[0][m].min()+P[0][m].max())/2,(P[1][m].min()+P[1][m].max())/2]); pr=(P[1][m].max()-P[1][m].min())/2
print('plate centre (%.3f,%.3f) radius %.3f'%(*pc,pr))
# red mug rim (top 1.2 cm of anything > 0.50 near the plate)
m=(np.hypot(P[0]-pc[0],P[1]-pc[1])<0.10)&(P[2]>0.50)
top=P[2][m].max(); hi=m&(P[2]>top-0.012)
mc=np.array([P[0][hi].mean(),P[1][hi].mean()])
print('red mug: top z %.3f (table+%.3f), rim centre (%.3f,%.3f) extent %.3f x %.3f -> %.3f m from plate centre'%(top,top-0.425,*mc,P[0][hi].max()-P[0][hi].min(),P[1][hi].max()-P[1][hi].min(),np.hypot(*(mc-pc))))
# pudding
m=(P[0]>0.05)&(P[0]<0.30)&(P[1]>0.08)&(P[1]<0.30)&(P[2]>0.44)&(P[2]<0.48)
print('pudding: x %.3f..%.3f y %.3f..%.3f top %.3f (n=%d)'%(P[0][m].min(),P[0][m].max(),P[1][m].min(),P[1][m].max(),P[2][m].max(),m.sum()))
print('  centre (%.3f,%.3f); plate +y edge at %.3f; gap to plate %.3f'%((P[0][m].min()+P[0][m].max())/2,(P[1][m].min()+P[1][m].max())/2,pc[1]+pr,P[1][m].min()-(pc[1]+pr)))
" 2>&1 | grep -v "^\["

# openrua op 102
timeout 120 python3 -c "
import numpy as np
from mugpose import cloud
P=cloud()
m=(P[0]>-0.05)&(P[0]<0.30)&(P[1]>-0.15)&(P[1]<0.12)&(P[2]>0.446)&(P[2]<0.459)&~((P[0]>0.09)&(P[0]<0.15)&(P[1]>0.085))
print('plate rim (pudding excluded): x %.3f..%.3f y %.3f..%.3f'%(P[0][m].min(),P[0][m].max(),P[1][m].min(),P[1][m].max()))
# y-profile at x=0.118 across the plate edge / pudding
r=np.abs(P[0]-0.118)<0.003; ys=P[1][r]; zs=P[2][r]; o=np.argsort(ys)
print(' '.join('%.3f:%.3f'%(y,z) for y,z in zip(ys[o],zs[o]) if 0.06<y<0.18))
" 2>&1 | grep -v "^\["
