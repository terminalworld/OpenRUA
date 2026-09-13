#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 30 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head -40; echo ----; timeout 20 ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
timeout 60 python3 - <<'EOF'
import rclpy, numpy as np, struct
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("probe")
got = {}
def grab(topic, T, key):
    sub = node.create_subscription(T, topic, lambda m: got.setdefault(key, m), 1)
    while key not in got: rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_subscription(sub)
grab("/tf", TFMessage, "tf")
for t in got["tf"].transforms:
    tr, q = t.transform.translation, t.transform.rotation
    print(f"{t.header.frame_id} -> {t.child_frame_id}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
# depth snapshot for birdview & agentview
for cam in ["agentview","birdview"]:
    grab(f"/{cam}/depth/image_raw", Image, cam+"d")
    d = got[cam+"d"]
    arr = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    np.save(f"{cam}_depth.npy", arr)
    print(cam, d.encoding, arr.shape, np.nanmin(arr), np.nanmax(arr))
rclpy.shutdown()
EOF

# openrua op 7
timeout 60 python3 - <<'EOF'
import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("probe2")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t
node.create_subscription(TFMessage, "/tf", cb, 50)
import time
end = time.time()+8
while time.time() < end: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    tr, q = t.transform.translation, t.transform.rotation
    print(f"{p} -> {c}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
rclpy.shutdown()
EOF

# openrua op 8
timeout 120 python3 - <<'EOF'
import numpy as np, cv2
d = np.load("birdview_depth.npy")
img = cv2.imread("birdview.png")
fx=fy=579.4112549695428; cx=320; cy=240
# birdview optical frame: world t=(-0.2,0,3.0), q=(0.7071,0.7071,0,0)
qx,qy,qz,qw = 0.7071,0.7071,0,0
def R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
Rm = R(qx,qy,qz,qw); t=np.array([-0.2,0,3.0])
v,u = np.mgrid[0:480,0:640]
X=(u-cx)*d/fx; Y=(v-cy)*d/fy; Z=d
P = np.stack([X,Y,Z],-1).reshape(-1,3) @ Rm.T + t
P = P.reshape(480,640,3)
np.save("birdview_world.npy", P)
zs = P[...,2]
# table height: mode of z in the table region
hist, edges = np.histogram(zs[np.isfinite(zs)], bins=300)
i = hist.argmax(); print("mode z:", edges[i], edges[i+1])
table_z = (edges[i]+edges[i+1])/2
mask = (zs > table_z+0.01) & (zs < table_z+0.5)
mask = mask.astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for k in range(1,n):
    if stats[k,4] < 15: continue
    m = lab==k
    pts = P[m]
    print(f"blob {k}: px area={stats[k,4]} centroid px=({cent[k][0]:.0f},{cent[k][1]:.0f}) world x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}")
EOF

# openrua op 9
timeout 120 python3 - <<'EOF'
import numpy as np, cv2
P = np.load("birdview_world.npy"); zs=P[...,2]
table_z = 0.425
mask = ((zs > table_z+0.008) & (zs < table_z+0.6)).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for k in range(1,n):
    if stats[k,4] < 5: continue
    pts = P[lab==k]
    print(f"blob {k}: area={stats[k,4]} px=({cent[k][0]:.0f},{cent[k][1]:.0f}) x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} cx={pts[:,0].mean():.3f} cy={pts[:,1].mean():.3f}")
EOF

# openrua op 10
timeout 120 python3 - <<'EOF'
import numpy as np, cv2
d = np.load("agentview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw = 0.6182,0.6182,-0.3432,-0.3432
n=np.sqrt(qx*qx+qy*qy+qz*qz+qw*qw); qx,qy,qz,qw=qx/n,qy/n,qz/n,qw/n
def R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
Rm = R(qx,qy,qz,qw); t=np.array([0.6066,0,0.96])
v,u = np.mgrid[0:480,0:640]
X=(u-cx)*d/fx; Y=(v-cy)*d/fy; Z=d
P = (np.stack([X,Y,Z],-1).reshape(-1,3) @ Rm.T + t).reshape(480,640,3)
np.save("agentview_world.npy", P)
zs=P[...,2]
print("table check at (100,420):", P[420,100])
mask = ((zs > 0.425+0.008) & (zs < 1.2)).astype(np.uint8)
nn, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for k in range(1,nn):
    if stats[k,4] < 20: continue
    pts = P[lab==k]
    print(f"blob {k}: area={stats[k,4]} px=({cent[k][0]:.0f},{cent[k][1]:.0f}) x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z=[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
# cream cheese region specifically
for (uu,vv) in [(348,262),(340,255),(355,270),(232,235),(232,215)]:
    print((uu,vv), P[vv,uu])
EOF

# openrua op 11
timeout 20 ros2 service list | grep -iE "fk|ik|plan|scene"; timeout 10 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 12
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Reusable robot helpers for this Panda workstation (world<->base handled)."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (no rotation)
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# hand pointing straight down, fingers separating along world Y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
# hand pointing down, fingers separating along world X (rotated 90deg about z)
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()) % 100000))
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    def fk_hand(self, q=None):
        """Return (pos_world, quat) of panda_hand."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    # ---------- planning ----------
    def ik_world(self, pos_world, quat, at_tcp=True, seed=None, attempts=3):
        """IK for the hand (or TCP) at a world pose. Returns arm joint list or None."""
        pos_world = np.array(pos_world, float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos_world = pos_world - TCP * R[:, 2]
        pb = pos_world - BASE_IN_WORLD
        if seed is None:
            seed = self.arm_q()
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print("  IK attempt failed:", res and res.error_code.val, flush=True)
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                tt = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code if rf.result() else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move_q done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, at_tcp=True, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos_world}")
        self.move_q(q, seconds)
        p, _ = self.tcp_world()
        print(f"  tcp now {p.round(4)} target {np.round(pos_world,4)}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        # a few settle ticks
        for _ in range(5):
            self.spin(0.1)
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}", flush=True)
        return r

    def servo(self, vx=0, vy=0, vz=0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(vx), float(vy), float(vz)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(0.05)

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        topic = f"/{cam}/color/image_raw"
        got = {}
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out
OPENRUA_EOF

# openrua op 13
timeout 120 python3 -u - <<'EOF'
from rob import *
r = Robot()
q = r.arm_q(); print("q", np.round(q,3))
pos, quat = r.fk_hand(q); print("hand world", pos.round(4), np.round(quat,4))
tcp, _ = r.tcp_world(q); print("tcp world", tcp.round(4))
print("gap", r.finger_gap())
# test IK for a pose above the cream cheese box
sol = r.ik_world([-0.148, 0.047, 0.60], Q_DOWN_Y)
print("ik sol", sol and np.round(sol,3))
if sol: print("check fk tcp", r.tcp_world(sol)[0].round(4))
EOF

# openrua op 14
timeout 60 python3 -u - <<'EOF'
import rclpy, numpy as np, time
from tf2_msgs.msg import TFMessage
from rob import quat_to_R
rclpy.init(); node = rclpy.create_node("chain")
seen = {}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id] = t
node.create_subscription(TFMessage, "/tf", cb, 50)
node.create_subscription(TFMessage, "/tf_static", cb, 50)
end=time.time()+6
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
def T(t):
    tr,q = t.transform.translation, t.transform.rotation
    M=np.eye(4); M[:3,:3]=quat_to_R(q.x,q.y,q.z,q.w); M[:3,3]=[tr.x,tr.y,tr.z]; return M
chain = ["panda_link0","panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]
M = np.eye(4)
for c in chain:
    if c in seen: M = M @ T(seen[c]); print(c, "parent", seen[c].header.frame_id, "->", M[:3,3].round(4))
    else: print("missing", c)
print("hand in world via TF:", M[:3,3].round(4))
print("hand R:\n", M[:3,:3].round(3))
EOF

# openrua op 15
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\]).*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # planner model frame == world here (verified by FK vs TF)/' rob.py && grep -n BASE_IN_WORLD rob.py | head -2 && timeout 120 python3 -u - <<'EOF'
from rob import *
r = Robot()
q = r.arm_q()
pos, quat = r.fk_hand(q); print("hand world", pos.round(4), np.round(quat,4))
print("tcp world", r.tcp_world(q)[0].round(4))
sol = r.ik_world([-0.148, 0.047, 0.60], Q_DOWN_Y)
print("ik sol", sol and np.round(sol,3))
if sol: print("check fk tcp", r.tcp_world(sol)[0].round(4), np.round(r.fk_hand(sol)[1],3))
sol2 = r.ik_world([-0.19, -0.138, 0.60], Q_DOWN_Y)
print("ik sol2", sol2 and np.round(sol2,3))
sol3 = r.ik_world([-0.005, 0.249, 0.80], Q_DOWN_Y)
print("ik sol3", sol3 and np.round(sol3,3))
EOF

# openrua op 16
timeout 180 python3 -u - <<'EOF'
from rob import *
r = Robot()
q0 = r.arm_q()
p0, quat0 = r.fk_hand(q0)
print("current hand", p0.round(4), np.round(quat0,4))
# IK back to the current hand pose
s = r.ik_world(p0, quat0, at_tcp=False)
print("ik->current:", np.round(s,3), "fk:", np.round(r.fk_hand(s)[1],3))
for name, Q in [("DOWN_Y", Q_DOWN_Y), ("DOWN_X", Q_DOWN_X), ("DOWN_Y45", (0.924,0.383,0,0))]:
    for tgt in ([-0.148,0.047,0.60], [-0.19,-0.138,0.60]):
        s = r.ik_world(tgt, Q)
        if s is None: print(name, tgt, "no sol"); continue
        pp, qq = r.fk_hand(s)
        print(name, tgt, "sol", np.round(s,3), "fk quat", np.round(qq,3), "tcp", r.tcp_world(s)[0].round(3))
EOF

# openrua op 17
python3 - <<'EOF'
s = open("rob.py").read()
s = s.replace('''def quat_to_R(x, y, z, w):''', '''def qmul(a, b):
    """quaternion product a*b, (x,y,z,w) order"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# machine fact (measured): the IK solver's tip frame is panda_hand yawed by
# -45deg (i.e. panda_link8). Request R_des * Rz(+45deg) to land the HAND at R_des.
IK_YAW_FIX = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))


def quat_to_R(x, y, z, w):''')
s = s.replace('''            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)''',
'''            qreq = qmul(quat, IK_YAW_FIX)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qreq)''')
open("rob.py","w").write(s)
EOF
timeout 180 python3 -u - <<'EOF'
from rob import *
r = Robot()
for name, Q in [("DOWN_Y", Q_DOWN_Y), ("DOWN_X", Q_DOWN_X)]:
    s = r.ik_world([-0.148,0.047,0.60], Q)
    pp, qq = r.fk_hand(s)
    print(name, "sol", np.round(s,3), "fk quat", np.round(qq,3), "tcp", r.tcp_world(s)[0].round(3), "fingers axis (world):", quat_to_R(*qq)[:,1].round(2))
EOF

# openrua op 18
cat > step1.py <<'EOF'
from rob import *
r = Robot()
print("gap before", r.finger_gap())
r.gripper(0.04)
print("moving above ketchup")
r.move_tcp([-0.19, -0.138, 0.68], Q_DOWN_Y, seconds=4.0)
print("q", np.round(r.arm_q(),3))
r.snap("birdview", "/workspace/bird2.png")
r.snap("agentview", "/workspace/agent2.png")
print("DONE")
EOF
timeout 900 python3 -u step1.py > step1.log 2>&1; cat step1.log

# openrua op 19
timeout 120 python3 -u - <<'EOF'
import numpy as np, cv2, rclpy
from sensor_msgs.msg import Image
from rob import quat_to_R
rclpy.init(); node=rclpy.create_node("d")
got={}
sub=node.create_subscription(Image,"/birdview/depth/image_raw",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
m=got["m"]; d=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width)
fx=fy=579.4112549695428; cx=320; cy=240
Rm=quat_to_R(0.7071,0.7071,0,0); t=np.array([-0.2,0,3.0])
v,u=np.mgrid[0:480,0:640]
P=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@Rm.T+t).reshape(480,640,3)
zs=P[...,2]
mask=((zs>0.433)&(zs<0.6)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for k in range(1,n):
    if stats[k,4]<5: continue
    pts=P[lab==k]
    print(f"blob {k}: area={stats[k,4]} px=({cent[k][0]:.0f},{cent[k][1]:.0f}) x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} c=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
rclpy.shutdown()
EOF

# openrua op 20
timeout 120 python3 -u - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from rob import quat_to_R
rclpy.init(); node=rclpy.create_node("d")
got={}
node.create_subscription(Image,"/birdview/depth/image_raw",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
m=got["m"]; d=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width)
fx=fy=579.4112549695428; cx=320; cy=240
Rm=quat_to_R(0.7071,0.7071,0,0); t=np.array([-0.2,0,3.0])
v,u=np.mgrid[0:480,0:640]
P=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@Rm.T+t).reshape(480,640,3)
zs=P[...,2]
# table z around box (px 330,251): sample ring
for (uu,vv) in [(330,235),(330,265),(310,251),(350,251),(300,300),(400,250),(250,200)]:
    print((uu,vv), P[vv,uu].round(4))
# histogram of table z within table region
reg = zs[150:330, 230:430]
h,e=np.histogram(reg[(reg>0.40)&(reg<0.47)],bins=35)
for hh,ee in zip(h,e): print(f"{ee:.4f} {hh}")
# box profile along its center row
row = int(cent) if False else 251
print("box row z:", zs[251,318:342].round(3))
print("box col z:", zs[243:260,330].round(3))
rclpy.shutdown()
EOF

# openrua op 21
timeout 120 python3 -u - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from rob import quat_to_R
rclpy.init(); node=rclpy.create_node("d")
got={}
node.create_subscription(Image,"/birdview/depth/image_raw",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
m=got["m"]; d=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width)
fx=fy=579.4112549695428; cx=320; cy=240
Rm=quat_to_R(0.7071,0.7071,0,0); t=np.array([-0.2,0,3.0])
v,u=np.mgrid[0:480,0:640]
P=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@Rm.T+t).reshape(480,640,3)
X,Y,Z=P[...,0],P[...,1],P[...,2]
reg=(X>-0.26)&(X<-0.13)&(Y>-0.21)&(Y<-0.07)&(Z>0.43)
pts=P[reg]
print("ketchup all: n",len(pts),"x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3),"z",pts[:,2].min().round(3),pts[:,2].max().round(3))
for lo,hi in [(0.43,0.50),(0.50,0.53),(0.53,0.56)]:
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f"z[{lo},{hi}) n={len(s)} x=[{s[:,0].min():.3f},{s[:,0].max():.3f}] y=[{s[:,1].min():.3f},{s[:,1].max():.3f}] c=({s[:,0].mean():.3f},{s[:,1].mean():.3f})")
# the top-down only sees the top surface: for a bottle the top surface is the body shoulder + cap. Footprint (all):
print("footprint center", pts[:,0].mean().round(3), pts[:,1].mean().round(3), "extent", (pts[:,0].max()-pts[:,0].min()).round(3), (pts[:,1].max()-pts[:,1].min()).round(3))
rclpy.shutdown()
EOF

# openrua op 22
cat > step2.py <<'EOF'
from rob import *
r = Robot()
K = np.array([-0.197, -0.139])
print("pre-grasp above ketchup, fingers along X")
r.move_tcp([K[0], K[1], 0.62], Q_DOWN_X, seconds=3.0)
print("descend")
r.move_tcp([K[0], K[1], 0.47], Q_DOWN_X, seconds=3.0)
r.snap("robot0_eye_in_hand", "/workspace/eih_k1.png")
r.snap("agentview", "/workspace/agent_k1.png")
print("close")
r.gripper(0.0)
print("gap after close:", r.finger_gap())
print("lift")
r.move_tcp([K[0], K[1], 0.75], Q_DOWN_X, seconds=3.0)
print("gap after lift:", r.finger_gap())
r.snap("agentview", "/workspace/agent_k2.png")
print("DONE")
EOF
timeout 1200 python3 -u step2.py > step2.log 2>&1; cat step2.log

# openrua op 23
timeout 60 python3 -u -c "
from rob import *
r=Robot(); print('q',np.round(r.arm_q(),3)); print('tcp',r.tcp_world()[0].round(4)); print('gap',r.finger_gap()); r.snap('agentview','/workspace/agent_now.png'); r.snap('robot0_eye_in_hand','/workspace/eih_now.png')"

# openrua op 24
cat > dbg.py <<'EOF'
from rob import *
import time
r = Robot()
q0 = r.arm_q(); print("q0", np.round(q0,3))
tcp0 = r.tcp_world(q0)[0]; print("tcp0", tcp0.round(4))
sol = r.ik_world([-0.197, -0.139, 0.62], Q_DOWN_X)
print("ik sol", np.round(sol,3), "tcp of sol", r.tcp_world(sol)[0].round(4))
# send and sample joint states during execution
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = list(ARM)
pt = JointTrajectoryPoint(positions=[float(x) for x in sol]); pt.time_from_start = Duration(sec=4)
goal.trajectory.points=[pt]
send = r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(r.node, send, timeout_sec=60)
gh = send.result(); rf = gh.get_result_async()
t0=time.time()
while not rf.done():
    r.spin(0.1)
    if int((time.time()-t0)*2) % 4 == 0:
        q = r.arm_q(); print(f"t={time.time()-t0:.1f} err={np.abs(np.array(q)-np.array(sol)).max():.3f} q={np.round(q,2)}", flush=True)
res = rf.result().result
print("code", res.error_code, res.error_string)
for i in range(6):
    q = r.arm_q(); print(f"post {i} err={np.abs(np.array(q)-np.array(sol)).max():.4f} tcp={r.tcp_world(q)[0].round(4)}"); time.sleep(0.5)
EOF
timeout 900 python3 -u dbg.py 2>&1 | tail -40

# openrua op 25
python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace('''        code = rf.result().result.error_code if rf.result() else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move_q done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err''','''        res = rf.result().result if rf.result() else None
        code = res.error_code if res else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move_q done code={code} ({res and res.error_string}) max_joint_err={err:.4f}", flush=True)
        if (code != 0 or err > 0.02) and retries > 0:
            print("  retrying same goal", flush=True)
            return self.move_q(q, max(seconds, 4.0), retries=retries - 1)
        return code, err''')
s=s.replace("    def move_q(self, q, seconds=3.0, via=None):","    def move_q(self, q, seconds=4.0, via=None, retries=2):")
s=s.replace("    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):","    def move_tcp(self, pos_world, quat, seconds=4.0, seed=None):")
open("rob.py","w").write(s)
EOF
cat > step3.py <<'EOF'
from rob import *
r = Robot()
K = np.array([-0.197, -0.139])
print("open"); r.gripper(0.04)
print("descend")
r.move_tcp([K[0], K[1], 0.47], Q_DOWN_X, seconds=4.0)
r.snap("robot0_eye_in_hand", "/workspace/eih_k1.png")
r.snap("agentview", "/workspace/agent_k1.png")
print("DONE")
EOF
timeout 1200 python3 -u step3.py > step3.log 2>&1; cat step3.log

# openrua op 26
python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace('''    def move_tcp(self, pos_world, quat, seconds=4.0, seed=None):
        q = self.ik_world(pos_world, quat, at_tcp=True, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos_world}")
        self.move_q(q, seconds)''','''    def move_tcp(self, pos_world, quat, seconds=4.0, seed=None):
        q0 = np.array(self.arm_q())
        best = None
        for _ in range(4):  # prefer the IK branch closest to where we are
            q = self.ik_world(pos_world, quat, at_tcp=True, seed=seed)
            if q is None:
                continue
            d = np.abs(np.array(q) - q0).max()
            if best is None or d < best[0]:
                best = (d, q)
            if d < 0.6:
                break
        if best is None:
            raise RuntimeError(f"IK failed for {pos_world}")
        d, q = best
        seconds = max(seconds, d / 0.3)  # <= ~0.3 rad/s on the fastest joint
        print(f"  joint delta {d:.3f} rad -> {seconds:.1f}s", flush=True)
        self.move_q(q, seconds)''')
open("rob.py","w").write(s)
EOF
cat > step4.py <<'EOF'
from rob import *
r = Robot()
print("lift clear")
r.move_tcp([-0.30, -0.139, 0.75], Q_DOWN_X, seconds=4.0)
r.snap("agentview", "/workspace/agent_k3.png")
r.snap("birdview", "/workspace/bird3.png")
print("DONE")
EOF
timeout 1200 python3 -u step4.py > step4.log 2>&1; cat step4.log

# openrua op 27
cat > seg.py <<'EOF'
"""Segment objects above the table from the birdview depth; print world boxes."""
import numpy as np, cv2, rclpy, sys
from sensor_msgs.msg import Image
from rob import quat_to_R
def grab():
    rclpy.init(); node=rclpy.create_node("seg")
    got={}
    node.create_subscription(Image,"/birdview/depth/image_raw",lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    m=got["m"]; d=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width)
    rclpy.shutdown()
    fx=fy=579.4112549695428; cx=320; cy=240
    Rm=quat_to_R(0.7071,0.7071,0,0); t=np.array([-0.2,0,3.0])
    v,u=np.mgrid[0:480,0:640]
    return (np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@Rm.T+t).reshape(480,640,3)
if __name__=="__main__":
    P=grab(); np.save("/workspace/bird_world.npy",P)
    zs=P[...,2]
    zmax=float(sys.argv[1]) if len(sys.argv)>1 else 0.7
    mask=((zs>0.433)&(zs<zmax)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    for k in range(1,n):
        if stats[k,4]<5: continue
        pts=P[lab==k]
        print(f"blob {k}: area={stats[k,4]} px=({cent[k][0]:.0f},{cent[k][1]:.0f}) x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} c=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
EOF
timeout 120 python3 -u seg.py 0.7

# openrua op 28
timeout 60 python3 -u - <<'EOF'
import numpy as np
P=np.load("/workspace/bird_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.20)&(X<0.0)&(Y>-0.20)&(Y<-0.09)&(Z>0.433)&(Z<0.6)
pts=P[m]
for x0 in np.arange(-0.175,-0.02,0.01):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.01)]
    if len(s): print(f"x={x0:.3f} n={len(s)} y=[{s[:,1].min():.3f},{s[:,1].max():.3f}] w={s[:,1].max()-s[:,1].min():.3f} yc={s[:,1].mean():.3f} zmax={s[:,2].max():.3f}")
EOF

# openrua op 29
cat > dbg2.py <<'EOF'
from rob import *
import time
from rosgraph_msgs.msg import Clock
r = Robot()
clk={}
r.node.create_subscription(Clock, "/clock", lambda m: clk.__setitem__("t", m.clock.sec+m.clock.nanosec*1e-9), 1)
sol = r.ik_world([-0.06, -0.146, 0.60], Q_DOWN_Y)
q0=np.array(r.arm_q()); print("delta", np.round(np.array(sol)-q0,3))
goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names=list(ARM)
pt = JointTrajectoryPoint(positions=[float(x) for x in sol]); pt.time_from_start = Duration(sec=6)
goal.trajectory.points=[pt]
r.spin(0.2); print("clock at start", clk.get("t"))
send = r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(r.node, send, timeout_sec=60)
gh = send.result(); rf = gh.get_result_async()
t0=time.time(); last=-1
while not rf.done():
    r.spin(0.05)
    if time.time()-last > 1.0:
        last=time.time(); q=r.arm_q()
        print(f"wall={time.time()-t0:5.1f} sim={clk.get('t',0):8.3f} err={np.abs(np.array(q)-np.array(sol)).max():.3f}", flush=True)
res=rf.result().result; print("code", res.error_code, res.error_string, "sim", clk.get("t"))
q=r.arm_q(); print("final err", np.abs(np.array(q)-np.array(sol)).max().round(4), "tcp", r.tcp_world(q)[0].round(4))
EOF
timeout 900 python3 -u dbg2.py 2>&1 | tail -40

# openrua op 30
sed -i 's|        seconds = max(seconds, d / 0.3)  # <= ~0.3 rad/s on the fastest joint|        seconds = max(seconds, d / 0.1)  # machine fact: controller lags above ~0.1 rad/s|' rob.py && grep -n "d / 0.1" rob.py && cat > step5.py <<'EOF'
from rob import *
r = Robot()
N = np.array([-0.06, -0.146])   # ketchup neck (lying bottle)
print("pre-grasp above neck, fingers along Y")
r.move_tcp([N[0], N[1], 0.60], Q_DOWN_Y)
print("open"); r.gripper(0.04)
print("descend")
r.move_tcp([N[0], N[1], 0.44], Q_DOWN_Y)
r.snap("robot0_eye_in_hand", "/workspace/eih_k2.png")
r.snap("agentview", "/workspace/agent_k4.png")
print("DONE")
EOF
timeout 1500 python3 -u step5.py > step5.log 2>&1; cat step5.log

# openrua op 31
cat > step6.py <<'EOF'
from rob import *
r = Robot()
N = np.array([-0.06, -0.146])
print("close"); r.gripper(0.0)
print("gap:", round(r.finger_gap(),4))
print("lift")
r.move_tcp([N[0], N[1], 0.62], Q_DOWN_Y)
print("gap after lift:", round(r.finger_gap(),4))
r.snap("agentview", "/workspace/agent_k5.png")
r.snap("robot0_eye_in_hand", "/workspace/eih_k3.png")
print("DONE")
EOF
timeout 1500 python3 -u step6.py > step6.log 2>&1; cat step6.log

# openrua op 32
timeout 120 python3 -u seg.py 0.62 2>&1 | grep -v "blob 1:"; timeout 60 python3 -u - <<'EOF'
import numpy as np
P=np.load("/workspace/bird_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.30)&(X<0.05)&(Y>-0.25)&(Y<-0.05)&(Z>0.45)&(Z<0.72)
pts=P[m]
for z0 in np.arange(0.45,0.72,0.02):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
    if len(s): print(f"z={z0:.2f} n={len(s)} x=[{s[:,0].min():.3f},{s[:,0].max():.3f}] y=[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 33
timeout 120 python3 -u - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from rob import quat_to_R
rclpy.init(); node=rclpy.create_node("d")
got={}
node.create_subscription(Image,"/agentview/depth/image_raw",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
m=got["m"]; d=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width)
fx=fy=579.4112549695428; cx=320; cy=240
q=np.array([0.6182,0.6182,-0.3432,-0.3432]); q/=np.linalg.norm(q)
Rm=quat_to_R(*q); t=np.array([0.6066,0,0.96])
v,u=np.mgrid[0:480,0:640]
P=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@Rm.T+t).reshape(480,640,3)
X,Y,Z=P[...,0],P[...,1],P[...,2]
mm=(X>-0.25)&(X<0.02)&(Y>-0.25)&(Y<-0.05)&(Z>0.44)&(Z<0.66)
pts=P[mm]
for z0 in np.arange(0.44,0.66,0.02):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
    if len(s): print(f"z={z0:.2f} n={len(s)} x=[{s[:,0].min():.3f},{s[:,0].max():.3f}] y=[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
rclpy.shutdown()
EOF

# openrua op 34
timeout 60 python3 tools/perception/cam_snap.py sideview /workspace/side_k5.png && timeout 60 python3 tools/perception/cam_snap.py frontview /workspace/front_k5.png

# openrua op 35
cat > step7.py <<'EOF'
from rob import *
r = Robot()
Q45 = (0.9238795, 0.3826834, 0.0, 0.0)   # hand yaw +45deg, pointing down
print("lift higher")
r.move_tcp([-0.06, -0.146, 0.78], Q_DOWN_Y)
print("gap:", round(r.finger_gap(),4))
print("over basket, diagonal")
r.move_tcp([0.024, 0.284, 0.78], Q45)
print("gap:", round(r.finger_gap(),4))
r.snap("birdview", "/workspace/bird_k6.png")
r.snap("agentview", "/workspace/agent_k6.png")
print("DONE")
EOF
timeout 1500 python3 -u step7.py > step7.log 2>&1; cat step7.log

# openrua op 36
timeout 120 python3 -u - <<'EOF'
import numpy as np, cv2
import subprocess
from seg import grab
P=grab(); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.15)&(X<0.15)&(Y>0.10)&(Y<0.40)&(Z>0.66)&(Z<0.80)
pts=P[m]
for z0 in np.arange(0.66,0.80,0.02):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
    if len(s): print(f"z={z0:.2f} n={len(s)} x=[{s[:,0].min():.3f},{s[:,0].max():.3f}] y=[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
img=cv2.imread("/workspace/bird_k6.png"); crop=img[240:340,330:450]; cv2.imwrite("/workspace/bird_k6_crop.png", cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 37
cat > step8.py <<'EOF'
from rob import *
r = Robot()
Q45 = (0.9238795, 0.3826834, 0.0, 0.0)
print("lower over basket")
r.move_tcp([0.024, 0.284, 0.70], Q45)
print("release"); r.gripper(0.04)
print("gap:", round(r.finger_gap(),4))
print("retreat up")
r.move_tcp([0.024, 0.284, 0.85], Q45)
r.snap("agentview", "/workspace/agent_k7.png")
r.snap("frontview", "/workspace/front_k7.png")
print("DONE")
EOF
timeout 1500 python3 -u step8.py > step8.log 2>&1; cat step8.log

# openrua op 38
cat > step9.py <<'EOF'
from rob import *
r = Robot()
C = np.array([-0.153, 0.046])
print("above box")
r.move_tcp([C[0], C[1], 0.60], Q_DOWN_Y)
print("gap:", round(r.finger_gap(),4))
print("descend")
r.move_tcp([C[0], C[1], 0.44], Q_DOWN_Y)
r.snap("robot0_eye_in_hand", "/workspace/eih_c1.png")
print("DONE")
EOF
timeout 1500 python3 -u step9.py > step9.log 2>&1; cat step9.log

# openrua op 39
cat > step10.py <<'EOF'
from rob import *
r = Robot()
C = np.array([-0.153, 0.046])
print("close"); r.gripper(0.0)
print("gap:", round(r.finger_gap(),4))
print("lift")
r.move_tcp([C[0], C[1], 0.70], Q_DOWN_Y)
g = r.finger_gap(); print("gap after lift:", round(g,4))
if g < 0.02:
    raise SystemExit("grasp lost")
print("over basket")
r.move_tcp([-0.01, 0.25, 0.75], Q_DOWN_Y)
print("gap:", round(r.finger_gap(),4))
r.snap("birdview", "/workspace/bird_c2.png")
print("release"); r.gripper(0.04)
print("retreat")
r.move_tcp([-0.01, 0.25, 0.88], Q_DOWN_Y)
r.snap("agentview", "/workspace/agent_c3.png")
print("DONE")
EOF
timeout 1500 python3 -u step10.py > step10.log 2>&1; cat step10.log

# openrua op 40
timeout 900 python3 -u -c "
from rob import *
r=Robot(); r.move_tcp([-0.25,-0.20,0.85], Q_DOWN_Y); r.snap('birdview','/workspace/bird_final.png'); r.snap('agentview','/workspace/agent_final.png'); print('gap', r.finger_gap())
" 2>&1 | tail -3; timeout 120 python3 -u seg.py 0.7 | grep -v "blob 1:"; timeout 60 python3 - <<'EOF'
import numpy as np, cv2
P=np.load("/workspace/bird_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
# basket interior contents: points inside the basket footprint above the table but below the rim
m=(X>-0.075)&(X<0.05)&(Y>0.18)&(Y<0.32)&(Z>0.43)&(Z<0.60)
pts=P[m]; print("points inside basket footprint (z 0.43-0.60):", len(pts))
for z0 in np.arange(0.43,0.60,0.02):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
    if len(s): print(f"  z={z0:.2f} n={len(s)} x=[{s[:,0].min():.3f},{s[:,0].max():.3f}] y=[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# anything left at the original object locations?
for name,(x0,x1,y0,y1) in {"ketchup area":(-0.25,-0.02,-0.20,-0.09),"box area":(-0.20,-0.10,0.02,0.07)}.items():
    mm=(X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>0.433)&(Z<0.7)
    print(name, "points above table:", int(mm.sum()))
img=cv2.imread("/workspace/bird_final.png"); crop=img[250:330,340:430]; cv2.imwrite("/workspace/bird_final_crop.png", cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 41
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Measured quirks of the /workspace Panda sim workstation (IK tip yaw offset, planner frame == world, controller speed limit, table height)
metadata:
  type: project
---

Facts measured on the Franka Panda sim workstation in /workspace (Sept 2026), none of which the shipped docs state:

- `/compute_fk` and `/compute_ik` (frame_id empty) work in the WORLD frame, not panda_link0 — verified FK vs the TF chain (world->panda_link0 is (-0.51,0,0.42) but FK output already includes it). Do not add the base offset.
- The IK solver's tip is yawed -45° about z relative to `panda_hand` (it is effectively panda_link8). Request `q_des * Rz(+45°)` to land the HAND at `q_des`; verify with FK.
- `follow_joint_trajectory` returns `-5` (goal tolerance violated) and stops mid-way if the fastest joint exceeds ~0.1 rad/s; it knocked over a bottle this way. Duration ≥ max_joint_delta / 0.1 gives code 0 on the first attempt.
- Table top is at world z=0.425; the gripper open gap measures 0.072–0.080 m; the gripper holds objects rigidly (a bottle grasped by its cap stays horizontal).
- Working helper library was written to /workspace/rob.py (Robot class: fk/ik/move_tcp/gripper/snap) and /workspace/seg.py (birdview depth -> world blobs).

**Why:** each of these cost a failed motion or a debugging round to discover.
**How to apply:** reuse rob.py if it still exists; otherwise re-verify the IK yaw offset with one FK check before the first motion.
OPENRUA_EOF

# openrua op 42
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK tip yawed -45°, planner frame == world, keep joints ≤0.1 rad/s, table z=0.425" >> MEMORY.md && cat MEMORY.md
