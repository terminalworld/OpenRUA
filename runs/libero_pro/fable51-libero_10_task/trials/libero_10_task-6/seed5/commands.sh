#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; find /workspace/tools -type f; ls -la /workspace/.ros

# openrua op 2
cat /workspace/tools/action/fjt_send.py /workspace/tools/action/gripper_cmd.py /workspace/tools/perception/cam_snap.py /workspace/tools/perception/px2world.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo; timeout 20 ros2 topic echo /birdview/color/camera_info --once | head -20

# openrua op 6
mkdir -p "$(dirname /workspace/survey.py)"
cat > /workspace/survey.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print camera poses in world, world->panda_link0, and hand FK."""
import rclpy, yaml
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("survey")
buf = Buffer()
TransformListener(buf, node)
frames = set()
def on_tf(m):
    for t in m.transforms:
        frames.add((t.header.frame_id, t.child_frame_id))
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage, "/tf_static", on_tf, qos)
node.create_subscription(TFMessage, "/tf", on_tf, 100)
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
print("TF edges:")
for e in sorted(frames):
    print("  ", e)
for cam in ["agentview", "birdview", "frontview", "sideview", "robot0_eye_in_hand", "robot0_robotview", "galleryview", "paperview"]:
    f = f"{cam}_optical_frame"
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{cam}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as ex:
        print(f"{cam}: {type(ex).__name__}: {ex}")
for f in ["panda_link0", "panda_hand"]:
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"world->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as ex:
        print(f"world->{f}: {type(ex).__name__}: {ex}")

# FK
cli = node.create_client(GetPositionFK, "/compute_fk")
if cli.wait_for_service(timeout_sec=10) and "m" in js:
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = ["panda_hand"]
    arm = [f"panda_joint{i}" for i in range(1, 8)]
    seed = JointState()
    for n, p in zip(js["m"].name, js["m"].position):
        if n in arm:
            seed.name.append(n); seed.position.append(p)
    req.robot_state.joint_state = seed
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r:
        for ps in r.pose_stamped:
            p, q = ps.pose.position, ps.pose.orientation
            print(f"FK panda_hand (frame '{ps.header.frame_id}'): p=({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f}) err={r.error_code.val}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 7
timeout 90 python3 survey.py

# openrua op 8
mkdir -p "$(dirname /workspace/iktest.py)"
cat > /workspace/iktest.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Probe which frame /compute_ik interprets poses in."""
import sys
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("iktest")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
arm = [f"panda_joint{i}" for i in range(1, 8)]
cur = dict(zip(js["m"].name, js["m"].position))
seed = JointState()
for n in arm:
    seed.name.append(n); seed.position.append(cur[n])
cli = node.create_client(GetPositionIK, "/compute_ik")
cli.wait_for_service(timeout_sec=10)

def ik(x, y, z, q=(0.9996, 0.0, -0.0284, 0.0)):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.ik_link_name = "panda_hand"
    req.ik_request.timeout.sec = 2
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    r = fut.result()
    if r is None:
        return "timeout"
    if r.error_code.val != 1:
        return f"err {r.error_code.val}"
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [round(sol[j], 3) for j in arm]

print("current:", [round(cur[j], 3) for j in arm])
print("world coords (-0.053,0,0.7776):", ik(-0.053, 0.0, 0.7776))
print("base  coords (0.457,0,0.3576):", ik(0.457, 0.0, 0.3576))
rclpy.shutdown()
OPENRUA_EOF

# openrua op 9
timeout 150 python3 iktest.py

# openrua op 10
mkdir -p "$(dirname /workspace/locate.py)"
cat > /workspace/locate.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab birdview color+depth, back-project to world, and report object blobs.

Usage: python3 locate.py [cam=birdview]
Saves <cam>_pts.npz with world xyz per pixel and prints table-height stats plus
blob centroids for colour masks.
"""
import sys
import numpy as np
import rclpy, cv2
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
rclpy.init()
node = rclpy.create_node("locate")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while not all(k in got for k in "cdi") or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
color = br.imgmsg_to_cv2(got["c"], "bgr8")
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.savez(f"{cam}_pts.npz", pw=pw, color=color, depth=depth)
cv2.imwrite(f"{cam}.png", color)
valid = np.isfinite(depth) & (depth > 0)
print("world z percentiles:", np.percentile(pw[valid][:, 2], [1, 5, 50, 95, 99]))
# table height estimate: most common z
hist, edges = np.histogram(pw[valid][:, 2], bins=200)
zt = edges[np.argmax(hist)]
print("table z ~", zt)
above = valid & (pw[:, :, 2] > zt + 0.01)
n, lab, stats, cents = cv2.connectedComponentsWithStats(above.astype(np.uint8), 8)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30:
        continue
    m = lab == i
    p = pw[m]
    c = color[m].mean(0)
    print(f"blob {i}: area={stats[i,4]} px bbox(u,v,w,h)={stats[i,:4].tolist()} "
          f"centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) world x[{p[:,0].min():.3f},{p[:,0].max():.3f}] "
          f"y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) BGR={c.round(0)}")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 11
timeout 120 python3 locate.py birdview

# openrua op 12
timeout 60 python3 -c "
import numpy as np, cv2
d=np.load('birdview_pts.npz'); pw=d['pw']; color=d['color']
z=pw[:,:,2]; valid=np.isfinite(z)&(z>0.2)
hist,edges=np.histogram(z[valid],bins=300)
zt=edges[np.argmax(hist)]; print('table z ~',zt, 'count',hist.max())
above=valid&(z>zt+0.012)&(z<zt+0.4)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; p=pw[m]; c=color[m].mean(0)
    print(f'blob {i}: area={stats[i,4]} bbox={stats[i,:4].tolist()} cpx=({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) BGR={c.round(0)}')
# table extents
tab=valid&(abs(z-zt)<0.01)
p=pw[tab]; print('table x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max())
"


# openrua op 13
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 14
timeout 120 python3 locate.py birdview >/dev/null && timeout 60 python3 -c "
import numpy as np, cv2
d=np.load('birdview_pts.npz'); pw=d['pw']; color=d['color']
z=pw[:,:,2]; valid=np.isfinite(z)&(z>0.2)
hist,edges=np.histogram(z[valid],bins=300)
zt=edges[np.argmax(hist)]; print('table z ~',zt)
above=valid&(z>zt+0.012)&(z<zt+0.4)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; p=pw[m]; c=color[m].mean(0)
    print(f'blob {i}: area={stats[i,4]} bbox={stats[i,:4].tolist()} cpx=({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) BGR={c.round(0)}')
"


# openrua op 15
timeout 120 python3 locate.py agentview >/dev/null && timeout 60 python3 -c "
import numpy as np, cv2
d=np.load('agentview_pts.npz'); pw=d['pw']; color=d['color']
z=pw[:,:,2]; valid=np.isfinite(z)&(z>0.2)
zt=0.425
above=valid&(z>zt+0.008)&(z<zt+0.4)&(pw[:,:,0]>-0.35)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; p=pw[m]; c=color[m].mean(0)
    print(f'blob {i}: area={stats[i,4]} bbox={stats[i,:4].tolist()} cpx=({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) BGR={c.round(0)}')
# pudding region pixel check
for (u,v) in [(383,288),(375,280),(390,295),(383,275)]:
    print((u,v), pw[v,u].round(3), color[v,u])
"


# openrua op 16
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 -c "
import numpy as np
d=np.load('birdview_pts.npz'); pw=d['pw']; color=d['color']
z=pw[:,:,2]
# red mug: rows/cols in birdview where z>0.5 near x~-0.22
m=(z>0.50)&(z<0.60)&(pw[:,:,0]<-0.15)&(pw[:,:,0]>-0.3)&(abs(pw[:,:,1])<0.12)
vs,us=np.nonzero(m)
print('red mug top pixels:',len(vs),'u',us.min(),us.max(),'v',vs.min(),vs.max())
for v in range(vs.min(),vs.max()+1):
    row=us[vs==v]; 
    print(v, row.min(), row.max(), 'x=%.3f'%pw[v,row.min(),0], 'y[%.3f,%.3f]'%(pw[v,row.min(),1],pw[v,row.max(),1]), ' '.join('%d'%(z[v,u]>0.5) for u in range(us.min(),us.max()+1)))
"


# openrua op 17
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control library: IK (world frame, verified), trajectory, gripper, FK.

Usage as CLI:
  python3 arm.py tcp X Y Z [secs]          # move TCP (fingertip centre) to X Y Z, hand pointing down, fingers along world y
  python3 arm.py tcp X Y Z secs yawdeg     # same, with yaw about world z (0 = fingers along y)
  python3 arm.py grip open|close
  python3 arm.py state                     # print joints, hand pose, tcp pose, finger gap
"""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP_OFF = 0.1034


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw_deg=0.0):
    """Hand z pointing to world -z; yaw about world z. yaw=0 -> fingers along world y."""
    # base: 180 deg about x -> (1,0,0,0); then yaw about world z: q = qz * qx
    h = math.radians(yaw_deg) / 2
    qz = (0.0, 0.0, math.sin(h), math.cos(h))
    qx = (1.0, 0.0, 0.0, 0.0)
    # quaternion product qz * qx
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_lib")
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory,
                                 "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli.wait_for_service(timeout_sec=20)
        self.fk_cli.wait_for_service(timeout_sec=20)
        self.traj.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, n=5, dt=0.1):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=dt)

    def joints(self):
        self._js = None
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        t0 = time.time()
        while self._wr is None and time.time() - t0 < 3:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return (f.x, f.y, f.z)

    def fk(self, q=None):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = JointState(name=ARM, position=list(q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p, o = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation
        return np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w)

    def tcp(self):
        p, q = self.fk()
        R = quat_to_R(*q)
        return p + TCP_OFF * R[:, 2], q

    def ik_hand(self, pos, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state = JointState(name=ARM, position=list(seed))
        req.ik_request.timeout.sec = 3
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, pos, quat, seed=None):
        R = quat_to_R(*quat)
        hand = np.array(pos, float) - TCP_OFF * R[:, 2]
        return self.ik_hand(hand, quat, seed)

    def move_joints(self, q, secs=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = self.arm_q()
        err = max(abs(a - b) for a, b in zip(cur, q))
        return code, err

    def move_tcp(self, pos, yaw_deg=0.0, secs=3.0, quat=None):
        quat = quat or down_quat(yaw_deg)
        q = self.ik_tcp(pos, quat)
        if q is None:
            print(f"IK FAILED for tcp {pos}")
            return False
        # guard against wild joint flips: report the joint delta
        cur = self.arm_q()
        delta = [abs(a - b) for a, b in zip(cur, q)]
        code, err = self.move_joints(q, secs)
        t, _ = self.tcp()
        print(f"move_tcp -> {np.round(pos,3)} code={code} joint_err={err:.4f} "
              f"maxdelta={max(delta):.2f} tcp_now={np.round(t,3)}")
        return code == 0 and err < 0.02

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        self.spin(10, 0.1)
        gap = self.finger_gap()
        print(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "tcp":
        x, y, z = map(float, sys.argv[2:5])
        secs = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
        yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
        ok = a.move_tcp((x, y, z), yaw, secs)
        sys.exit(0 if ok else 1)
    elif cmd == "grip":
        a.gripper(0.04 if sys.argv[2] == "open" else 0.0)
    elif cmd == "state":
        q = a.arm_q()
        print("joints", np.round(q, 3).tolist())
        p, o = a.fk(q)
        print("hand", np.round(p, 4).tolist(), np.round(o, 4).tolist())
        t, _ = a.tcp()
        print("tcp", np.round(t, 4).tolist())
        print("finger gap", round(a.finger_gap(), 4))
        print("wrench", a.wrench())
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 18
timeout 120 python3 arm.py state

# openrua op 19
timeout 600 python3 arm.py tcp -0.225 0.044 0.68 3

# openrua op 20
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 21
timeout 600 python3 arm.py tcp -0.225 0.044 0.59 2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 22
timeout 600 python3 arm.py tcp -0.225 0.044 0.555 2 && timeout 120 python3 arm.py state | grep -E "tcp|wrench"

# openrua op 23
timeout 300 python3 arm.py grip close && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
timeout 600 python3 arm.py tcp -0.225 0.044 0.72 3 && timeout 120 python3 arm.py state | grep -E "gap|wrench" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 25
timeout 120 python3 arm.py state | grep -E "gap|wrench|tcp" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 26
timeout 600 python3 arm.py tcp 0.135 0.041 0.72 4; timeout 120 python3 arm.py state | grep -E "gap|tcp"

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        code, err = self.move_joints(q, secs)\\n        t, _ = self.tcp()", "new_string": "        code, err = self.move_joints(q, secs)\\n        # tolerance violations on long goals are usually controller lag;\\n        # resending the same goal converges (machine fact, docs/30-action.md)\\n        for _ in range(3):\\n            if code == 0 and err < 0.02:\\n                break\\n            print(f\\"  retry: code={code} err={err:.4f}\\")\\n            code, err = self.move_joints(q, max(secs, 2.0))\\n        t, _ = self.tcp()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
timeout 900 python3 arm.py tcp 0.135 0.041 0.72 4; timeout 120 python3 arm.py state | grep -E "gap|tcp"

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 30
timeout 120 python3 locate.py birdview >/dev/null && timeout 60 python3 -c "
import numpy as np
d=np.load('birdview_pts.npz'); pw=d['pw']; z=pw[:,:,2]
# mug top ring: z between 0.70 and 0.75 (rim at tcp z + 0.02 ~ 0.74)
m=(z>0.68)&(z<0.76)&(pw[:,:,0]>0.0)&(pw[:,:,0]<0.3)&(abs(pw[:,:,1])<0.2)
p=pw[m]; print('rim pts',len(p),'x[%.3f,%.3f] y[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()), 'center approx', ((p[:,0].min()+p[:,0].max())/2, (p[:,1].min()+p[:,1].max())/2))
# plate
m2=(z>0.44)&(z<0.48)&(pw[:,:,0]>0.0)&(pw[:,:,0]<0.3)&(abs(pw[:,:,1])<0.2)
p=pw[m2]; print('plate pts',len(p),'x[%.3f,%.3f] y[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
"


# openrua op 31
timeout 600 python3 arm.py tcp 0.135 0.041 0.59 3; timeout 120 python3 arm.py state | grep -E "gap|tcp|wrench"

# openrua op 32
timeout 300 python3 arm.py grip open && timeout 600 python3 arm.py tcp 0.135 0.041 0.72 3 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 33
timeout 120 python3 locate.py birdview >/dev/null && timeout 60 python3 -c "
import numpy as np, cv2
d=np.load('birdview_pts.npz'); pw=d['pw']; color=d['color']; z=pw[:,:,2]
valid=np.isfinite(z)&(z>0.2)
above=valid&(z>0.437)&(z<0.8)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; p=pw[m]; c=color[m].mean(0)
    print(f'blob {i}: area={stats[i,4]} bbox={stats[i,:4].tolist()} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) BGR={c.round(0)}')
# pudding profile
m=(z>0.45)&(z<0.48)&(pw[:,:,0]>-0.12)&(pw[:,:,0]<0.03)&(pw[:,:,1]>0.04)&(pw[:,:,1]<0.14)
p=pw[m]; print('pudding top pts',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
vs,us=np.nonzero(m); print('px u',us.min(),us.max(),'v',vs.min(),vs.max())
"


# openrua op 34
timeout 900 python3 arm.py tcp -0.036 0.0875 0.57 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 35
timeout 600 python3 arm.py tcp -0.036 0.0875 0.445 2.5 && timeout 300 python3 arm.py grip close

# openrua op 36
timeout 900 python3 arm.py tcp -0.036 0.0875 0.60 3 && timeout 120 python3 arm.py state | grep gap && timeout 900 python3 arm.py tcp 0.135 0.15 0.60 4 90 && timeout 120 python3 arm.py state | grep -E "gap|hand"

# openrua op 37
timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 38
timeout 600 python3 arm.py tcp 0.135 0.15 0.455 3 90 && timeout 300 python3 arm.py grip open && timeout 600 python3 arm.py tcp 0.135 0.15 0.62 3 90 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 39
timeout 900 python3 -c "
from arm import Arm
import rclpy
a=Arm()
home=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
for i in range(3):
    code,err=a.move_joints(home,4.0); print('home code',code,'err',round(err,4))
    if code==0 and err<0.02: break
print('tcp',a.tcp()[0].round(3))
rclpy.shutdown()
" && timeout 120 python3 locate.py birdview >/dev/null && timeout 60 python3 -c "
import numpy as np, cv2
d=np.load('birdview_pts.npz'); pw=d['pw']; color=d['color']; z=pw[:,:,2]
valid=np.isfinite(z)&(z>0.2)
above=valid&(z>0.437)&(z<0.8)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; p=pw[m]; c=color[m].mean(0)
    print(f'blob {i}: area={stats[i,4]} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) BGR={c.round(0)}')
print('--- plate (z 0.44-0.46, x>0)')
m=(z>0.44)&(z<0.462)&(pw[:,:,0]>0.03)&(pw[:,:,0]<0.25)&(abs(pw[:,:,1])<0.1)
p=pw[m]; print('plate x[%.3f,%.3f] y[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
print('--- red mug rim (z>0.55, x>0)')
m=(z>0.55)&(z<0.6)&(pw[:,:,0]>0.0)&(pw[:,:,0]<0.3)
p=pw[m]; print('mug x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].max()))
print('--- pudding (z 0.45-0.48, y>0.1)')
m=(z>0.45)&(z<0.48)&(pw[:,:,0]>0.0)&(pw[:,:,0]<0.3)&(pw[:,:,1]>0.09)
p=pw[m]; print('pudding x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].max()))
"

# openrua op 40
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview
