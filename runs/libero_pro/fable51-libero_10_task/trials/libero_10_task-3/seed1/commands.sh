#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 245 245; timeout 60 python3 tools/perception/px2world.py agentview 340 200; timeout 60 python3 tools/perception/px2world.py agentview 340 180

# openrua op 6
cat > tfq.py <<'EOF'
import sys, rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tfq"); b = Buffer(); TransformListener(b, n)
pairs = [tuple(a.split(":")) for a in sys.argv[1:]]
import time
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.1)
for a, c in pairs:
    try:
        t = b.lookup_transform(a, c, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{a}->{c}: xyz=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"{a}->{c}: FAIL {e}")
print(b.all_frames_as_string())
EOF
timeout 60 python3 tfq.py world:panda_link0 world:panda_hand panda_link0:panda_hand world:agentview_optical_frame world:robot0_eye_in_hand_optical_frame

# openrua op 7
cat > cloud.py <<'EOF'
"""Dump a camera's full depth frame as world-frame points: <cam>_xyz.npy (H,W,3)."""
import sys, numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); n = rclpy.create_node("cloud"); b = Buffer(); TransformListener(b, n)
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while "d" not in got or "i" not in got or not b.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(n, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
H, W = d.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
t = b.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
pw = pc @ R.T + T
np.save(f"{cam}_xyz.npy", pw)
print(cam, pw.shape, "depth range", np.nanmin(d), np.nanmax(d))
EOF
timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py robot0_eye_in_hand && timeout 120 python3 cloud.py frontview

# openrua op 8
cat > q.py <<'EOF'
import numpy as np, sys
P = np.load("agentview_xyz.npy")
def at(u,v): print(f"agentview px({u},{v}) -> {P[v,u].round(4)}")
# table surface samples
for (u,v) in [(300,400),(150,450),(500,450),(250,350)]: at(u,v)
print("--- bottle column (u=340):")
for v in range(150,245,5): at(340,v)
print("--- bottle body row v=210:")
for u in range(320,360,3): at(u,210)
EOF
python3 q.py

# openrua op 9
cat > q2.py <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_xyz.npy")
Z = P[...,2]
# bottle: anything above table+5cm within x in [-0.3,0], y in [-0.1,0.15], excluding cabinet (x>0)
m = (Z > 0.95) & (P[...,0] > -0.3) & (P[...,0] < 0.0) & (P[...,1] > -0.1) & (P[...,1] < 0.15)
pts = P[m]
print("bottle pts", pts.shape)
for lo,hi in [(0.95,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.1),(1.1,1.2)]:
    s = pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f"z[{lo},{hi}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
# table height under eye-in-hand
t = P[(Z>0.85)&(Z<0.95)&(P[...,0]<0.0)]
print("table z", np.median(t[:,2]))
# cabinet/drawer region: x>0
c = P[(P[...,0]>0.0)&(Z>0.905)]
print("cabinet pts", c.shape)
for lo,hi in [(0.905,0.95),(0.95,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.2),(1.2,1.4)]:
    s = c[(c[:,2]>=lo)&(c[:,2]<hi)]
    if len(s): print(f"z[{lo},{hi}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF
python3 q2.py

# openrua op 10
python3 -c "
import cv2
im=cv2.imread('agentview.png'); c=im[150:400,330:640]; cv2.imwrite('crop_agent.png', cv2.resize(c,None,fx=2,fy=2))
im=cv2.imread('frontview.png'); c=im[250:430,320:600]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=2,fy=2))
"

# openrua op 11
cat > q3.py <<'EOF'
import numpy as np
P = np.load("agentview_xyz.npy")
def at(u,v): print(f"px({u},{v}) -> {P[v,u].round(4)}")
print("drawer interior floor / walls, column u=420:")
for v in range(245,390,5): at(420,v)
print("row v=320:")
for u in range(350,500,5): at(u,320)
print("cabinet top row v=200:")
for u in range(470,640,10): at(u,200)
print("handles: bottom drawer handle ~(355,310), mid (470,265), top (470,220)")
for (u,v) in [(352,300),(355,310),(358,320),(468,260),(472,270),(468,215),(472,225)]: at(u,v)
EOF
python3 q3.py

# openrua op 12
timeout 120 python3 cloud.py birdview && cat > q4.py <<'EOF'
import numpy as np
P = np.load("birdview_xyz.npy"); Z=P[...,2]
# grid summary: for region x in [-0.25,0.3], y in [-0.05,0.5], print max z per 2cm cell
xs = np.arange(-0.25,0.30,0.02); ys=np.arange(-0.05,0.50,0.02)
print("rows=x, cols=y (cm*100 of z above 0.90; '.' = table)")
print("      "+" ".join(f"{int(y*100):3d}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(P[...,0]>=x)&(P[...,0]<x+0.02)&(P[...,1]>=y)&(P[...,1]<y+0.02)
        if m.sum()==0: row.append("  -"); continue
        z=Z[m].max()
        row.append("  ." if z<0.905 else f"{int((z-0.9)*100):3d}")
    print(f"{x:5.2f} "+" ".join(row))
EOF
python3 q4.py

# openrua op 13
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control library for this Panda: state, FK/IK, trajectories, gripper.

CLI:
  python3 ctl.py state                       # joints + hand/TCP pose (world)
  python3 ctl.py movej j1,...,j7 [sec]       # joint move
  python3 ctl.py move x y z rx ry rz [sec] [--tcp]  # world pose, rotvec (deg) about world axes
  python3 ctl.py grip open|close
"""
import sys, math, time
import numpy as np
import rclpy, yaml
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # world->panda_link0 (TF, identity rotation)
TCP = float(M["hand"]["tcp_offset_m"])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def hand_pose(self, q=None):
        """FK: hand pose in WORLD -> (pos[3], Rot)."""
        if q is None:
            q = self.arm_q()
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp_pose(self, q=None):
        pos, R = self.hand_pose(q)
        return pos + TCP * R.as_matrix()[:, 2], R

    # ---------------- IK
    def solve_ik(self, pos_world, R, seed=None, tcp=False, tries=3):
        """pos_world: hand (or TCP if tcp=True) position in world; R: scipy Rotation."""
        pos_world = np.asarray(pos_world, float)
        if tcp:
            pos_world = pos_world - TCP * R.as_matrix()[:, 2]
        p_base = pos_world - BASE_IN_WORLD
        q = R.as_quat()
        self.ik.wait_for_service(5)
        seed = list(seed if seed is not None else self.arm_q())
        last = None
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            last = None if r is None else r.error_code.val
            # perturb seed and retry
            seed = [s + np.random.uniform(-0.3, 0.3) for s in seed]
        raise RuntimeError(f"IK failed (code {last}) for world pos {pos_world.round(3)}")

    # ---------------- motion
    def movej(self, targets, secs):
        """targets: list of joint vectors (waypoints); secs: list of time_from_start or single total."""
        if not isinstance(targets[0], (list, tuple, np.ndarray)):
            targets = [targets]
        if not isinstance(secs, (list, tuple)):
            n = len(targets)
            secs = [secs * (i + 1) / n for i in range(n)]
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for tq, ts in zip(targets, secs):
            pt = JointTrajectoryPoint(positions=[float(v) for v in tq])
            pt.time_from_start = Duration(sec=int(ts), nanosec=int((ts % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal not accepted")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        code = res.result().result.error_code if res.result() else None
        q = np.array(self.arm_q())
        err = np.abs(q - np.array(targets[-1])).max()
        print(f"  movej: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move(self, pos_world, R, secs=3.0, tcp=False, seed=None):
        q = self.solve_ik(pos_world, R, seed=seed, tcp=tcp)
        code, err = self.movej(q, secs)
        p, Rr = self.tcp_pose() if tcp else self.hand_pose()
        print(f"  reached {'tcp' if tcp else 'hand'} pos={p.round(4)} target={np.round(pos_world,4)} "
              f"rot_err_deg={np.degrees((Rr.inv()*R).magnitude()):.2f}")
        return q

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def R_from_axes(zaxis, yaxis):
    """Hand rotation with hand Z (approach) and hand Y (finger opening) given in world."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    y = np.asarray(yaxis, float); y = y - z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return Rot.from_matrix(np.column_stack([x, y, z]))


def fmt(pos, R):
    return f"pos={np.round(pos,4)} quat(xyzw)={np.round(R.as_quat(),4)} Z={np.round(R.as_matrix()[:,2],3)} Y={np.round(R.as_matrix()[:,1],3)}"


if __name__ == "__main__":
    a = sys.argv[1:]
    c = Ctl()
    if not a or a[0] == "state":
        j = c.joints()
        print("joints:", {k: round(v, 4) for k, v in j.items()})
        print("hand :", fmt(*c.hand_pose()))
        print("tcp  :", fmt(*c.tcp_pose()))
    elif a[0] == "movej":
        q = [float(x) for x in a[1].split(",")]
        c.movej(q, float(a[2]) if len(a) > 2 else 3.0)
    elif a[0] == "move":
        x, y, z, rx, ry, rz = map(float, a[1:7])
        secs = float(a[7]) if len(a) > 7 and not a[7].startswith("--") else 3.0
        R = Rot.from_rotvec(np.radians([rx, ry, rz]))
        c.move([x, y, z], R, secs, tcp="--tcp" in a)
    elif a[0] == "grip":
        c.gripper(GRIP["open_m"] if a[1] == "open" else GRIP["closed_m"])
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 14
timeout 120 python3 ctl.py state

# openrua op 15
cat > iktest.py <<'EOF'
import numpy as np, rclpy
from ctl import *
c = Ctl()
q0 = np.array(c.arm_q())
pos_fk_raw, R = c.hand_pose(); pos_fk_raw = pos_fk_raw - BASE_IN_WORLD
print("FK raw:", pos_fk_raw.round(4))
for label, p in [("raw(FK frame)", pos_fk_raw), ("raw - base", pos_fk_raw - BASE_IN_WORLD), ("raw + base", pos_fk_raw + BASE_IN_WORLD)]:
    try:
        # bypass BASE subtraction: pass p + BASE so solve_ik subtracts it back
        q = c.solve_ik(p + BASE_IN_WORLD, R, seed=q0, tries=1)
        print(label, "-> sol err", np.abs(np.array(q)-q0).max().round(4))
    except Exception as e:
        print(label, "->", e)
rclpy.shutdown()
EOF
timeout 300 python3 iktest.py

# openrua op 16
python3 - <<'EOF'
s = open("ctl.py").read()
s = s.replace('BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # world->panda_link0 (TF, identity rotation)',
              'BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # measured: /compute_fk and /compute_ik work in WORLD coords here')
old = s[s.index("        seed = list(seed if seed is not None else self.arm_q())"):s.index("    # ---------------- motion")]
new = '''        seed0 = list(seed if seed is not None else self.arm_q())
        seed = list(seed0)
        last = None
        sols = []
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                sols.append([sol[j] for j in JOINTS])
                if len(sols) >= 2 and k >= 1:
                    break
            else:
                last = None if r is None else r.error_code.val
            seed = [s + np.random.uniform(-0.2, 0.2) for s in seed0]
        if sols:
            d = [np.abs(np.array(s_) - np.array(seed0)).sum() for s_ in sols]
            return sols[int(np.argmin(d))]
        raise RuntimeError(f"IK failed (code {last}) for world pos {pos_world.round(3)}")

'''
s = s.replace(old, new)
s = s.replace("def solve_ik(self, pos_world, R, seed=None, tcp=False, tries=3):", "def solve_ik(self, pos_world, R, seed=None, tcp=False, tries=4):")
open("ctl.py","w").write(s)
EOF
timeout 300 python3 iktest.py

# openrua op 17
cat > iktest2.py <<'EOF'
import numpy as np, rclpy
from ctl import *
c = Ctl()
q0 = np.array(c.arm_q()); print("q0 ", q0.round(3))
pos, R = c.hand_pose()
for i in range(4):
    q = c.solve_ik(pos, R, seed=q0, tries=1)
    print("sol", np.array(q).round(3), " fk:", c.hand_pose(q)[0].round(4))
rclpy.shutdown()
EOF
timeout 300 python3 iktest2.py

# openrua op 18
cat > iktest3.py <<'EOF'
import numpy as np, rclpy
from ctl import *
c = Ctl()
q0 = np.array(c.arm_q())
pos, R = c.hand_pose()
print("cur  :", fmt(pos, R))
q = c.solve_ik(pos, R, seed=q0, tries=1)
p2, R2 = c.hand_pose(q)
print("sol  :", fmt(p2, R2), " rot err deg:", np.degrees((R.inv()*R2).magnitude()).round(2))
rclpy.shutdown()
EOF
timeout 300 python3 iktest3.py

# openrua op 19
python3 - <<'EOF'
s = open("ctl.py").read()
s = s.replace('            req.ik_request.group_name = M["planning"]["group"]\n',
              '            req.ik_request.group_name = M["planning"]["group"]\n            req.ik_request.ik_link_name = "panda_hand"\n')
open("ctl.py","w").write(s)
EOF
timeout 300 python3 iktest3.py

# openrua op 20
cat > step1.py <<'EOF'
import numpy as np, rclpy
from ctl import *
c = Ctl()
c.gripper(GRIP["open_m"])
R_down = R_from_axes(zaxis=[0,0,-1], yaxis=[0,-1,0])   # same as current: hand X = world X
c.move([-0.05, 0.15, 1.36], R_down, secs=4.0)
print(fmt(*c.hand_pose()))
rclpy.shutdown()
EOF
timeout 600 python3 step1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look1.png && timeout 120 python3 cloud.py robot0_eye_in_hand

# openrua op 21
cat > q5.py <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_xyz.npy"); Z=P[...,2]
xs = np.arange(-0.16,0.18,0.02); ys=np.arange(-0.06,0.30,0.02)
print("max z-0.90 in cm per 2cm cell; rows=x cols=y")
print("      "+" ".join(f"{int(round(y*100)):3d}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(P[...,0]>=x)&(P[...,0]<x+0.02)&(P[...,1]>=y)&(P[...,1]<y+0.02)&(Z<1.3)
        if m.sum()==0: row.append("  -"); continue
        z=Z[m].max()
        row.append("  ." if z<0.905 else f"{int(round((z-0.9)*100)):3d}")
    print(f"{x:5.2f} "+" ".join(row))
# fine measurement of drawer floor extents (z in [0.915,0.935])
f = P[(Z>0.915)&(Z<0.935)&(P[...,0]>-0.15)&(P[...,0]<0.15)&(P[...,1]>0.0)]
print("floor x[%.3f,%.3f] y[%.3f,%.3f] z=%.4f"%(f[:,0].min(),f[:,0].max(),f[:,1].min(),f[:,1].max(),np.median(f[:,2])))
w = P[(Z>0.97)&(Z<0.99)&(P[...,0]>-0.15)&(P[...,0]<0.15)&(P[...,1]>0.0)&(P[...,1]<0.21)]
print("wall tops x[%.3f,%.3f] y[%.3f,%.3f] z=%.4f"%(w[:,0].min(),w[:,0].max(),w[:,1].min(),w[:,1].max(),np.median(w[:,2])))
# per-y profile along the drawer center x=0: what's the top z
print("profile x in [-0.01,0.01]:")
for y in np.arange(0.0,0.30,0.01):
    m=(P[...,0]>-0.01)&(P[...,0]<0.01)&(P[...,1]>=y)&(P[...,1]<y+0.01)&(Z<1.3)
    if m.sum(): print(f"  y={y:.2f} zmax={Z[m].max():.3f} zmin={Z[m].min():.3f} n={m.sum()}")
EOF
python3 q5.py

# openrua op 22
timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png && timeout 120 python3 cloud.py agentview && python3 - <<'EOF'
import numpy as np
P = np.load("agentview_xyz.npy")
for v in range(140,260,5):
    print(f"px(340,{v}) -> {P[v,340].round(4)}")
EOF

# openrua op 23
python3 - <<'EOF'
import numpy as np
P = np.load("agentview_xyz.npy"); Z=P[...,2]
m = (P[...,0]>-0.22)&(P[...,0]<-0.10)&(P[...,1]>-0.02)&(P[...,1]<0.09)&(Z>0.905)&(Z<1.3)
b = P[m]
print("bottle pts", len(b), "zmax", b[:,2].max().round(4))
for lo in np.arange(0.90,1.08,0.01):
    s = b[(b[:,2]>=lo)&(b[:,2]<lo+0.01)]
    if len(s): print(f" z[{lo:.2f}] n={len(s):3d} xmin={s[:,0].min():.4f} y[{s[:,1].min():.4f},{s[:,1].max():.4f}] ywidth={s[:,1].max()-s[:,1].min():.4f}")
EOF

# openrua op 24
python3 - <<'EOF'
import numpy as np
P = np.load("agentview_xyz.npy"); Z=P[...,2]
# hand is at (-0.05,0.15,1.358) top-down, fingers along y. Measure everything near there.
for lo,hi in [(1.20,1.26),(1.26,1.30),(1.30,1.34),(1.34,1.38),(1.38,1.42),(1.42,1.46),(1.46,1.5)]:
    m=(Z>=lo)&(Z<hi)&(P[...,0]>-0.2)&(P[...,0]<0.15)&(P[...,1]>-0.1)&(P[...,1]<0.45)
    s=P[m]
    if len(s): print(f"z[{lo:.2f},{hi:.2f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 25
python3 -c "
import cv2
im=cv2.imread('agent2.png'); c=im[0:130,340:560]; cv2.imwrite('crop_hand.png', cv2.resize(c,None,fx=3,fy=3))
"

# openrua op 26
cat > step2.py <<'EOF'
import numpy as np, rclpy
from ctl import *
c = Ctl()
th = np.radians(17)
Zh = [0, np.cos(th), -np.sin(th)]          # look toward +y, 17 deg down
R = R_from_axes(zaxis=Zh, yaxis=[0, -np.sin(th), -np.cos(th)])  # X_h = +x world
print("target R X:", R.as_matrix()[:,0].round(3))
c.move([-0.08, -0.12, 1.08], R, secs=4.0)
print(fmt(*c.hand_pose()))
rclpy.shutdown()
EOF
timeout 600 python3 step2.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look2.png && timeout 120 python3 cloud.py robot0_eye_in_hand

# openrua op 27
cat > step2b.py <<'EOF'
import numpy as np, rclpy
from ctl import *
c = Ctl()
th = np.radians(17)
R = R_from_axes(zaxis=[0, np.cos(th), -np.sin(th)], yaxis=[0, -np.sin(th), -np.cos(th)])
q0 = np.array(c.arm_q()); print("now:", q0.round(3))
q = np.array(c.solve_ik([-0.08, -0.12, 1.08], R))
print("sol:", q.round(3)); print("diff:", (q-q0).round(3))
code, err = c.movej(q, 4.0)
print("after:", np.array(c.arm_q()).round(3))
print(fmt(*c.hand_pose()))
rclpy.shutdown()
EOF
timeout 600 python3 step2b.py

# openrua op 28
timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -20; timeout 60 python3 ctl.py state

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look3.png && timeout 120 python3 cloud.py robot0_eye_in_hand

# openrua op 30
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_xyz.npy"); Z=P[...,2]
ok = np.isfinite(Z)&(np.linalg.norm(P,axis=2)<5)
m = ok&(P[...,0]>-0.09)&(P[...,0]<0.09)&(P[...,1]>0.10)&(Z>0.90)&(Z<1.0)
s = P[m]
print("pts inside drawer-ish region:", len(s))
for y in np.arange(0.10,0.40,0.01):
    t = s[(s[:,1]>=y)&(s[:,1]<y+0.01)]
    if len(t): print(f" y={y:.2f} n={len(t):4d} z[{t[:,2].min():.3f},{t[:,2].max():.3f}] x[{t[:,0].min():.3f},{t[:,0].max():.3f}]")
EOF

# openrua op 31
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_xyz.npy"); Z=P[...,2]
ok = np.isfinite(Z)&(np.linalg.norm(P,axis=2)<5)
m = ok&(P[...,0]>-0.10)&(P[...,0]<0.10)&(P[...,1]>0.15)&(P[...,1]<0.224)&(Z>0.99)&(Z<1.13)
s = P[m]
print("handle pts:", len(s))
for z in np.arange(0.99,1.13,0.01):
    t = s[(s[:,2]>=z)&(s[:,2]<z+0.01)]
    if len(t): print(f" z={z:.2f} n={len(t):4d} y[{t[:,1].min():.3f},{t[:,1].max():.3f}] x[{t[:,0].min():.3f},{t[:,0].max():.3f}]")
# cabinet face y
f = P[ok&(P[...,0]>-0.10)&(P[...,0]<0.10)&(Z>1.04)&(Z<1.06)&(P[...,1]>0.2)&(P[...,1]<0.3)]
print("face-ish y median at z~1.05:", np.median(f[:,1]).round(4), "min", f[:,1].min().round(4))
EOF

# openrua op 32
cat >> ctl.py <<'EOF'


def move_checked(c, pos, R, secs=3.0, tcp=True, tol_m=0.01, tol_deg=3.0, retries=1):
    """IK -> verify FK of solution -> execute -> verify reached. Raises on mismatch."""
    for attempt in range(retries + 1):
        q = c.solve_ik(pos, R, tcp=tcp)
        p_fk, R_fk = (c.tcp_pose(q) if tcp else c.hand_pose(q))
        perr = np.linalg.norm(p_fk - np.asarray(pos)); rerr = np.degrees((R_fk.inv() * R).magnitude())
        if perr > tol_m or rerr > tol_deg:
            raise RuntimeError(f"IK solution FK mismatch: pos err {perr:.4f} rot err {rerr:.2f}")
        code, jerr = c.movej(q, secs)
        p, Rr = (c.tcp_pose() if tcp else c.hand_pose())
        perr = np.linalg.norm(p - np.asarray(pos)); rerr = np.degrees((Rr.inv() * R).magnitude())
        print(f"  -> at {p.round(4)} (target {np.round(pos,4)}) pos_err={perr:.4f} rot_err={rerr:.2f}deg code={code}")
        if perr <= tol_m and rerr <= tol_deg:
            return q
        if attempt < retries:
            print("  re-sending same goal (tracking lag)")
            c.movej(q, secs)
            p, Rr = (c.tcp_pose() if tcp else c.hand_pose())
            perr = np.linalg.norm(p - np.asarray(pos)); rerr = np.degrees((Rr.inv() * R).magnitude())
            print(f"  -> at {p.round(4)} pos_err={perr:.4f} rot_err={rerr:.2f}deg")
            if perr <= tol_m and rerr <= tol_deg:
                return q
    raise RuntimeError("motion did not converge to target")
EOF
cat > grasp.py <<'EOF'
import numpy as np, rclpy, subprocess
from ctl import *
c = Ctl()
BOTTLE = np.array([-0.1665, 0.0335])
GZ = 0.955
phi = np.radians(10)
R_g = R_from_axes(zaxis=[np.cos(phi), 0, -np.sin(phi)], yaxis=[0, -1, 0])   # approach +x, camera on top
print("grasp R: Z", R_g.as_matrix()[:,2].round(3), "Y", R_g.as_matrix()[:,1].round(3), "X", R_g.as_matrix()[:,0].round(3))
print("fingers:", c.fingers())
print("1) transit above/behind bottle")
move_checked(c, [-0.30, BOTTLE[1], 1.12], R_g, secs=5.0)
print("2) pre-grasp")
move_checked(c, [-0.27, BOTTLE[1], GZ], R_g, secs=4.0)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "pregrasp.png"], timeout=60)
print("3) approach")
move_checked(c, [BOTTLE[0] + 0.008, BOTTLE[1], GZ], R_g, secs=3.0)
np.save("q_grasp.npy", np.array(c.arm_q()))
rclpy.shutdown()
EOF
timeout 900 python3 grasp.py

# openrua op 33
timeout 60 python3 ctl.py state; timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png; timeout 60 python3 tools/perception/cam_snap.py frontview front3.png

# openrua op 34
timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P = np.load("birdview_xyz.npy"); Z=P[...,2]
xs = np.arange(-0.45,0.20,0.02); ys=np.arange(-0.25,0.35,0.02)
print("      "+" ".join(f"{int(round(y*100)):3d}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(P[...,0]>=x)&(P[...,0]<x+0.02)&(P[...,1]>=y)&(P[...,1]<y+0.02)
        if m.sum()==0: row.append("  -"); continue
        z=Z[m].max()
        row.append("  ." if z<0.905 else f"{int(round((z-0.9)*100)):3d}")
    print(f"{x:5.2f} "+" ".join(row))
EOF

# openrua op 35
cat >> ctl.py <<'EOF'


LINKS = ["panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand", "panda_leftfinger", "panda_rightfinger"]


def fk_links(c, q, links=LINKS):
    """World positions of several links for joint vector q -> dict name->pos."""
    c.fk.wait_for_service(5)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(links)
    req.robot_state.joint_state.name = list(JOINTS)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = c.fk.call_async(req)
    rclpy.spin_until_future_complete(c.node, fut, timeout_sec=30)
    r = fut.result()
    out = {}
    for name, ps in zip(r.fk_link_names, r.pose_stamped):
        p = ps.pose.position
        out[name] = np.array([p.x, p.y, p.z])
    return out


def path_check(c, q_from, q_to, n=15, z_floor=0.93, verbose=True):
    """Interpolate in joint space and report min z of links + TCP; returns (ok, samples)."""
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    worst = 9.0; samples = []
    for i in range(n + 1):
        q = q_from + (q_to - q_from) * i / n
        L = fk_links(c, q)
        tcp, _ = c.tcp_pose(q)
        L["tcp"] = tcp
        zmin_name = min(L, key=lambda k: L[k][2])
        samples.append((i, L))
        if L[zmin_name][2] < worst:
            worst = L[zmin_name][2]
        if verbose:
            print(f"   step {i:2d}: hand={L['panda_hand'].round(3)} tcp={tcp.round(3)} lowest={zmin_name}@{L[zmin_name][2]:.3f}")
    ok = worst >= z_floor
    print(f"  path lowest point z={worst:.3f} -> {'OK' if ok else 'TOO LOW'}")
    return ok, samples
EOF
cat > home.py <<'EOF'
import numpy as np, rclpy
from ctl import *
c = Ctl()
q_now = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
print("now:", q_now.round(3))
ok, _ = path_check(c, q_now, q_home, n=20, z_floor=0.98)
rclpy.shutdown()
EOF
timeout 600 python3 home.py

# openrua op 36
cat > home2.py <<'EOF'
import numpy as np, rclpy, sys
from ctl import *
c = Ctl()
q_now = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
q_mid = q_home.copy(); q_mid[0] = q_now[0]
print("stage A (fold up, joint1 fixed):")
okA, _ = path_check(c, q_now, q_mid, n=12, z_floor=0.99)
print("stage B (swing joint1):")
okB, _ = path_check(c, q_mid, q_home, n=12, z_floor=0.99)
if okA and okB and "--go" in sys.argv:
    c.movej(q_mid, 5.0)
    c.movej(q_home, 6.0)
    print(fmt(*c.hand_pose()))
rclpy.shutdown()
EOF
timeout 900 python3 home2.py --go 2>&1 | grep -v "step"

# openrua op 37
cat > home3.py <<'EOF'
import numpy as np, rclpy, sys
from ctl import *
c = Ctl()
q_now = np.array(c.arm_q())
# stage A0: lift the hand straight up 15 cm in current orientation (IK near current config)
pos, R = c.hand_pose()
target = pos + np.array([0, 0, 0.15])
q_up = np.array(c.solve_ik(target, R, seed=q_now))
print("q_now", q_now.round(3)); print("q_up ", q_up.round(3), "diff", np.abs(q_up-q_now).max().round(3))
okA0, _ = path_check(c, q_now, q_up, n=10, z_floor=0.99, verbose=False)
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
q_mid = q_home.copy(); q_mid[0] = q_now[0]
okA, _ = path_check(c, q_up, q_mid, n=12, z_floor=0.99, verbose=False)
okB, _ = path_check(c, q_mid, q_home, n=12, z_floor=0.99, verbose=False)
print(okA0, okA, okB)
if okA0 and okA and okB and "--go" in sys.argv:
    c.movej(q_up, 4.0)
    c.movej(q_mid, 5.0)
    c.movej(q_home, 6.0)
    print(fmt(*c.hand_pose()))
rclpy.shutdown()
EOF
timeout 900 python3 home3.py --go

# openrua op 38
cat > get_urdf.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("urdf_get")
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got = []
n.create_subscription(String, "/robot_description", lambda m: got.append(m.data), qos)
import time
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.2)
    if got: break
open("robot.urdf","w").write(got[0]); print(len(got[0]), "bytes")
rclpy.shutdown()
EOF
timeout 60 python3 get_urdf.py && grep -n "<joint name=\"panda_hand_joint\"\|<joint name=\"panda_joint8\"\|<joint name=\"panda_hand_tcp" -A6 robot.urdf | head -60; grep -c "<link" robot.urdf; grep -o 'name="[a-z_0-9]*"' robot.urdf | grep -i "link\|hand\|finger" | sort -u | head -30

# openrua op 39
mkdir -p "$(dirname /workspace/kin.py)"
cat > /workspace/kin.py <<'OPENRUA_EOF'
"""Local kinematics for the Panda (from robot.urdf), in the WORLD frame.

fk(q)            -> dict link -> (pos, R) for panda_link1..7, panda_hand, tcp
hand(q)          -> (pos, R) of panda_hand
ik(pos, R, q0)   -> damped-least-squares IK that stays continuous with q0
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (measured via TF)
TCP = 0.1034
# (xyz, rpy) of each joint origin, from the URDF; all axes are local Z
_J = [
    ((0, 0, 0.333), (0, 0, 0)),
    ((0, 0, 0), (-np.pi / 2, 0, 0)),
    ((0, -0.316, 0), (np.pi / 2, 0, 0)),
    ((0.0825, 0, 0), (np.pi / 2, 0, 0)),
    ((-0.0825, 0.384, 0), (-np.pi / 2, 0, 0)),
    ((0, 0, 0), (np.pi / 2, 0, 0)),
    ((0.088, 0, 0), (np.pi / 2, 0, 0)),
]
LIM = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07],
                [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])
_FIX = [(np.eye(3), np.array(xyz)) for xyz, _ in _J]
_ROT = [Rot.from_euler("xyz", rpy).as_matrix() for _, rpy in _J]
_HAND_R = Rot.from_euler("z", -np.pi / 4).as_matrix()


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk(q):
    q = np.asarray(q, float)
    p, R = BASE.copy(), np.eye(3)
    out = {}
    for i in range(7):
        p = p + R @ _FIX[i][1]
        R = R @ _ROT[i] @ _rz(q[i])
        out[f"panda_link{i+1}"] = (p.copy(), R.copy())
    p = p + R @ np.array([0, 0, 0.107])
    R = R @ _HAND_R
    out["panda_hand"] = (p.copy(), R.copy())
    out["tcp"] = (p + TCP * R[:, 2], R.copy())
    return out


def hand(q):
    return fk(q)["panda_hand"]


def tcp(q):
    return fk(q)["tcp"]


def _err(q, pos, R, use_tcp):
    p, Rq = fk(q)["tcp" if use_tcp else "panda_hand"]
    e_p = pos - p
    e_r = Rot.from_matrix(R @ Rq.T).as_rotvec()
    return np.concatenate([e_p, e_r])


def _jac(q, pos, R, use_tcp, h=1e-6):
    e0 = _err(q, pos, R, use_tcp)
    J = np.zeros((6, 7))
    for i in range(7):
        dq = np.zeros(7); dq[i] = h
        J[:, i] = (_err(q + dq, pos, R, use_tcp) - e0) / h
    return e0, J


def ik(pos, R, q0, use_tcp=True, iters=200, tol_p=5e-4, tol_r=2e-3,
       lam=0.05, max_step=0.15, null_bias=0.0):
    """Return q near q0 reaching (pos, R) or raise RuntimeError.
    null_bias>0 pulls toward q0 in the nullspace (keeps posture)."""
    pos = np.asarray(pos, float)
    q = np.array(q0, float)
    for _ in range(iters):
        e, J = _jac(q, pos, R, use_tcp)
        if np.linalg.norm(e[:3]) < tol_p and np.linalg.norm(e[3:]) < tol_r:
            return np.clip(q, LIM[:, 0], LIM[:, 1])
        # J maps dq -> d(err)?? err = target - current so d err = -J_true dq;
        # we computed J of err directly, so solve J dq = -e
        JJt = J @ J.T + lam**2 * np.eye(6)
        dq = -J.T @ np.linalg.solve(JJt, e)
        if null_bias > 0:
            N = np.eye(7) - J.T @ np.linalg.solve(JJt, J)
            dq += null_bias * (N @ (np.array(q0) - q))
        n = np.linalg.norm(dq)
        if n > max_step:
            dq *= max_step / n
        q = np.clip(q + dq, LIM[:, 0] + 0.02, LIM[:, 1] - 0.02)
    e = _err(q, pos, R, use_tcp)
    raise RuntimeError(f"local IK did not converge: pos err {np.linalg.norm(e[:3]):.4f} m, "
                       f"rot err {np.degrees(np.linalg.norm(e[3:])):.1f} deg")


def path_min_z(q_from, q_to, n=20, extra=()):
    """Lowest world z of any link origin / tcp / extra hand-frame points
    along a straight joint-space path."""
    q_from, q_to = np.asarray(q_from), np.asarray(q_to)
    worst = (np.inf, None, None)
    for t in np.linspace(0, 1, n + 1):
        q = q_from + t * (q_to - q_from)
        f = fk(q)
        pts = [(k, v[0]) for k, v in f.items() if k not in ("panda_link1", "panda_link2")]
        ph, Rh = f["panda_hand"]
        for k, off in extra:
            pts.append((k, ph + Rh @ np.asarray(off)))
        k, p = min(pts, key=lambda kp: kp[1][2])
        if p[2] < worst[0]:
            worst = (p[2], k, (t, p))
    return worst


# hand-frame points of the palm/finger envelope for clearance checks
HAND_PTS = [("palm+y", (0, 0.1, 0.03)), ("palm-y", (0, -0.1, 0.03)),
            ("palm+y_tip", (0, 0.1, 0.06)), ("palm-y_tip", (0, -0.1, 0.06)),
            ("cam", (0.05, 0, 0.03)), ("finger+y", (0, 0.045, 0.1034)),
            ("finger-y", (0, -0.045, 0.1034))]
OPENRUA_EOF

# openrua op 40
cat > kintest.py <<'EOF'
import numpy as np, kin
from ctl import Ctl, fk_links
c = Ctl()
q = np.array(c.arm_q()); print("q", q.round(3))
p, R = c.hand_pose(q); pl, Rl = kin.hand(q)
print("hand svc", p.round(4), "local", pl.round(4), "rot diff deg", np.degrees(np.arccos(np.clip((np.trace(R.as_matrix().T@Rl)-1)/2,-1,1))).round(2))
svc = fk_links(c, q)
for k,v in kin.fk(q).items():
    if k in svc: print(k, np.round(v[0]-svc[k],4))
# IK round trip: lift hand 15 cm
q2 = kin.ik(pl + [0,0,0.15], Rl, q, use_tcp=False)
print("q2", q2.round(3), "diff", np.abs(q2-q).max().round(3))
print(kin.hand(q2)[0].round(4))
c.node.destroy_node()
EOF
timeout 60 python3 kintest.py

# openrua op 41
cat > plan_home.py <<'EOF'
import numpy as np, kin, sys
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
ph, Rh = kin.hand(q0)
stages = [q0]
# stage 1: lift 15 cm
q1 = kin.ik(ph + [0,0,0.15], Rh, q0, use_tcp=False); stages.append(q1)
# stage 2: move hand toward home hand pose orientation gradually? just check direct to home
stages.append(q_home)
ok = True
for a, b in zip(stages, stages[1:]):
    z, k, (t, p) = kin.path_min_z(a, b, extra=kin.HAND_PTS)
    print(f"{a.round(2)} -> {b.round(2)}\n   min z {z:.3f} at {k} t={t:.2f} p={p.round(3)}")
    ok &= z > 0.99
print("all ok", ok)
if ok and "--go" in sys.argv:
    for b in stages[1:]:
        print("moving to", b.round(3), c.movej(list(b), 5.0))
        print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
EOF
timeout 120 python3 plan_home.py

# openrua op 42
cat > plan_home2.py <<'EOF'
import numpy as np, kin, sys
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
ph, Rh = kin.hand(q0)
q1 = kin.ik(ph + [0,0,0.15], Rh, q0, use_tcp=False)
# lift further to 1.40 and level the hand (top-down) keeping near q1
Rdown = kin.Rot.from_euler("xyz", [np.pi, 0, 0]).as_matrix()  # Z down, X along +x
cands = {}
for z in (1.35, 1.40):
    for yaw in (0, 45, 90, 135, 180, -45, -90, -135):
        R = kin.Rot.from_euler("z", np.radians(yaw)).as_matrix() @ Rdown
        try:
            q2 = kin.ik(ph + [0,0,z-ph[2]], R, q1, use_tcp=False, null_bias=0.1)
        except RuntimeError as e:
            continue
        cands[(z,yaw)] = q2
print(len(cands), "candidates")
best = None
for key, q2 in cands.items():
    q_mid = q_home.copy(); q_mid[0] = q2[0]
    st = [q0, q1, q2, q_mid, q_home]
    zs = [kin.path_min_z(a, b, extra=kin.HAND_PTS)[0] for a, b in zip(st, st[1:])]
    m = min(zs)
    print(key, np.round(zs,3), q2.round(2))
    if best is None or m > best[0]: best = (m, key, st)
print("best", best[0], best[1])
if best[0] > 0.99 and "--go" in sys.argv:
    for b in best[2][1:]:
        print("moving to", np.round(b,3), c.movej(list(b), 5.0))
        print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
EOF
timeout 300 python3 plan_home2.py

# openrua op 43
cat >> kin.py <<'EOF'


# world-frame obstacle boxes (xmin,xmax,ymin,ymax,zmin,zmax), incl. margins
OBST = {
    "table": (-1.5, 1.5, -1.5, 1.5, 0.0, 0.905),
    "bottle_fallen": (-0.37, -0.23, -0.09, 0.07, 0.9, 0.955),
    "bowl": (-0.06, 0.10, -0.12, 0.04, 0.9, 0.965),
    "board": (-0.26, 0.06, -0.28, -0.16, 0.9, 1.17),
    "cabinet": (-0.16, 0.16, 0.20, 0.46, 0.9, 1.15),
    "drawer": (-0.11, 0.11, 0.06, 0.24, 0.9, 1.0),
}


def body_points(q, hand_pts=HAND_PTS, seg=3):
    f = fk(q)
    names = ["panda_link2", "panda_link3", "panda_link4", "panda_link5",
             "panda_link6", "panda_link7", "panda_hand"]
    pts = []
    for a, b in zip(names, names[1:]):
        pa, pb = f[a][0], f[b][0]
        for t in np.linspace(0, 1, seg + 1)[1:]:
            pts.append((b, pa + t * (pb - pa)))
    ph, Rh = f["panda_hand"]
    pts.append(("tcp", f["tcp"][0]))
    for k, off in hand_pts:
        pts.append((k, ph + Rh @ np.asarray(off)))
    return pts


def in_box(p, b):
    return (b[0] <= p[0] <= b[1]) and (b[2] <= p[1] <= b[3]) and (b[4] <= p[2] <= b[5])


def collisions(q_from, q_to, n=25, ignore=(), hand_pts=HAND_PTS, skip_pts=()):
    """List of (t, point_name, obstacle) hits along a straight joint path."""
    q_from, q_to = np.asarray(q_from), np.asarray(q_to)
    hits = []
    for t in np.linspace(0, 1, n + 1):
        q = q_from + t * (q_to - q_from)
        for k, p in body_points(q, hand_pts):
            if k in skip_pts:
                continue
            for name, b in OBST.items():
                if name in ignore:
                    continue
                if in_box(p, b):
                    hits.append((round(float(t), 2), k, name, p.round(3)))
    return hits
EOF
cat > plan_home3.py <<'EOF'
import numpy as np, kin, sys
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
ph, Rh = kin.hand(q0)
q1 = kin.ik(ph + [0,0,0.15], Rh, q0, use_tcp=False)
Rdown = kin.Rot.from_euler("xyz", [np.pi, 0, 0]).as_matrix()
R = kin.Rot.from_euler("z", np.radians(135)).as_matrix() @ Rdown
q2 = kin.ik(ph + [0,0,1.35-ph[2]], R, q1, use_tcp=False, null_bias=0.1)
q_mid = q_home.copy(); q_mid[0] = q2[0]
st = [q0, q1, q2, q_mid, q_home]
allok = True
for a, b in zip(st, st[1:]):
    h = kin.collisions(a, b)
    print(np.round(b,2), "hits:", len(h), h[:3])
    allok &= not h
print("home hand", kin.hand(q_home)[0].round(3))
print("all ok", allok)
if allok and "--go" in sys.argv:
    for b in st[1:]:
        print("moving to", np.round(b,3), c.movej(list(b), 5.0))
        print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
EOF
timeout 120 python3 plan_home3.py

# openrua op 44
timeout 200 python3 plan_home3.py --go 2>&1 | tail -12

# openrua op 45
timeout 60 python3 ctl.py state; timeout 30 python3 tools/perception/cam_snap.py agentview agent4.png

# openrua op 46
cat > gohome.py <<'EOF'
import numpy as np, kin
from ctl import Ctl
c = Ctl()
q0 = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
h = kin.collisions(q0, q_home); print("hits", h[:3])
if not h:
    secs = max(3.0, np.abs(q_home - q0).max() / 0.15)
    print("secs", round(secs,1), c.movej(list(q_home), secs))
    print("now", np.array(c.arm_q()).round(3))
c.node.destroy_node()
EOF
timeout 120 python3 gohome.py

# openrua op 47
timeout 60 python3 cloud.py birdview && timeout 60 python3 cloud.py agentview && cat > survey.py <<'EOF'
import numpy as np
def hmap(P, xr, yr, cell=0.01, zmin=0.91):
    x,y,z = P[...,0].ravel(), P[...,1].ravel(), P[...,2].ravel()
    m = np.isfinite(z) & (x>xr[0])&(x<xr[1])&(y>yr[0])&(y<yr[1])&(z>zmin)
    return x[m],y[m],z[m]
for cam in ("birdview","agentview"):
    P = np.load(f"{cam}_xyz.npy")
    print("==", cam)
    # bottle region
    x,y,z = hmap(P, (-0.45,-0.15), (-0.15,0.12))
    if len(z):
        print("bottle pts", len(z), "x", x.min().round(3), x.max().round(3), "y", y.min().round(3), y.max().round(3), "ztop", z.max().round(3))
        # body vs neck: body where z>0.925
        b = z>0.925
        print(" body x", x[b].min().round(3), x[b].max().round(3), "y", y[b].min().round(3), y[b].max().round(3), "ztop", z[b].max().round(3), "ymean", y[b].mean().round(3))
        # per x-slice y-center & top
        for xs in np.arange(-0.40,-0.20,0.02):
            s = (x>=xs)&(x<xs+0.02)
            if s.sum()>3: print(f"  x[{xs:.2f}] n={s.sum():3d} y {y[s].min():.3f}..{y[s].max():.3f} yc {y[s].mean():.3f} ztop {z[s].max():.3f}")
    # drawer region: front panel & handle
    x,y,z = hmap(P, (-0.12,0.12), (0.0,0.26), zmin=0.915)
    print("drawer region pts", len(z))
    for ys in np.arange(0.0,0.26,0.01):
        s=(y>=ys)&(y<ys+0.01)
        if s.sum()>3: print(f"  y[{ys:.2f}] n={s.sum():3d} x {x[s].min():.3f}..{x[s].max():.3f} ztop {z[s].max():.3f} zmed {np.median(z[s]):.3f}")
EOF
python3 survey.py

# openrua op 48
cat > bottle.py <<'EOF'
import numpy as np
for cam in ("agentview","birdview"):
    P = np.load(f"{cam}_xyz.npy")
    x,y,z = P[...,0].ravel(), P[...,1].ravel(), P[...,2].ravel()
    m = np.isfinite(z)&(x>-0.45)&(x<-0.15)&(y>-0.15)&(y<0.15)&(z>0.912)&(z<1.0)
    x,y,z = x[m],y[m],z[m]
    print("==",cam, len(z), "ztop", z.max().round(3))
    # PCA axis of top points
    pts = np.c_[x,y]
    c = pts.mean(0); u,s,vt = np.linalg.svd(pts-c, full_matrices=False)
    ax = vt[0]; 
    if ax[0]<0: ax=-ax
    t = (pts-c)@ax; w = (pts-c)@vt[1]
    print("center", c.round(3), "axis", ax.round(3), "angle deg", np.degrees(np.arctan2(ax[1],ax[0])).round(1), "len", (t.max()-t.min()).round(3), "t range", t.min().round(3), t.max().round(3))
    for ts in np.arange(t.min(), t.max(), 0.02):
        s=(t>=ts)&(t<ts+0.02)
        if s.sum()>3: print(f"  t[{ts:+.3f}] n={s.sum():3d} width {w[s].max()-w[s].min():.3f} ztop {z[s].max():.3f} wc {w[s].mean():+.3f}")
    # body only (z>0.93) center
    b = z>0.932
    print("body(z>0.932) center", pts[b].mean(0).round(3), "t range", t[b].min().round(3), t[b].max().round(3))
EOF
python3 bottle.py

# openrua op 49
grep -n "def movej" -A40 /workspace/ctl.py | head -60

# openrua op 50
mkdir -p "$(dirname /workspace/motion.py)"
cat > /workspace/motion.py <<'OPENRUA_EOF'
"""Checked, paced motion on top of kin.py (local IK) and ctl.Ctl."""
import numpy as np
import kin
from kin import Rot
from ctl import Ctl

VMAX = 0.15  # rad/s per joint: the controller silently caps near ~0.19


def R_from_axes(z, y):
    z = np.asarray(z, float); z /= np.linalg.norm(z)
    y = np.asarray(y, float); y = y - (y @ z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def cart_path(q0, pos1, R1, use_tcp=True, step_m=0.03, step_deg=10, null_bias=0.05):
    """Straight-line (pos slerp) TCP/hand path from fk(q0) to (pos1,R1) as joint waypoints."""
    p0, R0 = (kin.tcp if use_tcp else kin.hand)(q0)
    pos1 = np.asarray(pos1, float)
    d = np.linalg.norm(pos1 - p0)
    ang = np.degrees((Rot.from_matrix(R0).inv() * Rot.from_matrix(R1)).magnitude())
    n = max(1, int(np.ceil(max(d / step_m, ang / step_deg))))
    rots = Rot.from_matrix(np.stack([R0, R1]))
    qs, q = [], np.array(q0, float)
    for i in range(1, n + 1):
        t = i / n
        p = p0 + t * (pos1 - p0)
        # slerp between two rotations
        R = (Rot.from_matrix(R0) * Rot.from_rotvec(
            t * (Rot.from_matrix(R0).inv() * Rot.from_matrix(R1)).as_rotvec())).as_matrix()
        q = kin.ik(p, R, q, use_tcp=use_tcp, null_bias=null_bias)
        qs.append(q)
    return qs


def check(qs, q0, ignore=(), skip_pts=(), hand_pts=kin.HAND_PTS):
    hits = []
    prev = np.array(q0)
    for q in qs:
        hits += kin.collisions(prev, q, n=8, ignore=ignore, skip_pts=skip_pts, hand_pts=hand_pts)
        prev = q
    return hits


def execute(c: Ctl, qs, ignore=(), skip_pts=(), hand_pts=kin.HAND_PTS, vmax=VMAX, label=""):
    q0 = np.array(c.arm_q())
    hits = check(qs, q0, ignore, skip_pts, hand_pts)
    if hits:
        seen = {}
        for h in hits:
            seen.setdefault((h[1], h[2]), h)
        raise RuntimeError(f"{label}: path hits {list(seen.values())[:6]}")
    # pacing by max joint delta per segment
    times, t, prev = [], 0.0, q0
    for q in qs:
        t += max(0.5, np.abs(np.array(q) - prev).max() / vmax)
        times.append(t); prev = np.array(q)
    code, err = c.movej([list(q) for q in qs], times)
    if code != 0 or err > 0.02:
        # one paced resend of the final target
        q_now = np.array(c.arm_q())
        code, err = c.movej(list(qs[-1]), max(1.0, np.abs(np.array(qs[-1]) - q_now).max() / vmax))
    q_now = np.array(c.arm_q())
    p, R = kin.tcp(q_now)
    print(f"  [{label}] done code={code} jerr={err:.4f} tcp={p.round(4)} Z={R[:,2].round(2)} Y={R[:,1].round(2)}")
    if err > 0.02:
        raise RuntimeError(f"{label}: did not converge, joint err {err:.3f}")
    return q_now


def goto(c, pos, R, use_tcp=True, label="", **kw):
    q0 = np.array(c.arm_q())
    qs = cart_path(q0, pos, R, use_tcp=use_tcp)
    return execute(c, qs, label=label, **kw)


def gotoj(c, q, label="", **kw):
    return execute(c, [np.array(q)], label=label, **kw)
OPENRUA_EOF

# openrua op 51
cat > pull.py <<'EOF'
import sys, numpy as np, kin, motion
from ctl import Ctl
c = Ctl()
th = np.radians(20)
R = motion.R_from_axes([0, np.sin(th), -np.cos(th)], [-1, 0, 0])
print("R cols X,Y,Z:", R[:,0].round(3), R[:,1].round(3), R[:,2].round(3))
X0 = 0.0
pre   = np.array([X0, 0.18, 1.12])
down  = np.array([X0, 0.18, 0.955])
pulled= np.array([X0, 0.12, 0.955])
up    = np.array([X0, 0.12, 1.12])
q = np.array(c.arm_q())
paths = []
for name, tgt, ign in [("pre", pre, ()), ("down", down, ("drawer",)), ("pull", pulled, ("drawer",)), ("up", up, ("drawer",))]:
    qs = motion.cart_path(q, tgt, R)
    hits = motion.check(qs, q, ignore=ign, skip_pts=("tcp","finger+y","finger-y") if ign else ())
    print(f"{name}: {len(qs)} wps, max dq {np.abs(qs[-1]-q).max():.2f}, hits {len(hits)} {hits[:2]}")
    print("   q_end", qs[-1].round(2), "palm pts:", [ (k, (kin.hand(qs[-1])[0] + kin.hand(qs[-1])[1] @ np.array(o)).round(3)) for k,o in kin.HAND_PTS[:2]])
    paths.append((name, qs, ign)); q = qs[-1]
if "--go" in sys.argv:
    print("fingers", c.gripper(0.0))
    for name, qs, ign in paths:
        motion.execute(c, qs, ignore=ign, skip_pts=("tcp","finger+y","finger-y") if ign else (), label=name)
c.node.destroy_node()
EOF
timeout 120 python3 pull.py

# openrua op 52
timeout 300 python3 pull.py --go 2>&1 | grep -v "^   q_end\|wps" ; timeout 60 python3 cloud.py agentview >/dev/null && python3 survey.py 2>/dev/null | sed -n '/== agentview/,$p' | grep "drawer\|y\[0\.[01]"

# openrua op 53
cat > drawer.py <<'EOF'
import numpy as np, sys
cam = sys.argv[1] if len(sys.argv)>1 else "agentview"
P = np.load(f"{cam}_xyz.npy")
x,y,z = P[...,0].ravel(), P[...,1].ravel(), P[...,2].ravel()
m = np.isfinite(z)&(x>-0.12)&(x<0.12)&(y>-0.02)&(y<0.26)&(z>0.912)&(z<1.05)
x,y,z = x[m],y[m],z[m]
for ys in np.arange(-0.02,0.26,0.01):
    s=(y>=ys)&(y<ys+0.01)
    if s.sum()>3: print(f"  y[{ys:+.2f}] n={s.sum():4d} x {x[s].min():+.3f}..{x[s].max():+.3f} ztop {z[s].max():.3f} zmed {np.median(z[s]):.3f}")
EOF
python3 drawer.py

# openrua op 54
cat > grasp2.py <<'EOF'
import sys, numpy as np, kin, motion
from ctl import Ctl
c = Ctl()
ax = np.array([0.559, -0.829, 0.0])          # bottle axis (neck toward -ax)
perp = np.array([0.829, 0.559, 0.0])
center = np.array([-0.288, 0.007])
q = np.array(c.arm_q())
best = None
for sgn in (1, -1):
    R = motion.R_from_axes([0,0,-1], sgn*perp)
    try:
        qs = motion.cart_path(q, [center[0], center[1], 1.05], R)
    except RuntimeError as e:
        print("ik fail", sgn, e); continue
    hits = motion.check(qs, q)
    d = np.abs(qs[-1]-q).max()
    print(f"sgn {sgn}: max dq {d:.2f} hits {len(hits)} X={R[:,0].round(2)} q_end {qs[-1].round(2)}")
    if not hits and (best is None or d < best[0]): best = (d, sgn, R, qs)
d, sgn, R, qs_pre = best
q1 = qs_pre[-1]
qs_down = motion.cart_path(q1, [center[0], center[1], 0.913], R)
skip = ("tcp","finger+y","finger-y")
print("down hits", motion.check(qs_down, q1, ignore=("bottle_fallen",), skip_pts=skip)[:3])
q2 = qs_down[-1]
qs_up = motion.cart_path(q2, [center[0], center[1], 1.10], R)
print("up hits", motion.check(qs_up, q2, ignore=("bottle_fallen",), skip_pts=skip)[:3])
if "--go" in sys.argv:
    print("open", c.gripper(0.04))
    motion.execute(c, qs_pre, label="pre")
    motion.execute(c, qs_down, ignore=("bottle_fallen",), skip_pts=skip, label="down")
    f = c.gripper(0.0); print("close ->", f)
    motion.execute(c, qs_up, ignore=("bottle_fallen",), skip_pts=skip, label="up")
    print("fingers after lift", c.fingers())
c.node.destroy_node()
EOF
timeout 120 python3 grasp2.py

# openrua op 55
timeout 300 python3 grasp2.py --go 2>&1 | grep -v "^sgn\|ik fail\|hits"

# openrua op 56
timeout 30 python3 tools/perception/cam_snap.py agentview agent5.png && timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 57
cat > place.py <<'EOF'
import sys, numpy as np, kin, motion
from ctl import Ctl
# scene update after the pull
kin.OBST.pop("bottle_fallen", None)
kin.OBST["drawer"] = (-0.11, 0.11, 0.07, 0.24, 0.9, 0.99)
kin.OBST["handle_low"] = (-0.05, 0.06, 0.035, 0.08, 0.9, 0.965)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
kin.OBST["bowl"] = (-0.06, 0.10, -0.12, 0.035, 0.9, 0.97)
# richer hand envelope + carried bottle (axis along hand X, center at hand Z=0.096, neck toward -X)
PTS = list(kin.HAND_PTS) + [("palmcorner+", (0.025, 0.1, 0.058)), ("palmcorner-", (0.025, -0.1, 0.058)),
        ("palmcorner2+", (-0.025, 0.1, 0.058)), ("palmcorner2-", (-0.025, -0.1, 0.058)), ("camlow", (0.05, 0, 0.058))]
BOT = [(f"bottle{t:+.2f}", (t, 0, 0.096 + dz)) for t in np.arange(-0.115, 0.045, 0.02) for dz in (-0.02, 0.02)]
c = Ctl()
q = np.array(c.arm_q())
th = np.radians(50)
Z = np.array([0, np.sin(th), -np.cos(th)]); f = np.array([0, np.cos(th), np.sin(th)])
plans = []
for sgn in (1, -1):
    R = motion.R_from_axes(Z, sgn*f)
    xc = -0.035 if R[0,0] < 0 else 0.035   # keep the 16 cm bottle centred in the 18 cm interior
    via = np.array([-0.12, 0.02, 1.16]); tgt = np.array([xc, 0.15, 1.02])
    try:
        qs1 = motion.cart_path(q, via, R); qs2 = motion.cart_path(qs1[-1], tgt, R)
    except RuntimeError as e:
        print("ik fail", sgn, e); continue
    h1 = motion.check(qs1, q, hand_pts=PTS+BOT)
    h2 = motion.check(qs2, qs1[-1], hand_pts=PTS+BOT, ignore=("drawer",), skip_pts=("tcp","finger+y","finger-y")+tuple(n for n,_ in BOT))
    # bottle points vs drawer: only allow inside the interior footprint
    print(f"sgn {sgn} X={R[:,0].round(2)} xc={xc}: dq1 {np.abs(qs1[-1]-q).max():.2f} hits1 {len(h1)} {h1[:2]}  dq2 {np.abs(qs2[-1]-qs1[-1]).max():.2f} hits2 {len(h2)} {h2[:2]}")
    print("   q_end", qs2[-1].round(2))
    if not h1 and not h2: plans.append((np.abs(qs1[-1]-q).max(), sgn, R, qs1, qs2, xc))
plans.sort(key=lambda p: p[0])
if plans and "--go" in sys.argv:
    _, sgn, R, qs1, qs2, xc = plans[0]
    print("using sgn", sgn)
    motion.execute(c, qs1, hand_pts=PTS+BOT, label="via")
    motion.execute(c, qs2, hand_pts=PTS+BOT, ignore=("drawer",), skip_pts=("tcp","finger+y","finger-y")+tuple(n for n,_ in BOT), label="place")
    ph, Rh = kin.hand(np.array(c.arm_q()))
    for n, o in PTS: print("  ", n, (ph + Rh @ np.array(o)).round(3))
c.node.destroy_node()
EOF
timeout 120 python3 place.py

# openrua op 58
timeout 400 python3 place.py --go 2>&1 | grep -v "^sgn\|ik fail\|q_end"

# openrua op 59
timeout 30 python3 tools/perception/cam_snap.py agentview agent6.png && timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png && python3 -c "
import cv2; a=cv2.imread('agent6.png'); cv2.imwrite('crop6.png', cv2.resize(a[150:400,330:640],(620,500)))"

# openrua op 60
cat > release.py <<'EOF'
import numpy as np, kin, motion
from ctl import Ctl
kin.OBST.pop("bottle_fallen", None)
kin.OBST["drawer"] = (-0.11, 0.11, 0.07, 0.24, 0.9, 0.99)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
c = Ctl()
print("open", c.gripper(0.04))
q = np.array(c.arm_q()); p, R = kin.tcp(q)
motion.goto(c, p + [0, -0.05, 0.12], R, label="retreat", ignore=("drawer",), skip_pts=("tcp","finger+y","finger-y"))
c.node.destroy_node()
EOF
timeout 200 python3 release.py && timeout 60 python3 cloud.py agentview >/dev/null && python3 drawer.py && timeout 30 python3 tools/perception/cam_snap.py agentview agent7.png

# openrua op 61
python3 -c "
import numpy as np
P=np.load('agentview_xyz.npy'); x,y,z=P[...,0].ravel(),P[...,1].ravel(),P[...,2].ravel()
m=np.isfinite(z)&(x>-0.09)&(x<0.09)&(y>0.105)&(y<0.225)&(z>0.93)&(z<0.98)
print('bottle-in-drawer pts',m.sum(),'x',x[m].min().round(3),x[m].max().round(3),'y',y[m].min().round(3),y[m].max().round(3),'ztop',z[m].max().round(3))
for xs in np.arange(-0.09,0.09,0.02):
    s=m&(x>=xs)&(x<xs+0.02)
    if s.sum()>3: print(f'  x[{xs:+.2f}] n={s.sum():3d} y {y[s].min():.3f}..{y[s].max():.3f} ztop {z[s].max():.3f}')
"; python3 -c "
import cv2; a=cv2.imread('agent7.png'); cv2.imwrite('crop7.png', cv2.resize(a[150:400,330:640],(620,500)))"

# openrua op 62
cat > close.py <<'EOF'
import sys, numpy as np, kin, motion
from ctl import Ctl
kin.OBST.pop("bottle_fallen", None)
kin.OBST["drawer"] = (-0.11, 0.11, 0.07, 0.24, 0.9, 0.99)
kin.OBST["handle_low"] = (-0.05, 0.06, 0.035, 0.08, 0.9, 0.965)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
kin.OBST["bowl"] = (-0.06, 0.10, -0.12, 0.035, 0.9, 0.97)
PTS = list(kin.HAND_PTS) + [("palmcorner+", (0.025, 0.1, 0.058)), ("palmcorner-", (0.025, -0.1, 0.058)),
        ("palmcorner2+", (-0.025, 0.1, 0.058)), ("palmcorner2-", (-0.025, -0.1, 0.058)), ("camlow", (0.05, 0, 0.058))]
skip = ("tcp", "finger+y", "finger-y")
c = Ctl()
th = np.radians(35)
R = motion.R_from_axes([0, np.sin(th), -np.cos(th)], [-1, 0, 0])
X0 = -0.025
q = np.array(c.arm_q())
stages = [("pre", [X0, 0.03, 1.10], ()), ("down", [X0, 0.03, 0.945], ("drawer", "handle_low")),
          ("push", [X0, 0.185, 0.945], ("drawer", "handle_low")), ("up", [X0, 0.185, 1.10], ("drawer", "handle_low"))]
plan = []
for name, tgt, ign in stages:
    qs = motion.cart_path(q, tgt, R)
    hits = motion.check(qs, q, ignore=ign, skip_pts=skip if ign else (), hand_pts=PTS)
    print(f"{name}: dq {np.abs(qs[-1]-q).max():.2f} hits {len(hits)} {hits[:2]}")
    plan.append((name, qs, ign)); q = qs[-1]
ph, Rh = kin.hand(q); print("end palm corner2-", (ph + Rh @ np.array((-0.025,-0.1,0.058))).round(3), "corner+", (ph + Rh @ np.array((0.025,0.1,0.058))).round(3))
if "--go" in sys.argv:
    print("close fingers", c.gripper(0.0))
    for name, qs, ign in plan:
        motion.execute(c, qs, ignore=ign, skip_pts=skip if ign else (), hand_pts=PTS, label=name)
c.node.destroy_node()
EOF
timeout 120 python3 close.py

# openrua op 63
cat > close.py <<'EOF'
import sys, numpy as np, kin, motion
from ctl import Ctl
kin.OBST.pop("bottle_fallen", None)
kin.OBST["drawer"] = (-0.11, 0.11, 0.07, 0.24, 0.9, 0.99)
kin.OBST["handle_low"] = (-0.05, 0.06, 0.035, 0.08, 0.9, 0.965)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
kin.OBST["bowl"] = (-0.03, 0.09, -0.10, 0.03, 0.9, 0.96)   # measured rim, small margin
PTS = list(kin.HAND_PTS) + [("palmcorner+", (0.025, 0.1, 0.058)), ("palmcorner-", (0.025, -0.1, 0.058)),
        ("palmcorner2+", (-0.025, 0.1, 0.058)), ("palmcorner2-", (-0.025, -0.1, 0.058)), ("camlow", (0.05, 0, 0.058))]
skip = ("tcp", "finger+y", "finger-y")
def Rt(deg):
    th = np.radians(deg); return motion.R_from_axes([0, np.sin(th), -np.cos(th)], [-1, 0, 0])
c = Ctl()
X0 = -0.025
q = np.array(c.arm_q())
stages = [("pre", [X0, 0.03, 1.10], Rt(20), ()), ("down", [X0, 0.03, 0.945], Rt(20), ("drawer", "handle_low")),
          ("push", [X0, 0.185, 0.945], Rt(40), ("drawer", "handle_low")), ("up", [X0, 0.185, 1.10], Rt(40), ("drawer", "handle_low"))]
plan = []
for name, tgt, R, ign in stages:
    qs = motion.cart_path(q, tgt, R)
    hits = motion.check(qs, q, ignore=ign, skip_pts=skip if ign else (), hand_pts=PTS)
    print(f"{name}: dq {np.abs(qs[-1]-q).max():.2f} hits {len(hits)} {hits[:2]}")
    ph, Rh = kin.hand(qs[-1]); print("   camlow", (ph + Rh @ np.array((0.05,0,0.058))).round(3), "corner2-", (ph + Rh @ np.array((-0.025,-0.1,0.058))).round(3))
    plan.append((name, qs, ign)); q = qs[-1]
if "--go" in sys.argv:
    print("close fingers", c.gripper(0.0))
    for name, qs, ign in plan:
        motion.execute(c, qs, ignore=ign, skip_pts=skip if ign else (), hand_pts=PTS, label=name)
c.node.destroy_node()
EOF
timeout 120 python3 close.py

# openrua op 64
timeout 400 python3 close.py --go 2>&1 | grep "gripper\|done\|Error\|error" ; timeout 60 python3 cloud.py agentview >/dev/null && python3 drawer.py && timeout 30 python3 tools/perception/cam_snap.py agentview agent8.png

# openrua op 65
python3 -c "
import numpy as np
P=np.load('agentview_xyz.npy'); x,y,z=P[...,0].ravel(),P[...,1].ravel(),P[...,2].ravel()
# anything on the table surface region that's not cabinet(y>0.2), bowl, board, or the arm (z>1.05)?
m=np.isfinite(z)&(z>0.912)&(z<1.05)&(y<0.19)&(x>-0.5)&(x<0.3)&(y>-0.4)
bowl=(x>-0.04)&(x<0.10)&(y>-0.11)&(y<0.03); board=(x>-0.27)&(x<0.07)&(y>-0.30)&(y<-0.15)
r=m&~bowl&~board
print('stray table points (not bowl/board/cabinet):', r.sum())
if r.sum(): print(' x',x[r].min().round(3),x[r].max().round(3),'y',y[r].min().round(3),y[r].max().round(3),'z',z[r].max().round(3))
m2=np.isfinite(z)&(x>-0.1)&(x<0.1)&(y>0.03)&(y<0.19)&(z>0.912)&(z<1.0)
print('points in front of cabinet face y 0.03..0.19 (open drawer would show here):', m2.sum())
"; python3 -c "
import cv2; a=cv2.imread('agent8.png'); cv2.imwrite('crop8.png', cv2.resize(a[120:420,300:640],(680,600)))"

# openrua op 66
python3 -c "
import numpy as np
P=np.load('agentview_xyz.npy'); x,y,z=P[...,0].ravel(),P[...,1].ravel(),P[...,2].ravel()
m=np.isfinite(z)&(z>0.912)&(z<1.05)&(y<0.19)&(x>-0.5)&(x<0.3)&(y>-0.4)
bowl=(x>-0.04)&(x<0.10)&(y>-0.11)&(y<0.03); board=(x>-0.27)&(x<0.07)&(y>-0.30)&(y<-0.15)
r=m&~bowl&~board
for ys in np.arange(-0.4,0.2,0.05):
    s=r&(y>=ys)&(y<ys+0.05)
    if s.sum(): print(f'y[{ys:+.2f}] n={s.sum()} x {x[s].min():.3f}..{x[s].max():.3f} ztop {z[s].max():.3f}')
"

# openrua op 67
cat > park.py <<'EOF'
import numpy as np, kin, motion
from ctl import Ctl
kin.OBST.pop("bottle_fallen", None); kin.OBST.pop("drawer", None)
kin.OBST["cabinet"] = (-0.16, 0.16, 0.17, 0.46, 0.9, 1.15)
c = Ctl()
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
q = np.array(c.arm_q())
# lift first, then joint move home
p, R = kin.tcp(q)
motion.goto(c, [p[0], p[1]-0.05, 1.25], R, label="lift")
motion.gotoj(c, q_home, label="home")
print("final joints", np.array(c.arm_q()).round(3), "fingers", c.fingers())
c.node.destroy_node()
EOF
timeout 300 python3 park.py 2>&1 | grep "done\|final\|Error"; timeout 30 python3 tools/perception/cam_snap.py agentview final.png && timeout 60 python3 cloud.py agentview >/dev/null && python3 drawer.py | tail -6

# openrua op 68
sed -i 's/kin.OBST\["cabinet"\] = (-0.16, 0.16, 0.17, 0.46, 0.9, 1.15)/kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)/; s/motion.goto(c, \[p\[0\], p\[1\]-0.05, 1.25\], R, label="lift")/motion.goto(c, [p[0], p[1]-0.08, 1.25], R, label="lift", ignore=("handles_up",), skip_pts=("tcp","finger+y","finger-y"))/' park.py && timeout 300 python3 park.py 2>&1 | grep "done\|final\|Error"; timeout 30 python3 tools/perception/cam_snap.py agentview final.png

# openrua op 69
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-quirks
description: Non-obvious facts about the Panda kitchen sim in /workspace (IK frames, controller speed cap, IK branch jumps) learned while doing the bottle-in-drawer task
metadata:
  type: project
---

Facts about the Panda/MoveIt kitchen sim in /workspace (verified 2026-09-12):
- `/compute_fk` and `/compute_ik` with empty frame_id work in the WORLD frame, not panda_link0 (world->panda_link0 = (-0.66, 0, 0.912), identity rotation), contrary to machine.yaml/docs.
- `/compute_ik` solves for panda_link8 by default (45° off about hand Z); set `ik_link_name="panda_hand"`. It also freely returns far-away branches (joint1 swinging >2 rad) even when seeded -> caused a collision once. Use the local DLS IK in `/workspace/kin.py` (URDF chain, validated to match service FK exactly) via `/workspace/motion.py` instead.
- The trajectory controller silently caps joint speed near ~0.19 rad/s; goals faster than that end with error_code -5 and joints far from target. Pace goals at <=0.15 rad/s per joint (motion.execute does this) -> error 0 every time.
- Gripper only open/closed; on a Ø41 mm bottle fingers stop at ~0.0225 each.

**Why:** These cost a knocked-over bottle and a pushed-in drawer before being discovered.
**How to apply:** Reuse kin.py/motion.py (collision boxes in kin.OBST, hand envelope points) and always dry-run paths before `--go`.
OPENRUA_EOF

# openrua op 70
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda sim quirks](panda-sim-quirks.md) — world-frame IK/FK, ~0.19 rad/s joint speed cap, IK branch jumps; use kin.py/motion.py" >> MEMORY.md
