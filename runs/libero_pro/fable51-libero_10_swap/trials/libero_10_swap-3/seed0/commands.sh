#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | head -40; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once --qos-durability transient_local 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -120

# openrua op 7
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
for _ in range(20): rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_yaml())
for f in ["panda_link0","panda_hand","birdview_optical_frame","agentview_optical_frame","sideview_optical_frame","frontview_optical_frame","robot0_eye_in_hand_optical_frame"]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, f"{tr.x:.4f} {tr.y:.4f} {tr.z:.4f}", f"q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    except Exception as e:
        print(f, "ERR", e)
EOF
timeout 60 python3 tfdump.py

# openrua op 8
cat > cloud.py <<'EOF'
"""Grab color+depth+info from a camera, save npz with world-frame points."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while not all(k in got for k in "dci") or not buf.can_transform("world", f"{cam}_optical_frame", rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", f"{cam}_optical_frame", rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.savez(f"{cam}_cloud.npz", pw=pw, color=color, depth=depth)
print("saved", pw.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
EOF
timeout 60 python3 cloud.py birdview && timeout 60 python3 cloud.py agentview

# openrua op 9
cat > analyze.py <<'EOF'
import numpy as np
d = np.load("birdview_cloud.npz"); pw, color = d["pw"], d["color"]
def at(u,v): 
    p = pw[v,u]; return f"({u},{v}) -> x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}  bgr={color[v,u]}"
# table surface sample
for uv in [(320,400),(330,258),(320,258),(340,258),(330,248),(330,268),(305,312),(275,265),(260,265),(295,265),(220,265),(300,235),(300,300),(190,230),(250,300)]:
    print(at(*uv))
# height map: z above table
z = pw[...,2]
print("table z median (region):", np.median(z[380:450, 200:450]))
EOF
python3 analyze.py

# openrua op 10
cat > analyze2.py <<'EOF'
import numpy as np, cv2
d = np.load("birdview_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]
# print a coarse height map over the table region (x -0.35..0.45, y -0.55..0.55) in 2cm cells
xs = np.arange(-0.35, 0.30, 0.02); ys = np.arange(-0.55, 0.30, 0.02)
print("      " + "".join(f"{yy:+.2f}"[1:4].rjust(4) if i%5==0 else "    " for i,yy in enumerate(ys)))
for xx in xs:
    row = ""
    for yy in ys:
        m = (np.abs(x-xx)<0.01)&(np.abs(y-yy)<0.01)
        if m.sum()==0: row += "   ."; continue
        h = np.median(z[m]) - 0.90
        row += f"{int(round(h*100)):4d}" if h>0.005 else "   ."
    print(f"{xx:+.2f} {row}")
EOF
python3 analyze2.py

# openrua op 11
cat > analyze3.py <<'EOF'
import numpy as np, sys
cam = sys.argv[1]
d = np.load(f"{cam}_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]
ok = np.isfinite(z)
# Bowl region candidates: x in [-0.22,-0.04], y in [-0.05, 0.14], z in 0.905..1.05
m = ok & (x>-0.22)&(x<-0.04)&(y>-0.05)&(y<0.14)&(z>0.905)&(z<1.10)
print("bowl pts", m.sum())
if m.sum():
    print(" x range", x[m].min(), x[m].max(), " y range", y[m].min(), y[m].max(), " z range", z[m].min(), z[m].max())
    # rim: top 10% of z
    zt = np.percentile(z[m], 95); mt = m & (z>zt-0.005)
    print(" rim z~", zt, " rim center x,y", x[mt].mean(), y[mt].mean(), " rim x range", x[mt].min(), x[mt].max(), " y range", y[mt].min(), y[mt].max())
    for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.06)]:
        mm = m&(z>=lo)&(z<hi)
        if mm.sum(): print(f"  z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
# Drawer: region x in [-0.26,0.03], y in [-0.24,-0.02], z>0.905
m = ok & (x>-0.26)&(x<0.03)&(y>-0.24)&(y<-0.02)&(z>0.905)&(z<1.12)
print("drawer pts", m.sum())
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.06),(1.06,1.12)]:
    mm = m&(z>=lo)&(z<hi)
    if mm.sum(): print(f"  z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
# Bottle
m = ok & (x>0.0)&(x<0.12)&(y>-0.12)&(y<0.0)&(z>0.905)&(z<1.2)
print("bottle pts", m.sum())
if m.sum(): print(" x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max(), "z", z[m].min(), z[m].max())
EOF
python3 analyze3.py agentview; echo ======; python3 analyze3.py birdview

# openrua op 12
timeout 60 python3 cloud.py sideview; timeout 60 python3 cloud.py frontview; cat > analyze4.py <<'EOF'
import numpy as np
for cam in ["agentview","sideview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    z = pw[...,2]; x = pw[...,0]; y = pw[...,1]
    # points within bowl footprint circle radius 7cm around (-0.138,0.04), z>0.902
    r = np.hypot(x+0.138, y-0.040)
    m = np.isfinite(z)&(r<0.075)&(z>0.902)&(z<1.10)
    print(cam, "n", m.sum())
    if m.sum():
        hist, edges = np.histogram(z[m], bins=np.arange(0.90,1.06,0.01))
        for h,e in zip(hist,edges):
            if h: 
                mm = m&(z>=e)&(z<e+0.01)
                print(f"   z {e:.2f}: {h:5d}  r[{r[mm].min():.3f},{r[mm].max():.3f}]")
EOF
python3 analyze4.py

# openrua op 13
cat > crop.py <<'EOF'
import cv2, sys
img = cv2.imread(sys.argv[1]); x0,y0,x1,y1 = map(int, sys.argv[2:6]); s = int(sys.argv[6]) if len(sys.argv)>6 else 4
crop = cv2.resize(img[y0:y1, x0:x1], None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
cv2.imwrite(sys.argv[7] if len(sys.argv)>7 else "crop.png", crop); print("ok")
EOF
python3 crop.py sideview.png 260 220 420 320 4 crop_side.png; python3 crop.py agentview.png 280 170 420 270 4 crop_agent.png

# openrua op 14
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position; timeout 60 python3 cloud.py agentview && python3 analyze4.py 2>/dev/null | head -12

# openrua op 15
timeout 60 python3 cloud.py birdview; timeout 60 python3 cloud.py sideview; cat > analyze5.py <<'EOF'
import numpy as np
for cam in ["agentview","birdview","sideview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    # bowl rim: points z in [0.945,0.97] near (-0.138,0.04)
    r = np.hypot(x+0.138, y-0.040)
    m = ok&(r<0.09)&(z>0.945)&(z<0.975)
    if m.sum(): print(cam, "rim n", m.sum(), "center", round(x[m].mean(),4), round(y[m].mean(),4), "x rng", round(x[m].min(),3), round(x[m].max(),3), "y rng", round(y[m].min(),3), round(y[m].max(),3), "zmax", round(z[m].max(),3))
    # drawer bottom (z 0.905-0.93) and drawer walls/front
    m = ok&(x>-0.28)&(x<0.05)&(y>-0.25)&(y<0.0)&(z>0.905)&(z<1.0)
    for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.0)]:
        mm = m&(z>=lo)&(z<hi)
        if mm.sum()>10: print(f"   {cam} drawer z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    # bottle
    m = ok&(x>-0.02)&(x<0.12)&(y>-0.15)&(y<0.0)&(z>0.93)&(z<1.2)
    if m.sum(): print("   bottle x", round(x[m].min(),3), round(x[m].max(),3), "y", round(y[m].min(),3), round(y[m].max(),3), "zmax", round(z[m].max(),3))
EOF
python3 analyze5.py

# openrua op 16
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 17
timeout 60 python3 cloud.py robot0_eye_in_hand && cat > analyze6.py <<'EOF'
import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
r = np.hypot(x+0.14, y-0.03)
m = ok&(r<0.09)&(z>0.945)&(z<0.975)
print("rim n", m.sum(), "center", x[m].mean(), y[m].mean(), "x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max(), "z", z[m].min(), z[m].max())
# fit circle to rim points
A = np.c_[2*x[m], 2*y[m], np.ones(m.sum())]; b = x[m]**2+y[m]**2
cx, cy, c = np.linalg.lstsq(A,b,rcond=None)[0]; R = np.sqrt(c+cx**2+cy**2)
print("circle fit center", cx, cy, "R", R)
# drawer: x in [-0.26,0.06], y in [-0.26,-0.03]
m = ok&(x>-0.26)&(x<0.06)&(y>-0.26)&(y<-0.03)&(z>0.905)&(z<1.0)
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.0)]:
    mm = m&(z>=lo)&(z<hi)
    if mm.sum()>10: print(f"drawer z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
# height map of drawer region at 1cm
print("drawer heightmap (cm above 0.90), rows x, cols y from -0.26 to -0.02")
ys = np.arange(-0.26,-0.02,0.01); xs=np.arange(-0.26,0.08,0.01)
print("     "+"".join(f"{int(round(yy*100)):4d}" for yy in ys))
for xx in xs:
    row=""
    for yy in ys:
        mm = ok&(np.abs(x-xx)<0.005)&(np.abs(y-yy)<0.005)
        row += f"{int(round((np.median(z[mm])-0.90)*100)):4d}" if mm.sum() else "   ."
    print(f"{int(round(xx*100)):4d} {row}")
EOF
python3 analyze6.py

# openrua op 18
cat > robot.py <<'EOF'
"""Helper lib: joint state, FK/IK via MoveIt, multi-point FJT, gripper."""
import math, time
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])   # panda_link0 in world (from TF)
TCP = M["hand"]["tcp_offset_m"]

def quat_to_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in JOINTS]

    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None: self.spin(0.2)
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk_hand(self, q=None):
        """hand pose in WORLD: (pos[3], quat[4] xyzw)"""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    def ik_hand(self, pos_world, quat, seed=None, timeout=30):
        """IK for hand pose in WORLD -> list of joint positions or None"""
        seed = seed if seed is not None else self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def ik_tcp(self, tcp_world, quat, seed=None):
        R = quat_to_R(*quat)
        return self.ik_hand(np.asarray(tcp_world) - TCP * R[:, 2], quat, seed)

    def move_joints(self, waypoints, seconds, timeout=600):
        """waypoints: list of joint vectors; times spread evenly to `seconds`"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        code = r.result.error_code if r else None
        err = np.abs(np.array(self.arm_q()) - np.array(waypoints[-1])).max()
        return code, err

    def move_tcp_line(self, tcp_target, quat, seconds, step=0.02, seed=None):
        """straight-ish Cartesian line for the TCP via IK waypoints"""
        start, _ = self.tcp()
        tcp_target = np.asarray(tcp_target, float)
        dist = np.linalg.norm(tcp_target - start)
        n = max(1, int(math.ceil(dist / step)))
        seed = seed if seed is not None else self.arm_q()
        wps = []
        for i in range(1, n + 1):
            p = start + (tcp_target - start) * i / n
            q = self.ik_tcp(p, quat, seed)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            # reject solutions that jump far from the seed (branch flips)
            if np.abs(np.array(q) - np.array(seed)).max() > 1.0:
                raise RuntimeError(f"IK branch jump at waypoint {i}/{n}: {np.round(q,3)} vs {np.round(seed,3)}")
            wps.append(q); seed = q
        return self.move_joints(wps, seconds)

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()
EOF
cat > probe.py <<'EOF'
from robot import *
r = Robot("probe")
print("q", np.round(r.arm_q(),4))
pos, quat = r.fk_hand(); print("hand", np.round(pos,4), np.round(quat,4))
tcp, _ = r.tcp(); print("tcp", np.round(tcp,4))
print("fingers", r.fingers())
print("wrench", r.wrench())
# finger positions via TF
from tf2_ros import Buffer, TransformListener
buf = Buffer(); TransformListener(buf, r.node)
for _ in range(20): r.spin(0.1)
for f in ["panda_leftfinger","panda_rightfinger","panda_hand"]:
    t = buf.lookup_transform("world", f, rclpy.time.Time()).transform.translation
    print(f, round(t.x,4), round(t.y,4), round(t.z,4))
# IK test for both candidate orientations
for quat in [(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0)]:
    q = r.ik_tcp([-0.189, 0.022, 1.05], quat)
    print("IK", quat, None if q is None else np.round(q,3))
EOF
timeout 120 python3 probe.py

# openrua op 19
cat > probe2.py <<'EOF'
from robot import *
import robot
r = Robot("probe2")
q0 = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = q0
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
ps = fut.result().pose_stamped[0]
print("FK frame:", repr(ps.header.frame_id), "pos", ps.pose.position)
# IK with world coords (no offset)
robot.BASE = np.zeros(3)
q = r.ik_hand([-0.203, 0.0, 1.2696], [1,0,0,0])
print("IK world-coords:", None if q is None else np.round(q,3))
robot.BASE = np.array([-0.66,0,0.912])
q = r.ik_hand([-0.203, 0.0, 1.2696], [1,0,0,0])
print("IK base-offset:", None if q is None else np.round(q,3))
EOF
timeout 120 python3 probe2.py

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])   # panda_link0 in world (from TF)", "new_string": "BASE = np.zeros(3)   # MoveIt model frame here IS world (FK header says world; verified by IK round-trip)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
cat > probe3.py <<'EOF'
from robot import *
r = Robot("probe3")
tcp, quat = r.tcp(); print("tcp now", np.round(tcp,4), np.round(quat,4))
for quat in [(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0)]:
    q = r.ik_tcp([-0.189, 0.022, 1.05], quat)
    print("IK hover", quat, None if q is None else np.round(q,3))
    if q is not None:
        t, qq = r.tcp(q); print("   FK check tcp", np.round(t,4), np.round(qq,4))
        q2 = r.ik_tcp([-0.189, 0.022, 0.938], quat, q); print("   IK grasp depth", None if q2 is None else np.round(q2,3))
        q3 = r.ik_tcp([-0.16, -0.16, 0.955], quat, q); print("   IK place", None if q3 is None else np.round(q3,3))
EOF
timeout 120 python3 probe3.py

# openrua op 22
python3 - <<'EOF'
import re
s = open("robot.py").read()
s = s.replace('req.ik_request.pose_stamped.header.frame_id = ""',
 'req.ik_request.pose_stamped.header.frame_id = ""\n        req.ik_request.ik_link_name = "panda_hand"  # group tip is link8 (45 deg yaw off); target the hand explicitly')
open("robot.py","w").write(s)
EOF
timeout 120 python3 probe3.py

# openrua op 23
cat > step1.py <<'EOF'
from robot import *
r = Robot("step1")
Q = (0.7071, 0.7071, 0, 0)
q = r.ik_tcp([-0.189, 0.022, 1.05], Q)
print("target q", np.round(q,3))
code, err = r.move_joints([q], 4.0)
print("code", code, "max joint err", round(err,4))
tcp, quat = r.tcp(); print("tcp now", np.round(tcp,4), np.round(quat,4))
EOF
timeout 600 python3 -u step1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
cat > resend.py <<'EOF'
import sys
from robot import *
r = Robot("resend")
q = [float(v) for v in sys.argv[1].split(",")]; secs = float(sys.argv[2]) if len(sys.argv)>2 else 3.0
print("before", np.round(r.arm_q(),3))
code, err = r.move_joints([q], secs)
print("code", code, "max joint err", round(err,4), "now", np.round(r.arm_q(),3))
tcp, quat = r.tcp(); print("tcp now", np.round(tcp,4), np.round(quat,4))
EOF
timeout 600 python3 -u resend.py 0.123,0.114,-0.074,-2.497,0.017,2.61,-0.751 3

# openrua op 25
timeout 600 python3 -u resend.py 0.123,0.114,-0.074,-2.497,0.017,2.61,-0.751 3

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
timeout 60 python3 cloud.py robot0_eye_in_hand && cat > bowlfit.py <<'EOF'
import numpy as np, sys
cam = sys.argv[1] if len(sys.argv)>1 else "robot0_eye_in_hand"
d = np.load(f"{cam}_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
cx0, cy0 = float(sys.argv[2]) if len(sys.argv)>2 else -0.134, float(sys.argv[3]) if len(sys.argv)>3 else 0.022
zlo = float(sys.argv[4]) if len(sys.argv)>4 else 0.945
r = np.hypot(x-cx0, y-cy0)
m = ok&(r<0.09)&(z>zlo)&(z<zlo+0.03)
print("rim n", m.sum(), "x", round(x[m].min(),4), round(x[m].max(),4), "y", round(y[m].min(),4), round(y[m].max(),4), "z", round(z[m].min(),4), round(z[m].max(),4))
# outer boundary: for angle bins take max r
ang = np.arctan2(y[m]-cy0, x[m]-cx0); rr = r[m]
pts=[]
for a in np.arange(-np.pi, np.pi, np.pi/18):
    mm = (ang>=a)&(ang<a+np.pi/18)
    if mm.sum(): 
        i = np.argmax(rr[mm]); pts.append((x[m][mm][i], y[m][mm][i]))
pts=np.array(pts)
A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]; b = pts[:,0]**2+pts[:,1]**2
cx, cy, c = np.linalg.lstsq(A,b,rcond=None)[0]; R = np.sqrt(c+cx**2+cy**2)
print(f"outer circle: center ({cx:.4f},{cy:.4f}) R {R:.4f}  from {len(pts)} boundary pts")
print("boundary pts x-min", pts[:,0].min(), "x-max", pts[:,0].max(), "y-min", pts[:,1].min(), "y-max", pts[:,1].max())
EOF
python3 bowlfit.py robot0_eye_in_hand

# openrua op 28
cat > step2.py <<'EOF'
from robot import *
r = Robot("step2")
Q = (0.7071, 0.7071, 0, 0)
gx, gy = -0.192, 0.041
code, err = r.move_tcp_line([gx, gy, 1.05], Q, 2.0)
print("shift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4))
code, err = r.move_tcp_line([gx, gy, 0.938], Q, 4.0)
print("descend code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), np.round(r.tcp()[1],3))
print("wrench", np.round(r.wrench()[0],3))
EOF
timeout 900 python3 -u step2.py

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py sideview && python3 crop.py sideview.png 260 200 460 340 3 crop_side.png

# openrua op 30
cat > grasp.py <<'EOF'
from robot import *
r = Robot("grasp")
print("close:", r.gripper(0.0))
print("fingers", r.fingers(), "wrench", np.round(r.wrench()[0],3))
EOF
timeout 300 python3 -u grasp.py

# openrua op 31
cat > lift.py <<'EOF'
from robot import *
r = Robot("lift")
Q = (0.7071, 0.7071, 0, 0)
t,_ = r.tcp()
code, err = r.move_tcp_line([t[0], t[1], 1.06], Q, 3.0)
print("lift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4))
print("fingers", r.fingers(), "wrench", np.round(r.wrench()[0],3))
EOF
timeout 600 python3 -u lift.py && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py sideview && python3 crop.py sideview.png 200 150 480 360 3 crop_side.png

# openrua op 33
timeout 60 python3 cloud.py agentview && timeout 60 python3 cloud.py frontview && timeout 60 python3 cloud.py birdview && for c in agentview frontview birdview; do python3 bowlfit.py $c -0.137 0.041 1.066; done

# openrua op 34
cat > handext.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    for lo,hi in [(1.075,1.10),(1.10,1.13),(1.13,1.16),(1.16,1.20),(1.20,1.25)]:
        m = ok&(z>=lo)&(z<hi)&(x>-0.35)&(x<0.0)&(y>-0.1)&(y<0.2)
        if m.sum()>5: print(f"{cam} z {lo}-{hi}: n={m.sum()} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}]")
EOF
python3 handext.py

# openrua op 35
cat > place.py <<'EOF'
from robot import *
r = Robot("place")
Q = (0.7071, 0.7071, 0, 0)
tx, ty = -0.1467, -0.158
code, err = r.move_tcp_line([tx, ty, 1.06], Q, 4.0)
print("transport code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4))
code, err = r.move_tcp_line([tx, ty, 1.0], Q, 2.5)
print("descend code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), np.round(r.tcp()[1],3), "fingers", np.round(r.fingers(),4))
print("wrench", np.round(r.wrench()[0],3))
EOF
timeout 900 python3 -u place.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 36
python3 crop.py sideview.png 200 120 480 360 3 crop_side.png

# openrua op 37
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py frontview; python3 crop.py frontview.png 150 220 350 400 3 crop_front.png

# openrua op 38
for c in agentview frontview sideview birdview; do timeout 60 python3 cloud.py $c >/dev/null; done; cat > state.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview","sideview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    # bowl rim: near (-0.08,-0.156), z 0.98..1.05, r<0.09
    r = np.hypot(x+0.08, y+0.156)
    m = ok&(r<0.10)&(z>0.975)&(z<1.045)&(x>-0.2)
    if m.sum()>10:
        print(f"{cam}: bowl-ish pts n={m.sum()} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}] z[{z[m].min():.3f},{z[m].max():.3f}]")
        for lo in np.arange(0.975,1.045,0.01):
            mm = m&(z>=lo)&(z<lo+0.01)
            if mm.sum()>3: print(f"    z {lo:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
EOF
python3 state.py

# openrua op 39
cat > lift2.py <<'EOF'
from robot import *
r = Robot("lift2")
Q = (0.7071, 0.7071, 0, 0)
t,_ = r.tcp()
code, err = r.move_tcp_line([t[0], t[1], 1.08], Q, 2.5)
print("lift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],3))
code, err = r.move_tcp_line([-0.16, -0.05, 1.08], Q, 2.5)
print("shift code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],3))
EOF
timeout 900 python3 -u lift2.py && for c in agentview frontview sideview birdview; do timeout 60 python3 cloud.py $c >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 40
cat > drawer_geo.py <<'EOF'
import numpy as np
for cam in ["birdview","agentview","frontview","sideview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    print("==", cam)
    # y-profile of max height along the drawer center line x in [-0.15,-0.05], for y from -0.26 to -0.04 at 5mm
    row = []
    for yy in np.arange(-0.26, -0.03, 0.005):
        m = ok&(x>-0.15)&(x<-0.05)&(np.abs(y-yy)<0.0025)
        row.append(f"{yy:+.3f}:{(np.max(z[m])-0.9)*100:4.1f}" if m.sum() else f"{yy:+.3f}:  --")
    print(" y-profile (max z cm) x∈[-0.15,-0.05]:"); print("  " + "  ".join(row[:16])); print("  " + "  ".join(row[16:32])); print("  " + "  ".join(row[32:]))
    row = []
    for xx in np.arange(-0.26, 0.06, 0.005):
        m = ok&(y>-0.20)&(y<-0.11)&(np.abs(x-xx)<0.0025)
        row.append(f"{xx:+.3f}:{(np.max(z[m])-0.9)*100:4.1f}" if m.sum() else f"{xx:+.3f}:  --")
    print(" x-profile (max z cm) y∈[-0.20,-0.11]:"); print("  " + "  ".join(row[:16])); print("  " + "  ".join(row[16:32])); print("  " + "  ".join(row[32:48])); print("  " + "  ".join(row[48:]))
EOF
python3 drawer_geo.py

# openrua op 41
cat > handle_geo.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview","birdview","sideview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    m = ok&(y>-0.219)&(y<-0.12)&(z>1.0)&(z<1.135)&(x>-0.22)&(x<0.02)
    print("==", cam, "pts in front of cabinet face, z>1.0:", m.sum())
    for lo in np.arange(1.0,1.135,0.01):
        mm = m&(z>=lo)&(z<lo+0.01)
        if mm.sum()>3: print(f"   z {lo:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    # cabinet face y position: points with z in [1.03,1.12], x in [-0.2,0], y>-0.3
    m = ok&(z>1.03)&(z<1.12)&(x>-0.2)&(x<0.0)&(y>-0.3)&(y<-0.15)
    if m.sum(): print("   face-ish pts y percentiles 5/50/95:", np.percentile(y[m],[5,50,95]).round(4))
EOF
python3 handle_geo.py

# openrua op 42
cat > handle2.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    m = ok&(y>-0.205)&(y<-0.19)&(z>0.98)&(z<1.06)&(x>-0.16)&(x<-0.04)
    print(cam, "middle handle bar: n", m.sum(), "z pct 1/5/50/95/99", np.percentile(z[m],[1,5,50,95,99]).round(4), "x", x[m].min().round(3), x[m].max().round(3), "y", y[m].min().round(3), y[m].max().round(3))
    # bottom drawer front panel top and inner face
    m = ok&(y>-0.11)&(y<-0.06)&(z>0.93)&(z<1.0)&(x>-0.19)&(x<-0.02)
    print(cam, "front panel: n", m.sum(), "z pct 50/95/99", np.percentile(z[m],[50,95,99]).round(4), "y pct 1/5/50/95/99", np.percentile(y[m],[1,5,50,95,99]).round(4))
    # bowl now: rim ring near TCP (-0.1595,-0.0504,1.08): bowl center approx (-0.107,-0.05)
    r = np.hypot(x+0.107, y+0.050)
    m = ok&(r<0.075)&(z>1.06)&(z<1.12)
    print(cam, "bowl rim: n", m.sum(), "z pct 50/95/99", np.percentile(z[m],[50,95,99]).round(4), "x", x[m].min().round(4), x[m].max().round(4), "y", y[m].min().round(4), y[m].max().round(4))
EOF
python3 handle2.py

# openrua op 43
cat > handext2.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview","sideview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    print("==", cam, "(TCP -0.1595,-0.0504,1.08; hand frame z=1.183)")
    for lo in np.arange(1.10,1.30,0.01):
        m = ok&(z>=lo)&(z<lo+0.01)&(x>-0.30)&(x<-0.03)&(y>-0.18)&(y<0.15)
        if m.sum()>5: print(f"   z {lo:.2f}: n={m.sum():4d} x[{x[m].min():.3f},{x[m].max():.3f}] y[{y[m].min():.3f},{y[m].max():.3f}]  (y rel TCP: {y[m].min()+0.0504:+.3f} .. {y[m].max()+0.0504:+.3f})")
EOF
python3 handext2.py

# openrua op 44
timeout 20 ros2 topic info /pipeline_state; timeout 20 ros2 topic echo /pipeline_state --once 2>&1 | head -40

# openrua op 45
cat state.py 2>/dev/null | head -30; timeout 60 python3 state.py 2>&1 | tail -15

# openrua op 46
cat > cur.py <<'EOF'
import numpy as np
from robot import Robot
r = Robot("cur")
q = r.arm_q(); t,quat = r.tcp(q); f = r.fingers(); w = r.wrench()
print("q", np.round(q,4).tolist()); print("tcp", np.round(t,4), "quat", np.round(quat,4)); print("fingers", f); print("wrench", np.round(w[0],2), np.round(w[1],2))
EOF
timeout 120 python3 cur.py 2>&1 | tail -5

# openrua op 47
cat cloud.py | head -40; timeout 120 python3 cloud.py frontview 2>&1 | tail -2; timeout 120 python3 cloud.py sideview 2>&1 | tail -2

# openrua op 48
cat > handz.py <<'EOF'
import numpy as np
TCP=np.array([-0.1595,-0.0504,1.08])
for cam in ["frontview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]; ok=np.isfinite(z)
    # column around the hand/bowl: x in [-0.30,0.0], y in [-0.12,0.10], z>0.95
    m=ok&(x>-0.30)&(x<0.0)&(y>-0.12)&(y<0.12)&(z>0.95)&(z<1.35)
    print(cam, m.sum())
    for lo in np.arange(0.95,1.35,0.01):
        mm=m&(z>=lo)&(z<lo+0.01)
        if mm.sum()>3: print(f"  z {lo:.2f} ({lo-1.08:+.3f}): n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
EOF
python3 handz.py

# openrua op 49
cat lift2.py; cat resend.py | head -20

# openrua op 50
cat > setdown.py <<'EOF'
from robot import *
r = Robot("setdown")
Q = (0.7071, 0.7071, 0, 0)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],2), flush=True)
rep("move", *r.move_tcp_line([-0.189, 0.039, 1.08], Q, 3.0))
rep("descend", *r.move_tcp_line([-0.189, 0.039, 0.985], Q, 2.0))
rep("descend2", *r.move_tcp_line([-0.189, 0.039, 0.938], Q, 2.0))
print("open", r.gripper(0.04), flush=True)
rep("lift", *r.move_tcp_line([-0.189, 0.039, 1.0], Q, 2.0))
EOF
timeout 600 python3 -u setdown.py 2>&1 | tail -8

# openrua op 51
timeout 120 python3 cloud.py birdview 2>&1 | tail -1; cat bowlfit.py | head -30; timeout 60 python3 bowlfit.py birdview -0.14 0.04 0.93 2>&1 | tail -6

# openrua op 52
for c in agentview frontview sideview; do timeout 120 python3 cloud.py $c 2>&1 | tail -1; timeout 60 python3 bowlfit.py $c -0.14 0.03 0.93 2>&1 | tail -3; done

# openrua op 53
timeout 60 python3 bowlfit.py agentview -0.14 0.02 0.945 2>&1 | tail -3; timeout 60 python3 bowlfit.py frontview -0.14 0.02 0.945 2>&1 | tail -3; python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=np.isfinite(z)&(np.hypot(x+0.14,y-0.02)<0.1)&(z>0.905)&(z<0.97)
for lo in np.arange(0.905,0.97,0.005):
    mm=m&(z>=lo)&(z<lo+0.005)
    if mm.sum()>3: print(f"z {lo:.3f} n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
EOF

# openrua op 54
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    m=np.isfinite(z)&(x>-0.25)&(x<-0.05)&(y>-0.04)&(y<0.12)&(z>0.945)&(z<0.965)
    cx0,cy0=-0.14,0.02
    r=np.hypot(x[m]-cx0,y[m]-cy0); ang=np.arctan2(y[m]-cy0,x[m]-cx0)
    pts=[]
    for a in np.arange(-np.pi,np.pi,np.pi/18):
        mm=(ang>=a)&(ang<a+np.pi/18)
        if mm.sum(): i=np.argmax(r[mm]); pts.append((x[m][mm][i],y[m][mm][i]))
    pts=np.array(pts); A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=pts[:,0]**2+pts[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c+cx**2+cy**2)
    res=np.hypot(pts[:,0]-cx,pts[:,1]-cy)-R
    print(cam, f"n={m.sum()} center ({cx:.4f},{cy:.4f}) R {R:.4f} resid max {np.abs(res).max():.4f} ztop {z[m].max():.4f}")
EOF

# openrua op 55
cat > regrasp.py <<'EOF'
from robot import *
r = Robot("regrasp")
Q = (0.7071, 0.7071, 0, 0)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],2), flush=True)
X, Y = -0.150-0.050, 0.031
rep("hover", *r.move_tcp_line([X, Y, 1.0], Q, 2.0))
rep("descend", *r.move_tcp_line([X, Y, 0.947], Q, 2.0))
print("close", r.gripper(0.0), flush=True)
rep("lift", *r.move_tcp_line([X, Y, 1.08], Q, 2.5))
EOF
timeout 600 python3 -u regrasp.py 2>&1 | tail -6

# openrua op 56
timeout 120 python3 cloud.py frontview 2>&1 | tail -1; timeout 120 python3 cloud.py agentview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    m=np.isfinite(z)&(x>-0.30)&(x<-0.05)&(y>-0.05)&(y<0.12)&(z>0.98)&(z<1.16)
    print(cam)
    for lo in np.arange(0.98,1.16,0.01):
        mm=m&(z>=lo)&(z<lo+0.01)
        if mm.sum()>3: print(f"  z {lo:.2f} ({lo-1.0795:+.3f}) n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    # rim fit at top
    mm=m&(z>1.07)&(z<1.10)&(y>-0.05)
    if mm.sum()>20:
        cx0,cy0=-0.15,0.031; r=np.hypot(x[mm]-cx0,y[mm]-cy0); ang=np.arctan2(y[mm]-cy0,x[mm]-cx0); pts=[]
        for a in np.arange(-np.pi,np.pi,np.pi/18):
            k=(ang>=a)&(ang<a+np.pi/18)
            if k.sum(): i=np.argmax(r[k]); pts.append((x[mm][k][i],y[mm][k][i]))
        pts=np.array(pts); A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=pts[:,0]**2+pts[:,1]**2
        cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; print("  rim circle", round(cx,4), round(cy,4), "R", round(np.sqrt(c+cx**2+cy**2),4), "ztop", round(z[mm].max(),4))
EOF

# openrua op 57
timeout 120 python3 cloud.py sideview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    m=np.isfinite(z)&(x>-0.16)&(x<-0.04)&(y>-0.215)&(y<-0.17)&(z>0.99)&(z<1.12)
    print(cam, m.sum())
    for lo in np.arange(0.99,1.12,0.005):
        mm=m&(z>=lo)&(z<lo+0.005)
        if mm.sum()>2: print(f"  z {lo:.3f} n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
EOF

# openrua op 58
python3 - <<'EOF'
import numpy as np
T=np.array([-0.2001,0.0313,1.0795])
for cam in ["frontview","agentview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    m=np.isfinite(z)&(x>-0.32)&(x<-0.08)&(y>-0.06)&(y<0.20)&(z>1.10)&(z<1.26)
    print(cam, m.sum())
    for lo in np.arange(1.10,1.26,0.005):
        mm=m&(z>=lo)&(z<lo+0.005)
        if mm.sum()>2: print(f"  z {lo:.3f} ({lo-T[2]:+.3f}) n={mm.sum():4d} x[{x[mm].min()-T[0]:+.3f},{x[mm].max()-T[0]:+.3f}] y[{y[mm].min()-T[1]:+.3f},{y[mm].max()-T[1]:+.3f}]")
EOF

# openrua op 59
cat > place2.py <<'EOF'
import sys
from robot import *
r = Robot("place2")
Q = (0.7071, 0.7071, 0, 0)
f0 = r.wrench()[0]
def rep(tag, code, err, stop=True):
    t = r.tcp()[0]; f = r.fingers(); w = r.wrench()[0]
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(t,4), "fingers", np.round(f,4), "wrench", np.round(w,2), flush=True)
    if stop and (np.abs(w - f0).max() > 3.0 or abs(f[0]) < 0.0005):
        print("ABORT: contact or lost grasp", flush=True); sys.exit(1)
X = -0.156
rep("transport", *r.move_tcp_line([X, -0.05, 1.08], Q, 3.0))
rep("approach", *r.move_tcp_line([X, -0.138, 1.035], Q, 2.5))
rep("descend a", *r.move_tcp_line([X, -0.138, 1.015], Q, 1.5))
rep("descend b", *r.move_tcp_line([X, -0.138, 0.997], Q, 1.5))
rep("slide", *r.move_tcp_line([X, -0.155, 0.997], Q, 1.5))
EOF
timeout 900 python3 -u place2.py 2>&1 | tail -8

# openrua op 60
timeout 120 python3 cloud.py agentview 2>&1 | tail -1; timeout 120 python3 cloud.py frontview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    m=np.isfinite(z)&(x>-0.21)&(x<0.0)&(y>-0.23)&(y<-0.08)&(z>0.94)&(z<1.02)
    print(cam, m.sum())
    for lo in np.arange(0.94,1.02,0.01):
        mm=m&(z>=lo)&(z<lo+0.01)
        if mm.sum()>3: print(f"  z {lo:.2f} n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
EOF

# openrua op 61
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    cx0,cy0=-0.105,-0.153
    m=np.isfinite(z)&(np.hypot(x-cx0,y-cy0)<0.065)&(z>0.94)&(z<1.012)
    print(cam, m.sum())
    for lo in np.arange(0.94,1.012,0.005):
        mm=m&(z>=lo)&(z<lo+0.005)
        if mm.sum()>3: print(f"  z {lo:.3f} n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    mm=m&(z>0.99)
    r=np.hypot(x[mm]-cx0,y[mm]-cy0); ang=np.arctan2(y[mm]-cy0,x[mm]-cx0); pts=[]
    for a in np.arange(-np.pi,np.pi,np.pi/18):
        k=(ang>=a)&(ang<a+np.pi/18)
        if k.sum(): i=np.argmax(r[k]); pts.append((x[mm][k][i],y[mm][k][i]))
    pts=np.array(pts); A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=pts[:,0]**2+pts[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; print("  rim circle", round(cx,4), round(cy,4), "R", round(np.sqrt(c+cx**2+cy**2),4), "from", len(pts))
EOF

# openrua op 62
cat > release.py <<'EOF'
from robot import *
r = Robot("release")
Q = (0.7071, 0.7071, 0, 0)
print("open", r.gripper(0.04), flush=True)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench()[0],2), flush=True)
rep("lift", *r.move_tcp_line([-0.156, -0.155, 1.08], Q, 2.0))
rep("back", *r.move_tcp_line([-0.156, -0.05, 1.10], Q, 2.0))
EOF
timeout 600 python3 -u release.py 2>&1 | tail -4

# openrua op 63
timeout 120 python3 cloud.py agentview 2>&1 | tail -1; timeout 120 python3 cloud.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["agentview","birdview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    cx0,cy0=-0.105,-0.153
    m=np.isfinite(z)&(np.hypot(x-cx0,y-cy0)<0.07)&(z>0.915)&(z<1.0)
    print(cam, m.sum())
    for lo in np.arange(0.915,1.0,0.005):
        mm=m&(z>=lo)&(z<lo+0.005)
        if mm.sum()>3: print(f"  z {lo:.3f} n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    mm=m&(z>0.965)&(z<0.985)
    if mm.sum()>20:
        r=np.hypot(x[mm]-cx0,y[mm]-cy0); ang=np.arctan2(y[mm]-cy0,x[mm]-cx0); pts=[]
        for a in np.arange(-np.pi,np.pi,np.pi/18):
            k=(ang>=a)&(ang<a+np.pi/18)
            if k.sum(): i=np.argmax(r[k]); pts.append((x[mm][k][i],y[mm][k][i]))
        pts=np.array(pts); A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=pts[:,0]**2+pts[:,1]**2
        cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; print("  rim circle", round(cx,4), round(cy,4), "R", round(np.sqrt(c+cx**2+cy**2),4), "ztop", round(z[mm].max(),4))
EOF

# openrua op 64
timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | tail -1; python3 crop.py agentview.png 150 120 500 400 2 agent_crop.png 2>&1 | tail -1

# openrua op 65
cat > base.py <<'EOF'
from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("base")
q = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0","panda_link4","panda_link6","panda_link7","panda_hand"]
req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for n, ps in zip(fut.result().fk_link_names, fut.result().pose_stamped):
    p = ps.pose.position; print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4))
EOF
timeout 60 python3 base.py 2>&1 | tail -5

# openrua op 66
python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=np.isfinite(z)&(x>-0.40)&(x<0.10)&(y>-0.35)&(y<0.05)&(z>0.905)&(z<1.15)
# occupancy grid in x,y: max z
xs=np.arange(-0.40,0.10,0.02); ys=np.arange(-0.35,0.05,0.02)
print("max z per cell (rows: x from -0.40; cols: y from -0.35 to 0.05)")
print("      "+" ".join(f"{v:+.2f}" for v in ys))
for xa in xs:
    row=[]
    for ya in ys:
        mm=m&(x>=xa)&(x<xa+0.02)&(y>=ya)&(y<ya+0.02)
        row.append(f"{z[mm].max():.3f}" if mm.sum()>2 else "  .  ")
    print(f"{xa:+.2f} "+" ".join(row))
EOF

# openrua op 67
cat > iktest.py <<'EOF'
from robot import *
r = Robot("iktest")
Qp = (0.5,-0.5,0.5,0.5)
print(np.round(quat_to_R(*Qp),3))
seed = r.arm_q()
for p in [(-0.19,-0.02,0.955),(-0.19,-0.06,0.955),(-0.19,-0.14,0.955),(-0.19,-0.22,0.955),(-0.19,-0.02,1.05)]:
    q = r.ik_tcp(p, Qp, seed)
    print(p, None if q is None else np.round(q,3))
    if q is not None:
        t,qq = r.tcp(q); print("   fk tcp", np.round(t,4), np.round(qq,3))
EOF
timeout 120 python3 iktest.py 2>&1 | tail -14

# openrua op 68
cat > iktest2.py <<'EOF'
from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("iktest2")
Qp = (0.5,-0.5,0.5,0.5)
LIM = np.array(FJT["limits_rad"])
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return {n:(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for n,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped)}
rng = np.random.default_rng(0)
sols = []
for i in range(40):
    seed = rng.uniform(LIM[:,0]*0.6, LIM[:,1]*0.6)
    q = r.ik_tcp((-0.19,-0.06,0.955), Qp, seed)
    if q is None: continue
    q = np.array(q)
    if abs(q[6])>2.6 or abs(q[0])>1.2: continue
    fk = fkall(q)
    zmin = min(v[2] for v in fk.values())
    key = tuple(np.round(q,1))
    if any(np.abs(np.array(s[0])-q).max()<0.2 for s in sols): continue
    sols.append((q, zmin, fk))
    print(np.round(q,3), "zmin", round(zmin,3), {k:np.round(v,3).tolist() for k,v in fk.items() if k in ("panda_link4","panda_link6","panda_link7")})
print(len(sols))
EOF
timeout 300 python3 iktest2.py 2>&1 | tail -20

# openrua op 69
cat > plan_push.py <<'EOF'
from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("planpush")
Qd = (0.7071,0.7071,0,0); Qp = (0.5,-0.5,0.5,0.5)
LINKS = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
q0 = np.array(r.arm_q())
qa = r.ik_tcp((-0.19,0.05,1.25), Qd, q0); print("qa", np.round(qa,3))
seedp = [0.01,0.899,0.318,-2.01,-1.421,1.281,-2.567]
qb = r.ik_tcp((-0.19,0.05,1.10), Qp, seedp); print("qb", np.round(qb,3))
t,qq = r.tcp(qb); print(" qb tcp", np.round(t,3), np.round(qq,3))
for i in range(0,21,2):
    q = np.array(qa) + (np.array(qb)-np.array(qa))*i/20
    P = fkall(q); tcp,_ = r.tcp(q)
    print(i, "minz links", round(P[:,2].min(),3), "tcp", np.round(tcp,3), "link7", np.round(P[4],3), "link6", np.round(P[3],3))
np.save("qa.npy", qa); np.save("qb.npy", qb)
EOF
timeout 300 python3 plan_push.py 2>&1 | tail -16

# openrua op 70
cat > push_dry.py <<'EOF'
from robot import *
r = Robot("pushdry")
Qp = (0.5,-0.5,0.5,0.5)
qb = np.load("qb.npy"); seed = list(qb)
path = [(-0.19,0.05,1.10)]
for z in np.arange(1.08, 0.96, -0.02): path.append((-0.19, 0.05, z))
path.append((-0.19,0.05,0.965))
for y in np.arange(0.03, -0.235, -0.02): path.append((-0.19, y, 0.965))
qs=[]
for p in path:
    q = r.ik_tcp(p, Qp, seed)
    if q is None: print("IK FAIL", p); break
    if np.abs(np.array(q)-np.array(seed)).max()>1.0: print("JUMP", p, np.round(q,3)); break
    qs.append(q); seed=q
    print(np.round(p,3), np.round(q,3))
EOF
timeout 300 python3 push_dry.py 2>&1 | tail -22

# openrua op 71
cat > push1.py <<'EOF'
from robot import *
r = Robot("push1")
Qd = (0.7071,0.7071,0,0); Qp = (0.5,-0.5,0.5,0.5)
def rep(tag, code, err):
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(r.tcp()[0],4), "q", np.round(r.arm_q(),3), "wrench", np.round(r.wrench()[0],2), flush=True)
print("close", r.gripper(0.0), flush=True)
rep("up", *r.move_tcp_line([-0.19,0.05,1.25], Qd, 3.0))
qb = list(np.load("qb.npy"))
for i in range(4):
    code, err = r.move_joints([qb], 12.0 if i==0 else 4.0)
    rep(f"reconf{i}", code, err)
    if err < 0.01: break
rep("down", *r.move_tcp_line([-0.19,0.05,0.965], Qp, 3.0))
rep("fwd", *r.move_tcp_line([-0.19,-0.03,0.965], Qp, 2.0))
EOF
timeout 900 python3 -u push1.py 2>&1 | tail -8

# openrua op 72
timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py sideview 2>&1 | tail -1

# openrua op 73
cat > recover.py <<'EOF'
from robot import *
r = Robot("recover")
qa = list(np.load("qa.npy"))
for i in range(3):
    code, err = r.move_joints([qa], 8.0 if i==0 else 4.0)
    print("back", i, code, round(err,4), np.round(r.arm_q(),3), "tcp", np.round(r.tcp()[0],3), "wrench", np.round(r.wrench()[0],2), flush=True)
    if err < 0.01: break
EOF
timeout 600 python3 -u recover.py 2>&1 | tail -4

# openrua op 74
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "    def ik_hand(self, pos_world, quat, seed=None, timeout=30):\\n        \\"\\"\\"IK for hand pose in WORLD -> list of joint positions or None\\"\\"\\"", "new_string": "    def ik_hand(self, pos_world, quat, seed=None, timeout=30, avoid=False):\\n        \\"\\"\\"IK for hand pose in WORLD -> list of joint positions or None\\"\\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 75
python3 - <<'EOF'
s=open("robot.py").read()
s=s.replace("req.ik_request.avoid_collisions = False","req.ik_request.avoid_collisions = bool(avoid)")
s=s.replace("""    def ik_tcp(self, tcp_world, quat, seed=None):
        R = quat_to_R(*quat)
        return self.ik_hand(np.asarray(tcp_world) - TCP * R[:, 2], quat, seed)""","""    def ik_tcp(self, tcp_world, quat, seed=None, avoid=False):
        R = quat_to_R(*quat)
        return self.ik_hand(np.asarray(tcp_world) - TCP * R[:, 2], quat, seed, avoid=avoid)""")
s=s.replace("""    def move_tcp_line(self, tcp_target, quat, seconds, step=0.02, seed=None):""","""    def move_tcp_line(self, tcp_target, quat, seconds, step=0.02, seed=None, avoid=False):""")
s=s.replace("""            q = self.ik_tcp(p, quat, seed)
            if q is None:""","""            q = self.ik_tcp(p, quat, seed, avoid=avoid)
            if q is None:""")
open("robot.py","w").write(s)
EOF
grep -n "avoid" robot.py

# openrua op 76
cat > iktest3.py <<'EOF'
from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("iktest3")
LIM = np.array(FJT["limits_rad"])
LINKS = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
rng = np.random.default_rng(1)
for name, Q in [("mount-up",(0.5,-0.5,0.5,0.5)),("mount-down",(0.5,0.5,-0.5,0.5))]:
    print("==", name, np.round(quat_to_R(*Q),2).tolist())
    sols=[]
    for i in range(60):
        seed = rng.uniform(LIM[:,0]*0.7, LIM[:,1]*0.7)
        q = r.ik_tcp((-0.19,-0.06,0.965), Q, seed, avoid=True)
        if q is None: continue
        q=np.array(q)
        if any(np.abs(s-q).max()<0.3 for s in sols): continue
        sols.append(q); P=fkall(q)
        print(np.round(q,3), "minz", round(P[:,2].min(),3), "l7", np.round(P[4],3), "l6", np.round(P[3],3), "l4", np.round(P[1],3))
EOF
timeout 600 python3 iktest3.py 2>&1 | tail -20

# openrua op 77
timeout 20 ros2 service list 2>/dev/null | grep -i -E "valid|plan|cartesian" 

# openrua op 78
cat > scene.py <<'EOF'
"""Add table/cabinet/bottle boxes to the MoveIt planning scene; add valid(q) helper."""
from robot import *
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
def box(id_, center, size):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = id_
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives.append(sp); co.primitive_poses.append(p); co.operation = CollisionObject.ADD
    return co
def apply_scene(r):
    cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(10)
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    req.scene.world.collision_objects = [
        box("table", (-0.1, 0.0, 0.85), (1.6, 1.6, 0.10)),          # top at 0.90
        box("cabinet", (-0.11, -0.33, 1.014), (0.26, 0.20, 0.228)),  # x[-0.24,0.02] y[-0.43,-0.23] top 1.128
        box("bottle", (0.045, -0.07, 0.98), (0.07, 0.08, 0.16)),
    ]
    fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return fut.result().success
def validator(r):
    cli = r.node.create_client(GetStateValidity, "/check_state_validity"); cli.wait_for_service(10)
    def valid(q):
        req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
        req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
        fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
        res = fut.result()
        return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
    return valid
if __name__ == "__main__":
    r = Robot("scene"); print("apply", apply_scene(r))
    valid = validator(r)
    print("current valid", valid(r.arm_q()))
    print("qb (collided) valid", valid(list(np.load("qb.npy"))))
    print("mid collided", valid([0.084,0.415,0.429,-2.06,-1.24,1.081,-2.55]))
EOF
timeout 120 python3 scene.py 2>&1 | tail -4

# openrua op 79
timeout 120 python3 cloud.py frontview 2>&1 | tail -1; timeout 120 python3 cloud.py sideview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
T=np.array([-0.19,0.05,1.25])
for cam in ["frontview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    m=np.isfinite(z)&(x>-0.45)&(x<0.05)&(y>-0.15)&(y<0.35)&(z>1.24)&(z<1.75)
    print(cam, m.sum())
    for lo in np.arange(1.25,1.75,0.02):
        mm=m&(z>=lo)&(z<lo+0.02)
        if mm.sum()>2: print(f"  z {lo:.2f} ({lo-T[2]:+.3f}) n={mm.sum():4d} x[{x[mm].min()-T[0]:+.3f},{x[mm].max()-T[0]:+.3f}] y[{y[mm].min()-T[1]:+.3f},{y[mm].max()-T[1]:+.3f}]")
EOF

# openrua op 80
timeout 120 python3 cloud.py agentview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
    m=np.isfinite(z)&(y>-0.095)&(y<-0.035)&(z>0.905)&(z<0.99)&(x>-0.3)&(x<0.05)
    print(cam, "panel/handle region")
    for lo in np.arange(0.905,0.99,0.01):
        mm=m&(z>=lo)&(z<lo+0.01)
        if mm.sum()>3: print(f"  z {lo:.3f} n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
    mm=m&(y>-0.062)  # handle bar only (protrudes beyond the face)
    if mm.sum()>3: print("  handle: x", round(x[mm].min(),3), round(x[mm].max(),3), "y", round(y[mm].min(),3), round(y[mm].max(),3), "z", round(z[mm].min(),3), round(z[mm].max(),3))
    # cabinet -x face
    mm=np.isfinite(z)&(y<-0.24)&(y>-0.40)&(z>0.95)&(z<1.12)&(x<-0.15)
    if mm.sum()>3: print("  cabinet side: x min", round(x[mm].min(),3))
EOF

# openrua op 81
cat > push_dry2.py <<'EOF'
from robot import *
from scene import validator
from moveit_msgs.srv import GetPositionFK
r = Robot("pushdry2")
valid = validator(r)
th = np.deg2rad(45)
d = np.array([np.sin(th), -np.cos(th), 0.0]); hx = np.array([0,0,1.0]); hy = np.cross(d, hx)
R = np.c_[hx, hy, d]
def R2q(R):
    w = np.sqrt(max(0,1+R[0,0]+R[1,1]+R[2,2]))/2
    x = np.sqrt(max(0,1+R[0,0]-R[1,1]-R[2,2]))/2; y = np.sqrt(max(0,1-R[0,0]+R[1,1]-R[2,2]))/2; z = np.sqrt(max(0,1-R[0,0]-R[1,1]+R[2,2]))/2
    x = np.copysign(x, R[2,1]-R[1,2]); y = np.copysign(y, R[0,2]-R[2,0]); z = np.copysign(z, R[1,0]-R[0,1])
    return np.array([x,y,z,w])
Q = R2q(R); print("Q", np.round(Q,4)); print(np.round(quat_to_R(*Q),3)); print(np.round(R,3))
np.save("Qpush.npy", Q)
LIM = np.array(FJT["limits_rad"])
LINKS = ["panda_link4","panda_link6","panda_link7"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
rng = np.random.default_rng(2)
sols=[]
for i in range(80):
    seed = rng.uniform(LIM[:,0]*0.7, LIM[:,1]*0.7)
    q = r.ik_tcp((-0.19, 0.0, 0.972), Q, seed, avoid=True)
    if q is None: continue
    q=np.array(q)
    if any(np.abs(s-q).max()<0.3 for s in sols): continue
    sols.append(q); P=fkall(q)
    print(np.round(q,3), "j6", round(q[5],2), "l4", np.round(P[0],3), "l6", np.round(P[1],3), "l7", np.round(P[2],3))
EOF
timeout 600 python3 push_dry2.py 2>&1 | tail -20

# openrua op 82
python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
m=np.isfinite(z)&(x>-0.9)&(x<0.3)&(y>-0.05)&(y<0.6)&(z>0.905)&(z<1.6)
xs=np.arange(-0.9,0.3,0.05); ys=np.arange(-0.05,0.6,0.05)
print("      "+" ".join(f"{v:+.2f}" for v in ys))
for xa in xs:
    row=[]
    for ya in ys:
        mm=m&(x>=xa)&(x<xa+0.05)&(y>=ya)&(y<ya+0.05)
        row.append(f"{z[mm].max():.2f} " if mm.sum()>2 else "  .   ")
    print(f"{xa:+.2f} "+"".join(row))
EOF

# openrua op 83
python3 - <<'EOF'
s=open("scene.py").read()
s=s.replace('''        box("bottle", (0.045, -0.07, 0.98), (0.07, 0.08, 0.16)),
    ]''','''        box("bottle", (0.045, -0.07, 0.98), (0.07, 0.08, 0.16)),
        box("shelf", (0.0, 0.30, 1.07), (0.30, 0.30, 0.34)),   # wooden rack x[-0.15,0.15] y[0.15,0.45] top 1.24
    ]''')
open("scene.py","w").write(s)
EOF
cat > plan_push2.py <<'EOF'
from robot import *
from scene import apply_scene, validator
from moveit_msgs.srv import GetPositionFK
r = Robot("planpush2"); print("scene", apply_scene(r)); valid = validator(r)
Q = np.load("Qpush.npy")
LINKS = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
qa = np.array(r.arm_q()); print("qa", np.round(qa,3))
seed = [1.259, 0.91, -0.725, -2.425, -1.63, 2.12, -2.282]
qc = r.ik_tcp((-0.19, 0.0, 1.10), Q, seed, avoid=True); print("qc", np.round(qc,3), valid(qc))
t,qq = r.tcp(qc); print(" qc tcp", np.round(t,3), np.round(qq,3))
bad = False
for i in range(0,21):
    q = qa + (np.array(qc)-qa)*i/20
    P = fkall(q); tcp,_ = r.tcp(q); v = valid(q)
    if i%2==0 or not v[0]: print(i, "valid", v, "minz", round(P[:,2].min(),3), "tcp", np.round(tcp,3), "l7", np.round(P[4],3), "l6", np.round(P[3],3), "l4", np.round(P[1],3))
    bad |= not v[0]
print("sweep ok" if not bad else "SWEEP BAD")
# cartesian path check
seed = list(qc); path=[]
for z in np.arange(1.08, 0.97, -0.02): path.append((-0.19,0.0,z))
path.append((-0.19,0.0,0.972))
for y in np.arange(-0.02,-0.245,-0.02): path.append((-0.19,y,0.972))
j6=[]
for p in path:
    q = r.ik_tcp(p, Q, seed, avoid=True)
    if q is None: print("IK FAIL", p); break
    if np.abs(np.array(q)-np.array(seed)).max()>1.0: print("JUMP", p); break
    seed=q; j6.append(q[5])
    print(np.round(p,3), np.round(q,3), valid(q)[0])
np.save("qc.npy", qc)
EOF
timeout 600 python3 plan_push2.py 2>&1 | tail -40

# openrua op 84
cat > chk.py <<'EOF'
from robot import *
from scene import validator
r = Robot("chk"); valid = validator(r)
Q = np.load("Qpush.npy")
seed = [0.968,1.12,-0.866,-2.431,-2.116,2.188,-1.843]
for y in [-0.21,-0.22,-0.23,-0.24]:
    q = r.ik_tcp((-0.19,y,0.972), Q, seed, avoid=False)
    print(y, None if q is None else (np.round(q,3), valid(q)))
    if q is not None: seed = q
EOF
timeout 120 python3 chk.py 2>&1 | tail -5

# openrua op 85
cat > push2.py <<'EOF'
import sys
from robot import *
r = Robot("push2")
Q = np.load("Qpush.npy"); qc = list(np.load("qc.npy"))
def rep(tag, code, err):
    t = r.tcp()[0]; w = r.wrench()[0]
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(t,4), "q", np.round(r.arm_q(),3), "wrench", np.round(w,2), flush=True)
    return t, w
for i in range(4):
    code, err = r.move_joints([qc], 12.0 if i==0 else 4.0)
    rep(f"reconf{i}", code, err)
    if err < 0.01: break
else:
    print("reconf did not converge"); sys.exit(1)
rep("down", *r.move_tcp_line([-0.19, 0.0, 0.972], Q, 3.0, avoid=True))
t, f0 = rep("pre", 0, 0.0)
y = 0.0
while y > -0.24:
    y = max(y - 0.02, -0.24)
    try:
        code, err = r.move_tcp_line([-0.19, y, 0.972], Q, 1.0)
    except RuntimeError as e:
        print("stop:", e); break
    t, w = rep(f"push y={y:.2f}", code, err)
    df = w - f0
    print("   dF", np.round(df,2), flush=True)
    if np.linalg.norm(df) > 25 or abs(t[1]-y) > 0.01:
        print("   contact/hard stop -> done pushing", flush=True); break
EOF
timeout 1200 python3 -u push2.py 2>&1 | tail -40

# openrua op 86
cat > backoff.py <<'EOF'
from robot import *
r = Robot("backoff"); Q = np.load("Qpush.npy")
code, err = r.move_tcp_line([-0.19, -0.20, 0.972], Q, 1.5)
print("backoff", code, round(err,4), "tcp", np.round(r.tcp()[0],4), "wrench", np.round(r.wrench()[0],2))
EOF
timeout 300 python3 backoff.py 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py agentview 2>&1 | tail -1; timeout 120 python3 cloud.py agentview 2>&1 | tail -1

# openrua op 87
python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]
ok=np.isfinite(z)
# anything in front of the cabinet face region (x in [-0.22,0.0], y > -0.225, z 0.905..1.0), excluding the robot (x<-0.16 & y>-0.20 region hand) 
m=ok&(x>-0.15)&(x<0.0)&(y>-0.235)&(y<-0.11)&(z>0.905)&(z<1.0)
print("points in former drawer volume (x -0.15..0, y -0.235..-0.11, z .905..1.0):", m.sum())
if m.sum(): print("  y range", round(y[m].min(),3), round(y[m].max(),3), "z", round(z[m].min(),3), round(z[m].max(),3))
# bottom drawer front face: points x in [-0.05,0.0], z 0.93..0.98
m=ok&(x>-0.06)&(x<0.0)&(z>0.93)&(z<0.98)&(y>-0.30)&(y<-0.05)
print("front face y max at x[-0.06,0]:", round(y[m].max(),3) if m.sum() else None, "n", m.sum())
# handle bar of bottom drawer
m=ok&(x>-0.14)&(x<-0.06)&(z>0.94)&(z<0.96)&(y>-0.30)&(y<-0.05)
print("bottom handle bar y range:", (round(y[m].min(),3), round(y[m].max(),3)) if m.sum() else None)
# bowl anywhere visible? black bowl rim height 0.95-0.98 on table outside cabinet
m=ok&(z>0.94)&(z<0.99)&(y>-0.20)&(x>-0.5)&(x<0.3)&~((x>-0.35)&(x<-0.05)&(y>-0.30)&(y<-0.05))  # exclude hand region
print("other objects at rim height:", m.sum(), (np.round([x[m].min(),x[m].max(),y[m].min(),y[m].max()],3) if m.sum() else ""))
EOF

# openrua op 88
cat > retreat.py <<'EOF'
from robot import *
r = Robot("retreat"); Q = np.load("Qpush.npy"); qa = list(np.load("qa.npy"))
code, err = r.move_tcp_line([-0.19, 0.0, 0.972], Q, 3.0); print("back", code, round(err,4), np.round(r.tcp()[0],3), flush=True)
code, err = r.move_tcp_line([-0.19, 0.0, 1.10], Q, 2.0, avoid=True); print("up", code, round(err,4), np.round(r.tcp()[0],3), flush=True)
for i in range(4):
    code, err = r.move_joints([qa], 12.0 if i==0 else 4.0)
    print("home", i, code, round(err,4), np.round(r.arm_q(),3), "tcp", np.round(r.tcp()[0],3), "wrench", np.round(r.wrench()[0],2), flush=True)
    if err < 0.01: break
EOF
timeout 900 python3 -u retreat.py 2>&1 | tail -6

# openrua op 89
timeout 60 python3 tools/perception/cam_snap.py frontview 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py birdview 2>&1 | tail -1; timeout 120 python3 cloud.py frontview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load("frontview_cloud.npz"); pw=d["pw"]; x,y,z=pw[...,0],pw[...,1],pw[...,2]; ok=np.isfinite(z)
m=ok&(x>-0.22)&(x<0.0)&(y>-0.30)&(y<0.0)&(z>0.905)&(z<0.995)
print("bottom-drawer region: y max", round(y[m].max(),3), "n", m.sum())
for lo in np.arange(0.91,0.995,0.01):
    mm=m&(z>=lo)&(z<lo+0.01)
    if mm.sum()>3: print(f"  z {lo:.2f} n={mm.sum():4d} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
EOF
