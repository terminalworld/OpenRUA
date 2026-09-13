#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; timeout 30 ros2 topic list; echo ---ACTIONS; timeout 30 ros2 action list; echo ---SERVICES; timeout 30 ros2 service list | grep -v -E "describe_parameters|get_parameter|list_parameters|set_parameters|get_type_description"; echo ---NODES; timeout 30 ros2 node list

# openrua op 4
cd /workspace; mkdir -p snaps; for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png & done; wait; ls -la snaps; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -120

# openrua op 6
cd /workspace; cat > tf_dump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),tr in sorted(seen.items()):
    print(f"{p:>28} -> {c:<32} t=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f}) q=({tr.rotation.x:.4f},{tr.rotation.y:.4f},{tr.rotation.z:.4f},{tr.rotation.w:.4f})")
EOF
timeout 30 python3 tf_dump.py

# openrua op 7
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/birdview_depth.png; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 8
cd /workspace; cat > bird_analyze.py <<'EOF'
import numpy as np, cv2
d = np.load("snaps/birdview_depth.npy")
img = cv2.imread("snaps/birdview.png")
fx=fy=579.4112549695428; cx=320; cy=240
# camera at (-0.2,0,3.0), R: cam x->world y, cam y->world x, cam z->world -z
def w(u,v):
    z = d[v,u]
    X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
print("depth range", np.nanmin(d), np.nanmax(d))
# height map
H = 3.0 - d
# table height: mode of a big region
print("table z sample", H[400,150], H[450,600], H[200,200])
# print heights along row 275 (white mug) and col
for name,(u,v) in {"white_mug":(232,275),"yellow_mug":(318,305),"mw_body_center":(425,295),"door_line":(365,240),"table":(200,400)}.items():
    print(name, (u,v), w(u,v), "h=",H[v,u])
# find objects above table: threshold
tab = np.median(H[350:470, 100:540])
print("table median z", tab)
mask = (H > tab+0.02)
mask[:200,:]=False  # ignore robot area roughly
ys,xs = np.where(mask)
print("above-table pixel bbox", xs.min(), xs.max(), ys.min(), ys.max())
# connected components
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    x,y,ww,hh,a = stats[i]
    if a<30: continue
    sub = H[y:y+hh, x:x+ww]; m = lab[y:y+hh, x:x+ww]==i
    print(f"comp {i}: bbox u[{x},{x+ww}] v[{y},{y+hh}] area={a} maxh={sub[m].max():.3f} centroid={cents[i]}")
    # world extents
    c = w(int(cents[i][0]), int(cents[i][1]))
    print("   world at centroid", c, " corners:", w(x,y)[:2], w(x+ww-1,y+hh-1)[:2])
EOF
python3 bird_analyze.py

# openrua op 9
cd /workspace; cat > bird2.py <<'EOF'
import numpy as np
d = np.load("snaps/birdview_depth.npy"); H = 3.0-d
fx=579.4112549695428
def w(u,v):
    z=d[v,u]; return np.array([-0.2+(v-240)*z/fx, (u-320)*z/fx, 3.0-z])
np.set_printoptions(precision=3, suppress=True, linewidth=200)
# microwave: row profile at v=300 across u 370..470
print("row v=300 heights u=370..470:"); print(np.round(H[300,370:470],3))
print("col u=430 heights v=220..360:"); print(np.round(H[220:360,430],3))
# door line: heights along v=236..246 at u=360
print("door col u=360 v=230..250:", np.round(H[230:250,360],3))
print("door row v=240 u=330..400:", np.round(H[240,330:400],3))
print("door row v=239 u=330..400:", np.round(H[239,330:400],3))
print("door row v=241 u=330..400:", np.round(H[241,330:400],3))
# white mug detail: heights along row 274 u 215..265
print("white mug row v=274:", np.round(H[274,215:265],3))
print("white mug col u=235:", np.round(H[255:295,235],3))
EOF
python3 bird2.py

# openrua op 10
cd /workspace; python3 -c "
import numpy as np
d=np.load('snaps/birdview_depth.npy'); H=3.0-d
print('     '+' '.join(f'{u:4d}' for u in range(326,470,4)))
for v in range(222,362,3):
    print(f'{v:4d} '+' '.join(f'{H[v,u]:4.2f}' if H[v,u]>0.905 else '   .' for u in range(326,470,4)))
"

# openrua op 11
cd /workspace; for c in agentview frontview; do timeout 60 python3 tools/perception/cam_snap.py /$c/depth/image_raw snaps/${c}_depth.png; timeout 20 ros2 topic echo /$c/color/camera_info --once | grep -A3 "^k:" | tail -1; done

# openrua op 12
cd /workspace; timeout 20 ros2 topic echo /agentview/color/camera_info --once | grep -A9 "^k:"; timeout 20 ros2 topic echo /frontview/color/camera_info --once | grep -A9 "^k:" | head -2

# openrua op 13
cd /workspace; cat > cloud.py <<'EOF'
import numpy as np, cv2, sys
fx=579.4112549695428; cx=320; cy=240
CAMS = {
 "agentview": ((0.6586,0.0,1.6104),(0.6380,0.6380,-0.3048,-0.3048)),
 "frontview": ((1.0,0.0,1.48),(0.5608,0.5608,-0.4306,-0.4306)),
 "birdview": ((-0.2,0.0,3.0),(0.7071,0.7071,0.0,0.0)),
 "sideview": ((-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069)),
}
def R_of(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(cam):
    d=np.load(f"snaps/{cam}_depth.npy")
    t,q=CAMS[cam]; R=R_of(q)
    v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
    X=(u-cx)*d/fx; Y=(v-cy)*d/fy if False else (v-cy)*d/fx
    P=np.stack([X,Y,d],-1).reshape(-1,3)@R.T+np.array(t)
    return P.reshape(d.shape+(3,))
if __name__=="__main__":
    cam=sys.argv[1]
    P=cloud(cam)
    np.save(f"snaps/{cam}_world.npy",P)
    # sanity: table height
    print("table z candidates:", np.percentile(P[...,2][np.isfinite(P[...,2])], [5,25,50]))
EOF
python3 cloud.py agentview; python3 cloud.py frontview; python3 cloud.py birdview

# openrua op 14
cd /workspace; cat > occ.py <<'EOF'
import numpy as np, sys
cam=sys.argv[1]; zlo=float(sys.argv[2]); zhi=float(sys.argv[3])
P=np.load(f"snaps/{cam}_world.npy").reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,2]>zlo)&(P[:,2]<zhi)
Q=P[m]
xs=np.arange(-0.45,0.30,0.02); ys=np.arange(-0.45,0.45,0.02)
print("occupancy (rows x from -0.45 up, cols y from -0.45 → +0.45), step 2cm; '#'=points")
print("      "+"".join(f"{y:+.2f}"[2] if i%5==0 else " " for i,y in enumerate(ys)))
print("      "+"".join(f"{abs(y)*100:03.0f}"[1] if i%5==0 else " " for i,y in enumerate(ys)))
for x in xs:
    row=""
    for y in ys:
        n=((Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,1]>=y)&(Q[:,1]<y+0.02)).sum()
        row+= "#" if n>=3 else ("." if n>0 else " ")
    print(f"{x:+.2f} {row}")
EOF
echo "=== agentview door/microwave layer z 0.93..1.12"; python3 occ.py agentview 0.93 1.12

# openrua op 15
cd /workspace; python3 -c "
import numpy as np
P=np.load('snaps/agentview_world.npy').reshape(-1,3)
def box(xl,xh,yl,yh,zl=0.85,zh=1.5):
    m=np.isfinite(P).all(1)&(P[:,0]>xl)&(P[:,0]<xh)&(P[:,1]>yl)&(P[:,1]<yh)&(P[:,2]>zl)&(P[:,2]<zh); return P[m]
Q=box(-0.2,0.0,-0.4,-0.2,0.905)
print('white mug pts',len(Q),'z range',Q[:,2].min(),Q[:,2].max(),'x',Q[:,0].min(),Q[:,0].max(),'y',Q[:,1].min(),Q[:,1].max())
import collections
h=np.histogram(Q[:,2],bins=np.arange(0.9,1.15,0.01)); print(list(zip(np.round(h[1][:-1],2),h[0])))
Q=box(-0.1,0.15,-0.12,0.1,0.905)
print('yellow mug pts',len(Q),'z range',Q[:,2].min(),Q[:,2].max(),'x',Q[:,0].min(),Q[:,0].max(),'y',Q[:,1].min(),Q[:,1].max())
# microwave cavity: look at points at y between 0.26 and 0.47, x -0.2..0.17
Q=box(-0.25,0.2,0.2,0.5,0.905)
print('microwave pts z range',Q[:,2].min(),Q[:,2].max())
h=np.histogram(Q[:,2],bins=np.arange(0.9,1.15,0.01)); print(list(zip(np.round(h[1][:-1],2),h[0])))
"


# openrua op 16
cd /workspace; python3 -c "
import numpy as np
np.set_printoptions(precision=3,suppress=True,linewidth=200)
P=np.load('snaps/agentview_world.npy').reshape(-1,3)
m=np.isfinite(P).all(1)
def box(xl,xh,yl,yh,zl,zh):
    s=m&(P[:,0]>xl)&(P[:,0]<xh)&(P[:,1]>yl)&(P[:,1]<yh)&(P[:,2]>zl)&(P[:,2]<zh); return P[s]
# inside footprint, below top
Q=box(-0.25,0.2,0.2,0.5,0.905,1.095)
print('interior-ish pts',len(Q))
# floor candidates: z<0.97
F=Q[Q[:,2]<0.97]; print('floor z pct',np.percentile(F[:,2],[5,50,95]),'x range',F[:,0].min(),F[:,0].max(),'y range',F[:,1].min(),F[:,1].max())
# for y slices, list x-range and z-range
for yl in np.arange(0.2,0.48,0.02):
    S=Q[(Q[:,1]>=yl)&(Q[:,1]<yl+0.02)]
    if len(S): print(f'y[{yl:.2f}] n={len(S)} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] z[{S[:,2].min():.3f},{S[:,2].max():.3f}]')
# back wall? points deepest in y
B=box(-0.25,0.2,0.2,0.6,0.905,1.095); print('max y seen', B[:,1].max())
# top surface
T=box(-0.25,0.2,0.2,0.6,1.095,1.2); print('top: x',T[:,0].min(),T[:,0].max(),'y',T[:,1].min(),T[:,1].max(),'z',T[:,2].mean())
"


# openrua op 17
cd /workspace; ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show moveit_msgs/srv/GetPositionIK | grep -v "^#" | head -40

# openrua op 18
mkdir -p "$(dirname /workspace/rlib.py)"
cat > /workspace/rlib.py <<'OPENRUA_EOF'
"""Small helper layer over this machine's ROS graph (see machine.yaml).

World <-> base: panda_link0 sits at world (-0.66, 0, 0.912), no rotation.
IK/FK talk to MoveIt in the base frame with an EMPTY frame_id.
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    # returns (x, y, z, w)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_axes(z_axis, y_axis):
    """Hand rotation with given approach (hand z) and finger-closing (hand y)
    directions, both in world; x completes the right-handed frame."""
    z = np.asarray(z_axis, float); z /= np.linalg.norm(z)
    y = np.asarray(y_axis, float); y -= z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


class Robot:
    def __init__(self, name="rlib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(zip(m.name, m.position)), 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    # ---------- sensing ----------
    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js.clear()
        while not all(j in self._js for j in ARM):
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.js()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def fk(self, q=None, link="panda_hand"):
        """Pose of link in WORLD: (pos[3], R[3x3])."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = quat_to_R((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        return pos, R

    # ---------- planning ----------
    def ik(self, pos_world, R, seed=None, tries=3, timeout=5.0):
        """Joint solution for HAND frame at pos_world/R. Returns list or None."""
        if seed is None:
            seed = self.arm_q()
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = np.asarray(pos_world, float) - BASE_IN_WORLD
        q = R_to_quat(R)
        pose = req.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, p)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        for _ in range(tries):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def ik_tcp(self, tcp_world, R, **kw):
        """IK with the target given for the fingertip point (TCP)."""
        hand = np.asarray(tcp_world, float) - TCP * R[:, 2]
        return self.ik(hand, R, **kw)

    # ---------- acting ----------
    def move(self, waypoints, seconds, verify=True):
        """waypoints: list of joint lists (or one), spaced evenly in time."""
        if not isinstance(waypoints[0], (list, tuple, np.ndarray)):
            waypoints = [waypoints]
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = None
        if verify:
            q = np.array(self.arm_q())
            err = float(np.abs(q - np.array(waypoints[-1], float)).max())
        return code, err

    def gripper(self, width, wait=120):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=wait)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, lin, ang=(0, 0, 0), ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def line_ik(self, p0, p1, R, n, seed=None):
        """IK along a straight world-space line for the HAND frame; returns
        list of joint solutions or raises on the first failure."""
        seed = list(seed if seed is not None else self.arm_q())
        out = []
        for i in range(1, n + 1):
            p = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * i / n
            q = self.ik(p, R, seed=seed)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            # reject branch jumps
            if np.abs(np.array(q) - np.array(seed)).max() > 1.0:
                raise RuntimeError(f"IK branch jump at waypoint {i}/{n}: {np.round(np.array(q)-np.array(seed),2)}")
            out.append(q); seed = q
        return out
OPENRUA_EOF

# openrua op 19
cd /workspace; timeout 120 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('probe')
q=r.arm_q(); print('q',np.round(q,4)); print('fingers',r.fingers())
pos,R=r.fk(q); print('hand world pos',pos); print('R',R)
print('tcp world', pos+TCP*R[:,2])
# test IK roundtrip
sol=r.ik(pos,R,seed=q); print('ik roundtrip', None if sol is None else np.round(sol,4))
"

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rlib.py", "old_string": "BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])", "new_string": "# Empirically (FK probe vs. camera TF) the planner\'s model frame IS the\\n# world frame on this machine: FK of panda_hand at the start pose returns\\n# (-0.203, 0, 1.27) which matches the eye-in-hand camera\'s world TF.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
cd /workspace; timeout 300 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('probe')
q=r.arm_q(); pos,R=r.fk(q); print('hand world',pos, 'z-axis',R[:,2],'x-axis',R[:,0])
for yh in ([1,0,0],[-1,0,0]):
    Rt=R_from_axes([0,1,0], yh)
    for p in ([-0.03,-0.05,1.02],[-0.03,-0.1,1.05]):
        sol=r.ik(p,Rt,seed=q)
        print('yh',yh,'p',p,'->',None if sol is None else np.round(sol,3))
"

# openrua op 22
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('step1')
q=r.arm_q()
Rt=R_from_axes([0,0.94,-0.34],[-1,0,0])
p=[-0.03,-0.18,1.13]
sol=r.ik(p,Rt,seed=q); print('sol',None if sol is None else np.round(sol,3))
code,err=r.move(sol,4.0); print('move code',code,'max err',err)
pos,R=r.fk(); print('hand now',pos,'z',R[:,2])
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent1.png

# openrua op 23
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('step1b')
sol=[-0.166,0.811,-0.355,-1.489,1.225,1.349,1.495]
print('q before',np.round(r.arm_q(),3))
code,err=r.move(sol,3.0); print('move code',code,'max err',err)
print('q after',np.round(r.arm_q(),3))
pos,R=r.fk(); print('hand now',pos,'z',R[:,2])
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 24
cd /workspace; cat > cloudtf.py <<'EOF'
"""Snapshot a camera's depth+color+TF and save a world-frame point cloud.
Usage: python3 cloudtf.py <camera> [tag]"""
import sys, time, numpy as np, cv2, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from rlib import quat_to_R
cam=sys.argv[1]; tag=sys.argv[2] if len(sys.argv)>2 else cam
rclpy.init(); node=rclpy.create_node("cloudtf")
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
node.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
frame=f"{cam}_optical_frame"
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id==frame and t.header.frame_id=="world": got["tf"]=t.transform
node.create_subscription(TFMessage,"/tf",tfcb,50)
node.create_subscription(TFMessage,"/tf_static",tfcb,QoSProfile(depth=50,durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0=time.time()
while not all(k in got for k in "dci") or "tf" not in got:
    rclpy.spin_once(node,timeout_sec=0.2)
    if time.time()-t0>30: raise SystemExit(f"timeout; have {list(got)}")
d=CvBridge().imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32)
c=CvBridge().imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
tr=got["tf"]; R=quat_to_R((tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w))
t=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@R.T+t
np.save(f"snaps/{tag}_world.npy",P.reshape(d.shape+(3,))); cv2.imwrite(f"snaps/{tag}.png",c)
print("cam at",t,"optical z axis",R[:,2],"saved snaps/%s"%tag)
EOF
timeout 60 python3 cloudtf.py robot0_eye_in_hand eih1 && python3 -c "
import numpy as np
np.set_printoptions(precision=3,suppress=True,linewidth=200)
P=np.load('snaps/eih1_world.npy').reshape(-1,3); m=np.isfinite(P).all(1)
def box(xl,xh,yl,yh,zl,zh):
    s=m&(P[:,0]>xl)&(P[:,0]<xh)&(P[:,1]>yl)&(P[:,1]<yh)&(P[:,2]>zl)&(P[:,2]<zh); return P[s]
T=box(-0.5,0.5,-0.5,0.8,0.895,0.905); print('table pts',len(T))
Q=box(-0.3,0.25,0.15,0.6,0.905,1.2)
print('microwave-region pts',len(Q))
for yl in np.arange(0.16,0.5,0.02):
    S=Q[(Q[:,1]>=yl)&(Q[:,1]<yl+0.02)]
    if len(S)>20: print(f'y[{yl:.2f}] n={len(S):5d} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] z[{S[:,2].min():.3f},{S[:,2].max():.3f}]  zhist',np.histogram(S[:,2],bins=np.arange(0.9,1.14,0.02))[0])
"


# openrua op 25
cd /workspace; python3 -c "
import numpy as np
P=np.load('snaps/eih1_world.npy').reshape(-1,3); m=np.isfinite(P).all(1)
S=P[m&(P[:,1]>0.20)&(P[:,1]<0.255)&(P[:,2]>0.91)&(P[:,2]<1.12)]
xs=np.arange(-0.24,0.18,0.01); zs=np.arange(1.11,0.91,-0.01)
print('front face occupancy (y 0.20..0.255): rows z top→bottom, cols x -0.24→0.18 step 1cm')
print('     '+''.join('|' if i%5==0 else ' ' for i in range(len(xs))))
for z in zs:
    row=''
    for x in xs:
        n=((S[:,0]>=x)&(S[:,0]<x+0.01)&(S[:,2]>=z-0.01)&(S[:,2]<z)).sum()
        row+='#' if n>=3 else '.'
    print(f'{z:.2f} {row}')
print('x ticks: ',[round(x,2) for x in xs[::5]])
"


# openrua op 26
cd /workspace; python3 -c "
import numpy as np
P=np.load('snaps/eih1_world.npy').reshape(-1,3); m=np.isfinite(P).all(1)
for (ylo,yhi) in [(0.235,0.25),(0.25,0.27),(0.27,0.30)]:
    S=P[m&(P[:,1]>ylo)&(P[:,1]<yhi)&(P[:,2]>0.91)&(P[:,2]<1.12)]
    xs=np.arange(-0.24,0.18,0.01); zs=np.arange(1.11,0.91,-0.01)
    print(f'--- y {ylo}..{yhi}: rows z top→bottom, cols x -0.24→0.18 step 1cm (ticks at -0.24,-0.19,-0.14,-0.09,-0.04,0.01,0.06,0.11,0.16)')
    for z in zs:
        row=''.join('#' if ((S[:,0]>=x)&(S[:,0]<x+0.01)&(S[:,2]>=z-0.01)&(S[:,2]<z)).sum()>=3 else '.' for x in xs)
        print(f'{z:.2f} {row}')
"


# openrua op 27
cd /workspace; timeout 60 python3 cloudtf.py frontview front1 && timeout 60 python3 cloudtf.py agentview agent1 && timeout 60 python3 cloudtf.py sideview side1 && python3 -c "
import numpy as np
np.set_printoptions(precision=3,suppress=True,linewidth=200)
for tag in ['front1','agent1','side1']:
    P=np.load(f'snaps/{tag}_world.npy').reshape(-1,3); m=np.isfinite(P).all(1)
    Q=P[m&(P[:,0]>-0.2)&(P[:,0]<0.02)&(P[:,1]>-0.42)&(P[:,1]<-0.18)&(P[:,2]>0.903)&(P[:,2]<1.2)]
    print(tag,'white mug pts',len(Q),'z',Q[:,2].min(),Q[:,2].max())
    for zl in np.arange(0.90,1.04,0.01):
        S=Q[(Q[:,2]>=zl)&(Q[:,2]<zl+0.01)]
        if len(S)>5: print(f'  z[{zl:.2f}] n={len(S):4d} x[{S[:,0].min():.3f},{S[:,0].max():.3f}] y[{S[:,1].min():.3f},{S[:,1].max():.3f}]')
"


# openrua op 28
cd /workspace; timeout 300 python3 -c "
from rlib import *
r=Robot('grip')
print('before',r.fingers())
print(r.gripper(0.04))
import time
for i in range(3):
    r.spin(0.5); print('fingers',r.fingers())
"

# openrua op 29
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('look2')
q=r.arm_q()
Rt=R_from_axes([0,0.93,-0.37],[-1,0,0])
p=[-0.085,-0.50,1.06]
sol=r.ik(p,Rt,seed=q); print('sol',None if sol is None else np.round(sol,3))
if sol:
    for i in range(2):
        code,err=r.move(sol,3.0); print('move code',code,'max err',err)
        if code==0: break
    pos,R=r.fk(); print('hand now',pos,'z',R[:,2])
" && timeout 60 python3 cloudtf.py robot0_eye_in_hand eih2

# openrua op 30
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('look2')
q=r.arm_q()
for yh in ([-1,0,0],[1,0,0]):
  for p in ([-0.085,-0.50,1.06],[-0.085,-0.45,1.1],[-0.12,-0.45,1.15]):
    Rt=R_from_axes([0,0.93,-0.37],yh)
    sol=r.ik(p,Rt,seed=q); print(yh,p,'sol',None if sol is None else np.round(sol,3))
"

# openrua op 31
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('probe3')
q=r.arm_q()
mug=np.array([-0.085,-0.288,0.95])
cands={'A +y':[0,1,0],'B diag':[0.5,0.866,0],'C from base':[0.89,-0.45,0],'D +x':[1,0,0],'E -y':[0,-1,0]}
for name,z in cands.items():
    z=np.array(z,float)
    for yh in ([0,0,1],[0,0,-1]):
        # fingers closing horizontally => hand y horizontal perpendicular to z: y = cross(z, up) or opposite
        yv=np.cross(z,[0,0,1])*yh[2]
        Rt=R_from_axes(z,yv)
        for tilt in (0.0,):
            hand=mug-0.10*z
            sol=r.ik(hand,Rt,seed=q)
            print(f'{name:12s} yh={np.round(yv,2)} hand={np.round(hand,3)} ->',None if sol is None else np.round(sol,2))
print('--- insertion')
for yh in ([-1,0,0],[1,0,0]):
    Rt=R_from_axes([0,1,0],yh)
    for p in ([-0.05,0.19,0.97],[-0.05,0.10,0.97],[-0.05,0.19,1.05]):
        sol=r.ik(p,Rt,seed=q); print(yh,p,'->',None if sol is None else np.round(sol,2))
"

# openrua op 32
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('look3')
q=r.arm_q()
z=np.array([0.93,0,-0.37]); 
for yh in ([0,1,0],[0,-1,0]):
  Rt=R_from_axes(z,yh)
  for p in ([-0.42,-0.29,1.05],[-0.40,-0.25,1.08]):
    sol=r.ik(p,Rt,seed=q); print(yh,p,'->',None if sol is None else np.round(sol,2))
    if sol:
        for i in range(2):
            code,err=r.move(sol,3.0); print('move',code,err)
            if code==0: break
        pos,R=r.fk(); print('hand',pos,'z',R[:,2]); raise SystemExit
" ; timeout 60 python3 cloudtf.py robot0_eye_in_hand eih3

# openrua op 33
cd /workspace; python3 -c "
import numpy as np
P=np.load('snaps/eih3_world.npy').reshape(-1,3); m=np.isfinite(P).all(1)
Q=P[m&(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.42)&(P[:,1]<-0.18)&(P[:,2]>0.902)&(P[:,2]<1.05)]
print('pts',len(Q),'z range',Q[:,2].min(),Q[:,2].max())
top=Q[Q[:,2]>1.005]; print('rim slice y range',top[:,1].min(),top[:,1].max(),'x range',top[:,0].min(),top[:,0].max())
yc=(top[:,1].min()+top[:,1].max())/2; print('yc est',yc)
for zl in np.arange(0.90,1.02,0.005):
    S=Q[(Q[:,2]>=zl)&(Q[:,2]<zl+0.005)]
    if len(S)>5:
        ymin=S[:,1].min(); ymax=S[:,1].max()
        print(f'z[{zl:.3f}] n={len(S):4d} y[{ymin:.3f},{ymax:.3f}] diam_from_-y_side={2*(yc-ymin):.3f} xmin={S[:,0].min():.3f}')
"


# openrua op 34
cd /workspace; timeout 30 ros2 service call /get_urdf std_srvs/srv/Trigger 2>/dev/null | head -c 300; echo; ros2 service type /get_urdf; timeout 60 python3 - <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("urdf")
got=[]
n.create_subscription(String,"/robot_description",lambda m:got.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
open("robot.urdf","w").write(got[0]); print("urdf chars",len(got[0]))
EOF
grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A12 'link name="panda_leftfinger"' robot.urdf | head -50

# openrua op 35
cd /workspace; python3 -c "
import numpy as np, struct
def stl_bounds(fn):
    d=open(fn,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    v=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f,'min',np.round(lo,4),'max',np.round(hi,4),'size',np.round(hi-lo,4))
"


# openrua op 36
find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head; find / -ipath "*panda*" -name "*.stl" 2>/dev/null | head

# openrua op 37
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('probe_int')
print('close gripper',r.gripper(0.0))
q=r.arm_q()
Rt=R_from_axes([0,0,-1],[1,0,0])
xc,yc=-0.084,-0.288
# above the mug: tips at 1.06 (5cm above rim)
hi=np.array([xc,yc,1.06+TCP]); lo=np.array([xc,yc,0.99+TCP])
s1=r.ik(hi,Rt,seed=q); print('s1',None if s1 is None else np.round(s1,3))
code,err=r.move(s1,4.0); print('move1',code,err)
if code!=0: code,err=r.move(s1,3.0); print('move1b',code,err)
pos,R=r.fk(); print('hand',pos,'tcp',pos+TCP*R[:,2],'z',R[:,2])
# read wrench baseline
import rclpy
from geometry_msgs.msg import WrenchStamped
w=[]; r.node.create_subscription(WrenchStamped,'/franka_robot_state_broadcaster/external_wrench',lambda m:w.append(m),10)
for _ in range(5): r.spin(0.2)
print('wrench baseline',w[-1].wrench.force if w else None)
s2=r.ik(lo,Rt,seed=s1); print('s2',np.round(s2,3))
code,err=r.move(s2,3.0); print('move2',code,err)
pos,R=r.fk(); print('hand',pos,'tcp',pos+TCP*R[:,2])
w.clear()
for _ in range(5): r.spin(0.2)
print('wrench after',w[-1].wrench.force if w else None)
print('fingers',r.fingers())
"; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_probe.png

# openrua op 38
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('home')
print('q now',np.round(r.arm_q(),3))
for cand in ([0,-0.5,0,-2.3,0,1.8,0.785],[0,-0.2,0,-2.0,0,1.8,0.785],[0,0.2,0,-1.8,0,2.0,0.785],[0,0.5,0,-1.6,0,2.1,0.785]):
    pos,R=r.fk(cand); print(cand,'-> hand',pos,'z',R[:,2],'tcp',pos+TCP*R[:,2])
"

# openrua op 39
cd /workspace; cat >> rlib.py <<'EOF'


def path_check(r, q0, q1, n=12, links=("panda_hand", "panda_link4", "panda_link6", "panda_link7")):
    """FK of several links along the straight joint-space path q0->q1.
    Returns list of (s, {link: pos}) and prints the lowest points."""
    out = []
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    for i in range(n + 1):
        s = i / n
        q = q0 + (q1 - q0) * s
        d = {}
        for L in links:
            p, R = r.fk(q, link=L)
            if L == "panda_hand":
                d["tcp"] = p + TCP * R[:, 2]
            d[L] = p
        out.append((s, d))
    return out


def print_path(out):
    for s, d in out:
        print(f"s={s:.2f} " + " ".join(f"{k}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})" for k, v in d.items()))
EOF
timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('chk')
q=r.arm_q(); home=[0,-0.5,0,-2.3,0,1.8,0.785]
print_path(path_check(r,q,home))
"

# openrua op 40
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('home')
home=[0,-0.5,0,-2.3,0,1.8,0.785]
for i in range(3):
    code,err=r.move(home,6.0); print('move',code,err)
    if code==0 and err<0.05: break
print('q',np.array(r.arm_q())); p,R=r.fk(); print('hand',p, 'z-axis',R[:,2])
"

# openrua op 41
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rlib.py", "old_string": "    def ik(self, pos_world, R, seed=None, tries=3, timeout=5.0):\\n        \\"\\"\\"Joint solution for HAND frame at pos_world/R. Returns list or None.\\"\\"\\"\\n        if seed is None:\\n            seed = self.arm_q()", "new_string": "    def ik(self, pos_world, R, seed=None, tries=8, timeout=2.0, max_jump=None):\\n        \\"\\"\\"Joint solution for HAND frame at pos_world/R. Returns list or None.\\n        max_jump: reject solutions farther than this (rad, per joint) from the\\n        seed; the seed is perturbed slightly between retries.\\"\\"\\"\\n        if seed is None:\\n            seed = self.arm_q()\\n        seed0 = np.array(seed, float)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 42
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rlib.py", "old_string": "        for _ in range(tries):\\n            fut = self.ik_cli.call_async(req)\\n            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)\\n            res = fut.result()\\n            if res is not None and res.error_code.val == 1:\\n                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                return [sol[j] for j in ARM]\\n        return None", "new_string": "        rng = np.random.default_rng(0)\\n        best = None\\n        for k in range(tries):\\n            s = seed0 if k == 0 else seed0 + rng.normal(0, 0.05 * k, len(seed0))\\n            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]\\n            fut = self.ik_cli.call_async(req)\\n            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)\\n            res = fut.result()\\n            if res is not None and res.error_code.val == 1:\\n                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                q = [sol[j] for j in ARM]\\n                jump = float(np.abs(np.array(q) - seed0).max())\\n                if max_jump is None or jump <= max_jump:\\n                    return q\\n                if best is None or jump < best[0]:\\n                    best = (jump, q)\\n        if best is not None:\\n            print(f\\"ik: only far solutions (min jump {best[0]:.2f} rad)\\")\\n        return None", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 43
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rlib.py", "old_string": "            q = self.ik(p, R, seed=seed)\\n            if q is None:\\n                raise RuntimeError(f\\"IK failed at waypoint {i}/{n}: {p}\\")\\n            # reject branch jumps\\n            if np.abs(np.array(q) - np.array(seed)).max() > 1.0:\\n                raise RuntimeError(f\\"IK branch jump at waypoint {i}/{n}: {np.round(np.array(q)-np.array(seed),2)}\\")\\n            out.append(q); seed = q", "new_string": "            q = self.ik(p, R, seed=seed, max_jump=max_jump)\\n            if q is None:\\n                raise RuntimeError(f\\"IK failed at waypoint {i}/{n}: {p}\\")\\n            out.append(q); seed = q", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 44
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rlib.py", "old_string": "    def line_ik(self, p0, p1, R, n, seed=None):", "new_string": "    def line_ik(self, p0, p1, R, n, seed=None, max_jump=0.6):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 45
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('plan')
q0=r.arm_q(); p,R=r.fk(); print('home R\n',R)
Rg=R_from_axes([0,0,-1],[1,0,0]); print('Rg\n',Rg)
for z in (1.30,1.20):
    q=r.ik([-0.084,-0.288,z],Rg,seed=q0,max_jump=1.5)
    print(z, None if q is None else np.array(q))
"

# openrua op 46
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('plan')
q0=np.array(r.arm_q())
for yax in ([1,0,0],[-1,0,0]):
    Rg=R_from_axes([0,0,-1],yax)
    q=r.ik([-0.084,-0.288,1.30],Rg,seed=q0,max_jump=2.6)
    print(yax, None if q is None else np.array(q), None if q is None else np.array(q)-q0)
"

# openrua op 47
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('pre')
q0=np.array(r.arm_q())
Rg=R_from_axes([0,0,-1],[-1,0,0])
q1=r.ik([-0.084,-0.288,1.30],Rg,seed=q0,max_jump=2.6)
print_path(path_check(r,q0,q1,n=6,links=('panda_hand','panda_link6')))
for i in range(3):
    code,err=r.move(q1,5.0); print('move',code,err)
    if code==0 and err<0.03: break
p,R=r.fk(); print('hand',p,'tcp',p+TCP*R[:,2]); print(R)
"

# openrua op 48
cd /workspace; timeout 120 python3 cloudtf.py robot0_eye_in_hand eih4 2>&1 | tail -3

# openrua op 49
cd /workspace; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/eih4_world.npy')
for v in range(250,340,8):
    row=W[v,270:370:8,2]; print(v, row)
# mug center pixel world xy
print(W[290,320])
"

# openrua op 50
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
from geometry_msgs.msg import WrenchStamped
r=Robot('probe')
wr={}
r.node.create_subscription(WrenchStamped,'/franka_robot_state_broadcaster/external_wrench',lambda m: wr.update(f=(m.wrench.force.x,m.wrench.force.y,m.wrench.force.z)),10)
def wrench():
    wr.clear()
    for _ in range(20):
        r.spin(0.1)
        if 'f' in wr: return np.array(wr['f'])
    return None
print('fingers',r.fingers(),'wrench',wrench())
q0=r.arm_q(); p,R=r.fk()
Rg=R
# descend to tcp 1.06
wps=r.line_ik(p,[-0.084,-0.288,1.06+TCP],Rg,4,seed=q0)
code,err=r.move(wps,4.0); print('down1',code,err)
if err>0.03: code,err=r.move(wps[-1],2.0); print('resend',code,err)
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'wrench',wrench())
# now step down 1 cm at a time to tcp 0.99
for zt in (1.03,1.01,1.00,0.99):
    q=r.ik([-0.084,-0.288,zt+TCP],Rg,seed=r.arm_q(),max_jump=0.3)
    code,err=r.move(q,1.5)
    p,R=r.fk(); print(f'target tcp z {zt}: code {code} err {err:.4f} tcp {p+TCP*R[:,2]} wrench {wrench()}')
"

# openrua op 51
cd /workspace; timeout 120 python3 cloudtf.py agentview agent2 2>&1 | tail -1; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/agent2_world.npy').reshape(-1,3)
m=np.isfinite(W).all(1)&(W[:,2]>0.95)&(W[:,2]<1.02)&(W[:,1]<-0.2)&(W[:,1]>-0.4)&(np.abs(W[:,0]+0.084)<0.1)
P=W[m]; print(len(P), P.mean(0), P.min(0), P.max(0))
"

# openrua op 52
cd /workspace; for c in frontview sideview; do timeout 120 python3 cloudtf.py $c ${c}_probe 2>&1 | tail -1; done; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
for tag in ('frontview_probe','sideview_probe'):
    W=np.load(f'snaps/{tag}_world.npy').reshape(-1,3)
    m=np.isfinite(W).all(1)&(np.abs(W[:,0]+0.084)<0.15)&(np.abs(W[:,1]+0.288)<0.12)&(W[:,2]>0.9)&(W[:,2]<1.25)
    P=W[m]; print(tag,len(P))
    # histogram of z with x,y extents per bin
    for z0 in np.arange(0.90,1.25,0.01):
        s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
        if len(s): print(f'z {z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
"

# openrua op 53
cd /workspace; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True,linewidth=200)
W=np.load('snaps/eih4_world.npy').reshape(-1,3)
m=np.isfinite(W).all(1)&(W[:,1]>-0.26)&(W[:,1]<-0.19)&(np.abs(W[:,0]+0.084)<0.04)&(W[:,2]>0.905)
P=W[m]; print(len(P))
# grid: y bins 4mm, x bins 4mm -> max z
ys=np.arange(-0.26,-0.19,0.004); xs=np.arange(-0.124,-0.044,0.004)
print('x:',xs)
for y0 in ys:
    row=[]
    for x0 in xs:
        s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.004)&(P[:,0]>=x0)&(P[:,0]<x0+0.004)]
        row.append(f'{s[:,2].max():.3f}' if len(s) else '  .  ')
    print(f'y {y0:+.3f}', ' '.join(row))
"

# openrua op 54
cd /workspace; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True,linewidth=200)
W=np.load('snaps/agent2_world.npy').reshape(-1,3)
W=W[np.isfinite(W).all(1)]
# microwave region
m=(W[:,0]>-0.2)&(W[:,0]<0.2)&(W[:,1]>0.2)&(W[:,1]<0.5)&(W[:,2]>0.9)
P=W[m]
# for y slices, z hist
for y0 in np.arange(0.20,0.46,0.01):
    s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.01)]
    if len(s)==0: print(f'y {y0:.2f} -'); continue
    zs=np.sort(s[:,2])
    print(f'y {y0:.2f} n{len(s):5d} z min {zs[0]:.3f} p5 {zs[int(len(zs)*0.05)]:.3f} p50 {zs[len(zs)//2]:.3f} p95 {zs[int(len(zs)*0.95)]:.3f} max {zs[-1]:.3f}  x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}]')
print('top surface: points z>1.09')
T=P[P[:,2]>1.09]; print(len(T), 'z', T[:,2].min(), T[:,2].max(), 'y', T[:,1].min(), T[:,1].max())
"

# openrua op 55
cd /workspace; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True,linewidth=250)
W=np.load('snaps/agent2_world.npy').reshape(-1,3)
W=W[np.isfinite(W).all(1)]
xs=np.arange(-0.24,0.20,0.02); ys=np.arange(0.16,0.32,0.01)
print('maxz grid; cols x:',xs)
for y0 in ys:
    row=[]
    for x0 in xs:
        s=W[(W[:,1]>=y0)&(W[:,1]<y0+0.01)&(W[:,0]>=x0)&(W[:,0]<x0+0.02)]
        row.append(f'{s[:,2].max():.3f}' if len(s) else '  .  ')
    print(f'y {y0:.2f}', ' '.join(row))
"

# openrua op 56
mkdir -p "$(dirname /workspace/geom.py)"
cat > /workspace/geom.py <<'OPENRUA_EOF'
"""Coarse geometric model of the scene for clearance checks (world frame, metres).

Everything is boxes.  Hand/finger/mug are sampled as point sets and tested
against the static solids with an SDF (negative = penetrating).
"""
import numpy as np

# ---------- static solids (axis aligned boxes: [xmin,xmax,ymin,ymax,zmin,zmax]) ----------
TABLE_Z = 0.90
MW_TOP = 1.108
OPEN_X = (-0.15, 0.05); OPEN_Z = (0.955, 1.085)     # opening in the front face y=0.25..0.27
CAV_X = (-0.151, 0.057); CAV_Y = (0.27, 0.42); CAV_Z = (0.942, 1.087)
BODY_X = (-0.18, 0.17); BODY_Y = (0.25, 0.455)

STATIC = [
    # table
    [-2, 2, -2, 2, TABLE_Z - 0.1, TABLE_Z],
    # microwave front frame: below opening (sill front), above opening (lip), left, right
    [BODY_X[0], BODY_X[1], BODY_Y[0], CAV_Y[0], TABLE_Z, OPEN_Z[0]],
    [BODY_X[0], BODY_X[1], BODY_Y[0], CAV_Y[0], OPEN_Z[1], MW_TOP],
    [BODY_X[0], OPEN_X[0], BODY_Y[0], CAV_Y[0], TABLE_Z, MW_TOP],
    [OPEN_X[1], BODY_X[1], BODY_Y[0], CAV_Y[0], TABLE_Z, MW_TOP],
    # cavity shell: floor, ceiling, left, right, back
    [BODY_X[0], BODY_X[1], CAV_Y[0], BODY_Y[1], TABLE_Z, CAV_Z[0]],
    [BODY_X[0], BODY_X[1], CAV_Y[0], BODY_Y[1], CAV_Z[1], MW_TOP],
    [BODY_X[0], CAV_X[0], CAV_Y[0], BODY_Y[1], TABLE_Z, MW_TOP],
    [CAV_X[1], BODY_X[1], CAV_Y[0], BODY_Y[1], TABLE_Z, MW_TOP],
    [BODY_X[0], BODY_X[1], CAV_Y[1], BODY_Y[1], TABLE_Z, MW_TOP],
    # control panel block protruding at the front right
    [0.055, BODY_X[1], 0.23, BODY_Y[0], TABLE_Z, MW_TOP],
]

# door: hinge at (-0.19, 0.26), free edge at (-0.27,-0.04); modelled as an
# oriented box 0.31 long, 0.02 thick, z 0.90..1.11
DOOR_HINGE = np.array([-0.19, 0.26])
DOOR_TIP = np.array([-0.27, -0.04])


def door_boxes(tip=DOOR_TIP):
    d = tip - DOOR_HINGE
    L = np.linalg.norm(d); u = d / L; n = np.array([-u[1], u[0]])
    return [(DOOR_HINGE, u, n, L, 0.02)]


def box_sdf(p, b):
    c = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
    h = np.array([(b[1] - b[0]) / 2, (b[3] - b[2]) / 2, (b[5] - b[4]) / 2])
    q = np.abs(p - c) - h
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def door_sdf(p, tip=DOOR_TIP):
    (h0, u, n, L, t), = door_boxes(tip)
    rel = p[:, :2] - h0
    a = rel @ u; b = rel @ n
    q = np.stack([np.abs(a - L / 2) - L / 2, np.abs(b) - t / 2,
                  np.abs(p[:, 2] - (0.90 + 1.11) / 2) - 0.105], -1)
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def static_sdf(p, door_tip=DOOR_TIP, include_door=True):
    p = np.atleast_2d(p)
    d = np.min([box_sdf(p, b) for b in STATIC], axis=0)
    if include_door:
        d = np.minimum(d, door_sdf(p, door_tip))
    return d


# ---------- hand model (panda_hand frame; z toward fingertips) ----------
HAND_BODY = [-0.035, 0.035, -0.102, 0.102, -0.03, 0.063]
CAM_MOUNT = [0.035, 0.065, -0.03, 0.03, -0.03, 0.02]
WRIST = [-0.05, 0.05, -0.05, 0.05, -0.20, -0.03]
FINGER_LEN = (0.063, 0.1124)     # z range of a finger
FINGER_HALF_W = 0.010            # along hand x
FINGER_THICK = 0.014             # along hand y, outward from the inner face


def box_points(b, step=0.005):
    xs = np.arange(b[0], b[1] + 1e-9, step); ys = np.arange(b[2], b[3] + 1e-9, step)
    zs = np.arange(b[4], b[5] + 1e-9, step)
    # surface only: 6 faces
    pts = []
    for X, Y in [(xs, ys)]:
        g = np.array(np.meshgrid(X, Y, indexing="ij")).reshape(2, -1).T
        pts += [np.c_[g, np.full(len(g), b[4])], np.c_[g, np.full(len(g), b[5])]]
    g = np.array(np.meshgrid(xs, zs, indexing="ij")).reshape(2, -1).T
    pts += [np.c_[g[:, 0], np.full(len(g), b[2]), g[:, 1]], np.c_[g[:, 0], np.full(len(g), b[3]), g[:, 1]]]
    g = np.array(np.meshgrid(ys, zs, indexing="ij")).reshape(2, -1).T
    pts += [np.c_[np.full(len(g), b[0]), g], np.c_[np.full(len(g), b[1]), g]]
    return np.vstack(pts)


def hand_points(finger_pos=(0.0, 0.0), step=0.005):
    """Points of hand+fingers in the hand frame. finger_pos: inner-face
    offsets of finger1 (+y) and finger2 (-y), metres."""
    pts = [box_points(HAND_BODY, step), box_points(CAM_MOUNT, step), box_points(WRIST, step)]
    p1, p2 = finger_pos
    pts.append(box_points([-FINGER_HALF_W, FINGER_HALF_W, p1, p1 + FINGER_THICK, *FINGER_LEN], step))
    pts.append(box_points([-FINGER_HALF_W, FINGER_HALF_W, -p2 - FINGER_THICK, -p2, *FINGER_LEN], step))
    return np.vstack(pts)


# ---------- mug model (mug frame: origin at base centre, z up, handle +y) ----------
MUG_R = 0.046; MUG_RI = 0.041; MUG_H = 0.112
HANDLE = [-0.012, 0.012, 0.046, 0.082, 0.02, 0.10]


def mug_points(step=0.005):
    ang = np.arange(0, 2 * np.pi, step / MUG_R)
    zs = np.arange(0, MUG_H + 1e-9, step)
    g = np.array(np.meshgrid(ang, zs, indexing="ij")).reshape(2, -1).T
    outer = np.c_[MUG_R * np.cos(g[:, 0]), MUG_R * np.sin(g[:, 0]), g[:, 1]]
    return np.vstack([outer, box_points(HANDLE, step)])


def transform(pts, R, t):
    return pts @ R.T + np.asarray(t)


def clearance(pts, **kw):
    d = static_sdf(pts, **kw)
    i = int(np.argmin(d))
    return float(d[i]), pts[i]
OPENRUA_EOF

# openrua op 57
cd /workspace; cat > grasp_eval.py <<'EOF'
import numpy as np
from geom import *
from rlib import R_from_axes, TCP

def rot_axis(axis, ang):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K

def grasp_hand_pose(mug_base, alpha, theta, d, mug_R=np.eye(3)):
    """Hand pose for a rim pinch. mug_base: world pos of mug base centre (mug frame
    origin); mug_R: mug orientation. Pinch at rim angle alpha (mug frame), fingers
    close radially; hand tilted theta about the closing axis (positive = hand leans
    toward -radial*... see code); d = TCP depth below rim."""
    c = np.array([np.cos(alpha), np.sin(alpha), 0.0])          # radial (mug frame)
    P = np.array([0.0435 * c[0], 0.0435 * c[1], MUG_H - d])     # TCP in mug frame
    y_h = c                                                     # closing dir
    t = np.array([-np.sin(alpha), np.cos(alpha), 0.0])          # tangential
    # z_h: start at -z, rotate about y_h by theta (leans along +/- tangential)
    z_h = rot_axis(y_h, theta) @ np.array([0, 0, -1.0])
    Rh_m = R_from_axes(z_h, y_h)                                # in mug frame
    Rh = mug_R @ Rh_m
    tcp_w = mug_R @ P + mug_base
    hand_w = tcp_w - TCP * Rh[:, 2]
    return hand_w, Rh, tcp_w

def scene_points(mug_base, mug_R, hand_w, Rh, finger=0.0025):
    hp = transform(hand_points((finger, finger)), Rh, hand_w)
    mp = transform(mug_points(), mug_R, mug_base)
    return hp, mp

def finger_mug_penetration(hand_w, Rh, mug_base, mug_R, finger=0.0025):
    """Depth (m) of hand/finger points inside the mug wall/handle, excluding the
    pad contact tolerance."""
    hp = transform(hand_points((finger, finger)), Rh, hand_w)
    q = (hp - mug_base) @ mug_R          # into mug frame
    r = np.hypot(q[:, 0], q[:, 1]); z = q[:, 2]
    in_wall = (r > MUG_RI - 0.002) & (r < MUG_R + 0.002) & (z > 0) & (z < MUG_H)
    in_handle = np.all([(q[:, i] > HANDLE[2 * i]) & (q[:, i] < HANDLE[2 * i + 1]) for i in range(3)], axis=0)
    in_floor = (r < MUG_R) & (z > 0) & (z < 0.009)
    return int(in_wall.sum()), int(in_handle.sum()), int(in_floor.sum())

if __name__ == "__main__":
    import itertools
    np.set_printoptions(precision=3, suppress=True)
    # grasp on table
    base0 = np.array([-0.084, -0.288, 0.90])
    for alpha_deg, theta_deg, d in itertools.product([180, 90, 270, 135, 225], [0, 20, 30, 40], [0.010, 0.015, 0.020]):
        for sgn in (1, -1):
            hw, Rh, tcp = grasp_hand_pose(base0, np.radians(alpha_deg), sgn * np.radians(theta_deg), d)
            pen = finger_mug_penetration(hw, Rh, base0, np.eye(3))
            hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
            cl, _ = clearance(hp)
            print(f"alpha {alpha_deg:3d} theta {sgn*theta_deg:+3d} d {d:.3f}  pen wall/handle/floor {pen}  static clr {cl:+.3f}")
EOF
timeout 300 python3 grasp_eval.py 2>&1 | head -130

# openrua op 58
cd /workspace; sed -i 's/in_wall = (r > MUG_RI - 0.002) \& (r < MUG_R + 0.002)/in_wall = (r > MUG_RI + 0.0005) \& (r < MUG_R - 0.0005)/' grasp_eval.py; timeout 300 python3 grasp_eval.py 2>&1 | grep -E "alpha (180|90) " | grep -v "theta  +0.*\n" | head -60

# openrua op 59
cd /workspace; timeout 300 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from grasp_eval import *
base0=np.array([-0.084,-0.288,0.90])
for th in (0,30):
    hw,Rh,tcp=grasp_hand_pose(base0,np.pi,np.radians(th),0.015)
    hp=transform(hand_points((0.0025,0.0025)),Rh,hw)
    q=(hp-base0)
    r=np.hypot(q[:,0],q[:,1]); z=q[:,2]
    m=(r>MUG_RI+0.0005)&(r<MUG_R-0.0005)&(z>0)&(z<MUG_H)
    print('theta',th,'tcp',tcp,'n',m.sum()); print(np.c_[q[m],r[m]][:12])
"

# openrua op 60
cd /workspace; timeout 300 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from grasp_eval import *
base0=np.array([-0.084,-0.288,0.90])
def pen(th,d,alpha=np.pi):
    hw,Rh,tcp=grasp_hand_pose(base0,alpha,np.radians(th),d)
    hp=transform(hand_points((0.0025,0.0025),step=0.0025),Rh,hw)
    q=(hp-base0); r=np.hypot(q[:,0],q[:,1]); z=q[:,2]
    m=(r>MUG_RI)&(r<MUG_R)&(z>0)&(z<MUG_H)
    if not m.any(): return 0.0
    return float(np.minimum(r[m]-MUG_RI, MUG_R-r[m]).max())
for d in (0.008,0.010,0.012,0.015,0.018):
    print(f'd {d:.3f}: '+' '.join(f'th{th}:{pen(th,d)*1000:.1f}mm' for th in (0,10,20,30,40,50,60)))
"

# openrua op 61
cd /workspace; timeout 300 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from grasp_eval import *
base0=np.array([-0.084,-0.288,0.90])
def pen(th,d,alpha=np.pi):
    hw,Rh,tcp=grasp_hand_pose(base0,alpha,np.radians(th),d)
    hl=hand_points((0.0025,0.0025),step=0.0025)
    hp=transform(hl,Rh,hw)
    q=(hp-base0); r=np.hypot(q[:,0],q[:,1]); z=q[:,2]
    band=(z>0)&(z<MUG_H)
    inner=hl[:,1]>0   # finger1 at +y_h = radial outward?  check which is inside
    # inside finger = points with r<0.0435 mostly
    fin=band&(r<0.0435)&(r>MUG_RI); fout=band&(r>=0.0435)&(r<MUG_R)
    a=(r[fin]-MUG_RI).max()*1000 if fin.any() else 0; b=(MUG_R-r[fout]).max()*1000 if fout.any() else 0
    return a,b
for d in (0.010,0.015,0.020):
    print(f'd {d:.3f}: '+' '.join(f'th{th}:in{pen(th,d)[0]:.1f}/out{pen(th,d)[1]:.1f}' for th in (0,10,20,30,40,50,60)))
"

# openrua op 62
cd /workspace; timeout 300 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from grasp_eval import *
base0=np.array([-0.084,-0.288,0.90])
def pen(th,d,alpha=np.pi):
    hw,Rh,tcp=grasp_hand_pose(base0,alpha,np.radians(th),d)
    hl=hand_points((0.0025,0.0025),step=0.0025)
    hp=transform(hl,Rh,hw)
    q=(hp-base0); r=np.hypot(q[:,0],q[:,1]); z=q[:,2]
    band=(z>0)&(z<MUG_H)&(hl[:,2]>0.06)
    inner=band&(hl[:,1]<0); outer=band&(hl[:,1]>0)
    a=(r[inner].max()-MUG_RI)*1000 if inner.any() else 0; b=(MUG_R-r[outer].min())*1000 if outer.any() else 0
    return a,b
for d in (0.010,0.015,0.020):
    print(f'd {d:.3f}: '+' '.join(f'th{th}:in{pen(th,d)[0]:.1f}/out{pen(th,d)[1]:.1f}' for th in (0,10,20,30,40,50,60)))
"

# openrua op 63
cd /workspace; cat > place_scan.py <<'EOF'
import numpy as np, itertools
from grasp_eval import *
np.set_printoptions(precision=3, suppress=True)
MP = mug_points(step=0.004)
def evaluate(x_m, y_m, phi, theta, d, alpha=np.pi, drop=0.01):
    Rm = rot_axis([1, 0, 0], phi)
    # place mug so its lowest point is `drop` above the cavity floor
    mp0 = MP @ Rm.T
    zmin = mp0[:, 2].min()
    base = np.array([x_m, y_m, CAV_Z[0] + drop - zmin])
    mp = mp0 + base
    hw, Rh, tcp = grasp_hand_pose(base, alpha, theta, d, mug_R=Rm)
    hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
    cm, pm = clearance(mp, include_door=False)
    ch, ph = clearance(hp, include_door=False)
    com = base + Rm @ np.array([0, 0, 0.056])
    return cm, ch, com, base, hw, Rh
if __name__ == "__main__":
    res = []
    for y_m, phi_d, th_d, d in itertools.product(np.arange(0.25, 0.36, 0.01), [0, 10, 15, 20, 25, 30], [0, 10, 20], [0.010, 0.015]):
        for sgn in (1, -1):
            cm, ch, com, base, hw, Rh = evaluate(-0.05, y_m, sgn * np.radians(phi_d), sgn * np.radians(th_d), d)
            res.append((min(cm, ch), cm, ch, y_m, sgn * phi_d, sgn * th_d, d, com[1], base[2]))
    res.sort(key=lambda r: -r[0])
    for r in res[:25]:
        print("clr %.3f (mug %.3f hand %.3f) y_m %.2f phi %+d theta %+d d %.3f  com_y %.3f base_z %.3f" % r)
EOF
timeout 900 python3 place_scan.py

# openrua op 64
cd /workspace; cat > lean_scan.py <<'EOF'
import numpy as np, itertools
from grasp_eval import *
np.set_printoptions(precision=3, suppress=True)
MP = mug_points(step=0.004)
def lean_pose(beta, y_p, z_p, x_m=-0.05):
    """Mug leaning toward -y by beta (top toward robot); its base's -y edge at (y_p, z_p)."""
    Rm = rot_axis([1, 0, 0], beta)
    edge_local = np.array([0, -MUG_R, 0.0])          # base -y edge in mug frame
    base = np.array([x_m, y_p, z_p]) - Rm @ edge_local
    return Rm, base
def evaluate(beta, y_p, z_p, theta, d, alpha=np.pi, x_m=-0.05):
    Rm, base = lean_pose(beta, y_p, z_p, x_m)
    mp = MP @ Rm.T + base
    hw, Rh, tcp = grasp_hand_pose(base, alpha, theta, d, mug_R=Rm)
    hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
    cm, pm = clearance(mp, include_door=False)
    ch, ph = clearance(hp, include_door=False)
    com = base + Rm @ np.array([0, 0, 0.056])
    return cm, ch, pm, ph, com, base, hw, Rh
if __name__ == "__main__":
    res = []
    for beta_d, y_p, z_p, th_d, d in itertools.product([20, 25, 30, 35], np.arange(0.23, 0.31, 0.01), [0.957, 0.945], [0, 10, 20], [0.010, 0.015]):
        cm, ch, pm, ph, com, base, hw, Rh = evaluate(np.radians(beta_d), y_p, z_p, np.radians(th_d), d)
        res.append((min(cm, ch), cm, ch, beta_d, y_p, z_p, th_d, d, com[1] - y_p, pm, ph))
    res.sort(key=lambda r: -r[0])
    for r in res[:30]:
        print("clr %.3f (mug %.3f hand %.3f) beta %d y_p %.2f z_p %.3f theta %d d %.3f  com-yp %+.3f mugpt %s handpt %s" % r)
EOF
timeout 900 python3 lean_scan.py

# openrua op 65
cd /workspace; cat > lie_scan.py <<'EOF'
import numpy as np, itertools
from grasp_eval import *
np.set_printoptions(precision=3, suppress=True)
MP = mug_points(step=0.004)
def lie_eval(y_m, x_m, psi, beta, theta, d, drop=0.008, alpha=np.pi):
    # yaw psi about z (handle direction), then roll beta about world x (top toward -y)
    Rm = rot_axis([1, 0, 0], beta) @ rot_axis([0, 0, 1], psi)
    mp0 = MP @ Rm.T
    # centre of the mug body (mid-height) placed at (x_m, y_m); lowest point at floor+drop
    c0 = Rm @ np.array([0, 0, MUG_H / 2])
    base = np.array([x_m - c0[0], y_m - c0[1], 0.0])
    base[2] = CAV_Z[0] + drop - (mp0[:, 2].min())
    mp = mp0 + base
    hw, Rh, tcp = grasp_hand_pose(base, alpha, theta, d, mug_R=Rm)
    hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
    cm, pm = clearance(mp, include_door=False)
    ch, ph = clearance(hp, include_door=False)
    return cm, ch, pm, ph, hw, Rh, tcp, mp
if __name__ == "__main__":
    for psi_d in (-90, 90):
        for y_m in np.arange(0.30, 0.37, 0.01):
            for beta_d in (90, 80, 70):
                cm, ch, pm, ph, hw, Rh, tcp, mp = lie_eval(y_m, -0.05, np.radians(psi_d), np.radians(beta_d), 0.0, 0.015)
                print(f"psi {psi_d} y_m {y_m:.2f} beta {beta_d}: mug clr {cm:+.3f} at {pm}  hand clr {ch:+.3f} at {ph}  mug y-span [{mp[:,1].min():.3f},{mp[:,1].max():.3f}] z-span [{mp[:,2].min():.3f},{mp[:,2].max():.3f}] tcp {tcp}")
EOF
timeout 600 python3 lie_scan.py

# openrua op 66
cd /workspace; timeout 600 python3 -c "
import numpy as np
from lie_scan import *
for psi_d in (-90, 90):
    for y_m in np.arange(0.31, 0.37, 0.01):
        for beta_d in (90, 80):
            cm, ch, pm, ph, hw, Rh, tcp, mp = lie_eval(y_m, -0.05, np.radians(psi_d), np.radians(beta_d), 0.0, 0.015, alpha=np.pi-np.radians(psi_d))
            print(f'psi {psi_d} y_m {y_m:.2f} beta {beta_d}: mug clr {cm:+.3f} at {pm}  hand clr {ch:+.3f} at {ph}  mug y[{mp[:,1].min():.3f},{mp[:,1].max():.3f}] z[{mp[:,2].min():.3f},{mp[:,2].max():.3f}] tcp {tcp} hand {hw} zh {Rh[:,2]}')
"

# openrua op 67
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('lift')
q0=r.arm_q(); p,R=r.fk()
wps=r.line_ik(p,[-0.084,-0.288,1.10+TCP],R,4,seed=q0)
code,err=r.move(wps,4.0); print('up',code,err)
if err>0.03: print('resend',r.move(wps[-1],2.0))
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'fingers',r.fingers())
" && for c in frontview agentview; do timeout 120 python3 cloudtf.py $c ${c}_fing 2>&1 | tail -1; done; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
for tag in ('frontview_fing','agentview_fing'):
    W=np.load(f'snaps/{tag}_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
    m=(np.abs(W[:,0]+0.084)<0.06)&(np.abs(W[:,1]+0.288)<0.06)&(W[:,2]>1.02)&(W[:,2]<1.20)
    P=W[m]; print(tag,len(P),'min z',P[:,2].min())
    for z0 in np.arange(1.06,1.20,0.005):
        s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.005)]
        if len(s): print(f'z {z0:.3f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
"

# openrua op 68
cd /workspace; sed -i 's/^FINGER_LEN = (0.063, 0.1124).*/FINGER_LEN = (0.063, 0.104)     # z range of a finger (tips measured at the TCP)/' geom.py && cat > sill_scan.py <<'EOF'
"""Scan: mug held upright, hovering h above the SILL (y_m ~0.26), hand tilted
back by theta about the closing axis (world x), rim pinch at the -x point."""
import numpy as np
from geom import *
from rlib import R_from_axes, TCP as TCPOFF
np.set_printoptions(precision=3, suppress=True)
SILL_Z = OPEN_Z[0]
mugpts = mug_points(0.004)
handpts = hand_points((0.0025, 0.0025), 0.004)

def sill_eval(x_m, y_m, h, theta_d, d):
    th = np.radians(theta_d)
    base = np.array([x_m, y_m, SILL_Z + h])
    rim_z = base[2] + MUG_H
    tcp = np.array([x_m - 0.0435, y_m, rim_z - d])
    Rh = R_from_axes([0, np.sin(th), -np.cos(th)], [-1, 0, 0])
    hand = tcp - TCPOFF * Rh[:, 2]
    cm, pm = clearance(transform(mugpts, np.eye(3), base))
    ch, ph = clearance(transform(handpts, Rh, hand))
    return cm, pm, ch, ph, hand, Rh, tcp

if __name__ == "__main__":
    for x_m in (-0.03,):
        for y_m in (0.255, 0.26, 0.265):
            for h in (0.003, 0.006):
                for th in (45, 50, 55, 60):
                    for d in (0.008, 0.010):
                        cm, pm, ch, ph, hand, Rh, tcp = sill_eval(x_m, y_m, h, th, d)
                        flag = "OK " if min(cm, ch) > 0.004 else "   "
                        print(f"{flag}y_m {y_m} h {h} th {th} d {d}: mug {cm:+.3f} at {pm} hand {ch:+.3f} at {ph}  hand {hand} tcp {tcp}")
EOF
python3 sill_scan.py

# openrua op 69
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('iktest')
q0=r.arm_q(); p,R=r.fk(); print('q0',np.array(q0)); print('hand',p,'R',R.round(2).tolist())
def test(name,hand,R,seed=q0,mj=None):
    q=r.ik(hand,R,seed=seed,max_jump=mj)
    if q is None: print(name,'IK FAIL'); return None
    p2,R2=r.fk(q); print(name,'ok q',np.array(q),'jump',np.abs(np.array(q)-np.array(seed)).max().round(2))
    return q
Rd=R_from_axes([0,0,-1],[-1,0,0])
qa=test('A pinch top-down', np.array([-0.1275,-0.288,0.997])+TCP*np.array([0,0,1]), Rd)
Rz=np.array([[-1,0,0],[0,-1,0],[0,0,1.0]])
qb=test('B rotated pi', np.array([-0.1275,-0.288,1.06])+TCP*np.array([0,0,1]), Rd@Rz, seed=qa)
Rh=R_from_axes([0,1,0],[1,0,0])
qc=test('C handle grasp table', [-0.06,-0.403,0.98], Rh)
qc2=test('C2 handle pre (further back)', [-0.06,-0.46,0.98], Rh, seed=qc if qc else q0)
qd=test('D insert', [-0.06,0.182,1.02], Rh, seed=qc if qc else q0)
qe=test('E pre-insert', [-0.06,0.05,1.035], Rh, seed=qd if qd else q0)
Rh2=R_from_axes([0,1,0],[-1,0,0])
qd2=test('D2 insert mount down', [-0.06,0.182,1.02], Rh2)
"

# openrua op 70
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('iktest2')
q0=r.arm_q()
Rh2=R_from_axes([0,1,0],[-1,0,0])
for x in (-0.12,-0.06,0.0):
  for y in (-0.40,-0.35,-0.30,-0.25,-0.20,-0.15):
    q=r.ik([x,y,0.98],Rh2,seed=q0,tries=4)
    print(x,y,'ok' if q is not None else 'FAIL', None if q is None else np.array(q))
qa=[-0.461,0.53,-0.035,-2.006,0.031,2.535,1.84]
Rd=R_from_axes([0,0,-1],[-1,0,0]); Rz=np.array([[-1,0,0],[0,-1,0],[0,0,1.0]])
seed=list(qa); seed[6]-=np.pi
qb=r.ik(np.array([-0.1275,-0.288,1.06])+TCP*np.array([0,0,1]), Rd@Rz, seed=seed, max_jump=0.5)
print('B',qb)
"

# openrua op 71
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('pinch')
print('open',r.gripper(0.04))
q0=r.arm_q(); p,R=r.fk()
Rd=R_from_axes([0,0,-1],[-1,0,0])
def go(tcp,R,n=3,t=3.0,seed=None,mj=0.6):
    p0,_=r.fk(); hand=np.array(tcp)-TCP*R[:,2]
    wps=r.line_ik(p0,hand,R,n,seed=seed,max_jump=mj)
    code,err=r.move(wps,t)
    if err>0.02: code,err=r.move(wps[-1],1.5)
    p1,R1=r.fk(); print('  at tcp',p1+TCP*R1[:,2],'code',code,'err',round(err,4))
    return wps[-1]
print('pre'); go([-0.1275,-0.288,1.06],Rd)
print('descend'); go([-0.1275,-0.288,0.997],Rd,n=3,t=3.0)
print('close',r.gripper(0.0))
print('lift'); go([-0.1275,-0.288,1.06],Rd,n=2,t=2.0)
print('fingers',r.fingers())
" && timeout 120 python3 cloudtf.py agentview agent_lift | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/agentview_lift_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
m=(np.abs(W[:,0]+0.084)<0.09)&(np.abs(W[:,1]+0.288)<0.09)&(W[:,2]>0.905)&(W[:,2]<1.2)
P=W[m]; print('pts',len(P))
for z0 in np.arange(0.90,1.20,0.02):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]
    if len(s): print(f'z {z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
"

# openrua op 72
cd /workspace; ls snaps | grep agent_lift; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/agent_lift_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
m=(np.abs(W[:,0]+0.084)<0.1)&(np.abs(W[:,1]+0.288)<0.1)&(W[:,2]>0.903)&(W[:,2]<1.2)
P=W[m]; print('pts',len(P))
for z0 in np.arange(0.90,1.20,0.02):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]
    if len(s): print(f'z {z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
"

# openrua op 73
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('rot')
q0=r.arm_q(); p,R=r.fk(); tcp=p+TCP*R[:,2]; print('tcp',tcp)
Rz=np.array([[-1,0,0],[0,-1,0],[0,0,1.0]]); R2=R@Rz
seed=list(q0); seed[6]-=np.pi
q1=r.ik(p,R2,seed=seed,max_jump=0.5); print('q1',np.array(q1))
# rotate in place: interpolate q7 only-ish (straight joint path)
wps=[list(np.array(q0)+(np.array(q1)-np.array(q0))*s) for s in (0.25,0.5,0.75,1.0)]
code,err=r.move(wps,4.0); print('rot',code,err)
if err>0.02: print(r.move(q1,1.5))
p,R=r.fk(); print('after rot tcp',p+TCP*R[:,2],'R',R.round(2).tolist(),'fingers',r.fingers())
def go(tcp,R,n=3,t=3.0,mj=0.6):
    p0,_=r.fk(); hand=np.array(tcp)-TCP*R[:,2]
    wps=r.line_ik(p0,hand,R,n,max_jump=mj)
    code,err=r.move(wps,t)
    if err>0.02: code,err=r.move(wps[-1],1.5)
    p1,R1=r.fk(); print('  at tcp',p1+TCP*R1[:,2],'code',code,'err',round(err,4))
print('carry'); go([-0.0765,-0.10,1.06],R,n=4,t=4.0)
print('lower'); go([-0.0765,-0.10,1.003],R,n=2,t=2.0)
print('open',r.gripper(0.04))
print('lift'); go([-0.0765,-0.10,1.10],R,n=2,t=2.0)
" && timeout 120 python3 cloudtf.py agentview agent_set | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/agent_set_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
m=(np.abs(W[:,0]+0.12)<0.12)&(np.abs(W[:,1]+0.10)<0.12)&(W[:,2]>0.903)&(W[:,2]<1.09)
P=W[m]; print('pts',len(P))
for z0 in np.arange(0.90,1.09,0.01):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
    if len(s): print(f'z {z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
top=P[P[:,2]>1.005]; print('rim-level pts centre', top[:,:2].mean(0) if len(top) else None)
"

# openrua op 74
cd /workspace; timeout 120 python3 tools/perception/cam_snap.py birdview snaps/bird_set.png; timeout 120 python3 tools/perception/cam_snap.py agentview snaps/agent_set.png

# openrua op 75
cd /workspace; timeout 300 python3 -c "
import numpy as np
from rlib import *
r=Robot('away')
p,R=r.fk(); hand=np.array([-0.15,-0.25,1.30])
wps=r.line_ik(p,hand,R,3); print(r.move(wps,3.0))
" && timeout 120 python3 cloudtf.py birdview bird_set2 | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/bird_set2_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
m=(W[:,0]>-0.2)&(W[:,0]<0.12)&(W[:,1]>-0.25)&(W[:,1]<0.08)&(W[:,2]>0.905)&(W[:,2]<1.05)
P=W[m]; print('pts',len(P))
for z0 in np.arange(0.90,1.05,0.01):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
    if len(s): print(f'z {z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
# separate the two mugs by clustering rim-level points (z>1.0)
top=P[P[:,2]>1.0]
from collections import Counter
# simple 2-means
c=np.array([[-0.06,-0.08],[0.027,-0.005]])
for it in range(10):
    lab=np.argmin(((top[:,None,:2]-c[None])**2).sum(-1),1)
    c=np.array([top[lab==k,:2].mean(0) for k in range(2)])
for k in range(2):
    s=top[lab==k]; print('cluster',k,'centre',c[k],'n',len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
# handle: points at z 0.95-0.99 farther than 0.05 from either centre
mid=P[(P[:,2]>0.95)&(P[:,2]<0.995)]
d=np.sqrt(((mid[:,None,:2]-c[None])**2).sum(-1)).min(1)
h=mid[d>0.052]; print('handle pts',len(h))
for k in range(2):
    hk=h[np.argmin(((h[:,None,:2]-c[None])**2).sum(-1),1)==k]
    if len(hk): print('handle of',k,'mean',hk[:,:2].mean(0),'dir',(hk[:,:2].mean(0)-c[k]))
"

# openrua op 76
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('handle')
c=np.array([-0.069,-0.063]); ang=np.radians(162.0); u=np.array([np.cos(ang),np.sin(ang),0])
zh=-u; yh=np.array([-u[1],u[0],0])  # perpendicular
Rh=R_from_axes(zh,yh); print('xh (should be +z)',Rh[:,0])
tcp=np.array([*(c+0.06*u[:2]),0.975]); print('tcp',tcp)
def go(tcp,R,n=3,t=3.0,mj=0.7):
    p0,_=r.fk(); hand=np.array(tcp)-TCP*R[:,2]
    wps=r.line_ik(p0,hand,R,n,max_jump=mj)
    code,err=r.move(wps,t)
    if err>0.02: code,err=r.move(wps[-1],1.5)
    p1,R1=r.fk(); print('  at tcp',p1+TCP*R1[:,2],'code',code,'err',round(err,4))
q0=r.arm_q()
pre=tcp-0.10*zh+np.array([0,0,0.06])
hand_pre=pre-TCP*zh
q=r.ik(hand_pre,Rh,seed=q0); print('pre ik',None if q is None else np.array(q))
if q is None: raise SystemExit('no ik')
# check path
out=path_check(r,q0,q,n=6); print_path(out)
code,err=r.move(q,4.0); print('to pre',code,err)
if err>0.02: print(r.move(q,2.0))
print('descend to approach height'); go(tcp-0.10*zh,Rh,n=2,t=2.0)
print('advance'); go(tcp,Rh,n=3,t=3.0)
print('close',r.gripper(0.0))
print('fingers',r.fingers())
"

# openrua op 77
cd /workspace; timeout 120 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('st')
q=r.arm_q(); print('q',np.array(q)); p,R=r.fk(); print('hand',p,'tcp',p+TCP*R[:,2]); print('R',R.round(2).tolist()); print('fingers',r.fingers())
for L in ('panda_link7','panda_link6','panda_link5','panda_link4'): print(L, r.fk(link=L)[0])
"; timeout 120 python3 tools/perception/cam_snap.py birdview snaps/bird_crash.png; timeout 120 python3 tools/perception/cam_snap.py agentview snaps/agent_crash.png

# openrua op 78
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('recover')
q0=r.arm_q(); p,R=r.fk()
wps=r.line_ik(p,p+np.array([0,0,0.15]),R,3,max_jump=0.8)
code,err=r.move(wps,3.0); print('up',code,err)
if err>0.02: print(r.move(wps[-1],2.0))
p,R=r.fk(); print('hand',p)
Rd=R_from_axes([0,0,-1],[-1,0,0])
q1=r.ik([-0.15,-0.25,1.30],Rd,seed=[-0.43,0.33,-0.04,-1.74,0.01,2.07,1.88]); print('q1',np.array(q1))
out=path_check(r,r.arm_q(),q1,n=8); print_path(out)
" 

# openrua op 79
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('recover2')
q0=r.arm_q(); p,R=r.fk()
wps=r.line_ik(p,[-0.40,-0.05,1.36],R,3,max_jump=0.8)
code,err=r.move(wps,3.0); print('up',code,err)
if err>0.02: print(r.move(wps[-1],2.0))
Rd=R_from_axes([0,0,-1],[-1,0,0])
q1=r.ik([-0.15,-0.25,1.36],Rd,seed=[-0.43,0.33,-0.04,-1.74,0.01,2.07,1.88]); print('q1',np.array(q1))
out=path_check(r,r.arm_q(),q1,n=8)
zmin=min(min(v[2] for v in d.values()) for s,d in out); print('min z along path',round(zmin,3))
for s,d in out: print(f's={s:.2f} tcp={d[\"tcp\"].round(2)}')
if zmin>1.15:
    code,err=r.move(q1,5.0); print('move',code,err)
    if err>0.02: print(r.move(q1,2.0))
p,R=r.fk(); print('hand',p, 'tcp',p+TCP*R[:,2])
"

# openrua op 80
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('recover3')
q0=r.arm_q(); p,R=r.fk(); print('hand',p,'q',np.array(q0))
Rd=R_from_axes([0,0,-1],[-1,0,0])
# candidate targets: pointing down, high; pick one with a joint path that stays high
best=None
for tgt in ([-0.30,-0.10,1.36],[-0.25,-0.15,1.38],[-0.20,-0.20,1.38],[-0.15,-0.25,1.36]):
    for seed in (q0,[-0.43,0.33,-0.04,-1.74,0.01,2.07,1.88]):
        q1=r.ik(tgt,Rd,seed=seed,tries=4)
        if q1 is None: continue
        out=path_check(r,q0,q1,n=8)
        zmin=min(min(v[2] for v in d.values()) for s,d in out)
        jump=np.abs(np.array(q1)-np.array(q0)).max()
        print(tgt,'zmin',round(zmin,3),'jump',round(jump,2))
        if zmin>1.12 and (best is None or zmin>best[0]): best=(zmin,q1,tgt)
print('best',best)
if best:
    code,err=r.move(best[1],5.0); print('move',code,err)
    if err>0.02: print(r.move(best[1],2.0))
    p,R=r.fk(); print('hand',p)
"

# openrua op 81
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('recover4')
q0=r.arm_q(); print('q',np.array(q0)); p,R=r.fk(); print('hand',p)
home=[-0.43,0.33,-0.04,-1.74,0.01,2.07,1.88]
out=path_check(r,q0,home,n=10,links=('panda_hand','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7'))
for s,d in out: print(f's={s:.2f} '+' '.join(f'{k}={v.round(2)}' for k,v in d.items() if k in ('tcp','panda_hand','panda_link7','panda_link6')))
zmin=min(min(v[2] for v in d.values()) for s,d in out); print('zmin',zmin)
"

# openrua op 82
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('recover5')
home=[-0.43,0.33,-0.04,-1.74,0.01,2.07,1.88]
code,err=r.move(home,6.0); print('home',code,err)
if err>0.02: print(r.move(home,2.0))
p,R=r.fk(); print('hand',p,'q',np.array(r.arm_q()))
" && timeout 120 python3 cloudtf.py birdview bird_after | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/bird_after_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
# door: tall thin structure z>1.0 in x<-0.15, y<0.27
D=W[(W[:,2]>1.02)&(W[:,2]<1.12)&(W[:,0]<-0.15)&(W[:,0]>-0.40)&(W[:,1]<0.27)&(W[:,1]>-0.15)]
print('door pts',len(D))
for y0 in np.arange(-0.10,0.27,0.03):
    s=D[(D[:,1]>=y0)&(D[:,1]<y0+0.03)]
    if len(s): print(f'y {y0:+.2f} n{len(s):3d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}]')
# mugs
P=W[(W[:,0]>-0.2)&(W[:,0]<0.12)&(W[:,1]>-0.25)&(W[:,1]<0.08)&(W[:,2]>1.0)&(W[:,2]<1.05)]
c=np.array([[-0.07,-0.06],[0.02,0.0]])
for it in range(10):
    lab=np.argmin(((P[:,None,:2]-c[None])**2).sum(-1),1); c=np.array([P[lab==k,:2].mean(0) for k in range(2)])
print('mug centres',c)
"

# openrua op 83
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
import time
r=Robot('slow')
home=[-0.43,0.33,-0.04,-1.74,0.01,2.07,1.88]
q0=np.array(r.arm_q()); print('q0',q0)
n=8; wps=[list(q0+(np.array(home)-q0)*(i+1)/n) for i in range(n)]
t0=time.time(); code,err=r.move(wps,12.0); print('home 12s',code,err,'wall',round(time.time()-t0,1))
print('q',np.array(r.arm_q()))
if err>0.02:
    code,err=r.move(home,3.0); print('resend',code,err)
p,R=r.fk(); print('hand',p)
"

# openrua op 84
cd /workspace; timeout 300 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('base')
print('link0',r.fk(link='panda_link0')[0]); print('link1',r.fk(link='panda_link1')[0])
Rd=R_from_axes([0,0,-1],[-1,0,0])
q0=r.arm_q()
for tgt in ([0.0625,0.001,1.10],[0.10,-0.20,1.10],[0.15,-0.20,1.10],[0.19,-0.20,1.10],[0.12,-0.25,1.10]):
    q=r.ik(tgt,Rd,seed=q0,tries=4); print(tgt,'ok' if q else 'FAIL', None if q is None else np.abs(np.array(q)-np.array(q0)).max().round(2))
"

# openrua op 85
cd /workspace; cat >> rlib.py <<'EOF'


def go_tcp(r, tcp, R, n=3, rate=0.6, min_t=2.0, mj=0.7, seed=None):
    """Straight-line TCP move (hand orientation R) with duration scaled to the
    joint distance; resends once if the controller stops short."""
    p0, _ = r.fk()
    hand = np.asarray(tcp, float) - TCP * R[:, 2]
    wps = r.line_ik(p0, hand, R, n, seed=seed, max_jump=mj)
    q0 = np.array(r.arm_q())
    dist = float(np.abs(np.array(wps[-1]) - q0).max())
    t = max(min_t, dist / rate)
    code, err = r.move(wps, t)
    if err > 0.02:
        code, err = r.move(wps[-1], max(1.5, err / rate))
    p1, R1 = r.fk()
    print(f"  tcp -> {np.round(p1 + TCP * R1[:, 2], 3)} code {code} err {err:.4f}")
    return wps[-1]


def go_joint(r, q1, rate=0.25, n=None, min_t=2.0):
    """Joint-space move with dense waypoints, slow enough for the controller."""
    q0 = np.array(r.arm_q()); q1 = np.array(q1, float)
    dist = float(np.abs(q1 - q0).max())
    t = max(min_t, dist / rate)
    n = n or max(2, int(np.ceil(t)))
    wps = [list(q0 + (q1 - q0) * (i + 1) / n) for i in range(n)]
    code, err = r.move(wps, t)
    if err > 0.02:
        code, err = r.move(list(q1), max(1.5, err / rate))
    print(f"  joint move code {code} err {err:.4f} ({t:.1f}s)")
    return code, err
EOF
timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('yellow')
Rd=R_from_axes([0,0,-1],[-1,0,0])
print('fingers',r.fingers())
print('pre'); go_tcp(r,[-0.0245,0.001,1.06],Rd,n=4)
print('descend'); go_tcp(r,[-0.0245,0.001,0.997],Rd,n=2)
print('close',r.gripper(0.0))
print('lift'); go_tcp(r,[-0.0245,0.001,1.06],Rd,n=2)
print('carry'); go_tcp(r,[0.0565,-0.17,1.06],Rd,n=4)
print('lower'); go_tcp(r,[0.0565,-0.17,1.003],Rd,n=2)
print('open',r.gripper(0.04))
print('lift'); go_tcp(r,[0.0565,-0.17,1.12],Rd,n=2)
" && timeout 120 python3 cloudtf.py birdview bird_y | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/bird_y_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
P=W[(W[:,0]>-0.2)&(W[:,0]<0.2)&(W[:,1]>-0.3)&(W[:,1]<0.1)&(W[:,2]>1.0)&(W[:,2]<1.05)]
c=np.array([[-0.07,-0.06],[0.10,-0.17]])
for it in range(10):
    lab=np.argmin(((P[:,None,:2]-c[None])**2).sum(-1),1); c=np.array([P[lab==k,:2].mean(0) for k in range(2)])
print('mug centres',c, 'counts',[int((lab==k).sum()) for k in range(2)])
mid=W[(W[:,0]>-0.2)&(W[:,0]<0.2)&(W[:,1]>-0.3)&(W[:,1]<0.1)&(W[:,2]>0.95)&(W[:,2]<0.995)]
d=np.sqrt(((mid[:,None,:2]-c[None])**2).sum(-1)); k=d.argmin(1); h=mid[d.min(1)>0.052]; kh=k[d.min(1)>0.052]
for j in range(2):
    hj=h[kh==j]
    if len(hj): v=hj[:,:2].mean(0)-c[j]; print('handle',j,'dir deg',np.degrees(np.arctan2(v[1],v[0])).round(1),'n',len(hj))
"

# openrua op 86
cd /workspace; timeout 120 python3 tools/perception/cam_snap.py agentview snaps/agent_y.png

# openrua op 87
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('rot2')
Rd=R_from_axes([0,0,-1],[-1,0,0])
psi=np.radians(112.0); c,s=np.cos(psi),np.sin(psi)
Rz=np.array([[c,-s,0],[s,c,0],[0,0,1.0]]); R2=Rz@Rd
print('pre'); go_tcp(r,[-0.1155,-0.06,1.06],Rd,n=4)
print('descend'); go_tcp(r,[-0.1155,-0.06,0.997],Rd,n=2)
print('close',r.gripper(0.0))
print('lift'); go_tcp(r,[-0.1155,-0.06,1.06],Rd,n=2)
q0=r.arm_q(); p,_=r.fk()
for dq7 in (-psi,+psi):
    seed=list(q0); seed[6]+=dq7
    q1=r.ik(p,R2,seed=seed,max_jump=0.5)
    if q1 is not None: print('rot solution with dq7',round(dq7,2),np.array(q1)-np.array(q0)); break
if q1 is None: raise SystemExit('no rotation ik')
go_joint(r,q1,rate=0.3)
p,R=r.fk(); print('R after',R.round(2).tolist(),'fingers',r.fingers())
print('carry'); go_tcp(r,[-0.0737,-0.1003,1.06],R,n=3)
print('lower'); go_tcp(r,[-0.0737,-0.1003,1.003],R,n=2)
print('open',r.gripper(0.04))
print('lift'); go_tcp(r,[-0.0737,-0.1003,1.15],R,n=2)
" && timeout 120 python3 cloudtf.py birdview bird_w | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/bird_w_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
P=W[(W[:,0]>-0.2)&(W[:,0]<0.0)&(W[:,1]>-0.2)&(W[:,1]<0.05)&(W[:,2]>1.0)&(W[:,2]<1.05)]
c=P[:,:2].mean(0); print('white centre',c,'n',len(P),'x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3))
mid=W[(W[:,0]>-0.2)&(W[:,0]<0.0)&(W[:,1]>-0.25)&(W[:,1]<0.05)&(W[:,2]>0.95)&(W[:,2]<0.995)]
d=np.linalg.norm(mid[:,:2]-c,axis=1); h=mid[d>0.052]
v=h[:,:2].mean(0)-c; print('handle dir deg',np.degrees(np.arctan2(v[1],v[0])).round(1),'n',len(h),'handle z',h[:,2].min().round(3),h[:,2].max().round(3),'handle y',h[:,1].min().round(3),h[:,1].max().round(3))
"

# openrua op 88
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('away2')
p,R=r.fk(); go_tcp(r,[-0.20,-0.30,1.25],R,n=3)
" && timeout 120 python3 cloudtf.py birdview bird_w2 | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/bird_w2_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
P=W[(W[:,0]>-0.25)&(W[:,0]<0.02)&(W[:,1]>-0.25)&(W[:,1]<0.05)&(W[:,2]>1.0)&(W[:,2]<1.05)]
print('n',len(P))
for z0 in np.arange(0.90,1.05,0.01):
    s=W[(W[:,0]>-0.25)&(W[:,0]<0.02)&(W[:,1]>-0.25)&(W[:,1]<0.05)&(W[:,2]>=z0)&(W[:,2]<z0+0.01)]
    if len(s): print(f'z {z0:.2f} n{len(s):4d} x[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
c=np.array([(P[:,0].min()+P[:,0].max())/2,(P[:,1].min()+P[:,1].max())/2]); print('white centre (bbox)',c,'mean',P[:,:2].mean(0))
mid=W[(W[:,0]>-0.25)&(W[:,0]<0.02)&(W[:,1]>-0.25)&(W[:,1]<0.05)&(W[:,2]>0.93)&(W[:,2]<0.995)]
d=np.linalg.norm(mid[:,:2]-c,axis=1); h=mid[d>0.052]
v=h[:,:2].mean(0)-c; print('handle dir deg',np.degrees(np.arctan2(v[1],v[0])).round(1),'n',len(h),'handle z',h[:,2].min().round(3),h[:,2].max().round(3),'handle x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3))
"

# openrua op 89
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('rot3')
Rd=R_from_axes([0,0,-1],[-1,0,0])
psi=np.radians(23.0); c,s=np.cos(psi),np.sin(psi)
Rz=np.array([[c,-s,0],[s,c,0],[0,0,1.0]]); R2=Rz@Rd
print('pre'); go_tcp(r,[-0.1175,-0.061,1.06],Rd,n=4)
print('descend'); go_tcp(r,[-0.1175,-0.061,0.997],Rd,n=2)
print('close',r.gripper(0.0))
print('lift'); go_tcp(r,[-0.1175,-0.061,1.06],Rd,n=2)
q0=r.arm_q(); p,_=r.fk()
seed=list(q0); seed[6]-=psi
q1=r.ik(p,R2,seed=seed,max_jump=0.3); print('dq',np.array(q1)-np.array(q0))
go_joint(r,q1,rate=0.12)
p,R=r.fk(); print('R after',R.round(3).tolist())
ang=np.degrees(np.arctan2(R[1,0],R[0,0])); print('hand x-axis angle deg',ang.round(1),'(started -90)')
print('carry'); go_tcp(r,[-0.13,-0.077,1.06],R,n=3)
print('lower'); go_tcp(r,[-0.13,-0.077,1.003],R,n=2)
print('open',r.gripper(0.04))
print('lift'); go_tcp(r,[-0.13,-0.077,1.15],R,n=2)
print('away'); go_tcp(r,[-0.20,-0.30,1.25],R,n=3)
" && timeout 120 python3 cloudtf.py birdview bird_w3 | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/bird_w3_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
box=(W[:,0]>-0.25)&(W[:,0]<0.02)&(W[:,1]>-0.25)&(W[:,1]<0.05)
P=W[box&(W[:,2]>1.0)&(W[:,2]<1.05)]
c=np.array([(P[:,0].min()+P[:,0].max())/2,(P[:,1].min()+P[:,1].max())/2]); print('white centre (bbox)',c,'n',len(P))
mid=W[box&(W[:,2]>0.93)&(W[:,2]<0.995)]
d=np.linalg.norm(mid[:,:2]-c,axis=1); h=mid[d>0.052]
v=h[:,:2].mean(0)-c; print('handle dir deg',np.degrees(np.arctan2(v[1],v[0])).round(1),'n',len(h),'handle z',h[:,2].min().round(3),h[:,2].max().round(3),'handle x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3))
"

# openrua op 90
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('rot3')
Rd=R_from_axes([0,0,-1],[-1,0,0])
q0=r.arm_q(); print('q0',np.array(q0))
pre=np.array([-0.1175,-0.061,1.06])+TCP*np.array([0,0,1])
best=None
for dq in (1.6,-1.6,0):
    seed=list(q0); seed[6]+=dq
    q1=r.ik(pre,Rd,seed=seed,max_jump=2.0)
    if q1 is None: continue
    out=path_check(r,q0,q1,n=8); zmin=min(min(v[2] for v in d.values()) for s,d in out)
    jump=np.abs(np.array(q1)-np.array(q0)).max(); print('dq',dq,'jump',jump.round(2),'zmin',round(zmin,3))
    if zmin>1.05 and (best is None or jump<best[0]): best=(jump,q1)
go_joint(r,best[1],rate=0.2)
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'R',R.round(2).tolist())
psi=np.radians(23.0); c,s=np.cos(psi),np.sin(psi)
Rz=np.array([[c,-s,0],[s,c,0],[0,0,1.0]]); R2=Rz@Rd
print('descend'); go_tcp(r,[-0.1175,-0.061,0.997],Rd,n=2)
print('close',r.gripper(0.0))
print('lift'); go_tcp(r,[-0.1175,-0.061,1.06],Rd,n=2)
q0=r.arm_q(); p,_=r.fk()
seed=list(q0); seed[6]-=psi
q1=r.ik(p,R2,seed=seed,max_jump=0.3); print('dq',np.array(q1)-np.array(q0))
go_joint(r,q1,rate=0.12)
p,R=r.fk(); print('hand x-axis angle deg',np.degrees(np.arctan2(R[1,0],R[0,0])).round(1),'(started -90)')
print('carry'); go_tcp(r,[-0.13,-0.077,1.06],R,n=3)
print('lower'); go_tcp(r,[-0.13,-0.077,1.003],R,n=2)
print('open',r.gripper(0.04))
print('lift'); go_tcp(r,[-0.13,-0.077,1.15],R,n=2)
print('away'); go_tcp(r,[-0.20,-0.30,1.25],R,n=3)
" && timeout 120 python3 cloudtf.py birdview bird_w3 | tail -1 && timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
W=np.load('snaps/bird_w3_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
box=(W[:,0]>-0.25)&(W[:,0]<0.02)&(W[:,1]>-0.25)&(W[:,1]<0.05)
P=W[box&(W[:,2]>1.0)&(W[:,2]<1.05)]
c=np.array([(P[:,0].min()+P[:,0].max())/2,(P[:,1].min()+P[:,1].max())/2]); print('white centre (bbox)',c,'n',len(P))
mid=W[box&(W[:,2]>0.93)&(W[:,2]<0.995)]
d=np.linalg.norm(mid[:,:2]-c,axis=1); h=mid[d>0.052]
v=h[:,:2].mean(0)-c; print('handle dir deg',np.degrees(np.arctan2(v[1],v[0])).round(1),'n',len(h),'handle z',h[:,2].min().round(3),h[:,2].max().round(3),'handle x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3))
"

# openrua op 91
cd /workspace; timeout 60 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
W=np.load('snaps/bird_w3_world.npy').reshape(-1,3); W=W[np.isfinite(W).all(1)]
box=(W[:,0]>-0.25)&(W[:,0]<0.02)&(W[:,1]>-0.25)&(W[:,1]<0.05)
P=W[box&(W[:,2]>1.0)&(W[:,2]<1.05)][:,:2]
# algebraic circle fit
A=np.c_[2*P,np.ones(len(P))]; b=(P**2).sum(1)
sol=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=sol[:2]; r=np.sqrt(sol[2]+cx**2+cy**2)
print('fit centre',cx,cy,'r',r)
res=np.linalg.norm(P-[cx,cy],axis=1)-r; print('resid std',res.std())
mid=W[box&(W[:,2]>0.93)&(W[:,2]<0.995)]
d=np.linalg.norm(mid[:,:2]-[cx,cy],axis=1); h=mid[d>0.052]
v=h[:,:2].mean(0)-[cx,cy]; print('handle dir deg',np.degrees(np.arctan2(v[1],v[0])),'handle centroid',h[:,:2].mean(0),'r range',d[d>0.052].min(),d[d>0.052].max())
print('handle z',h[:,2].min(),h[:,2].max())
"

# openrua op 92
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('hgrasp')
c=np.array([-0.102,-0.0567]); ang=np.radians(-78.6); u=np.array([np.cos(ang),np.sin(ang),0.0])
zh=-u; yh=np.array([-zh[1],zh[0],0.0])  # zh rotated -90deg about z
Rh=R_from_axes(zh,yh); print('xh',Rh[:,0].round(3))
tcp=np.array([*(c+0.054*u[:2]),0.979]); pre=tcp+0.07*u+np.array([0,0,0.0]); print('tcp',tcp,'pre',pre)
q0=r.arm_q()
hand_pre=pre-TCP*zh
q1=None
for seed in (q0,[-0.835,1.411,0.379,-0.902,0.79,1.218,1.633],[-0.96,1.224,0.391,-1.276,0.719,1.306,1.493]):
    q=r.ik(hand_pre,Rh,seed=seed,tries=4)
    if q is None: continue
    out=path_check(r,q0,q,n=10,links=('panda_hand','panda_link4','panda_link5','panda_link6','panda_link7'))
    zmin=min(min(v[2] for v in d.values()) for s,d in out)
    # hand/tcp xy path relative to door tip and mugs
    tcps=np.array([d['tcp'] for s,d in out]); l7=np.array([d['panda_link7'] for s,d in out])
    print('seed ok jump',np.abs(np.array(q)-np.array(q0)).max().round(2),'zmin',round(zmin,3)); print('  tcp path',tcps.round(2).tolist()); print('  l7 path',l7.round(2).tolist())
    if q1 is None: q1=q
print('chosen',np.array(q1))
"

# openrua op 93
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('hgrasp')
c=np.array([-0.102,-0.0567]); ang=np.radians(-78.6); u=np.array([np.cos(ang),np.sin(ang),0.0])
zh=-u; yh=np.array([-zh[1],zh[0],0.0]); Rh=R_from_axes(zh,yh); print('xh',Rh[:,0].round(3))
tcp=np.array([*(c+0.054*u[:2]),0.981]); pre=tcp+0.07*u; hover=pre+np.array([0,0,0.10])
q0=r.arm_q()
q=r.ik(hover-TCP*zh,Rh,seed=[-0.835,1.411,0.379,-0.902,0.79,1.218,1.633],tries=4)
print('q',np.array(q))
out=path_check(r,q0,q,n=12,links=('panda_hand','panda_link5','panda_link6','panda_link7'))
for s,d in out: print(round(s,2),{k:np.round(v,2).tolist() for k,v in d.items() if k in('tcp','panda_hand')})
zmin=min(min(v[2] for v in d.values()) for s,d in out); print('zmin',zmin)
np.save('snaps/q_hover.npy',np.array(q))
"

# openrua op 94
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('hgrasp')
c=np.array([-0.102,-0.0567]); ang=np.radians(-78.6); u=np.array([np.cos(ang),np.sin(ang),0.0])
zh=-u; yh=np.array([-zh[1],zh[0],0.0]); Rh=R_from_axes(zh,yh)
tcp=np.array([*(c+0.054*u[:2]),0.981]); pre=tcp+0.07*u
q=np.load('snaps/q_hover.npy')
go_joint(r,q,rate=0.25)
p,R=r.fk(); print('hover tcp',p+TCP*R[:,2],'R err',np.abs(R-Rh).max().round(3))
print('fingers',r.fingers())
go_tcp(r,pre,Rh,n=4,rate=0.4)
go_tcp(r,tcp,Rh,n=4,rate=0.3)
print('fingers before close',r.fingers())
r.gripper(0.0)
import time; time.sleep(1.0)
print('fingers after close',r.fingers())
"

# openrua op 95
cd /workspace; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_hg.png; timeout 300 python3 tools/perception/cam_snap.py frontview snaps/front_hg.png; timeout 300 python3 -c "
from rlib import *; import numpy as np; np.set_printoptions(precision=4,suppress=True)
r=Robot('chk'); p,R=r.fk(); print('tcp',p+TCP*R[:,2]); print('R',R); print('q',np.array(r.arm_q())); print('fingers',r.fingers())"

# openrua op 96
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('hg2')
r.gripper(0.04); import time; time.sleep(0.5); print('fingers',r.fingers())
p,R=r.fk(); tcp=p+TCP*R[:,2]
go_tcp(r,tcp+[0,0,0.06],R,n=3,rate=0.3)
# insertion reachability, mount UP (xh=+z), hand pointing +y
Rup=R_from_axes([0,1,0],[1,0,0]); print('xh',Rup[:,0])
for hx in (-0.03,-0.06,-0.09,-0.12):
  for hy in (0.05,0.12,0.182):
    for hz in (1.02,1.03):
      q=r.ik([hx,hy,hz],Rup,seed=r.arm_q(),tries=6)
      print(hx,hy,hz, None if q is None else np.array(q))
"

# openrua op 97
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('hg3')
c=np.array([-0.102,-0.0567]); ang=np.radians(-78.6); u=np.array([np.cos(ang),np.sin(ang),0.0])
zh=-u; yh=-np.array([-zh[1],zh[0],0.0]); Rh=R_from_axes(zh,yh); print('xh',Rh[:,0].round(3))
tcp=np.array([*(c+0.054*u[:2]),0.981]); pre=tcp+0.07*u; hover=pre+np.array([0,0,0.10])
q0=r.arm_q(); best=None
for seed in (q0,[-0.643,1.237,0.952,-1.775,0.951,2.267,-1.69],[-0.882,1.117,0.875,-1.91,0.868,2.078,-1.694]):
    q=r.ik(hover-TCP*zh,Rh,seed=seed,tries=4)
    if q is None: print('fail'); continue
    out=path_check(r,q0,q,n=12,links=('panda_hand','panda_link5','panda_link6','panda_link7'))
    zmin=min(min(v[2] for v in d.values()) for s,d in out)
    tcps=np.array([d['tcp'] for s,d in out])
    print('q',np.array(q),'jump',np.abs(np.array(q)-np.array(q0)).max().round(2),'zmin',round(zmin,3)); print('  tcp',tcps.round(2).tolist())
    if best is None: best=q
np.save('snaps/q_hover_up.npy',np.array(best))
"

# openrua op 98
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('hg4')
c=np.array([-0.102,-0.0567]); ang=np.radians(-78.6); u=np.array([np.cos(ang),np.sin(ang),0.0])
zh=-u; yh=-np.array([-zh[1],zh[0],0.0]); Rh=R_from_axes(zh,yh)
tcp=np.array([*(c+0.054*u[:2]),0.981]); pre=tcp+0.07*u; hover=pre+np.array([0,0,0.15])
p,R=r.fk(); go_tcp(r,p+TCP*R[:,2]+[0,0,0.05],R,n=3,rate=0.3)
q=r.ik(hover-TCP*zh,Rh,seed=[-1.045,1.351,0.826,-1.044,0.412,1.18,-1.309],tries=4); print('q',np.array(q))
go_joint(r,q,rate=0.2)
p,R=r.fk(); print('hover tcp',p+TCP*R[:,2],'xh',R[:,0])
go_tcp(r,pre,Rh,n=5,rate=0.3)
print('fingers',r.fingers())
go_tcp(r,tcp,Rh,n=4,rate=0.25)
p,R=r.fk(); print('at grasp tcp',p+TCP*R[:,2],'zh',R[:,2])
"

# openrua op 99
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *; import time
r=Robot('hg5'); r.gripper(0.0); time.sleep(1.0); print('fingers',r.fingers())
" ; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_hg2.png; timeout 300 python3 tools/perception/cam_snap.py frontview snaps/front_hg2.png

# openrua op 100
cd /workspace; timeout 600 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('hg6'); p,R=r.fk(); tcp=p+TCP*R[:,2]
go_tcp(r,tcp+[0,0,0.06],R,n=4,rate=0.15)
print('fingers',r.fingers())
" ; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_lift2.png; timeout 300 python3 tools/perception/cam_snap.py frontview snaps/front_lift2.png

# openrua op 101
cd /workspace; timeout 300 python3 cloudtf.py birdview lift2 >/dev/null; timeout 300 python3 cloudtf.py frontview lift2f >/dev/null; python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
P=np.load('snaps/lift2_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# white mug region: near (-0.10,-0.06), exclude hand (y<-0.10 z>1.0?)
m=(np.abs(P[:,0]+0.10)<0.09)&(np.abs(P[:,1]+0.05)<0.09)&(P[:,2]>0.96)
Q=P[m]; print('n',len(Q),'z range',Q[:,2].min(),Q[:,2].max())
top=Q[Q[:,2]>Q[:,2].max()-0.012]; print('rim pts',len(top),'rim z mean',top[:,2].mean(), 'z std',top[:,2].std())
# circle fit
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1); s=np.linalg.lstsq(A,b,rcond=None)[0]
cx,cy=s[0],s[1]; rr=np.sqrt(s[2]+cx**2+cy**2); print('rim centre',cx,cy,'r',rr)
# rim height vs angle
ang=np.degrees(np.arctan2(top[:,1]-cy,top[:,0]-cx))
for a0 in range(-180,180,45):
    sel=(ang>=a0)&(ang<a0+45)
    if sel.sum(): print(a0, round(top[sel,2].mean(),4), sel.sum())
F=np.load('snaps/lift2f_world.npy').reshape(-1,3); F=F[np.isfinite(F).all(1)]
m=(np.abs(F[:,0]+0.10)<0.06)&(np.abs(F[:,1]+0.05)<0.06)&(F[:,2]>0.93)&(F[:,2]<1.2)
G=F[m]; print('front: mug z min',G[:,2].min(),'max',G[:,2].max())
"

# openrua op 102
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('yaw'); p,R=r.fk(); tcp=p+TCP*R[:,2]; print('tcp',tcp)
Rup=R_from_axes([0,1,0],[-1,0,0]); print('xh',Rup[:,0])
q0=np.array(r.arm_q())
q=r.ik(tcp-TCP*Rup[:,2],Rup,seed=q0,tries=6,max_jump=0.6); print('q',np.array(q),'jump',np.abs(np.array(q)-q0).round(3))
out=path_check(r,q0,q,n=6,links=('panda_hand','panda_link7'))
for s,d in out: print(round(s,2),np.round(d['tcp'],3))
go_joint(r,q,rate=0.1,n=6,min_t=3)
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'zh',R[:,2],'xh',R[:,0],'fingers',r.fingers())
"

# openrua op 103
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('yaw'); p,R=r.fk(); tcp=p+TCP*R[:,2]; print('tcp',tcp)
Rup=R_from_axes([0,1,0],[1,0,0]); print('xh',Rup[:,0])
q0=np.array(r.arm_q())
q=r.ik(tcp-TCP*Rup[:,2],Rup,seed=q0,tries=6,max_jump=0.6); print('q',np.array(q),'jump',np.abs(np.array(q)-q0).round(3))
out=path_check(r,q0,q,n=6,links=('panda_hand','panda_link7'))
for s,d in out: print(round(s,2),np.round(d['tcp'],3))
go_joint(r,q,rate=0.1,n=6,min_t=3)
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'zh',R[:,2],'xh',R[:,0],'fingers',r.fingers())
"

# openrua op 104
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('carry'); p,R=r.fk(); tcp=p+TCP*R[:,2]
Rup=R_from_axes([0,1,0],[1,0,0])
go_tcp(r,[tcp[0],tcp[1],1.048],Rup,n=2,rate=0.15)
p,R=r.fk(); tcp=p+TCP*R[:,2]
tgt=np.array([-0.065,0.10,1.048])
wps=r.line_ik(tcp,tgt-TCP*Rup[:,2],Rup,8,seed=r.arm_q(),max_jump=0.5)
q=np.array(r.arm_q())
for w in wps:
    fkd=r.fk_links(w,('panda_hand','panda_link5','panda_link6','panda_link7')) if hasattr(r,'fk_links') else None
    print(np.round(np.array(w)-q,2)); q=np.array(w)
np.save('snaps/wps_carry.npy',np.array(wps))
" 2>&1 | tail -20; grep -n "def path_check" -A 25 rlib.py | head -40

# openrua op 105
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('carry2'); wps=np.load('snaps/wps_carry.npy'); q=np.array(r.arm_q())
allq=[q]+list(wps)
for i in range(len(allq)-1):
    out=path_check(r,allq[i],allq[i+1],n=2,links=('panda_hand','panda_link5','panda_link6','panda_link7'))
    for s,d in out[1:]:
        print(i,s,' '.join(f'{k}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})' for k,v in d.items()))
"

# openrua op 106
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('carry3'); p,R=r.fk(); Rup=R_from_axes([0,1,0],[1,0,0])
tgt=np.array([-0.065,0.10,1.048])
wps=r.line_ik(p,tgt-TCP*Rup[:,2],Rup,10,seed=r.arm_q(),max_jump=0.5)
q=np.array(r.arm_q()); allq=[q]+list(wps)
for i in range(len(allq)-1):
    print(np.round(allq[i+1]-allq[i],2))
    out=path_check(r,allq[i],allq[i+1],n=2,links=('panda_hand','panda_link5','panda_link7'))
    for s,d in out[1:]:
        print(i,s,' '.join(f'{k}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})' for k,v in d.items()))
np.save('snaps/wps_carry.npy',np.array(wps))
"

# openrua op 107
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('carry3'); p,R=r.fk(); Rup=R_from_axes([0,1,0],[1,0,0])
tgt=np.array([-0.065,0.10,1.048])
wps=[np.array(w) for w in r.line_ik(p,tgt-TCP*Rup[:,2],Rup,10,seed=r.arm_q(),max_jump=0.5)]
q=np.array(r.arm_q()); allq=[q]+wps
for i in range(len(allq)-1):
    out=path_check(r,allq[i],allq[i+1],n=1,links=('panda_hand','panda_link5','panda_link7'))
    s,d=out[-1]; print(i,np.round(allq[i+1]-allq[i],2),' '.join(f'{k}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})' for k,v in d.items()))
np.save('snaps/wps_carry.npy',np.array(wps))
"

# openrua op 108
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('carry4'); wps=[list(w) for w in np.load('snaps/wps_carry.npy')]
code,err=r.move(wps,10.0); print('carry code',code,'err',err)
if err>0.02: print(r.move(wps[-1],3.0))
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'zh',R[:,2],'xh',R[:,0],'fingers',r.fingers())
"; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_pre.png; timeout 300 python3 cloudtf.py birdview pre >/dev/null; python3 -c "
import numpy as np
P=np.load('snaps/pre_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(np.abs(P[:,0]+0.065)<0.08)&(np.abs(P[:,1]-0.154)<0.08)&(P[:,2]>1.05)&(P[:,2]<1.09)
Q=P[m]; print('n',len(Q))
top=Q[Q[:,2]>Q[:,2].max()-0.012]
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1); s=np.linalg.lstsq(A,b,rcond=None)[0]
cx,cy=s[0],s[1]; print('rim centre',round(cx,4),round(cy,4),'r',round(np.sqrt(s[2]+cx**2+cy**2),4),'rim z',round(top[:,2].mean(),4),'std',round(top[:,2].std(),4))
"

# openrua op 109
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('ins'); Rup=R_from_axes([0,1,0],[1,0,0])
go_tcp(r,[-0.065,0.0994,1.051],Rup,n=2,rate=0.15)
go_tcp(r,[-0.065,0.284,1.051],Rup,n=8,rate=0.15,min_t=6)
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'zh',R[:,2],'fingers',r.fingers())
"; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_in.png; timeout 300 python3 tools/perception/cam_snap.py sideview snaps/side_in.png

# openrua op 110
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *; import time
r=Robot('rel'); Rup=R_from_axes([0,1,0],[1,0,0])
go_tcp(r,[-0.0657,0.2837,1.030],Rup,n=2,rate=0.1)
r.gripper(0.04); time.sleep(1.0); print('fingers',r.fingers())
go_tcp(r,[-0.0657,0.10,1.030],Rup,n=6,rate=0.2,min_t=4)
p,R=r.fk(); print('tcp',p+TCP*R[:,2])
"; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_rel.png; timeout 300 python3 cloudtf.py birdview rel >/dev/null; timeout 300 python3 cloudtf.py agentview rela >/dev/null; python3 -c "
import numpy as np
for f in ('rel','rela'):
    P=np.load(f'snaps/{f}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.15)&(P[:,0]<0.05)&(P[:,1]>0.27)&(P[:,1]<0.42)&(P[:,2]>0.945)&(P[:,2]<1.08)
    Q=P[m]; print(f,'pts in cavity',len(Q))
    if len(Q):
        print(' z range',Q[:,2].min().round(4),Q[:,2].max().round(4),'y range',Q[:,1].min().round(3),Q[:,1].max().round(3),'x range',Q[:,0].min().round(3),Q[:,0].max().round(3))
        top=Q[Q[:,2]>Q[:,2].max()-0.012]
        if len(top)>10:
            A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1); s=np.linalg.lstsq(A,b,rcond=None)[0]
            cx,cy=s[0],s[1]; print(' rim centre',round(cx,4),round(cy,4),'r',round(np.sqrt(s[2]+cx**2+cy**2),4),'rim z',round(top[:,2].mean(),4),'std',round(top[:,2].std(),4),'n',len(top))
"

# openrua op 111
cd /workspace; timeout 300 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_in.png

# openrua op 112
cd /workspace; timeout 300 python3 cloudtf.py robot0_eye_in_hand eih >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/eih_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.15)&(P[:,0]<0.05)&(P[:,1]>0.27)&(P[:,1]<0.42)&(P[:,2]>0.945)&(P[:,2]<1.08)
Q=P[m]; print('pts in cavity',len(Q),'z',Q[:,2].min().round(4),Q[:,2].max().round(4),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'x',Q[:,0].min().round(3),Q[:,0].max().round(3))
# fit circle to the front wall points at a mid height
for z0 in (0.98,1.0,1.02):
    band=Q[np.abs(Q[:,2]-z0)<0.005]
    A=np.c_[2*band[:,0],2*band[:,1],np.ones(len(band))]; b=(band[:,:2]**2).sum(1); s=np.linalg.lstsq(A,b,rcond=None)[0]
    cx,cy=s[0],s[1]; print(z0,'centre',round(cx,4),round(cy,4),'r',round(np.sqrt(s[2]+cx**2+cy**2),4),'n',len(band))
top=Q[Q[:,2]>Q[:,2].max()-0.006]; print('top z',top[:,2].mean().round(4))
"

# openrua op 113
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
from rlib import *
r=Robot('away'); Rup=R_from_axes([0,1,0],[1,0,0])
go_tcp(r,[-0.066,0.10,1.25],Rup,n=3,rate=0.2)
go_tcp(r,[-0.10,-0.25,1.30],Rup,n=5,rate=0.2)
p,R=r.fk(); print('tcp',p+TCP*R[:,2])
"; timeout 300 python3 cloudtf.py birdview door1 >/dev/null; timeout 300 python3 tools/perception/cam_snap.py birdview snaps/bird_door1.png; python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
P=np.load('snaps/door1_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# door: points left of microwave body, above table, z>1.0 region x<-0.18
m=(P[:,0]>-0.45)&(P[:,0]<-0.17)&(P[:,1]>-0.15)&(P[:,1]<0.35)&(P[:,2]>1.05)&(P[:,2]<1.2)
Q=P[m]; print('door top pts',len(Q),'z',Q[:,2].min(),Q[:,2].max())
# PCA line fit
c=Q[:,:2].mean(0); u,s,vt=np.linalg.svd(Q[:,:2]-c); d=vt[0]; n=vt[1]
a=(Q[:,:2]-c)@d; b=(Q[:,:2]-c)@n
print('centre',c,'dir',d,'along range',a.min(),a.max(),'thick range',b.min(),b.max())
print('end A',c+a.min()*d,'end B',c+a.max()*d)
# microwave body top extents
m2=(P[:,2]>1.09)&(P[:,2]<1.13)&(P[:,0]>-0.2)&(P[:,0]<0.25)&(P[:,1]>0.15)&(P[:,1]<0.5)
B=P[m2]; print('mw top x',B[:,0].min(),B[:,0].max(),'y',B[:,1].min(),B[:,1].max(),'z',B[:,2].mean())
"

# openrua op 114
cd /workspace; python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
P=np.load('snaps/door1_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.45)&(P[:,0]<-0.17)&(P[:,1]>-0.15)&(P[:,1]<0.215)&(P[:,2]>0.93)
Q=P[m]; print('door pts',len(Q),'z',Q[:,2].min(),Q[:,2].max())
c=Q[:,:2].mean(0); u,s,vt=np.linalg.svd(Q[:,:2]-c); d=vt[0]; n=vt[1]
if d[1]<0: d=-d
a=(Q[:,:2]-c)@d; b=(Q[:,:2]-c)@n
print('centre',c,'dir',d,'along',a.min(),a.max(),'thick',b.min(),b.max())
print('tip',c+a.min()*d,'near end',c+a.max()*d)
print('angle deg',np.degrees(np.arctan2(d[1],d[0])))
# front face of microwave: points with z 0.95..1.08 near y 0.2-0.3, x -0.15..0.05 -> min y
m3=(P[:,0]>-0.19)&(P[:,0]<0.16)&(P[:,2]>0.95)&(P[:,2]<1.10)&(P[:,1]>0.15)&(P[:,1]<0.3)
F=P[m3]
for x0 in np.arange(-0.18,0.17,0.03):
    s=F[np.abs(F[:,0]-x0)<0.015]
    if len(s): print('x',round(x0,2),'front y min',s[:,1].min().round(3),'n',len(s))
"

# openrua op 115
cd /workspace; python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
P=np.load('snaps/door1_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.45)&(P[:,0]<-0.17)&(P[:,1]>-0.15)&(P[:,1]<0.215)&(P[:,2]>0.93)
Q=P[m]
h=np.array([-0.19,0.25]); d=np.array([np.cos(np.radians(-115.6)),np.sin(np.radians(-115.6))]); n=np.array([-d[1],d[0]])
print('n',n)
a=(Q[:,:2]-h)@d; b=(Q[:,:2]-h)@n
H,e=np.histogram(b,bins=np.arange(-0.06,0.05,0.005)); 
for i in range(len(H)): print(round(e[i],3),H[i], round(Q[(b>=e[i])&(b<e[i+1]),2].mean(),3) if H[i] else '')
print('along range',a.min(),a.max())
# where along the door are the b>0.02 points?
sel=b>0.02; print('handle-ish pts along',a[sel].min() if sel.any() else None,a[sel].max() if sel.any() else None,'z',Q[sel,2].min() if sel.any() else None,Q[sel,2].max() if sel.any() else None)
sel=b<-0.02; print('other side pts along',a[sel].min() if sel.any() else None,a[sel].max() if sel.any() else None,'z',Q[sel,2].min() if sel.any() else None,Q[sel,2].max() if sel.any() else None)
"

# openrua op 116
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('doorplan')
h=np.array([-0.19,0.25]); A=0.17; Z=1.0
def pose(th_deg,delta):
    th=np.radians(th_deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    tcp=np.r_[h+A*d-delta*n,Z]; R=R_from_axes([0,0,-1],[-n[1],n[0],0]); return tcp,R
tcp0,R0=pose(-118,0.10); print('start tcp',tcp0,'xh',R0[:,0])
q0=np.array(r.arm_q())
# hover above start
qh=r.ik(tcp0+[0,0,0.25]-TCP*R0[:,2],R0,seed=q0,tries=6); print('qh',qh)
if qh is not None:
    out=path_check(r,q0,qh,n=10,links=('panda_hand','panda_link5','panda_link7'))
    for s,d in out: print(round(s,1),' '.join(f'{k}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})' for k,v in d.items()))
    np.save('snaps/q_doorhover.npy',np.array(qh))
# arc feasibility from start
seed=qh; prev=None; ok=True
for th in np.arange(-118,0,5):
    tcp,R=pose(th,0.075); q=r.ik(tcp-TCP*R[:,2],R,seed=seed,tries=4,max_jump=0.5)
    if q is None: print('IK fail at',th); ok=False; break
    if prev is not None: print(th,'dq',np.round(np.array(q)-np.array(prev),2))
    prev=q; seed=q
print('arc ok',ok)
"

# openrua op 117
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('doorplan2')
h=np.array([-0.19,0.25]); Z=1.0
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def pose(th_deg,delta,A):
    th=np.radians(th_deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    tcp=np.r_[h+A*d-delta*n,Z]; R=R_from_axes([0,0,-1],[-n[1],n[0],0]); return tcp,R
rng=np.random.default_rng(0)
for A in (0.17,0.20):
    tcp,R=pose(-118,0.075,A); sols=[]
    for i in range(25):
        seed=list(rng.uniform(lim[:,0],lim[:,1]))
        q=r.ik(tcp-TCP*R[:,2],R,seed=seed,tries=1)
        if q is None: continue
        q=np.array(q); marg=np.minimum(q-lim[:,0],lim[:,1]-q).min()
        sols.append((marg,q))
    sols.sort(key=lambda s:-s[0])
    print('A',A,'nsol',len(sols))
    for m,q in sols[:4]: print('  margin',round(m,2),q)
" 2>&1 | grep -v '^ik:'

# openrua op 118
cd /workspace; timeout 1200 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('doorplan3')
h=np.array([-0.19,0.25]); Z=1.0; A=0.17
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def pose(th_deg,delta):
    th=np.radians(th_deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    tcp=np.r_[h+A*d-delta*n,Z]; R=R_from_axes([0,0,-1],[-n[1],n[0],0]); return tcp,R
starts=[[-0.985,-1.073,1.689,-2.742,1.467,2.072,-2.396],[-1.315,0.724,1.432,-2.758,-1.281,2.387,-0.613],[-0.425,0.011,0.812,-2.875,-0.03,2.882,-1.451],[2.151,1.032,-1.451,-2.743,1.449,2.109,-2.392]]
for s in starts:
    seed=s; path=[]; ok=True; minmarg=9
    for th in np.arange(-118,-1,4):
        tcp,R=pose(th,0.075); q=r.ik(tcp-TCP*R[:,2],R,seed=seed,tries=3,max_jump=0.45)
        if q is None: print('  fail at',th); ok=False; break
        q=np.array(q); minmarg=min(minmarg,np.minimum(q-lim[:,0],lim[:,1]-q).min()); path.append(q); seed=list(q)
    print('start',np.round(s,2),'ok',ok,'minmarg',round(minmarg,2),'end',path[-1] if path else None)
    if ok:
        dq=np.abs(np.diff(np.array(path),axis=0)).max(); print('  max step',round(dq,3))
        np.save('snaps/door_arc.npy',np.array(path)); break
" 2>&1 | grep -v '^ik:'

# openrua op 119
cd /workspace; timeout 1500 python3 -c "
import numpy as np; np.set_printoptions(precision=2,suppress=True)
from rlib import *
r=Robot('doorplan4')
h=np.array([-0.19,0.25]); Z=1.0
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def pose(th_deg,delta,A,mode):
    th=np.radians(th_deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    tcp=np.r_[h+A*d-delta*n,Z]
    if mode=='xface': R=R_from_axes([0,0,-1],[-n[1],n[0],0])   # x_h=-n
    elif mode=='ypos': R=R_from_axes([0,0,-1],[n[0],n[1],0])    # y_h=+n
    else: R=R_from_axes([0,0,-1],[-n[0],-n[1],0])               # y_h=-n
    return tcp,R
rng=np.random.default_rng(1)
def arc(mode,A,delta,th0,th1,step=4):
    tcp,R=pose(th0,delta,A,mode); sols=[]
    for i in range(30):
        q=r.ik(tcp-TCP*R[:,2],R,seed=list(rng.uniform(lim[:,0],lim[:,1])),tries=1)
        if q is not None:
            q=np.array(q); sols.append((np.minimum(q-lim[:,0],lim[:,1]-q).min(),q))
    sols.sort(key=lambda s:-s[0])
    for m,s in sols[:6]:
        seed=list(s); path=[]; ok=True; minmarg=9
        for th in np.arange(th0,th1+0.1,step):
            tcp,R=pose(th,delta,A,mode); q=r.ik(tcp-TCP*R[:,2],R,seed=seed,tries=3,max_jump=0.45)
            if q is None: ok=False; break
            q=np.array(q); minmarg=min(minmarg,np.minimum(q-lim[:,0],lim[:,1]-q).min()); path.append(q); seed=list(q)
        print(mode,A,th0,th1,'start marg',round(m,2),'ok',ok,'reached',th if not ok else th1,'minmarg',round(minmarg,2))
        if ok and minmarg>0.1: return np.array(path)
    return None
p1=arc('xface',0.10,0.045,-118,-50)
if p1 is not None: np.save('snaps/arc1.npy',p1); print('arc1 saved',p1[0],p1[-1])
for mode in ('ypos','yneg'):
    p2=arc(mode,0.12,0.117,-56,-1)
    if p2 is not None: np.save(f'snaps/arc2_{mode}.npy',p2); print('arc2',mode,'saved',p2[0],p2[-1])
" 2>&1 | grep -v '^ik:'

# openrua op 120
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=2,suppress=True)
from rlib import *
r=Robot('doorplan5')
for f in ('arc1','arc2_ypos','arc2_yneg'):
    P=np.load(f'snaps/{f}.npy'); print(f,len(P))
    for i in range(0,len(P),max(1,len(P)//4)) if True else []:
        d={}
        for L in ('panda_link3','panda_link4','panda_link5','panda_link7','panda_hand'):
            p,R=r.fk(list(P[i]),link=L); d[L]=p
        print('  ',i,' '.join(f'{k[-5:]}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})' for k,v in d.items()))
"

# openrua op 121
cd /workspace; timeout 1500 python3 -c "
import numpy as np; np.set_printoptions(precision=2,suppress=True)
from rlib import *
r=Robot('doorplan6')
h=np.array([-0.19,0.25]); Z=1.0
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def pose(th_deg,delta,A,mode):
    th=np.radians(th_deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    tcp=np.r_[h+A*d-delta*n,Z]
    if mode=='xface': R=R_from_axes([0,0,-1],[-n[1],n[0],0])
    elif mode=='ypos': R=R_from_axes([0,0,-1],[n[0],n[1],0])
    else: R=R_from_axes([0,0,-1],[-n[0],-n[1],0])
    return tcp,R
a1=np.load('snaps/arc1.npy'); qend=list(a1[-1]); print('arc1 end',np.array(qend))
for mode in ('ypos','yneg'):
    tcp,R=pose(-56,0.117,0.12,mode)
    # hover version first (lifted 0.15)
    for k in range(8):
        q=r.ik(tcp+[0,0,0.15]-TCP*R[:,2],R,seed=qend,tries=1)
        if q is None: continue
        q=np.array(q); marg=np.minimum(q-lim[:,0],lim[:,1]-q).min()
        print(mode,'hover sol',q,'marg',round(marg,2),'jump',np.abs(q-np.array(qend)).max().round(2))
        # continue arc from the descended pose
        seed=list(q); path=[]; ok=True; minmarg=9
        for th in np.arange(-56,0,4):
            t2,R2=pose(th,0.117,0.12,mode); qq=r.ik(t2-TCP*R2[:,2],R2,seed=seed,tries=3,max_jump=0.45)
            if qq is None: ok=False; break
            qq=np.array(qq); minmarg=min(minmarg,np.minimum(qq-lim[:,0],lim[:,1]-qq).min()); path.append(qq); seed=list(qq)
        print('   arc ok',ok,'minmarg',round(minmarg,2))
        if ok and minmarg>0.1:
            np.save(f'snaps/arc2b_{mode}.npy',np.array(path)); np.save(f'snaps/arc2b_{mode}_hover.npy',q); print('   saved'); break
" 2>&1 | grep -v '^ik:'

# openrua op 122
cd /workspace; timeout 1500 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('door_go1')
a1=np.load('snaps/arc1.npy'); q_start=list(a1[0])
p,R=r.fk(q_start); print('arc1 start hand',p,'tcp',p+TCP*R[:,2],'xh',R[:,0])
qh=r.ik(p+[0,0,0.15],R,seed=q_start,tries=4,max_jump=0.6); print('hover q',np.array(qh),'jump',np.abs(np.array(qh)-np.array(q_start)).max().round(2))
q0=np.array(r.arm_q())
out=path_check(r,q0,qh,n=12,links=('panda_hand','panda_link4','panda_link5','panda_link7'))
for s,d in out: print(round(s,2),' '.join(f'{k[-5:]}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})' for k,v in d.items()))
np.save('snaps/q_arc1_hover.npy',np.array(qh))
"

# openrua op 123
cd /workspace; timeout 1500 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *; import time
r=Robot('door_go2')
qh=np.load('snaps/q_arc1_hover.npy'); a1=np.load('snaps/arc1.npy')
r.gripper(0.0); time.sleep(0.5)
go_joint(r,qh,rate=0.2)
go_joint(r,a1[0],rate=0.15)
p,R=r.fk(); print('at arc1 start tcp',p+TCP*R[:,2],'xh',R[:,0])
steps=np.abs(np.diff(a1,axis=0)).max(1); T=max(6.0,steps.sum()/0.12); print('arc1 steps',steps.round(2),'T',round(T,1))
code,err=r.move([list(q) for q in a1[1:]],T); print('arc1 code',code,'err',round(err,4))
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'xh',R[:,0])
"; timeout 300 python3 tools/perception/cam_snap.py birdview snaps/bird_arc1.png; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_arc1.png

# openrua op 124
cd /workspace; timeout 300 python3 cloudtf.py birdview arc1 >/dev/null; python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
P=np.load('snaps/arc1_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
h=np.array([-0.19,0.25])
m=(P[:,2]>1.085)&(P[:,2]<1.115)&(P[:,1]<0.225)&(P[:,1]>-0.1)&(P[:,0]>-0.3)&(P[:,0]<0.1)
Q=P[m]; print('pts',len(Q))
rel=Q[:,:2]-h; ang=np.degrees(np.arctan2(rel[:,1],rel[:,0])); rad=np.linalg.norm(rel,axis=1)
H,e=np.histogram(ang,bins=np.arange(-130,10,4))
for i in range(len(H)):
    if H[i]: print(round(e[i]),H[i],'r range',rad[(ang>=e[i])&(ang<e[i+1])].min().round(3),rad[(ang>=e[i])&(ang<e[i+1])].max().round(3))
"

# openrua op 125
cd /workspace; timeout 1500 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('door_go3')
h=np.array([-0.19,0.25]); Z=1.0
def pose(th_deg,delta=0.117,A=0.12):
    th=np.radians(th_deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    return np.r_[h+A*d-delta*n,Z], R_from_axes([0,0,-1],[n[0],n[1],0])
qh=np.load('snaps/arc2b_ypos_hover.npy')
# lift straight up first
p,R=r.fk(); go_tcp(r,p+TCP*R[:,2]+[0,0,0.15],R,n=3,rate=0.2)
q0=np.array(r.arm_q())
# hover at -60
tcp,Rp=pose(-60); qh2=r.ik(tcp+[0,0,0.15]-TCP*Rp[:,2],Rp,seed=list(qh),tries=4,max_jump=0.6); print('hover60',np.array(qh2))
out=path_check(r,q0,qh2,n=8,links=('panda_hand','panda_link5','panda_link7'))
for s,d in out: print(round(s,2),' '.join(f'{k[-5:]}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})' for k,v in d.items()))
go_joint(r,qh2,rate=0.15)
qs=r.ik(tcp-TCP*Rp[:,2],Rp,seed=qh2,tries=4,max_jump=0.6)
go_joint(r,qs,rate=0.15)
p,R=r.fk(); print('at arc2 start tcp',p+TCP*R[:,2],'yh',R[:,1])
seed=qs; path=[]
for th in np.arange(-57,-2.9,3):
    t2,R2=pose(th); q=r.ik(t2-TCP*R2[:,2],R2,seed=seed,tries=3,max_jump=0.45); path.append(np.array(q)); seed=list(q)
path=np.array(path); np.save('snaps/arc2_exec.npy',path)
steps=np.abs(np.diff(np.vstack([qs,path]),axis=0)).max(1); T=max(6.0,steps.sum()/0.12); print('steps',steps.round(2),'T',round(T,1))
"

# openrua op 126
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('door_go4'); path=np.load('snaps/arc2_exec.npy')
code,err=r.move([list(q) for q in path],9.5); print('arc2 code',code,'err',round(err,4))
p,R=r.fk(); print('tcp',p+TCP*R[:,2],'yh',R[:,1])
"; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_arc2.png; timeout 300 python3 tools/perception/cam_snap.py birdview snaps/bird_arc2.png

# openrua op 127
cd /workspace; timeout 900 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('away2'); p,R=r.fk(); tcp=p+TCP*R[:,2]
go_tcp(r,tcp+[0,0,0.22],R,n=3,rate=0.2)
go_tcp(r,[-0.15,-0.20,1.25],R,n=5,rate=0.2)
p,R=r.fk(); print('tcp',p+TCP*R[:,2])
"; timeout 300 python3 cloudtf.py birdview final >/dev/null; timeout 300 python3 tools/perception/cam_snap.py birdview snaps/bird_final.png; timeout 300 python3 tools/perception/cam_snap.py frontview snaps/front_final.png; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_final.png; python3 -c "
import numpy as np; np.set_printoptions(precision=4,suppress=True)
P=np.load('snaps/final_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# anything left of the microwave / in front in the old door region?
m=(P[:,0]>-0.45)&(P[:,0]<-0.2)&(P[:,1]>-0.15)&(P[:,1]<0.22)&(P[:,2]>0.93)
print('pts in old open-door zone',m.sum())
# front face profile now
m3=(P[:,0]>-0.2)&(P[:,0]<0.17)&(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,1]>0.1)&(P[:,1]<0.3)
F=P[m3]
for x0 in np.arange(-0.18,0.17,0.03):
    s=F[np.abs(F[:,0]-x0)<0.015]
    if len(s): print('x',round(x0,2),'front y min',s[:,1].min().round(3),'z at min',s[np.argmin(s[:,1]),2].round(3),'n',len(s))
"

# openrua op 128
cd /workspace; timeout 1500 python3 -c "
import numpy as np; np.set_printoptions(precision=3,suppress=True)
from rlib import *
r=Robot('door_go5')
h=np.array([-0.19,0.25]); Z=1.0
def pose(th_deg,delta=0.117,A=0.12):
    th=np.radians(th_deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    return np.r_[h+A*d-delta*n,Z], R_from_axes([0,0,-1],[n[0],n[1],0])
path=np.load('snaps/arc2_exec.npy'); qend=list(path[-1])
tcp,Rp=pose(-4)
qh=r.ik(tcp+[0,0,0.20]-TCP*Rp[:,2],Rp,seed=qend,tries=4,max_jump=0.6)
q0=np.array(r.arm_q()); out=path_check(r,q0,qh,n=8,links=('panda_hand','panda_link5','panda_link7'))
zmin=min(min(v[2] for v in d.values()) for s,d in out); print('hover',np.array(qh),'zmin',round(zmin,3))
go_joint(r,qh,rate=0.15)
qs=r.ik(tcp-TCP*Rp[:,2],Rp,seed=qh,tries=4,max_jump=0.6); go_joint(r,qs,rate=0.15)
p,R=r.fk(); print('start tcp',p+TCP*R[:,2],'yh',R[:,1])
seed=qs; wps=[]
for th in (-3,-2,-1,0):
    t2,R2=pose(th); q=r.ik(t2-TCP*R2[:,2],R2,seed=seed,tries=3,max_jump=0.3); wps.append(list(q)); seed=q
code,err=r.move(wps,4.0); print('nudge code',code,'err',round(err,4))
p,R=r.fk(); tcp=p+TCP*R[:,2]; print('end tcp',tcp)
go_tcp(r,tcp+[0,0,0.22],R,n=3,rate=0.2)
go_tcp(r,[-0.15,-0.20,1.25],R,n=5,rate=0.2)
"; timeout 300 python3 cloudtf.py birdview final2 >/dev/null; python3 -c "
import numpy as np
P=np.load('snaps/final2_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m3=(P[:,0]>-0.2)&(P[:,0]<0.17)&(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,1]>0.1)&(P[:,1]<0.3)
F=P[m3]
for x0 in np.arange(-0.18,0.17,0.03):
    s=F[np.abs(F[:,0]-x0)<0.015]
    if len(s): print('x',round(x0,2),'front y min',s[:,1].min().round(3),'n',len(s))
"

# openrua op 129
cd /workspace; timeout 300 python3 tools/perception/cam_snap.py agentview snaps/agent_done.png; timeout 300 python3 -c "
from rlib import *; import numpy as np, time; np.set_printoptions(precision=3,suppress=True)
r=Robot('done'); q1=np.array(r.arm_q()); time.sleep(1.0); q2=np.array(r.arm_q())
print('arm still:',np.abs(q1-q2).max()<1e-3,'q',q2,'fingers',r.fingers())"
