#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -iv parameter && echo --- && ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 6
timeout 30 ros2 topic echo /tf --once 2>&1 | head -200

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a world-frame point cloud from a camera's depth + color and report
blobs standing above the table. Usage: python3 scene.py <camera>"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    z = depth
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
    pw = pc @ R.T + p0
    np.save(f"{cam}_pw.npy", pw)
    np.save(f"{cam}_color.npy", color)
    print("depth range", np.nanmin(z), np.nanmax(z))
    # table height estimate: mode of z in the central region
    zs = pw[..., 2]
    finite = zs[np.isfinite(zs)]
    hist, edges = np.histogram(finite, bins=200)
    table_z = edges[np.argmax(hist)]
    print("dominant z (table?)", table_z)
    mask = (zs > table_z + 0.015) & (zs < table_z + 0.5) & np.isfinite(zs)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        m = lab == i
        pts = pw[m]
        col = color[m].mean(0)
        x0, y0, w, h = stats[i, :4]
        print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i, 4]} "
              f"bbox=({x0},{y0},{w},{h}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
              f"y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} "
              f"center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scene.py birdview

# openrua op 9
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); color=np.load('birdview_color.npy')
zs=pw[...,2]
sel=zs[(zs>0.3)&(zs<0.6)]
hist,edges=np.histogram(sel,bins=300); tz=edges[np.argmax(hist)]; print('table z',tz)
mask=(zs>tz+0.01)&(zs<tz+0.5)
n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,4]<5: continue
    m=lab==i; pts=pw[m]; col=color[m].mean(0)
    print(f'blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i,4]} bbox={stats[i,:4]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}')
"


# openrua op 10
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); color=np.load('birdview_color.npy')
zs=pw[...,2]
sub=zs[293:318,270:302]
np.set_printoptions(linewidth=250,precision=2)
print((sub-0.425).round(2))
big=cv2.resize(color[280:330,255:320],None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST); cv2.imwrite('crop.png',big)
"


# openrua op 11
python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy')
box=pw[296:315,275:285]; print('box x',box[...,0].min(),box[...,0].max(),'y',box[...,1].min(),box[...,1].max(),'center',box[...,0].mean(),box[...,1].mean(),'ztop',box[...,2].max())
k=pw[296:307,285:299]; print('ketchup center',k[...,0].mean(),k[...,1].mean())
# alphabet can blob 6 region
zs=pw[...,2]; m=np.zeros_like(zs,bool); m[229:246,274:292]=True; m&=(zs>0.44)
c=pw[m]; print('can x',c[:,0].min(),c[:,0].max(),'y',c[:,1].min(),c[:,1].max(),'center',c[:,0].mean(),c[:,1].mean(),'ztop',c[:,2].max())
# basket
m=np.zeros_like(zs,bool); m[268:308,362:405]=True; m&=(zs>0.5)
b=pw[m]; print('basket x',b[:,0].min(),b[:,0].max(),'y',b[:,1].min(),b[:,1].max(),'center',b[:,0].mean(),b[:,1].mean(),'ztop',b[:,2].max())
# basket interior floor
m=np.zeros_like(zs,bool); m[268:308,362:405]=True; m&=(zs<0.5)
b=pw[m]; print('basket floor z', b[:,2].min(), b[:,2].max(), 'n',len(b))
"


# openrua op 12
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 13
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable helpers: FK/IK in world frame, trajectory, gripper, joint read.
World <-> base: base (panda_link0) sits at world (-0.51, 0, 0.42), no rotation.
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([-0.51, 0.0, 0.42])
TCP = M["hand"]["tcp_offset_m"]
# hand pointing down, fingers closing along world Y
Q_FY = (0.0, 1.0, 0.0, 0.0)
# hand pointing down, fingers closing along world X
Q_FX = (0.70710678, 0.70710678, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_quat(yaw):
    """Hand down, fingers closing along world direction rotated `yaw` from +Y."""
    # R = Rz(yaw) @ Ry(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # q = qz(yaw) * qy(pi) ; qy(pi) = (0,1,0,0)
    # (w1,v1)*(w2,v2) = (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w1, v1 = c, np.array([0, 0, s]); w2, v2 = 0.0, np.array([0, 1.0, 0])
    w = w1 * w2 - v1 @ v2
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

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
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def fk(self, q=None):
        """World-frame hand pose (pos, quat) and TCP pos."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = q if q is not None else self.arm_q()
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q4 = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_R(*q4)[:, 2]
        return pos, q4, tcp

    def ik(self, tcp_xyz, quat, seed=None, tries=5):
        """IK for a world-frame TCP position + hand orientation -> joint list."""
        R = quat_R(*quat)
        hand = np.array(tcp_xyz, float) - TCP * R[:, 2] - BASE
        seed = seed if seed is not None else self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = hand
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = list(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            print("IK attempt failed", None if res is None else res.error_code.val)
        return None

    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_xyz, quat, seconds=3.0, seed=None):
        q = self.ik(tcp_xyz, quat, seed=seed)
        if q is None:
            print("  IK FAILED for", tcp_xyz)
            return None
        code, err = self.move_q(q, seconds)
        pos, _, tcp = self.fk()
        print(f"  tcp now {tcp.round(4)} (target {np.round(tcp_xyz,4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from robot import *
r=Robot()
print('q',np.round(r.arm_q(),3))
pos,q,tcp=r.fk(); print('hand',pos.round(4),np.round(q,4),'tcp',tcp.round(4))
print('fingers',r.fingers())
# test IK for the can pregrasp
print(np.round(r.ik((-0.215,-0.166,0.60),Q_FY),3))
print(np.round(r.ik((-0.215,-0.166,0.60),Q_FX),3))
"

# openrua op 15
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png; python3 -c "
import numpy as np; d=np.load('eih_depth.npy'); print(d.shape, d[240,320], d[100,320], d[400,320], np.nanmin(d), np.nanmax(d))"; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 '^k:'

# openrua op 16
timeout 60 python3 -c "
import numpy as np
def rz(t): c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
def rx(t): c,s=np.cos(t),np.sin(t); return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])
def tr(x,y,z): T=np.eye(4); T[:3,3]=[x,y,z]; return T
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
# from URDF: joint origins (rpy, xyz) then rotate about z
J=[((0,0,0),(0,0,0.333)),((-1.5708,0,0),(0,0,0)),((1.5708,0,0),(0,-0.316,0)),((1.5708,0,0),(0.0825,0,0)),((-1.5708,0,0),(-0.0825,0.384,0)),((1.5708,0,0),(0,0,0)),((1.5708,0,0),(0.088,0,0))]
T=np.eye(4)
for (rpy,xyz),qi in zip(J,q):
    T=T@tr(*xyz)@rx(rpy[0])@rz(qi)
T=T@tr(0,0,0.107)@rz(-0.785398)
print('hand in base',T[:3,3].round(4)); print(T[:3,:3].round(3))
"


# openrua op 17
sed -i 's/^BASE = np.array(\[-0.51, 0.0, 0.42\])/BASE = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world (verified by FK vs camera TF)/' robot.py && timeout 120 python3 -c "
from robot import *
r=Robot()
pos,q,tcp=r.fk(); print('hand',pos.round(4),np.round(q,4),'tcp',tcp.round(4))
sol=r.ik(tcp,q); print('ik of current pose',np.round(sol,3),'current',np.round(r.arm_q(),3))
print('can pregrasp FY',np.round(r.ik((-0.215,-0.166,0.60),Q_FY),3))
print('can pregrasp FX',np.round(r.ik((-0.215,-0.166,0.60),Q_FX),3))
print('box FY',np.round(r.ik((0.086,-0.178,0.437),Q_FY),3))
print('basket FY',np.round(r.ik((0.0,0.265,0.70),Q_FY),3))
"

# openrua op 18
cat >> robot.py <<'EOF'


READY = [0.0, -0.5, 0.0, -2.2, 0.0, 1.7, 0.785]


def good_ik(r, tcp_xyz, yaws=(0.0, np.pi, np.pi / 2, -np.pi / 2), fixed_quat=None):
    """Try several yaws/seeds; return (q, quat) closest to the current config
    with a sane base-joint azimuth."""
    cur = np.array(r.arm_q())
    az = np.arctan2(tcp_xyz[1] - 0.0, tcp_xyz[0] + 0.51)
    seeds = [list(cur), READY[:1] and [az] + READY[1:]]
    quats = [fixed_quat] if fixed_quat is not None else [yaw_quat(y) for y in yaws]
    best = None
    for quat in quats:
        for s in seeds:
            q = r.ik(tcp_xyz, quat, seed=s, tries=2)
            if q is None:
                continue
            q = np.array(q)
            if abs(q[0] - az) > 1.2:  # avoid wrapped-around base solutions
                continue
            d = np.abs(q - cur).sum()
            if best is None or d < best[0]:
                best = (d, q, quat)
    return (best[1], best[2]) if best else (None, None)
EOF
timeout 200 python3 -c "
from robot import *
r=Robot()
for name,p,fq in [('can',(-0.215,-0.166,0.60),None),('box',(0.086,-0.178,0.437),Q_FY),('box2',(0.086,-0.178,0.437),(1.0,0,0,0)),('basket',(0.0,0.265,0.70),None),('ketchup',(0.062,-0.1225,0.52),Q_FX),('park',(-0.05,-0.32,0.53),Q_FX)]:
    q,quat=good_ik(r,p,fixed_quat=fq); print(name, None if q is None else np.round(q,3), quat and np.round(quat,3))
    if q is not None: print('   fk check', r.fk(list(q))[2].round(4))
"

# openrua op 19
python3 - <<'EOF'
import re
s=open('robot.py').read()
s=s.replace("    return (v[0], v[1], v[2], w)","    return (float(v[0]), float(v[1]), float(v[2]), float(w))")
s=s.replace("            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat","            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(c) for c in quat]")
s=s.replace("            req.ik_request.robot_state.joint_state.position = list(seed)","            req.ik_request.robot_state.joint_state.position = [float(c) for c in seed]")
s=s.replace("        req.robot_state.joint_state.position = q if q is not None else self.arm_q()","        req.robot_state.joint_state.position = [float(c) for c in (q if q is not None else self.arm_q())]")
s=s.replace("    seeds = [list(cur), READY[:1] and [az] + READY[1:]]","    seeds = [list(cur), [az] + READY[1:]]")
open('robot.py','w').write(s)
EOF
timeout 300 python3 -c "
from robot import *
r=Robot()
for name,p,fq in [('can',(-0.215,-0.166,0.60),None),('box',(0.086,-0.178,0.437),Q_FY),('box2',(0.086,-0.178,0.437),(1.0,0,0,0)),('basket',(0.0,0.265,0.70),None),('ketchup',(0.062,-0.1225,0.52),Q_FX),('park',(-0.05,-0.32,0.53),Q_FX)]:
    q,quat=good_ik(r,p,fixed_quat=fq); print(name, None if q is None else np.round(q,3), quat and np.round(quat,3))
    if q is not None: print('   fk check', r.fk(list(q))[2].round(4))
"

# openrua op 20
timeout 600 python3 -u -c "
from robot import *
r=Robot()
r.gripper(0.04)
q,quat=good_ik(r,(-0.215,-0.166,0.60)); print('quat',np.round(quat,3))
r.move_q(q,4.0)
print('tcp',r.fk()[2].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 21
timeout 600 python3 -u -c "
from robot import *
r=Robot()
quat=(-0.707,0.707,0.0,0.0)
q,_=good_ik(r,(-0.215,-0.166,0.445),fixed_quat=quat); r.move_q(q,2.5)
print('tcp',r.fk()[2].round(4))
r.gripper(0.0)
q,_=good_ik(r,(-0.215,-0.166,0.65),fixed_quat=quat); r.move_q(q,2.5)
print('tcp',r.fk()[2].round(4)); print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 22
python3 - <<'EOF'
s=open('robot.py').read()
s=s.replace("""        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err""","""        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        if err > 0.01 and retries > 0:
            print("  re-sending goal to converge")
            return self.move_q(q, max(2.0, seconds / 2), retries=retries - 1)
        return code, err""")
s=s.replace("    def move_q(self, q, seconds=3.0, via=None):","    def move_q(self, q, seconds=3.0, via=None, retries=2):")
open('robot.py','w').write(s)
EOF
timeout 900 python3 -u -c "
from robot import *
r=Robot()
q,quat=good_ik(r,(0.0,0.265,0.72)); print('quat',np.round(quat,3),'q',np.round(q,3))
r.move_q(q,4.0)
print('tcp',r.fk()[2].round(4)); print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 23
timeout 900 python3 -u -c "
from robot import *
r=Robot()
q,_=good_ik(r,(0.0,0.265,0.69),fixed_quat=(-1.0,0.0,0.0,0.0)); r.move_q(q,2.0)
print('tcp',r.fk()[2].round(4))
r.gripper(0.04)
q,_=good_ik(r,(0.0,0.265,0.80),fixed_quat=(-1.0,0.0,0.0,0.0)); r.move_q(q,2.5)
print('tcp',r.fk()[2].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
timeout 120 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); zs=pw[...,2]
m=(zs>0.50)&(zs<0.60); m[:, :260]=False; m[:, 320:]=False; m[:280,:]=False
p=pw[m]; print('ketchup upper part n',len(p),'center',p[:,0].mean().round(4),p[:,1].mean().round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
m=(zs>0.44)&(zs<0.47); m[:, :260]=False; m[:, 320:]=False; m[:280,:]=False
p=pw[m]; print('box n',len(p),'center',p[:,0].mean().round(4),p[:,1].mean().round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
"

# openrua op 25
timeout 900 python3 -u -c "
from robot import *
r=Robot()
P=(0.060,-0.123)
q,_=good_ik(r,(P[0],P[1],0.68),fixed_quat=Q_FX); r.move_q(q,4.0); print('tcp',r.fk()[2].round(4))
q,_=good_ik(r,(P[0],P[1],0.53),fixed_quat=Q_FX); r.move_q(q,2.5); print('tcp',r.fk()[2].round(4))
f=r.gripper(0.0)
if f[0] < 0.004: raise SystemExit('closed on air')
q,_=good_ik(r,(P[0],P[1],0.72),fixed_quat=Q_FX); r.move_q(q,2.5); print('tcp',r.fk()[2].round(4)); print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 26
timeout 900 python3 -u -c "
from robot import *
r=Robot()
P=(-0.08,-0.34)
q,_=good_ik(r,(P[0],P[1],0.72),fixed_quat=Q_FX); r.move_q(q,4.0); print('tcp',r.fk()[2].round(4))
q,_=good_ik(r,(P[0],P[1],0.56),fixed_quat=Q_FX); r.move_q(q,2.5); print('tcp',r.fk()[2].round(4))
r.gripper(0.04)
q,_=good_ik(r,(P[0],P[1],0.75),fixed_quat=Q_FX); r.move_q(q,2.5); print('tcp',r.fk()[2].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 27
timeout 120 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); zs=pw[...,2]
m=(zs>0.44)&(zs<0.47)&(pw[...,0]>0.0)&(pw[...,0]<0.2)&(pw[...,1]<-0.1)&(pw[...,1]>-0.3)
p=pw[m]; print('box n',len(p),'center',p[:,0].mean().round(4),p[:,1].mean().round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
"

# openrua op 28
timeout 900 python3 -u -c "
from robot import *
r=Robot()
P=(0.0856,-0.1779)
q,quat=good_ik(r,(P[0],P[1],0.60),yaws=(0.0,np.pi)); print('quat',np.round(quat,3)); r.move_q(q,4.0); print('tcp',r.fk()[2].round(4))
q,_=good_ik(r,(P[0],P[1],0.437),fixed_quat=quat); r.move_q(q,2.5); print('tcp',r.fk()[2].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 29
timeout 900 python3 -u -c "
from robot import *
r=Robot()
P=(0.0856,-0.1779)
f=r.gripper(0.0)
if f[0] < 0.004: raise SystemExit('closed on air')
q,_=good_ik(r,(P[0],P[1],0.65),fixed_quat=Q_FY); r.move_q(q,2.5); print('tcp',r.fk()[2].round(4)); print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
timeout 900 python3 -u -c "
from robot import *
r=Robot()
B=(0.0,0.265)
q,quat=good_ik(r,(B[0],B[1],0.72),yaws=(0.0,np.pi)); print('quat',np.round(quat,3)); r.move_q(q,4.0); print('tcp',r.fk()[2].round(4)); print('fingers',r.fingers())
q,_=good_ik(r,(B[0],B[1],0.69),fixed_quat=quat); r.move_q(q,2.0); print('tcp',r.fk()[2].round(4))
r.gripper(0.04)
q,_=good_ik(r,(B[0],B[1],0.82),fixed_quat=quat); r.move_q(q,2.5); print('tcp',r.fk()[2].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 31
timeout 120 python3 scene.py birdview | grep -v "blob 1:" && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 32
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); color=np.load('birdview_color.npy'); zs=pw[...,2]
tz=0.425
mask=(zs>tz+0.01)&(zs<tz+0.5)
n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,4]<5: continue
    m=lab==i; pts=pw[m]; col=color[m].mean(0)
    print(f'blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i,4]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}')
"

# openrua op 33
timeout 300 python3 -u -c "
from robot import *
r=Robot()
cur=np.array(r.arm_q()); print('cur',cur.round(3)); print('fk cur',r.fk()[2].round(4))
q,quat=good_ik(r,(0.0,0.265,0.82),yaws=(0.0,np.pi)); print('ik ',np.round(q,3),np.round(quat,3)); print('fk ik',r.fk(list(q))[2].round(4))
q,quat=good_ik(r,(0.0,0.265,0.72),yaws=(0.0,np.pi)); print('ik72',np.round(q,3),np.round(quat,3))
print('fingers',r.fingers())
"

# openrua op 34
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); color=np.load('birdview_color.npy'); zs=pw[...,2]
m=(zs>0.44)&(zs<0.47)&(pw[...,0]>-0.5)&(pw[...,0]<-0.25)&(pw[...,1]>0.45)&(pw[...,1]<0.75)
p=pw[m][:,:2]; c=p.mean(0); print('n',len(p),'center',c.round(4))
u,s,vt=np.linalg.svd(p-c); ax=vt[0]; print('long axis',ax.round(3),'angle from +x (deg)',np.degrees(np.arctan2(ax[1],ax[0])).round(1))
proj=(p-c)@vt.T; print('extent long',proj[:,0].ptp().round(3),'short',proj[:,1].ptp().round(3))
big=cv2.resize(color[190:215,440:470],None,fx=10,fy=10,interpolation=cv2.INTER_NEAREST); cv2.imwrite('crop.png',big)
"

# openrua op 35
timeout 900 python3 -u -c "
from robot import *
r=Robot()
P=(-0.368,0.594)
q,quat=good_ik(r,(P[0],P[1],0.60),yaws=(np.radians(-114),np.radians(66))); print('quat',np.round(quat,3),'q',np.round(q,3))
code,err=r.move_q(q,6.0); print('tcp',r.fk()[2].round(4))
assert err<0.01
q,_=good_ik(r,(P[0],P[1],0.437),fixed_quat=quat); code,err=r.move_q(q,3.0); print('tcp',r.fk()[2].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 36
timeout 300 python3 -u -c "
from robot import *
r=Robot()
pos,q,tcp=r.fk(); print('hand quat',np.round(q,3)); R=quat_R(*q); print('finger axis (hand y) in world',R[:,1].round(3),'angle',np.degrees(np.arctan2(R[1,1],R[0,1])).round(1)); print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py birdview && python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('crop.png', cv2.resize(im[170:240,410:500],None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))"

# openrua op 37
python3 - <<'EOF'
s=open('robot.py').read()
s=s.replace('''def yaw_quat(yaw):''','''def qmul(a, b):
    """Hamilton product of (x,y,z,w) quaternions."""
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


# MoveIt's panda_arm tip is panda_link8; panda_hand = link8 * Rz(-45deg).
# So a desired HAND orientation must be sent as link8 = hand * Rz(+45deg).
Q_HAND_TO_LINK8 = (0.0, 0.0, float(np.sin(np.pi / 8)), float(np.cos(np.pi / 8)))


def yaw_quat(yaw):''')
s=s.replace('''        R = quat_R(*quat)
        hand = np.array(tcp_xyz, float) - TCP * R[:, 2] - BASE
        seed = seed if seed is not None else self.arm_q()''','''        R = quat_R(*quat)
        hand = np.array(tcp_xyz, float) - TCP * R[:, 2] - BASE
        seed = seed if seed is not None else self.arm_q()
        quat = qmul(quat, Q_HAND_TO_LINK8)  # hand orientation -> link8 orientation''')
open('robot.py','w').write(s)
EOF
timeout 300 python3 -u -c "
from robot import *
r=Robot()
pos,q,tcp=r.fk(); print('hand quat',np.round(q,3),'q',np.round(r.arm_q(),3))
sol=r.ik(tcp,q); print('ik(current hand pose)',np.round(sol,3))
sol=r.ik(tcp,yaw_quat(np.radians(-114))); print('ik yaw -114',np.round(sol,3), 'j7 delta', round(sol[6]-r.arm_q()[6],3))
"

# openrua op 38
timeout 900 python3 -u -c "
from robot import *
r=Robot()
P=(-0.368,0.594)
quat=yaw_quat(np.radians(-114))
q,_=good_ik(r,(P[0],P[1],0.53),fixed_quat=quat); code,err=r.move_q(q,4.0); print('tcp',r.fk()[2].round(4)); assert err<0.01
q,_=good_ik(r,(P[0],P[1],0.437),fixed_quat=quat); code,err=r.move_q(q,3.0); print('tcp',r.fk()[2].round(4))
pos,qh,tcp=r.fk(); R=quat_R(*qh); print('finger axis angle',np.degrees(np.arctan2(R[1,1],R[0,1])).round(1))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 39
timeout 900 python3 -u -c "
from robot import *
r=Robot()
P=(-0.368,0.594)
quat=yaw_quat(np.radians(-114))
f=r.gripper(0.0)
if f[0] < 0.004: raise SystemExit('closed on air')
q,_=good_ik(r,(P[0],P[1],0.70),fixed_quat=quat); code,err=r.move_q(q,3.0); print('tcp',r.fk()[2].round(4)); print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 40
timeout 900 python3 -u -c "
from robot import *
r=Robot()
B=(-0.052,0.265)
q,_=good_ik(r,(B[0],B[1],0.74),fixed_quat=Q_FX); code,err=r.move_q(q,6.0); print('tcp',r.fk()[2].round(4)); assert err<0.01
pos,qh,tcp=r.fk(); R=quat_R(*qh); print('finger axis angle',np.degrees(np.arctan2(R[1,1],R[0,1])).round(1)); print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 41
timeout 900 python3 -u -c "
from robot import *
r=Robot()
B=(-0.052,0.265)
q,_=good_ik(r,(B[0],B[1],0.685),fixed_quat=Q_FX); code,err=r.move_q(q,3.0); print('tcp',r.fk()[2].round(4)); assert err<0.01
print('fingers before release',r.fingers())
r.gripper(0.04)
q,_=good_ik(r,(B[0],B[1],0.85),fixed_quat=Q_FX); code,err=r.move_q(q,3.0); print('tcp',r.fk()[2].round(4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 42
timeout 120 python3 scene.py birdview >/dev/null; python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); zs=pw[...,2]
# basket footprint x[-0.085,0.071] y[0.172,0.341]
m=(pw[...,0]>-0.085)&(pw[...,0]<0.071)&(pw[...,1]>0.172)&(pw[...,1]<0.341)&np.isfinite(zs)
print('basket region max z',zs[m].max().round(3))
# anything box-height on the table outside basket, near where the box was?
m2=(zs>0.44)&(zs<0.47)&(pw[...,0]>-0.5)&(pw[...,0]<-0.25)&(pw[...,1]>0.45)&(pw[...,1]<0.75)
print('pixels at old box spot',m2.sum())
m3=(zs>0.44)&(zs<0.47)&(pw[...,0]>0.0)&(pw[...,0]<0.2)&(pw[...,1]<-0.1)&(pw[...,1]>-0.3)
print('pixels at original box spot',m3.sum())
m4=(zs>0.49)&(zs<0.51)&(pw[...,0]>-0.26)&(pw[...,0]<-0.17)&(pw[...,1]>-0.21)&(pw[...,1]<-0.12)
print('pixels at original can spot',m4.sum())
"; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 43
timeout 600 python3 -u -c "
from robot import *
r=Robot()
q,_=good_ik(r,(-0.2,0.0,0.75),fixed_quat=Q_FY); r.move_q(q,4.0); print('tcp',r.fk()[2].round(4))
" && timeout 120 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); zs=pw[...,2]; color=np.load('birdview_color.npy')
m=(pw[...,0]>-0.085)&(pw[...,0]<0.071)&(pw[...,1]>0.172)&(pw[...,1]<0.341)&np.isfinite(zs)
print('basket region max z',zs[m].max().round(3),'(rim 0.627)')
inner=m&(pw[...,0]>-0.07)&(pw[...,0]<0.055)&(pw[...,1]>0.19)&(pw[...,1]<0.325)
zi=zs[inner]; print('inner floor z range',zi.min().round(3),zi.max().round(3))
# objects standing above table anywhere outside the basket and away from the robot
out=(zs>0.44)&(zs<0.62)&~m&(pw[...,0]>-0.45)
n,lab,st,ce=cv2.connectedComponentsWithStats(out.astype(np.uint8),8)
for i in range(1,n):
    if st[i,4]<5: continue
    p=pw[lab==i]; print(f'outside-basket blob: center=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) ztop={p[:,2].max():.3f} area={st[i,4]}')
big=cv2.resize(color[255:320,350:420],None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST); cv2.imwrite('crop.png',big)
"

# openrua op 44
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-moveit-frame-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-moveit-frame-quirks.md <<'OPENRUA_EOF'
---
name: panda-moveit-frame-quirks
description: Verified MoveIt IK/FK frame facts on the Panda workstation that contradict or extend the workspace docs (world-frame poses, link8 vs hand 45° yaw offset, -5 tolerance retries)
metadata:
  type: project
---

On the Franka Panda workstation in /workspace (verified 2026-09-12):
- `/compute_ik` and `/compute_fk` with empty `frame_id` return/accept WORLD-frame
  poses (panda_link0 is at world (-0.51, 0, 0.42)); the doc's "model frame is the
  arm base" claim is wrong here — FK of the hand matched the eye-in-hand camera TF only in world.
- The `panda_arm` group tip is `panda_link8`, not `panda_hand`: a commanded hand
  orientation must be right-multiplied by Rz(+45°) or the fingers land 45° off
  (irrelevant for cylinders, fatal for boxes).
- FollowJointTrajectory often returns -5 (tolerance violated) on big joint-7
  rotations; re-sending the same goal converges. Large aborted motions while
  holding an object can fling it; verify joint convergence before descending/releasing.
- Hand is ~20 cm wide along the finger axis: it cannot enter a ~15 cm basket, so
  release from above the rim (palm bottom = TCP - 0.045 m) and drop beside, not onto, other contents.

**Why:** these cost several iterations and one flung object.
**How to apply:** /workspace/robot.py already encodes all of this (`Q_HAND_TO_LINK8`, `move_q` retries, `good_ik`).
OPENRUA_EOF

# openrua op 45
cd /claude-config/projects/-workspace/memory && ls && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda MoveIt frame quirks](panda-moveit-frame-quirks.md) — IK/FK are world-frame; group tip is link8 (45° yaw offset from hand); -5 results need re-sends" >> MEMORY.md && cat MEMORY.md
