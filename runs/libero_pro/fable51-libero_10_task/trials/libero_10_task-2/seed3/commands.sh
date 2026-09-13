#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo "--- actions"; timeout 30 ros2 action list; echo "--- services"; timeout 30 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo "--- nodes"; timeout 30 ros2 node list

# openrua op 3
timeout 20 ros2 topic info /pipeline_state -v 2>&1 | head -30; echo ---; timeout 20 ros2 topic echo /pipeline_state --once 2>&1 | head -40; echo ---; timeout 20 ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 4
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq -c; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id"; echo ---; timeout 20 ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera once, then print world coords for
many pixels. Usage: scene.py <camera> u,v [u,v ...]  (also dumps world->panda_link0)
"""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def tf_mat(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4); T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    np.save(f"{cam}_depth.npy", d)
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    T = tf_mat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    if buf.can_transform("world", "panda_link0", rclpy.time.Time()):
        Tb = tf_mat(buf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
        print("world->panda_link0:", np.round(Tb[:3, 3], 4), "R diag", np.round(np.diag(Tb[:3, :3]), 3))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print("cam pos", np.round(T[:3, 3], 3))
    for (u, v) in pix:
        z = d[v, u]
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"({u},{v}) depth={z:.4f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 90 python3 scene.py birdview 243,272 295,268 375,278 378,238 320,305 200,350 450,350 320,200 320,240

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
z=3.0-d  # camera at z=3 looking down
fx=579.4112549695428
# world x = -0.2 + (v-240)*d/fx ; y = (u-320)*d/fx
h=z-0.9
mask=h>0.005
# restrict to table region roughly rows 160..400, cols 160..480, excluding robot (rows<230 & near col 320)
import scipy.ndimage as ndi
m=mask.copy()
m[:230,:]=False  # skip robot arm region roughly
lab,n=ndi.label(m)
for i in range(1,n+1):
    ys,xs=np.where(lab==i)
    if len(ys)<20: continue
    zz=h[lab==i]
    print(i,'px',len(ys),'u',xs.min(),xs.max(),'v',ys.min(),ys.max(),'hmax',zz.max().round(3),'hmed',np.median(zz).round(3))
    # world extents
    dd=d[lab==i]
    wx=-0.2+(ys-240)*dd/fx; wy=(xs-320)*dd/fx
    print('   world x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',wx.mean().round(3),wy.mean().round(3))
"


# openrua op 9
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); h=3.0-d-0.9
np.set_printoptions(linewidth=250,precision=3,suppress=True)
print('row 272 cols 210..320:'); print(h[272,210:320:2])
print('col 243 rows 240..310:'); print(h[240:310:2,243])
print('knob col 372 rows 225..255:'); print(h[225:255,372])
print('knob row 240 cols 355..390:'); print(h[240,355:390])
"


# openrua op 10
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 11
timeout 90 python3 scene.py birdview 243,272 && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); h=3.0-d-0.9
np.set_printoptions(linewidth=250,precision=3,suppress=True)
print('row 272 cols 210..330:'); print(h[272,210:330:2])
print('handle rows 255..285 col 300:'); print(h[255:285,300])
print('handle rows 255..285 col 290:'); print(h[255:285,290])
print('handle row 268 cols 270..325:'); print(h[268,270:325])
"

# openrua op 12
python3 -c "
import numpy as np, scipy.ndimage as ndi
d=np.load('birdview_depth.npy'); h=3.0-d-0.9
fx=579.4112549695428
def W(v,u,dd): return -0.2+(v-240)*dd/fx, (u-320)*dd/fx
# pan body: rim pixels with h>0.03 in cols 210..285
m=(h>0.03); m[:,285:]=False; m[:245,:]=False; m[310:,:]=False
ys,xs=np.where(m); dd=d[m]
wx,wy=W(ys,xs,dd)
print('pan rim x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',((wx.min()+wx.max())/2).round(3),((wy.min()+wy.max())/2).round(3), 'rim h', h[m].max().round(3))
# handle
m=(h>0.04); m[:, :283]=False; m[:,320:]=False; m[:260,:]=False; m[290:,:]=False
ys,xs=np.where(m); dd=d[m]; wx,wy=W(ys,xs,dd)
print('handle x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center x',wx.mean().round(4),'top h', h[m].max().round(3),'min h',h[m].min().round(3))
for u in range(284,318,4):
    col=h[260:290,u]; rows=np.where(col>0.04)[0]+260
    if len(rows): print(' col',u,'y=%.3f'%((u-320)*2.05/fx),'rows',rows.min(),rows.max(),'x=%.3f..%.3f'%(W(rows.min(),u,2.05)[0],W(rows.max(),u,2.05)[0]),'h=%.3f'%col.max())
# knob
m=(h>0.01); m[:, :350]=False; m[:,395:]=False; m[:222,:]=False; m[256:,:]=False
ys,xs=np.where(m); dd=d[m]; wx,wy=W(ys,xs,dd)
print('knob x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',wx.mean().round(4),wy.mean().round(4))
m2=m&(h>0.05); ys,xs=np.where(m2); dd=d[m2]; wx,wy=W(ys,xs,dd)
print('tab x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',wx.mean().round(4),wy.mean().round(4),'h',h[m2].max().round(3))
# stove
m=(h>0.01)&(h<0.04); m[:, :340]=False; m[:,410:]=False; m[:252,:]=False; m[312:,:]=False
ys,xs=np.where(m); dd=d[m]; wx,wy=W(ys,xs,dd)
print('stove x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',((wx.min()+wx.max())/2).round(3),((wy.min()+wy.max())/2).round(3),'top h',np.median(h[m]).round(3))
"


# openrua op 13
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); h=3.0-d-0.9
fx=579.4112549695428
def W(v,u,dd): return -0.2+(v-240)*dd/fx, (u-320)*dd/fx
m=(h>0.01)&(h<0.1); m[:, :356]=False; m[:,395:]=False; m[:222,:]=False; m[258:,:]=False
ys,xs=np.where(m); dd=d[m]; wx,wy=W(ys,xs,dd)
print('knob x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',((wx.min()+wx.max())/2).round(4),((wy.min()+wy.max())/2).round(4),'base h',np.median(h[m]).round(3))
m2=m&(h>0.045); ys,xs=np.where(m2); dd=d[m2]; wx,wy=W(ys,xs,dd)
print('tab x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',((wx.min()+wx.max())/2).round(4),((wy.min()+wy.max())/2).round(4),'h',h[m2].max().round(3))
np.set_printoptions(linewidth=250,precision=3,suppress=True)
print(h[224:256,358:392])
"


# openrua op 14
python3 -c "
import numpy as np, sys
d=np.load('birdview_depth.npy'); h=3.0-d-0.9
np.set_printoptions(linewidth=300,precision=2,suppress=True,threshold=100000)
sub=h[224:256,358:392]
for i,row in enumerate(sub):
    print(224+i, ''.join('#' if x>0.045 else ('o' if x>0.01 else '.') for x in row))
"


# openrua op 15
timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 20 ros2 interface show moveit_msgs/srv/GetPositionIK | grep -v "^#" | head -40

# openrua op 16
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable controller: FK/IK/trajectory/gripper with persistent clients.

World frame = panda_link0 + BASE offset (from TF, world->panda_link0).
All public poses are WORLD-frame TCP poses (fingertip centre).
"""
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (measured from TF)


def quat_down(yaw_deg=0.0):
    """Hand pointing straight down (hand +Z = world -Z), fingers closing
    along world Y for yaw=0; yaw rotates about world Z (deg)."""
    r = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    return r.as_quat()  # x,y,z,w


class Robot:
    def __init__(self):
        root = Path(__file__).resolve().parent
        self.M = yaml.safe_load((root / "machine.yaml").read_text())
        self.traj = next(a for a in self.M["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in self.M["actuators"] if a["kind"] == "gripper")
        self.joints = self.traj["joints"]
        self.tcp_off = float(self.M["hand"]["tcp_offset_m"])
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_ctl")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, self.M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.gc.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    # ---------- sensing ----------
    def joint_state(self, fresh=True):
        if fresh:
            self._js = None
        t0 = time.time()
        while self._js is None and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def finger(self):
        js = self.joint_state()
        return js.get("panda_finger_joint1")

    def fk(self, q=None):
        """World-frame TCP pose (pos, quat xyzw) for joint vector q."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(self.joints)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        R = Rot.from_quat(quat).as_matrix()
        tcp = pos + self.tcp_off * R[:, 2]
        return tcp, quat

    # ---------- planning ----------
    def ik(self, tcp_world, quat, seed=None):
        quat = np.asarray(quat, float)
        R = Rot.from_quat(quat).as_matrix()
        hand = np.asarray(tcp_world, float) - self.tcp_off * R[:, 2] - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = self.arm_q() if seed is None else seed
        req.ik_request.robot_state.joint_state.name = list(self.joints)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in self.joints]

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        if waypoints:
            for (wq, wt) in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"  move_q done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.ik(tcp_world, quat, seed)
        code, err = self.move_q(q, seconds)
        tcp, _ = self.fk()
        print(f"  tcp now {np.round(tcp, 4)} target {np.round(tcp_world, 4)}")
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(self.grip.get("max_effort", 30.0))
        fut = self.gc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        f = self.finger()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} finger={f:.4f}")
        return f

    def open(self):
        return self.gripper(self.grip["open_m"])

    def close(self):
        return self.gripper(self.grip["closed_m"])
OPENRUA_EOF

# openrua op 17
timeout 120 python3 -c "
from robot import *
r=Robot()
print('q',np.round(r.arm_q(),3))
tcp,quat=r.fk()
print('tcp world',np.round(tcp,4),'quat',np.round(quat,4))
print('euler xyz deg', np.round(Rot.from_quat(quat).as_euler('xyz',degrees=True),1))
print('quat_down(0)',np.round(quat_down(0),4),' quat_down(90)',np.round(quat_down(90),4))
# test IK for above the knob
q=r.ik([-0.207,0.191,1.02], quat_down(0))
print('ik knob above', np.round(q,3))
print('fk check', np.round(r.fk(q)[0],4))
"

# openrua op 18
timeout 90 python3 scene.py sideview 375,200 372,185 365,150 && timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 19
timeout 120 python3 -c "
import robot
from robot import *
robot.BASE[:]=0
r=Robot()
tcp,quat=r.fk()
print('tcp (no base add)',np.round(tcp,4))
for tgt in ([-0.207,0.191,1.02],[-0.207,0.191,0.935],[-0.076,-0.07,1.0],[-0.053,0.19,1.0]):
    try:
        q=r.ik(tgt, quat_down(0))
        print('ik',tgt,'->',np.round(q,3),'fk',np.round(r.fk(q)[0],4))
    except Exception as e: print('ik',tgt,e)
"

# openrua op 20
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK already answer in the world frame (verified vs cameras)/' robot.py && grep -n "^BASE" robot.py && timeout 120 python3 -c "
from robot import *
r=Robot()
q=r.ik([-0.207,0.191,1.02], quat_down(0))
p0,qu0=r.fk(q); R0=Rot.from_quat(qu0).as_matrix()
q2=list(q); q2[6]+=0.5
p1,qu1=r.fk(q2); R1=Rot.from_quat(qu1).as_matrix()
print('hand x axis before',np.round(R0[:,0],3),'after +0.5 on j7',np.round(R1[:,0],3))
print('hand y axis before',np.round(R0[:,1],3),'after',np.round(R1[:,1],3))
print('tcp before',np.round(p0,4),'after',np.round(p1,4))
"

# openrua op 21
timeout 120 python3 -c "
from robot import *
from moveit_msgs.srv import GetPositionFK
r=Robot()
q=r.ik([-0.207,0.191,1.02], quat_down(0))
req=GetPositionFK.Request(); req.fk_link_names=['panda_link8','panda_hand','panda_leftfinger','panda_rightfinger']
req.robot_state.joint_state.name=list(r.joints); req.robot_state.joint_state.position=[float(v) for v in q]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for name,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped):
    p=ps.pose; qu=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    print(name, 'frame',ps.header.frame_id,'pos',np.round([p.position.x,p.position.y,p.position.z],4),'euler',np.round(Rot.from_quat(qu).as_euler('xyz',degrees=True),1))
"

# openrua op 22
mkdir -p "$(dirname /workspace/knob.py)"
cat > /workspace/knob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Turn the stove knob: grasp the tab from above, rotate the wrist.
Usage: knob.py <delta_j7_rad>   (negative = CCW viewed from above)
"""
import sys
import numpy as np
from robot import Robot, quat_down

KNOB = np.array([-0.207, 0.191])
Z_ABOVE, Z_GRASP = 1.02, 0.932
# fingers close along world Y (tab lies along X): link8 yaw = -45 deg
Q = quat_down(-45)

delta = float(sys.argv[1])
r = Robot()
print("finger before", r.finger())
r.open()
print("-> above knob")
r.move_tcp([*KNOB, Z_ABOVE], Q, 4.0)
print("-> descend")
q = r.move_tcp([*KNOB, Z_GRASP], Q, 2.5)
print("j7 at grasp", q[6])
f = r.close()
print("finger after close", f)
q2 = list(r.arm_q())
q2[6] += delta
print("-> rotate j7 by", delta, "to", q2[6])
r.move_q(q2, 3.0)
tcp, _ = r.fk()
print("tcp after rotate", np.round(tcp, 4), "finger", r.finger())
print("DONE")
OPENRUA_EOF

# openrua op 23
timeout 900 python3 -u knob.py -1.0 2>&1 | tee knob.log

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py robot0_robotview && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 25
mkdir -p "$(dirname /workspace/pan.py)"
cat > /workspace/pan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Release knob, then grasp the pan handle and lift. Stage 1 of the place."""
import numpy as np
from robot import Robot, quat_down

r = Robot()
tcp, _ = r.fk()
print("start tcp", np.round(tcp, 4))
r.open()
# straight up off the knob
r.move_tcp([tcp[0], tcp[1], 1.05], quat_down(-45), 2.5)

# pan handle: runs along Y at x=-0.076, grasp mid-handle; fingers close along X
QH = quat_down(45)
GX, GY = -0.076, -0.06
print("-> above handle")
r.move_tcp([GX, GY, 1.05], QH, 4.0)
print("-> descend")
r.move_tcp([GX, GY, 0.930], QH, 2.5)
f = r.close()
print("finger after close", f)
print("-> lift")
r.move_tcp([GX, GY, 1.20], QH, 3.0)
print("finger after lift", r.finger())
print("DONE")
OPENRUA_EOF

# openrua op 26
timeout 1200 python3 -u pan.py 2>&1 | tee pan.log

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 28
timeout 90 python3 scene.py birdview 320,240 >/dev/null && timeout 120 python3 -c "
from robot import *
import numpy as np
r=Robot(); tcp,_=r.fk(); print('tcp',np.round(tcp,4))
d=np.load('birdview_depth.npy'); z=3.0-d
fx=579.4112549695428
def W(v,u,dd): return -0.2+(v-240)*dd/fx, (u-320)*dd/fx
m=(z>1.08)&(z<1.20)
ys,xs=np.where(m); dd=d[m]; wx,wy=W(ys,xs,dd)
print('pixels',len(ys),'x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3))
# ascii
sub=m[150:330:3, 150:450:3]
for i,row in enumerate(sub): print(150+3*i, ''.join('#' if x else '.' for x in row))
"

# openrua op 29
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); z=3.0-d
m=(z>1.08)&(z<1.20)
np.set_printoptions(precision=3,linewidth=200)
print('row 297 z:', z[297,210:260:3])
print('col 235 z:', z[268:330:3,235])
"

# openrua op 30
timeout 900 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([-0.100, 0.378, 1.20], quat_down(45), 5.0)
print('finger', r.finger())
" 2>&1 | tee carry.log && timeout 90 python3 scene.py birdview 320,240 >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); z=3.0-d
fx=579.4112549695428
def W(v,u,dd): return -0.2+(v-240)*dd/fx, (u-320)*dd/fx
m=(z>1.08)&(z<1.20); m[:, :330]=False
ys,xs=np.where(m); dd=d[m]; wx,wy=W(ys,xs,dd)
print('pixels',len(ys),'x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',((wx.min()+wx.max())/2).round(3),((wy.min()+wy.max())/2).round(3))
"

# openrua op 31
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); z=3.0-d
fx=579.4112549695428
m=(z>1.0)&(z<1.25); m[:, :330]=False; m[:, 420:]=False
ys,xs=np.where(m)
print('rows',ys.min(),ys.max(),'cols',xs.min(),xs.max())
np.set_printoptions(precision=3,linewidth=220)
cy=int(np.median(ys)); cx=int(np.median(xs))
print('center px',cx,cy)
print('row',cy,'z:', z[cy, xs.min():xs.max():3])
print('col',cx,'z:', z[ys.min():ys.max():3, cx])
sub=m[220:320:3, 330:420:2]
for i,row in enumerate(sub): print(220+3*i, ''.join('#' if x else '.' for x in row))
"

# openrua op 32
timeout 900 python3 -u -c "
from robot import *
r=Robot()
Q=quat_down(45)
r.move_tcp([-0.039, 0.374, 1.20], Q, 3.0)
r.move_tcp([-0.039, 0.374, 1.05], Q, 3.0)
r.move_tcp([-0.039, 0.374, 0.965], Q, 3.0)
print('finger', r.finger())
" 2>&1 | tee place.log && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 33
timeout 900 python3 -u -c "
from robot import *
r=Robot()
Q=quat_down(45)
r.open()
r.move_tcp([-0.039, 0.374, 1.10], Q, 2.5)
r.move_tcp([-0.15, 0.30, 1.25], Q, 3.0)
" 2>&1 | tee release.log && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 90 python3 scene.py birdview 320,240 > /dev/null && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); h=3.0-d-0.9
fx=579.4112549695428
def W(v,u,dd): return -0.2+(v-240)*dd/fx, (u-320)*dd/fx
# pan rim above stove: height between 0.05 and 0.09 (stove top .025 + pan .043)
m=(h>0.045)&(h<0.09); m[:, :330]=False; m[:, 430:]=False; m[:230,:]=False; m[330:,:]=False
ys,xs=np.where(m); dd=d[m]; wx,wy=W(ys,xs,dd)
print('pan rim px',len(ys),'x',wx.min().round(3),wx.max().round(3),'y',wy.min().round(3),wy.max().round(3),'center',((wx.min()+wx.max())/2).round(3),((wy.min()+wy.max())/2).round(3))
cy=int(np.median(ys)); cx=int(np.median(xs))
np.set_printoptions(precision=3,linewidth=220)
print('interior z row',cy, (h[cy, xs.min():xs.max():4]+0.9))
print('interior z col',cx, (h[ys.min():ys.max():4, cx]+0.9))
print('stove plate center (-0.053,0.19)')
"

# openrua op 34
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); h=3.0-d-0.9
sub=h[222:256,356:392]
for i,row in enumerate(sub): print(222+i, ''.join('#' if x>0.05 else ('o' if x>0.01 else '.') for x in row))
"

# openrua op 35
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-kitchen-machine-facts.md)"
cat > /claude-config/projects/-workspace/memory/panda-kitchen-machine-facts.md <<'OPENRUA_EOF'
---
name: panda-kitchen-machine-facts
description: Verified kinematic conventions of the Franka Panda kitchen workstation (/workspace) that contradict or extend the shipped docs
metadata:
  type: project
---

Verified on 2026-09-12 on the Panda kitchen sim (stove/knob/pan/moka pot, table top z=0.90, base at world (-0.66,0,0.912)):

- `/compute_fk` and `/compute_ik` (empty frame_id) answer in the WORLD frame, not the arm base as docs/30-action.md claims — do not add the base offset.
- IK targets `panda_link8`; `panda_hand` is rotated -45° about z from it. Finger-closing axis in world for link8 quat `Rz(yaw)*Rx(180)` has angle `yaw - 45°` (yaw=-45 → fingers close along Y, yaw=45 → along X).
- panda_joint7 positive = clockwise viewed from above (axis points down when the hand points down).
- Stove knob (mimicgen-style): hinge stops after ~0.53 rad; rotating CCW-from-above (j7 -= 1.0, trajectory returns error -5 at the stop) turned the burner red = ON.
- Pan grasped by the handle tilts ~30° about the finger-closing axis while carried; it re-levels when lowered onto a surface, so lower to bottom-contact height before releasing.
- FJT result -5 with large joint error right after the result is often stale — re-read joint state before deciding it failed.

**Why:** these cost several turns to discover; the shipped docs are generic and partly wrong on frames.
**How to apply:** reuse `/workspace/robot.py` (Robot class: fk/ik/move_tcp/gripper, `quat_down(yaw)`) and `/workspace/scene.py` (birdview depth → world coords) if they still exist.
OPENRUA_EOF

# openrua op 36
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda kitchen machine facts](panda-kitchen-machine-facts.md) — FK/IK are world-frame, link8 vs hand 45° offset, j7 direction, knob/pan handling" >> MEMORY.md && cat MEMORY.md
