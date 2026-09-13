#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | head -40; echo ----; ros2 node list

# openrua op 4
cd /workspace; ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls -la *.png

# openrua op 6
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | head -40

# openrua op 7
mkdir -p "$(dirname /workspace/grab_cam.py)"
cat > /workspace/grab_cam.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF(world<-optical) for a camera, save to <cam>.npz and <cam>.png."""
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
    rclpy.init()
    node = rclpy.create_node("grab_cam")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    import time
    for cam in sys.argv[1:]:
        color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
        depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        end = time.time() + 15
        while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        K = np.array(info.k).reshape(3, 3)
        np.savez(f"{cam}.npz", color=color, depth=depth, K=K, T=T)
        cv2.imwrite(f"{cam}.png", color)
        print(cam, color.shape, depth.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
        print("K", K.tolist())
        print("T", T.tolist())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
cd /workspace; timeout 120 python3 grab_cam.py birdview agentview robot0_eye_in_hand 2>&1 | grep -v "^\[" 

# openrua op 9
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview.npz'); depth=d['depth']; K=d['K']; T=d['T']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
z=depth
pc=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z,np.ones_like(z)],-1)
pw=pc@T.T
Z=pw[...,2]
print('table-ish Z histogram:')
h,e=np.histogram(Z[np.isfinite(Z)],bins=60)
for hh,ee in zip(h,e): print(f'{ee:.3f} {hh}')
np.save('bird_pw.npy',pw)
"


# openrua op 10
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('birdview.npz'); color=d['color']
pw=np.load('bird_pw.npy'); Z=pw[...,2]
mask=((Z>0.435)&(Z<0.72)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<15: continue
    m=lab==i
    P=pw[m]
    c=color[m].mean(0)
    print(f'comp{i} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) area={a} bbox=({x},{y},{w},{h}) world xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) zmax={P[:,2].max():.3f} zmean={P[:,2].mean():.3f} bgr={c.astype(int)}')
"


# openrua op 11
cd /workspace; python3 -c "
import numpy as np
pw=np.load('bird_pw.npy'); Z=pw[...,2]
for (u,v) in [(200,300),(450,200),(300,350),(330,230),(150,250),(500,300)]:
    print((u,v), pw[v,u][:3].round(3))
reg=Z[100:380,120:540]
h,e=np.histogram(reg[(reg>0.40)&(reg<0.48)],bins=40)
for hh,ee in zip(h,e): print(f'{ee:.4f} {hh}')
"


# openrua op 12
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('birdview.npz'); color=d['color'].copy()
pw=np.load('bird_pw.npy'); Z=pw[...,2]
m=(Z>0.43)&(Z<0.72)
color[m]=(0.5*color[m]+[0,127,0]).astype(np.uint8)
crop=color[80:400,100:560]
cv2.imwrite('bird_mask.png',cv2.resize(crop,None,fx=1.5,fy=1.5,interpolation=cv2.INTER_NEAREST))
"


# openrua op 13
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand.npz'); depth=d['depth']; K=d['K']; T=d['T']; color=d['color']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
pc=np.stack([(u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)
pw=pc@T.T
Z=pw[...,2]
print('Z range',np.nanmin(Z),np.nanmax(Z))
mask=((Z>0.435)&(Z<0.72)).astype(np.uint8)
mask[400:,:]=0  # gripper fingers at bottom
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<200: continue
    m=lab==i; P=pw[m]; c=color[m].mean(0)
    print(f'comp{i} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) area={a} bbox=({x},{y},{w},{h}) xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) zmax={P[:,2].max():.3f} bgr={c.astype(int)}')
"


# openrua op 14
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand.npz'); depth=d['depth']; K=d['K']; T=d['T']; color=d['color']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
pc=np.stack([(u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)
pw=pc@T.T
Z=pw[...,2]
reg=(slice(280,400),slice(200,340))
z=Z[reg]
h,e=np.histogram(z,bins=30)
for hh,ee in zip(h,e): print(f'{ee:.3f} {hh}')
m=np.zeros_like(Z,bool); m[reg]=(Z[reg]>0.52)&(Z[reg]<0.58)
P=pw[m]; print('can top: n',m.sum(),'xy',P[:,0].mean().round(3),P[:,1].mean().round(3),'z',P[:,2].mean().round(3), 'xrange',P[:,0].min().round(3),P[:,0].max().round(3),'yrange',P[:,1].min().round(3),P[:,1].max().round(3))
"


# openrua op 15
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand.npz'); depth=d['depth']; K=d['K']; T=d['T']; color=d['color']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
pc=np.stack([(u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)
pw=pc@T.T
Z=pw[...,2]
reg=(slice(280,400),slice(150,340))
m=np.zeros_like(Z,bool); m[reg]=(Z[reg]>0.50)&(Z[reg]<0.53)
P=pw[m]; print('can top: n',m.sum(),'xy',P[:,0].mean().round(3),P[:,1].mean().round(3),'z',P[:,2].mean().round(3), 'xrange',P[:,0].min().round(3),P[:,0].max().round(3),'yrange',P[:,1].min().round(3),P[:,1].max().round(3))
vis=color.copy(); vis[m]=(0,255,0); cv2.imwrite('eih_can.png',vis)
# blue can
reg=(slice(260,350),slice(430,540))
m=np.zeros_like(Z,bool); m[reg]=(Z[reg]>0.49)&(Z[reg]<0.53)
P=pw[m]; print('blue can top: n',m.sum(),'xy',P[:,0].mean().round(3),P[:,1].mean().round(3),'z',P[:,2].mean().round(3))
# cream cheese
reg=(slice(80,180),slice(480,545))
m=np.zeros_like(Z,bool); m[reg]=(Z[reg]>0.44)&(Z[reg]<0.47)
P=pw[m]; print('cream cheese top: n',m.sum(),'xy',P[:,0].mean().round(3),P[:,1].mean().round(3),'z',P[:,2].mean().round(3),'xrange',P[:,0].min().round(3),P[:,0].max().round(3),'yrange',P[:,1].min().round(3),P[:,1].max().round(3))
"


# openrua op 16
cd /workspace; ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show control_msgs/action/GripperCommand

# openrua op 17
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda workstation: one node, reusable clients.

World frame vs base: world->panda_link0 = (-0.51, 0, 0.42). IK/FK operate in
the base (panda_link0) frame with frame_id left EMPTY (machine fact).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM_JOINTS = TRAJ["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TABLE_Z_WORLD = 0.425


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("agent_robot")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 15
        while "m" not in self._js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_positions(self):
        j = self.joints()
        return [j[n] for n in ARM_JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def _arm_state(self, positions=None):
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = list(positions if positions is not None else self.arm_positions())
        return js

    def fk_hand(self, positions=None):
        """Hand (panda_hand) pose in base frame: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._arm_state(positions)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, positions=None):
        pos, q = self.fk_hand(positions)
        R = quat_to_R(*q)
        tcp = pos + TCP_OFF * R[:, 2]
        return tcp + BASE_IN_WORLD, q

    # ---- planning ----
    def ik_tcp_world(self, tcp_world, q, seed=None):
        """IK for a TCP position in WORLD frame with hand quaternion q (xyzw). Returns arm positions or None."""
        R = quat_to_R(*q)
        hand_world = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self._arm_state(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            log("IK failed", None if res is None else res.error_code.val, "for tcp", tcp_world)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM_JOINTS]

    # ---- acting ----
    def move_joints(self, positions, seconds=3.0, via=None):
        """Send one trajectory (optionally through 'via' waypoints, list of (positions, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        pts = []
        for pos, t in (via or []):
            pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "FJT goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = None if r is None else r.result.error_code
        now = self.arm_positions()
        err = float(np.max(np.abs(np.array(now) - np.array(positions))))
        log(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp_world(self, tcp_world, q, seconds=3.0, seed=None):
        sol = self.ik_tcp_world(tcp_world, q, seed)
        if sol is None:
            return None
        code, err = self.move_joints(sol, seconds)
        tcp, _ = self.tcp_world()
        log(f"tcp now world={tcp.round(4)} target={np.asarray(tcp_world).round(4)}")
        return code, err, tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        log(f"gripper cmd={width} reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} gap={gap:.4f}")
        return gap
OPENRUA_EOF

# openrua op 18
cd /workspace; timeout 120 python3 -u -c "
from robot import *
r=Robot()
print('joints',r.joints())
pos,q=r.fk_hand(); print('hand base',pos.round(4),'q',q.round(4))
tcp,_=r.tcp_world(); print('tcp world',tcp.round(4))
print('gap',r.finger_gap())
" 2>&1 | grep -v '^\['

# openrua op 19
cd /workspace; python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
T=np.eye(4)
for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
T=T@dh(0,0.107,0,0)  # flange
T=T@dh(0,0,0,-np.pi/4) # hand
print('hand base',T[:3,3].round(4)); print(T[:3,:3].round(3))
print('tcp base',(T[:3,3]+0.1034*T[:3,2]).round(4))
print('tcp world',(T[:3,3]+0.1034*T[:3,2]+[-0.51,0,0.42]).round(4))
"


# openrua op 20
cd /workspace; timeout 120 python3 -u -c "
from robot import *
r=Robot()
res=r.fk_hand()
print(res)
# check header frame of FK response
req = GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state=r._arm_state()
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
for ps in fut.result().pose_stamped: print(ps.header.frame_id, ps.pose.position)
q=[0.9996,0,-0.0284,0]
import numpy as np
for name,hand in [('world',[-0.053,0,0.7776]),('base',[0.457,0,0.3576])]:
    R=quat_to_R(*q); tcp=np.array(hand)+TCP_OFF*R[:,2]+BASE_IN_WORLD  # ik_tcp_world subtracts BASE_IN_WORLD again
    sol=r.ik_tcp_world(tcp,q)
    print(name, None if sol is None else np.round(sol,3))
print('current', np.round(r.arm_positions(),3))
" 2>&1 | grep -v '^\['

# openrua op 21
cd /workspace; python3 - <<'EOF'
import re
s=open('robot.py').read()
s=s.replace('BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])','# Verified empirically: MoveIt FK/IK on this machine use the WORLD frame as model\n# frame (panda_link0 sits at world (-0.51, 0, 0.42)); so no base offset is applied.\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])')
s=s.replace('"""Hand (panda_hand) pose in base frame: (xyz, quat xyzw)."""','"""Hand (panda_hand) pose in WORLD frame: (xyz, quat xyzw)."""')
open('robot.py','w').write(s)
EOF
grep -n "BASE_IN_WORLD" robot.py

# openrua op 22
cd /workspace; python3 - <<'EOF'
s=open('robot.py').read()
s=s.replace("""World frame vs base: world->panda_link0 = (-0.51, 0, 0.42). IK/FK operate in
the base (panda_link0) frame with frame_id left EMPTY (machine fact).""","""world->panda_link0 = (-0.51, 0, 0.42). Verified: MoveIt FK/IK here take/return
poses in the WORLD frame when frame_id is left EMPTY.""")
open('robot.py','w').write(s)
EOF
echo ok

# openrua op 23
mkdir -p "$(dirname /workspace/stage.py)"
cat > /workspace/stage.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Run one manipulation stage. Usage:
  stage.py pregrasp X Y Z_TCP        open gripper, move TCP (world) above target, top-down
  stage.py goto X Y Z_TCP [secs]     move TCP (world) top-down, no gripper change
  stage.py grasp Z_TCP               descend to Z at current XY, close gripper, report gap
  stage.py lift Z_TCP                rise to Z at current XY, report gap
  stage.py release                   open gripper
  stage.py pose                      print current TCP world pose and finger gap
Writes progress to stdout (run with python3 -u > file).
"""
import sys
import numpy as np
from robot import Robot, log

Q_DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand z -> world -z, fingers open along world y


def main():
    cmd = sys.argv[1]
    r = Robot()
    tcp, q = r.tcp_world()
    log(f"start tcp={tcp.round(4)} q={q.round(4)} gap={r.finger_gap():.4f}")
    if cmd == "pose":
        return
    if cmd == "release":
        r.gripper(0.04)
        return
    if cmd == "pregrasp":
        x, y, z = map(float, sys.argv[2:5])
        r.gripper(0.04)
        res = r.move_tcp_world([x, y, z], Q_DOWN, 4.0)
        if res is None:
            raise SystemExit("IK failed")
    elif cmd == "goto":
        x, y, z = map(float, sys.argv[2:5])
        secs = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0
        res = r.move_tcp_world([x, y, z], Q_DOWN, secs)
        if res is None:
            raise SystemExit("IK failed")
    elif cmd == "grasp":
        z = float(sys.argv[2])
        res = r.move_tcp_world([tcp[0], tcp[1], z], Q_DOWN, 3.0)
        if res is None:
            raise SystemExit("IK failed")
        gap = r.gripper(0.0)
        log(f"GRASP gap={gap:.4f} ({'holding' if gap > 0.004 else 'EMPTY'})")
    elif cmd == "lift":
        z = float(sys.argv[2])
        res = r.move_tcp_world([tcp[0], tcp[1], z], Q_DOWN, 3.0)
        if res is None:
            raise SystemExit("IK failed")
        log(f"after lift gap={r.finger_gap():.4f}")
    tcp, q = r.tcp_world()
    log(f"end tcp={tcp.round(4)} q={q.round(4)} gap={r.finger_gap():.4f}")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 24
cd /workspace; nohup python3 -u stage.py pregrasp -0.106 0.036 0.65 > s1.log 2>&1 &
sleep 45; grep -v '^\[' s1.log

# openrua op 25
cd /workspace; timeout 90 python3 grab_cam.py robot0_eye_in_hand 2>&1 | grep -v '^\[' | head -2; python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand.npz'); depth=d['depth']; K=d['K']; T=d['T']; color=d['color']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
pc=np.stack([(u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)
pw=pc@T.T; Z=pw[...,2]
m=(Z>0.50)&(Z<0.53); m[380:,:]=0
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if stats[i][4]<300: continue
    P=pw[lab==i]; print(f'comp px=({cent[i][0]:.0f},{cent[i][1]:.0f}) n={stats[i][4]} xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) z={P[:,2].mean():.3f} xr=({P[:,0].min():.3f},{P[:,0].max():.3f}) yr=({P[:,1].min():.3f},{P[:,1].max():.3f})')
"

# openrua op 26
cd /workspace; nohup python3 -u stage.py grasp 0.47 > s2.log 2>&1 &
sleep 40; grep -v '^\[' s2.log

# openrua op 27
cd /workspace; nohup python3 -u stage.py lift 0.78 > s3.log 2>&1 &
sleep 35; grep -v '^\[' s3.log; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1; echo snapped

# openrua op 28
cd /workspace; nohup python3 -u stage.py goto -0.009 0.274 0.78 5 > s4.log 2>&1 &
sleep 40; grep -v '^\[' s4.log

# openrua op 29
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null 2>&1; echo ok

# openrua op 30
cd /workspace; nohup sh -c 'python3 -u stage.py goto -0.009 0.274 0.70 3 && python3 -u stage.py release' > s5.log 2>&1 &
sleep 45; grep -v '^\[' s5.log

# openrua op 31
cd /workspace; nohup python3 -u stage.py goto -0.009 0.274 0.82 3 > s6.log 2>&1 &
sleep 30; grep -v '^\[' s6.log; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1; echo snapped

# openrua op 32
cd /workspace; nohup python3 -u stage.py pregrasp 0.092 -0.194 0.60 > s7.log 2>&1 &
sleep 40; grep -v '^\[' s7.log

# openrua op 33
cd /workspace; timeout 90 python3 grab_cam.py robot0_eye_in_hand 2>&1 | grep -v '^\[' | head -1; python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand.npz'); depth=d['depth']; K=d['K']; T=d['T']; color=d['color']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
pc=np.stack([(u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)
pw=pc@T.T; Z=pw[...,2]
m=(Z>0.44)&(Z<0.47); m[380:,:]=0
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if stats[i][4]<300: continue
    P=pw[lab==i]; print(f'comp px=({cent[i][0]:.0f},{cent[i][1]:.0f}) n={stats[i][4]} xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) z={P[:,2].mean():.3f} xr=({P[:,0].min():.3f},{P[:,0].max():.3f}) yr=({P[:,1].min():.3f},{P[:,1].max():.3f})')
"

# openrua op 34
cd /workspace; nohup sh -c 'python3 -u stage.py goto 0.094 -0.197 0.50 3 && python3 -u stage.py grasp 0.442' > s8.log 2>&1 &
sleep 60; grep -v '^\[' s8.log

# openrua op 35
cd /workspace; nohup python3 -u stage.py lift 0.70 > s9.log 2>&1 &
sleep 35; grep -v '^\[' s9.log; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1; echo snapped

# openrua op 36
cd /workspace; nohup sh -c 'python3 -u stage.py goto -0.009 0.274 0.80 5 && python3 -u stage.py goto 0.02 0.29 0.70 3 && python3 -u stage.py release && python3 -u stage.py goto 0.02 0.29 0.85 3' > s10.log 2>&1 &
sleep 100; grep -v '^\[' s10.log

# openrua op 37
cd /workspace; timeout 120 python3 grab_cam.py agentview robot0_eye_in_hand birdview 2>&1 | grep -v '^\[' | grep -v '^[KT] '

# openrua op 38
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand.npz'); depth=d['depth']; K=d['K']; T=d['T']; color=d['color']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
pc=np.stack([(u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)
pw=pc@T.T; Z=pw[...,2]
# basket interior footprint (from birdview): x in [-0.096,0.078], y in [0.187,0.361]
inb=(pw[...,0]>-0.085)&(pw[...,0]<0.07)&(pw[...,1]>0.2)&(pw[...,1]<0.35)
inb[380:,:]=0
z=Z[inb]
h,e=np.histogram(z,bins=25)
for hh,ee in zip(h,e): print(f'{ee:.3f} {hh}')
# top-most object surface(s) within basket footprint
for lo,hi,name in [(0.50,0.53,'can top?'),(0.53,0.56,'cheese top?')]:
    m=inb&(Z>lo)&(Z<hi)
    if m.sum()>100:
        P=pw[m]; print(name,'n',m.sum(),'xy',P[:,0].mean().round(3),P[:,1].mean().round(3),'z',P[:,2].mean().round(3),'xr',P[:,0].min().round(3),P[:,0].max().round(3),'yr',P[:,1].min().round(3),P[:,1].max().round(3))
"


# openrua op 39
pgrep -af "stage.py|grab_cam" || echo "no pending processes"
