#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -iE "ik|fk|plan|scene" ; echo --- ; ros2 topic echo /joint_states --once

# openrua op 5
for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8

# openrua op 7
cd /workspace; echo "red mug body:"; timeout 60 python3 tools/perception/px2world.py agentview 312 215; echo "grey mug:"; timeout 60 python3 tools/perception/px2world.py agentview 200 230; echo "pudding:"; timeout 60 python3 tools/perception/px2world.py agentview 385 270; echo "plate center:"; timeout 60 python3 tools/perception/px2world.py agentview 330 390; echo "table near plate:"; timeout 60 python3 tools/perception/px2world.py agentview 450 400

# openrua op 8
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A1 Translation; for p in "325 322" "277 262" "318 285" "305 245" "400 200"; do echo "px $p:"; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 9
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table from a camera's depth + color and report
world-frame centroid / bbox / top height / mean colour for each blob.

Usage: python3 scene.py [camera=birdview] [zmin=0.44]
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
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
    cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.44
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T  # world coords (h,w,3)
    np.save(f"{cam}_world.npy", P)
    cv2.imwrite(f"{cam}.png", color)
    valid = np.isfinite(depth) & (depth > 0)
    mask = valid & (P[..., 2] > zmin) & (P[..., 2] < 1.2)
    n, lab = cv2.connectedComponents(mask.astype(np.uint8))
    print(f"camera {cam}: {n-1} blobs with z>{zmin}")
    for i in range(1, n):
        m = lab == i
        if m.sum() < 15:
            continue
        pts = P[m]
        col = color[m].mean(0)[::-1]  # rgb
        vs, us = np.nonzero(m)
        print(f"blob {i}: n={m.sum()} px(u={us.min()}-{us.max()}, v={vs.min()}-{vs.max()}) "
              f"centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
              f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"ztop={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} rgb={col.astype(int)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
cd /workspace; timeout 120 python3 scene.py birdview 0.44

# openrua op 11
cd /workspace; timeout 120 python3 scene.py agentview 0.44; python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); 
# table height samples around plate
for (u,v) in [(325,360),(380,322),(270,322),(325,290),(200,300),(450,300)]:
    print((u,v), P[v,u].round(3))
"

# openrua op 12
cd /workspace; timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 1.0; timeout 120 python3 scene.py robot0_eye_in_hand 0.44

# openrua op 13
cd /workspace; python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_world.npy'); img=cv2.imread('robot0_eye_in_hand.png')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
# red hue: h<10 or h>170, s>80
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>80)&(hsv[...,2]>50)
red &= P[...,2]>0.45
vs,us=np.nonzero(red); print('red px',red.sum(), 'u',us.min(),us.max(),'v',vs.min(),vs.max())
pts=P[red]; print('red mug centroid',pts.mean(0).round(3),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
# grey mug: region u>430,v>270, light grey
g=(P[...,2]>0.45)&(hsv[...,1]<40)&(hsv[...,2]>90)
g[:, :430]=False; g[:270,:]=False
pts=P[g]; print('grey mug n',g.sum(),'centroid',pts.mean(0).round(3),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
"


# openrua op 14
cd /workspace; python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_world.npy'); img=cv2.imread('robot0_eye_in_hand.png')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>80)&(hsv[...,2]>50)&(P[...,2]>0.45)
m=np.zeros_like(red); m[350:410,260:380]=True; red&=m
pts=P[red]; print('red mug n',red.sum(),'centroid',pts.mean(0).round(3),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
g=(P[...,2]>0.45)&(P[...,2]<0.62)&(hsv[...,1]<40)&(hsv[...,2]>90)
g[:, :430]=False; g[:270,:]=False; g[395:,:]=False
pts=P[g]; print('grey mug n',g.sum(),'centroid',pts.mean(0).round(3),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
"


# openrua op 15
cd /workspace; timeout 300 python3 tools/action/fjt_send.py 1.3,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 3.0 && timeout 120 python3 scene.py birdview 0.445

# openrua op 16
cd /workspace; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 17
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable motion/perception helpers for this Panda (clients built once)."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.510, 0.000, 0.420])  # from tf world->panda_link0

# top-down grasp orientation with the finger-closing axis yawed by psi
# (psi=0: fingers close along world y; psi=90deg: along world x)
def down_quat(psi_deg=0.0):
    h = math.radians(psi_deg) / 2
    return (math.cos(h), math.sin(h), 0.0, 0.0)  # x y z w


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self.js = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    # ---------- sensing ----------
    def joints(self):
        self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose(self):
        """world-frame hand pose via FK: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = self.joints()
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_pose(self):
        xyz, q = self.hand_pose()
        R = quat_R(*q)
        return xyz + TCP * R[:, 2], q

    # ---------- acting ----------
    def move_joints(self, target, seconds=3.0, tries=3, tol=0.02):
        for i in range(tries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(x) for x in target])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            err = np.abs(np.array(self.joints()) - np.array(target)).max()
            print(f"  move_joints try{i}: code={code} max_err={err:.4f}", flush=True)
            if err < tol:
                return True
        return err < tol * 3

    def ik(self, xyz_world, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = np.array(xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(seed or self.joints())
        req.ik_request.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"  IK failed for {xyz_world} code={None if r is None else r.error_code.val}", flush=True)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_hand(self, xyz_world, quat, seconds=3.0):
        """IK to a world-frame HAND pose, then trajectory. Returns success."""
        q = self.ik(xyz_world, quat)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        p, _ = self.hand_pose()
        print(f"  hand now at {p.round(4)} (target {np.round(xyz_world,4)})", flush=True)
        return ok

    def move_tcp(self, xyz_world, quat, seconds=3.0):
        R = quat_R(*quat)
        hand = np.array(xyz_world) - TCP * R[:, 2]
        return self.move_hand(hand, quat, seconds)

    def gripper(self, open_):
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        print(f"  gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(self.fingers(),4)}", flush=True)
        return r

    def servo(self, dx=0, dy=0, dz=0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(dx), float(dy), float(dz)
        for _ in range(ticks):
            self.twist_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)
        stop = TwistStamped(); stop.header.frame_id = TWIST["frame"]
        self.twist_pub.publish(stop)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
OPENRUA_EOF

# openrua op 18
cd /workspace; cat > step1.py <<'EOF'
from robot import *
r = Robot()
print("joints", np.round(r.joints(),3))
print("hand FK", r.hand_pose())
print("tcp FK", r.tcp_pose()[0])
# go to a viewing pose above the red mug, fingers closing along world x
ok = r.move_hand([-0.185, 0.0, 0.90], down_quat(90), 4.0)
print("ok", ok, "joints", np.round(r.joints(),3))
EOF
timeout 600 python3 -u step1.py 2>&1 | tail -20

# openrua op 19
cd /workspace; cat > step1b.py <<'EOF'
from robot import *
import robot
r = Robot()
r.spin(1.0)
t = r.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
print("TF world->hand", t.transform.translation)
robot.BASE_IN_WORLD = np.zeros(3)
print("FK (no offset)", r.hand_pose())
print("IK world-coords:", r.ik([-0.185, 0.0, 0.90], down_quat(90)))
print("IK base-coords :", r.ik([-0.185+0.51, 0.0, 0.90-0.42], down_quat(90)))
EOF
timeout 300 python3 -u step1b.py 2>&1 | tail -20

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "BASE_IN_WORLD = np.array([-0.510, 0.000, 0.420])  # from tf world->panda_link0", "new_string": "# verified: /compute_fk output matches TF world->panda_hand exactly, so the\\n# planner\'s model frame IS world here (no base offset to apply)\\nBASE_IN_WORLD = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
cd /workspace; cat > step1.py <<'EOF'
from robot import *
r = Robot()
ok = r.move_hand([-0.185, 0.0, 0.90], down_quat(90), 4.0)
print("ok", ok, "joints", np.round(r.joints(),3))
print("hand", r.hand_pose())
EOF
timeout 600 python3 -u step1.py 2>&1 | tail -20 && timeout 120 python3 scene.py robot0_eye_in_hand 0.45

# openrua op 22
cd /workspace; python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_world.npy')
z=P[...,2]
m=np.zeros(z.shape,bool); m[240:350,260:380]=True
rim=m&(z>0.555)&(z<0.58)
pts=P[rim][:,:2]
print('rim n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
# fit circle (algebraic)
A=np.c_[2*pts, np.ones(len(pts))]; b=(pts**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('circle center',round(cx,4),round(cy,4),'R',round(R,4))
res=np.abs(np.hypot(pts[:,0]-cx,pts[:,1]-cy)-R); print('resid mean',res.mean().round(4),'max',res.max().round(4))
# exclude handle: keep points within R+0.01 and refit
keep=np.hypot(pts[:,0]-cx,pts[:,1]-cy)<R+0.008
pts2=pts[keep]; A=np.c_[2*pts2, np.ones(len(pts2))]; b=(pts2**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('refit center',round(cx,4),round(cy,4),'R',round(R,4))
# body at mid height z~0.50-0.53 (side walls visible?)
body=m&(z>0.46)&(z<0.55); pb=P[body][:,:2]; print('wall n',len(pb),'x',pb[:,0].min().round(3),pb[:,0].max().round(3),'y',pb[:,1].min().round(3),pb[:,1].max().round(3))
print('z top max', z[m&(z<0.7)].max().round(4))
"


# openrua op 23
cd /workspace; python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_world.npy'); z=P[...,2]
# table around pudding & mug (flat, below 0.449)
m=np.zeros(z.shape,bool); m[200:360,240:500]=True
tab=m&(z<0.449)&(z>0.40); print('table z mean',z[tab].mean().round(4),'std',z[tab].std().round(4), 'n',tab.sum())
# handle direction
m2=np.zeros(z.shape,bool); m2[240:350,260:380]=True
rim=m2&(z>0.555)&(z<0.58); pts=P[rim][:,:2]; c=np.array([-0.1821,0.0005])
d=np.hypot(*(pts-c).T); h=pts[d>0.05]; print('handle pts',len(h),'mean',h.mean(0).round(3),'dir',(h.mean(0)-c).round(3))
# pudding box: PCA of top face pts
m3=np.zeros(z.shape,bool); m3[230:330,380:470]=True
box=m3&(z>0.458)&(z<0.47); pb=P[box][:,:2]; c2=pb.mean(0)
u,s,vt=np.linalg.svd(pb-c2); ang=np.degrees(np.arctan2(vt[0,1],vt[0,0]))
proj=(pb-c2)@vt.T; print('pudding center',c2.round(4),'long axis angle from x',round(ang,1),'extent',(proj.max(0)-proj.min(0)).round(3), 'ztop', z[box].max().round(4))
"


# openrua op 24
cd /workspace; cat > step2.py <<'EOF'
from robot import *
r = Robot()
MUG = np.array([-0.1821, 0.0005]); RIM_Z = 0.5735; R_MUG = 0.0415
q = down_quat(90)  # fingers close along world x
grasp_xy = MUG + [R_MUG, 0]
print("pre-grasp"); r.move_tcp([*grasp_xy, 0.66], q, 4.0)
r.gripper(True)
print("descend");  r.move_tcp([*grasp_xy, 0.548], q, 3.0)
print("tcp", r.tcp_pose()[0].round(4), "fingers", r.fingers())
EOF
timeout 900 python3 -u step2.py 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 25
cd /workspace; cat > step3.py <<'EOF'
from robot import *
r = Robot()
q = down_quat(90)
r.gripper(False)
f = r.fingers(); print("fingers after close", f, "gap", round(f[0]-f[1],4))
p = r.tcp_pose()[0]
print("lift"); r.move_tcp([p[0], p[1], 0.80], q, 3.0)
print("fingers after lift", r.fingers())
EOF
timeout 900 python3 -u step3.py 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 26
cd /workspace; timeout 120 python3 scene.py agentview 0.60 2>&1 | grep -v "^camera"; python3 -c "
import numpy as np, cv2
P=np.load('agentview_world.npy'); img=cv2.imread('agentview.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
z=P[...,2]
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>90)&(z>0.60)&(z<1.0)
pts=P[red]; print('red n',red.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
# bottom rim of mug (lowest red points): centre
low=pts[pts[:,2]<pts[:,2].min()+0.015]; print('bottom pts',len(low),'mean xy',low[:,:2].mean(0).round(3))
"

# openrua op 27
cd /workspace; python3 -c "
import numpy as np, cv2
P=np.load('agentview_world.npy'); img=cv2.imread('agentview.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
z=P[...,2]
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>90)&(z>0.60)&(z<1.0)
pts=P[red]
for lo in np.arange(0.69,0.83,0.02):
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.02)]
    if len(s)>5: print(f'z {lo:.2f}: n={len(s)} xmax={s[:,0].max():.3f} xmed={np.median(s[:,0]):.3f} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] ymed={np.median(s[:,1]):.3f}')
"

# openrua op 28
cd /workspace; timeout 120 python3 scene.py birdview 0.60 2>&1 | grep -v "^camera"; python3 -c "
import numpy as np, cv2
P=np.load('birdview_world.npy'); img=cv2.imread('birdview.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
z=P[...,2]
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>90)&(z>0.60)&(z<1.0)
pts=P[red]; print('red n',red.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
vs,us=np.nonzero(red); print('px u',us.min(),us.max(),'v',vs.min(),vs.max())
"; python3 -c "
import cv2; img=cv2.imread('/workspace/birdview.png'); cv2.imwrite('/workspace/bird_crop.png', cv2.resize(img[200:300,250:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"

# openrua op 29
cd /workspace; cat > step4.py <<'EOF'
from robot import *
r = Robot()
q = down_quat(90)
PLATE = np.array([0.158, 0.020])
hand_xy = PLATE + [0.037, 0.015]
print("transit above plate"); r.move_tcp([*hand_xy, 0.80], q, 4.0)
print("fingers", r.fingers())
print("lower"); r.move_tcp([*hand_xy, 0.60], q, 3.0)
r.move_tcp([*hand_xy, 0.583], q, 2.0)
print("tcp", r.tcp_pose()[0].round(4))
r.gripper(True)
print("retreat"); r.move_tcp([*hand_xy, 0.75], q, 3.0)
EOF
timeout 1200 python3 -u step4.py 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
cd /workspace; cat > step5.py <<'EOF'
from robot import *
r = Robot()
# park above the pudding's neighbourhood but high, fingers closing along y
r.move_tcp([-0.08, 0.10, 0.80], down_quat(0), 4.0)
EOF
timeout 900 python3 -u step5.py 2>&1 | tail -5; timeout 120 python3 scene.py birdview 0.44 | grep -v "^camera"; python3 -c "
import numpy as np, cv2
P=np.load('birdview_world.npy'); img=cv2.imread('birdview.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV); z=P[...,2]
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>90)&(z>0.50)
pts=P[red]; print('red mug top: n',red.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
"

# openrua op 31
cd /workspace; python3 -c "
import numpy as np, cv2
P=np.load('birdview_world.npy'); z=P[...,2]
m=np.zeros(z.shape,bool); m[290:350,295:355]=True
mug=m&(z>0.52); pts=P[mug]; print('mug n',mug.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'centroid',pts[:,:2].mean(0).round(3),'ztop',z[mug].max().round(3))
plate=m&(z>0.44)&(z<0.47); pp=P[plate]; print('plate n',plate.sum(),'x',pp[:,0].min().round(3),pp[:,0].max().round(3),'y',pp[:,1].min().round(3),pp[:,1].max().round(3))
"; python3 -c "
import cv2; img=cv2.imread('/workspace/birdview.png'); cv2.imwrite('/workspace/bird_crop.png', cv2.resize(img[280:360,280:370],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 32
cd /workspace; python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
m=np.zeros(z.shape,bool); m[290:350,295:355]=True
mug=m&(z>0.52)&(z<0.65); pts=P[mug]; print('mug n',mug.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'centroid',pts[:,:2].mean(0).round(3),'ztop',z[mug].max().round(3))
"

# openrua op 33
cd /workspace; cat > step6.py <<'EOF'
from robot import *
r = Robot()
q = down_quat(0)  # fingers close along world y (box short side)
PUD = np.array([-0.0767, 0.1018])
r.gripper(True)
print("pre-grasp"); r.move_tcp([*PUD, 0.56], q, 4.0)
print("descend");   r.move_tcp([*PUD, 0.448], q, 3.0)
print("tcp", r.tcp_pose()[0].round(4))
r.gripper(False)
f = r.fingers(); print("gap", round(f[0]-f[1],4))
print("lift"); r.move_tcp([*PUD, 0.65], q, 3.0)
print("fingers after lift", r.fingers())
EOF
timeout 1200 python3 -u step6.py 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
cd /workspace; python3 - <<'EOF'
import re
s = open('robot.py').read()
s = s.replace("def move_joints(self, target, seconds=3.0, tries=3, tol=0.02):", "def move_joints(self, target, seconds=3.0, tries=4, tol=0.005):")
s = s.replace("""    def servo(self""", """    def settle(self, seconds=1.0):
        \"\"\"advance the paused sim by holding the current pose (lets the
        gripper finish moving, objects settle).\"\"\"
        self.move_joints(self.joints(), seconds, tries=1)
        return self.fingers()

    def servo(self""")
open('robot.py','w').write(s)
EOF
grep -n "def settle\|tries=4" robot.py

# openrua op 35
cd /workspace; timeout 120 python3 scene.py birdview 0.455 | grep -v "^camera"

# openrua op 36
cd /workspace; timeout 120 python3 scene.py robot0_eye_in_hand 0.455 | grep -v "^camera"; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); z=P[...,2]
box=(z>0.458)&(z<0.47)&(P[...,0]>-0.15)&(P[...,0]<0.0)&(P[...,1]>0.05)&(P[...,1]<0.16)
pb=P[box][:,:2]; c=pb.mean(0); u,s,vt=np.linalg.svd(pb-c); ang=np.degrees(np.arctan2(vt[0,1],vt[0,0])); proj=(pb-c)@vt.T
print('pudding center',c.round(4),'angle',round(ang,1),'extent',(proj.max(0)-proj.min(0)).round(3),'ztop',z[box].max().round(4),'n',box.sum())"

# openrua op 37
cd /workspace; cat > step7.py <<'EOF'
from robot import *
r = Robot()
q = down_quat(0)
PUD = np.array([-0.061, 0.0985])
r.gripper(True); r.settle(0.5)
print("pre-grasp"); r.move_tcp([*PUD, 0.55], q, 3.0)
print("descend");   r.move_tcp([*PUD, 0.442], q, 3.0)
print("tcp", r.tcp_pose()[0].round(4))
r.gripper(False)
f = r.settle(1.0); print("gap after settle", round(f[0]-f[1],4))
f = r.settle(0.5); print("gap after settle2", round(f[0]-f[1],4))
if f[0]-f[1] > 0.02:
    print("lift"); r.move_tcp([*PUD, 0.65], q, 3.0)
    print("fingers after lift", r.fingers())
else:
    print("GRASP FAILED, not lifting")
EOF
timeout 1200 python3 -u step7.py 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 38
cd /workspace; cat > step8.py <<'EOF'
from robot import *
r = Robot()
q = down_quat(0)
TGT = np.array([0.156, 0.155])
print("transit"); r.move_tcp([*TGT, 0.65], q, 4.0)
print("fingers", r.fingers())
print("lower"); r.move_tcp([*TGT, 0.50], q, 3.0); r.move_tcp([*TGT, 0.450], q, 2.0)
print("tcp", r.tcp_pose()[0].round(4))
r.gripper(True); r.settle(1.0)
print("retreat"); r.move_tcp([*TGT, 0.65], q, 3.0)
# park away for a clear view
r.move_hand([-0.25, -0.30, 0.85], down_quat(0), 4.0)
r.settle(1.0)
EOF
timeout 1500 python3 -u step8.py 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 39
cd /workspace; timeout 120 python3 scene.py birdview 0.455 | grep -v "^camera"; python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
sel=lambda m: P[m]
plate=(z>0.44)&(z<0.47)&(P[...,0]>0.05)&(P[...,0]<0.26)&(P[...,1]>-0.10)&(P[...,1]<0.10)
pp=sel(plate); print('plate  x[%.3f,%.3f] y[%.3f,%.3f] center (%.3f,%.3f)'%(pp[:,0].min(),pp[:,0].max(),pp[:,1].min(),pp[:,1].max(),(pp[:,0].min()+pp[:,0].max())/2,(pp[:,1].min()+pp[:,1].max())/2))
mug=(z>0.52)&(z<0.65)&(P[...,0]>0.05)&(P[...,0]<0.26)&(P[...,1]>-0.10)&(P[...,1]<0.10)
pm=sel(mug); print('redmug x[%.3f,%.3f] y[%.3f,%.3f] centroid (%.3f,%.3f) ztop %.3f'%(pm[:,0].min(),pm[:,0].max(),pm[:,1].min(),pm[:,1].max(),pm[:,0].mean(),pm[:,1].mean(),pm[:,2].max()))
pud=(z>0.455)&(z<0.48)&(P[...,0]>0.05)&(P[...,0]<0.26)&(P[...,1]>0.10)&(P[...,1]<0.25)
pd=sel(pud); print('pudding x[%.3f,%.3f] y[%.3f,%.3f] centroid (%.3f,%.3f) ztop %.3f n %d'%(pd[:,0].min(),pd[:,0].max(),pd[:,1].min(),pd[:,1].max(),pd[:,0].mean(),pd[:,1].mean(),pd[:,2].max(),pud.sum()))
"; python3 -c "
import cv2; img=cv2.imread('/workspace/birdview.png'); cv2.imwrite('/workspace/bird_crop.png', cv2.resize(img[270:370,270:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"

# openrua op 40
cd /workspace; pgrep -af "python3" | grep -v pgrep; ros2 topic echo /joint_states --once | grep -A9 position | tr '\n' ' '
